#!/usr/bin/env python3
"""
Melange Netlist Exporter for KiCad

Converts a KiCad XML intermediate netlist to a Melange .cir netlist file.

Usage:
  Standalone:  python melange_netlist_export.py input.xml output.cir
  KiCad plugin: Register in Eeschema → Export Netlist → Add Generator
                Command: python /path/to/melange_netlist_export.py "%I" "%O"

Reads Melange.* custom fields from KiCad symbols to generate .pot, .wiper,
.switch, .gang, and .model directives.

Dependencies: Python 3.6+ stdlib only (no pip installs).
"""

import sys
import os
import re
import xml.etree.ElementTree as ET
from collections import defaultdict, OrderedDict


# --- Value formatting ---

def format_value(value_str):
    """Pass through value strings, normalizing common KiCad formatting."""
    if not value_str:
        return "0"
    # Strip units that melange doesn't use (Ω, F, H, V, A)
    s = value_str.strip().rstrip("ΩFHVAΩfhva")
    # KiCad sometimes uses 'meg' instead of 'MEG'
    s = re.sub(r'(?i)meg', 'MEG', s)
    return s if s else value_str.strip()


def sanitize_node(name):
    """Convert a KiCad net name to a melange-compatible node name."""
    if not name:
        return "0"
    low = name.strip().lower()
    if low in ("gnd", "0", "/gnd", "net-gnd"):
        return "0"
    # Remove leading '/' from hierarchical names
    s = name.lstrip("/")
    # Replace special chars with underscore
    s = re.sub(r'[^a-zA-Z0-9_]', '_', s)
    # Collapse multiple underscores
    s = re.sub(r'_+', '_', s).strip('_')
    # Ensure it doesn't start with a digit (prefix with 'n')
    if s and s[0].isdigit():
        s = "n" + s
    return s.lower() if s else "node"


# --- Component type mapping ---

# Map KiCad reference prefix to melange element handling
REF_PREFIX_MAP = {
    'R': 'resistor',
    'C': 'capacitor',
    'L': 'inductor',
    'D': 'diode',
    'Q': 'bjt',
    'J': 'jfet',
    'M': 'mosfet',
    'T': 'triode',
    'U': 'opamp',
    'V': 'vsource',
    'I': 'isource',
    'Y': 'vca',
    'X': 'subckt',
    'K': 'coupling',
    'E': 'vcvs',
    'G': 'vccs',
}


def get_ref_prefix(ref):
    """Extract the alphabetic prefix from a reference designator."""
    prefix = ''
    for ch in ref:
        if ch.isalpha():
            prefix += ch
        else:
            break
    return prefix


class Component:
    """Parsed KiCad component with fields and net connections."""

    def __init__(self, ref, value, libpart):
        self.ref = ref
        self.value = value
        self.libpart = libpart  # (lib, part) tuple
        self.fields = {}        # name -> value (includes Melange.* fields)
        self.pin_nets = {}      # pin_number -> net_name


class MelangeExporter:
    """Converts KiCad XML netlist to Melange .cir format."""

    def __init__(self, xml_path):
        self.tree = ET.parse(xml_path)
        self.root = self.tree.getroot()
        self.components = OrderedDict()   # ref -> Component
        self.nets = {}                     # net_name -> [(ref, pin)]
        self.title = ""
        self.pot_directives = []
        self.wiper_directives = []
        self.switch_directives = []
        self.gang_directives = []
        self.model_directives = []

    def parse(self):
        """Parse the XML netlist."""
        self._parse_design()
        self._parse_components()
        self._parse_nets()

    def _parse_design(self):
        """Extract schematic title from design section."""
        design = self.root.find("design")
        if design is not None:
            source = design.find("source")
            if source is not None and source.text:
                base = os.path.splitext(os.path.basename(source.text))[0]
                self.title = base.replace("_", " ").replace("-", " ").title()

    def _parse_components(self):
        """Parse component list with fields."""
        comps = self.root.find("components")
        if comps is None:
            return
        for comp_elem in comps.findall("comp"):
            ref = comp_elem.get("ref", "")
            value_elem = comp_elem.find("value")
            value = value_elem.text if value_elem is not None and value_elem.text else ""

            libsource = comp_elem.find("libsource")
            lib = libsource.get("lib", "") if libsource is not None else ""
            part = libsource.get("part", "") if libsource is not None else ""

            comp = Component(ref, value, (lib, part))

            # Parse custom fields
            fields = comp_elem.find("fields")
            if fields is not None:
                for field in fields.findall("field"):
                    name = field.get("name", "")
                    val = field.text if field.text else ""
                    comp.fields[name] = val

            self.components[ref] = comp

    def _parse_nets(self):
        """Parse net connectivity and assign to components."""
        nets_elem = self.root.find("nets")
        if nets_elem is None:
            return
        for net in nets_elem.findall("net"):
            net_name = net.get("name", "")
            nodes = []
            for node in net.findall("node"):
                ref = node.get("ref", "")
                pin = node.get("pin", "")
                pinfunction = node.get("pinfunction", "")
                pintype = node.get("pintype", "")
                nodes.append((ref, pin, pinfunction, pintype))
                # Assign to component
                if ref in self.components:
                    self.components[ref].pin_nets[pin] = net_name

            self.nets[net_name] = nodes

    def _get_node(self, comp, pin_number):
        """Get the sanitized melange node name for a component pin."""
        net_name = comp.pin_nets.get(str(pin_number), "")
        return sanitize_node(net_name)

    def _detect_input_output(self):
        """Find input and output nodes from Melange.Input/Output fields or AudioInput/AudioOutput symbols."""
        input_node = "in"
        output_node = "out"
        for comp in self.components.values():
            if comp.fields.get("Melange.Input", "").lower() == "true":
                # The pin of this component connects to the input net
                for pin, net in comp.pin_nets.items():
                    node = sanitize_node(net)
                    if node != "0":
                        input_node = node
                        break
            if comp.fields.get("Melange.Output", "").lower() == "true":
                for pin, net in comp.pin_nets.items():
                    node = sanitize_node(net)
                    if node != "0":
                        output_node = node
                        break
        return input_node, output_node

    def _emit_component(self, comp):
        """Generate melange netlist line for a component. Returns (line, model_line)."""
        ref = comp.ref
        prefix = get_ref_prefix(ref)
        comp_type = REF_PREFIX_MAP.get(prefix, None)

        if comp_type is None:
            return None, None

        model_line = None

        if comp_type == 'resistor':
            n1 = self._get_node(comp, "1")
            n2 = self._get_node(comp, "2")
            val = format_value(comp.value)
            line = f"{ref} {n1} {n2} {val}"
            # Check for pot directive
            pot_field = comp.fields.get("Melange.Pot", "")
            if pot_field:
                label = comp.fields.get("Melange.Label", "")
                parts = pot_field.split()
                min_v = parts[0] if len(parts) > 0 else "1k"
                max_v = parts[1] if len(parts) > 1 else "100k"
                default_v = parts[2] if len(parts) > 2 else ""
                pot_str = f".pot {ref} {min_v} {max_v}"
                if default_v:
                    pot_str += f" {default_v}"
                if label:
                    pot_str += f' "{label}"'
                self.pot_directives.append(pot_str)
            # Check for switch directive
            switch_field = comp.fields.get("Melange.Switch", "")
            if switch_field:
                label = comp.fields.get("Melange.Label", "")
                values = switch_field.strip()
                sw_str = f".switch {ref} {values}"
                if label:
                    sw_str += f' "{label}"'
                self.switch_directives.append(sw_str)
            # Check for gang
            gang_label = comp.fields.get("Melange.Gang", "")
            if gang_label:
                inverted = comp.fields.get("Melange.GangInvert", "").lower() == "true"
                self.gang_directives.append((gang_label, ref, inverted))

        elif comp_type == 'capacitor':
            n1 = self._get_node(comp, "1")
            n2 = self._get_node(comp, "2")
            val = format_value(comp.value)
            line = f"{ref} {n1} {n2} {val}"
            # Caps can also be switched
            switch_field = comp.fields.get("Melange.Switch", "")
            if switch_field:
                label = comp.fields.get("Melange.Label", "")
                values = switch_field.strip()
                sw_str = f".switch {ref} {values}"
                if label:
                    sw_str += f' "{label}"'
                self.switch_directives.append(sw_str)

        elif comp_type == 'inductor':
            n1 = self._get_node(comp, "1")
            n2 = self._get_node(comp, "2")
            val = format_value(comp.value)
            line = f"{ref} {n1} {n2} {val}"

        elif comp_type == 'diode':
            # KiCad: pin K=cathode, pin A=anode
            # Melange: D name anode cathode model
            anode = self._get_node(comp, "2")    # pin 2 = A (anode)
            cathode = self._get_node(comp, "1")  # pin 1 = K (cathode)
            model = comp.value
            line = f"{ref} {anode} {cathode} {model}"
            model_line = self._make_model(comp, "D")

        elif comp_type == 'bjt':
            # KiCad: pin 1=C, 2=B, 3=E
            # Melange: Q name C B E model
            nc = self._get_node(comp, "1")
            nb = self._get_node(comp, "2")
            ne = self._get_node(comp, "3")
            model = comp.value
            line = f"{ref} {nc} {nb} {ne} {model}"
            model_line = self._make_model(comp, None)  # Type from Melange.Model

        elif comp_type == 'jfet':
            # KiCad: pin 1=D, 2=G, 3=S
            # Melange: J name D G S model
            nd = self._get_node(comp, "1")
            ng = self._get_node(comp, "2")
            ns = self._get_node(comp, "3")
            model = comp.value
            line = f"{ref} {nd} {ng} {ns} {model}"
            model_line = self._make_model(comp, None)

        elif comp_type == 'mosfet':
            # KiCad: pin 1=D, 2=G, 3=S (4=B optional)
            # Melange: M name D G S B model
            nd = self._get_node(comp, "1")
            ng = self._get_node(comp, "2")
            ns = self._get_node(comp, "3")
            nb = self._get_node(comp, "4") if "4" in comp.pin_nets else ns
            model = comp.value
            line = f"{ref} {nd} {ng} {ns} {nb} {model}"
            model_line = self._make_model(comp, None)

        elif comp_type == 'triode':
            # Melange: T name grid plate cathode model
            ng = self._get_node(comp, "1")
            np_ = self._get_node(comp, "2")
            nk = self._get_node(comp, "3")
            model = comp.value
            line = f"{ref} {ng} {np_} {nk} {model}"
            model_line = self._make_model(comp, "TRIODE")

        elif comp_type == 'opamp':
            # Melange: U name n+ n- out model
            nplus = self._get_node(comp, "1")
            nminus = self._get_node(comp, "2")
            nout = self._get_node(comp, "3")
            model = comp.value
            line = f"{ref} {nplus} {nminus} {nout} {model}"
            model_line = self._make_model(comp, "OA")

        elif comp_type == 'vca':
            # Melange: Y name sig+ sig- ctrl+ ctrl- model
            sp = self._get_node(comp, "1")
            sn = self._get_node(comp, "2")
            cp = self._get_node(comp, "3")
            cn = self._get_node(comp, "4")
            model = comp.value
            line = f"{ref} {sp} {sn} {cp} {cn} {model}"
            model_line = self._make_model(comp, "VCA")

        elif comp_type == 'vsource':
            n1 = self._get_node(comp, "1")
            n2 = self._get_node(comp, "2")
            val = format_value(comp.value)
            line = f"{ref} {n1} {n2} DC {val}"

        elif comp_type == 'isource':
            n1 = self._get_node(comp, "1")
            n2 = self._get_node(comp, "2")
            val = format_value(comp.value)
            line = f"{ref} {n1} {n2} DC {val}"

        elif comp_type == 'vcvs':
            # E name out+ out- ctrl+ ctrl- gain
            n1 = self._get_node(comp, "1")
            n2 = self._get_node(comp, "2")
            n3 = self._get_node(comp, "3")
            n4 = self._get_node(comp, "4")
            val = format_value(comp.value)
            line = f"{ref} {n1} {n2} {n3} {n4} {val}"

        elif comp_type == 'vccs':
            n1 = self._get_node(comp, "1")
            n2 = self._get_node(comp, "2")
            n3 = self._get_node(comp, "3")
            n4 = self._get_node(comp, "4")
            val = format_value(comp.value)
            line = f"{ref} {n1} {n2} {n3} {n4} {val}"

        elif comp_type == 'coupling':
            # K name L1 L2 coefficient (value field has "L1 L2 coeff" or similar)
            line = f"{ref} {comp.value}"

        elif comp_type == 'subckt':
            # X name nodes... model
            nodes = []
            for pin_num in sorted(comp.pin_nets.keys(), key=lambda x: int(x) if x.isdigit() else 0):
                nodes.append(self._get_node(comp, pin_num))
            model = comp.value
            line = f"{ref} {' '.join(nodes)} {model}"

        else:
            return None, None

        return line, model_line

    def _make_model(self, comp, default_type):
        """Generate a .model directive from Melange.Model field."""
        model_field = comp.fields.get("Melange.Model", "")
        if not model_field:
            # Try Sim.Params
            sim_params = comp.fields.get("Sim.Params", "")
            if sim_params and default_type:
                return f".model {comp.value} {default_type}({sim_params})"
            return None
        # model_field is like "OA(AOL=200000 ROUT=75)" or "TRIODE(MU=100 ...)"
        return f".model {comp.value} {model_field}"

    def export(self):
        """Generate the complete melange .cir netlist string."""
        self.parse()

        lines = []
        lines.append(self.title or "KiCad Imported Circuit")

        # Collect models (deduplicated by name)
        models = OrderedDict()
        component_lines = []

        # Process wiper components first to pair them
        wiper_pairs = {}  # label -> (cw_ref, ccw_ref, total_r, default_pos)
        for comp in self.components.values():
            wiper_field = comp.fields.get("Melange.Wiper", "")
            if wiper_field:
                label = comp.fields.get("Melange.Label", "")
                parts = wiper_field.split()
                total_r = parts[0] if parts else "100k"
                default_pos = parts[1] if len(parts) > 1 else "0.5"
                # Determine CW vs CCW from pin connectivity
                # Convention: this component IS the wiper (both legs)
                # The component ref is used as both R_cw and R_ccw with _cw/_ccw suffixes
                wiper_pairs[comp.ref] = (total_r, default_pos, label)

        for comp in self.components.values():
            # Skip audio I/O markers (not circuit elements)
            if comp.fields.get("Melange.Input", "").lower() == "true":
                continue
            if comp.fields.get("Melange.Output", "").lower() == "true":
                continue

            line, model_line = self._emit_component(comp)
            if line:
                component_lines.append(line)
            if model_line and comp.value not in models:
                models[comp.value] = model_line

        # Write models
        if models:
            lines.append("")
            lines.append("* --- Device Models ---")
            for model_line in models.values():
                lines.append(model_line)

        # Write components
        lines.append("")
        lines.append("* --- Circuit ---")
        for cl in component_lines:
            lines.append(cl)

        # Write directives
        directives = []
        directives.extend(self.pot_directives)

        # Wiper directives for Melange_Wiper components
        for ref, (total_r, default_pos, label) in wiper_pairs.items():
            cw_ref = f"{ref}_cw"
            ccw_ref = f"{ref}_ccw"
            w_str = f".wiper {cw_ref} {ccw_ref} {total_r} {default_pos}"
            if label:
                w_str += f' "{label}"'
            directives.append(w_str)

        directives.extend(self.switch_directives)

        # Gang directives: group by label
        gang_groups = defaultdict(list)
        for (label, ref, inverted) in self.gang_directives:
            prefix = "!" if inverted else ""
            gang_groups[label].append(f"{prefix}{ref}")
        for label, members in gang_groups.items():
            if len(members) >= 2:
                directives.append(f'.gang "{label}" {" ".join(members)}')

        if directives:
            lines.append("")
            lines.append("* --- Controls ---")
            for d in directives:
                lines.append(d)

        lines.append("")
        lines.append(".END")
        lines.append("")

        return "\n".join(lines)


def main():
    if len(sys.argv) < 3:
        print(f"Usage: {sys.argv[0]} <input.xml> <output.cir>", file=sys.stderr)
        print(f"  Converts KiCad XML intermediate netlist to Melange .cir format.", file=sys.stderr)
        print(f"", file=sys.stderr)
        print(f"  KiCad plugin: Register as netlist generator with:", file=sys.stderr)
        print(f'    python {os.path.abspath(__file__)} "%I" "%O"', file=sys.stderr)
        sys.exit(1)

    xml_path = sys.argv[1]
    cir_path = sys.argv[2]

    if not os.path.exists(xml_path):
        print(f"Error: Input file not found: {xml_path}", file=sys.stderr)
        sys.exit(1)

    try:
        exporter = MelangeExporter(xml_path)
        result = exporter.export()
    except ET.ParseError as e:
        print(f"Error: Failed to parse XML: {e}", file=sys.stderr)
        sys.exit(1)
    except Exception as e:
        print(f"Error: {e}", file=sys.stderr)
        sys.exit(1)

    with open(cir_path, "w") as f:
        f.write(result)

    print(f"Wrote melange netlist to: {cir_path}", file=sys.stderr)
    comp_count = sum(1 for c in exporter.components.values()
                     if not c.fields.get("Melange.Input", "").lower() == "true"
                     and not c.fields.get("Melange.Output", "").lower() == "true")
    print(f"  Components: {comp_count}", file=sys.stderr)
    print(f"  Models: {len([m for m in exporter.model_directives])}", file=sys.stderr)
    print(f"  Pots: {len(exporter.pot_directives)}", file=sys.stderr)
    print(f"  Switches: {len(exporter.switch_directives)}", file=sys.stderr)


if __name__ == "__main__":
    main()
