#!/usr/bin/env python3
"""Schematic image preprocessing for AI vision readability.

Preprocesses schematic images (PDF, PNG, JPG, TIFF) for optimal readability
by AI vision models like Claude. Handles the key constraint that Claude
downsamples all images to max 1568px on the long edge (~1.15 MP), so sending
oversized images wastes detail.

Pipeline: RGB -> Grayscale -> Denoise -> CLAHE contrast -> Unsharp mask -> Resize

Commands:
    render  — Extract a region from a PDF at high DPI, preprocess for readability
    enhance — Enhance an existing image (denoise, contrast, sharpen)
    tile    — Generate overlapping tiles from a large image for sequential AI reading
    crop    — Crop a region from an image using normalized or pixel coordinates

Usage:
    # Render region from PDF page 7
    python schematic_reader.py render schematic.pdf --page 7 --rect 0.1,0.2,0.5,0.6 --dpi 1200

    # Enhance an existing screenshot
    python schematic_reader.py enhance screenshot.png

    # Enhance with no processing (just resize for Claude)
    python schematic_reader.py enhance screenshot.png --no-enhance

    # Crop a region from a PNG (normalized coordinates, default)
    python schematic_reader.py crop image.png --rect 0.1,0.2,0.5,0.6

    # Crop using pixel coordinates
    python schematic_reader.py crop image.png --rect 100,200,500,600 --coords pixel

    # Generate tiles for reading a large schematic
    python schematic_reader.py tile large_scan.png --tile-size 1400 --overlap 200
"""

import argparse
import sys
import tempfile
from pathlib import Path

import cv2
import numpy as np

# ---------------------------------------------------------------------------
# Claude vision constraints
# ---------------------------------------------------------------------------
MAX_LONG_EDGE = 1500  # Leave headroom below 1568px hard limit
MAX_PIXELS = 1_150_000  # ~1.15 MP max

# ---------------------------------------------------------------------------
# Enhancement defaults
# ---------------------------------------------------------------------------
CLAHE_CLIP_LIMIT = 2.5
CLAHE_TILE_GRID = (8, 8)
UNSHARP_SIGMA = 1.0
UNSHARP_STRENGTH = 1.5
DENOISE_H = 8
DENOISE_TEMPLATE_WINDOW = 7
DENOISE_SEARCH_WINDOW = 21
BORDER_THRESHOLD = 240
BORDER_MARGIN = 20

# ---------------------------------------------------------------------------
# Supported image extensions
# ---------------------------------------------------------------------------
IMAGE_EXTENSIONS = {".png", ".jpg", ".jpeg", ".tiff", ".tif", ".bmp", ".webp"}


# ===================================================================
# Image processing pipeline
# ===================================================================

def enhance_image(img_gray: np.ndarray) -> np.ndarray:
    """Apply the full enhancement pipeline to a grayscale image.

    Pipeline: Denoise -> CLAHE contrast -> Unsharp mask sharpening.
    Optimized for schematic line art readability.
    """
    # Non-local means denoising — good for scan noise without blurring edges
    denoised = cv2.fastNlMeansDenoising(
        img_gray,
        h=DENOISE_H,
        templateWindowSize=DENOISE_TEMPLATE_WINDOW,
        searchWindowSize=DENOISE_SEARCH_WINDOW,
    )

    # CLAHE (Contrast Limited Adaptive Histogram Equalization)
    # Brings out faint annotations and component labels
    clahe = cv2.createCLAHE(clipLimit=CLAHE_CLIP_LIMIT, tileGridSize=CLAHE_TILE_GRID)
    enhanced = clahe.apply(denoised)

    # Unsharp mask — sharpens edges (wire traces, text)
    blurred = cv2.GaussianBlur(enhanced, (0, 0), UNSHARP_SIGMA)
    sharpened = cv2.addWeighted(
        enhanced, 1.0 + UNSHARP_STRENGTH, blurred, -UNSHARP_STRENGTH, 0
    )

    return sharpened


def crop_white_borders(
    img: np.ndarray,
    threshold: int = BORDER_THRESHOLD,
    margin: int = BORDER_MARGIN,
) -> np.ndarray:
    """Remove white borders from a grayscale image, keeping a small margin."""
    mask = img < threshold
    if not mask.any():
        return img

    rows = np.any(mask, axis=1)
    cols = np.any(mask, axis=0)
    rmin, rmax = np.where(rows)[0][[0, -1]]
    cmin, cmax = np.where(cols)[0][[0, -1]]

    h, w = img.shape[:2]
    rmin = max(0, rmin - margin)
    rmax = min(h - 1, rmax + margin)
    cmin = max(0, cmin - margin)
    cmax = min(w - 1, cmax + margin)

    return img[rmin : rmax + 1, cmin : cmax + 1]


def resize_for_claude(img: np.ndarray) -> np.ndarray:
    """Resize image to fit within Claude's vision constraints.

    Max 1500px on long edge, max 1.15 MP total.
    Uses INTER_AREA for downsampling (best for schematic line art).
    """
    h, w = img.shape[:2]
    total_pixels = h * w

    scale = 1.0
    long_edge = max(h, w)
    if long_edge > MAX_LONG_EDGE:
        scale = min(scale, MAX_LONG_EDGE / long_edge)
    if total_pixels > MAX_PIXELS:
        scale = min(scale, (MAX_PIXELS / total_pixels) ** 0.5)

    if scale >= 1.0:
        return img

    new_w = max(1, int(w * scale))
    new_h = max(1, int(h * scale))
    return cv2.resize(img, (new_w, new_h), interpolation=cv2.INTER_AREA)


def to_grayscale(img: np.ndarray) -> np.ndarray:
    """Convert an image to grayscale if it is not already."""
    if len(img.shape) == 3:
        if img.shape[2] == 4:
            return cv2.cvtColor(img, cv2.COLOR_BGRA2GRAY)
        return cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    return img


def process_image(img: np.ndarray, do_enhance: bool = True) -> np.ndarray:
    """Full preprocessing pipeline.

    If do_enhance is True:  grayscale -> enhance -> crop borders -> resize
    If do_enhance is False: grayscale -> resize (passthrough mode)
    """
    gray = to_grayscale(img)

    if do_enhance:
        enhanced = enhance_image(gray)
        cropped = crop_white_borders(enhanced)
        return resize_for_claude(cropped)
    else:
        return resize_for_claude(gray)


# ===================================================================
# Input loading helpers
# ===================================================================

def load_image(path: str) -> np.ndarray:
    """Load an image file (PNG, JPG, TIFF, etc.) and return as numpy array.

    Returns the image in BGR or grayscale format as read by OpenCV.
    """
    p = Path(path)
    if not p.exists():
        print(f"Error: File not found: {p}", file=sys.stderr)
        sys.exit(1)

    ext = p.suffix.lower()
    if ext not in IMAGE_EXTENSIONS:
        print(
            f"Error: Unsupported image format '{ext}'. "
            f"Supported: {', '.join(sorted(IMAGE_EXTENSIONS))}",
            file=sys.stderr,
        )
        sys.exit(1)

    img = cv2.imread(str(p), cv2.IMREAD_UNCHANGED)
    if img is None:
        print(f"Error: Could not read image: {p}", file=sys.stderr)
        sys.exit(1)

    return img


def render_from_pdf(
    pdf_path: str,
    rect: tuple[float, float, float, float] | None,
    dpi: int = 600,
    page_num: int = 0,
) -> np.ndarray:
    """Render a region (or full page) from a PDF and return as numpy array.

    Args:
        pdf_path: Path to the PDF file.
        rect: Normalized coordinates (x0, y0, x1, y1) where 0-1 maps to full
              page. If None, renders the entire page.
        dpi: Render resolution.
        page_num: PDF page number (0-indexed).

    Returns:
        Grayscale numpy array of the rendered region.
    """
    try:
        import fitz  # PyMuPDF
    except ImportError:
        print(
            "Error: PyMuPDF not installed. Run: pip install pymupdf",
            file=sys.stderr,
        )
        sys.exit(1)

    p = Path(pdf_path)
    if not p.exists():
        print(f"Error: File not found: {p}", file=sys.stderr)
        sys.exit(1)

    doc = fitz.open(str(p))
    if page_num < 0 or page_num >= len(doc):
        print(
            f"Error: Page {page_num} out of range (PDF has {len(doc)} pages)",
            file=sys.stderr,
        )
        doc.close()
        sys.exit(1)

    page = doc[page_num]
    page_rect = page.rect

    # Build clip rectangle
    clip = None
    if rect is not None:
        x0 = page_rect.x0 + rect[0] * page_rect.width
        y0 = page_rect.y0 + rect[1] * page_rect.height
        x1 = page_rect.x0 + rect[2] * page_rect.width
        y1 = page_rect.y0 + rect[3] * page_rect.height
        clip = fitz.Rect(x0, y0, x1, y1)

    # Render at specified DPI
    mat = fitz.Matrix(dpi / 72, dpi / 72)
    pix = page.get_pixmap(matrix=mat, clip=clip)

    # Convert to numpy array
    img = np.frombuffer(pix.samples, dtype=np.uint8)
    if pix.n == 4:  # RGBA
        img = img.reshape(pix.h, pix.w, 4)
        img = cv2.cvtColor(img, cv2.COLOR_RGBA2GRAY)
    elif pix.n == 3:  # RGB
        img = img.reshape(pix.h, pix.w, 3)
        img = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)
    else:  # Already grayscale
        img = img.reshape(pix.h, pix.w)

    doc.close()
    return img


def auto_output_path(input_path: str | None, suffix: str, output: str | None) -> Path:
    """Determine output path: use --output if given, else auto-name in /tmp/."""
    if output:
        p = Path(output)
        p.parent.mkdir(parents=True, exist_ok=True)
        return p

    if input_path:
        stem = Path(input_path).stem
    else:
        stem = "schematic"

    return Path(tempfile.gettempdir()) / f"{stem}_{suffix}.png"


def parse_rect(rect_str: str) -> tuple[float, float, float, float]:
    """Parse a comma-separated rect string into a 4-tuple of floats."""
    parts = rect_str.split(",")
    if len(parts) != 4:
        print(
            "Error: --rect must be 4 comma-separated values: x0,y0,x1,y1",
            file=sys.stderr,
        )
        sys.exit(1)
    try:
        return tuple(float(x) for x in parts)
    except ValueError:
        print(
            "Error: --rect values must be numbers: x0,y0,x1,y1",
            file=sys.stderr,
        )
        sys.exit(1)


# ===================================================================
# Commands
# ===================================================================

def cmd_render(args: argparse.Namespace) -> None:
    """Extract a region from a PDF at high DPI and preprocess for readability."""
    pdf_path = args.input
    dpi = args.dpi
    page_num = args.page

    rect = None
    if args.rect:
        rect = parse_rect(args.rect)
        # Validate normalized coordinates
        for v in rect:
            if v < 0.0 or v > 1.0:
                print(
                    "Error: --rect values must be in 0-1 range for PDF rendering",
                    file=sys.stderr,
                )
                sys.exit(1)

    print(f"Rendering page {page_num} at {dpi} DPI", end="")
    if rect:
        print(f", rect=({rect[0]:.3f},{rect[1]:.3f},{rect[2]:.3f},{rect[3]:.3f})", end="")
    print("...")

    raw = render_from_pdf(pdf_path, rect, dpi, page_num)
    print(f"  Raw render: {raw.shape[1]}x{raw.shape[0]} ({raw.shape[0] * raw.shape[1]:,} pixels)")

    result = process_image(raw, do_enhance=not args.no_enhance)
    print(f"  After processing: {result.shape[1]}x{result.shape[0]} ({result.shape[0] * result.shape[1]:,} pixels)")

    rect_label = f"{rect[0]:.1f}-{rect[1]:.1f}-{rect[2]:.1f}-{rect[3]:.1f}" if rect else "full"
    output_path = auto_output_path(pdf_path, f"p{page_num}_{rect_label}_{dpi}dpi", args.output)

    cv2.imwrite(str(output_path), result)
    print(f"  Saved: {output_path}")


def cmd_enhance(args: argparse.Namespace) -> None:
    """Enhance an existing image (denoise, contrast, sharpen, resize)."""
    img = load_image(args.input)
    print(f"Input: {img.shape[1]}x{img.shape[0]} ({img.shape[0] * img.shape[1]:,} pixels)")

    result = process_image(img, do_enhance=not args.no_enhance)
    print(f"Output: {result.shape[1]}x{result.shape[0]} ({result.shape[0] * result.shape[1]:,} pixels)")

    output_path = auto_output_path(args.input, "enhanced", args.output)
    cv2.imwrite(str(output_path), result)
    print(f"Saved: {output_path}")


def cmd_crop(args: argparse.Namespace) -> None:
    """Crop a region from an image using normalized (0-1) or pixel coordinates."""
    img = load_image(args.input)
    h, w = img.shape[:2]
    print(f"Input: {w}x{h} ({h * w:,} pixels)")

    rect = parse_rect(args.rect)

    if args.coords == "normalized":
        # Convert normalized 0-1 coordinates to pixel coordinates
        for v in rect:
            if v < 0.0 or v > 1.0:
                print(
                    "Error: Normalized --rect values must be in 0-1 range. "
                    "Use --coords pixel for pixel coordinates.",
                    file=sys.stderr,
                )
                sys.exit(1)
        x0 = int(rect[0] * w)
        y0 = int(rect[1] * h)
        x1 = int(rect[2] * w)
        y1 = int(rect[3] * h)
    else:
        # Pixel coordinates
        x0, y0, x1, y1 = int(rect[0]), int(rect[1]), int(rect[2]), int(rect[3])

    # Clamp to image bounds
    x0 = max(0, min(x0, w))
    y0 = max(0, min(y0, h))
    x1 = max(0, min(x1, w))
    y1 = max(0, min(y1, h))

    if x1 <= x0 or y1 <= y0:
        print(
            f"Error: Invalid crop region ({x0},{y0})-({x1},{y1}). "
            "x1 must be > x0 and y1 must be > y0.",
            file=sys.stderr,
        )
        sys.exit(1)

    cropped = img[y0:y1, x0:x1]
    print(f"  Cropped: {cropped.shape[1]}x{cropped.shape[0]} ({cropped.shape[0] * cropped.shape[1]:,} pixels)")

    result = process_image(cropped, do_enhance=not args.no_enhance)
    print(f"  After processing: {result.shape[1]}x{result.shape[0]} ({result.shape[0] * result.shape[1]:,} pixels)")

    output_path = auto_output_path(args.input, f"crop_{x0}-{y0}-{x1}-{y1}", args.output)
    cv2.imwrite(str(output_path), result)
    print(f"  Saved: {output_path}")


def cmd_tile(args: argparse.Namespace) -> None:
    """Generate overlapping tiles from a large image for sequential AI reading."""
    img = load_image(args.input)
    gray = to_grayscale(img)

    h, w = gray.shape[:2]
    tile_size = args.tile_size
    overlap = args.overlap
    step = tile_size - overlap

    if step <= 0:
        print(
            f"Error: Overlap ({overlap}) must be less than tile size ({tile_size})",
            file=sys.stderr,
        )
        sys.exit(1)

    # Determine output directory
    if args.output_dir:
        output_dir = Path(args.output_dir)
    else:
        output_dir = Path(tempfile.gettempdir()) / f"{Path(args.input).stem}_tiles"
    output_dir.mkdir(parents=True, exist_ok=True)

    do_enhance = not args.no_enhance
    print(f"Input: {w}x{h}, tile_size={tile_size}, overlap={overlap}, step={step}")

    tile_num = 0
    skipped = 0
    stem = Path(args.input).stem

    for y in range(0, h, step):
        for x in range(0, w, step):
            y1 = min(y + tile_size, h)
            x1 = min(x + tile_size, w)
            tile = gray[y:y1, x:x1]

            # Skip mostly-white tiles (no useful content)
            if np.mean(tile) > BORDER_THRESHOLD:
                skipped += 1
                continue

            if do_enhance:
                processed = enhance_image(tile)
            else:
                processed = tile
            processed = resize_for_claude(processed)

            tile_path = output_dir / f"{stem}_tile_{tile_num:03d}_r{y}_c{x}.png"
            cv2.imwrite(str(tile_path), processed)
            tile_num += 1

    print(f"Generated {tile_num} tiles in {output_dir}/")
    if skipped:
        print(f"  Skipped {skipped} mostly-white tiles")


# ===================================================================
# CLI
# ===================================================================

def main() -> None:
    parser = argparse.ArgumentParser(
        description="Preprocess schematic images for AI vision readability",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""\
examples:
  %(prog)s render schematic.pdf --page 7 --rect 0.1,0.2,0.5,0.6 --dpi 1200
  %(prog)s enhance screenshot.png
  %(prog)s enhance screenshot.png --no-enhance
  %(prog)s crop image.png --rect 0.1,0.2,0.5,0.6
  %(prog)s crop image.png --rect 100,200,500,600 --coords pixel
  %(prog)s tile large_scan.png --tile-size 1400 --overlap 200
""",
    )
    subparsers = parser.add_subparsers(dest="command", required=True)

    # --- render -----------------------------------------------------------
    p_render = subparsers.add_parser(
        "render",
        help="Extract a region from a PDF at high DPI, preprocess for readability",
    )
    p_render.add_argument("input", help="PDF file path")
    p_render.add_argument(
        "--rect",
        help="Region as x0,y0,x1,y1 (normalized 0-1 coordinates)",
    )
    p_render.add_argument(
        "--dpi", type=int, default=600, help="Render DPI (default: 600)"
    )
    p_render.add_argument(
        "--page", type=int, default=0, help="PDF page number, 0-indexed (default: 0)"
    )
    p_render.add_argument("--output", "-o", help="Output PNG path (default: auto in /tmp/)")
    p_render.add_argument(
        "--no-enhance",
        action="store_true",
        help="Skip enhancement pipeline, just resize for Claude",
    )
    p_render.set_defaults(func=cmd_render)

    # --- enhance ----------------------------------------------------------
    p_enhance = subparsers.add_parser(
        "enhance",
        help="Enhance an existing image (denoise, contrast, sharpen)",
    )
    p_enhance.add_argument("input", help="Input image path (PNG, JPG, TIFF)")
    p_enhance.add_argument("--output", "-o", help="Output PNG path (default: auto in /tmp/)")
    p_enhance.add_argument(
        "--no-enhance",
        action="store_true",
        help="Skip enhancement pipeline, just convert to grayscale and resize",
    )
    p_enhance.set_defaults(func=cmd_enhance)

    # --- crop -------------------------------------------------------------
    p_crop = subparsers.add_parser(
        "crop",
        help="Crop a region from an image",
    )
    p_crop.add_argument("input", help="Input image path (PNG, JPG, TIFF)")
    p_crop.add_argument(
        "--rect",
        required=True,
        help="Region as x0,y0,x1,y1",
    )
    p_crop.add_argument(
        "--coords",
        choices=["normalized", "pixel"],
        default="normalized",
        help="Coordinate system for --rect (default: normalized 0-1)",
    )
    p_crop.add_argument("--output", "-o", help="Output PNG path (default: auto in /tmp/)")
    p_crop.add_argument(
        "--no-enhance",
        action="store_true",
        help="Skip enhancement pipeline, just crop and resize",
    )
    p_crop.set_defaults(func=cmd_crop)

    # --- tile -------------------------------------------------------------
    p_tile = subparsers.add_parser(
        "tile",
        help="Generate overlapping tiles from a large image for sequential AI reading",
    )
    p_tile.add_argument("input", help="Input image path (PNG, JPG, TIFF)")
    p_tile.add_argument(
        "--tile-size",
        type=int,
        default=1400,
        help="Tile size in pixels (default: 1400)",
    )
    p_tile.add_argument(
        "--overlap",
        type=int,
        default=200,
        help="Overlap between tiles in pixels (default: 200)",
    )
    p_tile.add_argument(
        "--output-dir",
        help="Output directory (default: auto in /tmp/)",
    )
    p_tile.add_argument(
        "--no-enhance",
        action="store_true",
        help="Skip enhancement pipeline, just tile and resize",
    )
    p_tile.set_defaults(func=cmd_tile)

    args = parser.parse_args()
    args.func(args)


if __name__ == "__main__":
    main()
