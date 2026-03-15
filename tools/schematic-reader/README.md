# schematic-reader

Standalone Python tool for preprocessing schematic images for AI vision readability.

Claude downsamples all images to max 1568px on the long edge (~1.15 MP). Sending oversized images wastes detail — the extra pixels are silently discarded. This tool renders targeted crops at controlled DPI and preprocesses them for optimal AI readability.

## Setup

```bash
pip install -r tools/schematic-reader/requirements.txt
```

Dependencies: opencv-python-headless, pymupdf (for PDF support), numpy.

## Processing Pipeline

```
RGB -> Grayscale -> Denoise (NLM) -> CLAHE contrast -> Unsharp mask -> White border crop -> Resize
```

- **Denoise**: Non-local means denoising removes scan noise without blurring edges
- **CLAHE**: Adaptive histogram equalization brings out faint annotations and labels
- **Unsharp mask**: Sharpens wire traces and text
- **Border crop**: Removes white borders automatically
- **Resize**: Fits within Claude's 1500px / 1.15MP limits using INTER_AREA (best for line art)

Use `--no-enhance` on any command to skip the enhancement pipeline and only resize.

## Commands

### render

Extract a region from a PDF at high DPI and preprocess.

```bash
# Full page at 300 DPI
python tools/schematic-reader/schematic_reader.py render schematic.pdf --dpi 300

# Specific region from page 7 at 1200 DPI
python tools/schematic-reader/schematic_reader.py render schematic.pdf \
    --page 7 --rect 0.1,0.2,0.5,0.6 --dpi 1200

# Render without enhancement
python tools/schematic-reader/schematic_reader.py render schematic.pdf \
    --page 0 --rect 0.0,0.0,0.5,0.5 --no-enhance -o /tmp/raw_render.png
```

`--rect` uses normalized 0-1 coordinates: x0,y0,x1,y1 relative to the full page.

### enhance

Enhance an existing image file (PNG, JPG, TIFF).

```bash
# Full enhancement pipeline
python tools/schematic-reader/schematic_reader.py enhance screenshot.png

# Just convert to grayscale and resize for Claude (no denoising/contrast/sharpening)
python tools/schematic-reader/schematic_reader.py enhance screenshot.png --no-enhance

# Specify output path
python tools/schematic-reader/schematic_reader.py enhance photo.jpg -o /tmp/enhanced.png
```

### crop

Crop a region from an image. Supports both normalized (0-1) and pixel coordinates.

```bash
# Normalized coordinates (default) — top-left quadrant
python tools/schematic-reader/schematic_reader.py crop image.png --rect 0.0,0.0,0.5,0.5

# Pixel coordinates
python tools/schematic-reader/schematic_reader.py crop image.png \
    --rect 100,200,500,600 --coords pixel

# Crop without enhancement
python tools/schematic-reader/schematic_reader.py crop scan.tiff \
    --rect 0.2,0.3,0.8,0.9 --no-enhance
```

### tile

Generate overlapping tiles from a large image for reading sequentially. Automatically skips mostly-white tiles.

```bash
# Default settings (1400px tiles, 200px overlap)
python tools/schematic-reader/schematic_reader.py tile large_scan.png

# Custom tile size and overlap
python tools/schematic-reader/schematic_reader.py tile large_scan.png \
    --tile-size 1200 --overlap 300

# Tiles without enhancement, custom output directory
python tools/schematic-reader/schematic_reader.py tile large_scan.png \
    --no-enhance --output-dir /tmp/my_tiles/
```

## Output

All commands output PNG files. If no `--output` / `-o` is specified, files are auto-named and saved to `/tmp/`.

## Claude Vision Constraints

| Constraint | Value | Tool target |
|-----------|-------|-------------|
| Max long edge | 1568px | 1500px (headroom) |
| Max total pixels | ~1.15 MP | Respected |
| Best format | PNG | Always PNG output |
| Downsampling | Automatic by Claude | Pre-downsample with INTER_AREA |
