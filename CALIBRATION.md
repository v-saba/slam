# Camera Calibration Guide

This project now includes a comprehensive camera calibration pipeline to improve SLAM accuracy.

## Quick Start

### 1. Build the Project
```bash
cmake -S . -B build
cmake --build build
```

This creates two executables:
- `build/slam` - Main SLAM application
- `build/calibrate` - Camera calibration tool

### 2. Print Checkerboard Pattern

You need a checkerboard pattern for calibration. The default configuration expects:
- **9×6 internal corners** (10×7 squares total)
- **25mm square size**

You can download patterns from:
- [OpenCV Calibration Patterns](https://docs.opencv.org/4.x/da/d0d/tutorial_camera_calibration_pattern.html)
- Or search "checkerboard calibration pattern 9x6" online

### 3. Calibrate Your Camera

#### Option A: Using Existing Images
```bash
# Place checkerboard images in a directory
mkdir calibration_images
# Add your checkerboard photos to this directory

# Run calibration
./build/calibrate images calibration_images
```

#### Option B: Live Camera Calibration
```bash
# Use live camera (camera ID 0)
./build/calibrate live 0

# Press SPACE to capture images
# Press 'q' when you have enough images (minimum 10)
# Press ESC to cancel
```

### 4. Use Calibrated SLAM
```bash
# Run SLAM with calibration file
./build/slam data/IMG_1956.jpg data/IMG_1957.jpg output.ply camera_calibration.json

# Or without calibration (uses estimates)
./build/slam data/IMG_1956.jpg data/IMG_1957.jpg output.ply
```

## Calibration Tips

### Taking Good Calibration Images
1. **Fill the frame**: The checkerboard should cover most of the image
2. **Various angles**: Tilt the board in different directions
3. **Various distances**: Close and far shots
4. **Edge coverage**: Include shots with the board near image edges
5. **Avoid motion blur**: Keep the camera steady
6. **Good lighting**: Ensure clear contrast between squares

### Recommended Images
- **Minimum**: 10 images (but more is better)
- **Recommended**: 15-20 images
- **Ideal**: 25+ images from various poses

## Calibration Output

The calibration saves results to `camera_calibration.json`:

```json
{
  "camera_matrix": {
    "fx": 8064.123456,
    "fy": 8064.234567,
    "cx": 2016.345678,
    "cy": 1134.456789
  },
  "distortion_coefficients": [0.123, -0.456, 0.001, 0.002, 0.789],
  "image_size": {
    "width": 4032,
    "height": 2268
  },
  "metadata": {
    "reprojection_error": 0.234567,
    "num_images": 15,
    "board_size": [9, 6],
    "square_size_mm": 25.0,
    "calibration_date": 1642685400
  }
}
```

## Advanced Usage

### Custom Checkerboard Size
```bash
# For 7×5 internal corners with 30mm squares
./build/calibrate images calibration_images --board-size 7x5 --square-size 30
```

### Custom Output File
```bash
./build/calibrate images calibration_images --output my_camera.json
```

### Disable Visualization
```bash
./build/calibrate images calibration_images --no-display
```

### All Options
```bash
./build/calibrate <mode> [options]

Modes:
  images <directory>     - Calibrate using images from directory
  live [camera_id]       - Calibrate using live camera (default id: 0)

Options:
  --board-size WxH       - Checkerboard size (default: 9x6)
  --square-size SIZE     - Square size in mm (default: 25.0)
  --min-images N         - Minimum images needed (default: 10)
  --output FILE          - Output JSON file (default: camera_calibration.json)
  --no-display           - Don't show detection visualization
```

## Troubleshooting

### "No checkerboard found"
- Ensure good lighting and contrast
- Check the board size parameters match your pattern
- Try different angles and distances
- Make sure the entire checkerboard is visible

### High Reprojection Error
- Take more images from different angles
- Ensure the checkerboard is flat (not warped)
- Use better lighting
- Check that square size is correct

### SLAM Still Inaccurate
- Verify calibration quality (low reprojection error)
- Ensure the calibration was done with the same camera/settings
- Check that the JSON file is being loaded correctly

## Why Calibrate?

**Before Calibration** (estimated parameters):
- Approximate focal length from EXIF data
- Assumed principal point at image center
- No distortion correction
- Reduced 3D reconstruction accuracy

**After Calibration** (measured parameters):
- Precise focal length in pixels
- Actual principal point location
- Lens distortion coefficients
- Significantly improved SLAM accuracy

The calibration process typically improves 3D reconstruction accuracy by **2-5×** compared to estimated parameters.
