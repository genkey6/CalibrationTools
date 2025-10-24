# ChArUco Board Calibration Support

## Overview

This document describes the ChArUco board support added to the tag_based_sfm_calibrator for camera intrinsic calibration.

## What is ChArUco?

ChArUco boards combine the features of chessboard patterns and ArUco markers:
- **Chessboard corners** provide high-precision calibration points
- **ArUco markers** enable robust partial detection and corner identification
- Even if part of the board is occluded, calibration can still proceed with the visible corners

## Configuration

### Parameter Settings

To use ChArUco board for intrinsic calibration, set the following parameters in your configuration file:

```yaml
initial_intrinsic_calibration:
  board_type: "charuco"  # Select ChArUco board type
  tangent_distortion: true
  radial_distortion_coeffs: 2
  debug: true

  charuco:
    squares_x: 9         # Number of squares in X direction
    squares_y: 7         # Number of squares in Y direction
    square_length: 0.035 # Size of squares in meters
    marker_length: 0.025 # Size of ArUco markers in meters
    dictionary: "DICT_4X4_50" # ArUco dictionary type
```

### Supported ArUco Dictionaries

The following ArUco dictionaries are supported:
- `DICT_4X4_50`, `DICT_4X4_100`, `DICT_4X4_250`, `DICT_4X4_1000`
- `DICT_5X5_50`, `DICT_5X5_100`, `DICT_5X5_250`, `DICT_5X5_1000`
- `DICT_6X6_50`, `DICT_6X6_100`, `DICT_6X6_250`, `DICT_6X6_1000`
- `DICT_7X7_50`, `DICT_7X7_100`, `DICT_7X7_250`, `DICT_7X7_1000`
- `DICT_ARUCO_ORIGINAL`

## Creating a ChArUco Board

### Physical Board Requirements

1. **Size Constraint**: `square_length` must be greater than `marker_length`
2. **Recommended Ratio**: marker_length should be 60-80% of square_length for optimal detection
3. **Print Quality**: High contrast printing with minimal distortion

### Generating Board Images

You can generate ChArUco board images using OpenCV:

```python
import cv2
import cv2.aruco as aruco

# Create dictionary
dictionary = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)

# Create ChArUco board
board = aruco.CharucoBoard((9, 7), 0.035, 0.025, dictionary)

# Generate board image (1000x1000 pixels)
img = board.generateImage((1000, 1000))

# Save the image
cv2.imwrite("charuco_board.png", img)
```

## Advantages over Traditional Methods

### Compared to Chessboard
- **Partial visibility**: Can calibrate even when board is partially occluded
- **Automatic corner identification**: No ambiguity in corner ordering
- **Better for large FOV cameras**: Corners can be detected even at extreme angles

### Compared to AprilTag
- **Higher precision**: Chessboard corners provide sub-pixel accuracy
- **More calibration points**: Typically provides more points per image
- **Better coverage**: Uniform distribution of points across the image

## Usage Tips

1. **Image Collection**
   - Capture images from various angles and distances
   - Ensure the board covers different regions of the image
   - Aim for at least 20-30 images for robust calibration

2. **Lighting Conditions**
   - Ensure uniform lighting without shadows on the board
   - Avoid overexposure that might wash out marker details
   - Minimize reflections on glossy boards

3. **Board Positioning**
   - Include images with the board tilted at various angles
   - Cover the entire field of view across all images
   - Maintain focus - blurry images will reduce calibration accuracy

## Implementation Details

The ChArUco calibration is implemented in:
- Header: `include/tag_based_sfm_calibrator/intrinsics_calibration/charuco_calibrator.hpp`
- Source: `src/intrinsics_calibration/charuco_calibrator.cpp`

The implementation follows the same pattern as existing calibrators:
1. Inherits from `IntrinsicsCalibrator` base class
2. Implements `extractCalibrationPoints()` for corner detection
3. Implements `writeDebugImages()` for visualization
4. Integrates seamlessly with the existing calibration pipeline

## Debug Output

When `debug: true` is set, the calibrator generates debug images:
- `*_charuco_distorted.jpg`: Original image with detected corners
- `*_charuco_undistorted.jpg`: Undistorted image with reprojected points

These images help verify the calibration quality and corner detection accuracy.