# SLAM Point Cloud Reconstruction

This project implements a simplified SLAM (Simultaneous Localization and Mapping) system that reconstructs 3D point clouds from stereo image pairs using OpenCV.

## Overview

The system processes two consecutive images to:
1. Detect keypoints and descriptors
2. Match features between frames using a matcher
3. Estimate camera motion via Essential Matrix
4. Triangulate 3D points from correspondences
5. Export the resulting point cloud in PLY format

## Building

```bash
mkdir build
cd build
cmake ..
make
```

## Running

```bash
./slam data/IMG_1956.jpg data/IMG_1957.jpg cloud.ply
```

## Roadmap/TODO

1. Implement processing of a frame sequence into a single point cloud or a 3D model
2. More flexibility of choosing underlying algorithms
3. Camera intrinsics calibration