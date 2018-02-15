# Camera Calibration
Work in progress for camera calibration using adaptative segmentation and ellipse fitting, bassed on:
- Ankur Datta, Jun-Sik Kim, Takeo Kanade, “Accurate Camera Calibration using Iterative Refinement of Control Points”, 2009.
- CD Prakash, LJ Karam,“Camera calibration using adaptive segmentation and ellipse fitting for localizing control points”, Image Processing (ICIP), 2012.
- M. Vo, Z. Wang, L. Luu, and J. Ma, “Advanced geometric camera calibration for machine vision” ,Optical Engineering, Vol. 50, No. 11, 110503, 2011.

## Getting Started

To compile the code you can write in a terminal:

```
g++ CameraCalibration.cpp -o CameraCalibration -O3 `pkg-config opencv --cflags --libs` && ./CameraCalibration
```

### Prerequisites

You need to have opencv intalled on your system, it can be achived using the follow command

```
sudo apt-get install libopencv-dev
```

## Tests
[![Tests](https://img.youtube.com/vi/gK19Vt7svGA/0.jpg)](http://www.youtube.com/watch?v=gK19Vt7svGA)
## Authors

* **Raúl Romaní** - *Master student at UCSP*
* **Alonzo Quio** - *Master student at UCSP*

