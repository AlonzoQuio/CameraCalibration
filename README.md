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

**Calibration process using the PS3 and LifeCam camera**
<iframe src="https://drive.google.com/file/d/1-6Stt6fTLUBsY4v49cVZLxBB-sPrC_EZ/preview" width="320" height="240"></iframe>
<iframe src="https://drive.google.com/file/d/1bPcLLcuMQbmUxh6Cvmk4N40Jo71PfTUT/preview" width="320" height="240"></iframe>

**Pattern detection over undistort video of PS3 and LifeCam**
<iframe src="https://drive.google.com/file/d/1zCcGjmSh6GJETwaMNEnVtpMD_gMn_QfL/preview" width="320" height="240"></iframe>
<iframe src="https://drive.google.com/file/d/1bcuVyZn0pAIsiBMWQGvWYgB4nIPh-vdR/preview" width="320" height="240"></iframe>
## Authors

* **Raúl Romaní** - *Master student at UCSP*
* **Alonzo Quio** - *Master student at UCSP*
