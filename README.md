# Camera Calibration
Work in progress for camera calibration using adaptative segmentation and ellipse fitting, bassed on:
- Ankur Datta, Jun-Sik Kim, Takeo Kanade, “Accurate Camera Calibration using Iterative Refinement of Control Points”, 2009.
- CD Prakash, LJ Karam,“Camera calibration using adaptive segmentation and ellipse fitting for localizing control points”, Image Processing (ICIP), 2012.
- M. Vo, Z. Wang, L. Luu, and J. Ma, “Advanced geometric camera calibration for machine vision” ,Optical Engineering, Vol. 50, No. 11, 110503, 2011.

## Getting Started

To compile and run the code you can write in a terminal:

```
g++ CameraCalibration.cpp -o CameraCalibration -O3 `pkg-config opencv --cflags --libs` && ./CameraCalibration
```

### Prerequisites

You need to have opencv intalled on your system, it can be achived using the follow command

```
sudo apt-get install libopencv-dev
```

## Resources
- [Document](https://github.com/AlonzoQuio/CameraCalibration/blob/master/presentation/5_paper_CameraCalibration.pdf)
- [Slides](https://github.com/AlonzoQuio/CameraCalibration/blob/master/presentation/5_ppt_CameraCalibrationt.pdf)

## Camera calibration tests
### Rings pattern
**Calibration process using the PS3 and LifeCam camera**
<iframe src="https://drive.google.com/file/d/1-6Stt6fTLUBsY4v49cVZLxBB-sPrC_EZ/preview" width="320" height="240"></iframe>
<iframe src="https://drive.google.com/file/d/1bPcLLcuMQbmUxh6Cvmk4N40Jo71PfTUT/preview" width="320" height="240"></iframe>

**Pattern detection over undistort video of PS3 and LifeCam**
<iframe src="https://drive.google.com/file/d/1zCcGjmSh6GJETwaMNEnVtpMD_gMn_QfL/preview" width="320" height="240"></iframe>
<iframe src="https://drive.google.com/file/d/1bcuVyZn0pAIsiBMWQGvWYgB4nIPh-vdR/preview" width="320" height="240"></iframe>

### Asymmetric circles pattern using PS3 and LifeCam
<iframe src="https://drive.google.com/file/d/1KABRFlOmHyIXdY3bYYpu2HETIsErP-Dt/preview" width="320" height="240"></iframe>
<iframe src="https://drive.google.com/file/d/1CZt2dUcotLgbI58_ho13pI4oI6zNAB18/preview" width="320" height="240"></iframe>

### Chessboard pattern using PS3 and LifeCam
<iframe src="https://drive.google.com/file/d/1xuabf458WudQq2dFGHUNBDP8WD4lJSZs/preview" width="320" height="240"></iframe>
<iframe src="https://drive.google.com/file/d/13lU3olByIeWh2gVtupb2zufPZ2jfqeA7/preview" width="320" height="240"></iframe>

## Camera calibration iterative method
**Calibration process using the PS3**
<iframe src="https://drive.google.com/file/d/10mnysKsGLTYiP88S8Z-xScvVJ4TITnI1/preview" width="640" height="480"></iframe>

**Calibration process using the LifeCam camera**
<iframe src="https://drive.google.com/file/d/18J57fr_Dt0a-BeHqt0uQBm2tBMJAr8qh/preview" width="640" height="480"></iframe>

## Opengl Experiments
<iframe src="https://drive.google.com/file/d/1fbGKs7XkARdnUHcDwx6nhErDWmSSiRU4/preview" width="640" height="480"></iframe>

## Authors

* **Raúl Romaní** - *Master student at UCSP*
* **Alonzo Quio** - *Master student at UCSP*
* **Jose Jaita**  - *Master student at UCSP*
