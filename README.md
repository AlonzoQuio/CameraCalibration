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
```
Realtime test using PS3 camera
```
[![Realtime test using PS3 camera](https://img.youtube.com/vi/fiKpNXRzBqk/0.jpg)](http://www.youtube.com/watch?v=fiKpNXRzBqk)

```
Realtime test using MS Life cam camera
```

[![Realtime test using MS Life cam camera](https://img.youtube.com/vi/Mj2aXRs_yH8/0.jpg)](http://www.youtube.com/watch?v=Mj2aXRs_yH8)


```
Calibration process using the PS3 camera
```
[![Realtime test using MS Life cam camera](https://img.youtube.com/vi/SeETo6rh8Dc/0.jpg)](http://www.youtube.com/watch?v=SeETo6rh8Dc)

```
Pattern detection over undistort video PS3
```
[![Realtime test using MS Life cam camera](https://img.youtube.com/vi/kJqUyMwAxqw/0.jpg)](http://www.youtube.com/watch?v=kJqUyMwAxqw)

```
Calibration process using the Lifecam camera
```
[![Realtime test using MS Life cam camera](https://img.youtube.com/vi/3aO17O0WskU/0.jpg)](http://www.youtube.com/watch?v=3aO17O0WskU)

```
Pattern detection over undistort video Lifecam
```
[![Realtime test using MS Life cam camera](https://img.youtube.com/vi/G8c-f89xj18/0.jpg)](http://www.youtube.com/watch?v=G8c-f89xj18)

## Authors

* **Raúl Romaní** - *Master student at UCSP*
* **Alonzo Quio** - *Master student at UCSP*

