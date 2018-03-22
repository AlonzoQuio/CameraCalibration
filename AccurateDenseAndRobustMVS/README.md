# Accurate, Dense, and Robust Multi-View Stereopsis

Work in progress for camera calibration using adaptative segmentation and ellipse fitting, bassed on:
- Ankur Datta, Jun-Sik Kim, Takeo Kanade, “Accurate Camera Calibration using Iterative Refinement of Control Points”, 2009.
- CD Prakash, LJ Karam,“Camera calibration using adaptive segmentation and ellipse fitting for localizing control points”, Image Processing (ICIP), 2012.
- M. Vo, Z. Wang, L. Luu, and J. Ma, “Advanced geometric camera calibration for machine vision” ,Optical Engineering, Vol. 50, No. 11, 110503, 2011.

## Getting Started
The original source code can be downloaded from https://www.di.ens.fr/pmvs/
If you use the original code you should change some files, the process is described in the next link
https://github.com/mapillary/OpenSfM/blob/master/bin/export_pmvs.md

The source code provided in this github has those changes.
To compile the code you should run the make command in the source_code/main/ folder

### Prerequisites

First you need to install some dependencies using the next command

```
sudo apt-get install libgtk2.0-dev libglew1.6-dev libglew1.6
libdevil-dev libboost-all-dev libatlas-cpp-0.6-dev libatlas-
dev imagemagick libatlas3gf-base libcminpack-dev libg-
fortran3 libmetis-edf-dev libparmetis-dev freeglut3-dev
libgsl0-dev libblas-dev liblapack-dev liblapacke-dev libjpeg-dev
```

## Execution
And example of execution is 
```
pmvs "dir/to/option/" option.txt
```
In the dir/to/option/ folder you should have the folders txt,models and visualize to see more detail about every file needed you can see this link 
www.di.ens.fr/pmvs/documentation.html

## Authors

* **Raúl Romaní** - *Master student at UCSP*
* **Alonzo Quio** - *Master student at UCSP*
