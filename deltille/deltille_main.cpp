#include <iostream>
#include <iomanip>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "CameraCalibration.h"

using namespace cv;
using namespace std;

#define CALIBRATION_LIFECAM_VIDEO     "calibration_videos/Triangles_Blue.mp4"

// void window_setup() {
//     int window_w = 640;
//     int window_h = 480;
//     string window_name;
//     window_name = "CentersDistribution";

//     namedWindow(window_name, WINDOW_NORMAL);
//     resizeWindow(window_name, window_w, window_h);
//     moveWindow(window_name, 0, 0);

//     window_name = "CalibrationFrames";
//     namedWindow(window_name, WINDOW_NORMAL);
//     resizeWindow(window_name, window_w, window_h);
//     moveWindow(window_name, window_w, 0);

//     window_name = "InputFrame+Grid";
//     namedWindow(window_name, WINDOW_NORMAL);
//     resizeWindow(window_name, window_w, window_h);
//     moveWindow(window_name, window_w * 2, 0);

//     window_name = "Undistort";
//     namedWindow(window_name, WINDOW_NORMAL);
//     resizeWindow(window_name, window_w, window_h);
//     moveWindow(window_name, 0, window_h);
// }

/**
 * @brief Initialize windows names, sizes and positions.
 */
void window_setup() {
    int window_w = 360 * 1.77;
    int window_h = 240 * 1.77;
    //int window_w = 640;
    //int window_h = 480;
    int second_screen_offset = 0;//1360;
    string window_name;
    window_name = "CentersDistribution";
    namedWindow(window_name, WINDOW_NORMAL);
    resizeWindow(window_name, window_w, window_h);
    moveWindow(window_name, 0 + second_screen_offset, 0);

    window_name = "CalibrationFrames";
    namedWindow(window_name, WINDOW_NORMAL);
    resizeWindow(window_name, window_w, window_h);
    moveWindow(window_name, window_w + second_screen_offset, 0);

    window_name = "Undistort";
    namedWindow(window_name, WINDOW_NORMAL);
    resizeWindow(window_name, window_w, window_h);
    moveWindow(window_name, window_w * 2 + second_screen_offset, 0);

    window_name = "FrontoParallel";
    namedWindow(window_name, WINDOW_NORMAL);
    resizeWindow(window_name, window_w, window_h);
    moveWindow(window_name, 0 + second_screen_offset, window_h + 60);

    window_name = "Reproject";
    namedWindow(window_name, WINDOW_NORMAL);
    resizeWindow(window_name, window_w, window_h);
    moveWindow(window_name, window_w + second_screen_offset, window_h + 60);

    window_name = "Distort";
    namedWindow(window_name, WINDOW_NORMAL);
    resizeWindow(window_name, window_w, window_h);
    moveWindow(window_name, window_w * 2 + second_screen_offset, window_h + 60);
}

int main( int argc, char** argv ) {
    VideoCapture cap(CALIBRATION_LIFECAM_VIDEO);
    if ( !cap.isOpened() ) {
        cout << "Cannot open the video file. \n";
        return -1;
    }

    // CameraCalibrationDeltille camera_calibration;
    // CameraCalibrationDeltille camera_calibration(cap, 8, 5);
    // camera_calibration.test1(); 
    
    window_setup();
    CameraCalibrationDeltille camera_calibration(cap, 8, 5);
    int n_frames  = 40;
    int grid_cols = 4;
    int grid_rows = 3;
    camera_calibration.calibrate_camera_iterative(10, n_frames, grid_rows, grid_cols);



    /*for (int f = 0; f < camera_calibration.frames.size(); f++) {
        camera_calibration.undistort_image(camera_calibration.frames[f]);
        imshow("undistort", camera_calibration.frames[f]);
        waitKey(1);
    }*/

    waitKey(0);
    return 0;
}