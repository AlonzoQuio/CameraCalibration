#include <iostream>
#include "ImagePreprocessing.h"
#include "PatternSearch.h"

using namespace cv;
using namespace std;

#define LIFE_CAM "/home/alonzo/Documentos/Projects/CameraCalibration_2/video/calibration_mslifecam.avi"
#define KINECT_V2 "/home/alonzo/Documentos/Projects/CameraCalibration_2/video/calibration_kinectv2.avi"
#define PS3_EYE_CAM "/home/alonzo/Documentos/Projects/CameraCalibration_2/video/calibration_ps3eyecam.avi"
#define REAL_SENSE "/home/alonzo/Documentos/Projects/CameraCalibration_2/video/calibration_realsense.avi"

int main( int argc, char** argv ) {
    //long double execTime, prevCount, time;
    //execTime = prevCount = time = 0;
    Mat original, frame, frame_gray, masked;
    int wait_key = 10;
    int keep_per_frames = 2;
    Point mask_points[1][4];
    int n_frame = 1;
    int detected_points = 0;
    int success_frames = 0;
    Mat img;
    vector<PatterPoint> pattern_points;
    
    //VideoCapture cap(0);
    //VideoCapture cap(LIFE_CAM);
    VideoCapture cap(KINECT_V2);
    //VideoCapture cap(PS3_EYE_CAM);
    //VideoCapture cap(REAL_SENSE);

    if ( !cap.isOpened() ) {
        cout << "Cannot open the video file. \n";
        return -1;
    }

    cap.read(frame);
    double fps = cap.get(CV_CAP_PROP_FPS);
    int w = frame.rows;
    int h = frame.cols;
    mask_points[0][0]  = Point(0, 0);
    mask_points[0][1]  = Point(h, 0);
    mask_points[0][2]  = Point(h, w);
    mask_points[0][3]  = Point(0, w);
    string window_name = "Original";
    namedWindow(window_name, WINDOW_NORMAL);
    resizeWindow(window_name, 640, 480);
    moveWindow(window_name, 0, 0);

    window_name = "Masked";
    namedWindow(window_name, WINDOW_NORMAL);
    resizeWindow(window_name, 640, 480);
    moveWindow(window_name, 640, 0);

    window_name = "Threshold";
    namedWindow(window_name, WINDOW_NORMAL);
    resizeWindow(window_name, 640, 480);
    moveWindow(window_name, 640 * 2, 0);

    window_name = "Contours";
    namedWindow(window_name, WINDOW_NORMAL);
    resizeWindow(window_name, 640, 480);
    moveWindow(window_name, 0, 540);

    window_name = "Elipses";
    namedWindow(window_name, WINDOW_NORMAL);
    resizeWindow(window_name, 640, 480);
    moveWindow(window_name, 640, 540);

    window_name = "Result";
    namedWindow(window_name, WINDOW_NORMAL);
    resizeWindow(window_name, 640, 480);
    moveWindow(window_name, 640 * 2, 540);

    while (1) {

        if (!cap.read(frame)) {
            cout << "\n Cannot read the video file. \n";
            break;
        }
        //prevCount = getTickCount() * 1.0000;
        original = frame.clone();
        imshow("Original", original);

        //cv::GaussianBlur(frame, img, cv::Size(0, 0), 3);
        img = frame.clone();
        cv::addWeighted(frame, 1.5, img, -0.5, 0, img);
        frame = img;

        //equalizar_histograma(frame, w, h);
        //frame = equalizeIntensity(frame);
        //imshow("Equalized", frame);

        clean_using_mask(frame, w, h, mask_points);
        imshow("Masked", frame);
        masked = frame.clone();


        Mat thresh;

        cvtColor( frame, frame_gray, CV_BGR2GRAY );        
        
        //adaptiveThreshold(frame_gray, thresh, 125, ADAPTIVE_THRESH_MEAN_C, THRESH_BINARY, 41, 12);
        adaptiveThreshold(frame_gray, thresh, 255, ADAPTIVE_THRESH_GAUSSIAN_C, THRESH_BINARY, 41, 12);

        segmentar_bn(frame_gray, frame_gray, thresh, w, h);
        imshow("Threshold", frame_gray);

        detected_points = find_points(frame_gray, masked, original, w, h, mask_points, pattern_points, keep_per_frames);

        imshow("Elipses", masked);

        imshow("Contours", frame_gray);

        //if (n_frame == 264) {
        //    wait_key = 0;
        //}
        if (detected_points == 20) {
            success_frames ++;
        }
        std::ostringstream str;
        str << "Success rate " << success_frames << "/" << n_frame++;
        cout << str.str() << endl;
        //putText(original, str.str(), cvPoint(30,30), FONT_HERSHEY_COMPLEX_SMALL, 2, cvScalar(0,255,0), 1, CV_AA);

        imshow("Result", original );
        
        //time += execTime;
        //cout << "execTime = " << execTime << "; time = " << time << endl;
        //execTime = (getTickCount()*1.0000 - prevCount) / (getTickFrequency() * 1.0000);
        
        char t = (char)waitKey(wait_key);
        if ( t == 27)
            break;
        if (t == ' ') {
            if (wait_key == 0) {
                wait_key = 0;
            } else {
                wait_key = 0;
            }
        }
    }
    return 0;
}