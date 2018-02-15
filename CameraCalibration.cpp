#include <iostream>
#include "ImagePreprocessing.h"
#include "PatternSearch.h"

using namespace cv;
using namespace std;

//#define LIFE_CAM "/home/alonzo/Documentos/Projects/CameraCalibration_2/video/calibration_mslifecam.avi"
//#define KINECT_V2 "/home/alonzo/Documentos/Projects/CameraCalibration_2/video/calibration_kinectv2.avi"
#define KINECT_V2 "/home/alonzo/Documentos/Projects/CameraCalibration_2/video/calibration_kinectv2_litte.mp4"
//#define PS3_EYE_CAM "/home/alonzo/Documentos/Projects/CameraCalibration_2/video/calibration_ps3eyecam.avi"
#define REAL_SENSE "/home/alonzo/Documentos/Projects/CameraCalibration_2/video/calibration_realsense.avi"

#define LIFE_CAM "calibration_videos/calibration_mslifecam.avi"
#define PS3_EYE_CAM "calibration_videos/calibration_ps3eyecam.avi"

int main( int argc, char** argv ) {
    long double execTime, prevCount, time;
    execTime = prevCount = time = 0;
    Mat original, frame, frame_gray, masked;
    Mat m_success_rate, m_fps;
    int wait_key = 1;
    int original_wait_key = wait_key;
    int keep_per_frames = 2;
    Point mask_points[1][4];
    int n_frame = 1;
    int detected_points = 0;
    int success_frames = 0;
    Mat img;
    int total_frames;
    vector<PatterPoint> pattern_points;

    /* For video from webcam */
    //VideoCapture cap(0);
    //VideoCapture cap(LIFE_CAM);
    //VideoCapture cap(KINECT_V2);
    VideoCapture cap(PS3_EYE_CAM);
    //VideoCapture cap(REAL_SENSE);

    if ( !cap.isOpened() ) {
        cout << "Cannot open the video file. \n";
        return -1;
    }

    cap.read(frame);
    total_frames= cap.get(CAP_PROP_FRAME_COUNT);
    double fps = cap.get(CV_CAP_PROP_FPS);
    int w = frame.rows;
    int h = frame.cols;
    mask_points[0][0]  = Point(0, 0);
    mask_points[0][1]  = Point(h, 0);
    mask_points[0][2]  = Point(h, w);
    mask_points[0][3]  = Point(0, w);
    int window_w = 360 * 1.25;
    int window_h = 240 * 1.25;
    //int window_w = 640;
    //int window_h = 480;
    int second_screen_offste = 0;

    /* WINDOW SETUP */
    string window_name = "Original";
    namedWindow(window_name, WINDOW_NORMAL);
    resizeWindow(window_name, window_w, window_h);
    moveWindow(window_name, 0 + second_screen_offste, 0);

    window_name = "Masked";
    namedWindow(window_name, WINDOW_NORMAL);
    resizeWindow(window_name, window_w, window_h);
    moveWindow(window_name, window_w + second_screen_offste, 0);

    window_name = "Threshold";
    namedWindow(window_name, WINDOW_NORMAL);
    resizeWindow(window_name, window_w, window_h);
    moveWindow(window_name, window_w * 2 + second_screen_offste, 0);

    window_name = "Contours";
    namedWindow(window_name, WINDOW_NORMAL);
    resizeWindow(window_name, window_w, window_h);
    moveWindow(window_name, 0 + second_screen_offste, window_h + 40);

    window_name = "Elipses";
    namedWindow(window_name, WINDOW_NORMAL);
    resizeWindow(window_name, window_w, window_h);
    moveWindow(window_name, window_w + second_screen_offste, window_h + 40);

    window_name = "Result";
    namedWindow(window_name, WINDOW_NORMAL);
    resizeWindow(window_name, window_w, window_h);
    moveWindow(window_name, window_w * 2 + second_screen_offste, window_h + 40);

    /*window_name = "Rate";
    namedWindow(window_name, WINDOW_NORMAL);
    resizeWindow(window_name, window_w*3, 10);
    moveWindow(window_name, 0 + second_screen_offste, window_h*2 + 40);*/

    while (1) {

        prevCount = getTickCount() * 1.0000;
        if (!cap.read(frame)) {
            cout << "\n Cannot read the video file. \n";
            break;
        }
        original = frame.clone();
        imshow("Original", original);

        clean_using_mask(frame, w, h, mask_points);
        imshow("Masked", frame);
        masked = frame.clone();


        Mat thresh;
        cvtColor( frame, frame_gray, CV_BGR2GRAY );
        //adaptiveThreshold(frame_gray, thresh, 125, ADAPTIVE_THRESH_MEAN_C, THRESH_BINARY, 41, 12);
        adaptiveThreshold(frame_gray, thresh, 255, ADAPTIVE_THRESH_GAUSSIAN_C, THRESH_BINARY, 41, 12);

        segmentar(frame_gray, frame_gray, thresh, w, h);
        imshow("Threshold", frame_gray);

        detected_points = find_pattern_points(frame_gray, masked, original, w, h, mask_points, pattern_points, keep_per_frames);

        imshow("Contours", frame_gray);
        imshow("Elipses", masked);

        //if (n_frame == 264) {
        //    wait_key = 0;
        //}
        time += execTime;

        if (detected_points == 20) {
            success_frames ++;
        }

        imshow("Result", original );

        n_frame++;
        std::ostringstream fps, success_rate;
        fps << "FPS: " << 1 / execTime ;
        success_rate << "Success rate: " << success_frames << " / " << n_frame <<  " = " << success_frames * 100.0 / n_frame  << "%";
        m_success_rate = Mat::zeros(Size(window_w * 3, 40), CV_8UC3);
        putText(m_success_rate, fps.str(), cvPoint(window_w * 3 - 250, 30), FONT_HERSHEY_PLAIN, 2, cvScalar(0, 255, 0), 1, CV_AA);
        putText(m_success_rate, success_rate.str(), cvPoint(10, 30), FONT_HERSHEY_PLAIN, 2, cvScalar(0, 255, 0), 1, CV_AA);
        imshow("Rate", m_success_rate);

        execTime = (getTickCount() * 1.0000 - prevCount) / (getTickFrequency() * 1.0000);
        if(total_frames==n_frame){
            cout << success_rate.str() << endl;
            cout << fps.str() << endl;
        }
        char t = (char)waitKey(wait_key);
        if ( t == 27)
            break;
        if (t == ' ') {
            if (wait_key == 0) {
                wait_key = original_wait_key;
            } else {
                wait_key = 0;
            }
        }
    }
    return 0;
}