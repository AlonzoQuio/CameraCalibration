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
    Mat original, frame, frame_gray, masked;
    int wait_key = 10;
    int keep_per_frames = 2;
    Point mask_points[1][4];
    int n_frame = 1;
    int detected_points = 0;
    int success_frames = 0;

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

    window_name = "Equalized";
    namedWindow(window_name, WINDOW_NORMAL);
    resizeWindow(window_name, 640, 480);
    moveWindow(window_name, 640, 0);

    window_name = "Masked";
    namedWindow(window_name, WINDOW_NORMAL);
    resizeWindow(window_name, 640, 480);
    moveWindow(window_name, 640 * 2, 0);

    window_name = "Threshold";
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
    Mat img;
    vector<Point2f> pattern_points;

    while (1) {
        if (!cap.read(frame)) {
            cout << "\n Cannot read the video file. \n";
            break;
        }
        original = frame.clone();
        imshow("Original", original);

        //cv::GaussianBlur(frame, img, cv::Size(0, 0), 3);
        img = frame.clone();
        cv::addWeighted(frame, 1.5, img, -0.5, 0, img);
        frame = img;

        //equalizar_histograma(frame, w, h);
        frame = equalizeIntensity(frame);
        imshow("Equalized", frame);

        clean_using_mask(frame, w, h, mask_points);
        imshow("Masked", frame);
        masked = frame.clone();


        // imshow( "Equalized", frame );

        //clean_other_colors(src, w, h);
        rgb_to_gray(frame, w, h);

        //segmentar(frame, frame, w, h);
        Mat thresh;
        cvtColor( frame, frame_gray, CV_BGR2GRAY );
        adaptiveThreshold(frame_gray, thresh, 125, ADAPTIVE_THRESH_MEAN_C, THRESH_BINARY, 41, 12);
        //imshow("Threshold", frame);
        imshow("Threshold", thresh);
        frame_gray = thresh;

        //clean_using_mask(frame,w,h,mask_points);

        /// Convert image to gray and blur it
        //cvtColor( frame, frame_gray, CV_BGR2GRAY );
        //blur( src_gray, src_gray, Size(3, 3) );
        //GaussianBlur( frame_gray, frame_gray, Size( 3, 3 ), 0, 0 );

        detected_points = find_points(frame_gray, masked, original, w, h, mask_points, pattern_points, keep_per_frames);

        imshow("Elipses", masked);
        imshow("Result", original );

        //if (n_frame == 410) {
        //    wait_key = 0;
        //}
        if (detected_points == 20) {
            success_frames ++;
        }
        cout << "Success " <<  success_frames << "/" << n_frame++ << endl;

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