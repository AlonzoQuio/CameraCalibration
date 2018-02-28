#include <iostream>
#include "ImagePreprocessing.h"
#include "PatternSearch.h"
#include "CalibrateCamera.h"

using namespace cv;
using namespace std;

//#define LIFE_CAM "/home/alonzo/Documentos/Projects/CameraCalibration_2/video/calibration_mslifecam.avi"
//#define KINECT_V2 "/home/alonzo/Documentos/Projects/CameraCalibration_2/video/calibration_kinectv2.avi"
#define KINECT_V2 "/home/alonzo/Documentos/Projects/CameraCalibration_2/video/calibration_kinectv2_litte.mp4"
//#define PS3_EYE_CAM "/home/alonzo/Documentos/Projects/CameraCalibration_2/video/calibration_ps3eyecam.avi"
#define REAL_SENSE "/home/alonzo/Documentos/Projects/CameraCalibration_2/video/calibration_realsense.avi"

#define LIFE_CAM "calibration_videos/calibration_mslifecam.avi"
#define PS3_EYE_CAM "calibration_videos/calibration_ps3eyecam.avi"
#define CALIBRATION_VIDEO "/home/alonzo/Documentos/Projects/CameraCalibration/calibration_videos/ellipses.avi"
#define TEST_CALIBRATION_VIDEO "/home/alonzo/Documentos/Projects/CameraCalibration/out.avi"

int main( int argc, char** argv ) {
    long double execTime, prevCount, time;
    execTime = prevCount = time = 0;
    Mat original, frame, frame_gray, masked;
    Mat m_success_rate, m_fps;
    Mat m_calibration;
    Mat cameraMatrix;
    Mat distCoeffs;
    Mat thresh;

    int wait_key = 1;
    int original_wait_key = wait_key;
    int keep_per_frames = 2;
    Point mask_points[1][4];
    int n_frame = 1;
    int detected_points = 0;
    int success_frames = 0;
    Mat img;
    int total_frames;
    float rms = -1;
    vector<PatternPoint> pattern_points;
    vector<vector<Point2f>> set_points;
    int num_color_palette = 100;
    vector<Scalar> color_palette(num_color_palette);
    RNG rng(12345);
    for (int i = 0; i < num_color_palette; i++) {
        color_palette[i] = Scalar(rng.uniform(0, 255), rng.uniform(0, 255), rng.uniform(0, 255));
    }

    /* For video from webcam */
    //VideoCapture cap(0);

    //cap.set(cv::CAP_PROP_FPS,60);
    //int fps = cap.get(cv::CAP_PROP_FPS);
    //cout<<fps<<endl;

    //VideoCapture cap(LIFE_CAM);
    //VideoCapture cap(KINECT_V2);
    //VideoCapture cap(PS3_EYE_CAM);
    //VideoCapture cap(REAL_SENSE);
    VideoCapture cap(CALIBRATION_VIDEO);
    //VideoCapture cap(TEST_CALIBRATION_VIDEO);

    if ( !cap.isOpened() ) {
        cout << "Cannot open the video file. \n";
        return -1;
    }

    cap.read(frame);
    //total_frames = cap.get(CAP_PROP_FRAME_COUNT);
    int w = frame.rows;
    int h = frame.cols;
    Size imageSize(h, w);
    mask_points[0][0]  = Point(0, 0);
    mask_points[0][1]  = Point(h, 0);
    mask_points[0][2]  = Point(h, w);
    mask_points[0][3]  = Point(0, w);
    //int window_w = 360 * 1.25;
    //int window_h = 240 * 1.25;
    int window_w = 640;
    int window_h = 480;
    int second_screen_offste = 0;
    string window_name;

    /* WINDOW SETUP */
    //window_name = "Original";
    window_name = "Threshold";
    namedWindow(window_name, WINDOW_NORMAL);
    resizeWindow(window_name, window_w, window_h);
    moveWindow(window_name, 0 + second_screen_offste, 0);

    //window_name = "Masked";
    window_name = "Contours";
    namedWindow(window_name, WINDOW_NORMAL);
    resizeWindow(window_name, window_w, window_h);
    moveWindow(window_name, window_w + second_screen_offste, 0);

    //window_name = "Threshold";
    window_name = "Elipses";
    namedWindow(window_name, WINDOW_NORMAL);
    resizeWindow(window_name, window_w, window_h);
    moveWindow(window_name, window_w * 2 + second_screen_offste, 0);

    //window_name = "Contours";
    window_name = "Result";
    namedWindow(window_name, WINDOW_NORMAL);
    resizeWindow(window_name, window_w, window_h);
    moveWindow(window_name, 0 + second_screen_offste, window_h + 40);

    //window_name = "Elipses";
    window_name = "Calibration";
    namedWindow(window_name, WINDOW_NORMAL);
    resizeWindow(window_name, window_w, window_h);
    moveWindow(window_name, window_w + second_screen_offste, window_h + 40);

    //window_name = "Result";
    window_name = "Undistort";
    namedWindow(window_name, WINDOW_NORMAL);
    resizeWindow(window_name, window_w, window_h);
    moveWindow(window_name, window_w * 2 + second_screen_offste, window_h + 40);
    //resizeWindow(window_name, 960,640);
    //moveWindow(window_name, 0,0);

    while (1) {
        std::ostringstream fps, success_rate, rms_str;
        m_success_rate = Mat::zeros(Size(window_w * 3, 40), CV_8UC3);

        //cap.set(1, n_frame);
        prevCount = getTickCount() * 1.0000;
        if (!cap.read(frame)) {
            cout << "\n Cannot read the video file. \n";
            break;
        }
        /*// Use loaded parameters
        cameraMatrix = (Mat_<double>(3,3) << 638.35848013511, 0, 318.0732749667285,
                            0, 642.2840086097892, 254.4917790852329,
                            0, 0, 1);
        
        distCoeffs = (Mat_<double>(1,4) <<-0.305400774036524,
                          0.1616945505031097,
                          -0.002354577710289191,
                          0.004283624946433994,
                          -0.1375998250104978);
        rms = 0.1588;
        if (rms != -1) {
            Mat rview, map1, map2;
            initUndistortRectifyMap(cameraMatrix,
                                    distCoeffs,
                                    Mat(),
                                    getOptimalNewCameraMatrix(cameraMatrix, distCoeffs, imageSize, 1, imageSize, 0),
                                    imageSize,
                                    CV_16SC2,
                                    map1,
                                    map2);

            remap(frame, rview, map1, map2, INTER_LINEAR);
            //remap(original, rview, map1, map2, INTER_LINEAR);
            imshow("Undistort", rview);
            rms_str << "Reprojection error: " << rms;
            putText(m_success_rate, rms_str.str(), cvPoint(window_w * 1.5, 30), FONT_HERSHEY_PLAIN, 2, cvScalar(0, 255, 0), 1, CV_AA);
            frame = rview;
        }*/

        original = frame.clone();
        //imshow("Original", original);

        clean_using_mask(frame, w, h, mask_points);
        //imshow("Masked", frame);
        masked = frame.clone();

        cvtColor( frame, frame_gray, CV_BGR2GRAY );
        adaptiveThreshold(frame_gray, thresh, 255, ADAPTIVE_THRESH_GAUSSIAN_C, THRESH_BINARY, 41, 12);
        segmentar(frame_gray, frame_gray, thresh, w, h);
        imshow("Threshold", frame_gray);

        detected_points = find_pattern_points(frame_gray, masked, original, w, h, mask_points, pattern_points, keep_per_frames);

        imshow("Contours", frame_gray);
        imshow("Elipses", masked);

        if (n_frame == 315) {
            wait_key = 0;
        }
        time += execTime;

        if (detected_points == 20) {
            success_frames ++;
        }

        imshow("Result", original );

        n_frame++;

        if (rms != -1) {
            Mat rview, map1, map2;
            initUndistortRectifyMap(cameraMatrix,
                                    distCoeffs,
                                    Mat(),
                                    getOptimalNewCameraMatrix(cameraMatrix, distCoeffs, imageSize, 1, imageSize, 0),
                                    imageSize,
                                    CV_16SC2,
                                    map1,
                                    map2);

            remap(original, rview, map1, map2, INTER_LINEAR);
            imshow("Undistort", rview);
            rms_str << "Reprojection error: " << rms;
            putText(m_success_rate, rms_str.str(), cvPoint(window_w * 1.5, 30), FONT_HERSHEY_PLAIN, 2, cvScalar(0, 255, 0), 1, CV_AA);
        } else {
            if (n_frame % 15 == 0 && detected_points == 20) {
                vector<Point2f> temp(20);
                for (int i = 0; i < 20; i++) {
                    temp[i] = pattern_points[i].to_point2f();
                }
                set_points.push_back(temp);

                if (set_points.size() == 20) {
                    rms = calibrate_with_points(imageSize, cameraMatrix, distCoeffs, set_points);
                    cout << "cameraMatrix " << cameraMatrix << endl;
                    cout << "distCoeffs " << distCoeffs << endl;
                    wait_key = 0;
                }
            }
        }
        m_calibration = Mat::zeros(Size(h, w), CV_8UC3);
        for (int i = 0; i < set_points.size(); i++) {
            for (int j = 0; j < 20; j++) {
                circle(m_calibration, set_points[i][j], 10, color_palette[i]);
            }
        }
        imshow("Calibration", m_calibration);
        fps << execTime * 1000 << "ms" ;
        execTime = (getTickCount() * 1.0000 - prevCount) / (getTickFrequency() * 1.0000);
        success_rate << "Success rate: " << success_frames << " / " << n_frame <<  " = " << success_frames * 100.0 / n_frame ;// << "% with " << time/(n_frame-1) * 1000.0;
        putText(m_success_rate, fps.str(), cvPoint(window_w * 3 - 250, 30), FONT_HERSHEY_PLAIN, 2, cvScalar(0, 255, 0), 1, CV_AA);
        putText(m_success_rate, success_rate.str(), cvPoint(10, 30), FONT_HERSHEY_PLAIN, 2, cvScalar(0, 255, 0), 1, CV_AA);

        imshow("Rate", m_success_rate);

        //if (total_frames == n_frame) {
        //    cout << success_rate.str() << endl;
        //    cout << fps.str() << endl;
        //}
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
        //if (t == 'a') {
        //    n_frame--;
        //}
        //if (t == 'd') {
        //    n_frame++;
        //}
    }
    return 0;
}