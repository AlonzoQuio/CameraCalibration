#include <iostream>
#include <iomanip>
#include "ImagePreprocessing.h"
#include "PatternSearch.h"
#include "CalibrateCamera.h"

using namespace cv;
using namespace std;

#define CALIBRATION_PS3_OWN_VIDEO "/home/alonzo/Documentos/Projects/CameraCalibration/calibration_videos/ellipses.avi"
#define CALIBRATION_LIFECAM_VIDEO "/home/alonzo/Downloads/LifeCam_rings.avi"
//#define CALIBRATION_PS3_VIDEO     "/home/alonzo/Downloads/PS3_rings.avi"
#define CALIBRATION_PS3_VIDEO     "calibration_videos/PS3_Rings.webm"

int main( int argc, char** argv ) {
    long double execTime, prevCount, time;
    execTime = prevCount = time = 0;
    Mat original, frame, frame_gray, masked;
    Mat m_success_rate;
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
    int total_frames;
    float rms = -1;
    float distance_prom = 0.0;
    vector<PatternPoint> pattern_points;
    vector<vector<Point2f>> set_points;
    int num_color_palette = 100;
    vector<Scalar> color_palette(num_color_palette);
    RNG rng(12345);
    for (int i = 0; i < num_color_palette; i++) {
        color_palette[i] = Scalar(rng.uniform(0, 255), rng.uniform(0, 255), rng.uniform(0, 255));
    }

    //VideoCapture cap(0);

    //cap.set(cv::CAP_PROP_FPS,60);
    //int fps = cap.get(cv::CAP_PROP_FPS);
    //cout<<fps<<endl;
    
    //VideoCapture cap(CALIBRATION_LIFECAM_VIDEO);
    VideoCapture cap(CALIBRATION_PS3_VIDEO);
    //VideoCapture cap(CALIBRATION_PS3_OWN_VIDEO);

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
    int window_w = 360 * 1.25;
    int window_h = 240 * 1.25;
    //int window_w = 640;
    //int window_h = 480;
    int second_screen_offste = 0;//1360;
    string window_name;
    int stop_at_frame = -1; //1200 for ps3 600 for lifecam

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

    while (1) {
        std::ostringstream fps, success_rate, rms_str;
        m_success_rate = Mat::zeros(Size(window_w * 3, 40), CV_8UC3);

        //cap.set(1, n_frame);
        prevCount = getTickCount() * 1.0000;
        if (!cap.read(frame)) {
            cout << "\n Cannot read the video file. \n";
            break;
        }
        
        /*
        // PS3 Calib parameters
        cameraMatrix = (Mat_<double>(3, 3) << 842.277648121042, 0, 306.8028364233875,
                                              0, 845.9871738025694, 258.6145767605929,
                                              0, 0, 1);

        distCoeffs = (Mat_<double>(1, 5) << -0.3663343352388944,
                                             0.2236100121702693,
                                            -0.001952345044331583,
                                             0.004004688327188062,
                                            -0.2475043367750629);
        rms = 0.134671;
        */
        /*
        // LifeCam Calib parameters
        cameraMatrix = (Mat_<double>(3, 3) << 618.3848171468343, 0, 341.9353917933382,
                                                0, 621.9343954517286, 231.4488417101894,
                                                0, 0, 1);

        distCoeffs = (Mat_<double>(1, 5) << -0.02287003323277602,
                                             0.09690594726549269,
                                             0.004358383321919785,
                                            -0.002456174770498996,
                                            -0.4132612693983291);
        rms = 0.136174;
        */
        /*
        // PS3 own video parameters
        cameraMatrix = (Mat_<double>(3, 3) << 619.8529149086295, 0, 317.8602231908566,
                        0, 623.7464457495411, 257.8088409771084,
                        0, 0, 1);

        distCoeffs = (Mat_<double>(1, 5) << -0.2940861347010496,
                      0.1480524184491786,
                      -0.003080858295605339,
                      0.005568077204929082,
                      -0.07530280722975796);
        rms = 0.15688;
        */
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
            imshow("Result", frame);
            frame = rview;
        }

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

        if (n_frame == stop_at_frame) {
            wait_key = 0;
        }

        time += execTime;

        if (detected_points == 20) {
            success_frames ++;
            distance_prom += avgColinearDistance(pattern_points);
        }
        if (rms == -1) {
            imshow("Result", original );
        } else {
            imshow("Undistort", original );
        }

        if (rms == -1) {
            if (n_frame % 10 == 0 && detected_points == 20) { // 60 20 for ps3 and 30 20 for lifecam
                vector<Point2f> temp(20);
                for (int i = 0; i < 20; i++) {
                    temp[i] = pattern_points[i].to_point2f();
                }
                set_points.push_back(temp);

                if (set_points.size() == 30) {
                    rms = calibrate_with_points(imageSize, cameraMatrix, distCoeffs, set_points);
                    cout << "cameraMatrix " << cameraMatrix << endl;
                    cout << "distCoeffs " << distCoeffs << endl;
                    //wait_key = 0;
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
        fps << std::fixed << std::setprecision(2) << execTime * 1000 << "ms" ;
        execTime = (getTickCount() * 1.0000 - prevCount) / (getTickFrequency() * 1.0000);
        success_rate << "S. Rate: " << success_frames << "/" << n_frame <<  " = " << std::fixed << std::setprecision(2) << success_frames * 100.0 / n_frame  << "%";
        putText(m_success_rate, success_rate.str(), cvPoint(10, 30), FONT_HERSHEY_PLAIN, 2, cvScalar(0, 255, 0), 1, CV_AA);
        putText(m_success_rate, fps.str(), cvPoint(window_w * 2.5, 30), FONT_HERSHEY_PLAIN, 2, cvScalar(0, 255, 0), 1, CV_AA);

        rms_str << "Rep. error: " << std::fixed << std::setprecision(2) << rms << " AVG: " << std::fixed << std::setprecision(4) << distance_prom / success_frames;
        putText(m_success_rate, rms_str.str(), cvPoint(window_w * 1.2, 30), FONT_HERSHEY_PLAIN, 2, cvScalar(0, 255, 0), 1, CV_AA);

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
        /*if (t == 'c' && detected_points == 20) {
            cout << "Frame " << n_frame << endl;
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
        }*/
        n_frame++;

        //if (t == 'a') {
        //    n_frame--;
        //}
        //if (t == 'd') {
        //    n_frame++; 
        //}
    }
    return 0;
}