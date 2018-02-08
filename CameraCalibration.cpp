#include <iostream>
#include "ImagePreprocessing.h"
#include "PatternSearch.h"

using namespace cv;
using namespace std;

int main( int argc, char** argv ) {
    Mat frame;
    int wait_key = 60;
    int keep_per_frames = 2;
    //VideoCapture cap("calibration_videos/calibration_mslifecam.avi");
    VideoCapture cap("calibration_videos/calibration_ps3eyecam.avi");
    //VideoCapture cap("/home/alonzo/Documentos/Projects/CameraCalibration_2/video/calibration_mslifecam.avi");
    //VideoCapture cap("/home/alonzo/Documentos/Projects/CameraCalibration_2/video/calibration_kinectv2.avi");
    //VideoCapture cap("/home/alonzo/Documentos/Projects/CameraCalibration_2/video/calibration_ps3eyecam.avi");
    //VideoCapture cap("/home/alonzo/Documentos/Projects/CameraCalibration_2/video/calibration_realsense.avi");
    //VideoCapture cap("/home/alonzo/Documentos/Projects/CameraCalibration_2/video/calibration_ps3eyecam.avi");

    if ( !cap.isOpened() ) {
        cout << "Cannot open the video file. \n";
        return -1;
    }

    cap.read(frame);
    double fps = cap.get(CV_CAP_PROP_FPS);
    int w = frame.rows;
    int h = frame.cols;
    namedWindow("Result",WINDOW_NORMAL);
    resizeWindow("Result", 640,640);
    moveWindow("Result", 0,0);
    vector<Point2f> pattern_points;
    while (1) {
        if (!cap.read(frame)) {
            cout << "\n Cannot read the video file. \n";
            break;
        }

        //equalizar_histograma(frame, w, h);
        //rgb_to_gray(frame, w, h);
        //segmentar(frame, frame, w, h);
        Mat edges = CannyEdgeDetector(frame);

        int erosion_size = 1;
        Mat kernel = getStructuringElement( MORPH_ELLIPSE,
                                            Size( 2 * erosion_size + 1, 2 * erosion_size + 1 ),
                                            Point( erosion_size, erosion_size ) );
        //Mat edges;
        //cvtColor( frame, edges, CV_BGR2GRAY );
        //blur( edges, edges, Size(3, 3) );
        morphologyEx(edges, edges, MORPH_CLOSE, kernel);
        find_points(edges, frame,pattern_points,keep_per_frames);
        imshow( "Result", frame );

        char t = (char)waitKey(wait_key);
        if ( t == 27)
            break;
        if (t == ' ') {
            if (wait_key == 0) {
                wait_key = 1000;
            } else {
                wait_key = 0;
            }
        }
    }
    return 0;
}