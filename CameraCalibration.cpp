#include <iostream>
#include "ImagePreprocessing.h"
#include "PatternSearch.h"

using namespace cv;
using namespace std;

#define THRESHOLD 30

int main( int argc, char** argv ) {
    Mat src, src_gray;
    //VideoCapture cap("calibration_videos/calibration_realsense.avi");
    //VideoCapture cap("calibration_videos/calibration_ps3eyecam.avi");
    VideoCapture cap("calibration_videos/calibration_mslifecam.avi");
    //VideoCapture cap("calibration_videos/calibration_kinectv2.avi");
    //VideoCapture cap("calibration_videos/PadronAnillos_02.avi");
    if (!cap.isOpened()) {
        cout << "Cannot open the video file. \n";
        return -1;
    }
    cap.read(src);
    double fps = cap.get(CV_CAP_PROP_FPS);
    int w = src.rows;
    int h = src.cols;
    while (1) {
        if (!cap.read(src)){
            cout << "\n Cannot read the video file. \n";
            break;
        }

        equalizar_histograma(src, w, h);
        //clean_other_colors(src, w, h);
        rgb_to_gray(src, w, h);
        segmentar(src, src, w, h);

        /// Convert image to gray and blur it
        cvtColor( src, src_gray, CV_BGR2GRAY );
        blur( src_gray, src_gray, Size(3, 3) );
        // Find pattern
        find_pattern(src_gray, THRESHOLD);
        
        imshow( "source_window", src );

        int c = waitKey(20);
        if ( (char)c == 27 ) {
            break;
        }
    }
}