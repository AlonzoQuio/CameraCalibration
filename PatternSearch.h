#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

using namespace cv;
using namespace std;

void find_pattern(Mat &src_gray,int thresh) {
    Mat threshold_output;
    vector<vector<Point> > contours;
    vector<Vec4i> hierarchy;
    Point2f vertices[4];
    Point2f center, center_2;

    /// Detect edges using Threshold
    threshold( src_gray, threshold_output, thresh, 255, THRESH_BINARY );
    /// Find contours
    findContours( threshold_output, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );

    /// Find the rotated rectangles and ellipses for each contour
    vector<RotatedRect> minRect( contours.size() );
    vector<RotatedRect> minEllipse( contours.size() );

    for ( int i = 0; i < contours.size(); i++ ) {
        minRect[i] = minAreaRect( Mat(contours[i]) );
        if ( contours[i].size() > 5 ) {
            minEllipse[i] = fitEllipse( Mat(contours[i]) );
        }
    }

    /// Draw contours + rotated rects + ellipses
    Mat drawing = Mat::zeros( threshold_output.size(), CV_8UC3 );
    //cout << "nro elipses: " << contours.size() << endl;
    std::vector<int> pattern(contours.size());
    float range = 5;
    for ( int i = 0; i < contours.size(); i++ ) {
        for ( int j = 0; j < contours.size(); j++ ) {
            center = minEllipse[i].center;
            center_2 = minEllipse[j].center;
            if ( cv::norm(center - center_2) < range) {
                pattern[i] += 1;
            }
        }
        //Scalar color = Scalar( rng.uniform(0, 255), rng.uniform(0, 255), rng.uniform(0, 255) );
        // contour
        //drawContours( drawing, contours, i, color, 1, 8, vector<Vec4i>(), 0, Point() );
        // ellipse

        //minEllipse[i].points(vertices);
        //center = minEllipse[i].center;
        //// bottomLeft, topLeft, topRight, bottomRight.
        //double res_1 = cv::norm(vertices[0] - vertices[1]);
        //double res_2 = cv::norm(vertices[0] - vertices[3]);
        //double ratio = res_1 / res_2;
        //if (ratio < 1.5 && ratio > 0.5) {
        //    //ellipse( drawing, minEllipse[i], Scalar(0, 255, 0), 2, 8 );
        //    circle( drawing, center, 5, Scalar(0, 255, 0));
        //} else {
        //    ellipse( drawing, minEllipse[i], Scalar(255, 0, 0), 2, 8 );
        //}
        // rotated rectangle
        //Point2f rect_points[4]; minRect[i].points( rect_points );
        //for ( int j = 0; j < 4; j++ ) {
        //    line( drawing, rect_points[j], rect_points[(j + 1) % 4], color, 1, 8 );
        //}
    }
    for ( int i = 0; i < contours.size(); i++ ) {
        if (pattern[i] >= 2 && pattern[i] <= 4) {
            //cout << "Circles " << pattern[i] << endl;
            center = minEllipse[i].center;
            circle( drawing, center, 5, Scalar(0, 255, 0));
        }
    }
    /// Show in a window
    namedWindow( "Contours", CV_WINDOW_AUTOSIZE );
    // cv::resize(drawing, drawing,Size(300,300));
    imshow( "Contours", drawing );
}