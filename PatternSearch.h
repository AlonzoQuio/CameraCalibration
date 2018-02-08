#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

using namespace cv;
using namespace std;
void clean_from_elipses(Mat &out, vector<Point2f> pattern_centers);
void find_points(Mat &src_gray, Mat&original,vector<Point2f> &pattern_points,int &keep_per_frames) {
    int thresh = 30;

    Mat threshold_output;
    vector<vector<Point> > contours;
    vector<Vec4i> hierarchy;

    /// Detect edges using Threshold
    threshold( src_gray, threshold_output, thresh, 255, THRESH_BINARY );
    /// Find contours
    findContours( threshold_output, contours, hierarchy, CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );

    /// Find the rotated rectangles and ellipses for each contour
    vector<cv::RotatedRect> minRect( contours.size() );
    vector<cv::RotatedRect> minEllipse( contours.size() );

    for ( int i = 0; i < contours.size(); i++ ) {
        minRect[i] = minAreaRect( Mat(contours[i]) );
        if ( contours[i].size() > 5 ){
            minEllipse[i] = fitEllipse( Mat(contours[i]) );
        }
    }

    vector<Point2f> concentric;
    float minRadius = 2.00f;
    int nroCircles = 2;
    int count = 0;
    for ( int i = 0; i < contours.size(); i++ ){
        //circle(original,minEllipse[i].center,10,Scalar(0,0,255));
        //Rect rect1 = minEllipse[i].boundingRect() ;
        Point2f center1 = minEllipse[i].center;//(rect1.br() + rect1.tl()) * 0.5;
        for ( int j = 0; j < contours.size(); j++ ) {
            if (i == j) continue;

            //Rect rect2 = minEllipse[j].boundingRect() ;
            Point2f center2 = minEllipse[j].center;//(rect2.br() + rect2.tl()) * 0.5;

            if (cv::norm(center1 - center2) <= minRadius){
                count++;
            }

            if (count == nroCircles) {
                concentric.push_back(center1);
                break;
            }
        }
        count = 0;
    }

    //cout << "size: " << concentric.size() << endl;
    //for (int j = 0; j < concentric.size(); ++j)
    //{
    // int thickness = -1;
    // int lineType = 8;
    // circle( original,
    //         concentric[j],
    //         10,
    //         Scalar( 0, 255, 255 ),
    //         thickness,
    //         lineType );
    //}
    /**************************************************/

    vector<vector<Point2f>> points;
    minRadius = 8.50f;
    while ( concentric.size() != 0 ){
        Point2f point = concentric[0];

        vector<Point2f> temp;
        temp.push_back(point);

        for (int j = 1; j < concentric.size(); ++j){
            if (cv::norm(point - concentric[j]) < minRadius){
                temp.push_back(concentric[j]);
            }
        }

        if (temp.size() >= 2 && temp.size() <= 5){
            points.push_back(temp);
        }

        for (int i = 0; i < temp.size(); i++){
            concentric.erase(  std::remove(concentric.begin(), concentric.end(), temp[i])  , concentric.end()  );
        }
    }
    // cout << "temp: " << temp.size() << endl;
    //cout << "nro de cluster: " << points.size() << endl;
    //cout << "concentric size: " << concentric.size() << endl;

    vector<Point2f> medianPoints;

    for (int i = 0; i < points.size(); ++i) {
        Point2f median(0.0f, 0.0f);
        for (int j = 0; j < points[i].size(); ++j)
            median += points[i][j];
        medianPoints.push_back( Point2f(median.x / points[i].size(), median.y / points[i].size()) );
    }

    if (points.size() == 20){
        pattern_points.clear();
        for (int i = 0; i < medianPoints.size(); ++i) {
            int thickness = -1;
            int lineType = 8;
            circle( original,
                    medianPoints[i],
                    3,
                    Scalar( 0, 255, 0 ),
                    thickness,
                    lineType );
            pattern_points.push_back(medianPoints[i]);
            keep_per_frames = 2;
        }
    }
    if(keep_per_frames-->0){
        clean_from_elipses(original, pattern_points);
    }
}
float angle_between_two_points(Point2f p1, Point2f p2) {
    float angle = atan2(p1.y - p2.y, p1.x - p2.x);
    return angle * 180 / 3.1416;
}
float distance_to_rect(Point2f p1, Point2f p2, Point2f x) {
    float result = abs((p2.y - p1.y) * x.x - (p2.x - p1.x) * x.y + p2.x * p1.y - p2.y * p1.x) / sqrt(pow(p2.y - p1.y, 2) + pow(p2.x - p1.x, 2));
    return result;
}
vector<Point2f> more_distante_points(vector<Point2f>points) {
    float distance = 0;
    double temp;
    int p1, p2;
    for (int i = 0; i < points.size(); i++) {
        for (int j = 0; j < points.size(); j++) {
            if (i != j) {
                temp = cv::norm(points[i] - points[j]);
                if (distance < temp) {
                    distance = temp;
                    p1 = i;
                    p2 = j;
                }

            }
        }
    }
    if (points[p1].x < points[p2].x) {
        distance = p1;
        p1 = p2;
        p2 = distance;
    }
    vector<Point2f> p;
    p.push_back(points[p1]);
    p.push_back(points[p2]);
    return p;
}
void clean_from_elipses(Mat &drawing, vector<Point2f> pattern_centers) {
    if(pattern_centers.size()==0){
        return;
    }
    vector<Scalar> color_palette(5);
    color_palette[0] = Scalar(255, 0, 255);
    color_palette[1] = Scalar(255, 0, 0);
    color_palette[2] = Scalar(0, 255, 0);
    color_palette[3] = Scalar(0,0 , 255);

    int coincidendes = 0;
    int centers = pattern_centers.size();
    float pattern_range = 5;
    vector<Point2f> temp;
    vector<Point2f> line_points;
    vector<Point2f> limit_points;
    int line_color = 0;
    for (int i = 0; i < centers; i++) {
        for (int j = 0; j < centers; j++) {
            if (i != j) {
                temp.clear();
                line_points.clear();
                coincidendes = 0;
                for (int k = 0; k < centers; k++) {
                    if (distance_to_rect(pattern_centers[i], pattern_centers[j], pattern_centers[k]) < pattern_range) {
                        coincidendes++;
                        line_points.push_back(pattern_centers[k]);
                    } else {
                        //temp.push_back(pattern_centers[k]);
                    }
                }

                if (coincidendes == 5) {
                    line_points = more_distante_points(line_points);
                    bool found = false;
                    for (int l = 0; l < limit_points.size(); l++) {
                        if (limit_points[l].x == line_points[0].x && limit_points[l].y == line_points[0].y) {
                            found = true;
                        }
                    }
                    if (!found) {
                        limit_points.push_back(line_points[0]);
                        limit_points.push_back(line_points[1]);
                        if (line_color != 0) {
                            line(drawing, line_points[1], limit_points[line_color * 2 - 2], color_palette[line_color],2);
                        }
                        line(drawing, line_points[0], line_points[1], color_palette[line_color],2);
                        line_color++;
                    }
                }
            }
        }
    }
}