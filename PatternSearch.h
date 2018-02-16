#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <opencv2/calib3d/calib3d.hpp>

using namespace cv;
using namespace std;

/**
 * @details Class to save patter point information
 */
class PatterPoint {
public:
    float x;
    float y;
    float radio;
    int h_father;
    PatterPoint() {
        x = 0;
        y = 0;
        radio = 0;
    }
    PatterPoint(float x, float y, float radio, int h_father) {
        this->x = x;
        this->y = y;
        this->radio = radio;
        this->h_father = h_father;
    }
    float distance(PatterPoint p) {
        return sqrt(pow(x - p.x, 2) + pow(y - p.y, 2));
    }
    Point2f to_point2f() {
        return Point2f(x, y);
    }
    Point2f center() {
        return Point2f(x, y);
    }
};

void draw_lines_pattern_from_ellipses(Mat &out, vector<PatterPoint> &pattern_centers, vector<PatterPoint> new_pattern_points);
void update_mask_from_points(vector<PatterPoint> points, int w, int h, Point mask_point[][4]);
int mode_from_father(vector<PatterPoint> pattern_points);
int find_pattern_points(Mat &src_gray, Mat &masked, Mat&original, int w, int h, Point mask_point[][4], vector<PatterPoint> &pattern_points, int &keep_per_frames);
float angle_between_two_points(PatterPoint p1, PatterPoint p2);
float distance_to_rect(PatterPoint p1, PatterPoint p2, PatterPoint x);
vector<PatterPoint> more_distant_points(vector<PatterPoint>points);

/**
 * @details Function to support PatterPoint sort by hierarchy
 *
 * @param p1 First point
 * @param p2 Second point
 *
 * @return father hierarchy of point 1 is lower than father hierarchy of point 2
 */
bool sort_pattern_point_by_father(PatterPoint p1, PatterPoint p2) {
    return p1.h_father < p2.h_father;
}
bool sort_pattern_point_by_x(PatterPoint p1, PatterPoint p2) {
    return p1.x < p2.x;
}
bool sort_pattern_point_by_y(PatterPoint p1, PatterPoint p2) {
    return p1.y < p2.y;
}
/**
 * @details Return the father hierarchy mode from a vector of PatternPoints
 *
 * @param pattern_points Vector of points
 * @return father hierarchy mode
 */
int mode_from_father(vector<PatterPoint> points) {
    if (points.size() == 0) {
        return -1;
    }
    vector<PatterPoint> temp;
    for (int p = 0; p < points.size(); p++) {
        temp.push_back(points[p]);
    }
    sort(temp.begin(), temp.end(), sort_pattern_point_by_father);

    int number = temp[0].h_father;
    int mode = number;
    int count = 1;
    int countMode = 1;

    for (int p = 1; p < temp.size(); p++) {
        if (number == temp[p].h_father) {
            count++;
        } else {
            if (count > countMode) {
                countMode = count;
                mode = number;
            }
            count = 1;
            number =  temp[p].h_father;
        }
    }
    if (count > countMode) {
        return number;
    } else {
        return mode;
    }
}


int find_pattern_points(Mat &src_gray, Mat &masked, Mat&original, int w, int h, Point mask_point[][4], vector<PatterPoint> &pattern_points, int &keep_per_frames) {

    vector<vector<Point> > contours;
    vector<Vec4i> hierarchy;
    vector<PatterPoint> ellipses_temp;
    vector<PatterPoint> new_pattern_points;
    float radio_hijo;
    float radio;
    float radio_prom = 0;
    float distance;
    Scalar purple(255, 0, 255);
    Scalar red(0, 0, 255);
    Scalar yellow(0, 255, 255);
    Scalar blue(255, 0, 0);
    Scalar green(0, 255, 0);
    Scalar white(255, 255, 255);

    int erosion_size = 2;
    Mat kernel = getStructuringElement( MORPH_ELLIPSE,
                                        Size( 2 * erosion_size + 1, 2 * erosion_size + 1 ),
                                        Point( erosion_size, erosion_size ) );

    //erode( src_gray, src_gray, kernel );

    //morphologyEx(src_gray, src_gray, MORPH_CLOSE, kernel);

    findContours( src_gray, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );

    /* Find ellipses with a father and a son*/
    for (int c = 0; c < contours.size(); c++) {
        if (contours[c].size() > 4 && hierarchy[c][3] != -1) {
            RotatedRect elipse  = fitEllipse( Mat(contours[c]) );
            radio = (elipse.size.height + elipse.size.width) / 4;
            if (hierarchy[c][2] != -1) { //If has a son
                int hijo  = hierarchy[c][2];
                if (contours[hijo].size() > 4 ) {
                    RotatedRect elipseHijo  = fitEllipse( Mat(contours[hijo]) );
                    radio_hijo = (elipseHijo.size.height + elipseHijo.size.width) / 4;
                    /* Check center proximity */
                    if ( /*radio <= radio_hijo * 2 &&*/ cv::norm(elipse.center - elipseHijo.center) < radio_hijo / 2) {
                        ellipses_temp.push_back(PatterPoint(elipse.center.x, elipse.center.y, radio, hierarchy[c][3]));
                        ellipse(masked, elipseHijo, red, 2);
                        ellipse(masked, elipse, yellow, 2);
                    }
                }
            } else {
                ellipse(masked, elipse, purple, 2);
            }
        }
        drawContours( src_gray, contours, c, white, 2, 8, hierarchy, 0, Point() );
    }

    /* Filter ellipses how doesnt have another ones near to it */
    int count;
    for (int i = 0; i < ellipses_temp.size(); ++i) {
        count = 0;
        for (int j = 0; j < ellipses_temp.size(); ++j) {
            if (i == j) continue;
            radio = ellipses_temp[j].radio;

            distance = ellipses_temp[i].distance(ellipses_temp[j]);
            if (distance < radio * 5/*3.5*/) {
                line(masked, ellipses_temp[i].center(), ellipses_temp[j].center(), red, 1);
                count++;
            }
        }
        if (count >= 2) {
            radio = ellipses_temp[i].radio;
            radio_prom += radio;
            new_pattern_points.push_back(ellipses_temp[i]);
            circle(masked, ellipses_temp[i].center(), radio, blue, 5);
        }
    }
    radio_prom /= new_pattern_points.size();

    /* Clean false positive checking the father hierarchy */
    if (new_pattern_points.size() > 20) {
        int mode = mode_from_father(new_pattern_points);
        if (mode != -1) {
            RotatedRect elipse  = fitEllipse( Mat(contours[mode]) );
            ellipse(masked, elipse, white, 5);

            /* CLEAN USING MODE */
            vector<PatterPoint> temp ;
            for (int e = 0; e < new_pattern_points.size(); e++) {
                if (new_pattern_points[e].h_father == mode) {
                    temp.push_back(new_pattern_points[e]);
                }
            }
            new_pattern_points = temp;
        }
    }
    for (int e = 0; e < new_pattern_points.size(); e++) {
        circle(original, new_pattern_points[e].center(), new_pattern_points[e].radio, green, 5);
    }

    if (new_pattern_points.size() == 20) {
        keep_per_frames = 2;
        //pattern_points = new_pattern_points;

        /* Trakking */
        //vector<Point2f> points_2f(new_pattern_points.size());
        //for (int p = 0; p < new_pattern_points.size(); p++) {
        //    points_2f[p] = Point2f(new_pattern_points[p].x, new_pattern_points[p].y);
        //    putText(original, to_string(p), points_2f[p]/*cvPoint(10, 30)*/, FONT_HERSHEY_PLAIN, 2, cvScalar(0, 0, 255), 2, CV_AA);
        //}
        //RotatedRect boundRect = minAreaRect( Mat(points_2f) );
        //cout << "BoundRect angle " << boundRect.angle << endl;
        //Point2f rect_points[4];
        //boundRect.points( rect_points );
        //circle(original, rect_points[0], 10, Scalar(0, 255, 255), 10);
        //line(original, rect_points[0], rect_points[1], Scalar(0, 255, 255));
        //line(original, rect_points[1], rect_points[2], Scalar(0, 255, 255));
        //line(original, rect_points[2], rect_points[3], Scalar(0, 255, 255));
        //line(original, rect_points[3], rect_points[0], Scalar(0, 255, 255));
        draw_lines_pattern_from_ellipses(original, pattern_points, new_pattern_points);

    } else {
        if (keep_per_frames-- > 0) {
            new_pattern_points = pattern_points;
            draw_lines_pattern_from_ellipses(original, pattern_points, new_pattern_points);
        } else {
            new_pattern_points.clear();
            pattern_points.clear();
        }
    }

    update_mask_from_points(new_pattern_points, w, h, mask_point);
    return new_pattern_points.size();
}

/**
 * @details Build a line using points p1 and p2 and return the distance of this line to the points x
 *
 * @param p1 Point 1 to build the line
 * @param p2 Point 2 to build the line
 * @param x  Point to calc the distance to the line
 * @return Distance from the built line to x point
 */
float distance_to_rect(PatterPoint p1, PatterPoint p2, PatterPoint x) {
    float result = abs((p2.y - p1.y) * x.x - (p2.x - p1.x) * x.y + p2.x * p1.y - p2.y * p1.x) / sqrt(pow(p2.y - p1.y, 2) + pow(p2.x - p1.x, 2));
    return result;
}

/**
 * @details Calc the most distant points in a vector of points
 *
 * @param points Poinst to be evaluate
 * @return Most distant points order by x coordinate
 */
vector<PatterPoint> more_distant_points(vector<PatterPoint> points) {
    float distance = 0;
    double temp;
    int p1, p2;
    for (int i = 0; i < points.size(); i++) {
        for (int j = 0; j < points.size(); j++) {
            if (i != j) {
                temp = points[i].distance(points[j]);
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
    vector<PatterPoint> p;
    p.push_back(points[p1]);
    p.push_back(points[p2]);
    return p;
}
/**
 * @details Draw lines patter in drawing Mat from a vector of points
 *
 * @param drawing Mat to draw patter
 * @param pattern_centers Patter points found
 */
void draw_lines_pattern_from_ellipses(Mat &drawing, vector<PatterPoint> &pattern_centers, vector<PatterPoint> new_pattern_points) {
    if (new_pattern_points.size() < 20 && pattern_centers.size() < 20) {
        //cout << "Not enough points" << endl;
        return;
    } else {
        //cout << "Enough points" << endl;
    }
    vector<Scalar> color_palette(5);
    color_palette[0] = Scalar(255, 0, 255);
    color_palette[1] = Scalar(255, 0, 0);
    color_palette[2] = Scalar(0, 255, 0);
    color_palette[3] = Scalar(0, 0 , 255);

    int coincidendes = 0;
    int centers = pattern_centers.size();
    float pattern_range = 5;
    float distance;
    float min_distance;
    int replace_point;
    int line_color = 0;
    vector<PatterPoint> temp;
    vector<PatterPoint> line_points;
    vector<PatterPoint> limit_points;
    if (pattern_centers.size() == 0) {
        //cout << "Draw lines" << endl;
        //pattern_centers = new_pattern_points;
        centers = new_pattern_points.size();
        for (int i = 0; i < centers; i++) {
            putText(drawing, to_string(i), new_pattern_points[i].to_point2f()/*cvPoint(10, 30)*/, FONT_HERSHEY_PLAIN, 2, cvScalar(0, 0, 255), 2, CV_AA);
            for (int j = 0; j < centers; j++) {
                if (i != j) {
                    temp.clear();
                    line_points.clear();
                    coincidendes = 0;
                    for (int k = 0; k < centers; k++) {
                        if (distance_to_rect(new_pattern_points[i], new_pattern_points[j], new_pattern_points[k]) < pattern_range) {
                            coincidendes++;
                            line_points.push_back(new_pattern_points[k]);
                        }
                    }

                    if (coincidendes == 5) {
                        //line_points = more_distant_points(line_points);
                        sort(line_points.begin(), line_points.end(), sort_pattern_point_by_x);
                        if(line_points[4].x - line_points[0].x < line_points[0].radio){
                            sort(line_points.begin(), line_points.end(), sort_pattern_point_by_y);
                        }
                        bool found = false;
                        for (int l = 0; l < limit_points.size(); l++) {
                            if (limit_points[l].x == line_points[0].x && limit_points[l].y == line_points[0].y) {
                                found = true;
                            }
                        }
                        /*bool found = false;
                        for (int l = 0; l < limit_points.size(); l++) {
                            if (limit_points[l].x == line_points[0].x && limit_points[l].y == line_points[0].y) {
                                found = true;
                            }
                        }
                        */

                        if (!found) {
                            for (int l = 0; l < line_points.size(); l++) {
                                pattern_centers.push_back(line_points[l]);
                            }

                            limit_points.push_back(line_points[0]);
                            limit_points.push_back(line_points[4]);
                            //if (line_color != 0) {
                            //    line(drawing, line_points[4].to_point2f(), limit_points[line_color * 2 - 2].to_point2f(), color_palette[line_color], 2);
                            //}
                            //line(drawing, line_points[0].to_point2f(), line_points[4].to_point2f(), color_palette[line_color], 2);
                            //line_color++;
                        }
                    }
                }
            }
        }
    } else {
        //cout << "Pattern Traking" << endl;

        for (int p = 0; p < pattern_centers.size(); p++) {
            replace_point = 0;
            min_distance = 100;
            for (int n = 0; n < new_pattern_points.size(); n++) {
                distance = pattern_centers[p].distance(new_pattern_points[n]);
                if (min_distance > distance) {
                    min_distance = distance;
                    replace_point = n;
                }
            }
            if(min_distance > pattern_centers[p].radio){
                min_distance = -1;
                break;
            }
            line(drawing, pattern_centers[p].to_point2f(), new_pattern_points[replace_point].to_point2f(), Scalar(0, 255, 255), 1);
            pattern_centers[p] = new_pattern_points[replace_point];
            putText(drawing, to_string(p), pattern_centers[p].to_point2f()/*cvPoint(10, 30)*/, FONT_HERSHEY_PLAIN, 2, cvScalar(0, 0, 255), 2, CV_AA);
        }
        if(min_distance == -1){
            pattern_centers.clear();
            return;
        }

        line(drawing, pattern_centers[0].to_point2f(), pattern_centers[4].to_point2f(), Scalar(0, 255, 255), 1);
        line(drawing, pattern_centers[5].to_point2f(), pattern_centers[4].to_point2f(), Scalar(0, 255, 255), 1);
        line(drawing, pattern_centers[5].to_point2f(), pattern_centers[9].to_point2f(), Scalar(0, 255, 255), 1);
        line(drawing, pattern_centers[10].to_point2f(), pattern_centers[9].to_point2f(), Scalar(0, 255, 255), 1);
        line(drawing, pattern_centers[10].to_point2f(), pattern_centers[14].to_point2f(), Scalar(0, 255, 255), 1);
        line(drawing, pattern_centers[15].to_point2f(), pattern_centers[14].to_point2f(), Scalar(0, 255, 255), 1);
        line(drawing, pattern_centers[15].to_point2f(), pattern_centers[19].to_point2f(), Scalar(0, 255, 255), 1);
    }
}

/**
 * @details Update mask points from detected points
 *
 * @param points Patter points found
 * @param w Original image width
 * @param h Original image height
 * @param mask_point Array which store mask points
 */
void update_mask_from_points(vector<PatterPoint> points, int w, int h, Point mask_point[][4]) {
    if (points.size() < 20) {
        mask_point[0][0]  = Point(0, 0);
        mask_point[0][1]  = Point(h, 0);
        mask_point[0][2]  = Point(h, w);
        mask_point[0][3]  = Point(0, w);
        return;
    }
    vector<Point2f> points_2f(points.size());
    for (int p = 0; p < points.size(); p++) {
        points_2f[p] = Point2f(points[p].x, points[p].y);
    }
    RotatedRect boundRect = minAreaRect( Mat(points_2f) );
    Point2f rect_points[4];
    boundRect.points( rect_points );

    double scale = 1.5;
    Point mask_center(( rect_points[0].x +
                        rect_points[1].x +
                        rect_points[2].x +
                        rect_points[3].x) / 4,
                      (rect_points[0].y +
                       rect_points[1].y +
                       rect_points[2].y +
                       rect_points[3].y) / 4);

    mask_point[0][0]  = Point((rect_points[0].x - mask_center.x) * scale + mask_center.x, (rect_points[0].y - mask_center.y) * scale + mask_center.y);
    mask_point[0][1]  = Point((rect_points[1].x - mask_center.x) * scale + mask_center.x, (rect_points[1].y - mask_center.y) * scale + mask_center.y);
    mask_point[0][2]  = Point((rect_points[2].x - mask_center.x) * scale + mask_center.x, (rect_points[2].y - mask_center.y) * scale + mask_center.y);
    mask_point[0][3]  = Point((rect_points[3].x - mask_center.x) * scale + mask_center.x, (rect_points[3].y - mask_center.y) * scale + mask_center.y);
}