/**
* Copyright (C) 2017-present, Facebook, Inc.
*
* This library is free software; you can redistribute it and/or
* modify it under the terms of the GNU Lesser General Public
* License as published by the Free Software Foundation; either
* version 2.1 of the License, or (at your option) any later version.
*
* This library is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
* Lesser General Public License for more details.
*
* You should have received a copy of the GNU Lesser General Public
* License along with this library; if not, write to the Free Software
* Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
*/

#include "PolynomialFit.h"
#include "PolynomialSaddleDetectorContext.h"
//#include <deltille/target_detector.h>
#include "GridDetectorContext.h"
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
//#include <deltille/target_detector.h>
#include <iostream>
#include <chrono>
using namespace orp::calibration;
using namespace cv;
using namespace std;

namespace findSaddlesPoints {

template <typename SaddlePointType>
void drawPoints(Mat src , vector<SaddlePointType> points, cv::Scalar color ) {
    for (int i = 0 ; i < points.size() ; i ++ ) {
        //circle(src,points[i], 5, color);
        float x = points[i].x;
        float y = points[i].y;
        //cout << x << " " << y  << endl;
        double size = 5.0;
        line( src, Point2f(x - size, y), Point2f(x + size, y) , color);
        line( src, Point2f(x, y - size), Point2f(x, y + size) , color);

        if ( i != points.size() - 1 ) line( src, Point2f(x, y), Point2f(points[i + 1].x, points[i + 1].y) , Scalar(255, 255, 0));

        putText(src, to_string(i), Point2f(x, y), FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2);
    }
}


vector<Point2f>convertToPoint2f(vector<MonkeySaddlePointSpherical>points) {
    vector<Point2f>point2f;
    for (int i = 0; i < points.size(); ++i) {
        point2f.push_back(Point2f(points[i].x, points[i].y));
    }
    return point2f ;
}

bool sortbyY(Point2f i, Point2f j) {
    return (i.y < j.y);
}
bool sortbyDescY(Point2f i, Point2f j) {
    return (i.y > j.y);
}
bool sortbyX(Point2f i, Point2f j) {
    return (i.x < j.x);
}
/*
double distance(Point2f a, Point2f b) {
    return sqrt(    (a.x - b.x) * (a.x - b.x)  +   (a.y - b.y) * (a.y - b.y) );
}*/

float distance(Point2f p1, Point2f p2) {
    return pow(p1.x - p2.x, 2) + pow(p1.y - p2.y, 2);
}

float distance_to_rect(Point2f p1, Point2f p2, Point2f x) {
    float l2 = distance(p1, p2);
    if (l2 == 0.0) return sqrt(distance(p1, x));
    //float result = abs((p2.y - p1.y) * x.x - (p2.x - p1.x) * x.y + p2.x * p1.y - p2.y * p1.x) / sqrt(pow(p2.y - p1.y, 2) + pow(p2.x - p1.x, 2));
    //return result;
    float t = ((x.x - p1.x) * (p2.x - p1.x) + (x.y - p1.y) * (p2.y - p1.y)) / l2;
    t = max(0.0f, min(1.0f, t));
    float result = distance(x, Point2f(p1.x + t * (p2.x - p1.x),
                                       p1.y + t * (p2.y - p1.y)));
    return sqrt(result);
}

bool sort_pattern_point_by_x(Point2f p1, Point2f p2) {
    return p1.x < p2.x;
}

bool sort_pattern_point_by_y(Point2f p1, Point2f p2) {
    return p1.y < p2.y;
}

Point2f getBarycenter(vector<Point2f> points) {
    Point2f barycenter(0, 0);
    for (int i = 0 ; i < points.size() ; i ++ ) {
        barycenter += points[i];
    }
    barycenter.x /= points.size();
    barycenter.y /= points.size();
    return barycenter;

}

int getfarthestPoint(vector<Point2f> points, Point2f barycenter) {
    double max = 0 ;
    int j ;
    for (int i = 0; i < points.size(); ++i) {
        if (distance(points[i], barycenter) > max) {
            max = distance(points[i], barycenter);
            j = i ;
        }
    }
    return j ;
}

vector<Point2f> getCorners(vector<Point2f> &points, Point2f barycenter) {
    vector<Point2f> corners ;
    int i = getfarthestPoint(points, barycenter);
    corners.push_back(points[i]);
    points.erase(points.begin() + i);
    i = getfarthestPoint(points, barycenter);
    corners.push_back(points[i]);
    points.erase(points.begin() + i);
    i = getfarthestPoint(points, barycenter);
    corners.push_back(points[i]);
    points.erase(points.begin() + i);
    i = getfarthestPoint(points, barycenter);
    corners.push_back(points[i]);
    points.erase(points.begin() + i);
    return corners;
}

void order_points_in_lines(int odd, int even, double radio, vector<Point2f>&points) {
    sort(points.begin(), points.end(), sortbyDescY);
    int coincidendes;
    float pattern_range = 4;
    float min_distance;
    int n_points = points.size();
    vector<Point2f> line_points;
    vector<Point2f> limit_points;
    vector<Point2f> ordered_points;
    int rows = 0;

    for (int i = 0; i < n_points; i++) {
        for (int j = 0; j < n_points; j++) {
            if (i != j) {
                line_points.clear();
                coincidendes = 0;
                for (int k = 0; k < n_points; k++) {
                    min_distance = distance_to_rect(points[i], points[j], points[k]);
                    if (min_distance < pattern_range) {
                        coincidendes++;
                        line_points.push_back(points[k]);
                    }
                }
                //cout << "Coincidences found " << coincidendes << " for row " << rows << endl;
                if ((rows % 2 != 0 && coincidendes == odd) || (rows % 2 == 0 && coincidendes == even)) {
                    sort(line_points.begin(), line_points.end(), sort_pattern_point_by_x);
                    //if (line_points[4].x - line_points[0].x < radio) {
                    //    sort(line_points.begin(), line_points.end(), sort_pattern_point_by_y);
                    //}
                    bool found = false;
                    for (int l = 0; l < limit_points.size(); l++) {
                        for (int p = 0; p < line_points.size(); p++) {
                            if (limit_points[l].x == line_points[p].x && limit_points[l].y == line_points[p].y) {
                                found = true;
                            }
                        }
                    }
                    if (!found) {
                        rows++;
                        //cout << "Row " << rows - 1 << " is " << endl;
                        for (int l = 0; l < line_points.size(); l++) {
                            ordered_points.push_back(line_points[l]);
                            limit_points.push_back(line_points[l]);
                            //cout << line_points[l].x << " " << line_points[l].y << endl;
                            //putText(drawing, to_string(pattern_centers.size() - 1), line_points[l].to_point2f()/*cvPoint(10, 30)*/, FONT_HERSHEY_COMPLEX_SMALL, 1, cvScalar(0, 0, 255), 2);
                        }
                        //limit_points.push_back(line_points[0]);
                        //limit_points.push_back(line_points[line_points.size() - 1]);
                    }
                }
            }
        }
    }
    points.clear();
    for (int p = 0; p < ordered_points.size(); p++) {
        points.push_back(ordered_points[p]);
    }
    //cout << "Ordered points " << ordered_points.size() << endl;
}



bool isLeft(Point2f a, Point2f b, Point2f c){
    Point2f v1(b.x-a.x, b.y-a.y);   // Vector 1
    Point2f v2(c.x-a.x, c.y-a.y);   // Vector 2
    float det = v1.x*v2.y - v1.y*v2.x;  // Cross product
    return det < 0;
}

bool isRight(Point2f a, Point2f b, Point2f c){
    Point2f v1(b.x-a.x, b.y-a.y);   // Vector 1
    Point2f v2(c.x-a.x, c.y-a.y);   // Vector 2
    float det = v1.x*v2.y - v1.y*v2.x;  // Cross product
    return det > 0;
}
         //                      saddle                           sorted (lines)
void filterPoints(Mat& frame,vector<Point2f>& initialPoints, vector<Point2f>& points2f){
    
    vector<Point2f> removedLeftList, removedLeftAndRightList;
    for (int i = 0; i < initialPoints.size(); ++i)
    {
        // circle(frame, initialPoints[i], 2, Scalar(255, 255, 20), -1);
        if( isLeft(points2f[8], points2f[25], initialPoints[i]) 
            || isRight(points2f[16], points2f[33], initialPoints[i]) ){
            // circle(frame, initialPoints[i], 3, Scalar(0, 255, 255));
        }else{
            removedLeftList.push_back(initialPoints[i] );
        }
    }
    points2f = removedLeftList;
}

void load_object_points(int cols, int rows, vector<Point2f>& object_points_image) {
        Size boardSize(cols, rows);
        int squareSize = 20;
        //int squareSize_2 = 20;
        //int squareSize = 21;
        int squareSize_2 = 18;

        object_points_image.clear();
        //cout << "Object points " << endl;
        for ( int i = 0; i < boardSize.height; i++ ) {
            if (i % 2) {
                for ( int j = 0; j < boardSize.width + 1; j++ ) {
                    object_points_image.push_back(Point2f(j * 64.0 + 64 , float(480 - (i * 64.0 + 96.0))));
                }
            } else {
                for ( int j = 0; j < boardSize.width; j++ ) {
                    object_points_image.push_back(Point2f((2 * j + 1) / 2.0 * 64.0 + 64 , float(480 - (i * 64.0 + 96.0))));
                }
            }
        }
        
    }



bool findSaddleCenters(Mat &gray, vector<Point2f> &order_points, Mat frame, bool FP = false) {
    bool found = false ;

    vector<cv::Point> locations;
    PolynomialSaddleDetectorContext<MonkeySaddlePointSpherical, uint16_t, double> detector(gray);
    vector<MonkeySaddlePointSpherical> points;

    detector.findSaddles(points);

    vector<Point2f> points2f = convertToPoint2f(points);

    Mat InitialPointsFounded = frame.clone(); // (480, 600, CV_8UC3, Scalar(0, 0, 0));
    for (int p = 0; p < points2f.size(); p++) {
        circle(InitialPointsFounded, points2f[p], 2, Scalar(0, 255, 0), -1);
        // putText(InitialPointsFounded, to_string(p),  points2f[p], FONT_HERSHEY_COMPLEX_SMALL, 0.4,  cvScalar(255, 255, 255), 1);
    }

    if (FP)
    {
            /* Filter points with ideals points  */
            vector<Point2f> object_points_image;
            load_object_points(8, 5, object_points_image);

            vector<Point2f> points2f_filtered;
            for (int i = 0; i < object_points_image.size(); ++i)
            {
                bool found = false;
                for (int j = 0; j < points2f.size(); ++j)//
                {
                    if (norm(object_points_image[i] - points2f[j] ) < 10){
                        circle(InitialPointsFounded, points2f[j], 4, Scalar(0, 255, 255));
                        points2f_filtered.push_back(points2f[j]);
                        found = true;
                        break;
                    }
                }
                if (!found){
                    circle(InitialPointsFounded, object_points_image[i], 7, Scalar(0, 255, 255));
                    points2f_filtered.push_back(object_points_image[i]);
                }
            }
            // cout<<"points2f_filtered size: "<<42- points2f_filtered.size()<<endl;
            points2f = points2f_filtered;
            order_points_in_lines(9, 8, 5, points2f);
    }else{
        vector<Point2f> points2fInitial(points2f); 
        order_points_in_lines(9, 8, 5, points2f);

        if (points2f.size() == 42) { //'cause intial frames 
            filterPoints(InitialPointsFounded, points2fInitial, points2f);    
            order_points_in_lines(9, 8, 5, points2f);
        }
    }

    for (int p = 0; p < points2f.size(); p++) {
        circle(InitialPointsFounded, points2f[p], 1, Scalar(0, 0, 255), -1);
        putText(InitialPointsFounded, to_string(p),  points2f[p], FONT_HERSHEY_COMPLEX_SMALL, 0.5,  cvScalar(255, 255, 255), 0.5);
    }

    if (points2f.size() == 42 ) {
        order_points.clear();
        for (int p = 0; p < points2f.size(); p++) {
            order_points.push_back(points2f[p]);
            //putText(gray, to_string(p),  order_points[p], FONT_HERSHEY_COMPLEX_SMALL, 0.5,  cvScalar(255, 255, 255), 1);
        }
        found = order_points.size() == 42 ;
    }
    waitKey(1);
    return found;
}

}



// void test_two(){

//     string filename = "videos/test.avi";
//     VideoCapture capture(filename);
//     int i=0;
//     Mat I ;
//     for( ; ; )
//     {
//         capture >> I;
//         if(I.empty())
//             break;

//         Mat gray ;

//         auto t0 = chrono::high_resolution_clock::now();

//         cvtColor(I, gray, CV_BGR2GRAY);
//         vector<cv::Point> locations;
//         PolynomialSaddleDetectorContext<MonkeySaddlePointSpherical,uint16_t,double> detector(gray);
//         //SaddlePointVector refclust;
//         //detector.findSaddles(refclust);

//         // se cambio el metodo a publico
//         vector<MonkeySaddlePointSpherical> points;

//         // find initial saddles on lowres image...
//         // detector.getInitialSaddleLocations(gray,locations);
//         // Mat I_initial = I.clone();
//         // drawPoints(I_initial,locations,Scalar(0,0,255));
//         // imshow("Initial Points",I_initial);
//         // imwrite("results/"+to_string(i)+".png",I_initial);
//         // waitKey(10);
//         detector.findSaddles(points);

//         if(points.size() == 42 ){

//             cout << points.size() << endl;

//             auto t1 = chrono::high_resolution_clock::now();
//             printf("Time taked : %.3f ms\n",
//                          duration_cast<microseconds>(t1 - t0).count() / 1000.0);
//             //precomputePolaritiesAndNN(refclust);
//             //cout << locations << endl;
//             vector<Point2f> points2f = convertToPoint2f(points);
//             sort(points2f.begin(),points2f.end(),sortbyY);
//             Point2f barycenter = getBarycenter(points2f);

//             vector<Point2f>up ;
//             vector<Point2f>down ;
//             for (int i = 0; i < points2f.size(); ++i){
//                 if(points2f[i].y <  barycenter.y - 5 ){
//                     up.push_back(points2f[i]);
//                 }else{
//                     down.push_back(points2f[i]);
//                 }
//             }
//             //circle(I,barycenter, 5,Scalar(0,0,255));
//             //drawPoints(I,points2f,Scalar(0,0,255));
//             //drawChessboardCorners(I,Size(8,5),points2f,true);
//             //imshow("Deltille Corner Detection",I);
//             imwrite("images/frames/"+to_string(i)+".jpg",I);
//             waitKey(10);
//             i++;
//         }
//     }
// }

// int main(int argc, char const *argv[])
// {
//     test_one();
//     //  test_two();
//     return 0;
// }
