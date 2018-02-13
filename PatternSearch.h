#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

using namespace cv;
using namespace std;
void clean_from_elipses(Mat &out, vector<Point2f> pattern_centers);


float average(vector<float> data)
{
    float sum = 0;
    for (int i = 0; i < data.size(); ++i)
        sum += data[i];

    return sum / data.size();
}

void find_points(Mat &src_gray, Mat&original,vector<Point2f> &pattern_points,int &keep_per_frames) {
    


    Mat threshold_output;
    vector<vector<Point> > contours;
    vector<Vec4i> hierarchy;
    Point2f center, center_2;

    int THRESHOLD = 30;

    /// Detect edges using Threshold
    threshold( src_gray, threshold_output, THRESHOLD, 255, THRESH_BINARY );


    //Dilate
    int erosion_size = 2;
    Mat kernel = getStructuringElement( MORPH_ELLIPSE,
                                        Size( 2 * erosion_size + 1, 2 * erosion_size + 1 ),
                                        Point( erosion_size, erosion_size ) );

    erode( threshold_output, threshold_output, kernel );
    // erode( threshold_output, threshold_output, kernel );
    // erode( threshold_output, threshold_output, kernel );

    // morphologyEx(threshold_output, threshold_output, MORPH_CLOSE, kernel);

    // cv::resize(threshold_output, threshold_output, Size(600, 600));
    imshow( "source_window", threshold_output );



    /// Find contours
    findContours( threshold_output, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );


    vector<vector<Point> > contours_temp;
    vector<RotatedRect>    ellipses_temp;

    for (int contour = 0; contour < contours.size(); ++contour)
    {
        if (hierarchy[contour][2] != -1) { //si tiene un hijo

            int padre = contour;
            int hijo  = hierarchy[contour][2];

            if ( contours[padre].size() > 7 && contours[hijo].size() > 7 ) {
                RotatedRect elipseHijo  = fitEllipse( Mat(contours[hijo]) );
                RotatedRect elipsePadre = fitEllipse( Mat(contours[padre]) );

                if ( cv::norm(elipsePadre.center - elipseHijo.center) < 15) { //CALIBRAR DE ACUERDO A VIDEO
                    contours_temp.push_back(contours[padre] );
                    ellipses_temp.push_back(elipsePadre);

                    //  Find the area of contour
                    // double area=contourArea( contours[padre],false);
                    // cout<<area<<endl;
                }
            }
        }
    }

    /************ MIN DISTANCE ***************/
    vector<float> minAdjDistances;

    // cout << "Min distances" << endl;
    // find min distance for every ellipse center
    for (int i = 0; i < ellipses_temp.size(); ++i)
    {
        float minDistance = 1000000;
        for (int j = 0; j < ellipses_temp.size(); ++j)  //find min distance
        {
            if (i == j) continue;

            float distance = cv::norm(ellipses_temp[i].center - ellipses_temp[j].center);

            if (distance < minDistance )
                minDistance = distance;
        }
        if (minDistance > 18 &&  minDistance < 70) //CALCULAR ACUERDO AL VIDEO
        {
            // cout << minDistance << endl;
            minAdjDistances.push_back(minDistance);
        }
    }

    // cout << "minAdjDistances size: " << minAdjDistances.size() << endl;
    float mean = average(minAdjDistances);

    

    // cout << "mean: " << mean << endl;
    /************ end MIN DISTANCE ***************/


    /************ CLUSTER ELIPSE MODELO BASED ON THE MEAN ***************/
    //obtener propiedades caracteristicas de las ellipses
    vector<RotatedRect> cluster_ellipse_model;
    int counter = 0;
    for (int i = 0; i < ellipses_temp.size(); ++i)
    {
        for (int j = 0; j < ellipses_temp.size(); ++j)
        {
            if (i == j) continue;

            float distance = cv::norm(ellipses_temp[i].center - ellipses_temp[j].center);

            if (distance < 2 * mean ) //AJUSTAR POR VIDEO
                counter++;

            if (counter == 4) {
                cluster_ellipse_model.push_back(ellipses_temp[i]);
                break;
            }
        }
        counter = 0;
    }

    float avg_width, avg_height, sum_width = 0.0, sum_height = 0.0;

    for (int i = 0; i < cluster_ellipse_model.size(); ++i)
    {
        sum_height += cluster_ellipse_model[i].size.height;
        sum_width += cluster_ellipse_model[i].size.width;
    }
    avg_width = sum_width / cluster_ellipse_model.size();
    avg_height = sum_height / cluster_ellipse_model.size();

    // cout << "avg_width: " << avg_width << endl;
    // cout << "avg_height: " << avg_height << endl;

    /************ END CLUSTER ELIPSE MODELO BASED ON THE MEAN ***************/


    /************ FIND ELLIPSES v2 BASED ON SOME CONDITIONS ****************/

    // vector<RotatedRect> ellipses_temp_v2;
    // for (int contour = 0; contour < contours.size(); ++contour)
    // {
    //     if ( contours[contour].size() > 5 ) {

    //         RotatedRect ellipse = fitEllipse( Mat(contours[contour]) );

    //         //si el error relativo es menor del 30%
    //         if (abs(ellipse.size.width - avg_width) / ellipse.size.width < 0.30 &&
    //             abs(ellipse.size.height - avg_height) / ellipse.size.height < 0.30 )
    //         {
    //             ellipses_temp_v2.push_back(ellipse);
    //         }
    //     }
    // }
    /************ END FIND ELLIPSES v2 BASED ON SOME CONDITIONS ****************/


    /************ CLUSTER CONTROL POINTS BASED ON THE MEAN  ***************/
    vector<RotatedRect> cluster;
    for (int i = 0; i < ellipses_temp.size(); ++i)
    {
        for (int j = 0; j < ellipses_temp.size(); ++j)
        {
            if (i == j) continue;

            float distance = cv::norm(ellipses_temp[i].center - ellipses_temp[j].center);

            //si el error relativo es menor del 10% y tiene un vecino
            if (abs(ellipses_temp[i].size.width - avg_width) / ellipses_temp[i].size.width < 0.25 &&
                    abs(ellipses_temp[i].size.height - avg_height) / ellipses_temp[i].size.height < 0.25 &&
                    distance < 1.5 * mean)
            {
                cluster.push_back(ellipses_temp[i]);
                break;
            }
        }
    }

    // counter = 0;
    // for (int i = 0; i < ellipses_temp_v2.size(); ++i)
    // {
    //     for (int j = 0; j < ellipses_temp_v2.size(); ++j)
    //     {
    //         if (i == j) continue;

    //         float distance = cv::norm(ellipses_temp_v2[i].center - ellipses_temp_v2[j].center);

    //         //si el error relativo es menor del 10% y tiene un vecino
    //         if (abs(ellipses_temp_v2[i].size.width - avg_width) / ellipses_temp_v2[i].size.width        < 0.25 &&
    //             abs(ellipses_temp_v2[i].size.height - avg_height) / ellipses_temp_v2[i].size.height < 0.25 &&
    //                 distance < 1.9 * mean) // has to be different from the one used to fit ellipses_temp_v2
    //         {
    //             counter++;
    //         }

    //         if (counter == 2)
    //         {
    //             cluster.push_back(ellipses_temp_v2[i]);
    //             break;
    //         }
    //     }
    //     counter = 0;
    // }
    /************ END CLUSTER BASED ON THE MEAN AND SD ***************/

    // cout << "cluster.size(): " << cluster.size() << endl;
    // cout << "contours_temp size: " << contours_temp.size() << endl;


  



    






    if (cluster.size() == 20){
        pattern_points.clear();
        for (int i = 0; i < cluster.size(); ++i) {
            int thickness = -1;
            int lineType = 8;
            circle( original,
                    cluster[i].center,
                    3,
                    Scalar( 0, 255, 0 ),
                    thickness,
                    lineType );
            pattern_points.push_back(cluster[i].center);
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