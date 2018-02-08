#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
// highgui - an interface to video and image capturing.

#include <iostream>
// The header files for performing input and output.

using namespace cv;
// Namespace where all the C++ OpenCV functionality resides.

using namespace std;
// For input output operations.

Mat edgeDetector(Mat src);
Mat CannyEdgeDetector(Mat src);
void find_points(Mat &src_gray, Mat &original);
void clean_from_elipses(Mat &drawing, vector<Point2f> pattern_centers);

int main() {
	//VideoCapture cap("/home/alonzo/Documentos/Projects/CameraCalibration_2/video/calibration_mslifecam.avi");
	//VideoCapture cap("/home/alonzo/Documentos/Projects/CameraCalibration_2/video/calibration_kinectv2.avi");
	VideoCapture cap("/home/alonzo/Documentos/Projects/CameraCalibration_2/video/calibration_ps3eyecam.avi");
	//VideoCapture cap("/home/alonzo/Documentos/Projects/CameraCalibration_2/video/calibration_realsense.avi");
	//VideoCapture cap("/home/alonzo/Documentos/Projects/CameraCalibration_2/video/calibration_ps3eyecam.avi");

	// cap is the object of class video capture that tries to capture Bumpy.mp4
	if ( !cap.isOpened() )  // isOpened() returns true if capturing has been initialized.
	{
		cout << "Cannot open the video file. \n";
		return -1;
	}

	double fps = cap.get(CV_CAP_PROP_FPS); //get the frames per seconds of the video

	namedWindow("Camera calibration", CV_WINDOW_AUTOSIZE); //create a window called "MyVideo"

	// Mat fgbg = createBackgroundSubtractorMOG2();
	int wait_key = 60;
	while (1)
	{
		Mat frame;

		if (!cap.read(frame)) // if not success, break loop
		{
			cout << "\n Cannot read the video file. \n";
			// break;
		}
		Mat edges = CannyEdgeDetector(frame);

		// imshow("Camera calibration", edges);
		// imwrite("edges.png", edges);
		int erosion_size = 1;
        Mat kernel = getStructuringElement( MORPH_ELLIPSE,
                                             Size( 2 * erosion_size + 1, 2 * erosion_size + 1 ),
                                             Point( erosion_size, erosion_size ) );

        morphologyEx(edges, edges, MORPH_CLOSE, kernel);


		find_points(edges, frame );
		//namedWindow( "Contours", CV_WINDOW_AUTOSIZE );
		// cv::resize(drawing, drawing,Size(512,256));
		//imshow( "Contours", points );
		imshow( "Original", frame );
		char t = (char)waitKey(wait_key);
		if ( t == 27) // Wait for 'esc' key press to exit
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

Mat CannyEdgeDetector(Mat src) {

	int edgeThresh = 1;
	int lowThreshold = 50;
	int const max_lowThreshold = 100;
	int ratio = 3;
	int kernel_size = 3;

	Mat dst, detected_edges, src_gray;


	/// Convert the image to grayscale
	cvtColor( src, src_gray, CV_BGR2GRAY );



	/// Reduce noise with a kernel 3x3
	blur( src_gray, detected_edges, Size(3, 3) );

	/// Canny detector
	Canny( detected_edges, detected_edges, lowThreshold, lowThreshold * ratio, kernel_size );

	/// Using Canny's output as a mask, we display our result
	dst = Scalar::all(0);


	src_gray.copyTo( dst, detected_edges);

	/// Apply Histogram Equalization
	// cv::equalizeHist( dst, dst );

	return dst;

}


/** @function findPoints */
void find_points(Mat &src_gray, Mat&original) {
	int thresh = 30;
	int max_thresh = 255;
	RNG rng(12345);


	/// Convert image to gray and blur it
	// Mat src_gray;
	// cvtColor( src, src_gray, CV_BGR2GRAY );
	// blur( src_gray, src_gray, Size(3, 3) );


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

	for ( int i = 0; i < contours.size(); i++ )
	{
		minRect[i] = minAreaRect( Mat(contours[i]) );
		if ( contours[i].size() > 5 )
			minEllipse[i] = fitEllipse( Mat(contours[i]) );
	}

	/// Draw contours + rotated rects + ellipses
	//Mat drawing =  Mat::zeros( threshold_output.size(), CV_8UC3 );

	// Rect rect = minEllipse[i].boundingRect2f() ;
	vector<Point2f> concentric;
	float minRadius = 2.00f;
	int nroCircles = 2;
	int count = 0;
	for ( int i = 0; i < contours.size(); i++ )
	{
		Rect rect1 = minEllipse[i].boundingRect() ;
		Point2f center1 = (rect1.br() + rect1.tl()) * 0.5;
		for ( int j = 0; j < contours.size(); j++ )
		{
			if (i == j) continue;

			Rect rect2 = minEllipse[j].boundingRect() ;
			Point2f center2 = (rect2.br() + rect2.tl()) * 0.5;

			if (cv::norm(center1 - center2) <= minRadius)
				count++;

			if (count == nroCircles) {
				concentric.push_back(center1);
				break;
			}
		}
		count = 0;
	}
	//ppt  + doc en formato de paper

	cout << "size: " << concentric.size() << endl;
	// for (int j = 0; j < concentric.size(); ++j)
	// {
	// 	int thickness = -1;
	// 	int lineType = 8;
	// 	circle( drawing,
	// 	        concentric[j],
	// 	        3,
	// 	        Scalar( 255, 255, 255 ),
	// 	        thickness,
	// 	        lineType );
	// }
	/**************************************************/

	vector<vector<Point2f>> points;
	minRadius = 8.50f;
	while ( concentric.size() != 0 )
	{
		Point2f point = concentric[0];

		vector<Point2f> temp;
		temp.push_back(point);

		for (int j = 1; j < concentric.size(); ++j)
			if (cv::norm(point - concentric[j]) < minRadius)
				temp.push_back(concentric[j]);

		if (temp.size() >= 2 && temp.size() <= 5)
			points.push_back(temp);

		for (int i = 0; i < temp.size(); i++)
			concentric.erase(  std::remove(concentric.begin(), concentric.end(), temp[i])  , concentric.end()  );
	}
	// cout << "temp: " << temp.size() << endl;
	cout << "nro de cluster: " << points.size() << endl;
	cout << "concentric size: " << concentric.size() << endl;

	vector<Point2f> medianPoints;

	for (int i = 0; i < points.size(); ++i) {
		Point2f median(0.0f, 0.0f);
		for (int j = 0; j < points[i].size(); ++j)
			median += points[i][j];
		medianPoints.push_back( Point2f(median.x / points[i].size(), median.y / points[i].size()) );
	}

	if (points.size() == 20)
		for (int i = 0; i < medianPoints.size(); ++i) {
			int thickness = -1;
			int lineType = 8;
			circle( original,
			        medianPoints[i],
			        3,
			        Scalar( 255, 255, 255 ),
			        thickness,
			        lineType );
		}

	clean_from_elipses(original, medianPoints);



	// for (int i = 0; i < points.size(); ++i){
	// 	for (int j = 0; j < points[0].size(); ++j)
	// 	{
	// 		int thickness = -1;
	// 		int lineType = 8;
	// 		circle( drawing,
	// 		        points[i][j],
	// 		        3,
	// 		        Scalar( 255, 255, 255 ),
	// 		        thickness,
	// 		        lineType );
	// 	}
	// }






	// for ( int i = 0; i < contours.size(); i++ )
	// {
	//   Scalar color = Scalar( rng.uniform(0, 255), rng.uniform(0, 255), rng.uniform(0, 255) );
	//   // contour
	//   // drawContours( drawing, contours, i, color, 1, 8, vector<Vec4i>(), 0, Point() );

	//   // ellipse
	//   ellipse( drawing, minEllipse[i], color, 2, 8 );

	//   // rotated rectangle
	//   // Point2f rect_points[4]; minRect[i].points( rect_points );
	//   // for ( int j = 0; j < 4; j++ )
	//   //   line( drawing, rect_points[j], rect_points[(j + 1) % 4], color, 1, 8 );
	// }

	/// Show in a window
	// namedWindow( "Contours", CV_WINDOW_AUTOSIZE );
	// cv::resize(drawing, drawing,Size(512,256));
	// imshow( "Contours", drawing );
	//return drawing;
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
	if (pattern_centers.size() != 20) {
		return;
	}

	vector<Scalar> color_palette(5);
	color_palette[0] = Scalar(255, 0, 255);
	color_palette[1] = Scalar(255, 0, 0);
	color_palette[2] = Scalar(0, 255, 0);
	color_palette[3] = Scalar(0,0 , 255);
	/*
	ax + by + c = 0
	y = -(c + ax )/ b
	a = y1-y2,
	b = x2-x1,
	c = (x1-x2)*y1 + (y2-y1)*x1.
	*/
	int coincidendes = 0;
	int centers = pattern_centers.size();
	float pattern_range = 5;
	cout << "============" << endl;
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
						//line(drawing, pattern_centers[i], pattern_centers[j], color_palette[0]);
						line(drawing, line_points[0], line_points[1], color_palette[line_color],2);
						//cout << "Line Detected" << endl;
						//pattern_centers = temp;
						line_color++;
					}
				}
			}
		}
	}
	cout << "============" << endl;
}
void clean_from_elipses_back(Mat &drawing, vector<Point2f> pattern_centers) {
	//pattern_centers.clear();
	//pattern_centers.push_back(Point2f(390,220));
	//pattern_centers.push_back(Point2f(310,220.5));
	//pattern_centers.push_back(Point2f(266,221.25));
	//pattern_centers.push_back(Point2f(220,221.5));
	//pattern_centers.push_back(Point2f(350.75,220));
	//pattern_centers.push_back(Point2f(393,177));
	//pattern_centers.push_back(Point2f(354,176));
	//pattern_centers.push_back(Point2f(312,174.5));
	//pattern_centers.push_back(Point2f(267.5,174));
	//pattern_centers.push_back(Point2f(222,173));
	//pattern_centers.push_back(Point2f(396,134));
	//pattern_centers.push_back(Point2f(356,131));
	//pattern_centers.push_back(Point2f(314,128.25));
	//pattern_centers.push_back(Point2f(270,126));
	//pattern_centers.push_back(Point2f(224,124));
	//pattern_centers.push_back(Point2f(398.75,90));
	//pattern_centers.push_back(Point2f(359.25,86));
	//pattern_centers.push_back(Point2f(317,82));
	//pattern_centers.push_back(Point2f(272,78.75));
	//pattern_centers.push_back(Point2f(226.75,76));
	if (pattern_centers.size() != 20) {
		return;
	}
	//cout << "==================" << endl;
	//for(int i=0;i<pattern_centers.size();i++){
	//    cout << "pattern_centers.push_back(Point2f(" <<pattern_centers[i].x << "," << pattern_centers[i].y << "));" <<endl;
	//}
	//cout << "==================" << endl;
	int p_r = 0;
	int p_l = 0;
	int p_t = 0;
	int p_b = 0;

	for (int i = 1; i < pattern_centers.size(); i++) {
		if (pattern_centers[i].x > pattern_centers[p_r].x) {
			p_r = i;
		}
		if (pattern_centers[i].x < pattern_centers[p_l].x) {
			p_l = i;
		}
		if (pattern_centers[i].y < pattern_centers[p_t].y) {
			p_t = i;
		}
		if (pattern_centers[i].y > pattern_centers[p_b].y) {
			p_b = i;
		}
	}
	// BGR Pallete
	circle(drawing, pattern_centers[p_t], 10, Scalar(0, 0, 255));
	//circle(drawing, pattern_centers[p_b], 10, Scalar(255, 255, 255));
	circle(drawing, pattern_centers[p_r], 10, Scalar(255, 0, 0));
	//circle(drawing, pattern_centers[p_l], 10, Scalar(0, 255, 0));

	//for (int i = 0; i < pattern_centers.size(); i++) {
	//    circle(drawing, pattern_centers[i], 5, Scalar(0, 255, 0));
	//}
	vector<Scalar> color_palette(5);
	color_palette[0] = Scalar(0, 0, 255);
	color_palette[1] = Scalar(0, 128, 128);
	color_palette[2] = Scalar(0, 255, 0);
	color_palette[3] = Scalar(128, 128, 0);
	color_palette[4] = Scalar(255, 0, 0);
	/*
	ax + by + c = 0
	y = -(c + ax )/ b
	a = y1-y2,
	b = x2-x1,
	c = (x1-x2)*y1 + (y2-y1)*x1.
	*/
	int coincidendes = 0;
	int centers = pattern_centers.size();
	float pattern_range = 10;
	float a, b, c;
	for (int i = 0; i < centers; i++) {
		a = pattern_centers[p_t].y - pattern_centers[p_r].y;
		b = pattern_centers[p_r].x - pattern_centers[p_t].x;
		c = (pattern_centers[p_t].x - pattern_centers[p_r].x) * pattern_centers[p_t].y + (pattern_centers[p_r].y - pattern_centers[p_r].y) * pattern_centers[p_t].x;
		if (abs(-(c + a * pattern_centers[i].x ) / b - pattern_centers[i].y ) < pattern_range) {
			coincidendes++;
		}
	}
	cout << "coincidendes between t y r " << coincidendes << endl;
	vector<Point2f> temp;
	Point2f t_last;

	if (coincidendes == 5) {
		for (int j = 0; j < 4; j++) {
			temp.clear();
			line(drawing, pattern_centers[p_t], pattern_centers[p_r], color_palette[j]);
			t_last = pattern_centers[p_r];
			cout << "========" << endl;
			cout << "Selected points" << endl;
			cout << pattern_centers[p_t].x << " , " << pattern_centers[p_t].y << endl;
			cout << pattern_centers[p_r].x << " , " << pattern_centers[p_r].y << endl;
			cout << "========" << endl;
			for (int i = 0; i < pattern_centers.size(); i++) {
				a = pattern_centers[p_t].y - pattern_centers[p_r].y;
				b = pattern_centers[p_r].x - pattern_centers[p_t].x;
				c = (pattern_centers[p_t].x - pattern_centers[p_r].x) * pattern_centers[p_t].y + (pattern_centers[p_r].y - pattern_centers[p_t].y) * pattern_centers[p_t].x;
				if (abs(-(c + a * pattern_centers[i].x ) / b - pattern_centers[i].y ) < pattern_range) {
					coincidendes++;
					cout << pattern_centers[i].x << " , " << pattern_centers[i].y << endl;
				} else {
					temp.push_back(pattern_centers[i]);
				}
			}
			cout << "========" << endl;
			pattern_centers = temp;
			p_t = 0;
			p_r = 0;
			for (int i = 1; i < pattern_centers.size(); i++) {
				if (pattern_centers[i].x > pattern_centers[p_r].x) {
					p_r = i;
				}
				if (pattern_centers[i].y < pattern_centers[p_t].y) {
					p_t = i;
				}
			}
			line(drawing, t_last, pattern_centers[p_t], color_palette[j]);
		}
	} else {
		//for (int j = 0; j < 4; j++) {
		//	temp.clear();
		//	line(drawing, pattern_centers[p_t], pattern_centers[p_l], color_palette[j]);
		//	t_last = pattern_centers[p_t];
		//	for (int i = 0; i < pattern_centers.size(); i++) {
		//		a = pattern_centers[p_t].y - pattern_centers[p_l].y;
		//		b = pattern_centers[p_l].x - pattern_centers[p_t].x;
		//		c = (pattern_centers[p_t].x - pattern_centers[p_l].x) * pattern_centers[p_t].y + (pattern_centers[p_l].y - pattern_centers[p_t].y) * pattern_centers[p_t].x;
		//		if (abs(-(c + a * pattern_centers[i].x ) / b - pattern_centers[i].y ) < pattern_range) {
		//			coincidendes++;
		//		} else {
		//			temp.push_back(pattern_centers[i]);
		//		}
		//	}
		//	pattern_centers = temp;
		//	p_t = 0;
		//	p_l = 0;
		//	for (int i = 1; i < pattern_centers.size(); i++) {
		//		if (pattern_centers[i].x < pattern_centers[p_l].x) {
		//			p_l = i;
		//		}
		//		if (pattern_centers[i].y < pattern_centers[p_t].y) {
		//			p_t = i;
		//		}
		//	}
		//	line(drawing, t_last, pattern_centers[p_l], color_palette[j]);
		//}
	}

}