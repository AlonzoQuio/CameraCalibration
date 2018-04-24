#include <iostream>
#include <iomanip>
#include <string>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "deltille/findSaddlesPoints.h"
#include "deltille/DetectorParams.h"

#include "ImagePreprocessing.h"

#define REFINE_AVG       0
#define REFINE_BLEND      1
#define REFINE_VARICENTER 2
#define REFINE_NONE 3
#define REFINE_FP 4
#define REFINE_FP_IDEAL 5
#define REFINE_FP_INTERSECTION 6

using namespace findSaddlesPoints;

class CameraCalibration {
public:
	vector<vector<Point2f>> set_points;
	vector<Mat> frames;
	vector<Point3f> object_points;
	vector<Point3f> object_points_image;
	VideoCapture cap;
	Mat camera_matrix;
	Mat dist_coeffs;
	int w;
	int h;
	Size image_size;
	CameraCalibration(VideoCapture &cap) {
		this -> cap = cap;
		if ( !cap.isOpened() ) {
			cout << "Cannot open the video file. \n";
		}
		Mat frame;
		cap >> frame;
		w = frame.rows;
		h = frame.cols;
		image_size = frame.size();
		frame.release();
		//object_points.push_back(vector<Point3f>());
	}
	virtual Point2f calculate_pattern_center(vector<Point2f> pattern_points) = 0;
	virtual void calibrate_camera() = 0;
	virtual void load_object_points(int cols, int rows) = 0;
	virtual bool find_points_in_frame(Mat &frame, vector<Point2f> &points) = 0;

	void collect_points_fronto_parallel(int refine_type, int refine_fronto_parallel_type);
	/**
	* @brief Skip f frames using a simple for,
	*
	* @param cap Videocapture reference
	* @param f number of frames to be skiped
	*/
	void skip_frames(VideoCapture &cap, int f) {
		Mat frame;
		for (int i = 0; i < f; i++) {
			cap >> frame;
		}
		frame.release();
	}
	/**
	* @brief Remove the distortion of the image, the result its only for visualization pourposes
	*
	* @param frame         Video frame
	* @param w             Width of the frame
	* @param h             Height of the frame
	* @param camera_matrix Camera matrix
	* @param dist_coeffs   Distortion coefficients
	*/
	void undistort_image(Mat & frame) {
		Mat rview, map1, map2;
		Size imageSize(h, w);
		initUndistortRectifyMap(camera_matrix,
		                        dist_coeffs,
		                        Mat(),
		                        getOptimalNewCameraMatrix(camera_matrix, dist_coeffs, imageSize, 1, imageSize, 0),
		                        imageSize,
		                        CV_16SC2,
		                        map1,
		                        map2);

		remap(frame, rview, map1, map2, INTER_LINEAR);
		rview.copyTo(frame);
		rview.release();
		map1.release();
		map2.release();
	}

	/**
	 * @brief Choose some frame to cover the screen area
	 *
	 * @param cap       Videocapture reference
	 * @param w         Width of the frame
	 * @param h         Height of the frame
	 * @param n_frames  Number of frames to be selected
	 * @param frames    Vector of frame positions selected
	 * @param n_rows    Number of rows to define quads areas
	 * @param n_columns Number of cols to define quads areas
	 * @return          True if was posible to found the required number of frames
	 */
	bool select_frames_process(const int n_frames, const int n_rows, const int n_columns, Mat &m_calibration, Mat &m_centroids) {
		cap.set(CAP_PROP_POS_FRAMES, 0);
		int width  = h;
		int height = w;
		int blockSize_y = height / n_rows;
		int blockSize_x = width  / n_columns;
		int f = 0;
		Mat frame, src_gray;
		int on_success_skip = 14;
		int on_overflow_skip = 4;

		int num_color_palette = 50;
		vector<Scalar> color_palette(num_color_palette);
		RNG rng(12345);
		for (int i = 0; i < num_color_palette; i++)
			color_palette[i] = Scalar(rng.uniform(0, 255), rng.uniform(0, 255), rng.uniform(0, 255));

		int** quadBins = new int*[n_rows];
		for (int y_block = 0; y_block < n_rows; ++y_block) {
			quadBins[y_block] = new int[n_columns];
			for (int x_block = 0; x_block < n_columns; ++x_block) {
				quadBins[y_block][x_block] = 0;
			}
		}

		int selected_frames = 0;
		cout << "Start selection with " << selected_frames << " of " << n_frames << endl;
		int max_points = round((n_frames) / ((n_rows) * (n_columns)));
		if (max_points * n_rows * n_columns <= n_frames && (n_rows == 1 || n_columns == 1)) {
			max_points++;
		}

		while (selected_frames < n_frames && max_points * n_rows * n_columns > selected_frames) {
			vector<Point2f> pattern_points;

			if (!cap.read(frame)) {
				cout << "FinishVideo at frame " << f << endl;
				break;//return false;
			}
			//cvtColor( frame, src_gray, CV_BGR2GRAY );
			if (find_points_in_frame(frame, pattern_points)) {

				Point2f c = calculate_pattern_center(pattern_points);
				int x_block = floor(c.x / blockSize_x);
				int y_block = floor(c.y / blockSize_y);
				bool near_center = true;
				int block_radio = (blockSize_x + blockSize_y) / (2 * 3.0);
				Point2f block_center((x_block + 0.5)*blockSize_x, (y_block + 0.5)*blockSize_y);
				if (distance(block_center, c) > block_radio * block_radio /*&& quadBins[y_block][x_block] > 0*/) {
					near_center = false;
					//cout << "To far " << distance(block_center, c) << " to " << block_radio << endl;
				}
				circle(m_centroids, block_center, block_radio, Scalar(255, 0, 0));
				circle(m_centroids, Point2f(c.x, c.y), 1, Scalar(0, 0, 255));
				if (near_center) {
					if ( quadBins[y_block][x_block] < max_points) {
						circle(m_centroids, c, 6, Scalar(0, 255, 0), -1);

						//frames.push_back(f + 1);
						frames.push_back(frame.clone());
						//imwrite("frames/frame" + to_string(frames.size() - 1) + ".png", frame);
						selected_frames++;
						skip_frames(cap, on_success_skip);
						f += on_success_skip;

						quadBins[y_block][x_block] ++;
						for (int j = 0; j < pattern_points.size(); j++) {
							circle(m_calibration, pattern_points[j], 8, color_palette[selected_frames]);
						}
					} else {
						skip_frames(cap, on_overflow_skip);
						f += on_overflow_skip;
					}
				}
				for (int p = 0; p < pattern_points.size(); p++) {
					putText(frame, to_string(p), pattern_points[p], FONT_HERSHEY_COMPLEX_SMALL, 0.5, cvScalar(0, 0, 255), 1);
				}

			}
			//draw rectangle
			int yOffsetText = 25;
			//std::vector<float> percentage_values;
			for (int y_block = 0; y_block < n_rows; ++y_block) {
				for (int x_block = 0; x_block < n_columns; ++x_block) {
					int local_n_y = height / n_rows;  //nro de bloques
					int local_a_y = y_block * local_n_y;                 //lower bound
					int local_b_y = local_a_y + local_n_y;               //upper bound

					int local_n_x = width / n_columns;  //nro de bloques
					int local_a_x = x_block * local_n_x;                 //lower bound
					int local_b_x = local_a_x + local_n_x;               //upper bound

					std::ostringstream quads_str;
					quads_str << quadBins[y_block][x_block] << "/" << max_points ;
					rectangle(m_centroids, Point(local_a_x, local_a_y), Point(local_a_x + width * 0.1, local_a_y + yOffsetText), cvScalar(0, 0, 0), -1);
					putText(m_centroids, quads_str.str(), cvPoint(local_a_x, local_a_y + yOffsetText), FONT_HERSHEY_COMPLEX_SMALL, 1, cvScalar(0, 255, 255), 1, CV_AA);
					rectangle(m_centroids, Point(local_a_x, local_a_y), Point(local_b_x, local_b_y), cvScalar(255, 255, 255));
					rectangle(frame, Point(local_a_x, local_a_y), Point(local_b_x, local_b_y), cvScalar(255, 255, 255));
				}
			}
			rectangle(m_centroids, Point(width * 0.15, height - 35), Point(width * 0.85, height), cvScalar(0, 0, 0), -1);
			std::ostringstream total_str;
			total_str << "Selected: " << selected_frames << "/" << n_frames;
			putText(m_centroids, total_str.str(), cvPoint(width * 0.15, height - 10), FONT_HERSHEY_COMPLEX_SMALL, 1, cvScalar(255, 255, 0), 1, CV_AA);
			imshow("CentersDistribution", m_centroids);
			imshow("CalibrationFrames", m_calibration);
			// imshow("InputFrame+Grid", frame);
			imshow("Undistort", frame);
			//imshow("InputFrame+Grid+Gray", src_gray);
			f++;
			char t = waitKey(1);
			if (t == 'q') {
				exit(0);
			}
		}
		cout << "Finish selection with " << selected_frames << " of " << n_frames << endl;
		return selected_frames == n_frames;
	}
	/**
	* @brief Run the selection frames process
	*
	* @param cap       Videocapture reference
	* @param w         Width of the frame
	* @param h         Height of the frame
	* @param n_frames  Number of frames to be selected
	* @param frames    Vector of frame positions selected
	* @param n_rows    Number of rows to define quads areas
	* @param n_columns Number of cols to define quads areas
	* @return          True if was posible to found the required number of frames
	*/
	bool select_frames(const int n_frames, const int n_rows, const int n_columns) {
		int rows = n_rows;
		int cols = n_columns;
		int select_frames = n_frames;
		Mat m_calibration = Mat::zeros(Size(h, w), CV_8UC3);
		Mat m_centroids = Mat::zeros(Size(h, w), CV_8UC3);
		while (!select_frames_process(select_frames, rows, cols, m_calibration, m_centroids)) {
			rows --;
			cols --;
			select_frames = n_frames - frames.size();
			if (rows < 1 || cols < 1) {
				break;
			}
			m_centroids = Mat::zeros(Size(h, w), CV_8UC3);
		}
	}
	/**
	* @brief Search pattern points of choosen frames and save the points in the set_points array
	*
	* @param cap           VideoCapture reference
	* @param w             Width of the frame
	* @param h             Height of the frame
	* @param set_points    Set of points to use in the camera calibration
	*/
	void collect_points() {
		set_points.clear();
		for (int f = 0; f < frames.size(); f++) {
			vector<Point2f> pattern_points;
			if (find_points_in_frame(frames[f], pattern_points)) {
				set_points.push_back(pattern_points);
			}
		}
	}
	void calibrate_camera_iterative(int n_iterations, int n_frames, int grid_rows, int grid_cols) {
		Mat frontoParallel = Mat::zeros(Size(h, w), CV_8UC3);
		for (int i = 0; i < object_points_image.size(); i++) {
			circle(frontoParallel, Point2f(object_points_image[i].x, object_points_image[i].y), 10, Scalar(255, 0, 0));
		}
		imshow("FrontoParallel", frontoParallel);
		//waitKey(0);
		select_frames(n_frames, grid_rows, grid_cols);
		collect_points();
		calibrate_camera();
		for (int i = 0; i < n_iterations; i++) {
			//collect_points_fronto_parallel(REFINE_VARICENTER, REFINE_FP_INTERSECTION);
			collect_points_fronto_parallel(REFINE_NONE, REFINE_FP_IDEAL);
			calibrate_camera();
		}
	}
	/**
	* @brief Add the distortion to the points
	*
	* @param xy            Undistorted points
	* @param uv            Distorted points
	* @param camera_matrix Camera matrix
	* @param dist_coeffs   Distortion coefficients
	*/
	void distort_points(const vector<Point2f> &xy, vector<Point2f> &uv) {
		double fx = camera_matrix.at<double>(0, 0);
		double fy = camera_matrix.at<double>(1, 1);
		double cx = camera_matrix.at<double>(0, 2);
		double cy = camera_matrix.at<double>(1, 2);
		double k1 = dist_coeffs.at<double>(0, 0);
		double k2 = dist_coeffs.at<double>(0, 1);
		double p1 = dist_coeffs.at<double>(0, 2);
		double p2 = dist_coeffs.at<double>(0, 3);
		double k3 = dist_coeffs.at<double>(0, 4);

		double x;
		double y;
		double r2;
		double xDistort;
		double yDistort;
		for (int p = 0; p < xy.size(); p++) {
			x = (xy[p].x - cx) / fx;
			y = (xy[p].y - cy) / fy;
			r2 = x * x + y * y;

			// Radial distorsion
			xDistort = x * (1 + k1 * r2 + k2 * pow(r2, 2) + k3 * pow(r2, 3));
			yDistort = y * (1 + k1 * r2 + k2 * pow(r2, 2) + k3 * pow(r2, 3));

			// Tangential distorsion
			xDistort = xDistort + (2 * p1 * x * y + p2 * (r2 + 2 * x * x));
			yDistort = yDistort + (p1 * (r2 + 2 * y * y) + 2 * p2 * x * y);

			// Back to absolute coordinates.
			xDistort = xDistort * fx + cx;
			yDistort = yDistort * fy + cy;
			uv[p] = Point2f(xDistort, yDistort);
		}
	}
};

bool find_points_in_frame_FP(Mat &frame, vector<Point2f> &points) {
		Mat frame_gray, thresh;
		cvtColor( frame, frame_gray, CV_BGR2GRAY );

			detector_params.half_kernel_size = 15;
			detector_params.hessian_factor_threshold = 0.0;
		bool result = findSaddleCenters(frame_gray, points, frame, true);
			detector_params.half_kernel_size = 3;
			detector_params.hessian_factor_threshold = 0.35;

		frame_gray.release();
		thresh.release();
		return result;
	}


/**
 * @brief Search pattern points in the undistorted image, find a homography
 * to get a cannonical view, find patter points in the cannonical view and
 * refine points and save the points in the set_points array
 * @details [long description]
 *
 * @param cap           VideoCapture reference
 * @param w             Width of the frame
 * @param h             Height of the frame
 * @param set_points    Set of points to use in the camera calibration
 * @param camera_matrix Camera matrix
 * @param dist_coeffs   Distortion coefficients
 * @param refine_type   Tipe of refinement in every iteration
 */
void CameraCalibration::collect_points_fronto_parallel(int refine_type, int refine_fronto_parallel_type) {
	Size imageSize(h, w);
	Size boardSize(8, 5);
	int n_points = 42;
	Mat frame;
	Mat map1, map2;
	Mat input_undistorted;
	//vector<Point3f> points_real;
	vector<Point2f> temp(n_points);
	vector<Point2f> temp2(n_points);
	vector<vector<Point2f>> start_set_points;
	vector<vector<Point2f>> new_set_points;
	vector<Point2f> points_undistorted;
	vector<Point2f> points_fronto_parallel;

	set_points.clear();

	for (int f = 0; f < frames.size(); f++) {
		points_undistorted.clear();
		points_fronto_parallel.clear();
		frames[f].copyTo(frame);
		frame.copyTo(input_undistorted);
		undistort_image(input_undistorted);
		imshow("Undistort", input_undistorted);

		undistort(frame, input_undistorted, camera_matrix, dist_coeffs);
		// imshow("UndistortInput", input_undistorted);
		if (!find_points_in_frame(input_undistorted, points_undistorted)) {
			cout << "Not found in undistort" << endl;
			continue;
		} else {
			cout << "found" << endl;
		}

		Mat homography = cv::findHomography(points_undistorted, object_points_image);
		Mat inv_homography = cv::findHomography(object_points_image, points_undistorted);
		Mat img_in  = input_undistorted.clone();
		Mat img_out = input_undistorted.clone();
		cv::warpPerspective(img_in, img_out, homography, imageSize);
		// imshow("img_in", img_in);
		// imshow("img_out", img_out);
		
		// imwrite("frontoParallel/fp_" + to_string(f)+".png", img_out);

		for (int p = 0; p < n_points; p++) 
			circle(input_undistorted, points_undistorted[p], 2, Scalar(0, 255, 0));

		// HERE
		// resize(img_out, img_in, cv::Size(), 0.25, 0.25);
		// img_out = img_in;

		//adaptiveThreshold(img_in,img_in,255,ADAPTIVE_THRESH_GAUSSIAN_C,THRESH_BINARY,11,2);
		if (find_points_in_frame_FP(img_out, points_fronto_parallel)) {
			cout << "Found in frontoParallel " << endl;
			for (int p = 0; p < n_points; p++) {
				circle(img_out, points_fronto_parallel[p], 2, Scalar(0, 255, 0));
			}
			
			vector<Point2f> object_p_canonical;
			if (refine_fronto_parallel_type == REFINE_FP_IDEAL) {
				for (int p = 0; p < n_points; p++) {
					object_p_canonical.push_back(Point2f((points_fronto_parallel[p].x + object_points_image[p].x) / 2.0 ,
					                                     (points_fronto_parallel[p].y + object_points_image[p].y) / 2.0 ));
				}
			} else if (refine_fronto_parallel_type == REFINE_FP_INTERSECTION) {
				for (int p = 0; p < n_points; p++) {
					object_p_canonical.push_back(points_fronto_parallel[p]);
				}
				//refine_points_intersection(object_p_canonical);
			} else {
				for (int p = 0; p < n_points; p++) {
					object_p_canonical.push_back(points_fronto_parallel[p]);
				}
			}

			vector<Point2f> new_points2D(n_points);
			perspectiveTransform(object_p_canonical, new_points2D, inv_homography);			
			//for (int p = 0; p < n_points; p++) {
			//	circle(input_undistorted, new_points2D[p], 2, Scalar(0, 0, 255));
			//	circle(frame, new_points2D[p], 2, Scalar(0, 255, 0));
			//}

			//refine_points(points_undistorted, new_points2D, refine_type);

			vector<Point2f> new_points2D_distort(n_points);
			distort_points(new_points2D, new_points2D_distort);

			/*for (int p = 0; p < n_points; p++) {
				circle(frame, original_set_points[f][p], 2, Scalar(0, 0, 255));
			}*/
			//for (int p = 0; p < n_points; p++) {
			//	circle(frame, new_points2D_distort[p], 2, Scalar(255, 0, 0));
			//}
			set_points.push_back(new_points2D_distort);
			//start_set_points.push_back(points_undistorted);
			//new_set_points.push_back(new_points2D);
			imshow("FrontoParallel", img_out);
			imshow("Reproject", input_undistorted);
			imshow("Distort", frame);
		} else {
			cout << "Not found in FP" << endl;
		}
		img_in.release();
		img_out.release();
		homography.release();
		inv_homography.release();
		waitKey(1);
	}
	//cout << "\t" << avgColinearDistance(start_set_points);
	//cout << "\t" << avgColinearDistance(new_set_points) << endl;
}

class CameraCalibrationDeltille: public CameraCalibration {
public:
	CameraCalibrationDeltille(): CameraCalibration(cap) {}
	CameraCalibrationDeltille(VideoCapture &cap, int cols, int rows): CameraCalibration(cap) {
		load_object_points(cols, rows);
	}
	Point2f calculate_pattern_center(vector<Point2f> pattern_points) {
		return Point2f( (pattern_points[20].x + pattern_points[21].x) / 2.0,
		                (pattern_points[20].y + pattern_points[21].y) / 2.0);
	}

	bool find_points_in_frame(Mat &frame, vector<Point2f> &points) {
		Mat frame_gray, thresh;
		cvtColor( frame, frame_gray, CV_BGR2GRAY );
		bool result = findSaddleCenters(frame_gray, points, frame);
		frame_gray.release();
		thresh.release();
		return result;
	}

	void load_object_points(int cols, int rows) {
		Size boardSize(cols, rows);
		int squareSize = 20;
		//int squareSize_2 = 20;
		//int squareSize = 21;
		int squareSize_2 = 18;

		object_points.clear();
		object_points_image.clear();
		//cout << "Object points " << endl;
		for ( int i = 0; i < boardSize.height; i++ ) {
			if (i % 2) {
				for ( int j = 0; j < boardSize.width + 1; j++ ) {
					object_points.push_back(Point3f( j * squareSize , float(i * squareSize_2), 0));
					object_points_image.push_back(Point3f(j * 64.0 + 64 , float(480 - (i * 64.0 + 96.0)), 0));
				}
			} else {
				for ( int j = 0; j < boardSize.width; j++ ) {
					object_points.push_back(Point3f((2 * j + 1)*squareSize / 2.0, float(i * squareSize_2), 0));
					object_points_image.push_back(Point3f((2 * j + 1) / 2.0 * 64.0 + 64 , float(480 - (i * 64.0 + 96.0)), 0));
				}
			}
		}

	}

	void calibrate_camera() {
		/*cout << "Running calibration " << endl;
		vector<Mat> tvecs;
		vector<Mat> rvecs;
		for (int o = 0; o < object_points.size(); o++) {
			cout << o << " " << object_points[o].x << " " << object_points[o].y << endl;
		}
		vector<vector<Point3f>> set_object_points(1);
		cout << "Resize" << endl;
		set_object_points.resize(set_points.size(), object_points);
		cout << "CalibrateCamera" << endl;
		double rms = calibrateCamera(set_object_points, set_points, image_size, camera_matrix,
		                             dist_coeffs, rvecs, tvecs);
		cout << camera_matrix << endl;
		cout << dist_coeffs << endl;
		cout << "rms " << rms << endl;*/
		Size boardSize(8, 5);
		vector<Mat> tvecs;
		vector<Mat> rvecs;
		//int squareSize = 21;
		//int squareSize_2 = 18;
		int squareSize = 20;
		int squareSize_2 = 20;
		vector<vector<Point3f> > objectPoints(1);

		objectPoints[0].clear();
		cout << "Object points " << endl;
		for ( int i = 0; i < boardSize.height; i++ ) {
			if (i % 2) {
				for ( int j = 0; j < boardSize.width + 1; j++ ) {
					objectPoints[0].push_back(Point3f( j * squareSize , float(i * squareSize_2), 0));
				}
			} else {
				for ( int j = 0; j < boardSize.width; j++ ) {
					objectPoints[0].push_back(Point3f((2 * j + 1)*squareSize / 2.0, float(i * squareSize_2), 0));
				}
			}
		}

		cout << "set_points size: "<<set_points.size();
		objectPoints.resize(set_points.size(), object_points);

		double rms = calibrateCamera(objectPoints, set_points, image_size, camera_matrix,
		                             dist_coeffs, rvecs, tvecs);
		cout << camera_matrix << endl;
		cout << dist_coeffs   << endl;
		cout << "rms " << rms << endl;
	}
	void test1(){

		for (int i = 0; i < 39; ++i){

			Mat img = imread("frontoParallel/fp_"+ to_string(i) + ".png", 1);

			vector<Point2f> points_fronto_parallel;

			if (find_points_in_frame_FP(img, points_fronto_parallel)) {
				cout << "Found in frontoParallel " << endl;
				for (int p = 0; p < points_fronto_parallel.size(); p++) 
					circle(img, points_fronto_parallel[p], 2, Scalar(0, 255, 0));
			} else
				cout << "Not found in FP" << endl;

			imshow("FrontoParallel2", img);

			if (waitKey(0) == 27)
				break;
		}
	}

};