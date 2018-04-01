#include <iostream>
#include <iomanip>
#include "ImagePreprocessing.h"
#include "PatternSearch.h"
#include "CalibrateCamera.h"
#include "CalibrationUtils.h"

using namespace cv;
using namespace std;

#define CALIBRATION_PS3_OWN_VIDEO "/home/alonzo/Documentos/Projects/CameraCalibration/calibration_videos/ellipses.avi"
#define CALIBRATION_PS3_VIDEO     "/home/alonzo/Documents/avance4_videos/ps3_rings.webm"
#define CALIBRATION_LIFECAM_VIDEO     "/home/alonzo/Documents/avance4_videos/lifecam_rings.webm"

#define REFINE_PROM       0
#define REFINE_BLEND      1
#define REFINE_VARICENTER 2

bool find_points_in_frame(Mat &frame, Mat &output, int w, int h, vector<PatternPoint> &pattern_points, bool debug);
bool find_points_in_frame(Mat &frame, int w, int h, vector<PatternPoint> &pattern_points, bool debug);
void refine_points_prom(vector<PatternPoint> &old_points, vector<Point2f>&new_points);
void refine_points_blend(vector<PatternPoint> &old_points, vector<Point2f>&new_points);
void refine_points_varicenter(vector<PatternPoint> &old_points, vector<Point2f>&new_points);

void refine_points(vector<PatternPoint> &old_points, vector<Point2f> &new_points, int type) {
    switch (type) {
    case REFINE_PROM:
        refine_points_prom(old_points, new_points);
        break;
    case REFINE_BLEND:
        refine_points_blend(old_points, new_points);
        break;
    case REFINE_VARICENTER:
        refine_points_varicenter(old_points, new_points);
        break;
    }
}

void refine_points_prom(vector<PatternPoint> &old_points, vector<Point2f> &new_points) {
    for (int p = 0; p < new_points.size(); p++) {
        new_points[p].x = old_points[p].x * 0.5 + new_points[p].x * 0.5;
        new_points[p].y = old_points[p].y * 0.5 + new_points[p].y * 0.5;
    }
}

void refine_points_blend(vector<PatternPoint> &old_points, vector<Point2f>&new_points) {
    vector<Point2f> points(5);
    Vec4f line;
    float factor;
    float d_old;
    float d_new;
    for (int row = 0; row < 4; row++) {
        for (int c = 0; c < 5; c++) {
            points[c] = new_points[row * 5 + c];
        }
        fitLine(points, line, CV_DIST_L2,  0, 0.01, 0.01);
        Point2f p1;
        Point2f p2;
        p1.x = line[2];
        p1.y = line[3];

        p2.x = p1.x + new_points[row * 5 + 4].x * line[0];
        p2.y = p1.y + new_points[row * 5 + 4].x * line[1];

        for (int c = 0; c < 5; c++) {
            d_old = distance_to_rect(p1, p2, old_points[row * 5 + c].to_point2f());
            d_new = distance_to_rect(p1, p2, new_points[row * 5 + c]);
            factor = d_old / (d_old + d_new);
            new_points[row * 5 + c].x = old_points[row * 5 + c].x * (1 - factor) + new_points[row * 5 + c].x * factor;
            new_points[row * 5 + c].y = old_points[row * 5 + c].y * (1 - factor) + new_points[row * 5 + c].y * factor;
        }
    }
}

void refine_points_varicenter(vector<PatternPoint> &old_points, vector<Point2f>&new_points) {
    // TO DO
}

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
bool select_frames_process(VideoCapture &cap, int w, int h, const int n_frames, vector<int> &frames, const int n_rows, const int n_columns, Mat &m_calibration, Mat &m_centroids, const Mat camera_matrix, const Mat dist_coeffs) {
    cap.set(CAP_PROP_POS_FRAMES, 1);
    int width  = h;
    int height = w;
    int blockSize_y = height / n_rows;
    int blockSize_x = width  / n_columns;
    int f = 0;
    Mat frame;
    Vec3d eulerAngles;

    int on_success_skip = 29;
    int on_overflow_skip = 9;

    int num_color_palette = 100;
    vector<Scalar> color_palette(num_color_palette);
    RNG rng(12345);
    for (int i = 0; i < num_color_palette; i++)
        color_palette[i] = Scalar(rng.uniform(0, 255), rng.uniform(0, 255), rng.uniform(0, 255));

    Size boardSize(5, 4);
    int squareSize = 45;
    int points = 20;
    vector<Point3f> objectPoints;
    for ( int i = 0; i < boardSize.height; i++ ) {
        for ( int j = 0; j < boardSize.width; j++ ) {
            objectPoints.push_back(Point3f(  float(j * squareSize),
                                             float(i * squareSize), 0));
        }
    }
    vector<Point2f> temp(20);

    int** quadBins = new int*[n_rows];
    for (int y_block = 0; y_block < n_rows; ++y_block) {
        quadBins[y_block] = new int[n_columns];
        for (int x_block = 0; x_block < n_columns; ++x_block) {
            quadBins[y_block][x_block] = 0;
        }
    }

    int selected_frames = 0;
    cout << "Start selection with " << selected_frames << " of " << n_frames << endl;
    int max_points = round((n_frames + 4.0) / ((n_rows) * (n_columns)));
    if (max_points * n_rows * n_columns <= n_frames) {
        max_points++;
    }

    while (selected_frames < n_frames) {
        vector<PatternPoint> pattern_points;

        if (!cap.read(frame)) {
            break;
        }
        if (find_points_in_frame(frame, frame, w, h, pattern_points, false)) {

            int x_block = floor((pattern_points[7].x + pattern_points[12].x) / 2.0 / blockSize_x);
            int y_block = floor((pattern_points[7].y + pattern_points[12].y) / 2.0 / blockSize_y);
            int c_x = (pattern_points[7].x + pattern_points[12].x) / 2.0 ;
            int c_y = (pattern_points[7].y + pattern_points[12].y) / 2.0 ;
            bool near_center = true;
            int block_radio = (blockSize_x + blockSize_y) / (2 * 3.0);
            PatternPoint block_center((x_block + 0.5)*blockSize_x, (y_block + 0.5)*blockSize_y);
            if (block_center.distance(PatternPoint(c_x, c_y)) > block_radio && quadBins[y_block][x_block] > 0) {
                near_center = false;
            }

            circle(m_centroids, block_center.to_point2f(), block_radio, Scalar(255, 0, 0));
            circle(m_centroids, Point2f(c_x, c_y), 1, Scalar(0, 0, 255));
            bool rejected = false;
            if (near_center) {
                if ( quadBins[y_block][x_block] < max_points) {
                    for (int i = 0; i < 20; i++) {
                        temp[i] = pattern_points[i].to_point2f();
                    }
                    if (!camera_matrix.empty() && !dist_coeffs.empty()) {
                        getEulerAngles(objectPoints, temp, camera_matrix, dist_coeffs, eulerAngles);
                        float yaw = eulerAngles[1];
                        float pitch = eulerAngles[0];
                        float roll = eulerAngles[2];
                        if (!(yaw > -20 && yaw < 20 && roll > -20 && roll < 20 && (pitch > 150 || pitch < -150))) {
                            rejected = true;
                        }
                    }
                    if (!rejected) {
                        circle(m_centroids, Point2f(c_x, c_y), 6, Scalar(0, 255, 0), -1);

                        frames.push_back(f + 1);
                        selected_frames++;
                        skip_frames(cap, on_success_skip);
                        f += on_success_skip;

                        quadBins[y_block][x_block] ++;
                        for (int j = 0; j < 20; j++) {
                            circle(m_calibration, temp[j], 10, color_palette[selected_frames]);
                        }
                    }
                }
                else {
                    skip_frames(cap, on_overflow_skip);
                    f += on_overflow_skip;
                }
            }

        }
        //draw rectangle
        int yOffsetText = 25;
        for (int y_block = 0; y_block < n_rows; ++y_block) {
            for (int x_block = 0; x_block < n_columns; ++x_block) {
                int local_n_y = height / n_rows;  //n of blocks
                int local_a_y = y_block * local_n_y;                 //lower bound
                int local_b_y = local_a_y + local_n_y;               //upper bound

                int local_n_x = width / n_columns;  //n of blocks
                int local_a_x = x_block * local_n_x;                 //lower bound
                int local_b_x = local_a_x + local_n_x;               //upper bound

                std::ostringstream quads_str;
                //quads_str << std::fixed << std::setprecision(2) << quadBins[y_block][x_block] / max_points * 100.0 << "%";
                quads_str << quadBins[y_block][x_block] << "/" << max_points ;
                rectangle(m_centroids, Point(local_a_x, local_a_y), Point(local_a_x + width * 0.1, local_a_y + yOffsetText), cvScalar(0, 0, 0), -1);
                putText(m_centroids, quads_str.str(), cvPoint(local_a_x, local_a_y + yOffsetText), FONT_HERSHEY_PLAIN, 2, cvScalar(0, 255, 255), 2, CV_AA);
                rectangle(m_centroids, Point(local_a_x, local_a_y), Point(local_b_x, local_b_y), cvScalar(255, 255, 255));
                rectangle(frame, Point(local_a_x, local_a_y), Point(local_b_x, local_b_y), cvScalar(255, 255, 255));
            }
        }
        rectangle(m_centroids, Point(width * 0.15, height - 35), Point(width * 0.85, height), cvScalar(0, 0, 0), -1);
        //total_str << "total: " << std::fixed << std::setprecision(2) << 100 * sumPercentage << "%   " << "s.d.: " << standarDeviation(percentage_values);
        std::ostringstream total_str;
        total_str << "Selected: " << selected_frames << "/" << n_frames;
        putText(m_centroids, total_str.str(), cvPoint(width * 0.15, height - 10), FONT_HERSHEY_PLAIN, 2, cvScalar(255, 255, 0), 2, CV_AA);
        imshow("CentersDistribution", m_centroids);
        imshow("CalibrationFrames", m_calibration);
        imshow("Undistort", frame);
        f++;
        //skip_frames(cap,1);
        waitKey(1);
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
bool select_frames(VideoCapture &cap, int w, int h, const int n_frames, vector<int> &frames, const int n_rows, const int n_columns, const Mat camera_matrix, const Mat dist_coeffs) {
    frames.clear();
    int rows = n_rows;
    int cols = n_columns;
    int select_frames = n_frames;
    Mat m_calibration = Mat::zeros(Size(h, w), CV_8UC3);
    Mat m_centroids = Mat::zeros(Size(h, w), CV_8UC3);
    while (!select_frames_process(cap, w, h, select_frames, frames, rows, cols, m_calibration, m_centroids, camera_matrix, dist_coeffs)) {
        rows --;
        cols --;
        select_frames = n_frames - frames.size();
        if (rows < 1 || cols < 1) {
            break;
        }
        m_calibration = Mat::zeros(Size(h, w), CV_8UC3);
    }
}

/**
 * @brief Run camera calibration with the positions in the set_points vector
 *
 * @param w             Width of the frame
 * @param h             Height of the frame
 * @param set_points    Set of points to perform the calibration
 * @param camera_matrix Camera matrix result of the calibration
 * @param dist_coeffs   Distortion coefficients result of the calibration
 */
void calibrate_camera(int w, int h, vector<vector<Point2f>> &set_points, Mat & camera_matrix, Mat & dist_coeffs) {
    Size imageSize(h, w);
    float rms = calibrate_with_points(imageSize, camera_matrix, dist_coeffs, set_points);
    double fx = camera_matrix.at<double>(0, 0);
    double fy = camera_matrix.at<double>(1, 1);
    double cx = camera_matrix.at<double>(0, 2);
    double cy = camera_matrix.at<double>(1, 2);
    cout << set_points.size() << "\t" << rms << "\t" << fx << "\t" << fy << "\t" << cx << "\t" << cy << "\t" << avgColinearDistance(set_points);
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
void undistort_image(Mat & frame, int w, int h, const Mat camera_matrix, const Mat dist_coeffs) {
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
 * @brief Search pattern points of choosen frames and save the points in the set_points array
 *
 * @param cap           VideoCapture reference
 * @param w             Width of the frame
 * @param h             Height of the frame
 * @param set_points    Set of points to use in the camera calibration
 */
void collect_points(VideoCapture & cap, int w, int h, const vector<int> &frames, vector<vector<Point2f>> &set_points) {
    set_points.clear();
    Mat frame;
    for (int f = 0; f < frames.size(); f++) {
        cap.set(CAP_PROP_POS_FRAMES, frames[f]);
        cap.read(frame);
        vector<PatternPoint> pattern_points;
        if (find_points_in_frame(frame, frame, w, h, pattern_points, false)) {
            vector<Point2f> temp(20);
            for (int i = 0; i < 20; i++) {
                temp[i] = pattern_points[i].to_point2f();
            }
            set_points.push_back(temp);
        }
    }
    frame.release();
}

/**
 * @brief Add the distortion to the points
 *
 * @param xy            Undistorted points
 * @param uv            Distorted points
 * @param camera_matrix Camera matrix
 * @param dist_coeffs   Distortion coefficients
 */
void distortPoints(const vector<Point2f> &xy, vector<Point2f> &uv, const Mat & camera_matrix, const Mat & dist_coeffs) {
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
void collect_points_fronto_parallel(VideoCapture & cap, int w, int h, const vector<int> &frames, vector<vector<Point2f>> &original_set_points, vector<vector<Point2f>> &set_points, const Mat & camera_matrix, const Mat & dist_coeffs, int refine_type) {
    Size imageSize(h, w);
    Size boardSize(5, 4);
    int n_points = 20;
    float desp_w = h * 0.2;
    float desp_h = w * 0.2;
    float squareSize = w / 5.5;
    Mat frame;
    Mat map1, map2;
    Mat input_undistorted;
    vector<Point3f> points_real;
    vector<Point2f> temp(n_points);
    vector<Point2f> temp2(n_points);
    vector<vector<Point2f>> start_set_points;
    vector<vector<Point2f>> new_set_points;
    vector<PatternPoint> points_undistorted;
    vector<PatternPoint> points_fronto_parallel;

    for ( int i = 0; i < boardSize.height; i++ ) {
        for ( int j = 0; j < boardSize.width; j++ ) {
            points_real.push_back(Point3f(  float(j * squareSize + desp_w) ,
                                            float( w - (i * squareSize + desp_h)), 0));
        }
    }

    set_points.clear();

    for (int f = 0; f < frames.size(); f++) {
        points_undistorted.clear();
        points_fronto_parallel.clear();
        cap.set(1, frames[f]);
        cap.read(frame);

        // ONLY FOR VISUALIZATION PURPOSES
        initUndistortRectifyMap(camera_matrix,
                                dist_coeffs,
                                Mat(),
                                getOptimalNewCameraMatrix(camera_matrix, dist_coeffs, imageSize, 1, imageSize, 0),
                                imageSize,
                                CV_16SC2,
                                map1,
                                map2);
        remap(frame, input_undistorted, map1, map2, INTER_LINEAR);
        map1.release();
        map2.release();
        imshow("Undistort", input_undistorted);

        undistort(frame, input_undistorted, camera_matrix, dist_coeffs);
        if (!find_points_in_frame(input_undistorted, w, h, points_undistorted, false)) {
            continue;
        }
        for (int i = 0; i < n_points; i++) {
            temp[i] = points_undistorted[i].to_point2f();
        }

        Mat homography = cv::findHomography(temp, points_real);
        Mat inv_homography = cv::findHomography(points_real, temp);
        Mat img_in = input_undistorted.clone();
        Mat img_out = input_undistorted.clone();
        cv::warpPerspective(img_in, img_out, homography, imageSize);
        for (int p = 0; p < n_points; p++) {
            circle(input_undistorted, points_undistorted[p].to_point2f(), 2, Scalar(0, 255, 0));
        }
        if (find_points_in_frame(img_out, img_out, w, h, points_fronto_parallel, false)) {
            for (int p = 0; p < n_points; p++) {
                circle(img_out, points_fronto_parallel[p].to_point2f(), 2, Scalar(0, 255, 0));
            }

            vector<Point2f> object_p_canonical;
            for (int p = 0; p < 20; p++) {
                object_p_canonical.push_back(points_fronto_parallel[p].to_point2f());
            }
            vector<Point2f> new_points2D(n_points);

            vector<Point2f> new_points2D_distort(n_points);
            //cout << "FParallel error " << avgColinearDistance(points_fronto_parallel) << endl;
            perspectiveTransform(object_p_canonical, new_points2D, inv_homography);
            for (int p = 0; p < n_points; p++) {
                circle(input_undistorted, new_points2D[p], 2, Scalar(0, 0, 255));
                circle(frame, new_points2D[p], 2, Scalar(0, 255, 0));
            }

            refine_points(points_undistorted, new_points2D, refine_type);

            distortPoints(new_points2D, new_points2D_distort, camera_matrix, dist_coeffs);
            for (int p = 0; p < n_points; p++) {
                circle(frame, original_set_points[f][p], 2, Scalar(0, 0, 255));
            }
            for (int p = 0; p < n_points; p++) {
                circle(frame, new_points2D_distort[p], 2, Scalar(255, 0, 0));
            }
            set_points.push_back(new_points2D_distort);

            for (int i = 0; i < n_points; i++) {
                temp2[i] = points_undistorted[i].to_point2f();
            }
            start_set_points.push_back(temp2);
            new_set_points.push_back(new_points2D);

            imshow("FrontoParallel", img_out);
            imshow("Reproject", input_undistorted);
            imshow("Distort", frame);
        }
        img_in.release();
        img_out.release();
        homography.release();
        inv_homography.release();
        waitKey(1);
    }
    cout << "\t" << avgColinearDistance(start_set_points);
    cout << "\t" << avgColinearDistance(new_set_points) << endl;
}

/**
 * @brief Find patter points in the frame
 *
 * @param frame             Video frame
 * @param output            Frame output with the pattern detection
 * @param w                 Width of the frame
 * @param h                 Height of the frame
 * @param pattern_points    Pattern points found in the frame
 * @param debug             If debug in enabled show the step by step process of detection
 * @return                  True if we found all the 20 points
 */
bool find_points_in_frame(Mat & frame, Mat & output, int w, int h, vector<PatternPoint> &pattern_points, bool debug) {
    //clean_using_mask(frame, w, h, mask_points);
    Mat masked, frame_gray, thresh;
    Point mask_points[1][4];
    masked = frame.clone();
    cvtColor( frame, frame_gray, CV_BGR2GRAY );
    adaptiveThreshold(frame_gray, thresh, 255, ADAPTIVE_THRESH_GAUSSIAN_C, THRESH_BINARY, 41, 12);
    segmentar(frame_gray, frame_gray, thresh, w, h);
    if (debug) imshow("FrameGray", frame_gray);
    int keep_per_frames = 2;
    int detected_points = find_pattern_points(frame_gray, masked, output, w, h, mask_points, pattern_points, keep_per_frames);
    if (debug) imshow("Masked", masked);
    if (debug) imshow("Original", output);
    masked.release();
    frame_gray.release();
    thresh.release();
    return detected_points == 20;
}

/**
 * @brief Find patter points in the frame
 *
 * @param frame             Video frame
 * @param w                 Width of the frame
 * @param h                 Height of the frame
 * @param pattern_points    Pattern points found in the frame
 * @param debug             If debug in enabled show the step by step process of detection
 * @return                  True if we found all the 20 points
 */
bool find_points_in_frame(Mat & frame, int w, int h, vector<PatternPoint> &pattern_points, bool debug) {
    Mat output = frame.clone();
    return find_points_in_frame(frame, output, w, h, pattern_points, debug);
}

/**
 * @brief Initialize windows names, sizes and positions.
 */
void window_setup() {
    int window_w = 360 * 1.25;
    int window_h = 240 * 1.25;
    //int window_w = 640;
    //int window_h = 480;
    int second_screen_offset = 0;//1360;
    string window_name;
    window_name = "CentersDistribution";
    namedWindow(window_name, WINDOW_NORMAL);
    resizeWindow(window_name, window_w, window_h);
    moveWindow(window_name, 0 + second_screen_offset, 0);

    window_name = "CalibrationFrames";
    namedWindow(window_name, WINDOW_NORMAL);
    resizeWindow(window_name, window_w, window_h);
    moveWindow(window_name, window_w + second_screen_offset, 0);

    window_name = "Undistort";
    namedWindow(window_name, WINDOW_NORMAL);
    resizeWindow(window_name, window_w, window_h);
    moveWindow(window_name, window_w * 2 + second_screen_offset, 0);

    window_name = "FrontoParallel";
    namedWindow(window_name, WINDOW_NORMAL);
    resizeWindow(window_name, window_w, window_h);
    moveWindow(window_name, 0 + second_screen_offset, window_h + 60);

    window_name = "Reproject";
    namedWindow(window_name, WINDOW_NORMAL);
    resizeWindow(window_name, window_w, window_h);
    moveWindow(window_name, window_w + second_screen_offset, window_h + 60);

    window_name = "Distort";
    namedWindow(window_name, WINDOW_NORMAL);
    resizeWindow(window_name, window_w, window_h);
    moveWindow(window_name, window_w * 2 + second_screen_offset, window_h + 60);
}

int main( int argc, char** argv ) {
    int num_color_palette = 100;
    vector<Scalar> color_palette(num_color_palette);
    RNG rng(12345);
    for (int i = 0; i < num_color_palette; i++) {
        color_palette[i] = Scalar(rng.uniform(0, 255), rng.uniform(0, 255), rng.uniform(0, 255));
    }
    //VideoCapture cap(CALIBRATION_LIFECAM_VIDEO);
    VideoCapture cap(CALIBRATION_PS3_VIDEO);

    if ( !cap.isOpened() ) {
        cout << "Cannot open the video file. \n";
        return -1;
    }
    Mat frame;
    Mat camera_matrix, dist_coeffs;
    cap.read(frame);
    int w = frame.rows;
    int h = frame.cols;
    // Lifecam
    int n_rows = 3;
    int n_columns = 4;
    // PS3
    //n_rows = 3;
    //n_columns = 3;
    vector<int> frames;
    vector<vector<Point2f>> set_points;
    vector<vector<Point2f>> original_set_points;
    int frames_to_select = 40;
    window_setup();

    // To find initial calibration
    select_frames(cap, w, h, frames_to_select, frames, n_rows, n_columns, camera_matrix, dist_coeffs);
    collect_points(cap, w, h, frames, original_set_points);
    calibrate_camera(w, h, original_set_points, camera_matrix, dist_coeffs);

    // Use initial calibration to reject frames with high rotation
    select_frames(cap, w, h, frames_to_select, frames, n_rows, n_columns, camera_matrix, dist_coeffs);
    collect_points(cap, w, h, frames, original_set_points);
    calibrate_camera(w, h, original_set_points, camera_matrix, dist_coeffs);

    int n_iterations = 5;
    for (int i = 0; i < n_iterations; i++) {
        collect_points_fronto_parallel(cap, w, h, frames, original_set_points, set_points, camera_matrix, dist_coeffs, REFINE_BLEND);
        calibrate_camera(w, h, set_points, camera_matrix, dist_coeffs);
    }
    cout << endl;
    waitKey(0);
    return 0;
}