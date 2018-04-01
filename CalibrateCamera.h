#include <iostream>
#include "opencv2/calib3d.hpp"

using namespace std;
using namespace cv;

float calibrate_with_points(Size &imageSize, Mat &cameraMatrix, Mat &distCoeffs, vector<vector<Point2f>> &imagePoints) {

	Size boardSize(5, 4);
	float squareSize = 44.3;
	float aspectRatio = 1;
	vector<Mat> rvecs;
	vector<Mat> tvecs;
	vector<float> reprojErrs;
	vector<vector<Point3f> > objectPoints(1);

	distCoeffs = Mat::zeros(8, 1, CV_64F);

	objectPoints[0].resize(0);
	for ( int i = 0; i < boardSize.height; i++ ) {
		for ( int j = 0; j < boardSize.width; j++ ) {
			objectPoints[0].push_back(Point3f(  float(j * squareSize),
			                                    float(i * squareSize), 0));
		}
	}

	objectPoints.resize(imagePoints.size(), objectPoints[0]);

	double rms = calibrateCamera(objectPoints,
	                             imagePoints,
	                             imageSize,
	                             cameraMatrix,
	                             distCoeffs,
	                             rvecs,
	                             tvecs/*,
	                             CV_CALIB_ZERO_TANGENT_DIST*/);
	/*cout << "rvecs" << endl;
	for (int r = 0; r < imagePoints.size(); r++) {
		cout << "rvecs " << r << endl;
		cout << rvecs[r] << endl;
	}
	cout << "tvecs" << endl;
	for (int r = 0; r < imagePoints.size(); r++) {
		cout << "tvecs " << r << endl;
		cout << tvecs[r] << endl;
	}*/
	return rms;
}