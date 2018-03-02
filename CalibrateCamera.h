#include <iostream>
#include "opencv2/calib3d.hpp"

using namespace std;
using namespace cv;

static double computeReprojectionErrors(
    const vector<vector<Point3f> >& objectPoints,
    const vector<vector<Point2f> >& imagePoints,
    const vector<Mat>& rvecs, const vector<Mat>& tvecs,
    const Mat& cameraMatrix, const Mat& distCoeffs,
    vector<float>& perViewErrors )
{
	vector<Point2f> imagePoints2;
	int i, totalPoints = 0;
	double totalErr = 0, err;
	perViewErrors.resize(objectPoints.size());

	for ( i = 0; i < (int)objectPoints.size(); i++ )
	{
		projectPoints(Mat(objectPoints[i]), rvecs[i], tvecs[i],
		              cameraMatrix, distCoeffs, imagePoints2);
		err = norm(Mat(imagePoints[i]), Mat(imagePoints2), NORM_L2);
		int n = (int)objectPoints[i].size();
		perViewErrors[i] = (float)std::sqrt(err * err / n);
		totalErr += err * err;
		totalPoints += n;
	}

	return std::sqrt(totalErr / totalPoints);
}

float calibrate_with_points(Size &imageSize, Mat &cameraMatrix, Mat &distCoeffs, vector<vector<Point2f>> &imagePoints) {
	//vector<vector<Point2f> > imagePoints;
	
	Size boardSize(5, 4);
	//Size boardSize(4,5);
	//Pattern patternType;
	//float squareSize = 30;
	float squareSize = 45;
	float aspectRatio = 1;
	//int flags;
	//Mat cameraMatrix;
	//Mat distCoeffs;
	vector<Mat> rvecs;
	vector<Mat> tvecs;
	vector<float> reprojErrs;

	cout << "Calibrating camera" << endl;
	cameraMatrix = Mat::eye(3, 3, CV_64F);
	//if( flags & CALIB_FIX_ASPECT_RATIO )
	//    cameraMatrix.at<double>(0,0) = aspectRatio;

	distCoeffs = Mat::zeros(8, 1, CV_64F);

	vector<vector<Point3f> > objectPoints(1);

	objectPoints[0].resize(0);
	for ( int i = 0; i < boardSize.height; i++ ) {
		for ( int j = 0; j < boardSize.width; j++ ) {
			objectPoints[0].push_back(Point3f(  float(j * squareSize),
			                                    float(i * squareSize), 0));
		}
	}
	//calcChessboardCorners(boardSize, squareSize, objectPoints[0], patternType);

	objectPoints.resize(imagePoints.size(), objectPoints[0]);

	double rms = calibrateCamera(objectPoints,
	                             imagePoints,
	                             imageSize,
	                             cameraMatrix,
	                             distCoeffs,
	                             rvecs,
	                             tvecs,
	                             CV_CALIB_FIX_K4 | CV_CALIB_FIX_K5);
	cout << "rms " << rms << endl;
	double totalAvgErr = computeReprojectionErrors(objectPoints, imagePoints,
	                     rvecs, tvecs, cameraMatrix, distCoeffs, reprojErrs);
	cout << "totalAvgErr " << totalAvgErr << endl;
	return rms;
}