
#include "opencv2/core/opengl.hpp"
#include "opencv2/highgui.hpp"
void getEulerAngles(Mat &rotCamerMatrix, Vec3d &eulerAngles) {

    Mat cameraMatrix, rotMatrix, transVect, rotMatrixX, rotMatrixY, rotMatrixZ;
    double* _r = rotCamerMatrix.ptr<double>();
    double projMatrix[12] = {_r[0], _r[1], _r[2], 0,
                             _r[3], _r[4], _r[5], 0,
                             _r[6], _r[7], _r[8], 0
                            };

    decomposeProjectionMatrix( Mat(3, 4, CV_64FC1, projMatrix),
                               cameraMatrix,
                               rotMatrix,
                               transVect,
                               rotMatrixX,
                               rotMatrixY,
                               rotMatrixZ,
                               eulerAngles);
    cameraMatrix.release();
    rotMatrix.release();
    transVect.release();
    rotMatrixX.release();
    rotMatrixY.release();
    rotMatrixZ.release();
}
void getEulerAngles(vector<Point3f>objectPoints, vector<Point2f> points, Mat camera_matrix, Mat distortion_coeffs, Vec3d &eulerAngles) {
    Mat rvec(3, 1, DataType<double>::type);
    Mat tvec(3, 1, DataType<double>::type);
    Mat rotation;

    solvePnP(Mat(objectPoints), Mat(points), camera_matrix, distortion_coeffs, rvec, tvec);
    Rodrigues(rvec, rotation);
    getEulerAngles(rotation, eulerAngles);
}