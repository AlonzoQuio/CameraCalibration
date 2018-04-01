#include <iostream>
#include <GL/glut.h>

#include "opencv2/core/opengl.hpp"
#include "opencv2/highgui.hpp"

#include "ImagePreprocessing.h"
#include "PatternSearch.h"
#include "CalibrateCamera.h"
#include "libs/OBJ_Loader.h"

using namespace std;
using namespace cv;

const int win_width = 640;
const int win_height = 480;

//#define CALIBRATION_PS3_VIDEO "out.mp4"
#define CALIBRATION_PS3_VIDEO "/home/alonzo/Documents/PS3_rings.mp4"
//#define CALIBRATION_PS3_VIDEO "/home/alonzo/Documents/ellipses.mp4"
#define MODEL_FILE_PATH "obj/pattern.obj"
//#define MODEL_FILE_PATH "obj/landscape.obj"
//#define MODEL_FILE_PATH "obj/igloo2.obj"
//#define MODEL_FILE_PATH "obj/waylow_scarlett_2_5_6.obj"

struct DrawData {
    ogl::Arrays arr;
    ogl::Texture2D tex;
    ogl::Buffer indices;
};

Mat camera_matrix;
Mat distortion_coeffs;
vector<PatternPoint> pattern_points;
VideoCapture cap(CALIBRATION_PS3_VIDEO);

vector<GLint> obj_model;

void draw(void* userdata) {
    DrawData* data = static_cast<DrawData*>(userdata);
    ogl::render(data->arr, data->indices, ogl::TRIANGLES);
}

Mat frame, original, frame_gray, thresh, masked;
int detected_points;
Mat rvec(3, 1, DataType<double>::type);
Mat tvec(3, 1, DataType<double>::type);

int w;
int h;
int keep_per_frames = 2;
Point mask_points[1][4];
Mat pattern = imread("pattern.png");

void draw_cube() {
    glBegin(GL_LINES);
    // front
    glColor3f(0.0f, 1.0f, 0.0f);
    glVertex3f( 1.33, 0.0, -1.0);
    glVertex3f( 1.33, 1.0, -1.0);
    glVertex3f( 1.33, 1.0, -1.0);
    glVertex3f(  0.0, 1.0, -1.0);
    glVertex3f(  0.0, 1.0, -1.0);
    glVertex3f(  0.0, 0.0, -1.0);
    glVertex3f(  0.0, 0.0, -1.0);
    glVertex3f( 1.33, 0.0, -1.0);

    // back
    glColor3f(0.0f, 1.0f, 0.0f);
    glVertex3f( 1.33, 0.0, 1.0);
    glVertex3f( 1.33, 1.0, 1.0);
    glVertex3f( 1.33, 1.0, 1.0);
    glVertex3f(  0.0, 1.0, 1.0);
    glVertex3f(  0.0, 1.0, 1.0);
    glVertex3f(  0.0, 0.0, 1.0);
    glVertex3f(  0.0, 0.0, 1.0);
    glVertex3f( 1.33, 0.0, 1.0);

    // top
    glColor3f(1.0f, 0.0f, 0.0f);
    glVertex3f(  0.0, 1.0, -1.0);
    glVertex3f(  0.0, 1.0, 1.0);
    glVertex3f( 1.33, 1.0, -1.0);
    glVertex3f( 1.33, 1.0, 1.0);

    // bottom
    glColor3f(0.0f, 0.0f, 1.0f);
    glVertex3f(  0.0, 0.0, -1.0);
    glVertex3f(  0.0, 0.0, 1.0);
    glVertex3f( 1.33, 0.0, -1.0);
    glVertex3f( 1.33, 0.0, 1.0);

    glEnd();

}
Mat rview, map1, map2;
Size boardSize(5, 4);
int squareSize = 45;
int points = 20;
Mat rotation;
Vec3d eulerAngles;
Vec3d eulerAnglesTranform;
Mat m_homography;
DrawData data;

void display(void) {
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(45.0, (double)win_width / win_height, 0.1, 100.0);

    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();

    gluLookAt(1.5, 1.5, 2.5,
              1.33 / 2, 0.5, 0,
              0, 1, 0);
    glLineWidth(5);
    draw_cube();

    cap >> frame ;
    w = frame.rows;
    h = frame.cols;
    Size imageSize(h, w);

    initUndistortRectifyMap(camera_matrix,
                            distortion_coeffs,
                            Mat(),
                            getOptimalNewCameraMatrix(camera_matrix, distortion_coeffs, imageSize, 1, imageSize, 0),
                            imageSize,
                            CV_16SC2,
                            map1,
                            map2);

    remap(frame, rview, map1, map2, INTER_LINEAR);
    map1.release();
    map2.release();
    frame.release();
    frame = rview;

    original = frame.clone();
    clean_using_mask(frame, w, h, mask_points);
    masked = frame.clone();
    cvtColor( frame, frame_gray, CV_BGR2GRAY );
    adaptiveThreshold(frame_gray, thresh, 255, ADAPTIVE_THRESH_GAUSSIAN_C, THRESH_BINARY, 41, 12);
    segmentar(frame_gray, frame_gray, thresh, w, h);
    detected_points = find_pattern_points(frame_gray, masked, original, w, h, mask_points, pattern_points, keep_per_frames);
    if (detected_points == 20) {
        vector<Point3f> objectPoints;
        for ( int i = 0; i < boardSize.height; i++ ) {
            for ( int j = 0; j < boardSize.width; j++ ) {
                objectPoints.push_back(Point3f(  float(j * squareSize),
                                                 float(i * squareSize), 0));
            }
        }
        vector<Point2f> temp(points);
        for (int i = 0; i < 20; i++) {
            temp[i] = pattern_points[i].to_point2f();
        }
        solvePnP(Mat(objectPoints), Mat(temp), camera_matrix, distortion_coeffs, rvec, tvec);

        Rodrigues(rvec, rotation);
        getEulerAngles(rotation, eulerAngles);

        float yaw = eulerAngles[1];
        float pitch = eulerAngles[0];
        float roll = eulerAngles[2];
        if (pitch > 0) {
            pitch = 180.0 - pitch;
        } else if (pitch < 0) {
            pitch = -180.0 - pitch;
        }
        yaw = -yaw;

        float area = pattern_points[0].area(pattern_points[4], pattern_points[15]);
        float scale = area / (w * h);
        scale = pattern_points[0].distance(pattern_points[4]) / h;

        glTranslatef(pattern_points[0].x * 1.33 / 640, 1 - pattern_points[0].y * 1.0 / 480,1 -(0.5 - scale));
        glColor3f(1, 0, 1);
        glRotatef(-roll, 0, 0, 1);
        glRotatef(-pitch, 1, 0, 0);
        glRotatef( yaw, 0, 1, 0);

        glEnable(GL_TEXTURE_2D);
        draw(&data);
        glDisable(GL_TEXTURE_2D);

    }

    glutSwapBuffers();
    namedWindow("window", WINDOW_OPENGL);
    resizeWindow("window", 640, 480);
    moveWindow("window", 640, 0);
    imshow("window", original);
    glutPostRedisplay();
    frame.release();
    original.release();
    masked.release();
    thresh.release();
}

void init(void) {
    glClearColor( 0, 1, 1, 1);
    /*camera_matrix = (Mat_<double>(3, 3) << 619.8529149086295, 0, 317.8602231908566,
                        0, 623.7464457495411, 257.8088409771084,
                        0, 0, 1);

    distortion_coeffs = (Mat_<double>(1, 5) << -0.2940861347010496,
                         0.1480524184491786,
                         -0.003080858295605339,
                         0.005568077204929082,
                         -0.07530280722975796);*/
    
    camera_matrix = (Mat_<double>(3, 3) << 842.277648121042, 0, 306.8028364233875,
                        0, 845.9871738025694, 258.6145767605929,
                        0, 0, 1);

    distortion_coeffs = (Mat_<double>(1, 5) << -0.3663343352388944,
                         0.2236100121702693,
                         -0.001952345044331583,
                         0.004004688327188062,
                         -0.2475043367750629);
    objl::Loader Loader;

    bool loadout = Loader.LoadFile(MODEL_FILE_PATH);
    int num;
    for (int i = 0; i < Loader.LoadedMeshes.size(); i++) {
        num = glGenLists(1);
        obj_model.push_back(num);
        glNewList(num, GL_COMPILE);
        objl::Mesh curMesh = Loader.LoadedMeshes[i];
        glColor3f(curMesh.MeshMaterial.Kd.X , curMesh.MeshMaterial.Kd.Y , curMesh.MeshMaterial.Kd.Z);
        glBegin(GL_TRIANGLES);

        for (int j = 0; j < curMesh.Indices.size(); j += 3) {
            glVertex3f(curMesh.Vertices[curMesh.Indices[j]].Position.X / 10 , curMesh.Vertices[curMesh.Indices[j]].Position.Y / 10 , curMesh.Vertices[curMesh.Indices[j]].Position.Z / 10);
            glVertex3f(curMesh.Vertices[curMesh.Indices[j + 1]].Position.X / 10 , curMesh.Vertices[curMesh.Indices[j + 1]].Position.Y / 10 , curMesh.Vertices[curMesh.Indices[j + 1]].Position.Z / 10);
            glVertex3f(curMesh.Vertices[curMesh.Indices[j + 2]].Position.X / 10 , curMesh.Vertices[curMesh.Indices[j + 2]].Position.Y / 10 , curMesh.Vertices[curMesh.Indices[j + 2]].Position.Z / 10);
        }

        glEnd();
        glEndList();
    }
    Mat_<Vec3f> vertex(1, 4);

    vertex <<   Vec3f(    0.0, 0.33, 0.0),
           Vec3f(    0.0, 0.0, 0.0),
           Vec3f( 1.33 / 3, 0.0, 0.0),
           Vec3f( 1.33 / 3, 0.33, 0.0);

    Mat_<Vec2f> texCoords(1, 4);
    texCoords << Vec2f(0, 0), Vec2f(0, 1), Vec2f(1, 1), Vec2f(1, 0);

    Mat_<int> indices(1, 6);
    indices << 0, 1, 2, 2, 3, 0;
    data.arr.setVertexArray(vertex);
    data.arr.setTexCoordArray(texCoords);
    data.indices.copyFrom(indices);
    data.tex.copyFrom(pattern);

    glEnable(GL_TEXTURE_2D);
    data.tex.bind();
    glDisable(GL_TEXTURE_2D);

}

int main(int argc, char **argv) {
    if ( !cap.isOpened() ) {
        cout << "Cannot open the video file. \n";
        return -1;
    }
    glutInit(&argc, argv);
    glutInitWindowSize(win_width, win_height);
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
    glutCreateWindow("red 3D lighted cube");
    glutDisplayFunc(display);
    init();
    glutMainLoop();
    return 0;
}