#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <vector>

using namespace cv;
using namespace std;

/**
 * @details filter points outside from the mask
 * 
 * @param imagen image input and output
 * @param width image width
 * @param height image height
 * @param mask_points rectangle points to build the mask
 */
void clean_using_mask(Mat &imagen, int width, int height, Point mask_points[][4]) {
    const Point* ppt[1] = { mask_points[0] };
    int npt[] = { 4 };

    Mat mask = Mat::zeros( imagen.size(), CV_8UC3 );

    fillPoly( mask, ppt, npt, 1, Scalar( 255, 255, 255 ), 1 );

    for (int w = 0; w < width; w++) {
        for (int h = 0; h < height; h++) {
            Vec3b& pixel = imagen.at<Vec3b>(w, h);
            Vec3b& mask_value = mask.at<Vec3b>(w, h);
            if(mask_value[2] != 255){
                pixel[0] = 0;
                pixel[1] = 0;
                pixel[2] = 0;
            }
        }
    }
}

/**
 * @details Adaptative threshold segmentation complemented with opencv threshold for borders
 * 
 * @param in Image input in grayscale
 * @param out Image output in grayscale
 * @param adapThresh Image segmented using opencv threshold
 * @param w image width
 * @param h image height
 */
void segmentar(Mat &in, Mat &out, Mat adapThresh, int w, int h ) {

    int **intImg = new int*[w];
    for (int i = 0; i < w; i++) {
        intImg[i] = new int[h];
    }

    int sum = 0;
    for (int i = 0; i < w; i++) {
        sum = 0;
        for (int j = 0; j < h; j++) {
            unsigned char & pixel = in.at<unsigned char >(i, j);
            sum += pixel;
            if (i == 0) {
                intImg[i][j] = sum;
            } else {
                intImg[i][j] = intImg[i - 1][j] + sum;
            }
        }
    }
    int s = w / 8;
    int t = 15;
    int x1, x2, y1, y2;
    int count;
    for (int i = 0; i < w; i++) {
        for (int j = 0; j < h; j++) {
            x1 = i - s / 2;
            x2 = i + s / 2;
            y1 = j - s / 2;
            y2 = j + s / 2;

            if (x1 > 0 && x2 < w && y1 > 0 && y2 < h) {
                count = (x2 - x1) * (y2 - y1);
                sum = intImg[x2][y2] - intImg[x2][y1 - 1] - intImg[x1 - 1][y2] + intImg[x1 - 1][y1 - 1];
                unsigned char & pixel = in.at<unsigned char >(i, j);
                unsigned char & pixel_o = out.at<unsigned char >(i, j);
                if (pixel * count <= sum * (100 - t) / 100) {
                    pixel_o = 0;
                } else {
                    pixel_o = 255;
                }
            }else{
                unsigned char & pixel_o = out.at<unsigned char >(i, j);
                pixel_o = adapThresh.at<unsigned char>(i, j);
            }

        }
    }
    for (int i = 0; i < w; i++) {
        delete intImg[i];
    }
    delete intImg;
}