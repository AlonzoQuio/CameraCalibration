#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <vector>

using namespace cv;
using namespace std;
struct MatPixel {
    uchar b;
    uchar g;
    uchar r;
};

void rgb_to_gray(Mat &imagen, int width, int height) {
    //0.2126 * R + 0.7152 * G + 0.0722 * B
    int c;
    for (int w = 0; w < width; w++) {
        for (int h = 0; h < height; h++) {
            MatPixel& pixel = imagen.at<MatPixel>(w, h);
            c = pixel.r * 0.2126 + pixel.g * 0.7152 + pixel.b * 0.0722;
            pixel.r = c;
            pixel.g = c;
            pixel.b = c;
        }
    }
}
void segmentar(Mat &in, Mat &out, int w, int h/*, int **intImg*/) {
    int **intImg = new int*[w];
    for (int i = 0; i < w; i++) {
        intImg[i] = new int[h];
    }

    int sum = 0;
    for (int i = 0; i < w; i++) {
        sum = 0;
        for (int j = 0; j < h; j++) {
            MatPixel& pixel = in.at<MatPixel>(i, j);
            sum += pixel.r;
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
                MatPixel& pixel = in.at<MatPixel>(i, j);
                MatPixel& pixel_o = out.at<MatPixel>(i, j);
                if (pixel.r * count <= sum * (100 - t) / 100) {
                    pixel_o.r = 0;
                    pixel_o.g = 0;
                    pixel_o.b = 0;
                } else {
                    pixel_o.r = 255;
                    pixel_o.g = 255;
                    pixel_o.b = 255;
                }
            }else{
                MatPixel& pixel_o = out.at<MatPixel>(i, j);
                int c = (pixel_o.r + pixel_o.g+ pixel_o.b)/3;
                if(c < 100){
                    c = 0;
                }else{
                    c=255;
                }
                pixel_o.r = c;
                pixel_o.g = c;
                pixel_o.b = c;
            }
        }
    }
    //int **intImg = new int*[w];
    for (int i = 0; i < w; i++) {
        delete intImg[i];
    }
    delete intImg;
}

Mat equalizeIntensity(const Mat& inputImage)
{
    if(inputImage.channels() >= 3){
        Mat ycrcb;

        cvtColor(inputImage,ycrcb,CV_BGR2YCrCb);

        vector<Mat> channels;
        split(ycrcb,channels);

        equalizeHist(channels[0], channels[0]);

        Mat result;
        merge(channels,ycrcb);

        cvtColor(ycrcb,result,CV_YCrCb2BGR);

        return result;
    }
    return Mat();
}


void equalizar_histograma(Mat& imagen, int width, int height) {
    int *h_r, *h_g, *h_b;
    h_r = new int[256];
    h_g = new int[256];
    h_b = new int[256];
    for (int i = 0; i < 256; i++) {
        h_r[i] = 0;
        h_g[i] = 0;
        h_b[i] = 0;
    }
    for (int w = 0; w < width; w++) {
        for (int h = 0; h < height; h++) {
            MatPixel& pixel = imagen.at<MatPixel>(w, h);
            h_r[pixel.r] += 1;
            h_g[pixel.g] += 1;
            h_b[pixel.b] += 1;
        }
    }
    int max_r, min_r, max_g, min_g, max_b, min_b;
    max_r = max_g = max_b = 0;
    min_r = min_g = min_b = 255;
    for (int i = 0; i < 255; i++) {
        if (h_r[i] != 0) {
            if (i < min_r) {
                min_r = i;
            }
            if ( i > max_r) {
                max_r = i;
            }
        }
        if (h_g[i] != 0) {
            if (i < min_g) {
                min_g = i;
            }
            if ( i > max_g) {
                max_g = i;
            }
        }
        if (h_b[i] != 0) {
            if (i < min_b) {
                min_b = i;
            }
            if ( i > max_b) {
                max_b = i;
            }
        }
        h_r[i + 1] = h_r[i] + h_r[i + 1];
        h_g[i + 1] = h_g[i] + h_g[i + 1];
        h_b[i + 1] = h_b[i] + h_b[i + 1];
    }
    for (int w = 0; w < width; w++) {
        for (int h = 0; h < height; h++) {
            MatPixel& pixel = imagen.at<MatPixel>(w, h);
            pixel.r = round((h_r[pixel.r] * 1.0 - min_r ) / (width * height - 1) * 254);
            pixel.g = round((h_g[pixel.g] * 1.0 - min_g ) / (width * height - 1) * 254);
            pixel.b = round((h_b[pixel.b] * 1.0 - min_b ) / (width * height - 1) * 254);
        }
    }
}

void clean_using_mask(Mat &imagen, int width, int height, Point mask_points[][4]) {

    const Point* ppt[1] = { mask_points[0] };
    int npt[] = { 4 };

    Mat mask = Mat::zeros( imagen.size(), CV_8UC3 );

    fillPoly( mask, ppt, npt, 1, Scalar( 255, 255, 255 ), 1 );

    for (int w = 0; w < width; w++) {
        for (int h = 0; h < height; h++) {
            MatPixel& pixel = imagen.at<MatPixel>(w, h);
            MatPixel& mask_value = mask.at<MatPixel>(w, h);
            if(mask_value.r != 255){
                pixel.r = 0;
                pixel.g = 0;
                pixel.b = 0;
            }
        }
    }

}
void segmentar_bn(Mat &in, Mat &out, Mat adapThresh, int w, int h ) {

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