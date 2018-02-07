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

int main()
{
	VideoCapture cap("../videos/PadronAnillos_01.avi");
	// cap is the object of class video capture that tries to capture Bumpy.mp4
	if ( !cap.isOpened() )  // isOpened() returns true if capturing has been initialized.
	{
		cout << "Cannot open the video file. \n";
		return -1;
	}

	double fps = cap.get(CV_CAP_PROP_FPS); //get the frames per seconds of the video

	namedWindow("Camera calibration", CV_WINDOW_AUTOSIZE); //create a window called "MyVideo"

	while (1)
	{
		Mat frame;

		if (!cap.read(frame)) // if not success, break loop
		{
			cout << "\n Cannot read the video file. \n";
			break;
		}
		Mat edges = edgeDetector(frame);

		imshow("Camera calibration", edges);

		if (waitKey(fps) == 27) // Wait for 'esc' key press to exit
			break;
	}

	return 0;
}


Mat edgeDetector(Mat src){

  Mat  src_gray;
  Mat grad;
  int scale = 1;
  int delta = 0;
  int ddepth = CV_16S;

  int c;


  GaussianBlur( src, src, Size(3,3), 0, 0, BORDER_DEFAULT );

  /// Convert it to gray
  cvtColor( src, src_gray, CV_BGR2GRAY );

  /// Generate grad_x and grad_y
  Mat grad_x, grad_y;
  Mat abs_grad_x, abs_grad_y;

  /// Gradient X
  //Scharr( src_gray, grad_x, ddepth, 1, 0, scale, delta, BORDER_DEFAULT );
  Sobel( src_gray, grad_x, ddepth, 1, 0, 3, scale, delta, BORDER_DEFAULT );
  convertScaleAbs( grad_x, abs_grad_x );

  /// Gradient Y
  //Scharr( src_gray, grad_y, ddepth, 0, 1, scale, delta, BORDER_DEFAULT );
  Sobel( src_gray, grad_y, ddepth, 0, 1, 3, scale, delta, BORDER_DEFAULT );
  convertScaleAbs( grad_y, abs_grad_y );

  /// Total Gradient (approximate)
  addWeighted( abs_grad_x, 0.5, abs_grad_y, 0.5, 0, grad );
  return grad;
}