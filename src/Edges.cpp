#include "Edges.h"
using namespace cv;

Edges::Edges() {

}


Edges::~Edges() {
	
}

Mat Edges::sobel(const Mat& image) {
	int width = image.cols;
	int height = image.rows;

	// Gaussian
	Mat gaussian(height, width, CV_8UC1);
	GaussianBlur(image, gaussian, Size(3,3), 0, 0, BORDER_DEFAULT);

	// Sobel
	Mat grad_x, grad_y;
	Mat abs_grad_x, abs_grad_y;

	/// Gradient X
	Sobel( gaussian, grad_x, CV_16UC1, 1, 0, 3, 1, 0, BORDER_DEFAULT );
	convertScaleAbs( grad_x, abs_grad_x );

	/// Gradient Y
	Sobel( gaussian, grad_y, CV_16UC1, 0, 1, 3, 1, 0, BORDER_DEFAULT );
	convertScaleAbs( grad_y, abs_grad_y );

	// Magnitude and direction
	Mat gradientMag(height, width, CV_8UC1);

	for(int y = 0; y < height; y++)
		for(int x = 0; x < width; x++)
			gradientMag.at<uchar>(y,x) = sqrt(abs_grad_x.at<uchar>(y,x) * abs_grad_x.at<uchar>(y,x) + abs_grad_y.at<uchar>(y,x) * abs_grad_y.at<uchar>(y,x));

	// Thresholding
	Mat edgeMap;
	threshold(gradientMag, edgeMap, 0, 255, CV_THRESH_BINARY | CV_THRESH_OTSU);
		
	return edgeMap;
}

Mat Edges::WHT(const Mat& image, int threshold) {
	int width = image.cols;
	int height = image.rows;

	Mat edges(height, width, CV_8UC1, Scalar(0));

	Mat block(2, 2, CV_64FC1, Scalar(0.0));
	Mat transBlock;

	Mat kernel(2, 2, CV_64FC1, Scalar(1.0 / sqrt(2.0)));
	kernel.at<double>(1,1) = -1.0 / sqrt(2.0);

	for(int y = 0; y < height - 2; y += 2) {
		for(int x = 0; x < width - 2; x += 2) {
			for(int i = 0; i < 2; i++)
				for(int j = 0; j < 2; j++)
					block.at<double>(i,j) = (double)image.at<uchar>(y + i, x + j);

			transBlock = kernel * block * kernel;

			double mean = 0.0;

			for(int i = 0; i < 2; i++)
				for(int j = 0; j < 2; j++)
					if(i != 0 && j != 0)
						mean += abs(transBlock.at<double>(i,j));

			mean /= 3.0;

			double variance = 0.0;

			for(int i = 0; i < 2; i++)
				for(int j = 0; j < 2; j++)
					if(i != 0 && j != 0)
						variance += Math::square(abs(transBlock.at<double>(i,j)) - mean);

			variance /= 3.0;

			if(variance > threshold)
				for(int i = 0; i < 2; i++)
					for(int j = 0; j < 2; j++)
						edges.at<uchar>(y + i, x + j) = 255;
		}
	}

	return edges;
}

Mat Edges::direction(const Mat& image) {
	int width = image.cols;
	int height = image.rows;

	// Gaussian
	Mat gaussian(height, width, CV_8UC1);
	GaussianBlur(image, gaussian, Size(3,3), 0, 0, BORDER_DEFAULT);

	// Sobel
	Mat grad_x, grad_y;
	Mat abs_grad_x, abs_grad_y;

	/// Gradient X
	Sobel( gaussian, grad_x, CV_16UC1, 1, 0, 3, 1, 0, BORDER_DEFAULT );
	convertScaleAbs( grad_x, abs_grad_x );

	/// Gradient Y
	Sobel( gaussian, grad_y, CV_16UC1, 0, 1, 3, 1, 0, BORDER_DEFAULT );
	convertScaleAbs( grad_y, abs_grad_y );

	// Direction
	Mat gradientDir(height, width, CV_8UC1);

	double radians = 0.0;
	double degrees = 0.0;

	for(int y = 0; y < height; y++) {
		for(int x = 0; x < width; x++) {
			radians = atan2(abs_grad_y.at<uchar>(y,x), abs_grad_x.at<uchar>(y,x));
			gradientDir.at<uchar>(y,x) = Math::radianToDegree(radians);
		}
	}
	
	return gradientDir;
}
