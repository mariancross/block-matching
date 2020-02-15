#include "BDM.h"
using namespace std;
using namespace cv;

BDM::BDM() {

}

BDM::~BDM() { 
	
}

double BDM::MSE(const Mat& sourceImg, const Mat& referenceImg, const Point2D& sourcePoint, const Point2D& refPoint, int blockSize) {
	double output = 0.0;
	
	for(int y = 0; y < blockSize; y++)
		for(int x = 0; x < blockSize; x++)
			output += Math::square( sourceImg.at<uchar>(y + sourcePoint.getY(), x + sourcePoint.getX()) - referenceImg.at<uchar>(y + refPoint.getY(), x + refPoint.getX()) );
	
	output /= (double)(blockSize * blockSize);
	return output;
}

double BDM::SAD(const Mat& sourceImg, const Mat& referenceImg, const Point2D& sourcePoint, const Point2D& refPoint, int blockSize) {
	double output = 0.0;
	
	for(int y = 0; y < blockSize; y++)
		for(int x = 0; x < blockSize; x++)
			output += abs( sourceImg.at<uchar>(y + sourcePoint.getY(), x + sourcePoint.getX()) - referenceImg.at<uchar>(y + refPoint.getY(), x + refPoint.getX()) );
	
	return output;
}

double BDM::MAD(const Mat& sourceImg, const Mat& referenceImg, const Point2D& sourcePoint, const Point2D& refPoint, int blockSize) {
	double output = 0.0;
	
	for(int y = 0; y < blockSize; y++)
		for(int x = 0; x < blockSize; x++)
			output += abs( sourceImg.at<uchar>(y + sourcePoint.getY(), x + sourcePoint.getX()) - referenceImg.at<uchar>(y + refPoint.getY(), x + refPoint.getX()) );
	
	output /= (double)(blockSize * blockSize);
	return output;
}

double BDM::SSE(const Mat& sourceImg, const Mat& referenceImg, const Point2D& sourcePoint, const Point2D& refPoint, int blockSize) {
	double output = 0.0;
	
	for(int y = 0; y < blockSize; y++)
		for(int x = 0; x < blockSize; x++)
			output += Math::square( sourceImg.at<uchar>(y + sourcePoint.getY(), x + sourcePoint.getX()) - referenceImg.at<uchar>(y + refPoint.getY(), x + refPoint.getX()) );
	
	return output;
}

