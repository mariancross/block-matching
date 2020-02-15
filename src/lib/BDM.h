/**
  * @class BDM
  * @author Maria Santamaria
  * @date January 2014
  */

#ifndef BLOCK_DISTORTION_MEASURE_H
#define BLOCK_DISTORTION_MEASURE_H

#include <cmath>
#include <utility>
#include <opencv2/core/core.hpp>
#include "Math.h"
#include "Point2D.h"

class BDM {
	private:

	protected:

	public:
		BDM();		
		~BDM();
		
		// Mean Squared Error
		static double MSE(const cv::Mat& sourceImg, const cv::Mat& referenceImg, const Point2D& sourcePoint, const Point2D& refPoint, int blockSize);	
		
		// Sum of Absolute Differences
		static double SAD(const cv::Mat& sourceImg, const cv::Mat& referenceImg, const Point2D& sourcePoint, const Point2D& refPoint, int blockSize);
		
		// Mean of Absolute Differences
		static double MAD(const cv::Mat& sourceImg, const cv::Mat& referenceImg, const Point2D& sourcePoint, const Point2D& refPoint, int blockSize);
		
		// Sum of Squared Errors
		static double SSE(const cv::Mat& sourceImg, const cv::Mat& referenceImg, const Point2D& sourcePoint, const Point2D& refPoint, int blockSize);
};

#endif

