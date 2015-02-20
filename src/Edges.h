/**
  * @class Edges
  * @author Maria Santamaria
  * @date April 2014
  */

#ifndef EDGES_H
#define EDGES_H

#include <cmath>
#include <iostream>

#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "Math.h"

class Edges {
	private:
	
	protected:
	
	public:
		Edges();
		~Edges();
		
		static cv::Mat sobel(const cv::Mat& image);
		static cv::Mat WHT(const cv::Mat& image, int threshold = 25);

		static cv::Mat direction(const cv::Mat& image);
};

#endif
