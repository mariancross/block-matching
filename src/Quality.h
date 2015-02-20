/**
  * @class Quality
  * @author Maria Santamaria
  * @date April 2014
  */

#ifndef QUALITY_H
#define QUALITY_H

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>

class Quality {
	private:

	protected:

	public:
		Quality();
		~Quality();

		static double PSNR(const cv::Mat& I1, const cv::Mat& I2);
		static cv::Scalar MSSIM(const cv::Mat& i1, const cv::Mat& i2);
};

#endif
