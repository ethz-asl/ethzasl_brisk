/*
 * sseFilters.hpp
 *
 *  Created on: Aug 3, 2012
 *      Author: lestefan
 */

#ifndef SSEFILTERS_HPP_
#define SSEFILTERS_HPP_

namespace brisk{

// generic SSE-optimized 2D filter on CV_8U/CV_16S matrices. stores result in CV_16S matrix.
template<int X, int Y>
__inline__ void filter2D(cv::Mat& src, cv::Mat& dst, cv::Mat& kernel);

// generic SSE-optimized 2D filter CV_8U to CV_16S
template<int X, int Y>
__inline__ void filter2D8U(cv::Mat& src, cv::Mat& dst, cv::Mat& kernel);

// generic SSE-optimized 2D filter CV_16S to CV_16S
template<int X, int Y>
__inline__ void filter2D16S(cv::Mat& src, cv::Mat& dst, cv::Mat& kernel);

// 3-by-3 box filter CV_16S to CV_16S
__inline__ void filterBox3by316S(cv::Mat& src, cv::Mat& dst);

// 3-by-3 Gaussian filter CV_16S to CV_16S
__inline__ void filterGauss3by316S(cv::Mat& src, cv::Mat& dst);

// 3-by-3 Gaussian filter CV_32F to CV_32F
__inline__ void filterGauss3by332F(cv::Mat& src, cv::Mat& dst);

#include "implementation/sseFilters.hpp"

}


#endif /* SSEFILTERS_HPP_ */
