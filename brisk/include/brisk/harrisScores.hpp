/*
 * harrisScores.hpp
 *
 *  Created on: Apr 22, 2013
 *      Author: lestefan
 */

#ifndef HARRISSCORES_HPP_
#define HARRISSCORES_HPP_

#include <opencv2/opencv.hpp>
#include <brisk/brisk.h>

namespace brisk {

// This is a straightforward harris corner implementation.
// This is REALLY bad, it performs so many passes through the data...
void harrisScores_basic(const cv::Mat& src, cv::Mat& scores);

// No cv::Mat stuff.
void harrisScores_basic_noMats(const cv::Mat& src, cv::Mat& scores);
// Store dx and dy not dxdx, dxdy, dydy. This is slower.
void harrisScores_basic_noMats_store_dx_dy(const cv::Mat& src, cv::Mat& scores);

// SSE speeded up (dxdx dxdy and dydy only). based on harrisScores_basic_noMats(.) .
void harrisScores_sse(const cv::Mat& src, cv::Mat& scores);
void harrisScores_sse_full(const cv::Mat& src, cv::Mat& scores);

// This is slightly better, only one pass
// TODO(lestefan): fix me... verification fails. and it's slow.
void harrisScores_basic2(const cv::Mat& src, cv::Mat& scores);
}  // namespace brisk
#endif /* HARRISSCORES_HPP_ */
