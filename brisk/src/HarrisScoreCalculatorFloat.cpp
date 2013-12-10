/*
 * HarrisScoreCalculator.cpp
 *
 *  Created on: Aug 3, 2012
 *      Author: lestefan
 */

#include <iostream>

#include <brisk/HarrisScoreCalculatorFloat.hpp>
#include <brisk/sseFilters.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <sys/time.h>

namespace brisk {

void HarrisScoreCalculatorFloat::initializeScores() {
  cv::Mat DxDx1, DyDy1, DxDy1;
  cv::Mat DxDx, DyDy, DxDy;

  // pipeline
  getCovarEntries(_img, DxDx1, DyDy1, DxDy1);
  filterGauss3by332F(DxDx1, DxDx);
  filterGauss3by332F(DyDy1, DyDy);
  filterGauss3by332F(DxDy1, DxDy);
  cornerHarris(DxDx, DyDy, DxDy, _scores);
}

void HarrisScoreCalculatorFloat::get2dMaxima(
    std::vector<PointWithScore>& points, float absoluteThreshold) {
  // do the 8-neighbor nonmax suppression
  //struct timeval start, end;
  //gettimeofday(&start, NULL);
  const int stride = _scores.cols;
  const int rows_end = _scores.rows - 2;
  for (int j = 2; j < rows_end; ++j) {
    const float* p = &_scores.at<float>(j, 0);
    const float* const p_begin = p;
    const float* const p_end = &_scores.at<float>(j, stride - 2);
    bool last = false;
    while (p < p_end) {
      const float* const center = p;
      ++p;
      if (last) {
        last = false;
        continue;
      }
      //if(lastline.at<uchar>(0,i)) continue;
      if (*center < absoluteThreshold)
        continue;
      if (*(center + 1) > *center)
        continue;
      if (*(center - 1) > *center)
        continue;
      const float* const p1 = (center + stride);
      const float* const p2 = (center - stride);
      if (*p1 > *center)
        continue;
      if (*p2 > *center)
        continue;
      if (*(p1 + 1) > *center)
        continue;
      if (*(p1 - 1) > *center)
        continue;
      if (*(p2 + 1) > *center)
        continue;
      if (*(p2 - 1) > *center)
        continue;
      const int i = p - p_begin - 1;
#ifdef USE_SIMPLE_POINT_WITH_SCORE
      points.push_back(PointWithScore(*center, i, j));
#else
#error
      points.push_back(PointWithScore(cv::Point2i(i,j),*center));
#endif
    }
  }
  //gettimeofday(&end, NULL);
  //std::cout<<double(end.tv_sec-start.tv_sec)*1000.0+double(end.tv_usec-start.tv_usec)/1000.0<<std::endl;
}

// X and Y denote the size of the mask
void HarrisScoreCalculatorFloat::getCovarEntries(const cv::Mat& src,
                                                 cv::Mat& dxdx, cv::Mat& dydy,
                                                 cv::Mat& dxdy) {
//inline void getCovarEntriesLestefan(cv::Mat& src, cv::Mat& dx, cv::Mat& dy, cv::Mat& kernel){
  // sanity check

  int jump = 0;  // number of bytes
  if (src.type() == CV_8U)
    jump = 1;
  else if (src.type() == CV_16U)
    jump = 2;
  else
    assert(0 && "Unsupported type");

  cv::Mat kernel = cv::Mat::zeros(3, 3, CV_32F);
  kernel.at<float>(0, 0) = 0.09375;
  kernel.at<float>(1, 0) = 0.3125;
  kernel.at<float>(2, 0) = 0.09375;
  kernel.at<float>(0, 2) = -0.09375;
  kernel.at<float>(1, 2) = -0.3125;
  kernel.at<float>(2, 2) = -0.09375;

  const unsigned int X = 3;
  const unsigned int Y = 3;

  // dest will be floats
  dxdx = cv::Mat::zeros(src.rows, src.cols, CV_32F);
  dydy = cv::Mat::zeros(src.rows, src.cols, CV_32F);
  dxdy = cv::Mat::zeros(src.rows, src.cols, CV_32F);

  //dx=cv::Mat::zeros(src.rows,src.cols,CV_16S);
  //dy=cv::Mat::zeros(src.rows,src.cols,CV_16S);

  const unsigned int maxJ = ((src.cols - 2) / 4) * 4;
  const unsigned int maxI = src.rows - 2;
  const unsigned int stride = src.cols;

  for (unsigned int i = 0; i < maxI; ++i) {
    bool end = false;
    for (unsigned int j = 0; j < maxJ;) {
      //__m128i result = _mm_set_epi16 ( -127,-127,-127,-127,-127,-127,-127,-127,-127,-127);
      __m128 zeros;
      zeros = _mm_setr_ps(0.0, 0.0, 0.0, 0.0);
      __m128 result_dx = zeros;
      __m128 result_dy = zeros;
      // enter convolution with kernel
      for (unsigned int x = 0; x < X; ++x) {
        //if(dx&&x==1)continue; // jump, 0 kernel
        for (unsigned int y = 0; y < Y; ++y) {
          //if(!dx&&y==1)continue; // jump, 0 kernel
          const float m_dx = kernel.at<float>(y, x);
          const float m_dy = kernel.at<float>(x, y);
          __m128 mult_dx = _mm_setr_ps(m_dx, m_dx, m_dx, m_dx);
          __m128 mult_dy = _mm_setr_ps(m_dy, m_dy, m_dy, m_dy);
          //uchar* p=(src.data+(stride*(i+y))+x+j);
          __m128 i0;
          if (jump == 1) {
            const uchar* p = &src.at < uchar > (i + y, x + j);
            i0 = _mm_setr_ps(float(*p), float(*(p + 1)), float(*(p + 2)),
                             float(*(p + 3)));
          } else {
            const uint16_t* p = &src.at < uint16_t > (i + y, x + j);
            i0 = _mm_setr_ps(float(*p), float(*(p + 1)), float(*(p + 2)),
                             float(*(p + 3)));
          }

          if (m_dx != 0) {
            __m128 i_dx = _mm_mul_ps(i0, mult_dx);
            result_dx = _mm_add_ps(result_dx, i_dx);
          }

          if (m_dy != 0) {
            __m128 i_dy = _mm_mul_ps(i0, mult_dy);
            result_dy = _mm_add_ps(result_dy, i_dy);
          }
        }
      }

      // calculate covariance entries - remove precision (ends up being 4 bit), then remove 4 more bits
      __m128 i_dx_dx = _mm_mul_ps(result_dx, result_dx);
      __m128 i_dx_dy = _mm_mul_ps(result_dy, result_dx);
      __m128 i_dy_dy = _mm_mul_ps(result_dy, result_dy);

      // store
      _mm_storeu_ps(&dxdx.at<float>(i, j + 1), i_dx_dx);
      _mm_storeu_ps(&dxdy.at<float>(i, j + 1), i_dx_dy);
      _mm_storeu_ps(&dydy.at<float>(i, j + 1), i_dy_dy);

      // take care about end
      j += 4;
      if (j >= maxJ && !end) {
        j = stride - 2 - 4;
        end = true;
      }
    }
  }

  //cv::imshow("dxdx", (dxdx)*(1.0/512.0)+0.5);
  //cv::imshow("dxdx",dxdx);
  //cv::imshow("dydy",dydy*(1.0/512.0)+0.5);
  //cv::imshow("dxdy",dxdy*(1/255)+1);
  //cv::waitKey();
}

void HarrisScoreCalculatorFloat::cornerHarris(const cv::Mat& dxdxSmooth,
                                              const cv::Mat& dydySmooth,
                                              const cv::Mat& dxdySmooth,
                                              cv::Mat& dst) {

  // dest will be float
  dst = cv::Mat::zeros(dxdxSmooth.rows, dxdxSmooth.cols, CV_32F);
  const unsigned int maxJ = ((dxdxSmooth.cols - 2) / 8) * 8;
  const unsigned int maxI = dxdxSmooth.rows - 2;
  const unsigned int stride = dxdxSmooth.cols;

  for (unsigned int i = 0; i < maxI; ++i) {
    bool end = false;
    for (unsigned int j = 0; j < maxJ;) {
      __m128 dxdx = _mm_loadu_ps(&dxdxSmooth.at<float>(i, j));
      __m128 dydy = _mm_loadu_ps(&dydySmooth.at<float>(i, j));
      __m128 dxdy = _mm_loadu_ps(&dxdySmooth.at<float>(i, j));

      // determinant terms
      __m128 prod1 = _mm_mul_ps(dxdx, dydy);
      __m128 prod2 = _mm_mul_ps(dxdy, dxdy);

      // calculate the determinant
      __m128 det = _mm_sub_ps(prod1, prod2);

      // trace - uses kappa=1/16
      __m128 trace = _mm_add_ps(dxdx, dydy);
      __m128 trace_sq = _mm_mul_ps(trace, trace);
      __m128 trace_sq_00625 = _mm_mul_ps(
          trace_sq, _mm_setr_ps(0.0625, 0.0625, 0.0625, 0.0625));

      // form score
      __m128 score = _mm_sub_ps(det, trace_sq_00625);

      // store
      _mm_storeu_ps(&dst.at<float>(i, j), score);

      // take care about end
      j += 4;
      if (j >= maxJ && !end) {
        j = stride - 2 - 4;
        end = true;
      }
    }
  }

  //double max;
  //double min;
  //cv::minMaxIdx(dst, &min,&max);
  //cv::imshow("view", (dst-min)*(1/(max-min)));
  //cv::waitKey();

}

}
