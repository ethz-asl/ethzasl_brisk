/*
 * sseFilters.hpp
 *
 *  Created on: Jul 28, 2012
 *      Author: lestefan
 */

template<int X, int Y>
__inline__ void filter2D16S(cv::Mat& src, cv::Mat& dst, cv::Mat& kernel) {
  // sanity check
  assert(kernel.type() == CV_16S);
  assert(Y == kernel.rows);
  assert(X == kernel.cols);
  assert(X % 2 != 0);
  assert(Y % 2 != 0);
  int cx = X / 2;
  int cy = Y / 2;

  // dest will be 16 bit
  dst = cv::Mat::zeros(src.rows, src.cols, CV_16S);
  const unsigned int maxJ = ((src.cols - 2) / 8) * 8;
  const unsigned int maxI = src.rows - 2;
  const unsigned int stride = src.cols;

  for (unsigned int i = 0; i < maxI; ++i) {
    bool end = false;
    for (unsigned int j = 0; j < maxJ;) {
      //__m128i result = _mm_set_epi16 ( -127,-127,-127,-127,-127,-127,-127,-127,-127,-127);
      __m128i result = _mm_set_epi16(0, 0, 0, 0, 0, 0, 0, 0);
      // enter convolution with kernel
      for (unsigned int x = 0; x < X; ++x) {
        //if(dx&&x==1)continue; // jump, 0 kernel
        for (unsigned int y = 0; y < Y; ++y) {
          //if(!dx&&y==1)continue; // jump, 0 kernel
          const char m = kernel.at<short>(y, x);
          if (m == 0)
            continue;
          __m128i mult = _mm_set_epi16(m, m, m, m, m, m, m, m);
          __m128i i0 = _mm_loadu_si128(
              (__m128i *) &src.at<short>(i + y, j + x));
          __m128i i1 = _mm_mullo_epi16(i0, mult);
          result = _mm_add_epi16(result, i1);
        }
      }
      // store
      //uchar* p_r=(dst.data+(2*stride*(i+cy)))+2*cx+2*j;
      _mm_storeu_si128((__m128i *) &dst.at<short>(i + cy, j + cx), result);

      // take care about end
      j += 8;
      if (j >= maxJ && !end) {
        j = stride - 2 - 8;
        end = true;
      }
    }
  }

}

__inline__ void filterGauss3by316S(cv::Mat& src, cv::Mat& dst) {
  // sanity check
  const unsigned int X = 3;
  const unsigned int Y = 3;
  assert(X % 2 != 0);
  assert(Y % 2 != 0);
  int cx = X / 2;
  int cy = Y / 2;

  // dest will be 16 bit
  dst = cv::Mat::zeros(src.rows, src.cols, CV_16S);
  const unsigned int maxJ = ((src.cols - 2) / 8) * 8;
  const unsigned int maxI = src.rows - 2;
  const unsigned int stride = src.cols;

  for (unsigned int i = 0; i < maxI; ++i) {
    bool end = false;
    for (unsigned int j = 0; j < maxJ;) {
      // enter convolution with kernel. do the multiplication with 2/4 at the same time
      __m128i i00 = _mm_loadu_si128((__m128i *) &src.at<short>(i, j));
      __m128i i10 = _mm_slli_epi16(
          _mm_loadu_si128((__m128i *) &src.at<short>(i + 1, j)), 1);
      __m128i i20 = _mm_loadu_si128((__m128i *) &src.at<short>(i + 2, j));
      __m128i i01 = _mm_slli_epi16(
          _mm_loadu_si128((__m128i *) &src.at<short>(i, j + 1)), 1);
      __m128i i11 = _mm_slli_epi16(
          _mm_loadu_si128((__m128i *) &src.at<short>(i + 1, j + 1)), 2);
      __m128i i21 = _mm_slli_epi16(
          _mm_loadu_si128((__m128i *) &src.at<short>(i + 2, j + 1)), 1);
      __m128i i02 = _mm_loadu_si128((__m128i *) &src.at<short>(i, j + 2));
      __m128i i12 = _mm_slli_epi16(
          _mm_loadu_si128((__m128i *) &src.at<short>(i + 1, j + 2)), 1);
      __m128i i22 = _mm_loadu_si128((__m128i *) &src.at<short>(i + 2, j + 2));
      __m128i result = i11;
      // add up
      result = _mm_add_epi16(result, i00);
      result = _mm_add_epi16(result, i20);
      result = _mm_add_epi16(result, i02);
      result = _mm_add_epi16(result, i22);

      result = _mm_add_epi16(result, i10);
      result = _mm_add_epi16(result, i01);
      result = _mm_add_epi16(result, i12);
      result = _mm_add_epi16(result, i21);

      // store
      //uchar* p_r=(dst.data+(2*stride*(i+cy)))+2*cx+2*j;
      _mm_storeu_si128((__m128i *) &dst.at<short>(i + cy, j + cx), result);

      // take care about end
      j += 8;
      if (j >= maxJ && !end) {
        j = stride - 2 - 8;
        end = true;
      }
    }
  }
}

__inline__ void filterGauss3by332F(cv::Mat& src, cv::Mat& dst) {
  // sanity check
  static const unsigned int X = 3;
  static const unsigned int Y = 3;
  static const int cx = X / 2;
  static const int cy = Y / 2;

  // dest will be 16 bit
  dst = cv::Mat::zeros(src.rows, src.cols, CV_32F);
  const unsigned int maxJ = ((src.cols - 2) / 8) * 8;
  const unsigned int maxI = src.rows - 2;
  const unsigned int stride = src.cols;

  for (unsigned int i = 0; i < maxI; ++i) {
    bool end = false;
    for (unsigned int j = 0; j < maxJ;) {
      // enter convolution with kernel. do the multiplication with 2/4 at the same time
      __m128 i00 = _mm_loadu_ps(&src.at<float>(i, j));
      __m128 i10 = _mm_loadu_ps(&src.at<float>(i + 1, j));
      __m128 i20 = _mm_loadu_ps(&src.at<float>(i + 2, j));
      __m128 i01 = _mm_loadu_ps(&src.at<float>(i, j + 1));
      __m128 result = _mm_loadu_ps(&src.at<float>(i + 1, j + 1));
      __m128 i21 = _mm_loadu_ps(&src.at<float>(i + 2, j + 1));
      __m128 i02 = _mm_loadu_ps(&src.at<float>(i, j + 2));
      __m128 i12 = _mm_loadu_ps(&src.at<float>(i + 1, j + 2));
      __m128 i22 = _mm_loadu_ps(&src.at<float>(i + 2, j + 2));

      // add up
      result = _mm_add_ps(_mm_mul_ps(result, _mm_setr_ps(4, 4, 4, 4)), i00);
      result = _mm_add_ps(result, i20);
      result = _mm_add_ps(result, i02);
      result = _mm_add_ps(result, i22);

      __m128 result2 = _mm_add_ps(i01, i10);
      result2 = _mm_add_ps(result2, i12);
      result2 = _mm_add_ps(result2, i21);
      result = _mm_add_ps(_mm_mul_ps(result2, _mm_setr_ps(2, 2, 2, 2)), result);
      result = _mm_mul_ps(result2, _mm_setr_ps(0.0625, 0.0625, 0.0625, 0.0625));

      // store
      _mm_storeu_ps(&dst.at<float>(i + cy, j + cx), result);

      // take care about end
      j += 4;
      if (j >= maxJ && !end) {
        j = stride - 2 - 4;
        end = true;
      }
    }
  }
}

__inline__ void filterBox3by316S(cv::Mat& src, cv::Mat& dst) {
  // sanity check
  const unsigned int X = 3;
  const unsigned int Y = 3;
  assert(X % 2 != 0);
  assert(Y % 2 != 0);
  int cx = X / 2;
  int cy = Y / 2;

  // dest will be 16 bit
  dst = cv::Mat::zeros(src.rows, src.cols, CV_16S);
  const unsigned int maxJ = ((src.cols - 2) / 8) * 8;
  const unsigned int maxI = src.rows - 2;
  const unsigned int stride = src.cols;

  for (unsigned int i = 0; i < maxI; ++i) {
    bool end = false;
    for (unsigned int j = 0; j < maxJ;) {
      // enter convolution with kernel. do the multiplication with 2/4 at the same time
      __m128i i00 = _mm_loadu_si128((__m128i *) &src.at<short>(i, j));
      __m128i i10 = (_mm_loadu_si128((__m128i *) &src.at<short>(i + 1, j)));
      __m128i i20 = _mm_loadu_si128((__m128i *) &src.at<short>(i + 2, j));
      __m128i i01 = (_mm_loadu_si128((__m128i *) &src.at<short>(i, j + 1)));
      __m128i i11 = (_mm_loadu_si128((__m128i *) &src.at<short>(i + 1, j + 1)));
      __m128i i21 = (_mm_loadu_si128((__m128i *) &src.at<short>(i + 2, j + 1)));
      __m128i i02 = _mm_loadu_si128((__m128i *) &src.at<short>(i, j + 2));
      __m128i i12 = (_mm_loadu_si128((__m128i *) &src.at<short>(i + 1, j + 2)));
      __m128i i22 = _mm_loadu_si128((__m128i *) &src.at<short>(i + 2, j + 2));
      __m128i result = i11;
      // add up
      result = _mm_add_epi16(result, i00);
      result = _mm_add_epi16(result, i20);
      result = _mm_add_epi16(result, i02);
      result = _mm_add_epi16(result, i22);

      result = _mm_add_epi16(result, i10);
      result = _mm_add_epi16(result, i01);
      result = _mm_add_epi16(result, i12);
      result = _mm_add_epi16(result, i21);

      // store
      //uchar* p_r=(dst.data+(2*stride*(i+cy)))+2*cx+2*j;
      _mm_storeu_si128((__m128i *) &dst.at<short>(i + cy, j + cx), result);

      // take care about end
      j += 8;
      if (j >= maxJ && !end) {
        j = stride - 2 - 8;
        end = true;
      }
    }
  }
}

template<int X, int Y>
__inline__ void filter2D8U(cv::Mat& src, cv::Mat& dst, cv::Mat& kernel) {
  // sanity check
  assert(kernel.type() == CV_8S);
  assert(Y == kernel.rows);
  assert(X == kernel.cols);
  assert(X % 2 != 0);
  assert(Y % 2 != 0);
  int cx = X / 2;
  int cy = Y / 2;

  // dest will be 16 bit
  dst = cv::Mat::zeros(src.rows, src.cols, CV_16S);
  const unsigned int maxJ = ((src.cols - 2) / 16) * 16;
  const unsigned int maxI = src.rows - 2;
  const unsigned int stride = src.cols;

  __m128i mask_hi = _mm_set_epi8(0x00, 0xFF, 0x00, 0xFF, 0x00, 0xFF, 0x00, 0xFF,
                                 0x00, 0xFF, 0x00, 0xFF, 0x00, 0xFF, 0x00,
                                 0xFF);
  __m128i mask_lo = _mm_set_epi8(0xFF, 0x00, 0xFF, 0x00, 0xFF, 0x00, 0xFF, 0x00,
                                 0xFF, 0x00, 0xFF, 0x00, 0xFF, 0x00, 0xFF,
                                 0x00);

  for (unsigned int i = 0; i < maxI; ++i) {
    bool end = false;
    for (unsigned int j = 0; j < maxJ;) {
      //__m128i result = _mm_set_epi16 ( -127,-127,-127,-127,-127,-127,-127,-127,-127,-127);
      __m128i result_hi = _mm_set_epi16(0, 0, 0, 0, 0, 0, 0, 0);
      __m128i result_lo = _mm_set_epi16(0, 0, 0, 0, 0, 0, 0, 0);
      // enter convolution with kernel
      for (unsigned int x = 0; x < X; ++x) {
        //if(dx&&x==1)continue; // jump, 0 kernel
        for (unsigned int y = 0; y < Y; ++y) {
          //if(!dx&&y==1)continue; // jump, 0 kernel
          const char m = kernel.at<char>(y, x);
          if (m == 0)
            continue;
          //ROS_WARN_STREAM(short(m));
          __m128i mult = _mm_set_epi16(m, m, m, m, m, m, m, m);
          uchar* p = (src.data + (stride * (i + y)) + x + j);
          __m128i i0 = _mm_loadu_si128((__m128i *) p);
          __m128i i0_hi = _mm_and_si128(i0, mask_hi);
          __m128i i0_lo = _mm_srli_si128(_mm_and_si128(i0, mask_lo), 1);

          __m128i i_hi = _mm_mullo_epi16(i0_hi, mult);
          __m128i i_lo = _mm_mullo_epi16(i0_lo, mult);
          result_hi = _mm_add_epi16(result_hi, i_hi);
          result_lo = _mm_add_epi16(result_lo, i_lo);
        }
      }
      // store
      uchar* p_lo = (dst.data + (2 * stride * (i + cy))) + 2 * cx + 2 * j;
      uchar* p_hi = (dst.data + (2 * stride * (i + cy))) + 2 * cx + 2 * j + 16;
      _mm_storeu_si128((__m128i *) p_hi,
                       _mm_unpackhi_epi16(result_hi, result_lo));
      _mm_storeu_si128((__m128i *) p_lo,
                       _mm_unpacklo_epi16(result_hi, result_lo));

      // take care about end
      j += 16;
      if (j >= maxJ && !end) {
        j = stride - 2 - 16;
        end = true;
      }
    }
  }
}

// X and Y denote the size of the mask
template<int X, int Y>
__inline__ void filter2D(cv::Mat& src, cv::Mat& dst, cv::Mat& kernel) {
  if (src.type() == CV_8U)
    filter2D8U<X, Y>(src, dst, kernel);
  else if (src.type() == CV_16S)
    filter2D16S<X, Y>(src, dst, kernel);
  else
    assert(0 && "only CV_8U and CV_16S are supported src matrix types");
}
