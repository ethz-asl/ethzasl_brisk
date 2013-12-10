/*
 * harrisScores_sse_full.cpp
 *
 *  Created on: Apr 22, 2013
 *      Author: lestefan
 */

#include <brisk/harrisScores.hpp>
#include <smmintrin.h>

namespace brisk {

// masks
__m128i mask_lo_epi32 = _mm_set_epi16(0x0000, 0xFFFF, 0x0000, 0xFFFF, 0x0000,
                                      0xFFFF, 0x0000, 0xFFFF);
__m128i mask_hi_epi32 = _mm_set_epi16(0xFFFF, 0x0000, 0xFFFF, 0x0000, 0xFFFF,
                                      0x0000, 0xFFFF, 0x0000);

// helper function for smoothing
inline void smoothGauss_epi16(const short* const data, int i, int j, int cols,
                              __m128i& result_lo, __m128i& result_hi) {
  // load
  const __m128i d_m1_m1_lo=_mm_set_epi32(*(data+(i-1)*cols+j+5),*(data+(i-1)*cols+j+3),*(data+(i-1)*cols+j+1),*(data+(i-1)*cols+j-1));
  const __m128i d_m1_0_lo=_mm_set_epi32(*(data+(i-1)*cols+j+6),*(data+(i-1)*cols+j+4),*(data+(i-1)*cols+j+2),*(data+(i-1)*cols+j));
  const __m128i d_m1_p1_lo=_mm_set_epi32(*(data+(i-1)*cols+j+7),*(data+(i-1)*cols+j+5),*(data+(i-1)*cols+j+3),*(data+(i-1)*cols+j+1));
  const __m128i d_0_m1_lo=_mm_set_epi32(*(data+(i)*cols+j+5),*(data+(i)*cols+j+3),*(data+(i)*cols+j+1),*(data+(i)*cols+j-1));
  const __m128i d_0_0_lo=_mm_set_epi32(*(data+(i)*cols+j+6),*(data+(i)*cols+j+4),*(data+(i)*cols+j+2),*(data+(i)*cols+j));
  const __m128i d_0_p1_lo=_mm_set_epi32(*(data+(i)*cols+j+7),*(data+(i)*cols+j+5),*(data+(i)*cols+j+3),*(data+(i)*cols+j+1));
  const __m128i d_p1_m1_lo=_mm_set_epi32(*(data+(i+1)*cols+j+5),*(data+(i+1)*cols+j+3),*(data+(i+1)*cols+j+1),*(data+(i+1)*cols+j-1));
  const __m128i d_p1_0_lo=_mm_set_epi32(*(data+(i+1)*cols+j+6),*(data+(i+1)*cols+j+4),*(data+(i+1)*cols+j+2),*(data+(i+1)*cols+j));
  const __m128i d_p1_p1_lo=_mm_set_epi32(*(data+(i+1)*cols+j+7),*(data+(i+1)*cols+j+5),*(data+(i+1)*cols+j+3),*(data+(i+1)*cols+j+1));

  const __m128i d_m1_m1_hi=_mm_set_epi32(*(data+(i-1)*cols+j+5+1),*(data+(i-1)*cols+j+3+1),*(data+(i-1)*cols+j+1+1),*(data+(i-1)*cols+j-1+1));
  const __m128i d_m1_0_hi=_mm_set_epi32(*(data+(i-1)*cols+j+6+1),*(data+(i-1)*cols+j+4+1),*(data+(i-1)*cols+j+2+1),*(data+(i-1)*cols+j+1));
  const __m128i d_m1_p1_hi=_mm_set_epi32(*(data+(i-1)*cols+j+7+1),*(data+(i-1)*cols+j+5+1),*(data+(i-1)*cols+j+3+1),*(data+(i-1)*cols+j+1+1));
  const __m128i d_0_m1_hi=_mm_set_epi32(*(data+(i)*cols+j+5+1),*(data+(i)*cols+j+3+1),*(data+(i)*cols+j+1+1),*(data+(i)*cols+j-1+1));
  const __m128i d_0_0_hi=_mm_set_epi32(*(data+(i)*cols+j+6+1),*(data+(i)*cols+j+4+1),*(data+(i)*cols+j+2+1),*(data+(i)*cols+j+1));
  const __m128i d_0_p1_hi=_mm_set_epi32(*(data+(i)*cols+j+7+1),*(data+(i)*cols+j+5+1),*(data+(i)*cols+j+3+1),*(data+(i)*cols+j+1+1));
  const __m128i d_p1_m1_hi=_mm_set_epi32(*(data+(i+1)*cols+j+5+1),*(data+(i+1)*cols+j+3+1),*(data+(i+1)*cols+j+1+1),*(data+(i+1)*cols+j-1+1));
  const __m128i d_p1_0_hi=_mm_set_epi32(*(data+(i+1)*cols+j+6+1),*(data+(i+1)*cols+j+4+1),*(data+(i+1)*cols+j+2+1),*(data+(i+1)*cols+j+1));
  const __m128i d_p1_p1_hi=_mm_set_epi32(*(data+(i+1)*cols+j+7+1),*(data+(i+1)*cols+j+5+1),*(data+(i+1)*cols+j+3+1),*(data+(i+1)*cols+j+1+1));

  // Gaussian smoothing
  const __m128i s1_lo=_mm_add_epi32(_mm_add_epi32(d_m1_m1_lo,d_m1_p1_lo),_mm_add_epi32(d_p1_m1_lo,d_p1_p1_lo));
  const __m128i s1_hi=_mm_add_epi32(_mm_add_epi32(d_m1_m1_hi,d_m1_p1_hi),_mm_add_epi32(d_p1_m1_hi,d_p1_p1_hi));
  const __m128i s2_lo=_mm_add_epi32(_mm_add_epi32(d_0_m1_lo,d_m1_0_lo),_mm_add_epi32(d_p1_0_lo,d_0_p1_lo));
  const __m128i s2_hi=_mm_add_epi32(_mm_add_epi32(d_0_m1_hi,d_m1_0_hi),_mm_add_epi32(d_p1_0_hi,d_0_p1_hi));

  result_lo = _mm_add_epi32(_mm_add_epi32(_mm_slli_epi32(d_0_0_lo,2),_mm_slli_epi32(s2_lo,1)),s1_lo);
  result_lo = _mm_srai_epi32(result_lo,4);
  result_hi = _mm_add_epi32(_mm_add_epi32(_mm_slli_epi32(d_0_0_hi,2),_mm_slli_epi32(s2_hi,1)),s1_hi);
  result_hi = _mm_srai_epi32(result_hi,4);

}

// this is a straightforward harris corner implementation
// this is REALLY bad, it performs so many passes through the data...
void harrisScores_sse_full(const cv::Mat& src, cv::Mat& scores) {
  const int cols = src.cols;
  const int rows = src.rows;
  const int stride = src.step.p[0];
  const int maxJ = cols - 16 - 1;

  // allocate stuff
  short *DxDx1, *DyDy1, *DxDy1;
  scores = cv::Mat::zeros(rows, cols, CV_32S);
  DxDx1 = new short[rows * cols];
  DxDy1 = new short[rows * cols];
  DyDy1 = new short[rows * cols];

  // masks
  __m128i mask_lo = _mm_set_epi8(0x00, 0xFF, 0x00, 0xFF, 0x00, 0xFF, 0x00, 0xFF,
                                 0x00, 0xFF, 0x00, 0xFF, 0x00, 0xFF, 0x00,
                                 0xFF);
  __m128i mask_hi = _mm_set_epi8(0xFF, 0x00, 0xFF, 0x00, 0xFF, 0x00, 0xFF, 0x00,
                                 0xFF, 0x00, 0xFF, 0x00, 0xFF, 0x00, 0xFF,
                                 0x00);

  // consts
  __m128i const_3_epi16 = _mm_set_epi16(3, 3, 3, 3, 3, 3, 3, 3);
  __m128i const_10_epi16 = _mm_set_epi16(10, 10, 10, 10, 10, 10, 10, 10);

  // calculate gradients and products
  const uchar* data = src.data;
  for (int i = 1; i < rows - 1; ++i) {
    bool end = false;
    for (int j = 1; j < cols - 1;) {
      // load
      const __m128i src_m1_m1 = _mm_loadu_si128(
          (__m128i *) (data + (i - 1) * stride + j - 1));
      const __m128i src_m1_0 = _mm_loadu_si128(
          (__m128i *) (data + (i - 1) * stride + j));
      const __m128i src_m1_p1 = _mm_loadu_si128(
          (__m128i *) (data + (i - 1) * stride + j + 1));
      const __m128i src_0_m1 = _mm_loadu_si128(
          (__m128i *) (data + (i) * stride + j - 1));
      const __m128i src_0_0 = _mm_loadu_si128(
          (__m128i *) (data + (i) * stride + j));
      const __m128i src_0_p1 = _mm_loadu_si128(
          (__m128i *) (data + (i) * stride + j + 1));
      const __m128i src_p1_m1 = _mm_loadu_si128(
          (__m128i *) (data + (i + 1) * stride + j - 1));
      const __m128i src_p1_0 = _mm_loadu_si128(
          (__m128i *) (data + (i + 1) * stride + j));
      const __m128i src_p1_p1 = _mm_loadu_si128(
          (__m128i *) (data + (i + 1) * stride + j + 1));

      // Scharr x
      const __m128i dx_lo = _mm_slli_epi16(
          _mm_add_epi16(
              _mm_add_epi16(
                  _mm_mullo_epi16(
                      const_10_epi16,
                      _mm_sub_epi16(_mm_and_si128(mask_lo, src_0_m1),
                                    _mm_and_si128(mask_lo, src_0_p1))),
                  _mm_mullo_epi16(
                      const_3_epi16,
                      _mm_sub_epi16(_mm_and_si128(mask_lo, src_m1_m1),
                                    _mm_and_si128(mask_lo, src_m1_p1)))),
              _mm_mullo_epi16(
                  const_3_epi16,
                  _mm_sub_epi16(_mm_and_si128(mask_lo, src_p1_m1),
                                _mm_and_si128(mask_lo, src_p1_p1)))),
          3);
      const __m128i dx_hi = _mm_slli_epi16(
          _mm_add_epi16(
              _mm_add_epi16(
                  _mm_mullo_epi16(
                      const_10_epi16,
                      _mm_sub_epi16(
                          _mm_srli_si128(_mm_and_si128(mask_hi, src_0_m1), 1),
                          _mm_srli_si128(_mm_and_si128(mask_hi, src_0_p1), 1))),
                  _mm_mullo_epi16(
                      const_3_epi16,
                      _mm_sub_epi16(
                          _mm_srli_si128(_mm_and_si128(mask_hi, src_m1_m1), 1),
                          _mm_srli_si128(_mm_and_si128(mask_hi, src_m1_p1),
                                         1)))),
              _mm_mullo_epi16(
                  const_3_epi16,
                  _mm_sub_epi16(
                      _mm_srli_si128(_mm_and_si128(mask_hi, src_p1_m1), 1),
                      _mm_srli_si128(_mm_and_si128(mask_hi, src_p1_p1), 1)))),
          3);

      // Scharr y
      const __m128i dy_lo = _mm_slli_epi16(
          _mm_add_epi16(
              _mm_add_epi16(
                  _mm_mullo_epi16(
                      const_10_epi16,
                      _mm_sub_epi16(_mm_and_si128(mask_lo, src_m1_0),
                                    _mm_and_si128(mask_lo, src_p1_0))),
                  _mm_mullo_epi16(
                      const_3_epi16,
                      _mm_sub_epi16(_mm_and_si128(mask_lo, src_m1_m1),
                                    _mm_and_si128(mask_lo, src_p1_m1)))),
              _mm_mullo_epi16(
                  const_3_epi16,
                  _mm_sub_epi16(_mm_and_si128(mask_lo, src_m1_p1),
                                _mm_and_si128(mask_lo, src_p1_p1)))),
          3);
      const __m128i dy_hi = _mm_slli_epi16(
          _mm_add_epi16(
              _mm_add_epi16(
                  _mm_mullo_epi16(
                      const_10_epi16,
                      _mm_sub_epi16(
                          _mm_srli_si128(_mm_and_si128(mask_hi, src_m1_0), 1),
                          _mm_srli_si128(_mm_and_si128(mask_hi, src_p1_0), 1))),
                  _mm_mullo_epi16(
                      const_3_epi16,
                      _mm_sub_epi16(
                          _mm_srli_si128(_mm_and_si128(mask_hi, src_m1_m1), 1),
                          _mm_srli_si128(_mm_and_si128(mask_hi, src_p1_m1),
                                         1)))),
              _mm_mullo_epi16(
                  const_3_epi16,
                  _mm_sub_epi16(
                      _mm_srli_si128(_mm_and_si128(mask_hi, src_m1_p1), 1),
                      _mm_srli_si128(_mm_and_si128(mask_hi, src_p1_p1), 1)))),
          3);

      // dxdx dxdy dydy - since we have technically still chars, we only need the lo part
      const __m128i i_lo_dx_dx = _mm_mulhi_epi16(dx_lo, dx_lo);
      const __m128i i_lo_dy_dy = _mm_mulhi_epi16(dy_lo, dy_lo);
      const __m128i i_lo_dx_dy = _mm_mulhi_epi16(dx_lo, dy_lo);
      const __m128i i_hi_dx_dx = _mm_mulhi_epi16(dx_hi, dx_hi);
      const __m128i i_hi_dy_dy = _mm_mulhi_epi16(dy_hi, dy_hi);
      const __m128i i_hi_dx_dy = _mm_mulhi_epi16(dx_hi, dy_hi);

      // unpack - interleave, store
      _mm_storeu_si128((__m128i *) (DxDx1 + i * cols + j),
                       _mm_unpacklo_epi16(i_lo_dx_dx, i_hi_dx_dx));
      _mm_storeu_si128((__m128i *) (DxDx1 + i * cols + j + 8),
                       _mm_unpackhi_epi16(i_lo_dx_dx, i_hi_dx_dx));
      _mm_storeu_si128((__m128i *) (DxDy1 + i * cols + j),
                       _mm_unpacklo_epi16(i_lo_dx_dy, i_hi_dx_dy));
      _mm_storeu_si128((__m128i *) (DxDy1 + i * cols + j + 8),
                       _mm_unpackhi_epi16(i_lo_dx_dy, i_hi_dx_dy));
      _mm_storeu_si128((__m128i *) (DyDy1 + i * cols + j),
                       _mm_unpacklo_epi16(i_lo_dy_dy, i_hi_dy_dy));
      _mm_storeu_si128((__m128i *) (DyDy1 + i * cols + j + 8),
                       _mm_unpackhi_epi16(i_lo_dy_dy, i_hi_dy_dy));

      j += 16;
      if (j > maxJ && !end) {
        j = cols - 1 - 16;
        end = true;
      }
    }
  }

  // smooth gradient products and calculate score
  const int maxJ2 = cols - 2 - 8;
  for (int i = 2; i < rows - 2; ++i) {
    bool end = false;
    for (int j = 2; j < cols - 2;) {
      // dxdx Gaussian smoothing
      __m128i dxdx_lo, dxdx_hi;
      smoothGauss_epi16(DxDx1, i, j, cols, dxdx_lo, dxdx_hi);

      // dxdy Gaussian smoothing
      __m128i dxdy_lo, dxdy_hi;
      smoothGauss_epi16(DxDy1, i, j, cols, dxdy_lo, dxdy_hi);

      // dydy Gaussian smoothing
      __m128i dydy_lo, dydy_hi;
      smoothGauss_epi16(DyDy1, i, j, cols, dydy_lo, dydy_hi);

      __m128i dxdx_p_dydy_div_by_2_lo = _mm_srai_epi32(
          _mm_add_epi32(dxdx_lo, dydy_lo), 1);
      __m128i dxdx_p_dydy_div_by_2_hi = _mm_srai_epi32(
          _mm_add_epi32(dxdx_hi, dydy_hi), 1);

      __m128i score_lo = _mm_sub_epi32(
          _mm_sub_epi32(_mm_mullo_epi32(dxdx_lo, dydy_lo),
                        _mm_mullo_epi32(dxdy_lo, dxdy_lo)),
          _mm_srai_epi32(
              _mm_mullo_epi32(dxdx_p_dydy_div_by_2_lo, dxdx_p_dydy_div_by_2_lo),
              2));
      __m128i score_hi = _mm_sub_epi32(
          _mm_sub_epi32(_mm_mullo_epi32(dxdx_hi, dydy_hi),
                        _mm_mullo_epi32(dxdy_hi, dxdy_hi)),
          _mm_srai_epi32(
              _mm_mullo_epi32(dxdx_p_dydy_div_by_2_hi, dxdx_p_dydy_div_by_2_hi),
              2));

      // store *((int*)scores.data+i*stride+j)
      _mm_storeu_si128((__m128i *) (((int*) scores.data + i * stride + j)),
                       _mm_unpacklo_epi32(score_lo, score_hi));
      _mm_storeu_si128((__m128i *) (((int*) scores.data + i * stride + j + 4)),
                       _mm_unpackhi_epi32(score_lo, score_hi));

      j += 8;
      if (j > maxJ2 && !end) {
        j = cols - 2 - 8;
        end = true;
      }
    }
  }

  /*std::cout << "scores = ";
   for(int j=2; j<19; ++j){
   std::cout << scores.at<int>(3,j) << " ";
   }
   std::cout << std::endl;*/

  // cleanup
  delete DxDx1;
  delete DxDy1;
  delete DyDy1;
}

}  // namespace brisk

