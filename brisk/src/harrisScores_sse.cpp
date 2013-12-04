/*
 * harrisScores_sse.cpp
 *
 *  Created on: Apr 22, 2013
 *      Author: lestefan
 */

#include <brisk/harrisScores.hpp>

namespace brisk {

// this is a straightforward harris corner implementation
// this is REALLY bad, it performs so many passes through the data...
void harrisScores_sse(const cv::Mat& src, cv::Mat& scores) {
  const int cols = src.cols;
  const int rows = src.rows;
  const int stride = src.step.p[0];
  const int maxJ = cols - 1 - 16;

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
      //const __m128i src_0_0=_mm_loadu_si128((__m128i*)(data+(i)*stride+j));
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
  for (int i = 2; i < rows - 2; ++i) {
    for (int j = 2; j < cols - 2; j++) {
      // load
      // dxdx
      const short dxdx_m1_m1 = DxDx1[(i - 1) * cols + j - 1];
      const short dxdx_m1_0 = DxDx1[(i - 1) * cols + j];
      const short dxdx_m1_p1 = DxDx1[(i - 1) * cols + j + 1];
      const short dxdx_0_m1 = DxDx1[(i) * cols + j - 1];
      const short dxdx_0_0 = DxDx1[(i) * cols + j];
      const short dxdx_0_p1 = DxDx1[(i) * cols + j + 1];
      const short dxdx_p1_m1 = DxDx1[(i + 1) * cols + j - 1];
      const short dxdx_p1_0 = DxDx1[(i + 1) * cols + j];
      const short dxdx_p1_p1 = DxDx1[(i + 1) * cols + j + 1];

      // Gaussian smoothing
      int dxdx = ((4 * int(dxdx_0_0)
          + 2
              * (int(dxdx_m1_0) + int(dxdx_p1_0) + int(dxdx_0_m1)
                  + int(dxdx_0_p1))
          + (int(dxdx_m1_m1) + int(dxdx_m1_p1) + int(dxdx_p1_m1)
              + int(dxdx_p1_p1))) >> 4);

      // dxdy
      const short dxdy_m1_m1 = DxDy1[(i - 1) * cols + j - 1];
      const short dxdy_m1_0 = DxDy1[(i - 1) * cols + j];
      const short dxdy_m1_p1 = DxDy1[(i - 1) * cols + j + 1];
      const short dxdy_0_m1 = DxDy1[(i) * cols + j - 1];
      const short dxdy_0_0 = DxDy1[(i) * cols + j];
      const short dxdy_0_p1 = DxDy1[(i) * cols + j + 1];
      const short dxdy_p1_m1 = DxDy1[(i + 1) * cols + j - 1];
      const short dxdy_p1_0 = DxDy1[(i + 1) * cols + j];
      const short dxdy_p1_p1 = DxDy1[(i + 1) * cols + j + 1];

      // Gaussian smoothing
      int dxdy = ((4 * int(dxdy_0_0)
          + 2
              * (int(dxdy_m1_0) + int(dxdy_p1_0) + int(dxdy_0_m1)
                  + int(dxdy_0_p1))
          + (int(dxdy_m1_m1) + int(dxdy_m1_p1) + int(dxdy_p1_m1)
              + int(dxdy_p1_p1))) >> 4);

      // dydy
      const short dydy_m1_m1 = DyDy1[(i - 1) * cols + j - 1];
      const short dydy_m1_0 = DyDy1[(i - 1) * cols + j];
      const short dydy_m1_p1 = DyDy1[(i - 1) * cols + j + 1];
      const short dydy_0_m1 = DyDy1[(i) * cols + j - 1];
      const short dydy_0_0 = DyDy1[(i) * cols + j];
      const short dydy_0_p1 = DyDy1[(i) * cols + j + 1];
      const short dydy_p1_m1 = DyDy1[(i + 1) * cols + j - 1];
      const short dydy_p1_0 = DyDy1[(i + 1) * cols + j];
      const short dydy_p1_p1 = DyDy1[(i + 1) * cols + j + 1];

      // Gaussian smoothing
      int dydy = ((4 * int(dydy_0_0)
          + 2
              * (int(dydy_m1_0) + int(dydy_p1_0) + int(dydy_0_m1)
                  + int(dydy_0_p1))
          + (int(dydy_m1_m1) + int(dydy_m1_p1) + int(dydy_p1_m1)
              + int(dydy_p1_p1))) >> 4);

      int trace_div_by_2 = ((dxdx) + (dydy)) >> 1;
      *((int*) scores.data + i * stride + j) = ((((dxdx) * (dydy)))
          - (((dxdy) * (dxdy))))
          - ((((trace_div_by_2)) * ((trace_div_by_2)) >> 2));
    }
  }

  // cleanup
  delete DxDx1;
  delete DxDy1;
  delete DyDy1;
}

}  // namespace brisk

