/*
 Copyright (C) 2011  The Autonomous Systems Lab, ETH Zurich,
 Stefan Leutenegger, Simon Lynen and Margarita Chli.

 Copyright (C) 2013  The Autonomous Systems Lab, ETH Zurich,
 Stefan Leutenegger and Simon Lynen.

 BRISK - Binary Robust Invariant Scalable Keypoints
 Reference implementation of
 [1] Stefan Leutenegger,Margarita Chli and Roland Siegwart, BRISK:
 Binary Robust Invariant Scalable Keypoints, in Proceedings of
 the IEEE International Conference on Computer Vision (ICCV2011).

 This file is part of BRISK.

 All rights reserved.

 Redistribution and use in source and binary forms, with or without
 modification, are permitted provided that the following conditions are met:
 * Redistributions of source code must retain the above copyright
 notice, this list of conditions and the following disclaimer.
 * Redistributions in binary form must reproduce the above copyright
 notice, this list of conditions and the following disclaimer in the
 documentation and/or other materials provided with the distribution.
 * Neither the name of the <organization> nor the
 names of its contributors may be used to endorse or promote products
 derived from this software without specific prior written permission.

 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
 DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <brisk/internal/brisk-layer.h>
#include <tmmintrin.h>

namespace brisk {
// Construct a layer.
BriskLayer::BriskLayer(const cv::Mat& img, uchar upperThreshold,
                       uchar lowerThreshold, float scale, float offset) {
  upperThreshold_ = upperThreshold;
  lowerThreshold_ = lowerThreshold;

  img_ = img;
  scores_ = cv::Mat::zeros(img.rows, img.cols, CV_8U);
  // Attention: this means that the passed image reference must point to
  // persistent memory.
  scale_ = scale;
  offset_ = offset;
  // Create an agast detector.
  oastDetector_.reset(new agast::OastDetector9_16(img.cols, img.rows, 0));
  agastDetector_5_8_.reset(new agast::AgastDetector5_8(img.cols, img.rows, 0));

  // Calculate threshold map.
  CalculateThresholdMap();
}
// Derive a layer.
BriskLayer::BriskLayer(const BriskLayer& layer, int mode, uchar upperThreshold,
                       uchar lowerThreshold) {
  upperThreshold_ = upperThreshold;
  lowerThreshold_ = lowerThreshold;

  if (mode == CommonParams::HALFSAMPLE) {
    img_.create(layer.img().rows / 2, layer.img().cols / 2, CV_8U);
    HalfSample(layer.img(), img_);
    scale_ = layer.scale() * 2;
    offset_ = 0.5 * scale_ - 0.5;
  } else {
    img_.create(2 * (layer.img().rows / 3), 2 * (layer.img().cols / 3), CV_8U);
    TwoThirdSample(layer.img(), img_);
    scale_ = layer.scale() * 1.5;
    offset_ = 0.5 * scale_ - 0.5;
  }
  scores_ = cv::Mat::zeros(img_.rows, img_.cols, CV_8U);
  oastDetector_.reset(new agast::OastDetector9_16(img_.cols, img_.rows, 0));
  agastDetector_5_8_.reset(
      new agast::AgastDetector5_8(img_.cols, img_.rows, 0));

  // Calculate threshold map.
  CalculateThresholdMap();
}

// Fast/Agast.
// Wraps the agast class.
void BriskLayer::GetAgastPoints(uint8_t threshold,
                                std::vector<CvPoint>* keypoints) {
  oastDetector_->set_threshold(threshold, upperThreshold_, lowerThreshold_);
  oastDetector_->detect(img_.data, *keypoints, &thrmap_);

  // Also write scores.
  const int num = keypoints->size();
  const int imcols = img_.cols;

  for (int i = 0; i < num; i++) {
    const int offs = (*keypoints)[i].x + (*keypoints)[i].y * imcols;
    int thr = *(thrmap_.data + offs);
    oastDetector_->set_threshold(thr);
    *(scores_.data + offs) = oastDetector_->cornerScore(img_.data + offs);
  }
}
uint8_t BriskLayer::GetAgastScore(int x, int y, uint8_t threshold) {
  if (x < 3 || y < 3)
    return 0;
  if (x >= img_.cols - 3 || y >= img_.rows - 3)
    return 0;
  uint8_t& score = *(scores_.data + x + y * scores_.cols);
  if (score > 2) {
    return score;
  }
  oastDetector_->set_threshold(threshold - 1);
  score = oastDetector_->cornerScore(img_.data + x + y * img_.cols);
  if (score < threshold)
    score = 0;
  return score;
}

uint8_t BriskLayer::GetAgastScore_5_8(int x, int y, uint8_t threshold) {
  if (x < 2 || y < 2)
    return 0;
  if (x >= img_.cols - 2 || y >= img_.rows - 2)
    return 0;
  agastDetector_5_8_->set_threshold(threshold - 1);
  uint8_t score = agastDetector_5_8_->cornerScore(
      img_.data + x + y * img_.cols);
  if (score < threshold)
    score = 0;
  return score;
}

uint8_t BriskLayer::GetAgastScore(float xf, float yf, uint8_t threshold,
                                  float scale) {
  if (scale <= 1.0f) {
    // Just do an interpolation inside the layer.
    const int x = static_cast<int>(xf);
    const float rx1 = xf - static_cast<float>(x);
    const float rx = 1.0f - rx1;
    const int y = static_cast<int>(yf);
    const float ry1 = yf - static_cast<float>(y);
    const float ry = 1.0f - ry1;

    return rx * ry * GetAgastScore(x, y, threshold)
        + rx1 * ry * GetAgastScore(x + 1, y, threshold)
        + rx * ry1 * GetAgastScore(x, y + 1, threshold)
        + rx1 * ry1 * GetAgastScore(x + 1, y + 1, threshold);
  } else {
    // This means we overlap area smoothing.
    const float halfscale = scale / 2.0f;
    // Get the scores first:
    for (int x = static_cast<int>(xf - halfscale);
        x <= static_cast<int>(xf + halfscale + 1.0f); x++) {
      for (int y = static_cast<int>(yf - halfscale);
          y <= static_cast<int>(yf + halfscale + 1.0f); y++) {
        GetAgastScore(x, y, threshold);
      }
    }
    // Get the smoothed value.
    return Value(scores_, xf, yf, scale);
  }
}

// Access gray values (smoothed/interpolated).
uint8_t BriskLayer::Value(const cv::Mat& mat, float xf, float yf, float scale) {
  assert(!mat.empty());
  // Get the position.
  const int x = floor(xf);
  const int y = floor(yf);
  const cv::Mat& image = mat;
  const int& imagecols = image.cols;

  // Get the sigma_half:
  const float sigma_half = scale / 2;
  const float area = 4.0 * sigma_half * sigma_half;
  // Calculate output:
  int ret_val;
  if (sigma_half < 0.5) {
    // Interpolation multipliers:
    const int r_x = (xf - x) * 1024;
    const int r_y = (yf - y) * 1024;
    const int r_x_1 = (1024 - r_x);
    const int r_y_1 = (1024 - r_y);
    uchar* ptr = image.data + x + y * imagecols;
    // Just interpolate:
    ret_val = (r_x_1 * r_y_1 * static_cast<int>(*ptr));
    ptr++;
    ret_val += (r_x * r_y_1 * static_cast<int>(*ptr));
    ptr += imagecols;
    ret_val += (r_x * r_y * static_cast<int>(*ptr));
    ptr--;
    ret_val += (r_x_1 * r_y * static_cast<int>(*ptr));
    return 0xFF & ((ret_val + 512) / 1024 / 1024);
  }

  // This is the standard case (simple, not speed optimized yet):

  // Scaling:
  const int scaling = 4194304.0 / area;
  const int scaling2 = static_cast<float>(scaling) * area / 1024.0;

  // Calculate borders.
  const float x_1 = xf - sigma_half;
  const float x1 = xf + sigma_half;
  const float y_1 = yf - sigma_half;
  const float y1 = yf + sigma_half;

  const int x_left = static_cast<int>(x_1 + 0.5);
  const int y_top = static_cast<int>(y_1 + 0.5);
  const int x_right = static_cast<int>(x1 + 0.5);
  const int y_bottom = static_cast<int>(y1 + 0.5);

  // Overlap area - multiplication factors:
  const float r_x_1 = static_cast<float>(x_left) - x_1 + 0.5;
  const float r_y_1 = static_cast<float>(y_top) - y_1 + 0.5;
  const float r_x1 = x1 - static_cast<float>(x_right) + 0.5;
  const float r_y1 = y1 - static_cast<float>(y_bottom) + 0.5;
  const int dx = x_right - x_left - 1;
  const int dy = y_bottom - y_top - 1;
  const int A = (r_x_1 * r_y_1) * scaling;
  const int B = (r_x1 * r_y_1) * scaling;
  const int C = (r_x1 * r_y1) * scaling;
  const int D = (r_x_1 * r_y1) * scaling;
  const int r_x_1_i = r_x_1 * scaling;
  const int r_y_1_i = r_y_1 * scaling;
  const int r_x1_i = r_x1 * scaling;
  const int r_y1_i = r_y1 * scaling;

  // Now the calculation:
  uchar* ptr = image.data + x_left + imagecols * y_top;
  // First row:
  ret_val = A * static_cast<int>(*ptr);
  ptr++;
  const uchar* end1 = ptr + dx;
  for (; ptr < end1; ptr++) {
    ret_val += r_y_1_i * static_cast<int>(*ptr);
  }
  ret_val += B * static_cast<int>(*ptr);
  // Middle ones:
  ptr += imagecols - dx - 1;
  uchar* end_j = ptr + dy * imagecols;
  for (; ptr < end_j; ptr += imagecols - dx - 1) {
    ret_val += r_x_1_i * static_cast<int>(*ptr);
    ptr++;
    const uchar* end2 = ptr + dx;
    for (; ptr < end2; ptr++) {
      ret_val += static_cast<int>(*ptr) * scaling;
    }
    ret_val += r_x1_i * static_cast<int>(*ptr);
  }
  // Last row:
  ret_val += D * static_cast<int>(*ptr);
  ptr++;
  const uchar* end3 = ptr + dx;
  for (; ptr < end3; ptr++) {
    ret_val += r_y1_i * static_cast<int>(*ptr);
  }
  ret_val += C * static_cast<int>(*ptr);

  return 0xFF & ((ret_val + scaling2 / 2) / scaling2 / 1024);
}

// Threshold map.
void BriskLayer::CalculateThresholdMap() {
  // Allocate threshold map.
  cv::Mat tmpmax = cv::Mat::zeros(img_.rows, img_.cols, CV_8U);
  cv::Mat tmpmin = cv::Mat::zeros(img_.rows, img_.cols, CV_8U);
  thrmap_ = cv::Mat::zeros(img_.rows, img_.cols, CV_8U);

  const int rowstride = img_.cols;

  for (int y = 1; y < img_.rows - 1; y++) {
    int x = 1;
    while (x + 16 < img_.cols - 1) {
      // Access.
      uchar* p = img_.data + x - 1 + (y - 1) * rowstride;
#ifdef __ARM_NEON__
      // NEON version.
      uint8x16_t v_1_1 = vld1q_u8(reinterpret_cast<const uint8_t*>(p));
      p++;
      uint8x16_t v0_1 = vld1q_u8(reinterpret_cast<const uint8_t*>(p));
      p++;
      uint8x16_t v1_1 = vld1q_u8(reinterpret_cast<const uint8_t*>(p));
      p += rowstride;
      uint8x16_t v10 = vld1q_u8(reinterpret_cast<const uint8_t*>(p));
      p--;
      uint8x16_t v00 = vld1q_u8(reinterpret_cast<const uint8_t*>(p));
      p--;
      uint8x16_t v_10 = vld1q_u8(reinterpret_cast<const uint8_t*>(p));
      p += rowstride;
      uint8x16_t v_11 = vld1q_u8(reinterpret_cast<const uint8_t*>(p));
      p++;
      uint8x16_t v01 = vld1q_u8(reinterpret_cast<const uint8_t*>(p));
      p++;
      uint8x16_t v11 = vld1q_u8(reinterpret_cast<const uint8_t*>(p));

      // Min/max calculation.
      uint8x16_t max = vmaxq_u8(v_1_1, v0_1);
      uint8x16_t min = vminq_u8(v_1_1, v0_1);
      max = vmaxq_u8(max, v1_1);
      min = vminq_u8(min, v1_1);
      max = vmaxq_u8(max, v10);
      min = vminq_u8(min, v10);
      max = vmaxq_u8(max, v00);
      min = vminq_u8(min, v00);
      max = vmaxq_u8(max, v_10);
      min = vminq_u8(min, v_10);
      max = vmaxq_u8(max, v_11);
      min = vminq_u8(min, v_11);
      max = vmaxq_u8(max, v01);
      min = vminq_u8(min, v01);
      max = vmaxq_u8(max, v11);
      min = vminq_u8(min, v11);

      // Store data back:
      vst1q_u8(reinterpret_cast<uint8_t*>(tmpmax + x + y * rowstride), max);
      vst1q_u8(reinterpret_cast<uint8_t*>(tmpmin + x + y * rowstride), min);
#else
      // SSE version.
      __m128i v_1_1 = _mm_loadu_si128(reinterpret_cast<__m128i *>(p));
      p++;
      __m128i v0_1 = _mm_loadu_si128(reinterpret_cast<__m128i *>(p));
      p++;
      __m128i v1_1 = _mm_loadu_si128(reinterpret_cast<__m128i *>(p));
      p += rowstride;
      __m128i v10 = _mm_loadu_si128(reinterpret_cast<__m128i *>(p));
      p--;
      __m128i v00 = _mm_loadu_si128(reinterpret_cast<__m128i *>(p));
      p--;
      __m128i v_10 = _mm_loadu_si128(reinterpret_cast<__m128i *>(p));
      p += rowstride;
      __m128i v_11 = _mm_loadu_si128(reinterpret_cast<__m128i *>(p));
      p++;
      __m128i v01 = _mm_loadu_si128(reinterpret_cast<__m128i *>(p));
      p++;
      __m128i v11 = _mm_loadu_si128(reinterpret_cast<__m128i *>(p));

      // Min/max calc.
      __m128i max = _mm_max_epu8(v_1_1, v0_1);
      __m128i min = _mm_min_epu8(v_1_1, v0_1);
      max = _mm_max_epu8(max, v1_1);
      min = _mm_min_epu8(min, v1_1);
      max = _mm_max_epu8(max, v10);
      min = _mm_min_epu8(min, v10);
      max = _mm_max_epu8(max, v00);
      min = _mm_min_epu8(min, v00);
      max = _mm_max_epu8(max, v_10);
      min = _mm_min_epu8(min, v_10);
      max = _mm_max_epu8(max, v_11);
      min = _mm_min_epu8(min, v_11);
      max = _mm_max_epu8(max, v01);
      min = _mm_min_epu8(min, v01);
      max = _mm_max_epu8(max, v11);
      min = _mm_min_epu8(min, v11);

      // Store.
      _mm_storeu_si128(
          reinterpret_cast<__m128i *>(tmpmax.data + x + y * rowstride), max);
      _mm_storeu_si128(
          reinterpret_cast<__m128i *>(tmpmin.data + x + y * rowstride), min);
#endif  // __ARM_NEON__
      // Next block.
      x += 16;
    }
  }

  for (int y = 3; y < img_.rows - 3; y++) {
    int x = 3;
    while (x + 16 < img_.cols - 3) {
      // Access.
      uchar* p = img_.data + x + y * rowstride;
#ifdef __ARM_NEON__
      // NEON version //
      uint8x16_t v00 = vld1q_u8(reinterpret_cast<const uint8_t*>(p));
      p -= 2 + 2 * rowstride;
      uint8x16_t v_2_2 = vld1q_u8(reinterpret_cast<const uint8_t*>(p));
      p += 4;
      uint8x16_t v2_2 = vld1q_u8(reinterpret_cast<const uint8_t*>(p));
      p += 4 * rowstride;
      uint8x16_t v22 = vld1q_u8(reinterpret_cast<const uint8_t*>(p));
      p -= 4;
      uint8x16_t v_22 = vld1q_u8(reinterpret_cast<const uint8_t*>(p));

      p = tmpmax + x + (y - 2) * rowstride;
      uint8x16_t max0_2 = vld1q_u8(reinterpret_cast<const uint8_t*>(p));
      p += 4 * rowstride;
      uint8x16_t max02 = vld1q_u8(reinterpret_cast<const uint8_t*>(p));
      p -= 2 * rowstride + 2;
      uint8x16_t max_20 = vld1q_u8(reinterpret_cast<const uint8_t*>(p));
      p += 4;
      uint8x16_t max20 = vld1q_u8(reinterpret_cast<const uint8_t*>(p));

      p = tmpmin + x + (y - 2) * rowstride;
      uint8x16_t min0_2 = vld1q_u8(reinterpret_cast<const uint8_t*>(p));
      p += 4 * rowstride;
      uint8x16_t min02 = vld1q_u8(reinterpret_cast<const uint8_t*>(p));
      p -= 2 * rowstride + 2;
      uint8x16_t min_20 = vld1q_u8(reinterpret_cast<const uint8_t*>(p));
      p += 4;
      uint8x16_t min20 = vld1q_u8(reinterpret_cast<const uint8_t*>(p));

      // Min/max.
      uint8x16_t max = vmaxq_u8(v00, v_2_2);
      uint8x16_t min = vminq_u8(v00, v_2_2);
      max = vmaxq_u8(max, v2_2);
      min = vminq_u8(min, v2_2);
      max = vmaxq_u8(max, v22);
      min = vminq_u8(min, v22);
      max = vmaxq_u8(max, v_22);
      min = vminq_u8(min, v_22);
      max = vmaxq_u8(max, max0_2);
      min = vminq_u8(min, min0_2);
      max = vmaxq_u8(max, max02);
      min = vminq_u8(min, min02);
      max = vmaxq_u8(max, max_20);
      min = vminq_u8(min, min_20);
      max = vmaxq_u8(max, max20);
      min = vminq_u8(min, min20);

      // Store the data back:
      uint8x16_t diff = vsubq_u8(max, min);
      vst1q_u8(reinterpret_cast<uint8_t*> (thrmap_.get() + x + y * rowstride), diff);
#else
      // SSE version.
      __m128i v00 = _mm_loadu_si128(reinterpret_cast<__m128i *>(p));
      p -= 2 + 2 * rowstride;
      __m128i v_2_2 = _mm_loadu_si128(reinterpret_cast<__m128i *>(p));
      p += 4;
      __m128i v2_2 = _mm_loadu_si128(reinterpret_cast<__m128i *>(p));
      p += 4 * rowstride;
      __m128i v22 = _mm_loadu_si128(reinterpret_cast<__m128i *>(p));
      p -= 4;
      __m128i v_22 = _mm_loadu_si128(reinterpret_cast<__m128i *>(p));

      p = tmpmax.data + x + (y - 2) * rowstride;
      __m128i max0_2 = _mm_loadu_si128(reinterpret_cast<__m128i *>(p));
      p += 4 * rowstride;
      __m128i max02 = _mm_loadu_si128(reinterpret_cast<__m128i *>(p));
      p -= 2 * rowstride + 2;
      __m128i max_20 = _mm_loadu_si128(reinterpret_cast<__m128i *>(p));
      p += 4;
      __m128i max20 = _mm_loadu_si128(reinterpret_cast<__m128i *>(p));

      p = tmpmin.data + x + (y - 2) * rowstride;
      __m128i min0_2 = _mm_loadu_si128(reinterpret_cast<__m128i *>(p));
      p += 4 * rowstride;
      __m128i min02 = _mm_loadu_si128(reinterpret_cast<__m128i *>(p));
      p -= 2 * rowstride + 2;
      __m128i min_20 = _mm_loadu_si128(reinterpret_cast<__m128i *>(p));
      p += 4;
      __m128i min20 = _mm_loadu_si128(reinterpret_cast<__m128i *>(p));

      // Min / max.
      __m128i max = _mm_max_epu8(v00, v_2_2);
      __m128i min = _mm_min_epu8(v00, v_2_2);
      max = _mm_max_epu8(max, v2_2);
      min = _mm_min_epu8(min, v2_2);
      max = _mm_max_epu8(max, v22);
      min = _mm_min_epu8(min, v22);
      max = _mm_max_epu8(max, v_22);
      min = _mm_min_epu8(min, v_22);
      max = _mm_max_epu8(max, max0_2);
      min = _mm_min_epu8(min, min0_2);
      max = _mm_max_epu8(max, max02);
      min = _mm_min_epu8(min, min02);
      max = _mm_max_epu8(max, max_20);
      min = _mm_min_epu8(min, min_20);
      max = _mm_max_epu8(max, max20);
      min = _mm_min_epu8(min, min20);

      // Store.
      __m128i diff = _mm_sub_epi8(max, min);
      _mm_storeu_si128(reinterpret_cast<__m128i *>(thrmap_.data + x +
          y * rowstride), diff);
#endif  // __ARM_NEON__
      // Next block.
      x += 16;
    }
  }

  for (int x = std::max(1, 16 * ((img_.cols - 2) / 16) - 16); x < img_.cols - 1;
      x++) {
    for (int y = 1; y < img_.rows - 1; y++) {
      // Access.
      uchar* p = img_.data + x - 1 + (y - 1) * rowstride;
      int v_1_1 = *p;
      p++;
      int v0_1 = *p;
      p++;
      int v1_1 = *p;
      p += rowstride;
      int v10 = *p;
      p--;
      int v00 = *p;
      p--;
      int v_10 = *p;
      p += rowstride;
      int v_11 = *p;
      p++;
      int v01 = *p;
      p++;
      int v11 = *p;

      // Min/max calc.
      int max = std::max(v_1_1, v0_1);
      int min = std::min(v_1_1, v0_1);
      max = std::max(max, v1_1);
      min = std::min(min, v1_1);
      max = std::max(max, v10);
      min = std::min(min, v10);
      max = std::max(max, v00);
      min = std::min(min, v00);
      max = std::max(max, v_10);
      min = std::min(min, v_10);
      max = std::max(max, v_11);
      min = std::min(min, v_11);
      max = std::max(max, v01);
      min = std::min(min, v01);
      max = std::max(max, v11);
      min = std::min(min, v11);

      // Store.
      *(tmpmax.data + x + y * rowstride) = max;
      *(tmpmin.data + x + y * rowstride) = min;
    }
  }

  for (int x = std::max(3, 16 * ((img_.cols - 6) / 16) - 16); x < img_.cols - 3;
      x++) {
    for (int y = 3; y < img_.rows - 3; y++) {
      // Access.
      uchar* p = img_.data + x + y * rowstride;
      int v00 = *p;
      p -= 2 + 2 * rowstride;
      int v_2_2 = *p;
      p += 4;
      int v2_2 = *p;
      p += 4 * rowstride;
      int v22 = *p;
      p -= 4;
      int v_22 = *p;

      p = tmpmax.data + x + (y - 2) * rowstride;
      int max0_2 = *p;
      p += 4 * rowstride;
      int max02 = *p;
      p -= 2 * rowstride + 2;
      int max_20 = *p;
      p += 4;
      int max20 = *p;

      p = tmpmin.data + x + (y - 2) * rowstride;
      int min0_2 = *p;
      p += 4 * rowstride;
      int min02 = *p;
      p -= 2 * rowstride + 2;
      int min_20 = *p;
      p += 4;
      int min20 = *p;

      // Min / max.
      int max = std::max(v00, v_2_2);
      int min = std::min(v00, v_2_2);
      max = std::max(max, v2_2);
      min = std::min(min, v2_2);
      max = std::max(max, v22);
      min = std::min(min, v22);
      max = std::max(max, v_22);
      min = std::min(min, v_22);
      max = std::max(max, max0_2);
      min = std::min(min, min0_2);
      max = std::max(max, max02);
      min = std::min(min, min02);
      max = std::max(max, max_20);
      min = std::min(min, min_20);
      max = std::max(max, max20);
      min = std::min(min, min20);

      // Store.
      *(thrmap_.data + x + y * rowstride) = max - min;
    }
  }
}

void BriskLayer::HalfSample(const cv::Mat& srcimg, cv::Mat& dstimg) {
  // Take care with border...
  const uint16_t leftoverCols = ((srcimg.cols % 16) / 2);
  // Note: leftoverCols can be zero but this still false...
  const bool noleftover = (srcimg.cols % 16) == 0;

  // Make sure the destination image is of the right size:
  assert(srcimg.cols / 2 == dstimg.cols);
  assert(srcimg.rows / 2 == dstimg.rows);

  // Mask needed later:
  register __m128i mask = _mm_set_epi32(0x00FF00FF, 0x00FF00FF, 0x00FF00FF,
                                        0x00FF00FF);
  // To be added in order to make successive averaging correct:
  register __m128i ones = _mm_set_epi8(1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
                                       1, 1);

  // Data pointers:
  __m128i* p1 = reinterpret_cast<__m128i *>(srcimg.data);
  __m128i* p2 = reinterpret_cast<__m128i *>(srcimg.data+srcimg.cols);
  __m128i* p_dest = reinterpret_cast<__m128i *>(dstimg.data);
  unsigned char* p_dest_char;

  // Size:
  const unsigned int size = (srcimg.cols * srcimg.rows) / 16;
  const unsigned int hsize = srcimg.cols / 16;
  __m128i* p_end = p1 + size;
  unsigned int row = 0;
  const unsigned int end = hsize / 2;
  bool half_end;
  if (hsize % 2 == 0)
    half_end = false;
  else
    half_end = true;
  while (p2 < p_end) {
    for (unsigned int i = 0; i < end; i++) {
      // Load the two blocks of memory:
      __m128i upper;
      __m128i lower;
      if (noleftover) {
        upper = _mm_load_si128(p1);
        lower = _mm_load_si128(p2);
      } else {
        upper = _mm_loadu_si128(p1);
        lower = _mm_loadu_si128(p2);
      }

      __m128i result1 = _mm_adds_epu8(upper, ones);
      result1 = _mm_avg_epu8(upper, lower);

      // Increment the pointers:
      p1++;
      p2++;

      // Load the two blocks of memory:
      upper = _mm_loadu_si128(p1);
      lower = _mm_loadu_si128(p2);
      __m128i result2 = _mm_adds_epu8(upper, ones);
      result2 = _mm_avg_epu8(upper, lower);
      // Calculate the shifted versions:
      __m128i result1_shifted = _mm_srli_si128(result1, 1);
      __m128i result2_shifted = _mm_srli_si128(result2, 1);
      // Pack:
      __m128i result = _mm_packus_epi16(_mm_and_si128(result1, mask),
                                        _mm_and_si128(result2, mask));
      __m128i result_shifted = _mm_packus_epi16(
          _mm_and_si128(result1_shifted, mask),
          _mm_and_si128(result2_shifted, mask));
      // Average for the second time:
      result = _mm_avg_epu8(result, result_shifted);

      // Store to memory.
      _mm_storeu_si128(p_dest, result);

      // Increment the pointers:
      p1++;
      p2++;
      p_dest++;
    }
    // If we are not at the end of the row, do the rest:
    if (half_end) {
      // Load the two blocks of memory:
      __m128i upper;
      __m128i lower;
      if (noleftover) {
        upper = _mm_load_si128(p1);
        lower = _mm_load_si128(p2);
      } else {
        upper = _mm_loadu_si128(p1);
        lower = _mm_loadu_si128(p2);
      }

      __m128i result1 = _mm_adds_epu8(upper, ones);
      result1 = _mm_avg_epu8(upper, lower);

      // Increment the pointers:
      p1++;
      p2++;

      // Compute horizontal pairwise average and store.
      p_dest_char = reinterpret_cast<unsigned char*>(p_dest);
      const UCHAR_ALIAS* result = reinterpret_cast<UCHAR_ALIAS*>(&result1);
      for (unsigned int j = 0; j < 8; j++) {
        *(p_dest_char++) = (*(result + 2 * j) + *(result + 2 * j + 1)) / 2;
      }
    } else {
      p_dest_char = reinterpret_cast<unsigned char*>(p_dest);
    }

    if (noleftover) {
      row++;
      p_dest = reinterpret_cast<__m128i*>(dstimg.data + row * dstimg.cols);
      p1 = reinterpret_cast<__m128i*>(srcimg.data + 2 * row * srcimg.cols);
      p2 = p1 + hsize;
    } else {
      const unsigned char* p1_src_char = reinterpret_cast<unsigned char*>(p1);
      const unsigned char* p2_src_char = reinterpret_cast<unsigned char*>(p2);
      for (unsigned int k = 0; k < leftoverCols; k++) {
        uint16_t tmp = p1_src_char[k] + p1_src_char[k + 1]
            + p2_src_char[k] + p2_src_char[k + 1];
        *(p_dest_char++) = static_cast<unsigned char>(tmp / 4);
      }
      // Done with the two rows:
      row++;
      p_dest = reinterpret_cast<__m128i *>(dstimg.data + row * dstimg.cols);
      p1 = reinterpret_cast<__m128i *>(srcimg.data + 2 * row * srcimg.cols);
      p2 = reinterpret_cast<__m128i *>(srcimg.data + (2 * row + 1) *
          srcimg.cols);
    }
  }
}

void BriskLayer::TwoThirdSample(const cv::Mat& srcimg, cv::Mat& dstimg) {
  // Take care with the border...
  const uint16_t leftoverCols = ((srcimg.cols / 3) * 3) % 15;

  // Make sure the destination image is of the right size:
  assert((srcimg.cols / 3) * 2 == dstimg.cols);
  assert((srcimg.rows / 3) * 2 == dstimg.rows);

  // Masks:
  register __m128i mask1 = _mm_set_epi8(0x80, 0x80, 0x80, 0x80, 0x80, 0x80,
                                        0x80, 12, 0x80, 10, 0x80, 7, 0x80, 4,
                                        0x80, 1);
  register __m128i mask2 = _mm_set_epi8(0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 12,
                                        0x80, 10, 0x80, 7, 0x80, 4, 0x80, 1,
                                        0x80);
  register __m128i mask = _mm_set_epi8(0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 14,
                                       12, 11, 9, 8, 6, 5, 3, 2, 0);
  register __m128i store_mask = _mm_set_epi8(0, 0, 0, 0, 0, 0, 0x80, 0x80, 0x80,
                                             0x80, 0x80, 0x80, 0x80, 0x80, 0x80,
                                             0x80);

  // Data pointers:
  unsigned char* p1 = srcimg.data;
  unsigned char* p2 = p1 + srcimg.cols;
  unsigned char* p3 = p2 + srcimg.cols;
  unsigned char* p_dest1 = dstimg.data;
  unsigned char* p_dest2 = p_dest1 + dstimg.cols;
  unsigned char* p_end = p1 + (srcimg.cols * srcimg.rows);

  unsigned int row = 0;
  unsigned int row_dest = 0;
  int hsize = srcimg.cols / 15;
  while (p3 < p_end) {
    for (int i = 0; i < hsize; i++) {
      // Load three rows.
      __m128i first = _mm_loadu_si128(reinterpret_cast<__m128i *>(p1));
      __m128i second = _mm_loadu_si128(reinterpret_cast<__m128i *>(p2));
      __m128i third = _mm_loadu_si128(reinterpret_cast<__m128i *>(p3));

      // Upper row:
      __m128i upper = _mm_avg_epu8(_mm_avg_epu8(first, second), first);
      __m128i temp1_upper = _mm_or_si128(_mm_shuffle_epi8(upper, mask1),
                                         _mm_shuffle_epi8(upper, mask2));
      __m128i temp2_upper = _mm_shuffle_epi8(upper, mask);
      __m128i result_upper = _mm_avg_epu8(
          _mm_avg_epu8(temp2_upper, temp1_upper), temp2_upper);

      // Lower row:
      __m128i lower = _mm_avg_epu8(_mm_avg_epu8(third, second), third);
      __m128i temp1_lower = _mm_or_si128(_mm_shuffle_epi8(lower, mask1),
                                         _mm_shuffle_epi8(lower, mask2));
      __m128i temp2_lower = _mm_shuffle_epi8(lower, mask);
      __m128i result_lower = _mm_avg_epu8(
          _mm_avg_epu8(temp2_lower, temp1_lower), temp2_lower);

      // Store:
      if (i * 10 + 16 > dstimg.cols) {
        _mm_maskmoveu_si128(result_upper, store_mask,
                            reinterpret_cast<char*>(p_dest1));
        _mm_maskmoveu_si128(result_lower, store_mask,
                            reinterpret_cast<char*>(p_dest2));
      } else {
        _mm_storeu_si128(reinterpret_cast<__m128i *>(p_dest1), result_upper);
        _mm_storeu_si128(reinterpret_cast<__m128i *>(p_dest2), result_lower);
      }

      // Shift pointers:
      p1 += 15;
      p2 += 15;
      p3 += 15;
      p_dest1 += 10;
      p_dest2 += 10;
    }

    // Fill the remainder:
    for (unsigned int j = 0; j < leftoverCols; j += 3) {
      const uint16_t A1 = *(p1++);
      const uint16_t A2 = *(p1++);
      const uint16_t A3 = *(p1++);
      const uint16_t B1 = *(p2++);
      const uint16_t B2 = *(p2++);
      const uint16_t B3 = *(p2++);
      const uint16_t C1 = *(p3++);
      const uint16_t C2 = *(p3++);
      const uint16_t C3 = *(p3++);

      *(p_dest1++) = static_cast<unsigned char>(((4 * A1 + 2 * (A2 + B1) + B2)
          / 9) & 0x00FF);
      *(p_dest1++) = static_cast<unsigned char>(((4 * A3 + 2 * (A2 + B3) + B2)
          / 9) & 0x00FF);
      *(p_dest2++) = static_cast<unsigned char>(((4 * C1 + 2 * (C2 + B1) + B2)
          / 9) & 0x00FF);
      *(p_dest2++) = static_cast<unsigned char>(((4 * C3 + 2 * (C2 + B3) + B2)
          / 9) & 0x00FF);
    }

    // Increment row counter:
    row += 3;
    row_dest += 2;

    // Reset pointers
    p1 = srcimg.data + row * srcimg.cols;
    p2 = p1 + srcimg.cols;
    p3 = p2 + srcimg.cols;
    p_dest1 = dstimg.data + row_dest * dstimg.cols;
    p_dest2 = p_dest1 + dstimg.cols;
  }
}
}  // namespace brisk
