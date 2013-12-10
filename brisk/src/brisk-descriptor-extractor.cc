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

#include <fstream>

#include <brisk/brisk-descriptor-extractor.h>
#include <brisk/brisk-opencv.h>
#include <brisk/internal/helper-structures.h>
#include <brisk/internal/macros.h>
#include <brisk/internal/rdtsc-wrapper.h>

namespace cv {
const float BriskDescriptorExtractor::basicSize_ = 12.0;
const unsigned int BriskDescriptorExtractor::scales_ = 64;
// 40->4 Octaves - else, this needs to be adjusted...
const float BriskDescriptorExtractor::scalerange_ = 30;
// Discretization of the rotation look-up.
const unsigned int BriskDescriptorExtractor::n_rot_ = 1024;

BriskDescriptorExtractor::BriskDescriptorExtractor(bool rotationInvariant,
                                                   bool scaleInvariant,
                                                   float patternScale) {
  std::vector<float> rList;
  std::vector<int> nList;

  // This is the standard pattern found to be suitable also.
  rList.resize(5);
  nList.resize(5);
  const double f = 0.85 * patternScale;

  rList[0] = f * 0;
  rList[1] = f * 2.9;
  rList[2] = f * 4.9;
  rList[3] = f * 7.4;
  rList[4] = f * 10.8;

  nList[0] = 1;
  nList[1] = 10;
  nList[2] = 14;
  nList[3] = 15;
  nList[4] = 20;

  rotationInvariance = rotationInvariant;
  scaleInvariance = scaleInvariant;
  generateKernel(rList, nList, 5.85 * patternScale, 8.2 * patternScale);

}
BriskDescriptorExtractor::BriskDescriptorExtractor(
    std::vector<float> &radiusList, std::vector<int> &numberList,
    bool rotationInvariant, bool scaleInvariant, float dMax, float dMin,
    std::vector<int> indexChange) {
  rotationInvariance = rotationInvariant;
  scaleInvariance = scaleInvariant;
  generateKernel(radiusList, numberList, dMax, dMin, indexChange);
}

BriskDescriptorExtractor::BriskDescriptorExtractor(const std::string& fname,
                                                   bool rotationInvariant,
                                                   bool scaleInvariant) {

  rotationInvariance = rotationInvariant;
  scaleInvariance = scaleInvariant;

  // Not in use.
  dMax_ = 0;
  dMin_ = 0;

  std::ifstream myfile(fname.c_str());
  assert(myfile.is_open());

  // Read number of points.
  myfile >> points_;

  // Set up the patterns.
  patternPoints_ = new brisk::BriskPatternPoint[points_ * scales_ * n_rot_];
  brisk::BriskPatternPoint* patternIterator = patternPoints_;

  // Define the scale discretization:
  static const float lb_scale = log(scalerange_) / log(2.0);
  static const float lb_scale_step = lb_scale / (scales_);

  scaleList_ = new float[scales_];
  sizeList_ = new unsigned int[scales_];

  const float sigma_scale = 1.3;

  // First fill the unscaled and unrotated pattern:
  float* u_x = new float[points_];
  float* u_y = new float[points_];
  float* sigma = new float[points_];
  for (unsigned int i = 0; i < points_; i++) {
    myfile >> u_x[i];
    myfile >> u_y[i];
    myfile >> sigma[i];
  }

  // Now fill all the scaled and rotated versions.
  for (unsigned int scale = 0; scale < scales_; ++scale) {
    scaleList_[scale] = pow((double) 2.0, (double) (scale * lb_scale_step));
    sizeList_[scale] = 0;

    // Generate the pattern points look-up.
    double theta;
    for (size_t rot = 0; rot < n_rot_; ++rot) {
      for (unsigned int i = 0; i < points_; i++) {
        // This is the rotation of the feature.
        theta = double(rot) * 2 * M_PI / double(n_rot_);
        // Feature rotation plus angle of the point.
        patternIterator->x = scaleList_[scale]
            * (u_x[i] * cos(theta) - u_y[i] * sin(theta));
        patternIterator->y = scaleList_[scale]
            * (u_x[i] * sin(theta) + u_y[i] * cos(theta));
        // And the Gaussian kernel sigma.
        patternIterator->sigma = sigma_scale * scaleList_[scale] * sigma[i];

        // Adapt the sizeList if necessary.
        const unsigned int size = ceil(
            ((sqrt(
                patternIterator->x * patternIterator->x
                    + patternIterator->y * patternIterator->y))
                + patternIterator->sigma)) + 1;
        if (sizeList_[scale] < size) {
          sizeList_[scale] = size;
        }

        // Increment the iterator.
        ++patternIterator;
      }
    }
  }

  // Now also generate pairings.
  myfile >> noShortPairs_;
  shortPairs_ = new brisk::BriskShortPair[noShortPairs_];
  for (unsigned int p = 0; p < noShortPairs_; p++) {
    unsigned int i, j;
    myfile >> i;
    shortPairs_[p].i = i;
    myfile >> j;
    shortPairs_[p].j = j;
  }

  myfile >> noLongPairs_;
  longPairs_ = new brisk::BriskLongPair[noLongPairs_];
  for (unsigned int p = 0; p < noLongPairs_; p++) {
    unsigned int i, j;
    myfile >> i;
    longPairs_[p].i = i;
    myfile >> j;
    longPairs_[p].j = j;
    float dx = (u_x[j] - u_x[i]);
    float dy = (u_y[j] - u_y[i]);
    float norm_sq = dx * dx + dy * dy;
    longPairs_[p].weighted_dx = int((dx / (norm_sq)) * 2048.0 + 0.5);
    longPairs_[p].weighted_dy = int((dy / (norm_sq)) * 2048.0 + 0.5);
  }

  // Number of descriptor bits:
  strings_ = (int) ceil((float(noShortPairs_)) / 128.0) * 4 * 4;

  // Clean up.
  myfile.close();
}

void BriskDescriptorExtractor::generateKernel(std::vector<float> &radiusList,
                                              std::vector<int> &numberList,
                                              float dMax, float dMin,
                                              std::vector<int> indexChange) {
  dMax_ = dMax;
  dMin_ = dMin;

  // Get the total number of points.
  const int rings = radiusList.size();
  assert(radiusList.size() != 0 && radiusList.size() == numberList.size());
  points_ = 0;  // Remember the total number of points.
  for (int ring = 0; ring < rings; ring++) {
    points_ += numberList[ring];
  }
  // Set up the patterns.
  patternPoints_ = new brisk::BriskPatternPoint[points_ * scales_ * n_rot_];
  brisk::BriskPatternPoint* patternIterator = patternPoints_;

  // Define the scale discretization:
  static const float lb_scale = log(scalerange_) / log(2.0);
  static const float lb_scale_step = lb_scale / (scales_);

  scaleList_ = new float[scales_];
  sizeList_ = new unsigned int[scales_];

  const float sigma_scale = 1.3;

  for (unsigned int scale = 0; scale < scales_; ++scale) {
    scaleList_[scale] = pow((double) 2.0, (double) (scale * lb_scale_step));
    sizeList_[scale] = 0;

    // Generate the pattern points look-up.
    double alpha, theta;
    for (size_t rot = 0; rot < n_rot_; ++rot) {
      // This is the rotation of the feature.
      theta = double(rot) * 2 * M_PI / double(n_rot_);
      for (int ring = 0; ring < rings; ++ring) {
        for (int num = 0; num < numberList[ring]; ++num) {
          // The actual coordinates on the circle.
          alpha = (double(num)) * 2 * M_PI / double(numberList[ring]);
          // Feature rotation plus angle of the point.
          patternIterator->x = scaleList_[scale] * radiusList[ring]
              * cos(alpha + theta);
          patternIterator->y = scaleList_[scale] * radiusList[ring]
              * sin(alpha + theta);
          // And the gaussian kernel sigma.
          if (ring == 0) {
            patternIterator->sigma = sigma_scale * scaleList_[scale] * 0.5;
          } else {
            patternIterator->sigma = sigma_scale * scaleList_[scale]
                * (double(radiusList[ring])) * sin(M_PI / numberList[ring]);
          }
          // Adapt the sizeList if necessary.
          const unsigned int size = ceil(
              ((scaleList_[scale] * radiusList[ring]) + patternIterator->sigma))
              + 1;
          if (sizeList_[scale] < size) {
            sizeList_[scale] = size;
          }

          // Increment the iterator.
          ++patternIterator;
        }
      }
    }
  }

  // Now also generate pairings.
  shortPairs_ = new brisk::BriskShortPair[points_ * (points_ - 1) / 2];
  longPairs_ = new brisk::BriskLongPair[points_ * (points_ - 1) / 2];
  noShortPairs_ = 0;
  noLongPairs_ = 0;

  // Fill indexChange with 0..n if empty.
  unsigned int indSize = indexChange.size();
  if (indSize == 0) {
    indexChange.resize(points_ * (points_ - 1) / 2);
    indSize = indexChange.size();
    for (unsigned int i = 0; i < indSize; i++) {
      indexChange[i] = i;
    }
  }
  const float dMin_sq = dMin_ * dMin_;
  const float dMax_sq = dMax_ * dMax_;
  for (unsigned int i = 1; i < points_; i++) {
    for (unsigned int j = 0; j < i; j++) {  // Find all the pairs.
      // Point pair distance:
      const float dx = patternPoints_[j].x - patternPoints_[i].x;
      const float dy = patternPoints_[j].y - patternPoints_[i].y;
      const float norm_sq = (dx * dx + dy * dy);
      if (norm_sq > dMin_sq) {
        // Save to long pairs.
        brisk::BriskLongPair& longPair = longPairs_[noLongPairs_];
        longPair.weighted_dx = int((dx / (norm_sq)) * 2048.0 + 0.5);
        longPair.weighted_dy = int((dy / (norm_sq)) * 2048.0 + 0.5);
        longPair.i = i;
        longPair.j = j;
        ++noLongPairs_;
      }
      if (norm_sq < dMax_sq) {
        // Save to short pairs.
        // Make sure the user passes something sensible.
        assert(noShortPairs_ < indSize);
        brisk::BriskShortPair& shortPair =
            shortPairs_[indexChange[noShortPairs_]];
        shortPair.j = j;
        shortPair.i = i;
        ++noShortPairs_;
      }
    }
  }

  // Number of descriptor bits:
  strings_ = (int) ceil((float(noShortPairs_)) / 128.0) * 4 * 4;
}

// Simple alternative:
template<typename ImgPixel_T, typename IntegralPixel_T>
__inline__ IntegralPixel_T BriskDescriptorExtractor::smoothedIntensity(
    const cv::Mat& image, const cv::Mat& integral, const float key_x,
    const float key_y, const unsigned int scale, const unsigned int rot,
    const unsigned int point) const {

  // Get the float position.
  const brisk::BriskPatternPoint& briskPoint =
      patternPoints_[scale * n_rot_ * points_ + rot * points_ + point];

  const float xf = briskPoint.x + key_x;
  const float yf = briskPoint.y + key_y;
  const int x = int(xf);
  const int y = int(yf);
  const int& imagecols = image.cols;

  // Get the sigma:
  const float sigma_half = briskPoint.sigma;
  const float area = 4.0 * sigma_half * sigma_half;

  // Calculate output:
  int ret_val;
  if (sigma_half < 0.5) {
    // Interpolation multipliers:
    const int r_x = (xf - x) * 1024;
    const int r_y = (yf - y) * 1024;
    const int r_x_1 = (1024 - r_x);
    const int r_y_1 = (1024 - r_y);
    ImgPixel_T* ptr = (ImgPixel_T*) image.data + x + y * imagecols;
    // Just interpolate:
    ret_val = (r_x_1 * r_y_1 * IntegralPixel_T(*ptr));
    ptr++;
    ret_val += (r_x * r_y_1 * IntegralPixel_T(*ptr));
    ptr += imagecols;
    ret_val += (r_x * r_y * IntegralPixel_T(*ptr));
    ptr--;
    ret_val += (r_x_1 * r_y * IntegralPixel_T(*ptr));
    return (ret_val) / 1024;
  }

  // This is the standard case (simple, not speed optimized yet):
  // Scaling:
  const IntegralPixel_T scaling = 4194304.0 / area;
  const IntegralPixel_T scaling2 = float(scaling) * area / 1024.0;

  // The integral image is larger:
  const int integralcols = imagecols + 1;

  // Calculate borders.
  const float x_1 = xf - sigma_half;
  const float x1 = xf + sigma_half;
  const float y_1 = yf - sigma_half;
  const float y1 = yf + sigma_half;

  const int x_left = int(x_1 + 0.5);
  const int y_top = int(y_1 + 0.5);
  const int x_right = int(x1 + 0.5);
  const int y_bottom = int(y1 + 0.5);

  // Overlap area - multiplication factors:
  const float r_x_1 = float(x_left) - x_1 + 0.5;
  const float r_y_1 = float(y_top) - y_1 + 0.5;
  const float r_x1 = x1 - float(x_right) + 0.5;
  const float r_y1 = y1 - float(y_bottom) + 0.5;
  const int dx = x_right - x_left - 1;
  const int dy = y_bottom - y_top - 1;
  const IntegralPixel_T A = (r_x_1 * r_y_1) * scaling;
  const IntegralPixel_T B = (r_x1 * r_y_1) * scaling;
  const IntegralPixel_T C = (r_x1 * r_y1) * scaling;
  const IntegralPixel_T D = (r_x_1 * r_y1) * scaling;
  const IntegralPixel_T r_x_1_i = r_x_1 * scaling;
  const IntegralPixel_T r_y_1_i = r_y_1 * scaling;
  const IntegralPixel_T r_x1_i = r_x1 * scaling;
  const IntegralPixel_T r_y1_i = r_y1 * scaling;

  if (dx + dy > 2) {
    // Now the calculation:
    ImgPixel_T* ptr = (ImgPixel_T*) image.data + x_left + imagecols * y_top;
    // First the corners:
    ret_val = A * IntegralPixel_T(*ptr);
    ptr += dx + 1;
    ret_val += B * IntegralPixel_T(*ptr);
    ptr += dy * imagecols + 1;
    ret_val += C * IntegralPixel_T(*ptr);
    ptr -= dx + 1;
    ret_val += D * IntegralPixel_T(*ptr);

    // Next the edges:
    IntegralPixel_T* ptr_integral = (IntegralPixel_T*) integral.data + x_left
        + integralcols * y_top + 1;
    // Find a simple path through the different surface corners.
    const IntegralPixel_T tmp1 = (*ptr_integral);
    ptr_integral += dx;
    const IntegralPixel_T tmp2 = (*ptr_integral);
    ptr_integral += integralcols;
    const IntegralPixel_T tmp3 = (*ptr_integral);
    ptr_integral++;
    const IntegralPixel_T tmp4 = (*ptr_integral);
    ptr_integral += dy * integralcols;
    const IntegralPixel_T tmp5 = (*ptr_integral);
    ptr_integral--;
    const IntegralPixel_T tmp6 = (*ptr_integral);
    ptr_integral += integralcols;
    const IntegralPixel_T tmp7 = (*ptr_integral);
    ptr_integral -= dx;
    const IntegralPixel_T tmp8 = (*ptr_integral);
    ptr_integral -= integralcols;
    const IntegralPixel_T tmp9 = (*ptr_integral);
    ptr_integral--;
    const IntegralPixel_T tmp10 = (*ptr_integral);
    ptr_integral -= dy * integralcols;
    const IntegralPixel_T tmp11 = (*ptr_integral);
    ptr_integral++;
    const IntegralPixel_T tmp12 = (*ptr_integral);

    // Assign the weighted surface integrals:
    const IntegralPixel_T upper = (tmp3 - tmp2 + tmp1 - tmp12) * r_y_1_i;
    const IntegralPixel_T middle = (tmp6 - tmp3 + tmp12 - tmp9) * scaling;
    const IntegralPixel_T left = (tmp9 - tmp12 + tmp11 - tmp10) * r_x_1_i;
    const IntegralPixel_T right = (tmp5 - tmp4 + tmp3 - tmp6) * r_x1_i;
    const IntegralPixel_T bottom = (tmp7 - tmp6 + tmp9 - tmp8) * r_y1_i;

    return IntegralPixel_T(
        (ret_val + upper + middle + left + right + bottom) / scaling2);
  }

  // Now the calculation:
  ImgPixel_T* ptr = (ImgPixel_T*) image.data + x_left + imagecols * y_top;
  // First row:
  ret_val = A * IntegralPixel_T(*ptr);
  ptr++;
  const ImgPixel_T* end1 = ptr + dx;
  for (; ptr < end1; ptr++) {
    ret_val += r_y_1_i * IntegralPixel_T(*ptr);
  }
  ret_val += B * IntegralPixel_T(*ptr);
  // Middle ones:
  ptr += imagecols - dx - 1;
  const ImgPixel_T* end_j = ptr + dy * imagecols;
  for (; ptr < end_j; ptr += imagecols - dx - 1) {
    ret_val += r_x_1_i * IntegralPixel_T(*ptr);
    ptr++;
    const ImgPixel_T* end2 = ptr + dx;
    for (; ptr < end2; ptr++) {
      ret_val += IntegralPixel_T(*ptr) * scaling;
    }
    ret_val += r_x1_i * IntegralPixel_T(*ptr);
  }
  // Last row:
  ret_val += D * IntegralPixel_T(*ptr);
  ptr++;
  const ImgPixel_T* end3 = ptr + dx;
  for (; ptr < end3; ptr++) {
    ret_val += r_y1_i * IntegralPixel_T(*ptr);
  }
  ret_val += C * IntegralPixel_T(*ptr);

  return IntegralPixel_T((ret_val) / scaling2);
}

bool RoiPredicate(const float minX, const float minY, const float maxX,
                  const float maxY, const KeyPoint& keyPt) {
  const Point2f& pt = keyPt.pt;
  return (pt.x < minX) || (pt.x >= maxX) || (pt.y < minY) || (pt.y >= maxY);
}

// Computes the descriptor.
void BriskDescriptorExtractor::computeImpl(const Mat& image,
                                           std::vector<KeyPoint>& keypoints,
                                           Mat& descriptors) const {

  // Remove keypoints very close to the border.
  size_t ksize = keypoints.size();
  std::vector<int> kscales;  // Remember the scale per keypoint.
  kscales.resize(ksize);
  static const float log2 = 0.693147180559945;
  static const float lb_scalerange = log(scalerange_) / (log2);

  std::vector < KeyPoint > valid_kp;
  std::vector<int> valid_scales;
  valid_kp.reserve(keypoints.size());
  valid_scales.reserve(keypoints.size());

  static const float basicSize06 = basicSize_ * 0.6;
  unsigned int basicscale = 0;
  if (!scaleInvariance)
    basicscale = std::max(
        (int) (scales_ / lb_scalerange
            * (log(1.45 * basicSize_ / (basicSize06)) / log2) + 0.5),
        0);
  for (size_t k = 0; k < ksize; k++) {
    unsigned int scale;
    if (scaleInvariance) {
      scale = std::max(
          (int) (scales_ / lb_scalerange
              * (log(keypoints[k].size / (basicSize06)) / log2) + 0.5),
          0);
      // Saturate.
      if (scale >= scales_)
        scale = scales_ - 1;
      kscales[k] = scale;
    } else {
      scale = basicscale;
      kscales[k] = scale;
    }
    const int border = sizeList_[scale];
    const int border_x = image.cols - border;
    const int border_y = image.rows - border;
    if (!RoiPredicate(border, border, border_x, border_y, keypoints[k])) {
      valid_kp.push_back(keypoints[k]);
      valid_scales.push_back(kscales[k]);
    }
  }

  keypoints.swap(valid_kp);
  kscales.swap(valid_scales);
  ksize = keypoints.size();

  // First, calculate the integral image over the whole image:
  // current integral image.
  brisk::TimerSwitchable timer_integral_image(
      "1.0 Brisk Extraction: integral computation");
  cv::Mat _integral;  // The integral image.
  cv::Mat imageScaled;
  if (image.type() == CV_16UC1) {
    // 16 bit image - convert to float. this is simple but not the fastest...
    cv::Mat imageCvt;
    image.convertTo(imageCvt, CV_32FC1);
    imageScaled = imageCvt / 65536.0;
    //TODO(slynen): Put in the optimized integral computation.
    cv::integral(imageScaled, _integral, CV_32F);
  } else if (image.type() == CV_8UC1) {
    //TODO(slynen): Put in the optimized integral computation.
    cv::integral(image, _integral);
  } else {
    std::cout << "unsupported image format" << std::endl;
    std::cout.flush();
    exit(-1);
  }
  timer_integral_image.stop();

  int* _values = new int[points_];  // For temporary use.

  // Resize the descriptors:
  descriptors = cv::Mat::zeros(ksize, strings_, CV_8U);

  // Now do the extraction for all keypoints:

  // Temporary variables containing gray values at sample points:
  int t1;
  int t2;

  // The feature orientation.
  int direction0;
  int direction1;

  uchar* ptr = descriptors.data;
  for (size_t k = 0; k < ksize; k++) {
    int theta;
    cv::KeyPoint& kp = keypoints[k];
    const int& scale = kscales[k];
    int shifter = 0;
    int* pvalues = _values;
    const float& x = kp.pt.x;
    const float& y = kp.pt.y;
    if (kp.angle == -1) {
      if (!rotationInvariance) {
        // Don't compute the gradient direction, just assign a rotation of 0°.
        theta = 0;
      } else {
        // Get the gray values in the unrotated pattern.
        brisk::TimerSwitchable timerFancyOp11(
            "1.1.1 Brisk Extraction: rotation determination: sample points (per keypoint)");
        if (image.type() == CV_8UC1) {
          for (unsigned int i = 0; i < points_; i++) {
            *(pvalues++) = smoothedIntensity<uchar, int>(image, _integral, x, y,
                                                         scale, 0, i);
          }
        } else {
          for (unsigned int i = 0; i < points_; i++) {
            *(pvalues++) = int(
                65536.0
                    * smoothedIntensity<float, float>(imageScaled, _integral, x,
                                                      y, scale, 0, i));
          }
        }
        timerFancyOp11.stop();
        direction0 = 0;
        direction1 = 0;
        // Now iterate through the long pairings.
        brisk::TimerSwitchable timerFancyOp12(
            "1.1.2 Brisk Extraction: rotation determination: calculate gradient (per keypoint)");
        const brisk::BriskLongPair* max = longPairs_ + noLongPairs_;
        for (brisk::BriskLongPair* iter = longPairs_; iter < max; ++iter) {
          t1 = *(_values + iter->i);
          t2 = *(_values + iter->j);
          const int delta_t = (t1 - t2);
          // Update the direction:
          const int tmp0 = delta_t * (iter->weighted_dx) / 1024;
          const int tmp1 = delta_t * (iter->weighted_dy) / 1024;
          direction0 += tmp0;
          direction1 += tmp1;
        }
        timerFancyOp12.stop();
        kp.angle = atan2((float) direction1, (float) direction0) / M_PI * 180.0;
        theta = int((n_rot_ * kp.angle) / (360.0) + 0.5);
        if (theta < 0)
          theta += n_rot_;
        if (theta >= int(n_rot_))
          theta -= n_rot_;
      }
    } else {
      // Figure out the direction:
      if (!rotationInvariance) {
        theta = 0;
      } else {
        theta = (int) (n_rot_ * (kp.angle / (360.0)) + 0.5);
        if (theta < 0)
          theta += n_rot_;
        if (theta >= int(n_rot_))
          theta -= n_rot_;
      }
    }

    // Now also extract the stuff for the actual direction:
    // Let us compute the smoothed values.
    shifter = 0;
    pvalues = _values;
    // Get the gray values in the rotated pattern.
    brisk::TimerSwitchable timerFancyOp2(
        "1.2 Brisk Extraction: sample points (per keypoint)");
    if (image.type() == CV_8UC1) {
      for (unsigned int i = 0; i < points_; i++) {
        *(pvalues++) = smoothedIntensity<uchar, int>(image, _integral, x, y,
                                                     scale, theta, i);
      }
    } else {
      for (unsigned int i = 0; i < points_; i++) {
        *(pvalues++) = int(
            65536.0
                * smoothedIntensity<float, float>(imageScaled, _integral, x, y,
                                                  scale, theta, i));
      }
    }
    timerFancyOp2.stop();

    // Now iterate through all the pairings.
    brisk::TimerSwitchable timerFancyOp3(
        "1.3 Brisk Extraction: assemble bits (per keypoint)");
    brisk::UINT32_ALIAS* ptr2 = (brisk::UINT32_ALIAS*) ptr;
    const brisk::BriskShortPair* max = shortPairs_ + noShortPairs_;
    for (brisk::BriskShortPair* iter = shortPairs_; iter < max; ++iter) {
      t1 = *(_values + iter->i);
      t2 = *(_values + iter->j);
      if (t1 > t2) {
        *ptr2 |= ((1) << shifter);
      }  // Else already initialized with zero.
      // Take care of the iterators:
      ++shifter;
      if (shifter == 32) {
        shifter = 0;
        ++ptr2;
      }
    }
    timerFancyOp3.stop();

    ptr += strings_;
  }

  // Clean-up.
  _integral.release();
  delete[] _values;
}

int BriskDescriptorExtractor::descriptorSize() const {
  return strings_;
}

int BriskDescriptorExtractor::descriptorType() const {
  return CV_8U;
}

BriskDescriptorExtractor::~BriskDescriptorExtractor() {
  delete[] patternPoints_;
  delete[] shortPairs_;
  delete[] longPairs_;
  delete[] scaleList_;
  delete[] sizeList_;
}
}  // namespace cv