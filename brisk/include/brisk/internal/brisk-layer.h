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

#ifndef BRISK_INTERNAL_BRISK_LAYER_H_
#define BRISK_INTERNAL_BRISK_LAYER_H_

#include <memory>

#include <agast/agast5_8.h>
#include <agast/oast9_16.h>
#include <brisk/brisk-opencv.h>
#include <brisk/internal/macros.h>

namespace brisk {
// A layer in the Brisk detector pyramid.
class CV_EXPORTS BriskLayer {
 public:
  // Constructor arguments.
  struct CV_EXPORTS CommonParams {
    static const int HALFSAMPLE = 0;
    static const int TWOTHIRDSAMPLE = 1;
  };
  // Construct a base layer.
  BriskLayer(const cv::Mat& img, uchar upperThreshold, uchar lowerThreshold,
             float scale = 1.0f, float offset = 0.0f);
  // Derive a layer.
  BriskLayer(const BriskLayer& layer, int mode, uchar upperThreshold,
             uchar lowerThreshold);

  // Fast/Agast without non-max suppression.
  void getAgastPoints(uint8_t threshold, std::vector<CvPoint>& keypoints);

  // Get scores - attention, this is in layer coordinates, not scale=1 coordinates!
  uint8_t getAgastScore(int x, int y, uint8_t threshold);
  uint8_t getAgastScore_5_8(int x, int y, uint8_t threshold);
  uint8_t getAgastScore(float xf, float yf, uint8_t threshold, float scale =
                            1.0f);

  // Accessors.
  inline const cv::Mat& img() const {
    return img_;
  }
  inline const cv::Mat& scores() const {
    return scores_;
  }
  inline float scale() const {
    return scale_;
  }
  inline float offset() const {
    return offset_;
  }

  // Half sampling.
  static void halfsample(const cv::Mat& srcimg, cv::Mat& dstimg);
  // Two third sampling.
  static void twothirdsample(const cv::Mat& srcimg, cv::Mat& dstimg);

 private:
  // Access gray values (smoothed/interpolated).
  uint8_t value(const cv::Mat& mat, float xf, float yf, float scale);
  // Calculate threshold map.
  void calculateThresholdMap();
  // The image.
  cv::Mat img_;
  // Its Fast scores.
  cv::Mat scores_;
  // Its threshold map.
  cv::Mat thrmap_;
  // coordinate transformation.
  float scale_;
  float offset_;
  // Agast detectors.
  std::shared_ptr<agast::OastDetector9_16> oastDetector_;
  std::shared_ptr<agast::AgastDetector5_8> agastDetector_5_8_;

  uchar upperThreshold_;
  uchar lowerThreshold_;
};
}  // namespace brisk
#endif  // BRISK_INTERNAL_BRISK_LAYER_H_
