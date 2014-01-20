//
//    AGAST, an adaptive and generic corner detector based on the
//              accelerated segment test for a 8 pixel mask
//
//    Copyright (C) 2010  Elmar Mair
//    All rights reserved.
//
//    Redistribution and use in source and binary forms, with or without
//    modification, are permitted provided that the following conditions are met:
//        * Redistributions of source code must retain the above copyright
//          notice, this list of conditions and the following disclaimer.
//        * Redistributions in binary form must reproduce the above copyright
//          notice, this list of conditions and the following disclaimer in the
//          documentation and/or other materials provided with the distribution.
//        * Neither the name of the <organization> nor the
//          names of its contributors may be used to endorse or promote products
//          derived from this software without specific prior written permission.
//
//    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
//    ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
//    WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
//    DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
//    DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
//    (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
//    LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
//    ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
//    (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
//    SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#ifndef CLOSED_FEATURES2D_AGAST_ASTDETECTOR_H
#define CLOSED_FEATURES2D_AGAST_ASTDETECTOR_H

#include <iostream>
#include <vector>

#if HAVE_OPENCV
#include <opencv2/features2d/features2d.hpp>
#else
#include <features-2d-helpers/keypoint.h>
#endif

namespace agast {

#if HAVE_OPENCV
typedef cv::KeyPoint KeyPoint;
inline float& KeyPointX(KeyPoint& keypoint) {  // NOLINT
  return keypoint.pt.x;
}
inline float& KeyPointY(KeyPoint& keypoint) {  // NOLINT
  return keypoint.pt.y;
}
inline const float& KeyPointX(const KeyPoint& keypoint) {
  return keypoint.pt.x;
}
inline const float& KeyPointY(const KeyPoint& keypoint) {
  return keypoint.pt.y;
}
#else
typedef features_2d::Keypoint KeyPoint;
inline float& KeyPointX(KeyPoint& keypoint) {  // NOLINT
  return keypoint.x;
}
inline float& KeyPointY(KeyPoint& keypoint) {  // NOLINT
  return keypoint.y;
}
inline const float& KeyPointX(const KeyPoint& keypoint) {
  return keypoint.x;
}
inline const float& KeyPointY(const KeyPoint& keypoint) {
  return keypoint.y;
}
#endif

class AstDetector {
 public:
  AstDetector(int width, int height)
      : xsize_(width),
        ysize_(height),
        b_(-1) { }
  AstDetector(int width, int height, int thr)
      : xsize_(width),
        ysize_(height),
        b_(thr) { }
  virtual ~AstDetector() { }
  virtual void Detect(const unsigned char* im,
                      std::vector<agast::KeyPoint>& corners_all,
                      const unsigned char* thrmap = 0) = 0;
  virtual int GetBorderWidth() = 0;
  void Nms(const unsigned char* im,
           const std::vector<agast::KeyPoint>& corners_all,
           std::vector<agast::KeyPoint>& corners_nms);
  void ProcessImage(const unsigned char* im,
                    std::vector<agast::KeyPoint>& keypoints_nms) {
    std::vector<agast::KeyPoint> keypoints;
    Detect(im, keypoints, 0);
    Nms(im, keypoints, keypoints_nms);
  }
  void SetThreshold(int b_, int upperThreshold = 120,
                    int lowerThreshold = 50) {
    b_ = b_;
    upperThreshold_ = upperThreshold;
    lowerThreshold_ = lowerThreshold;
    cmpThreshold_ = (b_ * lowerThreshold_) / 100;
  }
  void SetImageSize(int xsize_, int ysize_) {
    xsize_ = xsize_;
    ysize_ = ysize_;
    InitPattern();
  }
  virtual int CornerScore(const unsigned char* p)=0;

 protected:
  virtual void InitPattern()=0;
  void Score(const unsigned char* i,
             const std::vector<agast::KeyPoint>& corners_all);
  void NonMaximumSuppression(const std::vector<agast::KeyPoint>& corners_all,
                             std::vector<agast::KeyPoint>& corners_nms);
  std::vector<int> scores_;
  std::vector<int> nmsFlags_;
  int xsize_, ysize_;
  int b_;
  int upperThreshold_;
  int lowerThreshold_;
  int cmpThreshold_;
};
}  // namespace agast
#endif  // CLOSED_FEATURES2D_AGAST_AGASTDETECTOR_H
