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

#ifndef CLOSED_FEATURES2D_AGAST_OAST9_16_H
#define CLOSED_FEATURES2D_AGAST_OAST9_16_H

#include <stdint.h>
#include <vector>
#include "./ast-detector.h"

namespace agast {

class Oast9_16_PatternAccessor {
 public:
  Oast9_16_PatternAccessor(const unsigned char* img, int width, int height)
      :
        img_(img),
        width_(width),
        height_(height) { }

  void SetCenter(float x_c, float y_c, float scale = 1.0) {
    x_c_ = x_c;
    y_c_ = y_c;
    scale_ = scale;
  }
  unsigned char operator()(unsigned int index);
   private:
  // image
  const unsigned char* img_;
  int width_;
  int height_;
  // center & scale
  float x_c_;
  float y_c_;
  float scale_;
  // the (unscaled) pattern
  static const int pattern_x[17];
  static const int pattern_y[17];
};

class OastDetector9_16 : public AstDetector {
 public:
  OastDetector9_16(int width, int height)
      : AstDetector(width, height) { }
  OastDetector9_16(int width, int height, int thr)
      : AstDetector(width, height, thr) {
    InitPattern();
  }
  ~OastDetector9_16() { }
  void Detect(const unsigned char* im, std::vector<agast::KeyPoint>& keypoints,
              const unsigned char* thrmap = 0);
  void Nms(const unsigned char* im,
           const std::vector<agast::KeyPoint>& keypoints,
           std::vector<agast::KeyPoint>& keypoints_nms);
  int GetBorderWidth() { return borderWidth; }
  int CornerScore(const unsigned char* p);
  int CornerScore(const unsigned char* img, float x, float y, float scale);
 private:
  static const int borderWidth = 3;
  int_fast16_t s_offset0_;
  int_fast16_t s_offset1_;
  int_fast16_t s_offset2_;
  int_fast16_t s_offset3_;
  int_fast16_t s_offset4_;
  int_fast16_t s_offset5_;
  int_fast16_t s_offset6_;
  int_fast16_t s_offset7_;
  int_fast16_t s_offset8_;
  int_fast16_t s_offset9_;
  int_fast16_t s_offset10_;
  int_fast16_t s_offset11_;
  int_fast16_t s_offset12_;
  int_fast16_t s_offset13_;
  int_fast16_t s_offset14_;
  int_fast16_t s_offset15_;

  void InitPattern() {
    s_offset0_ = (-3) + (0) * xsize_;
    s_offset1_ = (-3) + (-1) * xsize_;
    s_offset2_ = (-2) + (-2) * xsize_;
    s_offset3_ = (-1) + (-3) * xsize_;
    s_offset4_ = (0) + (-3) * xsize_;
    s_offset5_ = (1) + (-3) * xsize_;
    s_offset6_ = (2) + (-2) * xsize_;
    s_offset7_ = (3) + (-1) * xsize_;
    s_offset8_ = (3) + (0) * xsize_;
    s_offset9_ = (3) + (1) * xsize_;
    s_offset10_ = (2) + (2) * xsize_;
    s_offset11_ = (1) + (3) * xsize_;
    s_offset12_ = (0) + (3) * xsize_;
    s_offset13_ = (-1) + (3) * xsize_;
    s_offset14_ = (-2) + (2) * xsize_;
    s_offset15_ = (-3) + (1) * xsize_;
  }
};
}  // namespace agast
#endif  // CLOSED_FEATURES2D_AGAST_OAST9_16_H
