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

#ifndef CLOSED_FEATURES2D_AGAST_AGAST7_12S_H
#define CLOSED_FEATURES2D_AGAST_AGAST7_12S_H

#include <stdint.h>
#include <vector>
#include "./ast-detector.h"

namespace agast {

class AgastDetector7_12s : public AstDetector {
 public:
  AgastDetector7_12s(int width, int height)
      : AstDetector(width, height) { }
  AgastDetector7_12s(int width, int height, int thr)
      : AstDetector(width, height, thr) {
    InitPattern();
  }
  ~AgastDetector7_12s() { }
  void Detect(const unsigned char* im, std::vector<agast::KeyPoint>& keypoints,
              const unsigned char* thrmap);
  void Nms(const unsigned char* im,
           const std::vector<agast::KeyPoint>& keypoints,
           std::vector<agast::KeyPoint>& keypoints_nms);
  int GetBorderWidth() { return borderWidth; }
  int CornerScore(const unsigned char* p);

 private:
  static const int borderWidth = 2;
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

  void InitPattern() {
    s_offset0_ = (-2) + (0) * xsize_;
    s_offset1_ = (-2) + (-1) * xsize_;
    s_offset2_ = (-1) + (-2) * xsize_;
    s_offset3_ = (0) + (-2) * xsize_;
    s_offset4_ = (1) + (-2) * xsize_;
    s_offset5_ = (2) + (-1) * xsize_;
    s_offset6_ = (2) + (0) * xsize_;
    s_offset7_ = (2) + (1) * xsize_;
    s_offset8_ = (1) + (2) * xsize_;
    s_offset9_ = (0) + (2) * xsize_;
    s_offset10_ = (-1) + (2) * xsize_;
    s_offset11_ = (-2) + (1) * xsize_;
  }
};
}  // namespace agast
#endif  // CLOSED_FEATURES2D_AGAST_AGAST7_12S_H
