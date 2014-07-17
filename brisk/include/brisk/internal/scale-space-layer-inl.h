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

#ifndef INTERNAL_SCALE_SPACE_LAYER_INL_H_
#define INTERNAL_SCALE_SPACE_LAYER_INL_H_

#include <algorithm>
#include <vector>

#include <brisk/internal/uniformity-enforcement.h>
#include <brisk/internal/image-down-sampling.h>
#include <brisk/internal/timer.h>

namespace brisk {

template<class SCORE_CALCULATOR_T>
float ScaleSpaceLayer<SCORE_CALCULATOR_T>::max9(
    float i0, float i1, float i2, float i3, float i4, float i5, float i6, float i7,float i8){
  float max=i0;
  if(i1>max) max=i1;
  if(i2>max) max=i2;
  if(i3>max) max=i3;
  if(i4>max) max=i4;
  if(i5>max) max=i5;
  if(i6>max) max=i6;
  if(i7>max) max=i7;
  if(i8>max) max=i8;
  return max;
}

template<class SCORE_CALCULATOR_T>
ScaleSpaceLayer<SCORE_CALCULATOR_T>::ScaleSpaceLayer(const agast::Mat& img,
                                                     bool initScores) {
  Create(img);
}

template<class SCORE_CALCULATOR_T>
void ScaleSpaceLayer<SCORE_CALCULATOR_T>::Create(const agast::Mat& img,
                                                 bool initScores) {
  // Octave 0.
  _isOctave = true;
  _layerNumber = 0;

  // No adjacent layers (yet).
  _belowLayer_ptr = 0;
  _aboveLayer_ptr = 0;

  // Pass image and initialize score calculation.
  _scoreCalculator.SetImage(img, initScores);
  _img = img;

  // Scales and offsets.
  _offset_above = -0.25;
  _offset_below = 1.0 / 6.0;
  _scale_above = 2.0 / 3.0;
  _scale_below = 4.0 / 3.0;
  _scale = 1.0;
  _offset = 0.0;

  // By default no uniformity radius.
  _radius = 1;

  // Abs. threshold (for noise rejection).
  _absoluteThreshold = 0;

  // Generic mask.
  _LUT = agast::Mat::zeros(2 * 16 - 1, 2 * 16 - 1, CV_32F);
  for (int x = 0; x < 2 * 16 - 1; ++x) {
    for (int y = 0; y < 2 * 16 - 1; ++y) {
      _LUT.at<float>(y, x) = std::max(
          1 - static_cast<double>((15 - x) * (15 - x) + (15 - y) * (15 - y))
                  / static_cast<double>(15 * 15),
          0.0);
    }
  }
}

template<class SCORE_CALCULATOR_T>
ScaleSpaceLayer<SCORE_CALCULATOR_T>::ScaleSpaceLayer(
    ScaleSpaceLayer<ScoreCalculator_t>* layerBelow, bool initScores) {
  Create(layerBelow, initScores);
}

template<class SCORE_CALCULATOR_T>
void ScaleSpaceLayer<SCORE_CALCULATOR_T>::Create(
    ScaleSpaceLayer<ScoreCalculator_t>* layerBelow, bool initScores) {
  // For successive construction.
  //brisk::timing::Timer timerDownsample(
      //"0.0 BRISK Detection: Creation&Downsampling (per layer)");
  int type = layerBelow->_img.type();
  if (layerBelow->_isOctave) {
    if (layerBelow->_layerNumber >= 2) {
      // We can do the (cheaper) halfsampling.
      _img.create(layerBelow->_belowLayer_ptr->_img.rows / 2,
                  layerBelow->_belowLayer_ptr->_img.cols / 2, type);
      Halfsample(layerBelow->_belowLayer_ptr->_img, _img);

    } else {
      // We do the two-third sampling.
      _img.create((layerBelow->_img.rows / 3) * 2,
                  (layerBelow->_img.cols / 3) * 2, type);
      Twothirdsample(layerBelow->_img, _img);
    }
    // Keep track of where in the pyramid we are.
    _isOctave = false;
  } else {
    // We can do the (cheaper) halfsampling.
    _img.create(layerBelow->_belowLayer_ptr->_img.rows / 2,
                layerBelow->_belowLayer_ptr->_img.cols / 2, type);
    Halfsample(layerBelow->_belowLayer_ptr->_img, _img);

    // Keep track of where in the pyramid we are.
    _isOctave = true;
  }
  // Keep track.
  _layerNumber = layerBelow->_layerNumber + 1;
  _belowLayer_ptr = layerBelow;
  layerBelow->_aboveLayer_ptr = this;

  // Calculate coordinate transformation parameters.
  if (_isOctave) {
    _offset_above = -0.25;
    _offset_below = 1.0 / 6.0;
    _scale_above = 2.0 / 3.0;
    _scale_below = 4.0 / 3.0;
    _scale = pow(2.0, static_cast<double>(_layerNumber / 2));
    _offset = _scale * 0.5 - 0.5;
  } else {
    _offset_above = -1.0 / 6.0;
    _offset_below = 0.125;
    _scale_above = 0.75;
    _scale_below = 1.5;
    _scale = pow(2.0, static_cast<double>(_layerNumber / 2)) * 1.5;
    _offset = _scale * 0.5 - 0.5;
  }
  //timerDownsample.Stop();

  // By default no uniformity radius.
  _radius = 1;

  // Abs. threshold (for noise rejection).
  _absoluteThreshold = 0;

  // Initialize the score calculation.
  _scoreCalculator.SetImage(_img, initScores);

  // The above layer is undefined:
  _aboveLayer_ptr = 0;

  // Generic mask.
  _LUT = agast::Mat::zeros(2 * 16 - 1, 2 * 16 - 1, CV_32F);
  for (int x = 0; x < 2 * 16 - 1; ++x) {
    for (int y = 0; y < 2 * 16 - 1; ++y) {
      _LUT.at<float>(y, x) = std::max(
          1 - static_cast<double>((15 - x) * (15 - x) + (15 - y) * (15 - y))
                  / static_cast<double>(15 * 15),
          0.0);
    }
  }
}

template<class SCORE_CALCULATOR_T>
void ScaleSpaceLayer<SCORE_CALCULATOR_T>::SetUniformityRadius(double radius) {
  _radius = radius;
  if (radius == 0)
    _radius = 1;
}

// Feature detection.
template<class SCORE_CALCULATOR_T>
void ScaleSpaceLayer<SCORE_CALCULATOR_T>::DetectScaleSpaceMaxima(
    std::vector<agast::KeyPoint>& keypoints, bool enforceUniformity,
    bool doRefinement, bool usePassedKeypoints) {
  // First get the maxima points inside this layer.
  std::vector<typename ScoreCalculator_t::PointWithScore> points;
  if (usePassedKeypoints) {
    points.reserve(keypoints.size());
    for (size_t k = 0; k < keypoints.size(); ++k) {
      if (agast::KeyPointResponse(keypoints[k]) > 1e6) {
        points.push_back(
            typename ScoreCalculator_t::PointWithScore(
                agast::KeyPointResponse(keypoints[k]),
                agast::KeyPointX(keypoints[k]),
                agast::KeyPointY(keypoints[k])));
      }
    }
  } else {
    //brisk::timing::DebugTimer timerNonMaxSuppression2d(
        //"0.2 BRISK Detection: 2d nonmax suppression (per layer)");
    _scoreCalculator.Get2dMaxima(points, _absoluteThreshold);
    //timerNonMaxSuppression2d.Stop();
  }
  // Next check above and below. The code looks a bit stupid, but that's
  // for speed. We don't want to make the distinction analyzing whether or
  // not there is a layer above and below inside the loop.
  if (!usePassedKeypoints) {
    if (_aboveLayer_ptr != 0 && _belowLayer_ptr != 0) {
      // Check above and below
      //brisk::timing::DebugTimer timerNonMaxSuppression3d(
          //"0.3 BRISK Detection: 3d nonmax suppression (per layer)");
      std::vector<typename ScoreCalculator_t::PointWithScore> pt_tmp;
      pt_tmp.reserve(points.size());
      const float one_over_scale_above = 1.0 / _scale_above;
      const float one_over_scale_below = 1.0 / _scale_below;
      //std::cout<<"_scale_below:"<<_scale_below<<", _scale:"<<_scale<<", _scale_above:"<<_scale_above<<std::endl;
      for (typename std::vector<
          typename ScoreCalculator_t::PointWithScore>::const_iterator it =
          points.begin(); it != points.end(); ++it) {
        const typename ScoreCalculator_t::Score_t center = 1.5*float(it->score); // give 30% margin
        const double x = it->x;
        const double y = it->y;
        if (center < (typename ScoreCalculator_t::Score_t) (_absoluteThreshold))
          continue;

        bool ismax=true;
        for(float a = -1; a<=1; ++a){
          for(float b = -1; b<=1; ++b){
            if (center < ScoreBelow(x + a*one_over_scale_below, y + b*one_over_scale_below)){
              ismax=false;
              break;
            }
            if (center < ScoreAbove(x + a*one_over_scale_above, y + b*one_over_scale_above)){
              ismax=false;
              break;
            }
            //std::cout << ScoreBelow(x + a*one_over_scale_below, y + b*one_over_scale_below) << " ";
            //std::cout<<ScoreAbove(x + a*one_over_scale_above, y + b*one_over_scale_above)<<std::endl;
            //std::cout<<ScoreBelow(x + a*one_over_scale_below, y + b*one_over_scale_below)<<std::endl;
          }
          if(!ismax)
            break;
          //std::cout<<std::endl;
        }
        //std::cout<<"=============================="<<std::endl;
        if(!ismax)
          continue;

        pt_tmp.push_back(*it);
      }
      points.assign(pt_tmp.begin(), pt_tmp.end());
      //timerNonMaxSuppression3d.Stop();
    } else if (_aboveLayer_ptr != 0) {
      // Check above.
      //brisk::timing::DebugTimer timerNonMaxSuppression3d(
          //"0.3 BRISK Detection: 3d nonmax suppression (per layer)");
      std::vector<typename ScoreCalculator_t::PointWithScore> pt_tmp;
      pt_tmp.reserve(points.size());
      const float one_over_scale_above = 1.0 / _scale_above;
      //std::cout<<"one_over_scale_above:"<<one_over_scale_above<<std::endl;
      for (typename std::vector<
          typename ScoreCalculator_t::PointWithScore>::const_iterator it =
          points.begin(); it != points.end(); ++it) {
        const typename ScoreCalculator_t::Score_t center =  1.5*float(it->score); // give 30% margin
        if (center < (typename ScoreCalculator_t::Score_t) (_absoluteThreshold))
          continue;
        const double x = it->x;
        const double y = it->y;

        bool ismax=true;
        for(float a = -1; a<=1; ++a){
          for(float b = -1; b<=1; ++b){
            if (center < ScoreAbove(x + a*one_over_scale_above, y + b*one_over_scale_above)){
              ismax=false;
              break;
            }
            if(!ismax)
              break;
          }
        }
        if(!ismax)
          continue;

        pt_tmp.push_back(*it);
      }
      points.assign(pt_tmp.begin(), pt_tmp.end());
      //timerNonMaxSuppression3d.Stop();
    } else if (_belowLayer_ptr != 0) {
      // Check below.
      //brisk::timing::DebugTimer timerNonMaxSuppression3d(
          //"0.3 BRISK Detection: 3d nonmax suppression (per layer)");
      std::vector<typename ScoreCalculator_t::PointWithScore> pt_tmp;
      pt_tmp.reserve(points.size());
      const float one_over_scale_below = 1.0 / _scale_below;
      //std::cout<<"_scale_below:"<<_scale_below<<std::endl;
      for (typename std::vector<
          typename ScoreCalculator_t::PointWithScore>::const_iterator it =
          points.begin(); it != points.end(); ++it) {
        const typename ScoreCalculator_t::Score_t center =  1.5*float(it->score); // give 30% margin
        if (center < (typename ScoreCalculator_t::Score_t) (_absoluteThreshold))
          continue;
        const double x = it->x;
        const double y = it->y;

        bool ismax=true;
        for(float a = -1; a<=1; ++a){
          for(float b = -1; b<=1; ++b){
            if (center < ScoreBelow(x + a*one_over_scale_below, y + b*one_over_scale_below)){
              ismax=false;
              break;
            }
            if(!ismax)
              break;
          }
        }
        if(!ismax)
          continue;

        pt_tmp.push_back(*it);
      }
      points.assign(pt_tmp.begin(), pt_tmp.end());
      //timerNonMaxSuppression3d.Stop();
    }
  }

  // Uniformity enforcement.
  if (points.size() == 0)
    return;
  if (enforceUniformity && _radius > 0.0) {
    EnforceKeyPointUniformity(_LUT, _radius, _img.rows, _img.cols, _maxNumKpt,
                              points);
  }

  // 3d(/2d) subpixel refinement.
  //brisk::timing::DebugTimer timer_subpixel_refinement(
      //"0.4 BRISK Detection: "
      //"subpixel(&scale) refinement (per layer)");
  if (usePassedKeypoints)
    keypoints.clear();
  if (doRefinement) {
    for (typename std::vector<
        typename ScoreCalculator_t::PointWithScore>::const_iterator it =
        points.begin(); it != points.end(); ++it) {
      const int u = it->x;
      const int v = it->y;
      float delta_x=0;
      float delta_y=0;

      const double s_1_1 = _scoreCalculator.Score(u - 1, v - 1);
      const double s0_1 = _scoreCalculator.Score(u , v - 1);
      const double s1_1 = _scoreCalculator.Score(u + 1, v - 1);
      const double s_10 = _scoreCalculator.Score(u - 1, v);
      const double s00 = _scoreCalculator.Score(u , v );
      const double s10 = _scoreCalculator.Score(u + 1, v );
      const double s_11 = _scoreCalculator.Score(u - 1, v + 1);
      const double s01 = _scoreCalculator.Score(u , v + 1);
      const double s11 = _scoreCalculator.Score(u + 1, v + 1);

      //std::cout<<s_1_1<<","<<s0_1<<","<<s1_1<<std::endl;
      //std::cout<<s_10<<","<<s00<<","<<s10<<std::endl;
      //std::cout<<s_11<<","<<s01<<","<<s11<<std::endl;

      Subpixel2D(s_1_1, s0_1, s1_1,
                 s_10, s00, s10,
                 s_11, s01, s11,
                 delta_x,
                 delta_y);

      if(delta_x>0.99 || delta_x<-0.99 || delta_y>0.99 || delta_y<-0.99){
        delta_x=0.0;
        delta_y=0.0;
      }

      // scale refinement, if possible:
      float scale_refinement = 1.0f;
      // this does not seem to have a positive influence:
      /*if (_aboveLayer_ptr != 0 && _belowLayer_ptr != 0) {
        const float one_over_scale_above = 1.0 / _scale_above;
        const float one_over_scale_below = 1.0 / _scale_below;

        // get patch above
        const double a_1_1 = ScoreAbove(u - one_over_scale_above, v - one_over_scale_above);
        const double a0_1 = ScoreAbove(u, v - one_over_scale_above);
        const double a1_1 = ScoreAbove(u + one_over_scale_above, v - one_over_scale_above);
        const double a_10 = ScoreAbove(u - one_over_scale_above, v);
        const double a00 = ScoreAbove(u , v);
        const double a10 = ScoreAbove(u + one_over_scale_above, v);
        const double a_11 = ScoreAbove(u - one_over_scale_above, v + one_over_scale_above);
        const double a01 = ScoreAbove(u, v + one_over_scale_above);
        const double a11 = ScoreAbove(u + one_over_scale_above, v + one_over_scale_above);
        // get patch below
        const double b_1_1 = ScoreBelow(u - one_over_scale_below, v - one_over_scale_below);
        const double b0_1 = ScoreBelow(u, v - one_over_scale_below);
        const double b1_1 = ScoreBelow(u + one_over_scale_below, v - one_over_scale_below);
        const double b_10 = ScoreBelow(u - one_over_scale_below, v);
        const double b00 = ScoreBelow(u , v);
        const double b10 = ScoreBelow(u + one_over_scale_below, v);
        const double b_11 = ScoreBelow(u - one_over_scale_below, v + one_over_scale_below);
        const double b01 = ScoreBelow(u, v + one_over_scale_below);
        const double b11 = ScoreBelow(u + one_over_scale_below, v + one_over_scale_below);

        // refine above:
        float a_delta_x, a_delta_y;
        float a =  max9(a_1_1, a0_1, a1_1, a_10, a00, a10, a_11, a01, a11);
        Subpixel2D(a_1_1, a0_1, a1_1, a_10, a00, a10, a_11, a01, a11, a_delta_x, a_delta_y);

        // refine below:
        float b_delta_x, b_delta_y;
        float b = max9(b_1_1, b0_1, b1_1, b_10, b00, b10, b_11, b01, b11);
        Subpixel2D(b_1_1, b0_1, b1_1,b_10, b00, b10,b_11, b01, b11,b_delta_x,b_delta_y);

        // refine scale
        float s_max;
        if(_scale_above>0.7) // 1.333
          scale_refinement=Refine1D_1(b,s00,a,s_max);
        else
          scale_refinement=Refine1D(b,s00,a,s_max); // 1.5

        // interpolate coordinates
        if(scale_refinement>1.0){
          delta_x=(scale_refinement-1.0f)/(one_over_scale_above-1.0f)*a_delta_x*one_over_scale_above+
              (2.0f-scale_refinement)/(one_over_scale_above-1.0f)*delta_x;
          delta_y=(scale_refinement-1.0f)/(one_over_scale_above-1.0f)*a_delta_y*one_over_scale_above+
              (2.0f-scale_refinement)/(one_over_scale_above-1.0f)*delta_y;
        } else {
          delta_x=(scale_refinement)/(1.0f-one_over_scale_below)*b_delta_x*one_over_scale_below+
              (1.0f-scale_refinement)/(1.0f-one_over_scale_below)*delta_x;
          delta_y=(scale_refinement)/(1.0f-one_over_scale_below)*b_delta_y*one_over_scale_below+
              (1.0f-scale_refinement)/(1.0f-one_over_scale_below)*delta_y;
        }
        //std::cout<<b<<", "<<s00<<", "<<a<<", scale_refinement"<<scale_refinement<<std::endl;
        //scale_refinement=1.0;
      }*/

      //std::cout<<delta_x<<" , "<<delta_y<<std::endl;
      agast::KeyPoint keypoint;
      agast::KeyPointX(keypoint) = _scale * (it->x + delta_x + 0.5) - 0.5;
      agast::KeyPointY(keypoint) = _scale * (it->y + delta_y + 0.5) - 0.5;
      agast::KeyPointSize(keypoint) = _scale * scale_refinement * 12.0;
      agast::KeyPointAngle(keypoint) = -1;
      agast::KeyPointResponse(keypoint) = it->score;
      agast::KeyPointOctave(keypoint) = _layerNumber / 2;
      keypoints.push_back(keypoint);
    }
  } else {
    for (typename std::vector<
        typename ScoreCalculator_t::PointWithScore>::const_iterator it =
        points.begin(); it != points.end(); ++it) {
      agast::KeyPoint keypoint;
      agast::KeyPointX(keypoint) = _scale * (it->x + 0.5) - 0.5;
      agast::KeyPointY(keypoint) = _scale * (it->y + 0.5) - 0.5;
      agast::KeyPointSize(keypoint) = _scale * 12.0;
      agast::KeyPointAngle(keypoint) = -1;
      agast::KeyPointResponse(keypoint) = it->score;
      agast::KeyPointOctave(keypoint) = _layerNumber / 2;
      keypoints.push_back(keypoint);
    }
  }
  //timer_subpixel_refinement.Stop();
}

// Utilities.
template<class SCORE_CALCULATOR_T>
inline double ScaleSpaceLayer<SCORE_CALCULATOR_T>::ScoreAbove(double u,
                                                              double v) {
  //return 0;
  return _aboveLayer_ptr->_scoreCalculator.Score(
      int(_scale_above * (u + 0.5) - 0.5), int(_scale_above * (v + 0.5) - 0.5));
}
template<class SCORE_CALCULATOR_T>
inline double ScaleSpaceLayer<SCORE_CALCULATOR_T>::ScoreBelow(double u,
                                                              double v) {
  //return 0;
  //std::cout<<"_offset_below="<<_offset_below<<", _scale_below="<<_scale_below<<std::endl;
  return _belowLayer_ptr->_scoreCalculator.Score(
      int(_scale_below * (u + 0.5) - 0.5), int(_scale_below * (v + 0.5) - 0.5));
}

template<class SCORE_CALCULATOR_T>
inline bool ScaleSpaceLayer<SCORE_CALCULATOR_T>::Halfsample(
    const agast::Mat& srcimg, agast::Mat& dstimg) {
  if (srcimg.type() == CV_8UC1) {
    Halfsample8(srcimg, dstimg);
  } else if (srcimg.type() == CV_16UC1) {
    Halfsample16(srcimg, dstimg);
  } else {
    return false;
  }
  return true;
}

template<class SCORE_CALCULATOR_T>
inline bool ScaleSpaceLayer<SCORE_CALCULATOR_T>::Twothirdsample(
    const agast::Mat& srcimg, agast::Mat& dstimg) {
  if (srcimg.type() == CV_8UC1) {
    Twothirdsample8(srcimg, dstimg);
  } else if (srcimg.type() == CV_16UC1) {
    Twothirdsample16(srcimg, dstimg);
  } else {
    return false;
  }
  return true;
}

template<class SCORE_CALCULATOR_T>
__inline__ float ScaleSpaceLayer<SCORE_CALCULATOR_T>::Refine1D(const float s_05,
                                                               const float s0,
                                                               const float s05,
                                                               float& max) {
  int i_05 = static_cast<int>(1024.0 * s_05 + 0.5);
  int i0 = static_cast<int>(1024.0 * s0 + 0.5);
  int i05 = static_cast<int>(1024.0 * s05 + 0.5);

  //   16.0000  -24.0000    8.0000
  //  -40.0000   54.0000  -14.0000
  //   24.0000  -27.0000    6.0000

  int three_a = 16 * i_05 - 24 * i0 + 8 * i05;
  // Second derivative must be negative:
  if (three_a >= 0) {
    if (s0 >= s_05 && s0 >= s05) {
      max = s0;
      return 1.0;
    }
    if (s_05 >= s0 && s_05 >= s05) {
      max = s_05;
      return 0.75;
    }
    if (s05 >= s0 && s05 >= s_05) {
      max = s05;
      return 1.5;
    }
  }

  int three_b = -40 * i_05 + 54 * i0 - 14 * i05;
  // Calculate max location:
  float ret_val = -static_cast<float>(three_b) /
      static_cast<float>(2 * three_a);
  // Saturate and return
  if (ret_val < 0.75)
    ret_val = 0.75;
  else if (ret_val > 1.5)
    ret_val = 1.5;  // Allow to be slightly off bounds ...?
  int three_c = +24 * i_05 - 27 * i0 + 6 * i05;
  max = static_cast<float>(three_c) + static_cast<float>(three_a) *
      ret_val * ret_val + static_cast<float>(three_b) * ret_val;
  max /= 3072.0;
  return ret_val;
}

template<class SCORE_CALCULATOR_T>
__inline__ float ScaleSpaceLayer<SCORE_CALCULATOR_T>::Refine1D_1(
    const float s_05, const float s0, const float s05, float& max) {
  int i_05 = static_cast<int>(1024.0 * s_05 + 0.5);
  int i0 = static_cast<int>(1024.0 * s0 + 0.5);
  int i05 = static_cast<int>(1024.0 * s05 + 0.5);

  //  4.5000   -9.0000    4.5000
  // -10.5000   18.0000   -7.5000
  //  6.0000   -8.0000    3.0000

  int two_a = 9 * i_05 - 18 * i0 + 9 * i05;
  // Second derivative must be negative:
  if (two_a >= 0) {
    if (s0 >= s_05 && s0 >= s05) {
      max = s0;
      return 1.0;
    }
    if (s_05 >= s0 && s_05 >= s05) {
      max = s_05;
      return 0.6666666666666666666666666667;
    }
    if (s05 >= s0 && s05 >= s_05) {
      max = s05;
      return 1.3333333333333333333333333333;
    }
  }

  int two_b = -21 * i_05 + 36 * i0 - 15 * i05;
  // calculate max location:
  float ret_val = -static_cast<float>(two_b) / static_cast<float>(2 * two_a);
  // saturate and return
  if (ret_val < 0.6666666666666666666666666667)
    ret_val = 0.666666666666666666666666667;
  else if (ret_val > 1.33333333333333333333333333)
    ret_val = 1.333333333333333333333333333;
  int two_c = +12 * i_05 - 16 * i0 + 6 * i05;
  max = static_cast<float>(two_c) + static_cast<float>(two_a) * ret_val *
      ret_val + static_cast<float>(two_b) * ret_val;
  max /= 2048.0;
  return ret_val;
}

template<class SCORE_CALCULATOR_T>
__inline__ float ScaleSpaceLayer<SCORE_CALCULATOR_T>::Subpixel2D(
    const double s_0_0, const double s_0_1, const double s_0_2,
    const double s_1_0, const double s_1_1, const double s_1_2,
    const double s_2_0, const double s_2_1, const double s_2_2, float& delta_x,
    float& delta_y) {
  // The coefficients of the 2d quadratic function least-squares fit:
  double tmp1 = s_0_0 + s_0_2 - 2 * s_1_1 + s_2_0 + s_2_2;
  double coeff1 = 3 * (tmp1 + s_0_1 - ((s_1_0 + s_1_2) / 2.0) + s_2_1);
  double coeff2 = 3 * (tmp1 - ((s_0_1 + s_2_1) / 2.0) + s_1_0 + s_1_2);
  double tmp2 = s_0_2 - s_2_0;
  double tmp3 = (s_0_0 + tmp2 - s_2_2);
  double tmp4 = tmp3 - 2 * tmp2;
  double coeff3 = -3 * (tmp3 + s_0_1 - s_2_1);
  double coeff4 = -3 * (tmp4 + s_1_0 - s_1_2);
  double coeff5 = (s_0_0 - s_0_2 - s_2_0 + s_2_2) / 4.0;
  double coeff6 = -(s_0_0 + s_0_2
      - ((s_1_0 + s_0_1 + s_1_2 + s_2_1) / 2.0) - 5 * s_1_1 + s_2_0 + s_2_2)
      / 2.01;

  // 2nd derivative test:
  double H_det = 4 * coeff1 * coeff2 - coeff5 * coeff5;

  if (H_det == 0) {
    delta_x = 0.0;
    delta_y = 0.0;
    return static_cast<float>(coeff6) / 18.0;
  }

  if (!(H_det > 0 && coeff1 < 0)) {
    // The maximum must be at the one of the 4 patch corners.
    int tmp_max = coeff3 + coeff4 + coeff5;
    delta_x = 1.0;
    delta_y = 1.0;

    int tmp = -coeff3 + coeff4 - coeff5;
    if (tmp > tmp_max) {
      tmp_max = tmp;
      delta_x = -1.0;
      delta_y = 1.0;
    }
    tmp = coeff3 - coeff4 - coeff5;
    if (tmp > tmp_max) {
      tmp_max = tmp;
      delta_x = 1.0;
      delta_y = -1.0;
    }
    tmp = -coeff3 - coeff4 + coeff5;
    if (tmp > tmp_max) {
      tmp_max = tmp;
      delta_x = -1.0;
      delta_y = -1.0;
    }
    return static_cast<float>(tmp_max + coeff1 + coeff2 + coeff6) / 18.0;
  }

  // This is hopefully the normal outcome of the Hessian test.
  delta_x = static_cast<float>(2 * coeff2 * coeff3 - coeff4 * coeff5) /
      static_cast<float>(-H_det);
  delta_y = static_cast<float>(2 * coeff1 * coeff4 - coeff3 * coeff5) /
      static_cast<float>(-H_det);
  // TODO(lestefan): this is not correct, but easy, so perform a real boundary
  // maximum search:
  bool tx = false;
  bool tx_ = false;
  bool ty = false;
  bool ty_ = false;
  if (delta_x > 1.0)
    tx = true;
  else if (delta_x < -1.0)
    tx_ = true;
  if (delta_y > 1.0)
    ty = true;
  if (delta_y < -1.0)
    ty_ = true;

  if (tx || tx_ || ty || ty_) {
    // Get two candidates:
    float delta_x1 = 0.0, delta_x2 = 0.0, delta_y1 = 0.0, delta_y2 = 0.0;
    if (tx) {
      delta_x1 = 1.0;
      delta_y1 = -static_cast<float>(coeff4 + coeff5) /
          static_cast<float>(2 * coeff2);
      if (delta_y1 > 1.0)
        delta_y1 = 1.0;
      else if (delta_y1 < -1.0)
        delta_y1 = -1.0;
    } else if (tx_) {
      delta_x1 = -1.0;
      delta_y1 = -static_cast<float>(coeff4 - coeff5) /
          static_cast<float>(2 * coeff2);
      if (delta_y1 > 1.0)
        delta_y1 = 1.0;
      else if (delta_y1 < -1.0)
        delta_y1 = -1.0;
    }
    if (ty) {
      delta_y2 = 1.0;
      delta_x2 = -static_cast<float>(coeff3 + coeff5) /
          static_cast<float>(2 * coeff1);
      if (delta_x2 > 1.0)
        delta_x2 = 1.0;
      else if (delta_x2 < -1.0)
        delta_x2 = -1.0;
    } else if (ty_) {
      delta_y2 = -1.0;
      delta_x2 = -static_cast<float>(coeff3 - coeff5) /
          static_cast<float>(2 * coeff1);
      if (delta_x2 > 1.0)
        delta_x2 = 1.0;
      else if (delta_x2 < -1.0)
        delta_x2 = -1.0;
    }
    // Insert both options for evaluation which to pick.
    float max1 = (coeff1 * delta_x1 * delta_x1 + coeff2 * delta_y1 * delta_y1
        + coeff3 * delta_x1 + coeff4 * delta_y1 + coeff5 * delta_x1 * delta_y1
        + coeff6) / 18.0;
    float max2 = (coeff1 * delta_x2 * delta_x2 + coeff2 * delta_y2 * delta_y2
        + coeff3 * delta_x2 + coeff4 * delta_y2 + coeff5 * delta_x2 * delta_y2
        + coeff6) / 18.0;
    if (max1 > max2) {
      delta_x = delta_x1;
      delta_y = delta_x1;
      return max1;
    } else {
      delta_x = delta_x2;
      delta_y = delta_x2;
      return max2;
    }
  }
  // This is the case of the maximum inside the boundaries:
  return (coeff1 * delta_x * delta_x + coeff2 * delta_y * delta_y
      + coeff3 * delta_x + coeff4 * delta_y + coeff5 * delta_x * delta_y
      + coeff6) / 18.0;
}
}  // namespace brisk
#endif  // INTERNAL_SCALE_SPACE_LAYER_INL_H_
