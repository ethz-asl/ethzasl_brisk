/*
 * ScaleSpaceLayer.hpp
 *
 *  Created on: Aug 3, 2012
 *      Author: lestefan
 */

#ifndef SCALESPACELAYER_HPP_
#define SCALESPACELAYER_HPP_

#include <vector>
#include <opencv2/opencv.hpp>
#include <brisk/brisk.h>
#include <brisk/ScoreCalculator.hpp>

namespace brisk {

// A generic layer to be used within the ScaleSpace class.
template<class SCORE_CALCULTAOR_T>
class ScaleSpaceLayer {
 public:
  typedef SCORE_CALCULTAOR_T ScoreCalculator_t;
  ScaleSpaceLayer() {
  }
  ScaleSpaceLayer(const cv::Mat& img, bool initScores = true);  // Octave 0.
  ScaleSpaceLayer(ScaleSpaceLayer<ScoreCalculator_t>* layerBelow,
                  bool initScores = true);  // For successive construction.

  void create(const cv::Mat& img, bool initScores = true);  // Octave 0.
  void create(ScaleSpaceLayer<ScoreCalculator_t>* layerBelow, bool initScores =
                  true);  // For successive construction.

  void setUniformityRadius(double radius);
  void setMaxNumKpt(size_t maxNumKpt) {
    _maxNumKpt = maxNumKpt;
  }
  void setAbsoluteThreshold(double absoluteThreshold) {
    _absoluteThreshold = absoluteThreshold;
  }

  // Feature detection.
  void detectScaleSpaceMaxima(std::vector<cv::KeyPoint>& keypoints,
                              bool enforceUniformity = true, bool doRefinement =
                                  true,
                              bool usePassedKeypoints = false);

  // Subsampling.
  // Half sampling.
  static inline bool halfsample(const cv::Mat& srcimg, cv::Mat& dstimg);
  // 8 bit.
  static inline void halfsample8(const cv::Mat& srcimg, cv::Mat& dstimg);
  // for 16 bit input images.
  static inline void halfsample16(const cv::Mat& srcimg, cv::Mat& dstimg);
  // Two third sampling.
  static inline bool twothirdsample(const cv::Mat& srcimg, cv::Mat& dstimg);
  // 8 bit.
  static inline void twothirdsample8(const cv::Mat& srcimg, cv::Mat& dstimg);
  // for 16 bit input images.
  static inline void twothirdsample16(const cv::Mat& srcimg, cv::Mat& dstimg);
 protected:
  // Utilities.
  inline double scoreAbove(double u, double v);
  inline double scoreBelow(double u, double v);

  // 1d (scale) refinement.
  __inline__ float refine1D(const float s_05, const float s0, const float s05,
                            float& max);  // Around octave.
  __inline__ float refine1D_1(const float s_05, const float s0, const float s05,
                              float& max);  // Around intra.

  // 2D maximum refinement:
  __inline__ float subpixel2D(const double s_0_0, const double s_0_1,
                              const double s_0_2, const double s_1_0,
                              const double s_1_1, const double s_1_2,
                              const double s_2_0, const double s_2_1,
                              const double s_2_2, float& delta_x,
                              float& delta_y);

  // Layer properties.
  bool _isOctave;
  int _layerNumber;

  // Have a reference to the image for convenience:
  cv::Mat _img;

  // The score calculation.
  ScoreCalculator_t _scoreCalculator;

  // Remember next and previous layer.
  ScaleSpaceLayer* _aboveLayer_ptr;
  ScaleSpaceLayer* _belowLayer_ptr;

  // For coordinate transformations:
  double _offset_above, _offset_below;
  double _scale_above, _scale_below;
  double _scale;
  double _offset;

  // Uniformity enforcement related.
  double _radius;
  size_t _maxNumKpt;
  double _absoluteThreshold;
  cv::Mat _LUT;
};
#include "implementation/ScaleSpaceLayer.hpp"
}  // namespace brisk

#endif /* SCALESPACELAYER_HPP_ */
