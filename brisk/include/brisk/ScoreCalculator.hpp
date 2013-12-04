/*
 * ScoreCalculator.hpp
 *
 *  Created on: Aug 3, 2012
 *      Author: lestefan
 */

#ifndef SCORECALCULATOR_HPP_
#define SCORECALCULATOR_HPP_

#include <vector>

#include <opencv2/features2d/features2d.hpp>
#include <opencv2/core/core.hpp>
#include <brisk/brisk.h>
#include <brisk/rdtsc_wrapper.h>

namespace brisk {

// abstract base class to provide an interface for score calculation of any sort.
template<typename SCORE_TYPE>
class ScoreCalculator {
 public:
  typedef SCORE_TYPE Score_t;

  // helper struct for point storage
#ifdef USE_SIMPLE_POINT_WITH_SCORE
  struct PointWithScore {
    PointWithScore()
        : score(0),
          x(0),
          y(0) {
    }
    PointWithScore(Score_t score_, uint16_t x_, uint16_t y_)
        : score(score_),
          x(x_),
          y(y_) {
    }
    Score_t score;
    uint16_t x, y;
    // this is so terrible. but so fast:
    bool operator<(const PointWithScore& other) const {
      return score > other.score;
    }
  };
#else
#error
  struct PointWithScore {
    PointWithScore():
    pt(cv::Point2i(0,0)),score(0) {}
    PointWithScore(cv::Point2i pt_, Score_t score_):
    pt(pt_),score(score_) {}
    cv::Point2i pt;
    Score_t score;
    inline bool operator<(const PointWithScore& other) const {return score>=other.score;}
  };
#endif

  // constructor
  ScoreCalculator() {
  }
  // destructor
  virtual ~ScoreCalculator() {
  }

  // set image
  void setImage(const cv::Mat& img, bool initScores = true) {
    _img = img;
    if (initScores)
      initializeScores();
  }

  // calculate/get score - implement floating point and integer access
  virtual inline double score(double u, double v)=0;
  virtual inline Score_t score(int u, int v)=0;

  // 2d maximum query
  virtual void get2dMaxima(std::vector<PointWithScore>& points,
                           Score_t absoluteThreshold = 0)=0;
 protected:
  cv::Mat _img;  // the image we operate on
  cv::Mat _scores;  // store calculated scores
  virtual void initializeScores()=0;
};

} // namespace brisk

#endif /* SCORECALCULATOR_HPP_ */
