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

namespace brisk{

// abstract base class to provide an interface for score calculation of any sort.
template<typename SCORE_TYPE>
class ScoreCalculator{
public:
	typedef SCORE_TYPE Score_t;

	// helper struct for point storage
	struct PointWithScore{
		PointWithScore(cv::Point2i pt_, Score_t score_):
			pt(pt_),score(score_){}
		cv::Point2i pt;
		Score_t score;
	};

	// constructor
	ScoreCalculator(){}
	// destructor
	virtual ~ScoreCalculator(){}

	// set image
	void setImage(const cv::Mat& img, bool initScores=true) {
		_img=img;
		if(initScores)
			initializeScores();
	}

	// calculate/get score - implement floating point and integer access
	virtual inline double score(double u, double v)=0;
	virtual inline Score_t score(int u, int v)=0;

	// 2d maximum query
	virtual void get2dMaxima(std::vector<PointWithScore>& points, Score_t absoluteThreshold=0)=0;
protected:
	cv::Mat _img; // the image we operate on
	cv::Mat _scores; // store calculated scores
	virtual void initializeScores()=0;
};

}


#endif /* SCORECALCULATOR_HPP_ */
