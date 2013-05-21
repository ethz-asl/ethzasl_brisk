/*
 * HarrisScoreCalculator.hpp
 *
 *  Created on: Aug 3, 2012
 *      Author: lestefan
 */

#ifndef HARRISSCORECALCULATOR_HPP_
#define HARRISSCORECALCULATOR_HPP_

#include <opencv2/features2d/features2d.hpp>
#include <opencv2/core/core.hpp>
#include <brisk/brisk.h>
#include <brisk/ScoreCalculator.hpp>


namespace brisk{

// Harris score manager
class HarrisScoreCalculator : public ScoreCalculator<int>{
public:
	typedef ScoreCalculator<int> Base_t;

	// provide accessor implementations here in order to enable inlining
	inline double score(double u, double v){
		// simple bilinear interpolation - no checking (for speed)
		const int u_int=int(u);
		const int v_int=int(v);
		if(u_int+1>=_scores.cols||v_int+1>=_scores.rows||
			u_int<0||v_int<0)
			return 0.0;
		const double ru=u-double(u_int);
		const double rv=v-double(v_int);
		const double oneMinus_ru=1.0-ru;
		const double oneMinus_rv=1.0-rv;
		return oneMinus_rv*(oneMinus_ru*_scores.at<int>(v_int,u_int)
					+ ru*_scores.at<int>(v_int,u_int+1))
				+ rv*(oneMinus_ru*_scores.at<int>(v_int+1,u_int)
					+ ru*_scores.at<int>(v_int+1,u_int+1));
	}
	inline Base_t::Score_t score(int u, int v){
		return _scores.at<int>(v,u);
	}
	virtual void get2dMaxima(std::vector<PointWithScore>& points, int absoluteThreshold=0);
protected:
	// calculates the Harris scores
	virtual void initializeScores();

	// Harris specific
	static void getCovarEntries(
			const cv::Mat& src, cv::Mat& dxdx, cv::Mat& dydy, cv::Mat& dxdy);
	static void cornerHarris(
			const cv::Mat& dxdxSmooth, const cv::Mat& dydySmooth, const cv::Mat& dxdySmooth,
			cv::Mat& score);
};

}

#endif /* HARRISSCORECALCULATOR_HPP_ */
