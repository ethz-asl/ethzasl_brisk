/*
 * harris.h
 *
 *  Created on: Jul 28, 2012
 *      Author: lestefan
 */

#ifndef HARRIS_H_
#define HARRIS_H_

#include <opencv2/features2d/features2d.hpp>
#include <opencv2/core/core.hpp>

#include <brisk/sseFilters.hpp>

namespace brisk{

class HarrisFeatureDetector : public cv::FeatureDetector{
public:
	HarrisFeatureDetector(double radius);
	void setRadius(double radius);

protected:
	static __inline__ void getCovarEntries(
			const cv::Mat& src, cv::Mat& dxdx, cv::Mat& dydy, cv::Mat& dxdy);
	static __inline__ void cornerHarris(
			const cv::Mat& dxdxSmooth, const cv::Mat& dydySmooth, const cv::Mat& dxdySmooth,
			cv::Mat& score);
	static __inline__ void nonmaxSuppress(
			const cv::Mat& scores, std::vector<cv::KeyPoint>& keypoints);
	__inline__ void enforceUniformity(
			const cv::Mat& scores, std::vector<cv::KeyPoint>& keypoints) const;


	virtual void detectImpl( const cv::Mat& image,
			std::vector<cv::KeyPoint>& keypoints,
			const cv::Mat& mask=cv::Mat() ) const;

	double _radius;
	cv::Mat _LUT;
};


} // namespace brisk


#endif /* HARRIS_H_ */
