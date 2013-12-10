/*
 * ScaleSpaceFeatureDetector.hpp
 *
 *  Created on: Aug 5, 2012
 *      Author: lestefan
 */

#ifndef SCALESPACEFEATUREDETECTOR_HPP_
#define SCALESPACEFEATUREDETECTOR_HPP_

#include <sys/time.h>

#include <opencv2/features2d/features2d.hpp>
#include <opencv2/core/core.hpp>
#include <brisk/brisk.h>
#include <brisk/ScoreCalculator.hpp>
#include <brisk/ScaleSpaceLayer.hpp>
#include <brisk/rdtsc_wrapper.h>

namespace brisk{

// uses the common feature interface to construct a generic
// scale space detector from a given ScoreCalculator
template<class SCORE_CALCULTAOR_T>
class ScaleSpaceFeatureDetector : public cv::FeatureDetector{
public:
	ScaleSpaceFeatureDetector(size_t octaves, double uniformityRadius,
			double absoluteThreshold=0,size_t maxNumKpt=std::numeric_limits<size_t>::max()):
		_octaves(octaves),_uniformityRadius(uniformityRadius),
		_absoluteThreshold(absoluteThreshold), _maxNumKpt(maxNumKpt){
		scaleSpaceLayers.resize(
				std::max(_octaves*2,size_t(1)));
	}

	typedef SCORE_CALCULTAOR_T ScoreCalculator_t;
	void detect(const cv::Mat& image, std::vector<cv::KeyPoint>& keypoints,
			const cv::Mat& mask=cv::Mat() ) const {
	    if( image.empty() )
	        return;
	    CV_Assert( mask.empty() || (mask.type() == CV_8UC1 && mask.size() == image.size()) );
	    detectImpl( image, keypoints, mask );
	}

protected:
	virtual void detectImpl( const cv::Mat& image,
			std::vector<cv::KeyPoint>& keypoints,
			const cv::Mat& mask=cv::Mat() ) const{

		// find out, if we should use the provided keypoints
		bool usePassedKeypoints = false;
		//std::cout<<keypoints.size()<<std::endl;
		if(keypoints.size()>0)
			usePassedKeypoints = true;
		else
			keypoints.reserve(4000); // possibly speeds up things

		// construct scale space layers
		//gettimeofday(&start, NULL);
		scaleSpaceLayers[0].create(image,!usePassedKeypoints);
		scaleSpaceLayers[0].setUniformityRadius(_uniformityRadius);
		scaleSpaceLayers[0].setMaxNumKpt(_maxNumKpt);
		scaleSpaceLayers[0].setAbsoluteThreshold(_absoluteThreshold);
		for(size_t i=1; i<_octaves*2; ++i){
			//struct timeval start, end;
			//gettimeofday(&start, NULL);
			scaleSpaceLayers[i].create(&scaleSpaceLayers[i-1],!usePassedKeypoints);
			scaleSpaceLayers[i].setUniformityRadius(_uniformityRadius);
			scaleSpaceLayers[i].setMaxNumKpt(_maxNumKpt);
			scaleSpaceLayers[i].setAbsoluteThreshold(_absoluteThreshold);
			//gettimeofday(&end, NULL);
			//std::cout<<double(end.tv_sec-start.tv_sec)*1000.0+double(end.tv_usec-start.tv_usec)/1000.0<<std::endl;
		}
		//gettimeofday(&end, NULL);
		//std::cout<<double(end.tv_sec-start.tv_sec)*1000.0+double(end.tv_usec-start.tv_usec)/1000.0<<std::endl;
		// detect
		//gettimeofday(&start, NULL);
		for(size_t i=0; i<scaleSpaceLayers.size(); ++i){
			scaleSpaceLayers[i].detectScaleSpaceMaxima(keypoints,
					true,!usePassedKeypoints,usePassedKeypoints); // only do refinement, if no keypoints are passed
		}
		//gettimeofday(&end, NULL);
		//std::cout<<double(end.tv_sec-start.tv_sec)*1000.0+double(end.tv_usec-start.tv_usec)/1000.0<<std::endl;
	}

	size_t _octaves;
	double _uniformityRadius;
	double _absoluteThreshold;
	size_t _maxNumKpt;

	mutable std::vector<brisk::ScaleSpaceLayer<ScoreCalculator_t> > scaleSpaceLayers;

};



} // namespace brisk

#endif /* SCALESPACEFEATUREDETECTOR_HPP_ */
