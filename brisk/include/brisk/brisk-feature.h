/*
 * BriskFeature.hpp
 *
 *  Created on: Jan 1, 2014
 *      Author: lestefan
 */

#ifndef BRISKFEATURE_HPP_
#define BRISKFEATURE_HPP_

#include <brisk/brisk-opencv.h>
#include <brisk/brisk-descriptor-extractor.h>
#include <brisk/brisk-feature-detector.h>
#include <brisk/harris-feature-detector.h>
#include <brisk/harris-score-calculator.h>
#include <brisk/scale-space-feature-detector.h>

namespace brisk{

class BriskFeature : public cv::Feature2D{
public:
	BriskFeature( size_t octaves, double uniformityRadius, double absoluteThreshold = 0,
			size_t maxNumKpt = std::numeric_limits < size_t > ::max(),
			bool rotationInvariant=true,
            bool scaleInvariant=true) :
            	_briskDetector(octaves, uniformityRadius, absoluteThreshold, maxNumKpt),
            	_briskExtractor(rotationInvariant, scaleInvariant)
            {}

	virtual ~BriskFeature(){}

	/* cv::DescriptorExtractor interface */
	virtual int descriptorSize() const {return _briskExtractor.descriptorSize();}
	virtual int descriptorType() const {return _briskExtractor.descriptorType();}

	/* cv::Feature2D interface */
	virtual void operator()( cv::InputArray image, cv::InputArray mask,
	                                     std::vector<cv::KeyPoint>& keypoints,
	                                     cv::OutputArray descriptors,
	                                     bool useProvidedKeypoints=false ) const{
		// handle provided keypoints correctly
		if(!useProvidedKeypoints){
			keypoints.clear();
		}

		// convert input output arrays:
		cv::Mat descriptors_;
		cv::Mat image_ = image.getMat();
		cv::Mat mask_ = mask.getMat();

		// detection
		_briskDetector.detect(image_, keypoints, mask_); // this is already taking keypoints, if provided

		// extraction
		_briskExtractor.compute(image_, keypoints, descriptors_);
		descriptors.getMatRef() = descriptors_;
	}

protected:

	/* cv::FeatureDetector interface */
	virtual void detectImpl( const cv::Mat& image, std::vector<cv::KeyPoint>& keypoints,
			const cv::Mat& mask=cv::Mat() ) const{
		_briskDetector.detect(image, keypoints, mask);
	}

	/* cv::DescriporExtractor interface */
	virtual void computeImpl( const cv::Mat& image, std::vector<cv::KeyPoint>& keypoints,
			cv::Mat& descriptors ) const{
		_briskExtractor.computeImpl(image, keypoints, descriptors);
	}

	// we use members here instead of (triple) inheritance, in order to avoid diamonds of death
	brisk::ScaleSpaceFeatureDetector<brisk::HarrisScoreCalculator> _briskDetector;
	cv::BriskDescriptorExtractor _briskExtractor;

};




} // namespace brisk


#endif /* BRISKFEATURE_HPP_ */
