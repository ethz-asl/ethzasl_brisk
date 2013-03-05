/*
 * ScaleSpaceLayer.hpp
 *
 *  Created on: Aug 5, 2012
 *      Author: lestefan
 */

#include <sys/time.h>

template<class SCORE_CALCULTAOR_T>
ScaleSpaceLayer<SCORE_CALCULTAOR_T>::ScaleSpaceLayer(const cv::Mat& img, bool initScores){
	create(img);
}

template<class SCORE_CALCULTAOR_T>
void ScaleSpaceLayer<SCORE_CALCULTAOR_T>::create(const cv::Mat& img, bool initScores){
	// octave 0
	_isOctave=true;
	_layerNumber=0;

	// no adjacent layers (yet)
	_belowLayer_ptr=0;
	_aboveLayer_ptr=0;

	// pass image and initialize score calculation
	_scoreCalculator.setImage(img,initScores);
	_img=img;

	// scales and offsets
	_offset_above=-0.25;
	_offset_below=1.0/6.0;
	_scale_above=2.0/3.0;
	_scale_below=4.0/3.0;
	_scale=1.0;
	_offset=0.0;

	// by default no uniformity radius
	_radius=1;

	// abs. threshold (for noise rejection)
	_absoluteThreshold=0;

	// generic mask
	_LUT=cv::Mat::zeros(2*16-1,2*16-1,CV_32F);
	for(int x=0; x<2*16-1; ++x){
		for(int y=0; y<2*16-1; ++y){
			_LUT.at<float>(y,x)=
					std::max(1-double((15-x)*(15-x)+(15-y)*(15-y))/double(15*15),0.0);
			//std::cout<<_LUT.at<float>(y,x)<<" ";
		}
		//std::cout<<std::endl;
	}

}

template<class SCORE_CALCULTAOR_T>
ScaleSpaceLayer<SCORE_CALCULTAOR_T>::ScaleSpaceLayer(ScaleSpaceLayer<ScoreCalculator_t>* layerBelow, bool initScores){
	create(layerBelow,initScores);
}

template<class SCORE_CALCULTAOR_T>
void ScaleSpaceLayer<SCORE_CALCULTAOR_T>::create(ScaleSpaceLayer<ScoreCalculator_t>* layerBelow, bool initScores){
	// for successive construction
	int type=layerBelow->_img.type();
	if(layerBelow->_isOctave){
		if(layerBelow->_layerNumber>=2){
			// we can do the (cheaper) halfsampling
			_img.create(layerBelow->_belowLayer_ptr->_img.rows/2,
					layerBelow->_belowLayer_ptr->_img.cols/2,type);
			halfsample(layerBelow->_belowLayer_ptr->_img, _img);

		}
		else{
			// we do the two-third sampling
			_img.create((layerBelow->_img.rows/3)*2,(layerBelow->_img.cols/3)*2,type);
			twothirdsample(layerBelow->_img, _img);
		}
		// keep track of where in the pyramid we are
		_isOctave=false;
	}
	else{
		// we can do the (cheaper) halfsampling
		_img.create(layerBelow->_belowLayer_ptr->_img.rows/2,
				layerBelow->_belowLayer_ptr->_img.cols/2,type);
		halfsample(layerBelow->_belowLayer_ptr->_img, _img);

		// keep track of where in the pyramid we are
		_isOctave=true;
	}
	// keep track
	_layerNumber=layerBelow->_layerNumber+1;
	_belowLayer_ptr=layerBelow;
	layerBelow->_aboveLayer_ptr=this;

	// calculate coordinate transformation parameters
	if(_isOctave){
		_offset_above=-0.25;
		_offset_below=1.0/6.0;
		_scale_above=2.0/3.0;
		_scale_below=4.0/3.0;
		_scale=pow(2.0,double(_layerNumber/2));
		_offset=_scale*0.5-0.5;
	}
	else{
		_offset_above=-1.0/6.0;
		_offset_below=0.125;
		_scale_above=0.75;
		_scale_below=1.5;
		_scale=pow(2.0,double(_layerNumber/2))*1.5;
		_offset=_scale*0.5-0.5;
	}

	// by default no uniformity radius
	_radius=1;

	// abs. threshold (for noise rejection)
	_absoluteThreshold=0;

	// initialize the score calculation
	_scoreCalculator.setImage(_img,initScores);

	// the above layer is undefined:
	_aboveLayer_ptr=0;

	// generic mask
	_LUT=cv::Mat::zeros(2*16-1,2*16-1,CV_32F);
	for(int x=0; x<2*16-1; ++x){
		for(int y=0; y<2*16-1; ++y){
			_LUT.at<float>(y,x)=
					std::max(1-double((15-x)*(15-x)+(15-y)*(15-y))/double(15*15),0.0);
			//std::cout<<_LUT.at<float>(y,x)<<" ";
		}
		//std::cout<<std::endl;
	}
}

template<class SCORE_CALCULTAOR_T>
void ScaleSpaceLayer<SCORE_CALCULTAOR_T>::setUniformityRadius(double radius){
	_radius=radius;
	if(radius==0)
		_radius=1;
}

// used for sorting
template<class SCORE_CALCULTAOR_T>
__inline__ bool compareKeypointScore (
		typename SCORE_CALCULTAOR_T::PointWithScore kp_i,
		typename SCORE_CALCULTAOR_T::PointWithScore kp_j) {
	return (kp_i.score>kp_j.score);
}

// feature detection
template<class SCORE_CALCULTAOR_T>
void ScaleSpaceLayer<SCORE_CALCULTAOR_T>::detectScaleSpaceMaxima(std::vector<cv::KeyPoint>& keypoints,
		bool enforceUniformity, bool doRefinement, bool usePassedKeypoints){
	// first get the maxima points inside this layer
	std::vector<typename ScoreCalculator_t::PointWithScore> points;
	if(usePassedKeypoints){
		//std::cout<<"passed keypoints"<<std::endl;
		points.reserve(keypoints.size());
		for(size_t k=0; k<keypoints.size(); ++k){
			points.push_back(typename ScoreCalculator_t::PointWithScore(cv::Point2i(keypoints[k].pt.x,keypoints[k].pt.y),keypoints[k].response));
		}
	}
	else{
		//std::cout<<"calc keypoints"<<std::endl;
		_scoreCalculator.get2dMaxima(points,_absoluteThreshold);
	}

	// next check above and below. The code looks a bit stupid, but that's
	// for speed. We don't want to make the distinction analyzing whether or
	// not there is a layer above and below inside the loop.
	if(!usePassedKeypoints){
		if(_aboveLayer_ptr!=0&&_belowLayer_ptr!=0){
			// check above and below
			std::vector<typename ScoreCalculator_t::PointWithScore> pt_tmp;
			pt_tmp.reserve(points.size());
			const int one_over_scale_above = 1.0/_scale_above;
			const int one_over_scale_below = 1.0/_scale_below;
			for(typename std::vector<typename ScoreCalculator_t::PointWithScore>::const_iterator it=
					points.begin(); it!=points.end(); ++it){
				const typename ScoreCalculator_t::Score_t center=it->score;
				const cv::Point2i& pt=it->pt;
				if(center < (typename ScoreCalculator_t::Score_t)(_absoluteThreshold)) continue;
				if(center<scoreAbove(pt.x,pt.y)) continue;
				if(center<scoreBelow(pt.x,pt.y)) continue;
				if(center<scoreAbove(pt.x+one_over_scale_above,pt.y)) continue;
				if(center<scoreBelow(pt.x+one_over_scale_below,pt.y)) continue;
				if(center<scoreAbove(pt.x-one_over_scale_above,pt.y)) continue;
				if(center<scoreBelow(pt.x-one_over_scale_below,pt.y)) continue;
				if(center<scoreAbove(pt.x,pt.y+one_over_scale_above)) continue;
				if(center<scoreBelow(pt.x,pt.y+one_over_scale_below)) continue;
				if(center<scoreAbove(pt.x,pt.y-one_over_scale_above)) continue;
				if(center<scoreBelow(pt.x,pt.y-one_over_scale_below)) continue;
				if(center<scoreAbove(pt.x+one_over_scale_above,pt.y+one_over_scale_above)) continue;
				if(center<scoreBelow(pt.x+one_over_scale_below,pt.y+one_over_scale_below)) continue;
				if(center<scoreAbove(pt.x+one_over_scale_above,pt.y-one_over_scale_above)) continue;
				if(center<scoreBelow(pt.x+one_over_scale_below,pt.y-one_over_scale_below)) continue;
				if(center<scoreAbove(pt.x-one_over_scale_above,pt.y+one_over_scale_above)) continue;
				if(center<scoreBelow(pt.x-one_over_scale_below,pt.y+one_over_scale_below)) continue;
				if(center<scoreAbove(pt.x-one_over_scale_above,pt.y-one_over_scale_above)) continue;
				if(center<scoreBelow(pt.x-one_over_scale_below,pt.y-one_over_scale_below)) continue;
				pt_tmp.push_back(*it);
			}
			points.assign(pt_tmp.begin(),pt_tmp.end());
		}
		else if(_aboveLayer_ptr!=0){
			// check above
			std::vector<typename ScoreCalculator_t::PointWithScore> pt_tmp;
			pt_tmp.reserve(points.size());
			const int one_over_scale_above = 1.0/_scale_above;
			for(typename std::vector<typename ScoreCalculator_t::PointWithScore>::const_iterator it=
					points.begin(); it!=points.end(); ++it){
				const typename ScoreCalculator_t::Score_t center=it->score;
				if(center < (typename ScoreCalculator_t::Score_t)(_absoluteThreshold)) continue;
				const cv::Point2i& pt=it->pt;
				if(center<scoreAbove(pt.x,pt.y)) continue;
				if(center<scoreAbove(pt.x+one_over_scale_above,pt.y)) continue;
				if(center<scoreAbove(pt.x-one_over_scale_above,pt.y)) continue;
				if(center<scoreAbove(pt.x,pt.y+one_over_scale_above)) continue;
				if(center<scoreAbove(pt.x,pt.y-one_over_scale_above)) continue;
				if(center<scoreAbove(pt.x+one_over_scale_above,pt.y+one_over_scale_above)) continue;
				if(center<scoreAbove(pt.x+one_over_scale_above,pt.y-one_over_scale_above)) continue;
				if(center<scoreAbove(pt.x-one_over_scale_above,pt.y+one_over_scale_above)) continue;
				if(center<scoreAbove(pt.x-one_over_scale_above,pt.y-one_over_scale_above)) continue;
				pt_tmp.push_back(*it);
			}
			points.assign(pt_tmp.begin(),pt_tmp.end());

		}
		else if(_belowLayer_ptr!=0){
			// check below
			std::vector<typename ScoreCalculator_t::PointWithScore> pt_tmp;
			pt_tmp.reserve(points.size());
			const int one_over_scale_below = 1.0/_scale_below;
			for(typename std::vector<typename ScoreCalculator_t::PointWithScore>::const_iterator it=
					points.begin(); it!=points.end(); ++it){
				const typename ScoreCalculator_t::Score_t center=it->score;
				if(center < (typename ScoreCalculator_t::Score_t)(_absoluteThreshold)) continue;
				const cv::Point2i& pt=it->pt;
				if(center<scoreBelow(pt.x,pt.y)) continue;
				if(center<scoreBelow(pt.x+one_over_scale_below,pt.y)) continue;
				if(center<scoreBelow(pt.x-one_over_scale_below,pt.y)) continue;
				if(center<scoreBelow(pt.x,pt.y+one_over_scale_below)) continue;
				if(center<scoreBelow(pt.x,pt.y-one_over_scale_below)) continue;
				if(center<scoreBelow(pt.x+one_over_scale_below,pt.y+one_over_scale_below)) continue;
				if(center<scoreBelow(pt.x+one_over_scale_below,pt.y-one_over_scale_below)) continue;
				if(center<scoreBelow(pt.x-one_over_scale_below,pt.y+one_over_scale_below)) continue;
				if(center<scoreBelow(pt.x-one_over_scale_below,pt.y-one_over_scale_below)) continue;
				pt_tmp.push_back(*it);
			}
			points.assign(pt_tmp.begin(),pt_tmp.end());
		}
	}

	// uniformity enforcement
	if(points.size()==0) return;
	if(enforceUniformity&&_radius>0.0){

		std::vector<typename ScoreCalculator_t::PointWithScore> pt_tmp;

		// sort
		std::sort(points.begin(), points.end(), compareKeypointScore<SCORE_CALCULTAOR_T>);
		const float maxScore=points.front().score;

		keypoints.reserve(keypoints.size()+points.size()); // allow appending

		// store occupancy
		cv::Mat occupancy;
		const float scaling=15.0/float(_radius);
		occupancy=cv::Mat::zeros((_img.rows)*ceil(scaling)+32,(_img.cols)*ceil(scaling)+32,CV_8U);

		// go through the sorted keypoints and reject too close ones

		for(typename std::vector<typename ScoreCalculator_t::PointWithScore>::const_iterator it=points.begin();
				it!=points.end(); ++it){

			const int cy=(it->pt.y*scaling+16);
			//std::cout<<cy<<" "<<it->pt.y<<std::endl;
			const int cx=(it->pt.x*scaling+16);

			// check if this is a high enough score
			const double s0=double(occupancy.at<uchar>(cy,cx));
			//const double s1=s0*s0;
			const float nsc1=sqrtf(sqrtf(it->score/maxScore))*255.0f;

			if(nsc1<s0)
				continue;

			// masks
			const float nsc=0.99*nsc1;
			//std::cout<<it->score<<std::endl;

			//std::cout<<nsc<<std::endl;
			for(int y=0; y<2*16-1; ++y){
				__m128i mem1=_mm_loadu_si128 ((__m128i*)&occupancy.at<uchar>(cy+y-15,cx-15));
				__m128i mem2=_mm_loadu_si128 ((__m128i*)&occupancy.at<uchar>(cy+y-15,cx+1));
				__m128i mask1 = _mm_set_epi8(ceil(_LUT.at<float>(y,15)*nsc),ceil(_LUT.at<float>(y,14)*nsc),ceil(_LUT.at<float>(y,13)*nsc),ceil(_LUT.at<float>(y,12)*nsc),ceil(_LUT.at<float>(y,11)*nsc),ceil(_LUT.at<float>(y,10)*nsc),ceil(_LUT.at<float>(y,9)*nsc),ceil(_LUT.at<float>(y,8)*nsc),ceil(_LUT.at<float>(y,7)*nsc),ceil(_LUT.at<float>(y,6)*nsc),ceil(_LUT.at<float>(y,5)*nsc),ceil(_LUT.at<float>(y,4)*nsc),ceil(_LUT.at<float>(y,3)*nsc),ceil(_LUT.at<float>(y,2)*nsc),ceil(_LUT.at<float>(y,1)*nsc),ceil(_LUT.at<float>(y,0)*nsc));
				__m128i mask2 = _mm_set_epi8(0,ceil(_LUT.at<float>(y,30)*nsc),ceil(_LUT.at<float>(y,29)*nsc),ceil(_LUT.at<float>(y,28)*nsc),ceil(_LUT.at<float>(y,27)*nsc),ceil(_LUT.at<float>(y,26)*nsc),ceil(_LUT.at<float>(y,25)*nsc),ceil(_LUT.at<float>(y,24)*nsc),ceil(_LUT.at<float>(y,23)*nsc),ceil(_LUT.at<float>(y,22)*nsc),ceil(_LUT.at<float>(y,21)*nsc),ceil(_LUT.at<float>(y,20)*nsc),_LUT.at<float>(y,19)*nsc,ceil(_LUT.at<float>(y,18)*nsc),ceil(_LUT.at<float>(y,17)*nsc),ceil(_LUT.at<float>(y,16)*nsc));
				_mm_storeu_si128((__m128i*)&occupancy.at<uchar>(cy+y-15,cx-15),
						_mm_max_epu8 (mem1, mask1));
				_mm_storeu_si128((__m128i*)&occupancy.at<uchar>(cy+y-15,cx+1),
						_mm_max_epu8 (mem2, mask2));
			}

			// store
			pt_tmp.push_back(*it);
			//std::cout <<  it->pt << cv::Point2f(_scale*(float(it->pt.x)+_offset),_scale*(float(it->pt.y)+_offset))<<std::endl;
		}
		points.assign(pt_tmp.begin(),pt_tmp.end());

		//cv::imshow("Raw",occupancy);
		//cv::waitKey();
	}


	// 3d(/2d) subpixel refinement!!
	if(usePassedKeypoints)
		keypoints.clear();
	if(doRefinement){
		for(typename std::vector<typename ScoreCalculator_t::PointWithScore>::const_iterator it=points.begin();
						it!=points.end(); ++it){
			const int u=it->pt.x;
			const int v=it->pt.y;
			//std::cout<<it->score<<std::endl;
			float delta_x;
			float delta_y;
			subpixel2D(_scoreCalculator.score(u-1,v-1), _scoreCalculator.score(u,v-1), _scoreCalculator.score(u+1,v-1),
					_scoreCalculator.score(u-1,v), _scoreCalculator.score(u,v), _scoreCalculator.score(u+1,v),
					_scoreCalculator.score(u-1,v+1), _scoreCalculator.score(u,v+1), _scoreCalculator.score(u+1,v+1),
					delta_x, delta_y);
			// todo: 3d refinement
			keypoints.push_back(cv::KeyPoint(cv::Point2f(_scale*((it->pt.x+delta_x)+_offset),_scale*((it->pt.y+delta_y)+_offset)),_scale*12.0, -1,
					it->score, _layerNumber/2));
		}
	}
	else{
		for(typename std::vector<typename ScoreCalculator_t::PointWithScore>::const_iterator it=points.begin();
								it!=points.end(); ++it){
			keypoints.push_back(cv::KeyPoint(cv::Point2f(_scale*((it->pt.x)+_offset),_scale*((it->pt.y)+_offset)),_scale*12.0, -1,
								it->score, _layerNumber/2));
		}
	}
}

// utilities
template<class SCORE_CALCULTAOR_T>
inline double ScaleSpaceLayer<SCORE_CALCULTAOR_T>::scoreAbove(double u, double v){
	//std::cout<<_layerNumber<<" score above for "<<u<<","<<v<<" _scale_above="<<_scale_above<<std::endl;
	return _aboveLayer_ptr->_scoreCalculator.score(_scale_above*(u+_offset_above),_scale_above*(v+_offset_above));
}
template<class SCORE_CALCULTAOR_T>
inline double ScaleSpaceLayer<SCORE_CALCULTAOR_T>::scoreBelow(double u, double v){
	return _belowLayer_ptr->_scoreCalculator.score(_scale_below*(u+_offset_below),_scale_below*(v+_offset_below));
}

// half sampling
template<class SCORE_CALCULTAOR_T>
inline bool ScaleSpaceLayer<SCORE_CALCULTAOR_T>::halfsample(const cv::Mat& srcimg, cv::Mat& dstimg){
	//std::cout<<srcimg.cols<<" "<<dstimg.cols<<std::endl;
	std::cout.flush();
	if(srcimg.type()==CV_8UC1){
		halfsample8(srcimg,dstimg);
	}
	else if(srcimg.type()==CV_16UC1){
		halfsample16(srcimg,dstimg);
	}
	else{
		return false;
	}
	return true;
}

// half sampling
template<class SCORE_CALCULTAOR_T>
inline void ScaleSpaceLayer<SCORE_CALCULTAOR_T>::halfsample16(const cv::Mat& srcimg, cv::Mat& dstimg){
	// make sure the destination image is of the right size:
	assert(srcimg.cols/2==dstimg.cols);
	assert(srcimg.rows/2==dstimg.rows);
	assert(srcimg.type()==CV_16UC1);

	//const int cols=(srcimg.cols/2)*2;
	const int colsMax=(srcimg.cols/2)*2-16;
	const int rows=(srcimg.rows/2)*2-1;

	const register __m128i ones = _mm_set_epi16 (1,1,1,1,1,1,1,1);
	for(int y=0; y<rows; y+=2){
		bool end=false;
		int x_store=0;
		for(int x=0; x<=colsMax; x+=16){

			assert(x+15<srcimg.cols);
			assert(y+1<srcimg.rows);
			// load block of four
			__m128i i00=_mm_set_epi16(srcimg.at<uint16_t>(y,x+14),srcimg.at<uint16_t>(y,x+12),srcimg.at<uint16_t>(y,x+10),srcimg.at<uint16_t>(y,x+8),srcimg.at<uint16_t>(y,x+6),srcimg.at<uint16_t>(y,x+4),srcimg.at<uint16_t>(y,x+2),srcimg.at<uint16_t>(y,x));
			__m128i i01=_mm_set_epi16(srcimg.at<uint16_t>(y,x+15),srcimg.at<uint16_t>(y,x+13),srcimg.at<uint16_t>(y,x+11),srcimg.at<uint16_t>(y,x+9),srcimg.at<uint16_t>(y,x+7),srcimg.at<uint16_t>(y,x+5),srcimg.at<uint16_t>(y,x+3),srcimg.at<uint16_t>(y,x+1));
			const int y1=y+1;
			__m128i i10=_mm_set_epi16(srcimg.at<uint16_t>(y1,x+14),srcimg.at<uint16_t>(y1,x+12),srcimg.at<uint16_t>(y1,x+10),srcimg.at<uint16_t>(y1,x+8),srcimg.at<uint16_t>(y1,x+6),srcimg.at<uint16_t>(y1,x+4),srcimg.at<uint16_t>(y1,x+2),srcimg.at<uint16_t>(y1,x));
			__m128i i11=_mm_set_epi16(srcimg.at<uint16_t>(y1,x+15),srcimg.at<uint16_t>(y1,x+13),srcimg.at<uint16_t>(y1,x+11),srcimg.at<uint16_t>(y1,x+9),srcimg.at<uint16_t>(y1,x+7),srcimg.at<uint16_t>(y1,x+5),srcimg.at<uint16_t>(y1,x+3),srcimg.at<uint16_t>(y1,x+1));

			// average
			i10=_mm_adds_epu16(i10,ones);
			__m128i result1=_mm_avg_epu16(i00,i01);
			i10=_mm_adds_epu16(i10,ones);
			__m128i result2=_mm_avg_epu16(i10,i11);

			__m128i result=_mm_avg_epu16(result1,result2);

			// store
			assert(x_store+7<dstimg.cols);
			assert(y/2<dstimg.rows);
			//std::cout<<dstimg.at<uint16_t>(y/2,x/2+7)<<"  ";
			//std::cout<<y/2<<","<<x/2<<":"<<dstimg.cols<<std::endl;
			//std::cout.flush();
			_mm_storeu_si128((__m128i*)&(dstimg.at<uint16_t>(y/2,x_store)),result);

			x_store+=8;

			if(end) break;
			if(x+16>=colsMax){
				x=colsMax-16;
				x_store=dstimg.cols-8;
				end=true;
			}
		}
	}
}

// half sampling
template<class SCORE_CALCULTAOR_T>
inline void ScaleSpaceLayer<SCORE_CALCULTAOR_T>::halfsample8(const cv::Mat& srcimg, cv::Mat& dstimg){
	const unsigned short leftoverCols = ((srcimg.cols%16)/2);// take care with border...
	const bool noleftover = (srcimg.cols%16)==0; // note: leftoverCols can be zero but this still false...

	// make sure the destination image is of the right size:
	assert(srcimg.cols/2==dstimg.cols);
	assert(srcimg.rows/2==dstimg.rows);

	// mask needed later:
	register __m128i mask = _mm_set_epi32 (0x00FF00FF, 0x00FF00FF, 0x00FF00FF, 0x00FF00FF);
	// to be added in order to make successive averaging correct:
	register __m128i ones = _mm_set_epi8 (1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1);

	// data pointers:
	__m128i* p1=(__m128i*)srcimg.data;
	__m128i* p2=(__m128i*)(srcimg.data+srcimg.cols);
	__m128i* p_dest=(__m128i*)dstimg.data;
	unsigned char* p_dest_char;//=(unsigned char*)p_dest;

	// size:
	const unsigned int size = (srcimg.cols*srcimg.rows)/16;
	const unsigned int hsize = srcimg.cols/16;
	__m128i* p_end=p1+size;
	unsigned int row=0;
	const unsigned int end=hsize/2;
	bool half_end;
	if(hsize%2==0)
		half_end=false;
	else
		half_end=true;
	while(p2<p_end){
		for(unsigned int i=0; i<end;i++){
			// load the two blocks of memory:
			__m128i upper;
			__m128i lower;
			if(noleftover){
				upper=_mm_load_si128(p1);
				lower=_mm_load_si128(p2);
			}
			else{
				upper=_mm_loadu_si128(p1);
				lower=_mm_loadu_si128(p2);
			}

			__m128i result1=_mm_adds_epu8 (upper, ones);
			result1=_mm_avg_epu8 (upper, lower);

			// increment the pointers:
			p1++;
			p2++;

			// load the two blocks of memory:
			upper=_mm_loadu_si128(p1);
			lower=_mm_loadu_si128(p2);
			__m128i result2=_mm_adds_epu8 (upper, ones);
			result2=_mm_avg_epu8 (upper, lower);
			// calculate the shifted versions:
			__m128i result1_shifted = _mm_srli_si128 (result1, 1);
			__m128i result2_shifted = _mm_srli_si128 (result2, 1);
			// pack:
			__m128i result=_mm_packus_epi16 (_mm_and_si128 (result1, mask),
					_mm_and_si128 (result2, mask));
			__m128i result_shifted = _mm_packus_epi16 (_mm_and_si128 (result1_shifted, mask),
					_mm_and_si128 (result2_shifted, mask));
			// average for the second time:
			result=_mm_avg_epu8(result,result_shifted);

			// store to memory
			_mm_storeu_si128 (p_dest, result);

			// increment the pointers:
			p1++;
			p2++;
			p_dest++;
			//p_dest_char=(unsigned char*)p_dest;
		}
		// if we are not at the end of the row, do the rest:
		if(half_end){
			// load the two blocks of memory:
			__m128i upper;
			__m128i lower;
			if(noleftover){
				upper=_mm_load_si128(p1);
				lower=_mm_load_si128(p2);
			}
			else{
				upper=_mm_loadu_si128(p1);
				lower=_mm_loadu_si128(p2);
			}

			__m128i result1=_mm_adds_epu8 (upper, ones);
			result1=_mm_avg_epu8 (upper, lower);

			// increment the pointers:
			p1++;
			p2++;

			// compute horizontal pairwise average and store
			p_dest_char=(unsigned char*)p_dest;
			const uchar* result=(uchar*)&result1;
			for(unsigned int j=0; j<8; j++){
				*(p_dest_char++)=(*(result+2*j)+*(result+2*j+1))/2;
			}
			//p_dest_char=(unsigned char*)p_dest;
		}
		else{
			p_dest_char=(unsigned char*)p_dest;
		}

		if(noleftover){
			row++;
			p_dest=(__m128i*)(dstimg.data+row*dstimg.cols);
			p1=(__m128i*)(srcimg.data+2*row*srcimg.cols);
			//p2=(__m128i*)(srcimg.data+(2*row+1)*srcimg.cols);
			//p1+=hsize;
			p2=p1+hsize;
		}
		else{
			const unsigned char* p1_src_char=(unsigned char*)(p1);
			const unsigned char* p2_src_char=(unsigned char*)(p2);
			for(unsigned int k=0; k<leftoverCols; k++){
				unsigned short tmp = p1_src_char[k]+p1_src_char[k+1]+
						p2_src_char[k]+p2_src_char[k+1];
				*(p_dest_char++)=(unsigned char)(tmp/4);
			}
			// done with the two rows:
			row++;
			p_dest=(__m128i*)(dstimg.data+row*dstimg.cols);
			p1=(__m128i*)(srcimg.data+2*row*srcimg.cols);
			p2=(__m128i*)(srcimg.data+(2*row+1)*srcimg.cols);
		}
	}
}

template<class SCORE_CALCULTAOR_T>
inline bool ScaleSpaceLayer<SCORE_CALCULTAOR_T>::twothirdsample(const cv::Mat& srcimg, cv::Mat& dstimg){
	//std::cout<<srcimg.cols<<" "<<dstimg.cols<<std::endl;
	std::cout.flush();
	if(srcimg.type()==CV_8UC1){
		twothirdsample8(srcimg,dstimg);
	}
	else if(srcimg.type()==CV_16UC1){
		twothirdsample16(srcimg,dstimg);
	}
	else{
		return false;
	}
	return true;
}

template<class SCORE_CALCULTAOR_T>
inline void ScaleSpaceLayer<SCORE_CALCULTAOR_T>::twothirdsample16(const cv::Mat& srcimg, cv::Mat& dstimg){
	assert(srcimg.type()==CV_16UC1);

	// make sure the destination image is of the right size:
	assert((srcimg.cols/3)*2==dstimg.cols);
	assert((srcimg.rows/3)*2==dstimg.rows);

	//const int cols=(srcimg.cols/2)*2;
	const int colsMax=(srcimg.cols/3)*3-12;
	const int rows=(srcimg.rows/3)*3-2;

	for(int y=0; y<rows; y+=3){
		bool end=false;
		int x_store=0;
		for(int x=0; x<=colsMax; x+=12){

			assert(x+11<srcimg.cols);
			assert(y+2<srcimg.rows);

			// use 2 3x3 blocks
			const int y1=y+1;
			const int y2=y1+1;

			// assemble epi32 registers
			// top row
			__m128i i0_corners=_mm_set_epi32(srcimg.at<uint16_t>(y,x+5),srcimg.at<uint16_t>(y,x+3),srcimg.at<uint16_t>(y,x+2),srcimg.at<uint16_t>(y,x));
			i0_corners=_mm_slli_epi32(i0_corners,2); //*4
			const uint32_t m01=srcimg.at<uint16_t>(y,x+1)<<1;//*2
			const uint32_t m04=srcimg.at<uint16_t>(y,x+4)<<1;//*2
			__m128i i0_middle=_mm_set_epi32(m04,m04,m01,m01);

			// middle row
			__m128i i1_leftright=_mm_set_epi32(srcimg.at<uint16_t>(y1,x+5),srcimg.at<uint16_t>(y1,x+3),srcimg.at<uint16_t>(y1,x+2),srcimg.at<uint16_t>(y1,x));
			i1_leftright=_mm_slli_epi32(i1_leftright,1); //*2
			const uint32_t m11=srcimg.at<uint16_t>(y1,x+1);
			const uint32_t m14=srcimg.at<uint16_t>(y1,x+4);
			__m128i i1_middle=_mm_set_epi32(m14,m14,m11,m11);

			// bottom row
			__m128i i2_corners=_mm_set_epi32(srcimg.at<uint16_t>(y2,x+5),srcimg.at<uint16_t>(y2,x+3),srcimg.at<uint16_t>(y2,x+2),srcimg.at<uint16_t>(y2,x));
			i2_corners=_mm_slli_epi32(i2_corners,2); //*4
			const uint32_t m21=srcimg.at<uint16_t>(y2,x+1)<<1;//*2
			const uint32_t m24=srcimg.at<uint16_t>(y2,x+4)<<1;//*2
			__m128i i2_middle=_mm_set_epi32(m24,m24,m21,m21);

			// average
			__m128i result1=_mm_add_epi32(i1_middle,i1_leftright);
			// top output row
			__m128i result0=_mm_add_epi32(i0_corners,i0_middle);
			result0=_mm_add_epi32(result0,result1);

			// bottom output row
			__m128i result2=_mm_add_epi32(i2_corners,i2_middle);
			result2=_mm_add_epi32(result2,result1);

			// assemble epi32 registers---right blocks
			const int xp=x+6;
			// top row
			i0_corners=_mm_set_epi32(srcimg.at<uint16_t>(y,xp+5),srcimg.at<uint16_t>(y,xp+3),srcimg.at<uint16_t>(y,xp+2),srcimg.at<uint16_t>(y,xp));
			i0_corners=_mm_slli_epi32(i0_corners,2); //*4
			const uint32_t m01p=srcimg.at<uint16_t>(y,xp+1)<<1;//*2
			const uint32_t m04p=srcimg.at<uint16_t>(y,xp+4)<<1;//*2
			i0_middle=_mm_set_epi32(m04p,m04p,m01p,m01p);

			// middle row
			i1_leftright=_mm_set_epi32(srcimg.at<uint16_t>(y1,xp+5),srcimg.at<uint16_t>(y1,xp+3),srcimg.at<uint16_t>(y1,xp+2),srcimg.at<uint16_t>(y1,xp));
			i1_leftright=_mm_slli_epi32(i1_leftright,1); //*2
			const uint32_t m11p=srcimg.at<uint16_t>(y1,xp+1);
			const uint32_t m14p=srcimg.at<uint16_t>(y1,xp+4);
			i1_middle=_mm_set_epi32(m14p,m14p,m11p,m11p);

			// bottom row
			i2_corners=_mm_set_epi32(srcimg.at<uint16_t>(y2,xp+5),srcimg.at<uint16_t>(y2,xp+3),srcimg.at<uint16_t>(y2,xp+2),srcimg.at<uint16_t>(y2,xp));
			i2_corners=_mm_slli_epi32(i2_corners,2); //*4
			const uint32_t m21p=srcimg.at<uint16_t>(y2,xp+1)<<1;//*2
			const uint32_t m24p=srcimg.at<uint16_t>(y2,xp+4)<<1;//*2
			i2_middle=_mm_set_epi32(m24p,m24p,m21p,m21p);

			// average
			result1=_mm_add_epi32(i1_middle,i1_leftright);
			// top output row
			__m128i result0p=_mm_add_epi32(i0_corners,i0_middle);
			result0p=_mm_add_epi32(result0p,result1);

			// bottom output row
			__m128i result2p=_mm_add_epi32(i2_corners,i2_middle);
			result2p=_mm_add_epi32(result2p,result1);

			// divide by 9 - not sure if this is very safe...
			((int*)&result0p)[0]/=9;((int*)&result0p)[1]/=9;((int*)&result0p)[2]/=9;((int*)&result0p)[3]/=9;
			((int*)&result2p)[0]/=9;((int*)&result2p)[1]/=9;((int*)&result2p)[2]/=9;((int*)&result2p)[3]/=9;
			((int*)&result0)[0]/=9;((int*)&result0)[1]/=9;((int*)&result0)[2]/=9;((int*)&result0)[3]/=9;
			((int*)&result2)[0]/=9;((int*)&result2)[1]/=9;((int*)&result2)[2]/=9;((int*)&result2)[3]/=9;

			// pack
			__m128i store0=_mm_packs_epi32 (result0,result0p);
			__m128i store2=_mm_packs_epi32 (result2,result2p);

			// store
			assert(x_store+7<dstimg.cols);
			assert(y/3*2<dstimg.rows);
			assert(y/3*2+1<dstimg.rows);
			_mm_storeu_si128((__m128i*)&(dstimg.at<uint16_t>(y/3*2,x_store)),store0);
			_mm_storeu_si128((__m128i*)&(dstimg.at<uint16_t>(y/3*2+1,x_store)),store2);

			x_store+=8;

			if(end) break;
			if(x+12>=colsMax){
				x=colsMax-12;
				x_store=dstimg.cols-8;
				end=true;
			}


		}
	}
}

template<class SCORE_CALCULTAOR_T>
inline void ScaleSpaceLayer<SCORE_CALCULTAOR_T>::twothirdsample8(const cv::Mat& srcimg, cv::Mat& dstimg){
	const unsigned short leftoverCols = ((srcimg.cols/3)*3)%15;// take care with border...

	// make sure the destination image is of the right size:
	assert((srcimg.cols/3)*2==dstimg.cols);
	assert((srcimg.rows/3)*2==dstimg.rows);

	// masks:
	register __m128i mask1 = _mm_set_epi8 (0x80,0x80,0x80,0x80,0x80,0x80,0x80,12,0x80,10,0x80,7,0x80,4,0x80,1);
	register __m128i mask2 = _mm_set_epi8 (0x80,0x80,0x80,0x80,0x80,0x80,12,0x80,10,0x80,7,0x80,4,0x80,1,0x80);
	register __m128i mask = _mm_set_epi8 (0x80,0x80,0x80,0x80,0x80,0x80,14,12,11,9,8,6,5,3,2,0);
	register __m128i store_mask = _mm_set_epi8 (0,0,0,0,0,0,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80);

	// data pointers:
	unsigned char* p1=srcimg.data;
	unsigned char* p2=p1+srcimg.cols;
	unsigned char* p3=p2+srcimg.cols;
	unsigned char* p_dest1 = dstimg.data;
	unsigned char* p_dest2 = p_dest1+dstimg.cols;
	unsigned char* p_end=p1+(srcimg.cols*srcimg.rows);

	unsigned int row=0;
	unsigned int row_dest=0;
	int hsize = srcimg.cols/15;
	while(p3<p_end){
		for(int i=0; i<hsize; i++){
			// load three rows
			__m128i first = _mm_loadu_si128((__m128i*)p1);
			__m128i second = _mm_loadu_si128((__m128i*)p2);
			__m128i third = _mm_loadu_si128((__m128i*)p3);

			// upper row:
			__m128i upper = _mm_avg_epu8(_mm_avg_epu8(first,second),first);
			__m128i temp1_upper = _mm_or_si128(_mm_shuffle_epi8(upper,mask1),_mm_shuffle_epi8(upper,mask2));
			__m128i temp2_upper=_mm_shuffle_epi8(upper,mask);
			__m128i result_upper = _mm_avg_epu8(_mm_avg_epu8(temp2_upper,temp1_upper),temp2_upper);

			// lower row:
			__m128i lower = _mm_avg_epu8(_mm_avg_epu8(third,second),third);
			__m128i temp1_lower = _mm_or_si128(_mm_shuffle_epi8(lower,mask1),_mm_shuffle_epi8(lower,mask2));
			__m128i temp2_lower=_mm_shuffle_epi8(lower,mask);
			__m128i result_lower = _mm_avg_epu8(_mm_avg_epu8(temp2_lower,temp1_lower),temp2_lower);

			// store:
			if(i*10+16>dstimg.cols){
				_mm_maskmoveu_si128(result_upper, store_mask, (char*)p_dest1);
				_mm_maskmoveu_si128(result_lower, store_mask, (char*)p_dest2);
			}
			else{
				_mm_storeu_si128 ((__m128i*)p_dest1, result_upper);
				_mm_storeu_si128 ((__m128i*)p_dest2, result_lower);
			}

			// shift pointers:
			p1+=15;
			p2+=15;
			p3+=15;
			p_dest1+=10;
			p_dest2+=10;
		}

		// fill the remainder:
		for(unsigned int j = 0; j<leftoverCols;j+=3){
			const unsigned short A1=*(p1++);
			const unsigned short A2=*(p1++);
			const unsigned short A3=*(p1++);
			const unsigned short B1=*(p2++);
			const unsigned short B2=*(p2++);
			const unsigned short B3=*(p2++);
			const unsigned short C1=*(p3++);
			const unsigned short C2=*(p3++);
			const unsigned short C3=*(p3++);

			*(p_dest1++)=(unsigned char)(((4*A1+2*(A2+B1)+B2)/9)&0x00FF);
			*(p_dest1++)=(unsigned char)(((4*A3+2*(A2+B3)+B2)/9)&0x00FF);
			*(p_dest2++)=(unsigned char)(((4*C1+2*(C2+B1)+B2)/9)&0x00FF);
			*(p_dest2++)=(unsigned char)(((4*C3+2*(C2+B3)+B2)/9)&0x00FF);
		}


		// increment row counter:
		row+=3;
		row_dest+=2;

		// reset pointers
		p1=srcimg.data+row*srcimg.cols;
		p2=p1+srcimg.cols;
		p3=p2+srcimg.cols;
		p_dest1 = dstimg.data+row_dest*dstimg.cols;
		p_dest2 = p_dest1+dstimg.cols;
	}
}

template<class SCORE_CALCULTAOR_T>
__inline__ float ScaleSpaceLayer<SCORE_CALCULTAOR_T>::refine1D(const float s_05,
		const float s0, const float s05, float& max){
	int i_05=int(1024.0*s_05+0.5);
	int i0=int(1024.0*s0+0.5);
	int i05=int(1024.0*s05+0.5);

	//   16.0000  -24.0000    8.0000
	//  -40.0000   54.0000  -14.0000
	//   24.0000  -27.0000    6.0000

	int three_a=16*i_05-24*i0+8*i05;
	// second derivative must be negative:
	if(three_a>=0){
		if(s0>=s_05 && s0>=s05){
			max=s0;
			return 1.0;
		}
		if(s_05>=s0 && s_05>=s05){
			max=s_05;
			return 0.75;
		}
		if(s05>=s0 && s05>=s_05){
			max=s05;
			return 1.5;
		}
	}

	int three_b=-40*i_05+54*i0-14*i05;
	// calculate max location:
	float ret_val=-float(three_b)/float(2*three_a);
	// saturate and return
	if(ret_val<0.75) ret_val= 0.75;
	else if(ret_val>1.5) ret_val= 1.5; // allow to be slightly off bounds ...?
	int three_c = +24*i_05  -27*i0    +6*i05;
	max=float(three_c)+float(three_a)*ret_val*ret_val+float(three_b)*ret_val;
	max/=3072.0;
	return ret_val;
}

template<class SCORE_CALCULTAOR_T>
__inline__ float ScaleSpaceLayer<SCORE_CALCULTAOR_T>::refine1D_1(const float s_05,
		const float s0, const float s05, float& max){
	int i_05=int(1024.0*s_05+0.5);
	int i0=int(1024.0*s0+0.5);
	int i05=int(1024.0*s05+0.5);

	//  4.5000   -9.0000    4.5000
	//-10.5000   18.0000   -7.5000
	//  6.0000   -8.0000    3.0000

	int two_a=9*i_05-18*i0+9*i05;
	// second derivative must be negative:
	if(two_a>=0){
		if(s0>=s_05 && s0>=s05){
			max=s0;
			return 1.0;
		}
		if(s_05>=s0 && s_05>=s05){
			max=s_05;
			return 0.6666666666666666666666666667;
		}
		if(s05>=s0 && s05>=s_05){
			max=s05;
			return 1.3333333333333333333333333333;
		}
	}

	int two_b=-21*i_05+36*i0-15*i05;
	// calculate max location:
	float ret_val=-float(two_b)/float(2*two_a);
	// saturate and return
	if(ret_val<0.6666666666666666666666666667) ret_val= 0.666666666666666666666666667;
	else if(ret_val>1.33333333333333333333333333) ret_val= 1.333333333333333333333333333;
	int two_c = +12*i_05  -16*i0    +6*i05;
	max=float(two_c)+float(two_a)*ret_val*ret_val+float(two_b)*ret_val;
	max/=2048.0;
	return ret_val;
}

template<class SCORE_CALCULTAOR_T>
__inline__ float ScaleSpaceLayer<SCORE_CALCULTAOR_T>::subpixel2D(const double s_0_0, const double s_0_1, const double s_0_2,
		const double s_1_0, const double s_1_1, const double s_1_2,
		const double s_2_0, const double s_2_1, const double s_2_2,
		float& delta_x, float& delta_y){

	//std::cout<<s_0_0<<" "<<s_0_1<<" "<<s_0_2<<std::endl
	//		<<s_1_0<<" "<<s_1_1<<" "<<s_1_2<<std::endl
	//		<<s_2_0<<" "<<s_2_1<<" "<<s_2_2<<" "<<std::endl;

	// the coefficients of the 2d quadratic function least-squares fit:
	register double tmp1 =        s_0_0 + s_0_2 - 2*s_1_1 + s_2_0 + s_2_2;
	register double coeff1 = 3*(tmp1 + s_0_1 - ((s_1_0 + s_1_2)/2.0) + s_2_1);
	register double coeff2 = 3*(tmp1 - ((s_0_1+ s_2_1)/2.0) + s_1_0 + s_1_2 );
	register double tmp2 =                                  s_0_2 - s_2_0;
	register double tmp3 =                         (s_0_0 + tmp2 - s_2_2);
	register double tmp4 =                                   tmp3 -2*tmp2;
	register double coeff3 =                    -3*(tmp3 + s_0_1 - s_2_1);
	register double coeff4 =                    -3*(tmp4 + s_1_0 - s_1_2);
	register double coeff5 =            (s_0_0 - s_0_2 - s_2_0 + s_2_2)/4.0;
	register double coeff6 = -(s_0_0  + s_0_2 - ((s_1_0 + s_0_1 + s_1_2 + s_2_1)/2.0) - 5*s_1_1  + s_2_0  + s_2_2)/2.01;


	// 2nd derivative test:
	register double H_det=4*coeff1*coeff2 - coeff5*coeff5;

	if(H_det==0){
		delta_x=0.0;
		delta_y=0.0;
		return float(coeff6)/18.0;
	}

	if(!(H_det>0&&coeff1<0)){
		// The maximum must be at the one of the 4 patch corners.
		int tmp_max=coeff3+coeff4+coeff5;
		delta_x=1.0; delta_y=1.0;

		int tmp = -coeff3+coeff4-coeff5;
		if(tmp>tmp_max){
			tmp_max=tmp;
			delta_x=-1.0; delta_y=1.0;
		}
		tmp = coeff3-coeff4-coeff5;
		if(tmp>tmp_max){
			tmp_max=tmp;
			delta_x=1.0; delta_y=-1.0;
		}
		tmp = -coeff3-coeff4+coeff5;
		if(tmp>tmp_max){
			tmp_max=tmp;
			delta_x=-1.0; delta_y=-1.0;
		}
		return float(tmp_max+coeff1+coeff2+coeff6)/18.0;
	}

	// this is hopefully the normal outcome of the Hessian test
	delta_x=float(2*coeff2*coeff3 - coeff4*coeff5)/float(-H_det);
	delta_y=float(2*coeff1*coeff4 - coeff3*coeff5)/float(-H_det);
	// TODO: this is not correct, but easy, so perform a real boundary maximum search:
	bool tx=false; bool tx_=false; bool ty=false; bool ty_=false;
	if(delta_x>1.0) tx=true;
	else if(delta_x<-1.0) tx_=true;
	if(delta_y>1.0) ty=true;
	if(delta_y<-1.0) ty_=true;

	if(tx||tx_||ty||ty_){
		// get two candidates:
		float delta_x1=0.0, delta_x2=0.0, delta_y1=0.0, delta_y2=0.0;
		if(tx) {
			delta_x1=1.0;
			delta_y1=-float(coeff4+coeff5)/float(2*coeff2);
			if(delta_y1>1.0) delta_y1=1.0; else if (delta_y1<-1.0) delta_y1=-1.0;
		}
		else if(tx_) {
			delta_x1=-1.0;
			delta_y1=-float(coeff4-coeff5)/float(2*coeff2);
			if(delta_y1>1.0) delta_y1=1.0; else if (delta_y1<-1.0) delta_y1=-1.0;
		}
		if(ty) {
			delta_y2=1.0;
			delta_x2=-float(coeff3+coeff5)/float(2*coeff1);
			if(delta_x2>1.0) delta_x2=1.0; else if (delta_x2<-1.0) delta_x2=-1.0;
		}
		else if(ty_) {
			delta_y2=-1.0;
			delta_x2=-float(coeff3-coeff5)/float(2*coeff1);
			if(delta_x2>1.0) delta_x2=1.0; else if (delta_x2<-1.0) delta_x2=-1.0;
		}
		// insert both options for evaluation which to pick
		float max1 = (coeff1*delta_x1*delta_x1+coeff2*delta_y1*delta_y1
				+coeff3*delta_x1+coeff4*delta_y1
				+coeff5*delta_x1*delta_y1
				+coeff6)/18.0;
		float max2 = (coeff1*delta_x2*delta_x2+coeff2*delta_y2*delta_y2
				+coeff3*delta_x2+coeff4*delta_y2
				+coeff5*delta_x2*delta_y2
				+coeff6)/18.0;
		if(max1>max2) {
			delta_x=delta_x1;
			delta_y=delta_x1;
			return max1;
		}
		else{
			delta_x=delta_x2;
			delta_y=delta_x2;
			return max2;
		}
	}

	//std::cout << "delta_x="<<delta_x<< ", delta_y="<<delta_y<<std::endl;

	// this is the case of the maximum inside the boundaries:
	return (coeff1*delta_x*delta_x+coeff2*delta_y*delta_y
			+coeff3*delta_x+coeff4*delta_y
			+coeff5*delta_x*delta_y
			+coeff6)/18.0;
}
