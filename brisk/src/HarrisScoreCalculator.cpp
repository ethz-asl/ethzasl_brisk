/*
 * HarrisScoreCalculator.cpp
 *
 *  Created on: Aug 3, 2012
 *      Author: lestefan
 */

#include <iostream>

#include <brisk/HarrisScoreCalculator.hpp>
#include <brisk/sseFilters.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <sys/time.h>
#include <brisk/harrisScores.hpp>

namespace brisk{

void HarrisScoreCalculator::initializeScores(){
	/*cv::Mat DxDx1,DyDy1,DxDy1;
	cv::Mat DxDx,DyDy,DxDy;

	// pipeline
	TimerSwitchable timerFancyOp1("0.1.0 BRISK Detection: 2nd moment matrix images computation (per layer)");
	getCovarEntries(_img, DxDx1,DyDy1,DxDy1);
	timerFancyOp1.stop();
	TimerSwitchable timerFancyOp2("0.1.1 BRISK Detection: 3 times Gaussian filter of 2nd mom. mat. (per layer)");
	filterGauss3by316S(DxDx1, DxDx);
	filterGauss3by316S(DyDy1, DyDy);
	filterGauss3by316S(DxDy1, DxDy);
	timerFancyOp2.stop();
	TimerSwitchable timerFancyOp3("0.1.2 BRISK Detection: Harris corner score image (per layer)");
	cornerHarris(DxDx,DyDy,DxDy,_scores);
	timerFancyOp3.stop();*/

	TimerSwitchable timerFancyOp1("0.1 BRISK Detection: Harris score (per layer)");
	harrisScores_sse_full(_img,_scores);
	timerFancyOp1.stop();
}

void HarrisScoreCalculator::get2dMaxima(std::vector<PointWithScore>& points, int absoluteThreshold){
	// do the 8-neighbor nonmax suppression
	//struct timeval start, end;
	//gettimeofday(&start, NULL);
	const int stride=_scores.cols;
	const int rows_end=_scores.rows-2;
	points.reserve(4000);
	for( int j = 2; j < rows_end ; ++j ){
		const int* p=&_scores.at<int>(j,2);
		const int* const p_begin=p;
		const int* const p_end=&_scores.at<int>(j,stride-2);
		bool last=false;
		while(p<p_end){
			const int center=*p;
			const int* const center_p=p;
			++p;
			if(last) {last=false; continue;}
			//if(lastline.at<uchar>(0,i)) continue;
			if(center<absoluteThreshold) continue;
			if(*(center_p+1)>center) continue;
			if(*(center_p-1)>center) continue;
			const int* const p1=(center_p+stride);
			if(*p1>center) continue;
			const int* const p2=(center_p-stride);
			if(*p2>center) continue;
			if(*(p1+1)>center) continue;
			if(*(p1-1)>center) continue;
			if(*(p2+1)>center) continue;
			if(*(p2-1)>center) continue;
			const int i=p-p_begin-1;

#ifdef USE_SIMPLE_POINT_WITH_SCORE
			points.push_back(PointWithScore(center,i,j));
#else
#error
			points.push_back(PointWithScore(cv::Point2i(i,j),center));
#endif
		}
	}
	//gettimeofday(&end, NULL);
	//std::cout<<double(end.tv_sec-start.tv_sec)*1000.0+double(end.tv_usec-start.tv_usec)/1000.0<<std::endl;
}

// X and Y denote the size of the mask
void HarrisScoreCalculator::getCovarEntries(
		const cv::Mat& src, cv::Mat& dxdx, cv::Mat& dydy, cv::Mat& dxdy){
//inline void getCovarEntriesLestefan(cv::Mat& src, cv::Mat& dx, cv::Mat& dy, cv::Mat& kernel){
	// sanity check

	assert(src.type()==CV_8U);
	cv::Mat kernel=cv::Mat::zeros(3,3,CV_16S);
	kernel.at<short>(0,0)=3*8;
	kernel.at<short>(1,0)=10*8;
	kernel.at<short>(2,0)=3*8;
	kernel.at<short>(0,2)=-3*8;
	kernel.at<short>(1,2)=-10*8;
	kernel.at<short>(2,2)=-3*8;

	const unsigned int X=3;
	const unsigned int Y=3;
	const unsigned int cx=1;
	const unsigned int cy=1;

	// dest will be 16 bit
	dxdx=cv::Mat::zeros(src.rows,src.cols,CV_16S);
	dydy=cv::Mat::zeros(src.rows,src.cols,CV_16S);
	dxdy=cv::Mat::zeros(src.rows,src.cols,CV_16S);

	//dx=cv::Mat::zeros(src.rows,src.cols,CV_16S);
	//dy=cv::Mat::zeros(src.rows,src.cols,CV_16S);

	const unsigned int maxJ=((src.cols-2)/16)*16;
	const unsigned int maxI=src.rows-2;
	const unsigned int stride=src.cols;

	__m128i mask_hi = _mm_set_epi8(0x00,0xFF,0x00,0xFF,0x00,0xFF,0x00,0xFF,0x00,0xFF,0x00,0xFF,0x00,0xFF,0x00,0xFF);
	__m128i mask_lo = _mm_set_epi8(0xFF,0x00,0xFF,0x00,0xFF,0x00,0xFF,0x00,0xFF,0x00,0xFF,0x00,0xFF,0x00,0xFF,0x00);

	for(unsigned int i=0; i<maxI; ++i){
		bool end=false;
		for(unsigned int j=0; j<maxJ; ){
			//__m128i result = _mm_set_epi16 ( -127,-127,-127,-127,-127,-127,-127,-127,-127,-127);
			__m128i zeros;
			zeros=_mm_xor_si128(zeros,zeros);
			__m128i result_hi_dx = zeros;
			__m128i result_lo_dx = zeros;
			__m128i result_hi_dy = zeros;
			__m128i result_lo_dy = zeros;
			// enter convolution with kernel
			for(unsigned int x=0;x<X;++x){
				//if(dx&&x==1)continue; // jump, 0 kernel
				for(unsigned int y=0;y<Y;++y){
					//if(!dx&&y==1)continue; // jump, 0 kernel
					const short m_dx=kernel.at<short>(y,x);
					const short m_dy=kernel.at<short>(x,y);
					__m128i mult_dx = _mm_set_epi16(m_dx,m_dx,m_dx,m_dx,m_dx,m_dx,m_dx,m_dx);
					__m128i mult_dy = _mm_set_epi16(m_dy,m_dy,m_dy,m_dy,m_dy,m_dy,m_dy,m_dy);
					uchar* p=(src.data+(stride*(i+y))+x+j);
					__m128i i0 = _mm_loadu_si128 ((__m128i*)p);
					__m128i i0_hi=_mm_and_si128(i0,mask_hi);
					__m128i i0_lo=_mm_srli_si128(_mm_and_si128(i0,mask_lo),1);

					if(m_dx!=0){
						__m128i i_hi_dx = _mm_mullo_epi16 (i0_hi, mult_dx);
						__m128i i_lo_dx = _mm_mullo_epi16 (i0_lo, mult_dx);
						result_hi_dx=_mm_add_epi16(result_hi_dx,i_hi_dx);
						result_lo_dx=_mm_add_epi16(result_lo_dx,i_lo_dx);
					}

					if(m_dy!=0){
						__m128i i_hi_dy = _mm_mullo_epi16 (i0_hi, mult_dy);
						__m128i i_lo_dy = _mm_mullo_epi16 (i0_lo, mult_dy);
						result_hi_dy=_mm_add_epi16(result_hi_dy,i_hi_dy);
						result_lo_dy=_mm_add_epi16(result_lo_dy,i_lo_dy);
					}
				}
			}

			// calculate covariance entries - remove precision (ends up being 4 bit), then remove 4 more bits
			__m128i i_hi_dx_dx = _mm_srai_epi16(_mm_mulhi_epi16 (result_hi_dx, result_hi_dx),4);
			__m128i i_hi_dy_dy = _mm_srai_epi16(_mm_mulhi_epi16 (result_hi_dy, result_hi_dy),4);
			__m128i i_hi_dx_dy = _mm_srai_epi16(_mm_mulhi_epi16 (result_hi_dy, result_hi_dx),4);
			__m128i i_lo_dx_dx = _mm_srai_epi16(_mm_mulhi_epi16 (result_lo_dx, result_lo_dx),4);
			__m128i i_lo_dy_dy = _mm_srai_epi16(_mm_mulhi_epi16 (result_lo_dy, result_lo_dy),4);
			__m128i i_lo_dx_dy = _mm_srai_epi16(_mm_mulhi_epi16 (result_lo_dy, result_lo_dx),4);

			// store
			uchar* p_lo_dxdx=(dxdx.data+(2*stride*(i+cy)))+2*cx+2*j;
			uchar* p_hi_dxdx=(dxdx.data+(2*stride*(i+cy)))+2*cx+2*j+16;
			_mm_storeu_si128 ((__m128i*)p_hi_dxdx,_mm_unpackhi_epi16 (i_hi_dx_dx, i_lo_dx_dx));
			_mm_storeu_si128 ((__m128i*)p_lo_dxdx,_mm_unpacklo_epi16 (i_hi_dx_dx, i_lo_dx_dx));
			uchar* p_lo_dydy=(dydy.data+(2*stride*(i+cy)))+2*cx+2*j;
			uchar* p_hi_dydy=(dydy.data+(2*stride*(i+cy)))+2*cx+2*j+16;
			_mm_storeu_si128 ((__m128i*)p_hi_dydy,_mm_unpackhi_epi16 (i_hi_dy_dy, i_lo_dy_dy));
			_mm_storeu_si128 ((__m128i*)p_lo_dydy,_mm_unpacklo_epi16 (i_hi_dy_dy, i_lo_dy_dy));
			uchar* p_lo_dxdy=(dxdy.data+(2*stride*(i+cy)))+2*cx+2*j;
			uchar* p_hi_dxdy=(dxdy.data+(2*stride*(i+cy)))+2*cx+2*j+16;
			_mm_storeu_si128 ((__m128i*)p_hi_dxdy,_mm_unpackhi_epi16 (i_hi_dx_dy, i_lo_dx_dy));
			_mm_storeu_si128 ((__m128i*)p_lo_dxdy,_mm_unpacklo_epi16 (i_hi_dx_dy, i_lo_dx_dy));

			// take care about end
			j+=16;
			if(j>=maxJ&&!end){
				j=stride-2-16;
				end=true;
			}
		}
	}

	//cv::imshow("dxdx",dxdx);
	//cv::imshow("dydy",dydy);
	//cv::imshow("dxdy",dxdy);
	//cv::waitKey();
}

void HarrisScoreCalculator::cornerHarris(
		const cv::Mat& dxdxSmooth, const cv::Mat& dydySmooth, const cv::Mat& dxdySmooth, cv::Mat& dst){

	// dest will be 16 bit
	dst=cv::Mat::zeros(dxdxSmooth.rows,dxdxSmooth.cols,CV_32S);
	const unsigned int maxJ=((dxdxSmooth.cols-2)/8)*8;
	const unsigned int maxI=dxdxSmooth.rows-2;
	const unsigned int stride=dxdxSmooth.cols;

	for(unsigned int i=0; i<maxI; ++i){
		bool end=false;
		for(unsigned int j=0; j<maxJ; ){
			__m128i dxdx = _mm_loadu_si128 ((__m128i*)&dxdxSmooth.at<short>(i,j));
			__m128i dydy = _mm_loadu_si128 ((__m128i*)&dydySmooth.at<short>(i,j));
			__m128i dxdy = _mm_loadu_si128 ((__m128i*)&dxdySmooth.at<short>(i,j));

			// determinant terms
			__m128i prod1_lo=_mm_mullo_epi16(dxdx,dydy);
			__m128i prod1_hi=_mm_mulhi_epi16(dxdx,dydy);
			__m128i prod2_lo=_mm_mullo_epi16(dxdy,dxdy);
			__m128i prod2_hi=_mm_mulhi_epi16(dxdy,dxdy);
			__m128i prod1_1=_mm_unpacklo_epi16(prod1_lo,prod1_hi);
			__m128i prod1_2=_mm_unpackhi_epi16(prod1_lo,prod1_hi);
			__m128i prod2_1=_mm_unpacklo_epi16(prod2_lo,prod2_hi);
			__m128i prod2_2=_mm_unpackhi_epi16(prod2_lo,prod2_hi);

			// calculate the determinant
			__m128i det_1=_mm_sub_epi32(prod1_1,prod2_1);
			__m128i det_2=_mm_sub_epi32(prod1_2,prod2_2);

			// trace - uses kappa=1/16
			__m128i trace_quarter=_mm_srai_epi16(_mm_add_epi16(_mm_srai_epi16(dxdx,1),_mm_srai_epi16(dydy,1)),1);
			__m128i trace_sq_00625_lo=_mm_mullo_epi16(trace_quarter,trace_quarter);
			__m128i trace_sq_00625_hi=_mm_mulhi_epi16(trace_quarter,trace_quarter);
			__m128i trace_sq_00625_1=_mm_unpacklo_epi16(trace_sq_00625_lo,trace_sq_00625_hi);
			__m128i trace_sq_00625_2=_mm_unpackhi_epi16(trace_sq_00625_lo,trace_sq_00625_hi);

			// form score
			__m128i score_1=_mm_sub_epi32(det_1,trace_sq_00625_1);
			__m128i score_2=_mm_sub_epi32(det_2,trace_sq_00625_2);

			// store
			_mm_storeu_si128 ((__m128i*)&dst.at<int>(i,j),score_1);
			_mm_storeu_si128 ((__m128i*)&dst.at<int>(i,j+4),score_2);

			// take care about end
			j+=8;
			if(j>=maxJ&&!end){
				j=stride-2-8;
				end=true;
			}
		}
	}
}

}
