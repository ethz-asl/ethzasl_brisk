/*
    BRISK - Binary Robust Invariant Scalable Keypoints
    Reference implementation of
    [1] Stefan Leutenegger,Margarita Chli and Roland Siegwart, BRISK:
    	Binary Robust Invariant Scalable Keypoints, in Proceedings of
    	the IEEE International Conference on Computer Vision (ICCV2011).

    Copyright (C) 2011  The Autonomous Systems Lab, ETH Zurich,
    Stefan Leutenegger, Simon Lynen and Margarita Chli.

    This file is part of BRISK.

    BRISK is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    BRISK is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with BRISK.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <opencv2/features2d/features2d.hpp>
#include <opencv2/core/core.hpp>
#include <brisk/brisk.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <fstream>
#include <agast/oast9_16.h>
#include <agast/agast7_12s.h>
#include <agast/agast5_8.h>
#include <stdlib.h>
#include <tmmintrin.h>
#include <sys/time.h>

using namespace cv;

const float BriskDescriptorExtractor::basicSize_    =12.0;
const unsigned int BriskDescriptorExtractor::scales_=64;
const float BriskDescriptorExtractor::scalerange_   =30;        // 40->4 Octaves - else, this needs to be adjusted...
const unsigned int BriskDescriptorExtractor::n_rot_ =1024;	 // discretization of the rotation look-up

const float BriskScaleSpace::basicSize_             =12.0;

const int BriskScaleSpace::maxThreshold_=1;
const int BriskScaleSpace::dropThreshold_=5;
const int BriskScaleSpace::minDrop_=15;
const uchar BriskScaleSpace::defaultLowerThreshold=10; //28
const uchar BriskScaleSpace::defaultUpperThreshold=230;

// constructors
BriskDescriptorExtractor::BriskDescriptorExtractor(bool rotationInvariant,
		bool scaleInvariant, float patternScale){

	std::vector<float> rList;
	std::vector<int> nList;

	// this is the standard pattern found to be suitable also
	rList.resize(5);
	nList.resize(5);
	const double f=0.85*patternScale;

	rList[0]=f*0;
	rList[1]=f*2.9;
	rList[2]=f*4.9;
	rList[3]=f*7.4;
	rList[4]=f*10.8;

	nList[0]=1;
	nList[1]=10;
	nList[2]=14;
	nList[3]=15;
	nList[4]=20;

	rotationInvariance=rotationInvariant;
	scaleInvariance=scaleInvariant;
	generateKernel(rList,nList,5.85*patternScale,8.2*patternScale);

}
BriskDescriptorExtractor::BriskDescriptorExtractor(std::vector<float> &radiusList,
		std::vector<int> &numberList, bool rotationInvariant, bool scaleInvariant,
		float dMax, float dMin, std::vector<int> indexChange){
	rotationInvariance=rotationInvariant;
	scaleInvariance=scaleInvariant;
	generateKernel(radiusList,numberList,dMax,dMin,indexChange);
}

BriskDescriptorExtractor::BriskDescriptorExtractor(const std::string& fname, bool rotationInvariant,
		bool scaleInvariant){

	rotationInvariance=rotationInvariant;
	scaleInvariance=scaleInvariant;

	// not in use
	dMax_=0;
	dMin_=0;

	std::ifstream myfile(fname.c_str());
	assert(myfile.is_open());

	// read number of points
	myfile>>points_;
	//std::cout<<"points_: "<<points_<<std::endl;

	// set up the patterns
	patternPoints_=new BriskPatternPoint[points_*scales_*n_rot_];
	BriskPatternPoint* patternIterator=patternPoints_;

	// define the scale discretization:
	static const float lb_scale=log(scalerange_)/log(2.0);
	static const float lb_scale_step = lb_scale/(scales_);

	scaleList_=new float[scales_];
	sizeList_=new unsigned int[scales_];

	const float sigma_scale=1.3;

	// first fill the unscaled and unrotated pattern:
	float* u_x=new float[points_];
	float* u_y=new float[points_];
	float* sigma=new float[points_];
	for(unsigned int i=0; i<points_; i++){
		myfile>>u_x[i];
		myfile>>u_y[i];
		myfile>>sigma[i];
		//std::cout << u_x[i] <<" "<< u_y[i] <<" "<< sigma[i] << std::endl;
	}

	// now fill all the scaled and rotated versions
	for(unsigned int scale = 0; scale <scales_; ++scale){
		scaleList_[scale]=pow((double)2.0,(double)(scale*lb_scale_step));
		sizeList_[scale]=0;

		// generate the pattern points look-up
		double theta;
		for(size_t rot=0; rot<n_rot_; ++rot){
			for(unsigned int i=0; i<points_; i++){
				theta = double(rot)*2*M_PI/double(n_rot_); // this is the rotation of the feature

				patternIterator->x=scaleList_[scale]*(u_x[i]*cos(theta)-u_y[i]*sin(theta)); // feature rotation plus angle of the point
				patternIterator->y=scaleList_[scale]*(u_x[i]*sin(theta)+u_y[i]*cos(theta));
				// and the Gaussian kernel sigma
				patternIterator->sigma = sigma_scale*scaleList_[scale]*sigma[i];

				//if(scale==0&&rot==0)
					//std::cout<<int(patternIterator-patternPoints_)<<" "<<patternIterator->x<<" "<<patternIterator->y<<std::endl;
					//std::cout<<patternIterator->sigma<<std::endl;
					//std::cout<<sqrt(patternIterator->x*patternIterator->x+patternIterator->y*patternIterator->y)<<std::endl;


				// adapt the sizeList if necessary
				const unsigned int size=ceil(((
						sqrt(patternIterator->x*patternIterator->x+patternIterator->y*patternIterator->y))
						+patternIterator->sigma))+1;
				if(sizeList_[scale]<size){
					sizeList_[scale]=size;
				}

				// increment the iterator
				++patternIterator;
			}
			//if (rot==0)
				//std::cout<<"scale:"<<scale<<" "<<sizeList_[scale]<<std::endl;
		}
	}

	// now also generate pairings
	myfile>>noShortPairs_;
	shortPairs_ = new BriskShortPair[noShortPairs_];
	for(unsigned int p= 0; p<noShortPairs_; p++){
		unsigned int i,j;
		myfile>>i;
		shortPairs_[p].i=i;
		myfile>>j;
		shortPairs_[p].j=j;
	}

	myfile>>noLongPairs_;
	longPairs_ = new BriskLongPair[noLongPairs_];
	for(unsigned int p= 0; p<noLongPairs_; p++){
		unsigned int i,j;
		myfile>>i;
		longPairs_[p].i=i;
		myfile>>j;
		longPairs_[p].j=j;
		float dx=(u_x[j]-u_x[i]);//((sigma[i]+sigma[j])/2.f);
		float dy=(u_y[j]-u_y[i]);//((sigma[i]+sigma[j])/2.f);
		float norm_sq=dx*dx+dy*dy;
		longPairs_[p].weighted_dx=int((dx/(norm_sq))*2048.0+0.5);
		longPairs_[p].weighted_dy=int((dy/(norm_sq))*2048.0+0.5);
	}

	// no bits:
	strings_=(int)ceil((float(noShortPairs_))/128.0)*4*4;
	//std::cout<<"noLongPairs_: "<<noLongPairs_<<std::endl;
	//std::cout<<"noShortPairs_: "<<noShortPairs_<<std::endl;
	//std::cout<<"strings_: "<<strings_<<std::endl;
	//std::cout<<"sizeList_[0]: "<<sizeList_[0]<<std::endl;

	// clean up
	myfile.close();
}

void BriskDescriptorExtractor::generateKernel(std::vector<float> &radiusList,
		std::vector<int> &numberList, float dMax, float dMin,
		std::vector<int> indexChange){

	dMax_=dMax;
	dMin_=dMin;

	// get the total number of points
	const int rings=radiusList.size();
	assert(radiusList.size()!=0&&radiusList.size()==numberList.size());
	points_=0; // remember the total number of points
	for(int ring = 0; ring<rings; ring++){
		points_+=numberList[ring];
	}
	// set up the patterns
	patternPoints_=new BriskPatternPoint[points_*scales_*n_rot_];
	BriskPatternPoint* patternIterator=patternPoints_;

	// define the scale discretization:
	static const float lb_scale=log(scalerange_)/log(2.0);
	static const float lb_scale_step = lb_scale/(scales_);

	scaleList_=new float[scales_];
	sizeList_=new unsigned int[scales_];

	const float sigma_scale=1.3;

	for(unsigned int scale = 0; scale <scales_; ++scale){
		scaleList_[scale]=pow((double)2.0,(double)(scale*lb_scale_step));
		sizeList_[scale]=0;

		// generate the pattern points look-up
		double alpha, theta;
		for(size_t rot=0; rot<n_rot_; ++rot){
			theta = double(rot)*2*M_PI/double(n_rot_); // this is the rotation of the feature
			for(int ring = 0; ring<rings; ++ring){
				for(int num=0; num<numberList[ring]; ++num){
					// the actual coordinates on the circle
					alpha = (double(num))*2*M_PI/double(numberList[ring]);
					patternIterator->x=scaleList_[scale]*radiusList[ring]*cos(alpha+theta); // feature rotation plus angle of the point
					patternIterator->y=scaleList_[scale]*radiusList[ring]*sin(alpha+theta);
					// and the gaussian kernel sigma
					if(ring==0){
						patternIterator->sigma = sigma_scale*scaleList_[scale]*0.5;
					}
					else{
						patternIterator->sigma = sigma_scale*scaleList_[scale]*(double(radiusList[ring]))*sin(M_PI/numberList[ring]);
					}
					//if(scale==0&&rot==0)
						//std::cout<<patternIterator->sigma<<std::endl;
						//std::cout<<sqrt(patternIterator->x*patternIterator->x+patternIterator->y*patternIterator->y)<<std::endl;

					// adapt the sizeList if necessary
					const unsigned int size=ceil(((scaleList_[scale]*radiusList[ring])+patternIterator->sigma))+1;
					if(sizeList_[scale]<size){
						sizeList_[scale]=size;
					}

					// increment the iterator
					++patternIterator;
				}
			}
			//if (rot==0)
				//std::cout<<"scale:"<<scale<<" "<<sizeList_[scale]<<std::endl;
		}
	}

	// now also generate pairings
	shortPairs_ = new BriskShortPair[points_*(points_-1)/2];
	longPairs_ = new BriskLongPair[points_*(points_-1)/2];
	noShortPairs_=0;
	noLongPairs_=0;

	// fill indexChange with 0..n if empty
	unsigned int indSize=indexChange.size();
	if(indSize==0) {
		indexChange.resize(points_*(points_-1)/2);
		indSize=indexChange.size();
		for(unsigned int i=0; i<indSize; i++){
			indexChange[i]=i;
		}
	}
	const float dMin_sq =dMin_*dMin_;
	const float dMax_sq =dMax_*dMax_;
	for(unsigned int i= 1; i<points_; i++){
		for(unsigned int j= 0; j<i; j++){ //(find all the pairs)
			// point pair distance:
			const float dx=patternPoints_[j].x-patternPoints_[i].x;
			const float dy=patternPoints_[j].y-patternPoints_[i].y;
			const float norm_sq=(dx*dx+dy*dy);
			//std::cout<<i+1<<","<<j+1<<" - "<<patternPoints_[i].x<<","<<patternPoints_[i].y<<" - "<<patternPoints_[j].x<<","<<patternPoints_[j].y<<" - "<<dMax_sq<<" "<<norm_sq<<std::endl;
			if(norm_sq>dMin_sq){
				// save to long pairs
				BriskLongPair& longPair=longPairs_[noLongPairs_];
				longPair.weighted_dx=int((dx/(norm_sq))*2048.0+0.5);
				longPair.weighted_dy=int((dy/(norm_sq))*2048.0+0.5);
				longPair.i = i;
				longPair.j = j;
				++noLongPairs_;
			}
			/*else*/ if (norm_sq<dMax_sq){
				// save to short pairs
				assert(noShortPairs_<indSize); // make sure the user passes something sensible
				BriskShortPair& shortPair = shortPairs_[indexChange[noShortPairs_]];
				shortPair.j = j;
				shortPair.i = i;
				++noShortPairs_;
			}
		}
	}

	// no bits:
	strings_=(int)ceil((float(noShortPairs_))/128.0)*4*4;
	//std::cout<<"noLongPairs_: "<<noLongPairs_<<std::endl;
	//std::cout<<"noShortPairs_: "<<noShortPairs_<<std::endl;
	//std::cout<<"strings_: "<<strings_<<std::endl;
	//std::cout<<"sizeList_[0]: "<<sizeList_[0]<<std::endl;
}

// simple alternative:
template<typename ImgPixel_T, typename IntegralPixel_T>
__inline__ IntegralPixel_T BriskDescriptorExtractor::smoothedIntensity(const cv::Mat& image,
		const cv::Mat& integral,const float key_x,
		const float key_y, const unsigned int scale,
		const unsigned int rot, const unsigned int point) const{

	// get the float position
	const BriskPatternPoint& briskPoint = patternPoints_[scale*n_rot_*points_ + rot*points_ + point];

	const float xf=briskPoint.x+key_x;
	const float yf=briskPoint.y+key_y;
	const int x = int(xf);
	const int y = int(yf);
	const int& imagecols=image.cols;

	// get the sigma:
	const float sigma_half=briskPoint.sigma;
	const float area=4.0*sigma_half*sigma_half;

	// calculate output:
	int ret_val;
	if(sigma_half<0.5){
		//interpolation multipliers:
		const int r_x=(xf-x)*1024;
		const int r_y=(yf-y)*1024;
		const int r_x_1=(1024-r_x);
		const int r_y_1=(1024-r_y);
		ImgPixel_T* ptr=(ImgPixel_T*)image.data+x+y*imagecols;
		// just interpolate:
		ret_val=(r_x_1*r_y_1*IntegralPixel_T(*ptr));
		ptr++;
		ret_val+=(r_x*r_y_1*IntegralPixel_T(*ptr));
		ptr+=imagecols;
		ret_val+=(r_x*r_y*IntegralPixel_T(*ptr));
		ptr--;
		ret_val+=(r_x_1*r_y*IntegralPixel_T(*ptr));
		return (ret_val)/1024;
	}

	// this is the standard case (simple, not speed optimized yet):

	// scaling:
	const IntegralPixel_T scaling = 4194304.0/area;
	const IntegralPixel_T scaling2=float(scaling)*area/1024.0;

	// the integral image is larger:
	const int integralcols=imagecols+1;

	// calculate borders
	const float x_1=xf-sigma_half;
	const float x1=xf+sigma_half;
	const float y_1=yf-sigma_half;
	const float y1=yf+sigma_half;

	const int x_left=int(x_1+0.5);
	const int y_top=int(y_1+0.5);
	const int x_right=int(x1+0.5);
	const int y_bottom=int(y1+0.5);

	// overlap area - multiplication factors:
	const float r_x_1=float(x_left)-x_1+0.5;
	const float r_y_1=float(y_top)-y_1+0.5;
	const float r_x1=x1-float(x_right)+0.5;
	const float r_y1=y1-float(y_bottom)+0.5;
	const int dx=x_right-x_left-1;
	const int dy=y_bottom-y_top-1;
	const IntegralPixel_T A=(r_x_1*r_y_1)*scaling;
	const IntegralPixel_T B=(r_x1*r_y_1)*scaling;
	const IntegralPixel_T C=(r_x1*r_y1)*scaling;
	const IntegralPixel_T D=(r_x_1*r_y1)*scaling;
	const IntegralPixel_T r_x_1_i=r_x_1*scaling;
	const IntegralPixel_T r_y_1_i=r_y_1*scaling;
	const IntegralPixel_T r_x1_i=r_x1*scaling;
	const IntegralPixel_T r_y1_i=r_y1*scaling;

	if(dx+dy>2){
		// now the calculation:
		ImgPixel_T* ptr=(ImgPixel_T*)image.data+x_left+imagecols*y_top;
		// first the corners:
		ret_val=A*IntegralPixel_T(*ptr);
		ptr+=dx+1;
		ret_val+=B*IntegralPixel_T(*ptr);
		ptr+=dy*imagecols+1;
		ret_val+=C*IntegralPixel_T(*ptr);
		ptr-=dx+1;
		ret_val+=D*IntegralPixel_T(*ptr);

		// next the edges:
		IntegralPixel_T* ptr_integral=(IntegralPixel_T*)integral.data+x_left+integralcols*y_top+1;
		// find a simple path through the different surface corners
		const IntegralPixel_T tmp1=(*ptr_integral);
		ptr_integral+=dx;
		const IntegralPixel_T tmp2=(*ptr_integral);
		ptr_integral+=integralcols;
		const IntegralPixel_T tmp3=(*ptr_integral);
		ptr_integral++;
		const IntegralPixel_T tmp4=(*ptr_integral);
		ptr_integral+=dy*integralcols;
		const IntegralPixel_T tmp5=(*ptr_integral);
		ptr_integral--;
		const IntegralPixel_T tmp6=(*ptr_integral);
		ptr_integral+=integralcols;
		const IntegralPixel_T tmp7=(*ptr_integral);
		ptr_integral-=dx;
		const IntegralPixel_T tmp8=(*ptr_integral);
		ptr_integral-=integralcols;
		const IntegralPixel_T tmp9=(*ptr_integral);
		ptr_integral--;
		const IntegralPixel_T tmp10=(*ptr_integral);
		ptr_integral-=dy*integralcols;
		const IntegralPixel_T tmp11=(*ptr_integral);
		ptr_integral++;
		const IntegralPixel_T tmp12=(*ptr_integral);

		// assign the weighted surface integrals:
		const IntegralPixel_T upper=(tmp3-tmp2+tmp1-tmp12)*r_y_1_i;
		const IntegralPixel_T middle=(tmp6-tmp3+tmp12-tmp9)*scaling;
		const IntegralPixel_T left=(tmp9-tmp12+tmp11-tmp10)*r_x_1_i;
		const IntegralPixel_T right=(tmp5-tmp4+tmp3-tmp6)*r_x1_i;
		const IntegralPixel_T bottom=(tmp7-tmp6+tmp9-tmp8)*r_y1_i;

		return IntegralPixel_T((ret_val+upper+middle+left+right+bottom)/scaling2);
	}

	// now the calculation:
	ImgPixel_T* ptr=(ImgPixel_T*)image.data+x_left+imagecols*y_top;
	// first row:
	ret_val=A*IntegralPixel_T(*ptr);
	ptr++;
	const ImgPixel_T* end1 = ptr+dx;
	for(; ptr<end1; ptr++){
		ret_val+=r_y_1_i*IntegralPixel_T(*ptr);
	}
	ret_val+=B*IntegralPixel_T(*ptr);
	// middle ones:
	ptr+=imagecols-dx-1;
	const ImgPixel_T* end_j=ptr+dy*imagecols;
	for(; ptr<end_j; ptr+=imagecols-dx-1){
		ret_val+=r_x_1_i*IntegralPixel_T(*ptr);
		ptr++;
		const ImgPixel_T* end2 = ptr+dx;
		for(; ptr<end2; ptr++){
			ret_val+=IntegralPixel_T(*ptr)*scaling;
		}
		ret_val+=r_x1_i*IntegralPixel_T(*ptr);
	}
	// last row:
	ret_val+=D*IntegralPixel_T(*ptr);
	ptr++;
	const ImgPixel_T* end3 = ptr+dx;
	for(; ptr<end3; ptr++){
		ret_val+=r_y1_i*IntegralPixel_T(*ptr);
	}
	ret_val+=C*IntegralPixel_T(*ptr);

	return IntegralPixel_T((ret_val)/scaling2);
}

bool RoiPredicate(const float minX, const float minY,
		const float maxX, const float maxY, const KeyPoint& keyPt){
	const Point2f& pt = keyPt.pt;
	return (pt.x < minX) || (pt.x >= maxX) || (pt.y < minY) || (pt.y >= maxY);
}

// computes the descriptor
void BriskDescriptorExtractor::computeImpl(const Mat& image,
		std::vector<KeyPoint>& keypoints, Mat& descriptors) const{

	//Remove keypoints very close to the border
	size_t ksize=keypoints.size();
	std::vector<int> kscales; // remember the scale per keypoint
	kscales.resize(ksize);
	static const float log2 = 0.693147180559945;
	static const float lb_scalerange = log(scalerange_)/(log2);
	//std::vector<cv::KeyPoint>::iterator beginning = keypoints.begin();

	//slynen{
	std::vector<KeyPoint> valid_kp;
	std::vector<int> valid_scales;
	valid_kp.reserve(keypoints.size());
	valid_scales.reserve(keypoints.size());
	//}

	//std::vector<int>::iterator beginningkscales = kscales.begin();
	static const float basicSize06=basicSize_*0.6;
	unsigned int basicscale=0;
	if(!scaleInvariance)
		basicscale=std::max((int)(scales_/lb_scalerange*(log(1.45*basicSize_/(basicSize06))/log2)+0.5),0);
	for(size_t k=0; k<ksize; k++){
		unsigned int scale;
		if(scaleInvariance){
			scale=std::max((int)(scales_/lb_scalerange*(log(keypoints[k].size/(basicSize06))/log2)+0.5),0);
			// saturate
			if(scale>=scales_) scale = scales_-1;
			kscales[k]=scale;
		}
		else{
			scale = basicscale;
			kscales[k]=scale;
		}
		const int border = sizeList_[scale];
		const int border_x=image.cols-border;
		const int border_y=image.rows-border;
		//slynen{ //from the timing it makes actually no difference
		if(!RoiPredicate(border, border,border_x,border_y,keypoints[k])){
			valid_kp.push_back(keypoints[k]);
			valid_scales.push_back(kscales[k]);
		}
		//		if(RoiPredicate(border, border,border_x,border_y,keypoints[k])){
		//			keypoints.erase(beginning+k);
		//			kscales.erase(beginningkscales+k);
		//			if(k==0){
		//				beginning=keypoints.begin();
		//				beginningkscales = kscales.begin();
		//			}
		//			ksize--;
		//			k--;
		//		}
	}

	keypoints.swap(valid_kp);
	kscales.swap(valid_scales);
	ksize = keypoints.size();
	//}

	// first, calculate the integral image over the whole image:
	// current integral image
	cv::Mat _integral; // the integral image
	cv::Mat imageScaled;
	if(image.type()==CV_16UC1){
		// 16 bit image - convert to float. this is simple but not the fastest...
		cv::Mat imageCvt;
		image.convertTo(imageCvt,CV_32FC1);
		imageScaled=imageCvt/65536.0;
		cv::integral(imageScaled, _integral, CV_32F);
		//cv::imshow("Points",_integral);
		//std::cout<<_integral.at<float>(_integral.rows-1,_integral.cols-1)<<std::endl;
	}
	else if (image.type()==CV_8UC1){
		cv::integral(image, _integral);
	}
	else{
		std::cout<<"unsupported image format"<<std::endl;
		std::cout.flush();
		exit(-1);
	}

	int* _values=new int[points_]; // for temporary use

	// resize the descriptors:
	descriptors=cv::Mat::zeros(ksize,strings_, CV_8U);

	// now do the extraction for all keypoints:

	// temporary variables containing gray values at sample points:
	int t1;
	int t2;

	// the feature orientation
	int direction0;
	int direction1;

	uchar* ptr = descriptors.data;
	for(size_t k=0; k<ksize; k++){
		int theta;
		cv::KeyPoint& kp=keypoints[k];
		const int& scale=kscales[k];
		int shifter=0;
		int* pvalues =_values;
		const float& x=kp.pt.x;
		const float& y=kp.pt.y;
		if(kp.angle==-1){
			if (!rotationInvariance){
				// don't compute the gradient direction, just assign a rotation of 0°
				theta=0;
			}
			else{
				// get the gray values in the unrotated pattern
				if(image.type()==CV_8UC1){
					for(unsigned int i = 0; i<points_; i++){
						*(pvalues++)=smoothedIntensity<uchar,int>(image, _integral, x,
								y, scale, 0, i);
					}
				}
				else{
					for(unsigned int i = 0; i<points_; i++){
						*(pvalues++)=int(65536.0*smoothedIntensity<float,float>(imageScaled, _integral, x,
								y, scale, 0, i));
						/*std::cout<<(65536.0*smoothedIntensity<float,float>(imageScaled, _integral, x,
								y, scale, 0, i))<<std::endl;*/
					}
				}

				direction0=0;
				direction1=0;
				// now iterate through the long pairings
				const BriskLongPair* max=longPairs_+noLongPairs_;
				for(BriskLongPair* iter=longPairs_; iter<max; ++iter){
					t1=*(_values+iter->i);
					t2=*(_values+iter->j);
					const int delta_t=(t1-t2);
					// update the direction:
					const int tmp0=delta_t*(iter->weighted_dx)/1024;
					const int tmp1=delta_t*(iter->weighted_dy)/1024;
					direction0+=tmp0;
					direction1+=tmp1;
				}
				kp.angle=atan2((float)direction1,(float)direction0)/M_PI*180.0;
				theta=int((n_rot_*kp.angle)/(360.0)+0.5);
				if(theta<0)
					theta+=n_rot_;
				if(theta>=int(n_rot_))
					theta-=n_rot_;
			}
		}
		else{
			// figure out the direction:
			//int theta=rotationInvariance*round((_n_rot*atan2(direction.at<int>(0,0),direction.at<int>(1,0)))/(2*M_PI));
			if(!rotationInvariance){
				theta=0;
			}
			else{
				theta=(int)(n_rot_*(kp.angle/(360.0))+0.5);
				if(theta<0)
					theta+=n_rot_;
				if(theta>=int(n_rot_))
					theta-=n_rot_;
			}
		}

		// now also extract the stuff for the actual direction:
		// let us compute the smoothed values
		shifter=0;

		//unsigned int mean=0;
		pvalues =_values;
		// get the gray values in the rotated pattern
		if(image.type()==CV_8UC1){
			for(unsigned int i = 0; i<points_; i++){
				*(pvalues++)=smoothedIntensity<uchar,int>(image, _integral, x,
						y, scale, theta, i);
			}
		}
		else{
			for(unsigned int i = 0; i<points_; i++){
				*(pvalues++)=int(65536.0*smoothedIntensity<float,float>(imageScaled, _integral, x,
						y, scale, theta, i));
			}
		}

		// now iterate through all the pairings
		UINT32_ALIAS* ptr2=(UINT32_ALIAS*)ptr;
		const BriskShortPair* max=shortPairs_+noShortPairs_;
		for(BriskShortPair* iter=shortPairs_; iter<max;++iter){
			t1=*(_values+iter->i);
			t2=*(_values+iter->j);
			//std::cout<< int(t1>t2)<< " ";cout.flush();
			if(t1>t2){
				*ptr2|=((1)<<shifter);
			} // else already initialized with zero
			// take care of the iterators:
			++shifter;
			if(shifter==32){
				shifter=0;
				++ptr2;
				//while(1);
			}
		}


		ptr+=strings_;
	}

	// clean-up
	_integral.release();
	delete [] _values;
}

int BriskDescriptorExtractor::descriptorSize() const{
	return strings_;
}

int BriskDescriptorExtractor::descriptorType() const{
	return CV_8U;
}

BriskDescriptorExtractor::~BriskDescriptorExtractor(){
	delete [] patternPoints_;
	delete [] shortPairs_;
	delete [] longPairs_;
	delete [] scaleList_;
	delete [] sizeList_;
}

BriskFeatureDetector::BriskFeatureDetector(int thresh, int octaves, bool suppressScaleNonmaxima){
	threshold=thresh;
	this->octaves=octaves;
	m_suppressScaleNonmaxima=suppressScaleNonmaxima;
}

void BriskFeatureDetector::detectImpl( const cv::Mat& image,
		std::vector<cv::KeyPoint>& keypoints,
		const cv::Mat& mask) const
{
	BriskScaleSpace briskScaleSpace(octaves,m_suppressScaleNonmaxima);
	briskScaleSpace.constructPyramid(image,threshold);
	briskScaleSpace.getKeypoints(keypoints);

	// remove invalid points
	removeInvalidPoints(mask, keypoints);
}


// construct telling the octaves number:
BriskScaleSpace::BriskScaleSpace(uint8_t _octaves, bool suppressScaleNonmaxima){
	m_suppressScaleNonmaxima=suppressScaleNonmaxima;
	if(_octaves==0)
		layers_=1;
	else
		layers_=2*_octaves;
}
BriskScaleSpace::~BriskScaleSpace(){

}
// construct the image pyramids
void BriskScaleSpace::constructPyramid(const cv::Mat& image, uchar threshold){

	// set correct size:
	pyramid_.clear();

	// assign threshold
	threshold_=threshold;

	// fill the pyramid:
	pyramid_.push_back(BriskLayer(image.clone(),defaultUpperThreshold,defaultLowerThreshold));
	if(layers_>1){
		pyramid_.push_back(BriskLayer(pyramid_.back(),BriskLayer::CommonParams::TWOTHIRDSAMPLE,
				//float(defaultUpperThreshold)/1.2,float(defaultLowerThreshold)/1.2));
				(defaultUpperThreshold),(defaultLowerThreshold)));
	}
	const int octaves2=layers_;

	for(uint8_t i=2; i<octaves2; i+=2){
		pyramid_.push_back(BriskLayer(pyramid_[i-2],BriskLayer::CommonParams::HALFSAMPLE,
				//float(defaultUpperThreshold)/sqrt(float(i)),float(defaultLowerThreshold)/sqrt(float(i))));
				(defaultUpperThreshold),(defaultLowerThreshold)));
		pyramid_.push_back(BriskLayer(pyramid_[i-1],BriskLayer::CommonParams::HALFSAMPLE,
				(defaultUpperThreshold),(defaultLowerThreshold)));
				//float(defaultUpperThreshold)/1.2/sqrt(float(i)),
				//float(defaultLowerThreshold)/1.2/sqrt(float(i))));
	}
}

void BriskScaleSpace::getKeypoints(std::vector<cv::KeyPoint>& keypoints){
	// make sure keypoints is empty
	keypoints.resize(0);
	keypoints.reserve(2000);

	std::vector<std::vector<CvPoint> > agastPoints;
	agastPoints.resize(layers_);

	// go through the octaves and intra layers and calculate fast corner scores:
	for(uint8_t i = 0; i<layers_; i++){
		// call OAST16_9 without nms
		BriskLayer& l=pyramid_[i];
		l.getAgastPoints(threshold_,agastPoints[i]);
	}

	if(!m_suppressScaleNonmaxima){
		for(uint8_t i = 0; i<layers_; i++){
			// just do a simple 2d subpixel refinement...
			const int num=agastPoints[i].size();
			for(int n=0; n < num; n++){
				const CvPoint& point=agastPoints.at(0)[n];
				// first check if it is a maximum:
				if (!isMax2D(i, point.x, point.y))
					continue;

				// let's do the subpixel and float scale refinement:
				cv::BriskLayer& l=pyramid_[i];
				register int s_0_0 = l.getAgastScore(point.x-1, point.y-1, 1);
				register int s_1_0 = l.getAgastScore(point.x,   point.y-1, 1);
				register int s_2_0 = l.getAgastScore(point.x+1, point.y-1, 1);
				register int s_2_1 = l.getAgastScore(point.x+1, point.y,   1);
				register int s_1_1 = l.getAgastScore(point.x,   point.y,   1);
				register int s_0_1 = l.getAgastScore(point.x-1, point.y,   1);
				register int s_0_2 = l.getAgastScore(point.x-1, point.y+1, 1);
				register int s_1_2 = l.getAgastScore(point.x,   point.y+1, 1);
				register int s_2_2 = l.getAgastScore(point.x+1, point.y+1, 1);
				float delta_x, delta_y;
				float max = subpixel2D(s_0_0, s_0_1, s_0_2,
						s_1_0, s_1_1, s_1_2,
						s_2_0, s_2_1, s_2_2,
						delta_x, delta_y);

				// store:
				keypoints.push_back(cv::KeyPoint(float(point.x)+delta_x, float(point.y)+delta_y, basicSize_*l.scale(), -1, max,0));
				//std::cout<<"asdf"<<std::endl;
			}
		}
		return;
	}

	if(layers_==1){
		// just do a simple 2d subpixel refinement...
		const int num=agastPoints[0].size();
		for(int n=0; n < num; n++){
			const CvPoint& point=agastPoints.at(0)[n];
			// first check if it is a maximum:
			if (!isMax2D(0, point.x, point.y))
				continue;

			// let's do the subpixel and float scale refinement:
			cv::BriskLayer& l=pyramid_[0];
			register int s_0_0 = l.getAgastScore(point.x-1, point.y-1, 1);
			register int s_1_0 = l.getAgastScore(point.x,   point.y-1, 1);
			register int s_2_0 = l.getAgastScore(point.x+1, point.y-1, 1);
			register int s_2_1 = l.getAgastScore(point.x+1, point.y,   1);
			register int s_1_1 = l.getAgastScore(point.x,   point.y,   1);
			register int s_0_1 = l.getAgastScore(point.x-1, point.y,   1);
			register int s_0_2 = l.getAgastScore(point.x-1, point.y+1, 1);
			register int s_1_2 = l.getAgastScore(point.x,   point.y+1, 1);
			register int s_2_2 = l.getAgastScore(point.x+1, point.y+1, 1);
			float delta_x, delta_y;
			float max = subpixel2D(s_0_0, s_0_1, s_0_2,
					s_1_0, s_1_1, s_1_2,
					s_2_0, s_2_1, s_2_2,
					delta_x, delta_y);

			// store:
			keypoints.push_back(cv::KeyPoint(float(point.x)+delta_x, float(point.y)+delta_y, basicSize_, -1, max,0));

		}
		return;
	}

	float x,y,scale,score;
	for(uint8_t i = 0; i<layers_; i++){
		cv::BriskLayer& l=pyramid_[i];
		const int num=agastPoints[i].size();
		if(i==layers_-1){
			for(int n=0; n < num; n++){
				const CvPoint& point=agastPoints.at(i)[n];
				// consider only 2D maxima...
				if (!isMax2D(i, point.x, point.y))
					continue;

				bool ismax;
				float dx, dy;
				getScoreMaxBelow(i, point.x, point.y,
						l.getAgastScore(point.x,   point.y, 1), ismax,
						dx, dy);
				if(!ismax)
					continue;

				// get the patch on this layer:
				register int s_0_0 = l.getAgastScore(point.x-1, point.y-1, 1);
				register int s_1_0 = l.getAgastScore(point.x,   point.y-1, 1);
				register int s_2_0 = l.getAgastScore(point.x+1, point.y-1, 1);
				register int s_2_1 = l.getAgastScore(point.x+1, point.y,   1);
				register int s_1_1 = l.getAgastScore(point.x,   point.y,   1);
				register int s_0_1 = l.getAgastScore(point.x-1, point.y,   1);
				register int s_0_2 = l.getAgastScore(point.x-1, point.y+1, 1);
				register int s_1_2 = l.getAgastScore(point.x,   point.y+1, 1);
				register int s_2_2 = l.getAgastScore(point.x+1, point.y+1, 1);
				float delta_x, delta_y;
				float max = subpixel2D(s_0_0, s_0_1, s_0_2,
						s_1_0, s_1_1, s_1_2,
						s_2_0, s_2_1, s_2_2,
						delta_x, delta_y);

				// store:
				keypoints.push_back(cv::KeyPoint((float(point.x)+delta_x)*l.scale()+l.offset(),
						(float(point.y)+delta_y)*l.scale()+l.offset(), basicSize_*l.scale(), -1, max,i));
			}
		}
		else{
			// not the last layer:
			for(int n=0; n < num; n++){
				const CvPoint& point=agastPoints.at(i)[n];

				// first check if it is a maximum:
				if (!isMax2D(i, point.x, point.y))
					continue;

				// let's do the subpixel and float scale refinement:
				bool ismax;
				score=refine3D(i,point.x, point.y,x,y,scale,ismax);
				if(!ismax){
					continue;
				}

				// finally store the detected keypoint:
				//if(score>float(threshold_)){ // DEBUG
				keypoints.push_back(cv::KeyPoint(x, y, basicSize_*scale, -1, score,i));
				//}
			}
		}
	}
}

// interpolated score access with recalculation when needed:
__inline__ int BriskScaleSpace::getScoreAbove(const uint8_t layer,
		const int x_layer, const int y_layer){
	assert(layer<layers_-1);
	cv::BriskLayer& l=pyramid_[layer+1];
	if(layer%2==0){ // octave
		const int sixths_x=4*x_layer-1;
		const int x_above=sixths_x/6;
		const int sixths_y=4*y_layer-1;
		const int y_above=sixths_y/6;
		const int r_x=(sixths_x%6);
		const int r_x_1=6-r_x;
		const int r_y=(sixths_y%6);
		const int r_y_1=6-r_y;
		uint8_t score = 0xFF&((r_x_1*r_y_1*l.getAgastScore(x_above,y_above,1) +
				r_x*r_y_1*l.getAgastScore(x_above+1,y_above,1) +
				r_x_1*r_y*l.getAgastScore(x_above,y_above+1,1) +
				r_x*r_y*l.getAgastScore(x_above+1,y_above+1,1)+18)/36);

		return score;
	}
	else{ // intra
		const int eighths_x=6*x_layer-1;
		const int x_above=eighths_x/8;
		const int eighths_y=6*y_layer-1;
		const int y_above=eighths_y/8;
		const int r_x=(eighths_x%8);
		const int r_x_1=8-r_x;
		const int r_y=(eighths_y%8);
		const int r_y_1=8-r_y;
		uint8_t score = 0xFF&((r_x_1*r_y_1*l.getAgastScore(x_above,y_above,1) +
				r_x*r_y_1*l.getAgastScore(x_above+1,y_above,1) +
				r_x_1*r_y*l.getAgastScore(x_above,y_above+1,1) +
				r_x*r_y*l.getAgastScore(x_above+1,y_above+1,1)+32)/64);
		return score;
	}
}
__inline__ int BriskScaleSpace::getScoreBelow(const uint8_t layer,
		const int x_layer, const int y_layer){
	assert(layer);
	cv::BriskLayer& l=pyramid_[layer-1];
	int sixth_x;
	int quarter_x;
	float xf;
	int sixth_y;
	int quarter_y;
	float yf;

	// scaling:
	float offs;
	float area;
	int scaling;
	int scaling2;

	if(layer%2==0){ // octave
		sixth_x=8*x_layer+1;
		xf=float(sixth_x)/6.0;
		sixth_y=8*y_layer+1;
		yf=float(sixth_y)/6.0;

		// scaling:
		offs = 3.0/4.0;
		area=4.0*offs*offs;
		scaling = 4194304.0/area;
		scaling2=float(scaling)*area;
	}
	else{
		quarter_x=6*x_layer+1;
		xf=float(quarter_x)/4.0;
		quarter_y=6*y_layer+1;
		yf=float(quarter_y)/4.0;

		// scaling:
		offs = 2.0/3.0;
		area=4.0*offs*offs;
		scaling = 4194304.0/area;
		scaling2=float(scaling)*area;
	}

	// calculate borders
	const float x_1=xf-offs;
	const float x1=xf+offs;
	const float y_1=yf-offs;
	const float y1=yf+offs;

	const int x_left=int(x_1+0.5);
	const int y_top=int(y_1+0.5);
	const int x_right=int(x1+0.5);
	const int y_bottom=int(y1+0.5);

	// overlap area - multiplication factors:
	const float r_x_1=float(x_left)-x_1+0.5;
	const float r_y_1=float(y_top)-y_1+0.5;
	const float r_x1=x1-float(x_right)+0.5;
	const float r_y1=y1-float(y_bottom)+0.5;
	const int dx=x_right-x_left-1;
	const int dy=y_bottom-y_top-1;
	const int A=(r_x_1*r_y_1)*scaling;
	const int B=(r_x1*r_y_1)*scaling;
	const int C=(r_x1*r_y1)*scaling;
	const int D=(r_x_1*r_y1)*scaling;
	const int r_x_1_i=r_x_1*scaling;
	const int r_y_1_i=r_y_1*scaling;
	const int r_x1_i=r_x1*scaling;
	const int r_y1_i=r_y1*scaling;

	// first row:
	int ret_val=A*int(l.getAgastScore(x_left,y_top,1));
	for(int X=1; X<=dx; X++){
		ret_val+=r_y_1_i*int(l.getAgastScore(x_left+X,y_top,1));
	}
	ret_val+=B*int(l.getAgastScore(x_left+dx+1,y_top,1));
	// middle ones:
	for(int Y=1; Y<=dy; Y++){
		ret_val+=r_x_1_i*int(l.getAgastScore(x_left,y_top+Y,1));


		for(int X=1; X<=dx; X++){
			ret_val+=int(l.getAgastScore(x_left+X,y_top+Y,1))*scaling;
		}
		ret_val+=r_x1_i*int(l.getAgastScore(x_left+dx+1,y_top+Y,1));
	}
	// last row:
	ret_val+=D*int(l.getAgastScore(x_left,y_top+dy+1,1));
	for(int X=1; X<=dx; X++){
		ret_val+=r_y1_i*int(l.getAgastScore(x_left+X,y_top+dy+1,1));
	}
	ret_val+=C*int(l.getAgastScore(x_left+dx+1,y_top+dy+1,1));

	return ((ret_val+scaling2/2)/scaling2);
}

__inline__ bool BriskScaleSpace::isMax2D(const uint8_t layer,
		const int x_layer, const int y_layer){
	const cv::Mat& scores = pyramid_[layer].scores();
	cv::BriskLayer& l=pyramid_[layer];
	const int scorescols = scores.cols;
	uchar* data=scores.data + y_layer*scorescols + x_layer;
	// decision tree:
	const uchar center = (*data);
	/*data--;
	const uchar s_10=*data;
	if(center<s_10) return false;
	data+=2;
	const uchar s10=*data;
	if(center<s10) return false;
	data-=(scorescols+1);
	const uchar s0_1=*data;
	if(center<s0_1) return false;
	data+=2*scorescols;
	const uchar s01=*data;
	if(center<s01) return false;
	data--;
	const uchar s_11=*data;
	if(center<s_11) return false;
	data+=2;
	const uchar s11=*data;
	if(center<s11) return false;
	data-=2*scorescols;
	const uchar s1_1=*data;
	if(center<s1_1) return false;
	data-=2;
	const uchar s_1_1=*data;
	if(center<s_1_1) return false;*/

	const uchar s_10=l.getAgastScore(x_layer-1,y_layer,center);
	if(center<s_10) return false;
	const uchar s10=l.getAgastScore(x_layer+1,y_layer,center);
	if(center<s10) return false;
	const uchar s0_1=l.getAgastScore(x_layer,y_layer-1,center);
	if(center<s0_1) return false;
	const uchar s01=l.getAgastScore(x_layer,y_layer+1,center);
	if(center<s01) return false;
	const uchar s_11=l.getAgastScore(x_layer-1,y_layer+1,center);
	if(center<s_11) return false;
	const uchar s11=l.getAgastScore(x_layer+1,y_layer+1,center);
	if(center<s11) return false;
	const uchar s1_1=l.getAgastScore(x_layer+1,y_layer-1,center);;
	if(center<s1_1) return false;
	const uchar s_1_1=l.getAgastScore(x_layer-1,y_layer-1,center);;
	if(center<s_1_1) return false;


	// reject neighbor maxima
	std::vector<int> delta;
	// put together a list of 2d-offsets to where the maximum is also reached
	if(center==s_1_1) {
		delta.push_back(-1);
		delta.push_back(-1);
	}
	if(center==s0_1) {
		delta.push_back(0);
		delta.push_back(-1);
	}
	if(center==s1_1) {
		delta.push_back(1);
		delta.push_back(-1);
	}
	if(center==s_10) {
		delta.push_back(-1);
		delta.push_back(0);
	}
	if(center==s10) {
		delta.push_back(1);
		delta.push_back(0);
	}
	if(center==s_11) {
		delta.push_back(-1);
		delta.push_back(1);
	}
	if(center==s01) {
		delta.push_back(0);
		delta.push_back(1);
	}
	if(center==s11) {
		delta.push_back(1);
		delta.push_back(1);
	}
	const unsigned int deltasize=delta.size();
	if(deltasize!=0){
		// in this case, we have to analyze the situation more carefully:
		// the values are gaussian blurred and then we really decide
		data=scores.data + y_layer*scorescols + x_layer;
		int smoothedcenter=4*center+2*(s_10+s10+s0_1+s01)+s_1_1+s1_1+s_11+s11;
		for(unsigned int i=0; i<deltasize;i+=2){
			data=scores.data + (y_layer-1+delta[i+1])*scorescols + x_layer+delta[i]-1;
			int othercenter=*data;
			data++;
			othercenter+=2*(*data);
			data++;
			othercenter+=*data;
			data+=scorescols;
			othercenter+=2*(*data);
			data--;
			othercenter+=4*(*data);
			data--;
			othercenter+=2*(*data);
			data+=scorescols;
			othercenter+=*data;
			data++;
			othercenter+=2*(*data);
			data++;
			othercenter+=*data;
			if(othercenter>smoothedcenter) return false;
		}
	}
	return true;
}

// 3D maximum refinement centered around (x_layer,y_layer)
__inline__ float BriskScaleSpace::refine3D(const uint8_t layer,
		const int x_layer, const int y_layer,
		float& x, float& y, float& scale, bool& ismax){
	ismax=true;
	BriskLayer& thisLayer=pyramid_[layer];
	const int center = thisLayer.getAgastScore(x_layer,y_layer,1);

	// check and get above maximum:
	float delta_x_above, delta_y_above;
	float max_above = getScoreMaxAbove(layer,x_layer, y_layer,
			center, ismax,
			delta_x_above, delta_y_above);

	if(!ismax) return 0.0;

	float max; // to be returned
	bool doScaleRefinement=true;

	if(layer%2==0){ // on octave
		// treat the patch below:
		float delta_x_below, delta_y_below;
		float max_below_float;
		uchar max_below_uchar=0;
		if(layer==0){
			// guess the lower intra octave...
			BriskLayer& l=pyramid_[0];
			register int s_0_0 = l.getAgastScore_5_8(x_layer-1, y_layer-1, 1);
			max_below_uchar=s_0_0;
			register int s_1_0 = l.getAgastScore_5_8(x_layer,   y_layer-1, 1);
			if(s_1_0>max_below_uchar) max_below_uchar=s_1_0;
			register int s_2_0 = l.getAgastScore_5_8(x_layer+1, y_layer-1, 1);
			if(s_2_0>max_below_uchar) max_below_uchar=s_2_0;
			register int s_2_1 = l.getAgastScore_5_8(x_layer+1, y_layer,   1);
			if(s_2_1>max_below_uchar) max_below_uchar=s_2_1;
			register int s_1_1 = l.getAgastScore_5_8(x_layer,   y_layer,   1);
			if(s_1_1>max_below_uchar) max_below_uchar=s_1_1;
			register int s_0_1 = l.getAgastScore_5_8(x_layer-1, y_layer,   1);
			if(s_0_1>max_below_uchar) max_below_uchar=s_0_1;
			register int s_0_2 = l.getAgastScore_5_8(x_layer-1, y_layer+1, 1);
			if(s_0_2>max_below_uchar) max_below_uchar=s_0_2;
			register int s_1_2 = l.getAgastScore_5_8(x_layer,   y_layer+1, 1);
			if(s_1_2>max_below_uchar) max_below_uchar=s_1_2;
			register int s_2_2 = l.getAgastScore_5_8(x_layer+1, y_layer+1, 1);
			if(s_2_2>max_below_uchar) max_below_uchar=s_2_2;

			max_below_float = subpixel2D(s_0_0, s_0_1, s_0_2,
					s_1_0, s_1_1, s_1_2,
					s_2_0, s_2_1, s_2_2,
					delta_x_below, delta_y_below);
			max_below_float = max_below_uchar;
		}
		else{
			max_below_float = getScoreMaxBelow(layer,x_layer, y_layer,
					center, ismax,
					delta_x_below, delta_y_below);
			if(!ismax) return 0;
		}

		// get the patch on this layer:
		register int s_0_0 = thisLayer.getAgastScore(x_layer-1, y_layer-1,1);
		register int s_1_0 = thisLayer.getAgastScore(x_layer,   y_layer-1,1);
		register int s_2_0 = thisLayer.getAgastScore(x_layer+1, y_layer-1,1);
		register int s_2_1 = thisLayer.getAgastScore(x_layer+1, y_layer,1);
		register int s_1_1 = thisLayer.getAgastScore(x_layer,   y_layer,1);
		register int s_0_1 = thisLayer.getAgastScore(x_layer-1, y_layer,1);
		register int s_0_2 = thisLayer.getAgastScore(x_layer-1, y_layer+1,1);
		register int s_1_2 = thisLayer.getAgastScore(x_layer,   y_layer+1,1);
		register int s_2_2 = thisLayer.getAgastScore(x_layer+1, y_layer+1,1);

		// second derivative needs to be sufficiently large
		if(layer==0){
			if(s_1_1-maxThreshold_<=int(max_above)){
				doScaleRefinement=false;
			}
		}
		else{
			if((s_1_1-maxThreshold_<(max_above))||(s_1_1-maxThreshold_<(max_below_float))){
				if((s_1_1-minDrop_>(max_above))||(s_1_1-minDrop_>(max_below_float))){
					// this means, it's an edge on the scale axis.
					doScaleRefinement=false;
					//std::cout<<".";
				}
				else{
					// no clear max, no edge -> discard
					ismax=false;
					//std::cout<<":";
					return 0.0f;
				}
			}
		}

		float delta_x_layer, delta_y_layer;
		float max_layer = subpixel2D(s_0_0, s_0_1, s_0_2,
				s_1_0, s_1_1, s_1_2,
				s_2_0, s_2_1, s_2_2,
				delta_x_layer, delta_y_layer);



		// calculate the relative scale (1D maximum):
		if(doScaleRefinement){
			if(layer==0){
				scale=refine1D_2(max_below_float,
						std::max(float(center),max_layer),
						max_above,max);
			}
			else{
				scale=refine1D(max_below_float,
						std::max(float(center),max_layer),
						max_above,max);
			}
		}
		else{
			scale=1.0;
			max=max_layer;
		}

		if(scale>1.0){
			// interpolate the position:
			const float r0=(1.5-scale)/.5;
			const float r1=1.0-r0;
			x=(r0*delta_x_layer+r1*delta_x_above+float(x_layer))
							*thisLayer.scale()+thisLayer.offset();
			y=(r0*delta_y_layer+r1*delta_y_above+float(y_layer))
							*thisLayer.scale()+thisLayer.offset();
		}
		else{
			if(layer==0){
				// interpolate the position:
				const float r0=(scale-0.5)/0.5;
				const float r_1=1.0-r0;
				x=r0*delta_x_layer+r_1*delta_x_below+float(x_layer);
				y=r0*delta_y_layer+r_1*delta_y_below+float(y_layer);
			}
			else{
				// interpolate the position:
				const float r0=(scale-0.75)/0.25;
				const float r_1=1.0-r0;
				x=(r0*delta_x_layer+r_1*delta_x_below+float(x_layer))
								*thisLayer.scale()+thisLayer.offset();
				y=(r0*delta_y_layer+r_1*delta_y_below+float(y_layer))
								*thisLayer.scale()+thisLayer.offset();
			}
		}
	}
	else{
		// on intra
		// check the patch below:
		float delta_x_below, delta_y_below;
		float max_below = getScoreMaxBelow(layer,x_layer, y_layer,
				center, ismax,
				delta_x_below, delta_y_below);
		if(!ismax) return 0.0;

		// get the patch on this layer:
		register int s_0_0 = thisLayer.getAgastScore(x_layer-1, y_layer-1,1);
		register int s_1_0 = thisLayer.getAgastScore(x_layer,   y_layer-1,1);
		register int s_2_0 = thisLayer.getAgastScore(x_layer+1, y_layer-1,1);
		register int s_2_1 = thisLayer.getAgastScore(x_layer+1, y_layer,1);
		register int s_1_1 = thisLayer.getAgastScore(x_layer,   y_layer,1);
		register int s_0_1 = thisLayer.getAgastScore(x_layer-1, y_layer,1);
		register int s_0_2 = thisLayer.getAgastScore(x_layer-1, y_layer+1,1);
		register int s_1_2 = thisLayer.getAgastScore(x_layer,   y_layer+1,1);
		register int s_2_2 = thisLayer.getAgastScore(x_layer+1, y_layer+1,1);

		// second derivative needs to be sufficiently large
		if((s_1_1-maxThreshold_<(max_above))||(s_1_1-maxThreshold_<(max_below))){
			if((s_1_1-minDrop_>(max_above))||(s_1_1-minDrop_>(max_below))){
				// this means, it's an edge on the scale axis.
				doScaleRefinement=false;
				//std::cout<<".";
			}
			else{
				// no clear max, no edge -> discard
				ismax=false;
				//std::cout<<":";
				return 0.0f;
			}
		}



		float delta_x_layer, delta_y_layer;
		float max_layer = subpixel2D(s_0_0, s_0_1, s_0_2,
				s_1_0, s_1_1, s_1_2,
				s_2_0, s_2_1, s_2_2,
				delta_x_layer, delta_y_layer);

		if(doScaleRefinement){
			// calculate the relative scale (1D maximum):
			scale=refine1D_1(max_below,
					std::max(float(center),max_layer),
					max_above,max);
		}
		else{
			scale=1.0;
			max=max_layer;
		}

		if(scale>1.0){
			// interpolate the position:
			const float r0=4.0-scale*3.0;
			const float r1=1.0-r0;
			x=(r0*delta_x_layer+r1*delta_x_above+float(x_layer))
							*thisLayer.scale()+thisLayer.offset();
			y=(r0*delta_y_layer+r1*delta_y_above+float(y_layer))
							*thisLayer.scale()+thisLayer.offset();
		}
		else{
			// interpolate the position:
			const float r0=scale*3.0-2.0;
			const float r_1=1.0-r0;
			x=(r0*delta_x_layer+r_1*delta_x_below+float(x_layer))
							*thisLayer.scale()+thisLayer.offset();
			y=(r0*delta_y_layer+r_1*delta_y_below+float(y_layer))
							*thisLayer.scale()+thisLayer.offset();
		}
	}

	// calculate the absolute scale:
	scale*=thisLayer.scale();

	// that's it, return the refined maximum:
	return max;
}

// return the maximum of score patches above or below
__inline__ float BriskScaleSpace::getScoreMaxAbove(const uint8_t layer,
		const int x_layer, const int y_layer,
		const int thr, bool& ismax,
		float& dx, float& dy){

	int threshold=thr+dropThreshold_;

	ismax=false;
	// relevant floating point coordinates
	float x_1;
	float x1;
	float y_1;
	float y1;

	// the layer above
	assert(layer+1<layers_);
	BriskLayer& layerAbove=pyramid_[layer+1];

	if(layer%2==0) {
		// octave
		x_1=float(4*(x_layer)-1-2)/6.0;
		x1=float(4*(x_layer)-1+2)/6.0;
		y_1=float(4*(y_layer)-1-2)/6.0;
		y1=float(4*(y_layer)-1+2)/6.0;
	}
	else{
		// intra
		x_1=float(6*(x_layer)-1-3)/8.0f;
		x1=float(6*(x_layer)-1+3)/8.0f;
		y_1=float(6*(y_layer)-1-3)/8.0f;
		y1=float(6*(y_layer)-1+3)/8.0f;
	}


	// check the first row
	int max_x = x_1+1;
	int max_y = y_1+1;
	float tmp_max;
	float max=layerAbove.getAgastScore(x_1,y_1,1);
	if(max>threshold) return 0;
	for(int x=x_1+1; x<=int(x1); x++){
		tmp_max=layerAbove.getAgastScore(float(x),y_1,1);
		if(tmp_max>threshold) return 0;
		if(tmp_max>max){
			max=tmp_max;
			max_x = x;
		}
	}
	tmp_max=layerAbove.getAgastScore(x1,y_1,1);
	if(tmp_max>threshold) return 0;
	if(tmp_max>max){
		max=tmp_max;
		max_x = int(x1);
	}

	// middle rows
	for(int y=y_1+1; y<=int(y1); y++){
		tmp_max=layerAbove.getAgastScore(x_1,float(y),1);
		if(tmp_max>threshold) return 0;
		if(tmp_max>max){
			max=tmp_max;
			max_x = int(x_1+1);
			max_y = y;
		}
		for(int x=x_1+1; x<=int(x1); x++){
			tmp_max=layerAbove.getAgastScore(x,y,1);
			if(tmp_max>threshold) return 0;
			if(tmp_max>max){
				max=tmp_max;
				max_x = x;
				max_y = y;
			}
		}
		tmp_max=layerAbove.getAgastScore(x1,float(y),1);
		if(tmp_max>threshold) return 0;
		if(tmp_max>max){
			max=tmp_max;
			max_x = int(x1);
			max_y = y;
		}
	}

	// bottom row
	tmp_max=layerAbove.getAgastScore(x_1,y1,1);
	if(tmp_max>max){
		max=tmp_max;
		max_x = int(x_1+1);
		max_y = int(y1);
	}
	for(int x=x_1+1; x<=int(x1); x++){
		tmp_max=layerAbove.getAgastScore(float(x),y1,1);
		if(tmp_max>max){
			max=tmp_max;
			max_x = x;
			max_y = int(y1);
		}
	}
	tmp_max=layerAbove.getAgastScore(x1,y1,1);
	if(tmp_max>max){
		max=tmp_max;
		max_x = int(x1);
		max_y = int(y1);
	}

	//find dx/dy:
	register int s_0_0 = layerAbove.getAgastScore(max_x-1, max_y-1,1);
	register int s_1_0 = layerAbove.getAgastScore(max_x,   max_y-1,1);
	register int s_2_0 = layerAbove.getAgastScore(max_x+1, max_y-1,1);
	register int s_2_1 = layerAbove.getAgastScore(max_x+1, max_y,1);
	register int s_1_1 = layerAbove.getAgastScore(max_x,   max_y,1);
	register int s_0_1 = layerAbove.getAgastScore(max_x-1, max_y,1);
	register int s_0_2 = layerAbove.getAgastScore(max_x-1, max_y+1,1);
	register int s_1_2 = layerAbove.getAgastScore(max_x,   max_y+1,1);
	register int s_2_2 = layerAbove.getAgastScore(max_x+1, max_y+1,1);
	float dx_1, dy_1;
	float refined_max=subpixel2D(s_0_0, s_0_1,  s_0_2,
			s_1_0, s_1_1, s_1_2,
			s_2_0, s_2_1, s_2_2,
			dx_1, dy_1);

	// calculate dx/dy in above coordinates
	float real_x = float(max_x)+dx_1;
	float real_y = float(max_y)+dy_1;
	bool returnrefined=true;
	if(layer%2==0){
		dx=(real_x*6.0f+1.0f)/4.0f-float(x_layer);
		dy=(real_y*6.0f+1.0f)/4.0f-float(y_layer);
	}
	else{
		dx=(real_x*8.0+1.0)/6.0-float(x_layer);
		dy=(real_y*8.0+1.0)/6.0-float(y_layer);
	}

	// saturate
	if(dx>1.0f) {dx=1.0f;returnrefined=false;}
	if(dx<-1.0f) {dx=-1.0f;returnrefined=false;}
	if(dy>1.0f) {dy=1.0f;returnrefined=false;}
	if(dy<-1.0f) {dy=-1.0f;returnrefined=false;}

	// done and ok.
	ismax=true;
	if(returnrefined){
		return std::max(refined_max,max);
	}
	return max;
}

__inline__ float BriskScaleSpace::getScoreMaxBelow(const uint8_t layer,
		const int x_layer, const int y_layer,
		const int thr, bool& ismax,
		float& dx, float& dy){
	int threshold=thr+dropThreshold_;

	ismax=false;

	// relevant floating point coordinates
	float x_1;
	float x1;
	float y_1;
	float y1;

	if(layer%2==0){
		// octave
		x_1=float(8*(x_layer)+1-4)/6.0;
		x1=float(8*(x_layer)+1+4)/6.0;
		y_1=float(8*(y_layer)+1-4)/6.0;
		y1=float(8*(y_layer)+1+4)/6.0;
	}
	else{
		x_1=float(6*(x_layer)+1-3)/4.0;
		x1=float(6*(x_layer)+1+3)/4.0;
		y_1=float(6*(y_layer)+1-3)/4.0;
		y1=float(6*(y_layer)+1+3)/4.0;
	}

	// the layer below
	assert(layer>0);
	BriskLayer& layerBelow=pyramid_[layer-1];

	// check the first row
	int max_x = x_1+1;
	int max_y = y_1+1;
	float tmp_max;
	float max=layerBelow.getAgastScore(x_1,y_1,1);
	if(max>threshold) return 0;
	for(int x=x_1+1; x<=int(x1); x++){
		tmp_max=layerBelow.getAgastScore(float(x),y_1,1);
		if(tmp_max>threshold) return 0;
		if(tmp_max>max){
			max=tmp_max;
			max_x = x;
		}
	}
	tmp_max=layerBelow.getAgastScore(x1,y_1,1);
	if(tmp_max>threshold) return 0;
	if(tmp_max>max){
		max=tmp_max;
		max_x = int(x1);
	}

	// middle rows
	for(int y=y_1+1; y<=int(y1); y++){
		tmp_max=layerBelow.getAgastScore(x_1,float(y),1);
		if(tmp_max>threshold) return 0;
		if(tmp_max>max){
			max=tmp_max;
			max_x = int(x_1+1);
			max_y = y;
		}
		for(int x=x_1+1; x<=int(x1); x++){
			tmp_max=layerBelow.getAgastScore(x,y,1);
			if(tmp_max>threshold) return 0;
			if(tmp_max==max){
				const int t1=2*(
						layerBelow.getAgastScore(x-1,y,1)
						+layerBelow.getAgastScore(x+1,y,1)
						+layerBelow.getAgastScore(x,y+1,1)
						+layerBelow.getAgastScore(x,y-1,1))
						+(layerBelow.getAgastScore(x+1,y+1,1)
								+layerBelow.getAgastScore(x-1,y+1,1)
								+layerBelow.getAgastScore(x+1,y-1,1)
								+layerBelow.getAgastScore(x-1,y-1,1));
				const int t2=2*(
						layerBelow.getAgastScore(max_x-1,max_y,1)
						+layerBelow.getAgastScore(max_x+1,max_y,1)
						+layerBelow.getAgastScore(max_x,max_y+1,1)
						+layerBelow.getAgastScore(max_x,max_y-1,1))
						+(layerBelow.getAgastScore(max_x+1,max_y+1,1)
								+layerBelow.getAgastScore(max_x-1,max_y+1,1)
								+layerBelow.getAgastScore(max_x+1,max_y-1,1)
								+layerBelow.getAgastScore(max_x-1,max_y-1,1));
				if(t1>t2){
					max_x = x;
					max_y = y;
				}
			}
			if(tmp_max>max){
				max=tmp_max;
				max_x = x;
				max_y = y;
			}
		}
		tmp_max=layerBelow.getAgastScore(x1,float(y),1);
		if(tmp_max>threshold) return 0;
		if(tmp_max>max){
			max=tmp_max;
			max_x = int(x1);
			max_y = y;
		}
	}

	// bottom row
	tmp_max=layerBelow.getAgastScore(x_1,y1,1);
	if(tmp_max>max){
		max=tmp_max;
		max_x = int(x_1+1);
		max_y = int(y1);
	}
	for(int x=x_1+1; x<=int(x1); x++){
		tmp_max=layerBelow.getAgastScore(float(x),y1,1);
		if(tmp_max>max){
			max=tmp_max;
			max_x = x;
			max_y = int(y1);
		}
	}
	tmp_max=layerBelow.getAgastScore(x1,y1,1);
	if(tmp_max>max){
		max=tmp_max;
		max_x = int(x1);
		max_y = int(y1);
	}

	//find dx/dy:
	register int s_0_0 = layerBelow.getAgastScore(max_x-1, max_y-1,1);
	register int s_1_0 = layerBelow.getAgastScore(max_x,   max_y-1,1);
	register int s_2_0 = layerBelow.getAgastScore(max_x+1, max_y-1,1);
	register int s_2_1 = layerBelow.getAgastScore(max_x+1, max_y,1);
	register int s_1_1 = layerBelow.getAgastScore(max_x,   max_y,1);
	register int s_0_1 = layerBelow.getAgastScore(max_x-1, max_y,1);
	register int s_0_2 = layerBelow.getAgastScore(max_x-1, max_y+1,1);
	register int s_1_2 = layerBelow.getAgastScore(max_x,   max_y+1,1);
	register int s_2_2 = layerBelow.getAgastScore(max_x+1, max_y+1,1);
	float dx_1, dy_1;
	float refined_max=subpixel2D(s_0_0, s_0_1,  s_0_2,
			s_1_0, s_1_1, s_1_2,
			s_2_0, s_2_1, s_2_2,
			dx_1, dy_1);

	// calculate dx/dy in above coordinates
	float real_x = float(max_x)+dx_1;
	float real_y = float(max_y)+dy_1;
	bool returnrefined=true;
	if(layer%2==0){
		dx=(real_x*6.0+1.0)/8.0-float(x_layer);
		dy=(real_y*6.0+1.0)/8.0-float(y_layer);
	}
	else{
		dx=(real_x*4.0-1.0)/6.0-float(x_layer);
		dy=(real_y*4.0-1.0)/6.0-float(y_layer);
	}

	// saturate
	if(dx>1.0) {dx=1.0;returnrefined=false;}
	if(dx<-1.0) {dx=-1.0;returnrefined=false;}
	if(dy>1.0) {dy=1.0;returnrefined=false;}
	if(dy<-1.0) {dy=-1.0;returnrefined=false;}

	// done and ok.
	ismax=true;
	if(returnrefined){
		return std::max(refined_max,max);
	}
	return max;
}

__inline__ float BriskScaleSpace::refine1D(const float s_05,
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

__inline__ float BriskScaleSpace::refine1D_1(const float s_05,
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

__inline__ float BriskScaleSpace::refine1D_2(const float s_05,
		const float s0, const float s05, float& max){
	int i_05=int(1024.0*s_05+0.5);
	int i0=int(1024.0*s0+0.5);
	int i05=int(1024.0*s05+0.5);

	//   18.0000  -30.0000   12.0000
	//  -45.0000   65.0000  -20.0000
	//   27.0000  -30.0000    8.0000

	int a=2*i_05-4*i0+2*i05;
	// second derivative must be negative:
	if(a>=0){
		if(s0>=s_05 && s0>=s05){
			max=s0;
			return 1.0;
		}
		if(s_05>=s0 && s_05>=s05){
			max=s_05;
			return 0.7;
		}
		if(s05>=s0 && s05>=s_05){
			max=s05;
			return 1.5;
		}
	}

	int b=-5*i_05+8*i0-3*i05;
	// calculate max location:
	float ret_val=-float(b)/float(2*a);
	// saturate and return
	if(ret_val<0.7) ret_val= 0.7;
	else if(ret_val>1.5) ret_val= 1.5; // allow to be slightly off bounds ...?
	int c = +3*i_05  -3*i0    +1*i05;
	max=float(c)+float(a)*ret_val*ret_val+float(b)*ret_val;
	max/=1024;
	return ret_val;
}

__inline__ float BriskScaleSpace::subpixel2D(const int s_0_0, const int s_0_1, const int s_0_2,
		const int s_1_0, const int s_1_1, const int s_1_2,
		const int s_2_0, const int s_2_1, const int s_2_2,
		float& delta_x, float& delta_y){

	// the coefficients of the 2d quadratic function least-squares fit:
	register int tmp1 =        s_0_0 + s_0_2 - 2*s_1_1 + s_2_0 + s_2_2;
	register int coeff1 = 3*(tmp1 + s_0_1 - ((s_1_0 + s_1_2)<<1) + s_2_1);
	register int coeff2 = 3*(tmp1 - ((s_0_1+ s_2_1)<<1) + s_1_0 + s_1_2 );
	register int tmp2 =                                  s_0_2 - s_2_0;
	register int tmp3 =                         (s_0_0 + tmp2 - s_2_2);
	register int tmp4 =                                   tmp3 -2*tmp2;
	register int coeff3 =                    -3*(tmp3 + s_0_1 - s_2_1);
	register int coeff4 =                    -3*(tmp4 + s_1_0 - s_1_2);
	register int coeff5 =            (s_0_0 - s_0_2 - s_2_0 + s_2_2)<<2;
	register int coeff6 = -(s_0_0  + s_0_2 - ((s_1_0 + s_0_1 + s_1_2 + s_2_1)<<1) - 5*s_1_1  + s_2_0  + s_2_2)<<1;


	// 2nd derivative test:
	register int H_det=4*coeff1*coeff2 - coeff5*coeff5;

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

	// this is the case of the maximum inside the boundaries:
	return (coeff1*delta_x*delta_x+coeff2*delta_y*delta_y
			+coeff3*delta_x+coeff4*delta_y
			+coeff5*delta_x*delta_y
			+coeff6)/18.0;
}

// construct a layer
BriskLayer::BriskLayer(const cv::Mat& img, uchar upperThreshold, uchar lowerThreshold,
		float scale, float offset) {

	upperThreshold_=upperThreshold;
	lowerThreshold_=lowerThreshold;

	//std::cout<<int(threshold)<<std::endl;
	img_=img;
	scores_=cv::Mat::zeros(img.rows,img.cols,CV_8U);
	// attention: this means that the passed image reference must point to persistent memory
	scale_=scale;
	offset_=offset;
	// create an agast detector
	oastDetector_ = new agast::OastDetector9_16(img.cols, img.rows, 0);
	agastDetector_5_8_ = new agast::AgastDetector5_8(img.cols, img.rows, 0);

	// calculate threshold map
	//timeval t1,t2;
	//gettimeofday(&t1,NULL);
	//for(int i=0; i<1000; i++)
	calculateThresholdMap();
	//gettimeofday(&t2,NULL);
	//std::cout<<"thresholdMap: "<<double(t2.tv_sec-t1.tv_sec)+double(t2.tv_usec-t1.tv_usec)/1000000.0<<" ms"<<std::endl;

	// DEBUG: display
	//cv::imshow("test",thrmap_);
	//cv::waitKey();
}
// derive a layer
BriskLayer::BriskLayer(const BriskLayer& layer, int mode,
		uchar upperThreshold, uchar lowerThreshold){

	upperThreshold_=upperThreshold;
	lowerThreshold_=lowerThreshold;

	//std::cout<<int(threshold)<<std::endl;
	if(mode==CommonParams::HALFSAMPLE){
		img_.create(layer.img().rows/2, layer.img().cols/2,CV_8U);
		halfsample(layer.img(), img_);
		scale_= layer.scale()*2;
		offset_=0.5*scale_-0.5;
	}
	else {
		img_.create(2*(layer.img().rows/3), 2*(layer.img().cols/3),CV_8U);
		twothirdsample(layer.img(), img_);
		scale_= layer.scale()*1.5;
		offset_=0.5*scale_-0.5;
	}
	scores_=cv::Mat::zeros(img_.rows,img_.cols,CV_8U);
	oastDetector_ = new agast::OastDetector9_16(img_.cols, img_.rows, 0);
	agastDetector_5_8_ = new agast::AgastDetector5_8(img_.cols, img_.rows, 0);

	// calculate threshold map
	calculateThresholdMap();

	//cv::imshow("test",thrmap_);
	//cv::waitKey();
}

// Fast/Agast
// wraps the agast class
void BriskLayer::getAgastPoints(uint8_t threshold, std::vector<CvPoint>& keypoints){
	oastDetector_->set_threshold(threshold, upperThreshold_, lowerThreshold_);
	oastDetector_->detect(img_.data,keypoints,&thrmap_);

	// also write scores
	const int num=keypoints.size();
	const int imcols=img_.cols;

	for(int i=0; i<num; i++){
		const int offs=keypoints[i].x+keypoints[i].y*imcols;
		int thr=*(thrmap_.data+offs);
		oastDetector_->set_threshold(thr);
		*(scores_.data+offs)=oastDetector_->cornerScore(img_.data+offs);
	}
}
inline uint8_t BriskLayer::getAgastScore(int x, int y, uint8_t threshold){
	if(x<3||y<3) return 0;
	if(x>=img_.cols-3||y>=img_.rows-3) return 0;
	uint8_t& score=*(scores_.data+x+y*scores_.cols);
	if(score>2) { return score; }
	oastDetector_->set_threshold(threshold-1);
	score = oastDetector_->cornerScore(img_.data+x+y*img_.cols);
	if (score<threshold) score = 0;
	return score;
}

inline uint8_t BriskLayer::getAgastScore_5_8(int x, int y, uint8_t threshold){
	if(x<2||y<2) return 0;
	if(x>=img_.cols-2||y>=img_.rows-2) return 0;
	agastDetector_5_8_->set_threshold(threshold-1);
	uint8_t score = agastDetector_5_8_->cornerScore(img_.data+x+y*img_.cols);
	if (score<threshold) score = 0;
	return score;
}

inline uint8_t BriskLayer::getAgastScore(float xf, float yf, uint8_t threshold, float scale){
	if(scale<=1.0f){
		// just do an interpolation inside the layer
		const int x=int(xf);
		const float rx1=xf-float(x);
		const float rx=1.0f-rx1;
		const int y=int(yf);
		const float ry1=yf-float(y);
		const float ry=1.0f-ry1;

		return rx*ry*getAgastScore(x, y, threshold)+
				rx1*ry*getAgastScore(x+1, y, threshold)+
				rx*ry1*getAgastScore(x, y+1, threshold)+
				rx1*ry1*getAgastScore(x+1, y+1, threshold);
	}
	else{
		// this means we overlap area smoothing
		const float halfscale = scale/2.0f;
		// get the scores first:
		for(int x=int(xf-halfscale); x<=int(xf+halfscale+1.0f); x++){
			for(int y=int(yf-halfscale); y<=int(yf+halfscale+1.0f); y++){
				getAgastScore(x, y, threshold);
			}
		}
		// get the smoothed value
		return value(scores_,xf,yf,scale);
	}
}

// access gray values (smoothed/interpolated)
__inline__ uint8_t BriskLayer::value(const cv::Mat& mat, float xf, float yf, float scale){
	assert(!mat.empty());
	// get the position
	const int x = floor(xf);
	const int y = floor(yf);
	const cv::Mat& image=mat;
	const int& imagecols=image.cols;

	// get the sigma_half:
	const float sigma_half=scale/2;
	const float area=4.0*sigma_half*sigma_half;
	// calculate output:
	int ret_val;
	if(sigma_half<0.5){
		//interpolation multipliers:
		const int r_x=(xf-x)*1024;
		const int r_y=(yf-y)*1024;
		const int r_x_1=(1024-r_x);
		const int r_y_1=(1024-r_y);
		uchar* ptr=image.data+x+y*imagecols;
		// just interpolate:
		ret_val=(r_x_1*r_y_1*int(*ptr));
		ptr++;
		ret_val+=(r_x*r_y_1*int(*ptr));
		ptr+=imagecols;
		ret_val+=(r_x*r_y*int(*ptr));
		ptr--;
		ret_val+=(r_x_1*r_y*int(*ptr));
		return 0xFF&((ret_val+512)/1024/1024);
	}

	// this is the standard case (simple, not speed optimized yet):

	// scaling:
	const int scaling = 4194304.0/area;
	const int scaling2=float(scaling)*area/1024.0;

	// calculate borders
	const float x_1=xf-sigma_half;
	const float x1=xf+sigma_half;
	const float y_1=yf-sigma_half;
	const float y1=yf+sigma_half;

	const int x_left=int(x_1+0.5);
	const int y_top=int(y_1+0.5);
	const int x_right=int(x1+0.5);
	const int y_bottom=int(y1+0.5);

	// overlap area - multiplication factors:
	const float r_x_1=float(x_left)-x_1+0.5;
	const float r_y_1=float(y_top)-y_1+0.5;
	const float r_x1=x1-float(x_right)+0.5;
	const float r_y1=y1-float(y_bottom)+0.5;
	const int dx=x_right-x_left-1;
	const int dy=y_bottom-y_top-1;
	const int A=(r_x_1*r_y_1)*scaling;
	const int B=(r_x1*r_y_1)*scaling;
	const int C=(r_x1*r_y1)*scaling;
	const int D=(r_x_1*r_y1)*scaling;
	const int r_x_1_i=r_x_1*scaling;
	const int r_y_1_i=r_y_1*scaling;
	const int r_x1_i=r_x1*scaling;
	const int r_y1_i=r_y1*scaling;

	// now the calculation:
	uchar* ptr=image.data+x_left+imagecols*y_top;
	// first row:
	ret_val=A*int(*ptr);
	ptr++;
	const uchar* end1 = ptr+dx;
	for(; ptr<end1; ptr++){
		ret_val+=r_y_1_i*int(*ptr);
	}
	ret_val+=B*int(*ptr);
	// middle ones:
	ptr+=imagecols-dx-1;
	uchar* end_j=ptr+dy*imagecols;
	for(; ptr<end_j; ptr+=imagecols-dx-1){
		ret_val+=r_x_1_i*int(*ptr);
		ptr++;
		const uchar* end2 = ptr+dx;
		for(; ptr<end2; ptr++){
			ret_val+=int(*ptr)*scaling;
		}
		ret_val+=r_x1_i*int(*ptr);
	}
	// last row:
	ret_val+=D*int(*ptr);
	ptr++;
	const uchar* end3 = ptr+dx;
	for(; ptr<end3; ptr++){
		ret_val+=r_y1_i*int(*ptr);
	}
	ret_val+=C*int(*ptr);

	return 0xFF&((ret_val+scaling2/2)/scaling2/1024);
}

// threshold map
__inline__ void BriskLayer::calculateThresholdMap(){

	// allocate threshold map
	cv::Mat tmpmax=cv::Mat::zeros(img_.rows,img_.cols,CV_8U);
	cv::Mat tmpmin=cv::Mat::zeros(img_.rows,img_.cols,CV_8U);
	thrmap_=cv::Mat::zeros(img_.rows,img_.cols,CV_8U);

	const int rowstride=img_.cols;

	// SSE version //

	for(int y=1; y<img_.rows-1; y++){
		int x=1;
		while(x+16<img_.cols-1){

			// access
			uchar* p=img_.data+x-1+(y-1)*rowstride;
			__m128i v_1_1 = _mm_loadu_si128((__m128i*)p);
			p++;
			__m128i v0_1 = _mm_loadu_si128((__m128i*)p);
			p++;
			__m128i v1_1 = _mm_loadu_si128((__m128i*)p);
			p+=rowstride;
			__m128i v10 = _mm_loadu_si128((__m128i*)p);
			p--;
			__m128i v00 = _mm_loadu_si128((__m128i*)p);
			p--;
			__m128i v_10 = _mm_loadu_si128((__m128i*)p);
			p+=rowstride;
			__m128i v_11 = _mm_loadu_si128((__m128i*)p);
			p++;
			__m128i v01 = _mm_loadu_si128((__m128i*)p);
			p++;
			__m128i v11 = _mm_loadu_si128((__m128i*)p);

			// min/max calc
			__m128i max = _mm_max_epu8(v_1_1,v0_1);
			__m128i min = _mm_min_epu8(v_1_1,v0_1);
			max = _mm_max_epu8(max,v1_1);
			min = _mm_min_epu8(min,v1_1);
			max = _mm_max_epu8(max,v10);
			min = _mm_min_epu8(min,v10);
			max = _mm_max_epu8(max,v00);
			min = _mm_min_epu8(min,v00);
			max = _mm_max_epu8(max,v_10);
			min = _mm_min_epu8(min,v_10);
			max = _mm_max_epu8(max,v_11);
			min = _mm_min_epu8(min,v_11);
			max = _mm_max_epu8(max,v01);
			min = _mm_min_epu8(min,v01);
			max = _mm_max_epu8(max,v11);
			min = _mm_min_epu8(min,v11);

			// store
			_mm_storeu_si128 ((__m128i*)(tmpmax.data+x+y*rowstride), max);
			_mm_storeu_si128 ((__m128i*)(tmpmin.data+x+y*rowstride), min);

			// next block
			x+=16;
		}
	}

	for(int y=3; y<img_.rows-3; y++){
		int x=3;
		while(x+16<img_.cols-3){

			// access
			uchar* p=img_.data+x+y*rowstride;
			__m128i v00 = _mm_loadu_si128((__m128i*)p);
			p-=2+2*rowstride;
			__m128i v_2_2 = _mm_loadu_si128((__m128i*)p);
			p+=4;
			__m128i v2_2 = _mm_loadu_si128((__m128i*)p);
			p+=4*rowstride;
			__m128i v22 = _mm_loadu_si128((__m128i*)p);
			p-=4;
			__m128i v_22 = _mm_loadu_si128((__m128i*)p);

			p=tmpmax.data+x+(y-2)*rowstride;
			__m128i max0_2 = _mm_loadu_si128((__m128i*)p);
			p+=4*rowstride;
			__m128i max02 = _mm_loadu_si128((__m128i*)p);
			p-=2*rowstride+2;
			__m128i max_20 = _mm_loadu_si128((__m128i*)p);
			p+=4;
			__m128i max20 = _mm_loadu_si128((__m128i*)p);

			p=tmpmin.data+x+(y-2)*rowstride;
			__m128i min0_2 = _mm_loadu_si128((__m128i*)p);
			p+=4*rowstride;
			__m128i min02 = _mm_loadu_si128((__m128i*)p);
			p-=2*rowstride+2;
			__m128i min_20 = _mm_loadu_si128((__m128i*)p);
			p+=4;
			__m128i min20 = _mm_loadu_si128((__m128i*)p);

			// min/max
			__m128i max = _mm_max_epu8(v00,v_2_2);
			__m128i min = _mm_min_epu8(v00,v_2_2);
			max = _mm_max_epu8(max,v2_2);
			min = _mm_min_epu8(min,v2_2);
			max = _mm_max_epu8(max,v22);
			min = _mm_min_epu8(min,v22);
			max = _mm_max_epu8(max,v_22);
			min = _mm_min_epu8(min,v_22);
			max = _mm_max_epu8(max,max0_2);
			min = _mm_min_epu8(min,min0_2);
			max = _mm_max_epu8(max,max02);
			min = _mm_min_epu8(min,min02);
			max = _mm_max_epu8(max,max_20);
			min = _mm_min_epu8(min,min_20);
			max = _mm_max_epu8(max,max20);
			min = _mm_min_epu8(min,min20);

			// store
			__m128i diff=_mm_sub_epi8(max,min);
			_mm_storeu_si128 ((__m128i*)(thrmap_.data+x+y*rowstride), diff);

			// next block
			x+=16;
		}
	}

	for(int x=max(1,16*((img_.cols-2)/16)-16); x<img_.cols-1; x++){
		for(int y=1; y<img_.rows-1; y++){
			// access
			uchar* p=img_.data+x-1+(y-1)*rowstride;
			int v_1_1 = *p;
			p++;
			int v0_1 = *p;
			p++;
			int v1_1 = *p;
			p+=rowstride;
			int v10 = *p;
			p--;
			int v00 = *p;
			p--;
			int v_10 = *p;
			p+=rowstride;
			int v_11 = *p;
			p++;
			int v01 = *p;
			p++;
			int v11 = *p;

			// min/max calc
			int max = std::max(v_1_1,v0_1);
			int min = std::min(v_1_1,v0_1);
			max = std::max(max,v1_1);
			min = std::min(min,v1_1);
			max = std::max(max,v10);
			min = std::min(min,v10);
			max = std::max(max,v00);
			min = std::min(min,v00);
			max = std::max(max,v_10);
			min = std::min(min,v_10);
			max = std::max(max,v_11);
			min = std::min(min,v_11);
			max = std::max(max,v01);
			min = std::min(min,v01);
			max = std::max(max,v11);
			min = std::min(min,v11);

			// store
			*(tmpmax.data+x+y*rowstride)=max;
			*(tmpmin.data+x+y*rowstride)=min;

		}
	}

	for(int x=max(3,16*((img_.cols-6)/16)-16); x<img_.cols-3; x++){
		for(int y=3; y<img_.rows-3; y++){
			// access
			uchar* p=img_.data+x+y*rowstride;
			int v00 =*p;
			p-=2+2*rowstride;
			int v_2_2 = *p;
			p+=4;
			int v2_2 = *p;
			p+=4*rowstride;
			int v22 = *p;
			p-=4;
			int v_22 = *p;

			p=tmpmax.data+x+(y-2)*rowstride;
			int max0_2 = *p;
			p+=4*rowstride;
			int max02 = *p;
			p-=2*rowstride+2;
			int max_20 = *p;
			p+=4;
			int max20 = *p;

			p=tmpmin.data+x+(y-2)*rowstride;
			int min0_2 = *p;
			p+=4*rowstride;
			int min02 = *p;
			p-=2*rowstride+2;
			int min_20 = *p;
			p+=4;
			int min20 = *p;

			// min/max
			int max = std::max(v00,v_2_2);
			int min = std::min(v00,v_2_2);
			max = std::max(max,v2_2);
			min = std::min(min,v2_2);
			max = std::max(max,v22);
			min = std::min(min,v22);
			max = std::max(max,v_22);
			min = std::min(min,v_22);
			max = std::max(max,max0_2);
			min = std::min(min,min0_2);
			max = std::max(max,max02);
			min = std::min(min,min02);
			max = std::max(max,max_20);
			min = std::min(min,min_20);
			max = std::max(max,max20);
			min = std::min(min,min20);

			// store
			*(thrmap_.data+x+y*rowstride) = max-min;
		}
	}

	/*
	for(int x=max(3,16*((img_.cols-4)/16)-16); x<img_.cols-3; x++){
		for(int y=3; y<img_.rows-3; y++){
			uchar* p=img_.data+x-1+(y-3)*rowstride;
			uchar v;
			const uchar* p_end1=p+3;
			uchar max=*p;
			uchar min=max;
			p++;
			while(p<p_end1){
				v=*p;
				max = std::max(max,v);
				min = std::min(min,v);
				p++;
			}
			p+=rowstride;
			const uchar* p_end2=p-5;
			while(p>p_end2){
				v=*p;
				max = std::max(max,v);
				min = std::min(min,v);
				p--;
			}
			p+=rowstride;
			const uchar* p_end3=p+7;
			while(p<p_end3){
				v=*p;
				max = std::max(max,v);
				min = std::min(min,v);
				p++;
			}
			p+=rowstride-1;
			const uchar* p_end4=p-7;
			while(p>p_end4){
				v=*p;
				max = std::max(max,v);
				min = std::min(min,v);
				p--;
			}
			p+=rowstride+1;
			const uchar* p_end5=p+7;
			while(p<p_end5){
				v=*p;
				max = std::max(max,v);
				min = std::min(min,v);
				p++;
			}
			p+=rowstride-2;
			const uchar* p_end6=p-5;
			while(p>p_end6){
				v=*p;
				max = std::max(max,v);
				min = std::min(min,v);
				p--;
			}
			p+=rowstride+2;
			const uchar* p_end7=p+3;
			while(p<p_end7){
				v=*p;
				max = std::max(max,v);
				min = std::min(min,v);
				p++;
			}

			// assign the threshold
			uchar diff=(max-min);
	 *(thrmap_.data+x+rowstride*y)=diff;

		}
	}*/
}

/*
 * // threshold map
__inline__ void BriskLayer::calculateThresholdMap(){

	// allocate threshold map
	thrmap_=cv::Mat::zeros(img_.rows,img_.cols,CV_8U);

	const int rowstride=img_.cols;

	// SSE version //
	for(int y=3; y<img_.rows-3; y++){
		int x=3;
		while(x+16<img_.cols-3){
			uchar* p=img_.data+x-1+(y-3)*rowstride;
			__m128i v;
			const uchar* p_end1=p+3;
			__m128i max=_mm_loadu_si128((__m128i*)p);
			__m128i min=max;
			p++;
			while(p<p_end1){
				v=_mm_loadu_si128((__m128i*)p);
				max = _mm_max_epu8(max,v);
				min = _mm_min_epu8(min,v);
				p++;
			}
			p+=rowstride;
			const uchar* p_end2=p-5;
			while(p>p_end2){
				v=_mm_loadu_si128((__m128i*)p);
				max = _mm_max_epu8(max,v);
				min = _mm_min_epu8(min,v);
				p--;
			}
			p+=rowstride;
			const uchar* p_end3=p+7;
			while(p<p_end3){
				v=_mm_loadu_si128((__m128i*)p);
				max = _mm_max_epu8(max,v);
				min = _mm_min_epu8(min,v);
				p++;
			}
			p+=rowstride-1;
			const uchar* p_end4=p-7;
			while(p>p_end4){
				v=_mm_loadu_si128((__m128i*)p);
				max = _mm_max_epu8(max,v);
				min = _mm_min_epu8(min,v);
				p--;
			}
			p+=rowstride+1;
			const uchar* p_end5=p+7;
			while(p<p_end5){
				v=_mm_loadu_si128((__m128i*)p);
				max = _mm_max_epu8(max,v);
				min = _mm_min_epu8(min,v);
				p++;
			}
			p+=rowstride-2;
			const uchar* p_end6=p-5;
			while(p>p_end6){
				v=_mm_loadu_si128((__m128i*)p);
				max = _mm_max_epu8(max,v);
				min = _mm_min_epu8(min,v);
				p--;
			}
			p+=rowstride+2;
			const uchar* p_end7=p+3;
			while(p<p_end7){
				v=_mm_loadu_si128((__m128i*)p);
				max = _mm_max_epu8(max,v);
				min = _mm_min_epu8(min,v);
				p++;
			}

			// store
			__m128i diff=_mm_sub_epi8(max,min);
			_mm_storeu_si128 ((__m128i*)(thrmap_.data+x+y*rowstride), diff);*/
/*uchar* diff_p=(thrmap_.data+x+y*rowstride);
			for(int i=0; i<16; i++){
				int diffval=(*diff_p);
				if((int(lowerThreshold_)*int(threshold))/100>diffval){
					(*diff_p)=0;
					diff_p++;
					continue;
				}
				if (diffval>upperThreshold_){
					diffval=upperThreshold_;
				}
				else if (diffval<lowerThreshold_){
					diffval=lowerThreshold_;
				}
				uchar newval=(diffval*int(threshold))/100;
				(*diff_p)=newval;
				//std::cout<<int(newval)<<":"<<int(diffval)<<"$"<<int(threshold)<<" ";
				diff_p++;
			}*/
/*

			// next block
			x+=16;
		}
	}


	for(int x=max(3,16*((img_.cols-4)/16)-16); x<img_.cols-3; x++){
		for(int y=3; y<img_.rows-3; y++){
			uchar* p=img_.data+x-1+(y-3)*rowstride;
			uchar v;
			const uchar* p_end1=p+3;
			uchar max=*p;
			uchar min=max;
			p++;
			while(p<p_end1){
				v=*p;
				max = std::max(max,v);
				min = std::min(min,v);
				p++;
			}
			p+=rowstride;
			const uchar* p_end2=p-5;
			while(p>p_end2){
				v=*p;
				max = std::max(max,v);
				min = std::min(min,v);
				p--;
			}
			p+=rowstride;
			const uchar* p_end3=p+7;
			while(p<p_end3){
				v=*p;
				max = std::max(max,v);
				min = std::min(min,v);
				p++;
			}
			p+=rowstride-1;
			const uchar* p_end4=p-7;
			while(p>p_end4){
				v=*p;
				max = std::max(max,v);
				min = std::min(min,v);
				p--;
			}
			p+=rowstride+1;
			const uchar* p_end5=p+7;
			while(p<p_end5){
				v=*p;
				max = std::max(max,v);
				min = std::min(min,v);
				p++;
			}
			p+=rowstride-2;
			const uchar* p_end6=p-5;
			while(p>p_end6){
				v=*p;
				max = std::max(max,v);
				min = std::min(min,v);
				p--;
			}
			p+=rowstride+2;
			const uchar* p_end7=p+3;
			while(p<p_end7){
				v=*p;
				max = std::max(max,v);
				min = std::min(min,v);
				p++;
			}

			// assign the threshold
			uchar diff=(max-min);
 *(thrmap_.data+x+rowstride*y)=diff;*/
/*if((int(lowerThreshold_)*int(threshold))/100>int(diff)){
 *(thrmap_.data+x+rowstride*y)=0;
				continue;
			}
			if (diff>upperThreshold_){
				diff=upperThreshold_;
			}
			else if (diff<lowerThreshold_){
				diff=lowerThreshold_;
			}
			uchar newval=int(diff)*threshold/100;
 *(thrmap_.data+x+rowstride*y)=newval;*/
/*}
	}
}
 */


// half sampling
inline void BriskLayer::halfsample(const cv::Mat& srcimg, cv::Mat& dstimg){
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
			const UCHAR_ALIAS* result=(UCHAR_ALIAS*)&result1;
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

inline void BriskLayer::twothirdsample(const cv::Mat& srcimg, cv::Mat& dstimg){
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

// adapted from OpenCV 2.3 features2d/matcher.hpp
Ptr<DescriptorMatcher> BruteForceMatcherSse::clone( bool emptyTrainData ) const
{
	BruteForceMatcherSse* matcher = new BruteForceMatcherSse(distance);
	if( !emptyTrainData )
	{
		std::transform( trainDescCollection.begin(), trainDescCollection.end(),
						matcher->trainDescCollection.begin(), clone_op );
	}
	return matcher;
}

void BruteForceMatcherSse::knnMatchImpl( const Mat& queryDescriptors, vector<vector<DMatch> >& matches, int k,
												const vector<Mat>& masks, bool compactResult )
{
	commonKnnMatchImpl( *this, queryDescriptors, matches, k, masks, compactResult );
}

void BruteForceMatcherSse::radiusMatchImpl( const Mat& queryDescriptors, vector<vector<DMatch> >& matches,
												   float maxDistance, const vector<Mat>& masks, bool compactResult )
{
	commonRadiusMatchImpl( *this, queryDescriptors, matches, maxDistance, masks, compactResult );
}

inline void BruteForceMatcherSse::commonKnnMatchImpl( BruteForceMatcherSse& matcher,
						  const Mat& queryDescriptors, vector<vector<DMatch> >& matches, int knn,
						  const vector<Mat>& masks, bool compactResult )
 {
	 typedef HammingSse::ValueType ValueType;
	 typedef HammingSse::ResultType DistanceType;
	 CV_DbgAssert( !queryDescriptors.empty() );
	 CV_Assert( DataType<ValueType>::type == queryDescriptors.type() );

	 int dimension = queryDescriptors.cols;
	 matches.reserve(queryDescriptors.rows);

	 size_t imgCount = matcher.trainDescCollection.size();
	 vector<Mat> allDists( imgCount ); // distances between one query descriptor and all train descriptors
	 for( size_t i = 0; i < imgCount; i++ )
		allDists[i] = Mat( 1, matcher.trainDescCollection[i].rows, DataType<DistanceType>::type );

	 for( int qIdx = 0; qIdx < queryDescriptors.rows; qIdx++ )
	 {
		 if( matcher.isMaskedOut( masks, qIdx ) )
		 {
			 if( !compactResult ) // push empty vector
				 matches.push_back( vector<DMatch>() );
		 }
		 else
		 {
			 // 1. compute distances between i-th query descriptor and all train descriptors
			 for( size_t iIdx = 0; iIdx < imgCount; iIdx++ )
			 {
				 CV_Assert( DataType<ValueType>::type == matcher.trainDescCollection[iIdx].type() ||  matcher.trainDescCollection[iIdx].empty() );
				 CV_Assert( queryDescriptors.cols == matcher.trainDescCollection[iIdx].cols ||
							matcher.trainDescCollection[iIdx].empty() );

				 const ValueType* d1 = (const ValueType*)(queryDescriptors.data + queryDescriptors.step*qIdx);
				 allDists[iIdx].setTo( Scalar::all(std::numeric_limits<DistanceType>::max()) );
				 for( int tIdx = 0; tIdx < matcher.trainDescCollection[iIdx].rows; tIdx++ )
				 {
					 if( masks.empty() || matcher.isPossibleMatch(masks[iIdx], qIdx, tIdx) )
					 {
						 const ValueType* d2 = (const ValueType*)(matcher.trainDescCollection[iIdx].data +
																  matcher.trainDescCollection[iIdx].step*tIdx);
						 allDists[iIdx].at<DistanceType>(0, tIdx) = matcher.distance(d1, d2, dimension);
					 }
				 }
			 }

			 // 2. choose k nearest matches for query[i]
			 matches.push_back( vector<DMatch>() );
			 vector<vector<DMatch> >::reverse_iterator curMatches = matches.rbegin();
			 for( int k = 0; k < knn; k++ )
			 {
				 DMatch bestMatch;
				 bestMatch.distance = std::numeric_limits<float>::max();
				 for( size_t iIdx = 0; iIdx < imgCount; iIdx++ )
				 {
					 if( !allDists[iIdx].empty() )
					 {
						 double minVal;
						 Point minLoc;
						 minMaxLoc( allDists[iIdx], &minVal, 0, &minLoc, 0 );
						 if( minVal < bestMatch.distance )
								 bestMatch = DMatch( qIdx, minLoc.x, (int)iIdx, (float)minVal );
					 }
				 }
				 if( bestMatch.trainIdx == -1 )
					 break;

				 allDists[bestMatch.imgIdx].at<DistanceType>(0, bestMatch.trainIdx) = std::numeric_limits<DistanceType>::max();
				 curMatches->push_back( bestMatch );
			 }
			 //TODO should already be sorted at this point?
			 std::sort( curMatches->begin(), curMatches->end() );
		 }
	 }
}

inline void BruteForceMatcherSse::commonRadiusMatchImpl( BruteForceMatcherSse& matcher,
							 const Mat& queryDescriptors, vector<vector<DMatch> >& matches, float maxDistance,
							 const vector<Mat>& masks, bool compactResult )
{
	typedef HammingSse::ValueType ValueType;
	typedef HammingSse::ResultType DistanceType;
	CV_DbgAssert( !queryDescriptors.empty() );
	CV_Assert( DataType<ValueType>::type == queryDescriptors.type() );

	int dimension = queryDescriptors.cols;
	matches.reserve(queryDescriptors.rows);

	size_t imgCount = matcher.trainDescCollection.size();
	for( int qIdx = 0; qIdx < queryDescriptors.rows; qIdx++ )
	{
		if( matcher.isMaskedOut( masks, qIdx ) )
		{
			if( !compactResult ) // push empty vector
				matches.push_back( vector<DMatch>() );
		}
		else
		{
			matches.push_back( vector<DMatch>() );
			vector<vector<DMatch> >::reverse_iterator curMatches = matches.rbegin();
			for( size_t iIdx = 0; iIdx < imgCount; iIdx++ )
			{
				CV_Assert( DataType<ValueType>::type == matcher.trainDescCollection[iIdx].type() ||
						   matcher.trainDescCollection[iIdx].empty() );
				CV_Assert( queryDescriptors.cols == matcher.trainDescCollection[iIdx].cols ||
						   matcher.trainDescCollection[iIdx].empty() );

				const ValueType* d1 = (const ValueType*)(queryDescriptors.data + queryDescriptors.step*qIdx);
				for( int tIdx = 0; tIdx < matcher.trainDescCollection[iIdx].rows; tIdx++ )
				{
					if( masks.empty() || matcher.isPossibleMatch(masks[iIdx], qIdx, tIdx) )
					{
						const ValueType* d2 = (const ValueType*)(matcher.trainDescCollection[iIdx].data +
																 matcher.trainDescCollection[iIdx].step*tIdx);
						DistanceType d = matcher.distance(d1, d2, dimension);

						//std::cout<<"d="<<d<<std::endl;
						if( d < maxDistance )
							curMatches->push_back( DMatch( qIdx, tIdx, (int)iIdx, (float)d ) );
					}
				}
			}
			std::sort( curMatches->begin(), curMatches->end() );
		}
	}
}
