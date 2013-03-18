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

#include <opencv2/opencv.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <brisk/brisk.h>
#include <brisk/harris.h>
#include <brisk/HarrisScoreCalculator.hpp>
#include <brisk/HarrisScoreCalculatorFloat.hpp>
#include <brisk/ScaleSpaceFeatureDetector.hpp>
#include "../include/brisk/brisk.h"
#include <fstream>
#include <iostream>
#include <list>
#include <iomanip>
#include <brisk/harris.h>
#include <sys/time.h>

//standard configuration for the case of no file given
const int n=12;
const float r=2.5; // found 8-9-11, r=3.6, exponent 1.5

void help(char** argv){
	std::cout << "This command line tool lets you evaluate different keypoint "
			<< "detectors, descriptors and matchers." << std::endl
			<< "usage:" << std::endl
			<< argv[0] << " <dataset> <2nd> <detector> <descriptor> [descFile1 descFile1]" << std::endl
			<< "    " << "dataset:    Folder containing the images. The images must be of .ppm "<< std::endl
			<< "    " << "            format. They must be named img#.ppm, and there must be "<< std::endl
			<< "    " << "            corresponding homographies named H1to#." << std::endl
			<< "    " << "            You can also use the prefix rot-, then 2nd will be the" << std::endl
			<< "    " << "            rotation in degrees." << std::endl
			<< "    " << "2nd:        Number of the 2nd image (the 1st one is always img1.ppm)"<< std::endl
			<< "    " << "            or the rotation in degrees, if rot- used." << std::endl
			<< "    " << "detector:   Feature detector, e.g. AGAST, or BRISK. You can add the "<< std::endl
			<< "    " << "            threshold, e.g. BRISK80 or SURF2000"<< std::endl
			<< "    " << "descriptor: Feature descriptor, e.g. SURF, BRIEF, BRISK or U-BRISK."<< std::endl
			<< "    " << "[descFile]: Optional: files with descriptors to act as detected points."<< std::endl;
}

int main(int argc, char ** argv) {

	//std::cout<<sizeof(cv::Point2i)<<" "<<sizeof(CvPoint)<<std::endl;

	// process command line args
	if(argc != 3 && argc != 7 && argc != 1){
		help(argv);
		return 1;
	}

	// names of the two image files
	std::string fname1;
	std::string fname2;
	cv::Mat imgRGB1;
	cv::Mat imgRGB2;
	cv::Mat imgRGB3;
	bool do_rot=false;
	// standard file extensions
	std::vector<std::string> fextensions;
	fextensions.push_back(".bmp");
	fextensions.push_back(".jpeg");
	fextensions.push_back(".jpg");
	fextensions.push_back(".jpe");
	fextensions.push_back(".jp2");
	fextensions.push_back(".png");
	fextensions.push_back(".pgm");
	fextensions.push_back(".ppm");
	fextensions.push_back(".sr");
	fextensions.push_back(".ras");
	fextensions.push_back(".tiff");
	fextensions.push_back(".tif");

	// if no arguments are passed: 
	if(argc==1){
		int i=0;
		int fextensions_size=fextensions.size();
		while(imgRGB1.empty()||imgRGB2.empty()){
			fname1 = "../images/img1"+fextensions[i];
			fname2 = "../images/img2"+fextensions[i];
			imgRGB1 = cv::imread(fname1);
			imgRGB2 = cv::imread(fname2);
			i++;
			if(i>=fextensions_size) break;
		}
		if (imgRGB2.empty()||imgRGB2.empty())
		{
			std::cout<<"image(s) "<<fname1<<", "<<fname2<<" not found." << std::endl;
			return 2;
		}
	}
	else{
		if(strncmp("rot-", argv[1], 4)==0){
			do_rot=true;
			int i=0;
			int fextensions_size=fextensions.size();
			while(imgRGB1.empty()){
				fname1 = std::string(argv[1]+4)+"/img1"+fextensions[i];
				imgRGB1 = cv::imread(fname1);
				i++;
				if(i>=fextensions_size) break;
			}
			if (imgRGB2.empty())
			{
				std::cout<<"image not found." << std::endl;
				return 2;
			}
		}
		else{
			int i=0;
			int fextensions_size=fextensions.size();
			while(imgRGB1.empty()||imgRGB2.empty()){
				fname1 = std::string(argv[1])+"/img1"+fextensions[i];
				fname2 = std::string(argv[1])+"/img"+std::string(argv[2])+fextensions[i];
				imgRGB1 = cv::imread(fname1);
				imgRGB2 = cv::imread(fname2);
				i++;
				if(i>=fextensions_size) break;
			}
			if (imgRGB2.empty()||imgRGB2.empty())
			{
				std::cout<<"image(s)"<<fname1<<", "<<fname2<<" not found." << std::endl;
				return 2;
			}
		}
		//unsigned int N=atoi(argv[3]);
		if (imgRGB1.empty())
		{
			fname1 = std::string(argv[1]+4)+"/img1.pgm";
			imgRGB1 = cv::imread(fname1);
			if (imgRGB1.empty()){
				std::cout<<"image not found at " << fname1 << std::endl;
				return 2;
			}
		}
	}

	// convert to grayscale
	cv::Mat imgGray1;
	cv::cvtColor(imgRGB1, imgGray1, CV_BGR2GRAY);
	cv::Mat imgGray2;
	if(!do_rot){
		cv::cvtColor(imgRGB2, imgGray2, CV_BGR2GRAY);
	}

	// run FAST in first image
	int threshold;

	// create the detector:
	cv::Ptr<cv::FeatureDetector> harrisdet = new brisk::ScaleSpaceFeatureDetector<brisk::HarrisScoreCalculatorFloat>(
			3,30);
	cv::Ptr<cv::FeatureDetector> briskdet = new cv::BriskFeatureDetector(60,4);

	int its = 1;
	struct timeval start;
	struct timeval end;
	double dt;

std::cout<<"Running "<<its<<" iterations of Harris and BRISK"<<std::endl;



//warm up
int kpt = 0;	
	for(int i = 0;i<4;++i){
	std::vector<cv::KeyPoint> keypoints;
		harrisdet->detect(imgGray1,keypoints);
		if(!keypoints.empty())
			kpt += keypoints.at(0).pt.x; //make sure the detection is not optimized away
	}
		
//HARRIS
	gettimeofday(&start, NULL);
	std::vector<cv::KeyPoint> keypoints1;
	for(int i = 0;i<its;++i){
		std::vector<cv::KeyPoint> keypoints;
		harrisdet->detect(imgGray1,keypoints);
		if(!keypoints.empty())
			kpt += keypoints.at(0).pt.x; //make sure the detection is not optimized away

		if(i==0)
			keypoints1=keypoints;
	}
	gettimeofday(&end, NULL);
	dt = ((end.tv_sec * 1000000 + end.tv_usec) - (start.tv_sec * 1000000 + start.tv_usec))/ 1000.;

	std::cout<<"Harris took "<<dt / (double)its<<" ms per iteration for image "<<imgGray1.cols<<"x"<<imgGray1.rows<<" pixels"<<std::endl;
	
	cv::Mat out;
	cv::drawKeypoints(imgGray1,keypoints1,out,cv::Scalar(255,0,0),cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
	cv::imshow("img1",out);
	cv::waitKey();

//BRISK
gettimeofday(&start, NULL);		
	for(int i = 0;i<its;++i){
	std::vector<cv::KeyPoint> keypoints;
		briskdet->detect(imgGray1,keypoints);
		if(!keypoints.empty())
			kpt += keypoints.at(0).pt.x; //make sure the detection is not optimized away
	}
	gettimeofday(&end, NULL);
	dt = ((end.tv_sec * 1000000 + end.tv_usec) - (start.tv_sec * 1000000 + start.tv_usec))/ 1000.;
	std::cout<<"BRISK took "<<dt / (double)its<<" ms per iteration for image "<<imgGray1.cols<<"x"<<imgGray1.rows<<" pixels"<<std::endl;



	return 0;
}
