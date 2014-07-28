/*
 * mikolajczyk.cpp
 *
 *  Created on: Jan 28, 2014
 *      Author: lestefan
 */

#include <sys/time.h>
#include <opencv2/opencv.hpp>
#include <brisk/brisk.h>
#include <brisk/brisk-v1.h>
#include <fstream>
#include <iostream>
#include <opencv2/nonfree/features2d.hpp>
#include "opencv2/nonfree/nonfree.hpp"

void help(char** argv) {
  std::cout
      << "This command line tool lets you evaluate different keypoint "
      << "detectors, descriptors and matchers."
      << std::endl
      << "usage:"
      << std::endl
      << argv[0]
      << " <dataset> <2nd> <detector> <descriptor> [descFile1 descFile1]"
      << std::endl
      << "    "
      << "dataset:    Folder containing the images. The images must be of .ppm "
      << std::endl
      << "    "
      << "            format. They must be named img#.ppm, and there must be "
      << std::endl
      << "    "
      << "            corresponding homographies named H1to#."
      << std::endl
      << "    "
      << "            You can also use the prefix rot-, then 2nd will be the"
      << std::endl
      << "    "
      << "            rotation in degrees."
      << std::endl
      << "    "
      << "2nd:        Number of the 2nd image (the 1st one is always img1.ppm)"
      << std::endl
      << "    "
      << "            or the rotation in degrees, if rot- used."
      << std::endl
      << "    "
      << "detector:   Feature detector, e.g. FAST, or BRISK. You can add the "
      << std::endl
      << "    "
      << "            threshold, e.g. BRISK50 or SURF2000"
      << std::endl
      << "    "
      << "descriptor: Feature descriptor, e.g. SURF, BRIEF, BRISK, U-BRISK,..."
      << std::endl
      << "    "
      << "[descFile]: Optional: files with descriptors to act as detected points."
      << std::endl;
}

cv::Mat outimg;
cv::Mat imgRGB1;
cv::Mat imgRGB2;
cv::Mat imgRGB3;
std::vector<cv::KeyPoint> keypoints, keypoints2;
cv::Mat descriptors, descriptors2;
cv::Ptr<cv::DescriptorMatcher> descriptorMatcher;
std::vector<std::vector<cv::DMatch> > matches;
double alpha=0.01;
int alpha_slider=1;
int alpha_slider_max = 100;
bool hamming = true;

void on_trackbar( int, void* )
{
 alpha = (double) alpha_slider/alpha_slider_max ;

 if (hamming)
   descriptorMatcher->radiusMatch(descriptors, descriptors2, matches, alpha*float(alpha_slider_max));
 else
   descriptorMatcher->radiusMatch(descriptors, descriptors2, matches, alpha);

 drawMatches(imgRGB1, keypoints, imgRGB2, keypoints2, matches,outimg,
             cv::Scalar(0, 255, 0), cv::Scalar(0, 0, 255),
             std::vector<std::vector<char> >(),
             cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);

 cv::imshow("Matches", outimg);
}

int main(int argc, char ** argv) {

  // process command line args
  if (argc != 5 && argc != 7) {
    help(argv);
    return 1;
  }

  // names of the two image files
  std::string fname1;
  std::string fname2;

  bool do_rot = false;
  if (strncmp("rot-", argv[1], 4) == 0) {
    do_rot = true;
    fname1 = std::string(argv[1] + 4) + "/img1.ppm";
    imgRGB1 = cv::imread(fname1);
  } else {
    fname1 = std::string(argv[1]) + "/img1.ppm";
    fname2 = std::string(argv[1]) + "/img" + std::string(argv[2]) + ".ppm";
    // open the images
    imgRGB1 = cv::imread(fname1);
    imgRGB2 = cv::imread(fname2);
    if (imgRGB2.empty()) {
      fname1 = std::string(argv[1]) + "/img1.pgm";
      fname2 = std::string(argv[1]) + "/img" + std::string(argv[2]) + ".pgm";
      // open the images
      imgRGB1 = cv::imread(fname1);
      imgRGB2 = cv::imread(fname2);
      if (imgRGB2.empty()) {
        std::cout << "image not found at " << fname2 << std::endl;
        return 2;
      }
    }
  }
  //unsigned int N=atoi(argv[3]);
  if (imgRGB1.empty()) {
    fname1 = std::string(argv[1] + 4) + "/img1.pgm";
    imgRGB1 = cv::imread(fname1);
    if (imgRGB1.empty()) {
      std::cout << "image not found at " << fname1 << std::endl;
      return 2;
    }
  }

  // convert to grayscale
  cv::Mat imgGray1;
  cv::cvtColor(imgRGB1, imgGray1, CV_BGR2GRAY);
  cv::Mat imgGray2;
  if (!do_rot) {
    cv::cvtColor(imgRGB2, imgGray2, CV_BGR2GRAY);
  }

  // project features into second image
  cv::Mat H;
  // read the homography matrix from file
  if (!do_rot) {
    std::string fname3 = std::string(argv[1]) + "/H1to" + std::string(argv[2])
        + "p";
    std::ifstream Hfile(fname3.c_str());
    if (!Hfile.good()) {
      std::cout << "Homografy not found at " << fname3 << std::endl;
      return 3;
    }

    H.create(3, 3, CV_64F);
    for (size_t i = 0; i < 3; i++) {
      for (size_t j = 0; j < 3; j++) {
        Hfile >> H.at<double>(i, j);
      }
    }
    Hfile.close();
  } else {
    float angle = float(atoi(argv[2])) / 180.0 * M_PI;
    cv::Mat H2 = cv::getRotationMatrix2D(
        cv::Point2f(imgRGB1.cols / 2.0 - 0.5, imgRGB1.rows / 2.0 - 0.5),
        angle * 180 / M_PI, 1);
    int height = fabs(cos(angle)) * imgRGB1.rows
        + fabs(sin(angle)) * imgRGB1.cols;
    int width = fabs(sin(angle)) * imgRGB1.rows
        + fabs(cos(angle)) * imgRGB1.cols;
    //int height=imgRGB1.cols;
    //int width=imgRGB1.cols;
    H2.at<double>(0, 2) += (width - imgRGB1.cols) / 2.0;
    H2.at<double>(1, 2) += (height - imgRGB1.rows) / 2.0;

    cv::warpAffine(imgGray1, imgGray2, H2, cv::Size(width, height));
    cv::warpAffine(imgRGB1, imgRGB2, H2, cv::Size(width, height));

    H.create(3, 3, CV_64F);
    H.at<double>(0, 0) = H2.at<double>(0, 0);
    H.at<double>(0, 1) = H2.at<double>(0, 1);
    H.at<double>(0, 2) = H2.at<double>(0, 2);
    H.at<double>(1, 0) = H2.at<double>(1, 0);
    H.at<double>(1, 1) = H2.at<double>(1, 1);
    H.at<double>(1, 2) = H2.at<double>(1, 2);
    H.at<double>(2, 0) = 0;
    H.at<double>(2, 1) = 0;
    H.at<double>(2, 2) = 1;

    // save the rotated image
    imwrite("rotated.pgm", imgGray2);
  }

  float region_scale = 1.0;

  // run FAST in first image
  int threshold;

  // create the detector:
  cv::Ptr<cv::FeatureDetector> detector;
  if (strncmp("FAST", argv[3], 4) == 0) {
    threshold = atoi(argv[3] + 4);
    if (threshold == 0)
      threshold = 30;
    detector = new cv::FastFeatureDetector(threshold, true);
  } else if (strncmp("ORB", argv[3], 3) == 0) {
    threshold = atoi(argv[3] + 3);
    detector = new cv::OrbFeatureDetector(threshold);
  } else if (strncmp("S-BRISKUNIFORM", argv[3], 14) == 0) {
    threshold = atoi(argv[3] + 14);
    if (threshold == 0)
      threshold = 50;
    detector =
        new brisk::ScaleSpaceFeatureDetector<brisk::HarrisScoreCalculator>(
            0, threshold, 160);
  } else if (strncmp("BRISKUNIFORM", argv[3], 12) == 0) {
    threshold = atoi(argv[3] + 12);
    if (threshold == 0)
      threshold = 50;
    detector =
        new brisk::ScaleSpaceFeatureDetector<brisk::HarrisScoreCalculator>(
            3, threshold, 160);
  } else if (strncmp("S-BRISKOLD", argv[3], 10) == 0) {
    threshold = atoi(argv[3] + 10);
    if (threshold == 0)
      threshold = 50;
    detector =
        new brisk_v1::BriskFeatureDetector(threshold, 0);
  } else if (strncmp("BRISKOLD", argv[3], 8) == 0) {
    threshold = atoi(argv[3] + 8);
    if (threshold == 0)
      threshold = 50;
    detector =
        new brisk_v1::BriskFeatureDetector(threshold, 3);
  } else if (strncmp("S-BRISK", argv[3], 7) == 0) {
    threshold = atoi(argv[3] + 7);
    if (threshold == 0)
      threshold = 50;
    detector =
        new brisk::ScaleSpaceFeatureDetector<brisk::HarrisScoreCalculator>(
            0, 0, threshold);
  } else if (strncmp("BRISK", argv[3], 5) == 0) {
    threshold = atoi(argv[3] + 5);
    if (threshold == 0)
      threshold = 50;
    detector =
        new brisk::ScaleSpaceFeatureDetector<brisk::HarrisScoreCalculator>(
            3, 0, threshold);
  } else if (strncmp("SURF", argv[3], 4) == 0) {
    threshold = atoi(argv[3] + 4);
    if (threshold == 0)
      threshold = 400;
    detector = new cv::SurfFeatureDetector(threshold);
  } else if (strncmp("SIFT", argv[3], 4) == 0) {
    float edgeThreshold = atof(argv[3] + 4);
    if (edgeThreshold == 0)
      edgeThreshold = 10.0;
    region_scale=4.0;
    detector = new cv::SiftFeatureDetector(0, 3, 0.04, edgeThreshold);
  } else {
    detector = cv::FeatureDetector::create(argv[3]);
  }
  if (detector.empty()) {
    std::cout << "Detector " << argv[3] << " not recognized. Check spelling!"
              << std::endl;
    return 3;
  }

  // run the detector:
  if (argc == 7) {
    // try to read descriptor files
    std::string desc1 = std::string(argv[5]);
    std::string desc2 = std::string(argv[6]);
    std::ifstream descf1(desc1.c_str());
    if (!descf1.good()) {
      std::cout << "Descriptor file not found at " << desc1 << std::endl;
      return 3;
    }
    std::ifstream descf2(desc2.c_str());
    if (!descf2.good()) {
      std::cout << "Descriptor file not found at " << desc2 << std::endl;
      return 3;
    }

    // fill the keypoints
    std::string str1;
    std::stringstream strstrm1;
    std::getline(descf1, str1);
    std::getline(descf1, str1);
    while (!descf1.eof()) {
      std::getline(descf1, str1);
      float x, y, a;
      strstrm1.str(str1);
      strstrm1 >> x;
      strstrm1 >> y;
      strstrm1 >> a;
      float r = sqrt(1.0 / a);
      keypoints.push_back(cv::KeyPoint(x, y, 4.0 * r));
    }
    std::string str2;
    std::stringstream strstrm2;
    std::getline(descf2, str2);
    std::getline(descf2, str2);
    while (!descf2.eof()) {
      std::getline(descf2, str2);
      float x, y, a;
      strstrm2.str(str2);
      strstrm2 >> x;
      strstrm2 >> y;
      strstrm2 >> a;
      float r = sqrt(1.0 / a);
      keypoints2.push_back(cv::KeyPoint(x, y, 4.0 * r));
    }

    // clean up
    descf1.close();
    descf2.close();
  } else {
    detector->detect(imgGray1, keypoints);
    // first image:
    detector->detect(imgGray2, keypoints2);
  }


  float repeatability;
  int count;
  cv::evaluateFeatureDetector(imgGray2, imgGray1, H, &keypoints, &keypoints2,
                              repeatability, count, detector);
  std::cout << "detection repeatability:" << repeatability << std::endl;


  // now the extractor:
  cv::Ptr<cv::DescriptorExtractor> descriptorExtractor;
  // now the extractor:
  bool intVals = false;
  if (std::string(argv[4]) == "BRISKOLD") {
    descriptorExtractor = new brisk::BriskDescriptorExtractor(
        true, true, brisk::BriskDescriptorExtractor::briskV1);
    intVals = true;
  } else if (std::string(argv[4]) == "U-BRISKOLD") {
    descriptorExtractor = new cv::BriskDescriptorExtractor(
        false, true, brisk::BriskDescriptorExtractor::briskV1);
    intVals = true;
  } else if (std::string(argv[4]) == "SU-BRISKOLD") {
    descriptorExtractor = new cv::BriskDescriptorExtractor(
        false, false, brisk::BriskDescriptorExtractor::briskV1);
    intVals = true;
  } else if (std::string(argv[4]) == "S-BRISKOLD") {
    descriptorExtractor = new cv::BriskDescriptorExtractor(
        true, false, brisk::BriskDescriptorExtractor::briskV1);
    intVals = true;
  } else if (std::string(argv[4]) == "BRISK") {
    descriptorExtractor = new brisk::BriskDescriptorExtractor(
        true, true,  brisk::BriskDescriptorExtractor::briskV2);
    intVals = true;
  } else if (std::string(argv[4]) == "U-BRISK") {
    descriptorExtractor = new cv::BriskDescriptorExtractor(
        false, true, brisk::BriskDescriptorExtractor::briskV2);
    intVals = true;
  } else if (std::string(argv[4]) == "SU-BRISK") {
    descriptorExtractor = new cv::BriskDescriptorExtractor(
        false, false, brisk::BriskDescriptorExtractor::briskV2);
    intVals = true;
  } else if (std::string(argv[4]) == "S-BRISK") {
    descriptorExtractor = new cv::BriskDescriptorExtractor(
        true, false, brisk::BriskDescriptorExtractor::briskV2);
    intVals = true;
  } else if (std::string(argv[4]) == "BRISK-L") {
    descriptorExtractor = new brisk::BriskDescriptorExtractor(
        true, true,  brisk::BriskDescriptorExtractor::briskV2, 2.35);
    intVals = true;
  } else if (std::string(argv[4]) == "U-BRISK-L") {
    descriptorExtractor = new cv::BriskDescriptorExtractor(
        false, true, brisk::BriskDescriptorExtractor::briskV2, 2.35);
    intVals = true;
  } else if (std::string(argv[4]) == "SU-BRISK-L") {
    descriptorExtractor = new cv::BriskDescriptorExtractor(
        false, false, brisk::BriskDescriptorExtractor::briskV2, 2.35);
    intVals = true;
  } else if (std::string(argv[4]) == "S-BRISK-L") {
    descriptorExtractor = new cv::BriskDescriptorExtractor(
        true, false, brisk::BriskDescriptorExtractor::briskV2, 2.35);
    intVals = true;
  } else if (std::string(argv[4]) == "BRISK-B") {
    descriptorExtractor = new brisk::BriskDescriptorExtractor(
        true, true,  brisk::BriskDescriptorExtractor::briskV2, 1.0);
    intVals = true;
  } else if (std::string(argv[4]) == "U-BRISK-B") {
    descriptorExtractor = new cv::BriskDescriptorExtractor(
        false, true, brisk::BriskDescriptorExtractor::briskV2, 1.0);
    intVals = true;
  } else if (std::string(argv[4]) == "SU-BRISK-B") {
    descriptorExtractor = new cv::BriskDescriptorExtractor(
        false, false, brisk::BriskDescriptorExtractor::briskV2, 1.0);
    intVals = true;
  } else if (std::string(argv[4]) == "S-BRISK-B") {
    descriptorExtractor = new cv::BriskDescriptorExtractor(
        true, false, brisk::BriskDescriptorExtractor::briskV2, 1.0);
    intVals = true;
  } else if (std::string(argv[4]) == "BRIEF") {
    descriptorExtractor = new cv::BriefDescriptorExtractor(64);
    intVals = true;
  } else if (std::string(argv[4]) == "ORB") {
     descriptorExtractor = new cv::OrbDescriptorExtractor();
     intVals = true;
  } else if (std::string(argv[4]) == "SURF") {
    descriptorExtractor = new cv::SurfDescriptorExtractor();
    hamming = false;
  } else if (std::string(argv[4]) == "SIFT") {
    descriptorExtractor = new cv::SiftDescriptorExtractor();
    hamming = false;
  } else if (std::string(argv[4]) == "FREAK") {
    descriptorExtractor = new cv::FREAK(true,true);
    intVals = true;
  } else if (std::string(argv[4]) == "U-FREAK") {
    descriptorExtractor = new cv::FREAK(false, true);
    intVals = true;
  } else if (std::string(argv[4]) == "SU-FREAK") {
    descriptorExtractor = new cv::FREAK(false, false);
    intVals = true;
  } else if (std::string(argv[4]) == "S-FREAK") {
    descriptorExtractor = new cv::FREAK(true, false);
    intVals = true;
  } else {
    descriptorExtractor = cv::DescriptorExtractor::create(argv[4]);
  }
  if (descriptorExtractor.empty()) {
    hamming = false;
    std::cout << "Descriptor " << argv[4] << " not recognized. Check spelling!"
              << std::endl;
    return 4;
  }

  // get the descriptors
  std::vector<cv::DMatch> indices;
  // first image
  descriptorExtractor->compute(imgGray2, keypoints2, descriptors2);
  // and the second one
  descriptorExtractor->compute(imgGray1, keypoints, descriptors);

  if (!intVals)
    assert(descriptors.type()==5);
  // must be floats, otherwise not supported by this program

  // now get the real count:
  cv::evaluateFeatureDetector(imgGray2, imgGray1, H, &keypoints, &keypoints2,
                              repeatability, count, detector);
  std::cout << "#correspondences:" << count << std::endl;
  std::cout << "detection repeatability:" << repeatability << std::endl;

  // save to files readable by the Mikolajczyk evaluation Matlab Toolset
  std::stringstream ofname1, ofname2;
  ofname1<<argv[1]<<"/"<<argv[3]<<"-"<<argv[4]<<"-1.kpts";
  ofname2<<argv[1]<<"/"<<argv[3]<<"-"<<argv[4]<<"-"<<argv[2]<<".kpts";
  std::ofstream file1(ofname1.str());
  std::ofstream file2(ofname2.str());

  const int descIntLength = descriptors.cols * 8;
  assert(descIntLength==descriptors2.cols*8);

  // descriptor dimension
  if (intVals) {
    file1 << descIntLength << std::endl;
    file2 << descIntLength << std::endl;
  } else {
    file1 << descriptors.cols << std::endl;
    file2 << descriptors2.cols << std::endl;
  }
  // number of points
  file1 << keypoints.size() << std::endl;
  for (size_t k = 0; k < keypoints.size(); k++) {
    const float r = region_scale*keypoints[k].size / 4.0;
    const float a = 1.0 / (r * r);
    file1 << keypoints[k].pt.x << " " << keypoints[k].pt.y << " " << a << " "
          << 0.0 << " " << a;
    if (intVals) {
      /*cv::UINT32_ALIAS* desc=(cv::UINT32_ALIAS*)(&(descriptors.at<uchar>(k,0)));
       for(int i=0; i<descIntLength; i++){
       file1<<" "<<desc[i];
       }*/
      for (int i = 0; i < descriptors.cols; i++) {
        for (int j = 0; j < 8; j++) {
          float outbit = 0.0;
          if (int(descriptors.at<uchar>(k, i) & (1 << j)) > 0)
            outbit = 1.0;
          file1 << " " << outbit;
        }
      }
    } else {
      for (int i = 0; i < descriptors.cols; i++) {
        file1 << " " << descriptors.at<float>(k, i);
      }
    }
    file1 << std::endl;
  }
  file2 << keypoints2.size() << std::endl;
  for (size_t k = 0; k < keypoints2.size(); k++) {
    const float r = region_scale*keypoints2[k].size / 4.0;
    const float a = 1.0 / (r * r);
    file2 << keypoints2[k].pt.x << " " << keypoints2[k].pt.y << " " << a << " "
          << 0.0 << " " << a;
    if (intVals) {
      /*cv::UINT32_ALIAS* desc=(cv::UINT32_ALIAS*)(&(descriptors2.at<uchar>(k,0)));
       for(int i=0; i<descIntLength; i++){
       file2<<" "<<desc[i];
       }*/
      for (int i = 0; i < descriptors.cols; i++) {
        for (int j = 0; j < 8; j++) {
          float outbit = 0.0;
          if (int(descriptors2.at<uchar>(k, i) & (1 << j)) > 0)
            outbit = 1.0;
          file2 << " " << outbit;
        }
      }
    } else {
      for (int i = 0; i < descriptors2.cols; i++)
        file2 << " " << descriptors2.at<float>(k, i);
    }
    file2 << std::endl;
  }
  file1.close();
  file2.close();

  std::ofstream infofile("Matlab/Mikolajczyk/info.m");
  infofile << "%this file was automatically generated with " << argv[0]
           << std::endl;
  infofile << "image1='../../" << fname1 << "'; % image 1" << std::endl;
  if (do_rot)
    infofile << "image2='../../" << "rotated.pgm" << "'; % image 2"
             << std::endl;
  else
    infofile << "image2='../../" << fname2 << "'; % image 2" << std::endl;
  infofile << "H1to2=" << H << "; % homography from image 1 to image2"
           << std::endl;
  infofile << "dataset='" << argv[1] << " " << argv[2]
           << "'; % dataset used, sequence number of 2nd image" << std::endl;
  infofile << "detector='" << argv[3] << "'; % detector and threshold"
           << std::endl;
  infofile << "descriptor='" << argv[4] << "'; % descriptor" << std::endl;
  infofile << "keypoints1=[" << std::endl;
  for (unsigned int k = 0; k < keypoints.size(); k++)
    infofile << keypoints[k].pt.x << " " << keypoints[k].pt.y << " "
             << keypoints[k].size << " " << keypoints[k].angle << std::endl;
  infofile << "];" << std::endl;
  infofile << "keypoints2=[" << std::endl;
  for (unsigned int k = 0; k < keypoints2.size(); k++)
    infofile << keypoints2[k].pt.x << " " << keypoints2[k].pt.y << " "
             << keypoints2[k].size << " " << keypoints2[k].angle << std::endl;
  infofile << "];" << std::endl;
  infofile.close();

  std::cout << "wrote: " << std::endl
      << ofname1.str() << std::endl
      << ofname2.str() << std::endl
      << "Matlab/Mikolajczyk/info.m" << std::endl;

  if (hamming){
    descriptorMatcher = new brisk::BruteForceMatcherSse;
    alpha_slider = descriptors.cols;
    alpha_slider_max = descriptors.cols*8; // bits
  }
  else
    descriptorMatcher = new cv::BFMatcher(cv::NORM_L2);

  /*/ save detections
  cv::Mat det1, det2;
  cv::drawKeypoints(imgRGB1,keypoints,det1,cv::Scalar(0,255,255),cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
  cv::drawKeypoints(imgRGB2,keypoints2,det2,cv::Scalar(0,255,255),cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
  std::cout<<keypoints.size()<<std::endl;
  std::cout<<keypoints2.size()<<std::endl;
  cv::imwrite("/home/lestefan/Desktop/det1.png",det1);
  cv::imwrite("/home/lestefan/Desktop/det2.png",det2);*/

  /*/ Create Trackbar
  cv::namedWindow("Matches");
  cv::createTrackbar( "Matching Threshold", "Matches", &alpha_slider, alpha_slider_max, on_trackbar );

  // Show matches
  on_trackbar( alpha_slider, 0 );

   char key=0;
   while(key!=27){
     key=cvWaitKey(10);
   }*/

  //cv::imwrite("out.ppm",outimg)
  /*imgRGB1.release();
   imgRGB2.release();
   imgGray1.release();
   imgGray2.release();
   outimg.release();*/

  return 0;
}
