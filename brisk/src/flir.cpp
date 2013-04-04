/*
 * flir.cpp
 *
 *  Created on: Feb 28, 2013
 *      Author: lestefan
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
#include <boost/thread.hpp>
#include <ros/ros.h>
#include <ros/package.h>
#include <image_transport/image_transport.h>
#include <opencv/highgui.h>
#if ROS_VERSION_MINIMUM(1, 9, 44)
#include <cv_bridge/cv_bridge.h>
#else
#include <cv_bridge/CvBridge.h>
#endif
#include <falsecolor.h>

cv::Ptr<cv::FeatureDetector> detector;
cv::Ptr<cv::DescriptorExtractor> extractor;
cv::Ptr<cv::DescriptorMatcher> matcher;

cv::Mat descriptorsLast;
std::vector<cv::KeyPoint> keypointsLast;
cv::Mat img_mono8Last;

class converter_16_8
{
  enum
  {
    histminmembersperbucket = 10,
  };
private:
  uint16_t max_;
  uint16_t min_;
  static converter_16_8* inst_;
  bool firstframe_;
public:
  converter_16_8()
  {
    min_ = std::numeric_limits<uint16_t>::max();
    max_ = 0;
    firstframe_ = true;
  }

  ~converter_16_8()
  {
    delete inst_;
    inst_ = NULL;
  }

  static converter_16_8& Instance()
  {
    if (!inst_)
    {
      inst_ = new converter_16_8;
    }
    return *inst_;
  }

  void convert_to8bit(const cv::Mat& img16, cv::Mat& img8)
  {

    uint16_t min = std::numeric_limits<uint16_t>::max();
    uint16_t max = 0;

    //make a histogram of intensities
    typedef std::map<uint16_t, int> hist_t;
    hist_t hist;

    for (int i = 0; i < img16.cols; ++i)
    {
      for (int j = 0; j < img16.rows; ++j)
      {
        uint16_t temp = img16.at<uint16_t>(j, i);
        hist[temp]++;
      }
    }

    //find the main section of the histogram
    for (hist_t::const_iterator it = hist.begin(); it != hist.end(); ++it)
    {
      if (it->second > histminmembersperbucket)
      {
        if (it->first > max)
        {
          max = it->first;
        }
        if (it->first < min)
        {
          min = it->first;
        }
      }
    }

    if (firstframe_)
    {
      min_ = min;
      max_ = max;
    }

    //exp smoothing
    double expsm = 0.95;
    min_ = expsm * min_ + (1. - expsm) * min;
    max_ = expsm * max_ + (1. - expsm) * max;

    for (int i = 0; i < img16.cols; ++i)
    {
      for (int j = 0; j < img16.rows; ++j)
      {
        double temp = (double)(img16.at<uint16_t>(j, i) - min_);
        int val = ((temp / (max_ - min_)) * 255);
        val = val > std::numeric_limits<uint8_t>::max() ? std::numeric_limits<uint8_t>::max() : val < 0 ? 0 : val; //saturate
        img8.at<uint8_t>(j, i) = (uint8_t)val;
      }
    }

    firstframe_ = false;
  }
};

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{

#if ROS_VERSION_MINIMUM(1, 9, 44)
#else
  sensor_msgs::CvBridge bridge;
#endif
  try
  {

    // get image

#if ROS_VERSION_MINIMUM(1, 9, 44)
    cv_bridge::CvImagePtr ptr = cv_bridge::toCvCopy(msg, "mono16");
#else
    cv::Mat img=bridge.imgMsgToCv(msg, "mono16");
#endif
    cv::Mat img = ptr->image;

    // only at first call
    if (img_mono8Last.cols == 0)
    {
      img_mono8Last = cv::Mat::zeros(img.rows, img.cols, CV_8UC1);
    }

    cv::Mat half_img(img.rows * 2 / 3, img.cols * 2 / 3, CV_16UC1);
    //timeval start, end;
    //gettimeofday(&start, NULL);
    //for(int i=0; i<100; ++i)
    //brisk::ScaleSpaceLayer<brisk::HarrisScoreCalculatorFloat>::twothirdsample16(img,half_img);
    //gettimeofday(&end, NULL);
    //double dt = ((end.tv_sec * 1000000 + end.tv_usec) - (start.tv_sec * 1000000 + start.tv_usec))/ 1000.;
    //std::cout<<dt/100<<"ms mono16"<<std::endl;

    //cv::Mat raw=bridge.imgMsgToCv(msg, "mono16");
    //cv::Mat img;
    //cv::medianBlur(raw,img, 3);
    cv::Mat img_mono32F;
    cv::Mat half_img_mono32F;
    img.convertTo(img_mono32F, CV_32F);
    half_img.convertTo(half_img_mono32F, CV_32F);

    /*BRISKing*/
    // descriptors
    std::vector<cv::KeyPoint> keypoints;
    detector->detect(img, keypoints);

    // extract
    cv::Mat descriptors;
    extractor->compute(img, keypoints, descriptors);

    // match
    std::vector<std::vector<cv::DMatch> > matches;
    matcher->radiusMatch(descriptorsLast, descriptors, matches, 50);


    //convert 8 bit and false color conversion
    static bool dofalsecolor = false;

    cv::Mat img_mono8_ir, img_mono8;
    img_mono8_ir.create(img.rows, img.cols, CV_8UC1);
    converter_16_8::Instance().convert_to8bit(img, img_mono8_ir);

    if (dofalsecolor)
    {
      convertFalseColor(img_mono8_ir, img_mono8, palette::False_color_palette4);
    }
    else
    {
      img_mono8 = img_mono8_ir;
    }

    // draw images
    cv::Mat drawing;
    cv::drawKeypoints(img_mono8, keypoints, drawing, cv::Scalar(255, 0, 0), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS); // blue,multisize

    cv::Mat matchImg;
    cv::drawMatches(img_mono8Last, keypointsLast, img_mono8, keypoints, matches, matchImg, cv::Scalar(0, 255, 0),
                    cv::Scalar(255, 0, 0), std::vector<std::vector<char> >(),
                    cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS | cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);

    // display
    cv::imshow("Raw", img_mono8);
    cv::imshow("Matches", matchImg);
    //cv::imshow("Half", half_img_mono8);
    cv::imshow("Points", drawing);

    char key = cv::waitKey(20);
    if (key == 'c')
    {
      dofalsecolor = !dofalsecolor;
    }

    // memory
    descriptorsLast = descriptors;
    keypointsLast = keypoints;
    img_mono8Last = img_mono8;
    /*gettimeofday(&start, NULL);
     for(int i=0; i<100; ++i)
     brisk::ScaleSpaceLayer<brisk::HarrisScoreCalculatorFloat>::halfsample(img_mono8,half_img_mono8);
     gettimeofday(&end, NULL);
     dt = ((end.tv_sec * 1000000 + end.tv_usec) - (start.tv_sec * 1000000 + start.tv_usec))/ 1000.;
     std::cout<<dt/100<<"ms mono8"<<std::endl;*/
  }

#if ROS_VERSION_MINIMUM(1, 9, 44)
  catch (cv_bridge::Exception& e)
#else
  catch (sensor_msgs::CvBridgeException& e)
#endif
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "flir");
  std::string path = ros::package::getPath("brisk");
  detector = new brisk::ScaleSpaceFeatureDetector<brisk::HarrisScoreCalculatorFloat>(2, 14, 80);
  extractor = new cv::BriskDescriptorExtractor(path + "/brisk.ptn", false, true);
  matcher = new cv::BruteForceMatcherSse();

  ros::NodeHandle nh;
  cvNamedWindow("Raw");
  cvNamedWindow("Matches");
  cvNamedWindow("Points");
  cvMoveWindow("Raw", 200, 50);
  cvMoveWindow("Matches", 200, 350);
  cvMoveWindow("Points", 550, 50);

  cvStartWindowThread();
  image_transport::ImageTransport it(nh);
  image_transport::Subscriber sub = it.subscribe("cam2/image_raw", 1, imageCallback);
  ros::spin();
  cvDestroyWindow("Raw");
  cvDestroyWindow("Matches");
  cvDestroyWindow("Points");

  return 0;
}

converter_16_8* converter_16_8::inst_ = NULL;
