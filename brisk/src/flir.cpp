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

#define ROS_MIN_MAJOR 1
#define ROS_MIN_MINOR 8
#define ROS_MIN_PATCH 16


#if ROS_VERSION_MINIMUM(ROS_MIN_MAJOR, ROS_MIN_MINOR, ROS_MIN_PATCH)
#include <cv_bridge/cv_bridge.h>
#else
#include <cv_bridge/CvBridge.h>
#endif
#include <flir/falsecolor.h>


cv::Ptr<cv::FeatureDetector> detector;
cv::Ptr<cv::DescriptorExtractor> extractor;
cv::Ptr<cv::DescriptorMatcher> matcher;

cv::Mat descriptorsLast;
std::vector<cv::KeyPoint> keypointsLast;
cv::Mat img_mono8Last;

class ImagePublisher {
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Publisher image_pub_;

  ImagePublisher():it_(nh_) {
    std::cout<<"Advertised /flir_demo/image_out"<<std::endl;
    image_pub_ = it_.advertise("/flir_demo/image_out", 10);
    seq_ = 0;
  }
  ~ImagePublisher() {
    delete inst_;
    inst_ = NULL;
  }

 public:
  static ImagePublisher* Instance() {
    if (inst_ == NULL) {
      inst_ = new ImagePublisher();
    }
    assert(inst_);
    return inst_;
  }

  void publish(cv::Mat image){
    if(image_pub_.getNumSubscribers()){
      cv_bridge::CvImagePtr cv_ptr(new cv_bridge::CvImage);
      cv_ptr->encoding = "bgr8";
      cv_ptr->header.seq = seq_++;
      cv_ptr->header.stamp = ros::Time::now();
      cv_ptr->image = image;
      sensor_msgs::ImagePtr msg = cv_ptr->toImageMsg();
      image_pub_.publish(msg);
    }
  }

  static ImagePublisher* inst_;
  int seq_;
};

void imageCallback(const sensor_msgs::ImageConstPtr& msg) {

#if ROS_VERSION_MINIMUM(ROS_MIN_MAJOR, ROS_MIN_MINOR, ROS_MIN_PATCH)
#else
  sensor_msgs::CvBridge bridge;
#endif
  try {

    // get image

#if ROS_VERSION_MINIMUM(ROS_MIN_MAJOR, ROS_MIN_MINOR, ROS_MIN_PATCH)
    cv_bridge::CvImagePtr ptr = cv_bridge::toCvCopy(msg, "mono16");
    cv::Mat img = ptr->image;
#else
    cv::Mat img = bridge.imgMsgToCv(msg, "mono16");
#endif

    // only at first call
    if (img_mono8Last.cols == 0) {
      img_mono8Last = cv::Mat::zeros(img.rows, img.cols, CV_8UC1);
    }

    cv::Mat half_img(img.rows * 2 / 3, img.cols * 2 / 3, CV_16UC1);

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

    bool doTempScaling = true;
    converter_16_8::Instance().convert_to8bit(img, img_mono8_ir, doTempScaling);
    //converter_16_8::Instance().toneMapping(img, img_mono8_ir);

    if (dofalsecolor) {
      convertFalseColor(img_mono8_ir, img_mono8, palette::False_color_palette4, doTempScaling,
                        converter_16_8::Instance().getMin(), converter_16_8::Instance().getMax());
    } else {
      img_mono8 = img_mono8_ir;
    }

    // draw images
    cv::Mat drawing;
    cv::drawKeypoints(img_mono8, keypoints, drawing, cv::Scalar(255, 0, 0), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);  // blue,multisize

    cv::Mat matchImg;
    cv::drawMatches(img_mono8Last, keypointsLast, img_mono8, keypoints, matches, matchImg, cv::Scalar(0, 255, 0),
                    cv::Scalar(255, 0, 0), std::vector<std::vector<char> >(),
                    cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS | cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);

    // display

    cv::Mat fullImg(img.rows * 2, img.cols * 2, CV_8UC3);
    cv::Mat dst_tl = fullImg(cv::Rect(0, 0, img.cols, img.rows));
    cv::Mat img_monorgb;
    if(img_mono8.type() == CV_8UC1){
      cv::cvtColor(img_mono8, img_monorgb, CV_GRAY2BGR);
      img_monorgb.copyTo(dst_tl);
    }else{
      img_mono8.copyTo(dst_tl);
    }
    cv::Mat dst_tr = fullImg(cv::Rect(img.cols, 0, img.cols, img.rows));
    drawing.copyTo(dst_tr);
    cv::Mat dst_bl = fullImg(cv::Rect(0, img.rows, img.cols * 2, img.rows));
    matchImg.copyTo(dst_bl);

    cv::imshow("Points", fullImg);

    ImagePublisher::Instance()->publish(fullImg);

    char key = cv::waitKey(20);
    if (key == 'c') {
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

#if ROS_VERSION_MINIMUM(ROS_MIN_MAJOR, ROS_MIN_MINOR, ROS_MIN_PATCH)
  catch (cv_bridge::Exception& e)
#else
  catch (sensor_msgs::CvBridgeException& e)
#endif
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "flir");
  std::string path = ros::package::getPath("brisk");
  detector = new brisk::ScaleSpaceFeatureDetector<brisk::HarrisScoreCalculatorFloat>(2, 14, 50);
  extractor = new cv::BriskDescriptorExtractor(path + "/brisk.ptn", false, true);
  matcher = new cv::BruteForceMatcherSse();

  ImagePublisher::Instance();

  ros::NodeHandle nh;
  cvNamedWindow("Points");
  cvMoveWindow("Points", 50, 50);

  cvStartWindowThread();
  image_transport::ImageTransport it(nh);
  image_transport::Subscriber sub = it.subscribe("cam2/image_raw", 1, imageCallback);
  ros::spin();
  cvDestroyWindow("Raw");
  cvDestroyWindow("Matches");
  cvDestroyWindow("Points");

  return 0;
}

ImagePublisher* ImagePublisher::inst_ = NULL;
