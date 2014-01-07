/*
 Copyright (C) 2013  The Autonomous Systems Lab, ETH Zurich,
 Stefan Leutenegger and Simon Lynen.

 BRISK - Binary Robust Invariant Scalable Keypoints
 Reference implementation of
 [1] Stefan Leutenegger,Margarita Chli and Roland Siegwart, BRISK:
 Binary Robust Invariant Scalable Keypoints, in Proceedings of
 the IEEE International Conference on Computer Vision (ICCV2011).

 This file is part of BRISK.

 All rights reserved.

 Redistribution and use in source and binary forms, with or without
 modification, are permitted provided that the following conditions are met:
 * Redistributions of source code must retain the above copyright
 notice, this list of conditions and the following disclaimer.
 * Redistributions in binary form must reproduce the above copyright
 notice, this list of conditions and the following disclaimer in the
 documentation and/or other materials provided with the distribution.
 * Neither the name of the <organization> nor the
 names of its contributors may be used to endorse or promote products
 derived from this software without specific prior written permission.

 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
 DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <fstream>
#include <iomanip>
#include <iostream>
#include <list>

#include <brisk/brisk.h>
#include <brisk/internal/timer.h>
#include <glog/logging.h>
#include <opencv2/opencv.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/nonfree/features2d.hpp>
#include <opencv2/nonfree/nonfree.hpp>

#include "./bench-ds.h"
#include "./image-io.h"

void runpipeline(std::vector<DatasetEntry>& dataset,
                 const std::string& briskbasepath);
bool runverification(std::vector<DatasetEntry>& current_dataset,
                     std::vector<DatasetEntry>& verification_dataset);
void draw(std::vector<DatasetEntry>& dataset);

enum Parameters {
  doKeypointDetection = true,
  doDescriptorComputation = true,  //before switching this to false, you need to add a function which removes keypoints that are too close to the border.
  exitfirstfail = false,  //stop on first image for which the verification fails
  drawKeypoints = false,  //drawimages with keypoints

  BRISK_absoluteThreshold = 20,
  BRISK_uniformityradius = 30,
  BRISK_octaves = 0,
  BRISK_maxNumKpt = 4294967296,  // should be generic enough...
  BRISK_scaleestimation = true,
  BRISK_rotationestimation = true,
};

int main(int /*argc*/, char ** argv) {
  google::InitGoogleLogging(argv[0]);

  std::string imagepath = "./test_data/";
  std::string datasetfilename = "data.set";

  std::string datasetfullpath = imagepath + "/" + datasetfilename;

  std::cout << "Checking if there is a dataset at ..." << datasetfullpath;
  std::ifstream dataset_file_stream(datasetfullpath.c_str());
  bool have_dataset = dataset_file_stream.good();
  dataset_file_stream.close();

  std::cout << (have_dataset ? " True " : " False") << std::endl;

  std::vector < DatasetEntry > dataset;

  if (!have_dataset) {
    /***
     * READ IMAGES
     */
    std::cout << "No dataset found at " << datasetfullpath
        << " will create one from the images in " << imagepath << std::endl;

    std::vector<std::string> imgpaths;
    bool doLexicalsort = true;
    std::vector<std::string> search_paths;
    search_paths.push_back(imagepath);
    brisk::Getfilelists(search_paths, doLexicalsort, "", &imgpaths);

    /**
     * MAKE THE DATASET
     */
    std::cout << "Reading dataset path: " << imagepath << " got images: "
        << std::endl;
    for (std::vector<std::string>::iterator it = imgpaths.begin(), end =
        imgpaths.end(); it != end; ++it) {
      dataset.push_back(DatasetEntry());
      DatasetEntry& image = dataset.at(dataset.size() - 1);
      image.readImage(*it);
      std::cout << image.print();
    }

    /**
     * RUN THE PIPELINE
     */

    runpipeline(dataset, datasetfullpath);

    /**
     * SAVE THE DATASET
     */
    std::cout << "Done handling images from " << imagepath << ". " << std::endl
        << "Now saving to " << datasetfullpath << std::endl << std::endl;

    std::ofstream ofs(std::string(datasetfullpath).c_str());
    {
      boost::archive::binary_oarchive oa(ofs);
      oa << dataset;
    }
    std::cout << "Done. Now re-run to check against the dataset." << std::endl;
    return 0;
  } else {
    /***
     * LOAD REFERENCE CHECK FILES AND DATASET
     */
    std::cout << "Dataset found at " << datasetfullpath << std::endl;

    /**
     * DESERIALIZE THE DATASET
     */
    std::ifstream ifs(std::string(datasetfullpath).c_str());
    {
      boost::archive::binary_iarchive ia(ifs);
      ia >> dataset;
    }
    std::vector < DatasetEntry > verifyds;
    verifyds = dataset;  //nice deep copy

    std::cout << "Loaded dataset:" << std::endl;
    int i = 0;
    for (std::vector<DatasetEntry>::const_iterator it = dataset.begin(), end =
        dataset.end(); it != end; ++it, ++i) {
      std::cout << i << ": " << it->print() << std::endl;
    }

    //remove all processed data, only keep the images
    for (std::vector<DatasetEntry>::iterator it = dataset.begin(), end = dataset
        .end(); it != end; ++it)
      it->clear_processed_data(doDescriptorComputation, doKeypointDetection);

    /**
     * RUN THE PIPELINE ON THE DATASET
     */
    //for(size_t i=0; i<100; ++i)
    runpipeline(dataset, datasetfullpath);

    /**
     * RUN THE VERIFICATION
     * fingers crossed :)
     */
    bool verificationOK = runverification(dataset, verifyds);
    if (verificationOK) {
      std::cout << std::endl << "******* Verification success *******"
          << std::endl << std::endl;
    } else {
      std::cout << std::endl << "******* Verification failed *******"
          << std::endl << std::endl;
    }
    if (drawKeypoints) {
      draw (dataset);
    }

    for (int i = 0; verificationOK && i < 100; ++i) {
      brisk::timing::DebugTimer timerOverall("BRISK overall");
      runpipeline(dataset, datasetfullpath);
      timerOverall.Stop();
    }

    return !verificationOK;
  }
}

void runpipeline(std::vector<DatasetEntry>& dataset,
                 const std::string& briskbasepath) {

  std::cout << "Running the pipeline..." << std::endl;

  /***
   * DETECTION
   */
  //prepare BRISK detector

  cv::Ptr<cv::FeatureDetector> detector =
      new brisk::ScaleSpaceFeatureDetector<brisk::HarrisScoreCalculator>(
          BRISK_octaves, BRISK_uniformityradius, BRISK_absoluteThreshold);

  if (doKeypointDetection || dataset.at(0).keypoints_.empty()) {
    for (std::vector<DatasetEntry>::iterator it = dataset.begin(), end = dataset
        .end(); it != end; ++it) {

      it->setThisAsCurrentEntry();  //now you can query for the current image to add tags to timers etc.

      /*/ hack: benchmark OpenCv Harris:
       cv::Mat dst = cv::Mat::zeros( it->imgGray_.size(), CV_32FC1 );

       /// Detector parameters
       int blockSize = 2;
       int apertureSize = 3;
       double k = 0.04;

       /// Detecting corners
       TimerSwitchableDetail timerdetect0(DatasetEntry::getCurrentImageName() + "_openCV_harris_detect", true);
       timerdetect0.start();
       cv::cornerHarris( it->imgGray_, dst, blockSize, apertureSize, k, cv::BORDER_DEFAULT );
       timerdetect0.stop();
       std::cout << dst.at<float>(10,10)<<std::endl; // scared of dead code eliminations...*/

      brisk::timing::DebugTimer timerdetect(
          DatasetEntry::getCurrentEntry()->path_ + "_detect");
      detector->detect(it->imgGray_, it->keypoints_);
      timerdetect.Stop();

      //Test userdata
      Blob& blob = DatasetEntry::getCurrentEntry()->getBlob("testImage");  //is generated if non existant
      if (!blob.hasverificationData()) {
        blob.setVerificationData(it->imgGray_.data,
                                 it->imgGray_.rows * it->imgGray_.cols);
      } else {
        blob.setCurrentData(it->imgGray_.data,
                            it->imgGray_.rows * it->imgGray_.cols);
      }
    }
  }

  // Extraction.
  cv::Ptr<cv::DescriptorExtractor> descriptorExtractor =
      new brisk::BriskDescriptorExtractor(BRISK_rotationestimation,
                                          BRISK_scaleestimation);
  if (doDescriptorComputation || dataset.at(0).descriptors_.rows == 0) {
    for (std::vector<DatasetEntry>::iterator it = dataset.begin(), end = dataset
        .end(); it != end; ++it) {
      it->setThisAsCurrentEntry();  //now you can query for the current image to add tags to timers etc.
      brisk::timing::Timer timerextract(
          DatasetEntry::getCurrentEntry()->path_ + "_extract");
      descriptorExtractor->compute(it->imgGray_, it->keypoints_,
                                   it->descriptors_);
      timerextract.Stop();
    }
  }

  /***
   * OUTPUT RDTSC TIMING
   */
  brisk::timing::Timing::Print(std::cout);

}

bool runverification(std::vector<DatasetEntry>& current_dataset,
                     std::vector<DatasetEntry>& verification_dataset) {

  /***
   * DO VALIDATION
   */

  if (current_dataset.size() != verification_dataset.size()) {
    throw std::runtime_error("Failed on database number of entries");
    return false;
  }

  bool failed = false;
  //now go through every image
  for (std::vector<DatasetEntry>::iterator it_curr = current_dataset.begin(),
      it_verif = verification_dataset.begin(), end_curr = current_dataset.end(),
      end_verif = verification_dataset.end();
      it_curr != end_curr && it_verif != end_verif; ++it_curr, ++it_verif) {
    it_curr->setThisAsCurrentEntry();  //now you can query for the current image to add tags to timers etc.
    try {
      failed |= (*it_curr != *it_verif);
    } catch (std::exception& e) {
      std::cout << "------" << std::endl << "Failed on image " << it_curr->path_
          << std::endl << "* Error: " << e.what() << "------" << std::endl;
      failed = true;
      if (exitfirstfail)
        throw e;
    }
  }
  return !failed;
}

void draw(std::vector<DatasetEntry>& dataset) {
  /***
   * DRAWING
   */
  cv::namedWindow("Keypoints");
  for (std::vector<DatasetEntry>::iterator it = dataset.begin(), end = dataset
      .end(); it != end; ++it) {
    it->setThisAsCurrentEntry();  //now you can query DatasetEntry::getCurrentImageName() for the current image to add tags to timers etc.
    cv::Mat out;
    cv::drawKeypoints(it->imgGray_, it->keypoints_, out, cv::Scalar::all(-1),
                      cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
    cv::imshow("Keypoints", out);
    cv::waitKey();
  }
}

DatasetEntry* DatasetEntry::current_entry = NULL;
