/*
 * HDR.h
 *
//============================================================================
// Author      : Alexandre Benoit (benoit.alexandre.vision@gmail.com)
// Version     : 0.2
// Copyright   : Alexandre Benoit, LISTIC Lab, december 2011
// Description : HighDynamicRange compression (tone mapping) for image sequences with the help of the Gipsa/Listic's retina in C++, Ansi-style
// Known issues: the input OpenEXR sequences can have bad computed pixels that should be removed
//               => a simple method consists of cutting histogram edges (a slider for this on the UI is provided)
//               => however, in image sequences, this histogramm cut must be done in an elegant way from frame to frame... still not done...
//============================================================================

 */

#ifndef HDR_H_
#define HDR_H_

#include <iostream>
#include <stdio.h>
#include <cstring>

#include "opencv2/opencv.hpp"

// simple procedure for 1D curve tracing
static void drawPlot(const cv::Mat curve, const std::string figureTitle, const int lowerLimit, const int upperLimit)
{
    //std::cout<<"curve size(h,w) = "<<curve.size().height<<", "<<curve.size().width<<std::endl;
    cv::Mat displayedCurveImage = cv::Mat::ones(200, curve.size().height, CV_8U);

    cv::Mat windowNormalizedCurve;
    normalize(curve, windowNormalizedCurve, 0, 200, CV_MINMAX, CV_32F);

    displayedCurveImage = cv::Scalar::all(255); // set a white background
    int binW = cvRound((double)displayedCurveImage.cols/curve.size().height);

    for( int i = 0; i < curve.size().height; i++ )
        rectangle( displayedCurveImage, cv::Point(i*binW, displayedCurveImage.rows),
                cv::Point((i+1)*binW, displayedCurveImage.rows - cvRound(windowNormalizedCurve.at<float>(i))),
                cv::Scalar::all(0), -1, 8, 0 );
    rectangle( displayedCurveImage, cv::Point(0, 0),
            cv::Point((lowerLimit)*binW, 200),
            cv::Scalar::all(128), -1, 8, 0 );
    rectangle( displayedCurveImage, cv::Point(displayedCurveImage.cols, 0),
            cv::Point((upperLimit)*binW, 200),
            cv::Scalar::all(128), -1, 8, 0 );

    cv::imshow(figureTitle, displayedCurveImage);
}

/*
 * objective : get the gray level map of the input image and rescale it to the range [0-255] if rescale0_255=TRUE, simply trunks else
 */
static void rescaleGrayLevelMat(const cv::Mat &inputMat, cv::Mat &outputMat, const float histogramClippingLimit, const bool rescale0_255)
 {
     // adjust output matrix wrt the input size but single channel
     std::cout<<"Input image rescaling with histogram edges cutting (in order to eliminate bad pixels created during the HDR image creation) :"<<std::endl;
     //std::cout<<"=> image size (h,w,channels) = "<<inputMat.size().height<<", "<<inputMat.size().width<<", "<<inputMat.channels()<<std::endl;
     //std::cout<<"=> pixel coding (nbchannel, bytes per channel) = "<<inputMat.elemSize()/inputMat.elemSize1()<<", "<<inputMat.elemSize1()<<std::endl;

     // get min and max values to use afterwards if no 0-255 rescaling is used
     double maxInput, minInput, histNormRescalefactor=1.f;
     double histNormOffset=0.f;
     minMaxLoc(inputMat, &minInput, &maxInput);
     histNormRescalefactor=255.f/(maxInput-minInput);
     histNormOffset=minInput;
     std::cout<<"Hist max,min = "<<maxInput<<", "<<minInput<<" => scale, offset = "<<histNormRescalefactor<<", "<<histNormOffset<<std::endl;
     // rescale between 0-255, keeping floating point values
     cv::Mat normalisedImage;
     cv::normalize(inputMat, normalisedImage, 0.f, 255.f, cv::NORM_MINMAX);
     if (rescale0_255)
        normalisedImage.copyTo(outputMat);
     // extract a 8bit image that will be used for histogram edge cut
     cv::Mat intGrayImage;
     if (inputMat.channels()==1)
     {
         normalisedImage.convertTo(intGrayImage, CV_8U);
     }else
     {
         cv::Mat rgbIntImg;
         normalisedImage.convertTo(rgbIntImg, CV_8UC3);
         cvtColor(rgbIntImg, intGrayImage, CV_BGR2GRAY);
     }

     // get histogram density probability in order to cut values under above edges limits (here 5-95%)... usefull for HDR pixel errors cancellation
     cv::Mat dst, hist;
     int histSize = 256;
     calcHist(&intGrayImage, 1, 0, cv::Mat(), hist, 1, &histSize, 0);
     cv::Mat normalizedHist;

     normalize(hist, normalizedHist, 1.f, 0.f, cv::NORM_L1, CV_32F); // normalize histogram so that its sum equals 1

     // compute density probability
     cv::Mat denseProb=cv::Mat::zeros(normalizedHist.size(), CV_32F);
     denseProb.at<float>(0)=normalizedHist.at<float>(0);
     int histLowerLimit=0, histUpperLimit=0;
     for (int i=1;i<normalizedHist.size().height;++i)
     {
         denseProb.at<float>(i)=denseProb.at<float>(i-1)+normalizedHist.at<float>(i);
         //std::cout<<normalizedHist.at<float>(i)<<", "<<denseProb.at<float>(i)<<std::endl;
         if ( denseProb.at<float>(i)<histogramClippingLimit)
             histLowerLimit=i;
         if ( denseProb.at<float>(i)<1.f-histogramClippingLimit)
             histUpperLimit=i;
     }
     // deduce min and max admitted gray levels
     float minInputValue = (float)histLowerLimit/histSize*255.f;
     float maxInputValue = (float)histUpperLimit/histSize*255.f;

     std::cout<<"=> Histogram limits "
             <<"\n\t"<<histogramClippingLimit*100.f<<"% index = "<<histLowerLimit<<" => normalizedHist value = "<<denseProb.at<float>(histLowerLimit)<<" => input gray level = "<<minInputValue
             <<"\n\t"<<(1.f-histogramClippingLimit)*100.f<<"% index = "<<histUpperLimit<<" => normalizedHist value = "<<denseProb.at<float>(histUpperLimit)<<" => input gray level = "<<maxInputValue
             <<std::endl;
     //drawPlot(denseProb, "input histogram density probability", histLowerLimit, histUpperLimit);
     drawPlot(normalizedHist, "input histogram", histLowerLimit, histUpperLimit);

    if(rescale0_255) // rescale between 0-255 if asked to
    {
        cv::threshold( outputMat, outputMat, maxInputValue, maxInputValue, 2 ); //THRESH_TRUNC, clips values above maxInputValue
        cv::threshold( outputMat, outputMat, minInputValue, minInputValue, 3 ); //THRESH_TOZERO, clips values under minInputValue
        // rescale image range [minInputValue-maxInputValue] to [0-255]
        outputMat-=minInputValue;
        outputMat*=255.f/(maxInputValue-minInputValue);
    }else
    {
        inputMat.copyTo(outputMat);
        // update threshold in the initial input image range
        maxInputValue=(float)((maxInputValue-255.f)/histNormRescalefactor+maxInput);
        minInputValue=(float)(minInputValue/histNormRescalefactor+minInput);
        std::cout<<"===> Input Hist clipping values (max,min) = "<<maxInputValue<<", "<<minInputValue<<std::endl;
        cv::threshold( outputMat, outputMat, maxInputValue, maxInputValue, 2 ); //THRESH_TRUNC, clips values above maxInputValue
        cv::threshold( outputMat, outputMat, minInputValue, minInputValue, 3 ); //
    }
 }

 // basic callback method for interface management
 cv::Mat inputImage;
 cv::Mat imageInputRescaled;
 float globalRescalefactor=1;
 cv::Scalar globalOffset=0;
 int histogramClippingValue;
 static void callBack_rescaleGrayLevelMat(int, void*)
 {
     std::cout<<"Histogram clipping value changed, current value = "<<histogramClippingValue<<std::endl;
    // rescale and process
    inputImage+=globalOffset;
    inputImage*=globalRescalefactor;
    inputImage+=cv::Scalar(50, 50, 50, 50); // WARNING value linked to the hardcoded value (200.0) used in the globalRescalefactor in order to center on the 128 mean value... experimental but... basic compromise
    rescaleGrayLevelMat(inputImage, imageInputRescaled, (float)histogramClippingValue/100.f, true);

 }

 cv::Ptr<cv::Retina> retina;
 int retinaHcellsGain;
 int localAdaptation_photoreceptors, localAdaptation_Gcells;
 static void callBack_updateRetinaParams(int, void*)
 {
     retina->setupOPLandIPLParvoChannel(true, true, (float)(localAdaptation_photoreceptors/200.0), 0.5f, 0.43f, (float)retinaHcellsGain, 1.f, 7.f, (float)(localAdaptation_Gcells/200.0));
 }

 int colorSaturationFactor;
 static void callback_saturateColors(int, void*)
 {
     retina->setColorSaturation(true, (float)colorSaturationFactor);
 }

// loadNewFrame : loads a n image wrt filename parameters. it also manages image rescaling/histogram edges cutting (acts differently at first image i.e. if firstTimeread=true)
static void loadNewFrame(const std::string filenamePrototype, const int currentFileIndex, const bool firstTimeread)
{
     char *currentImageName=NULL;
    currentImageName = (char*)malloc(sizeof(char)*filenamePrototype.size()+10);

    // grab the first frame
    sprintf(currentImageName, filenamePrototype.c_str(), currentFileIndex);

     //////////////////////////////////////////////////////////////////////////////
     // checking input media type (still image, video file, live video acquisition)
     std::cout<<"RetinaDemo: reading image : "<<currentImageName<<std::endl;
     // image processing case
     // declare the retina input buffer... that will be fed differently in regard of the input media
     inputImage = cv::imread(currentImageName, -1); // load image in RGB mode
     std::cout<<"=> image size (h,w) = "<<inputImage.size().height<<", "<<inputImage.size().width<<std::endl;
     if (inputImage.empty())
     {
        help("could not load image, program end");
            return;;
         }

    // rescaling/histogram clipping stage
    // rescale between 0 and 1
    // TODO : take care of this step !!! maybe disable of do this in a nicer way ... each successive image should get the same transformation... but it depends on the initial image format
    double maxInput, minInput;
    minMaxLoc(inputImage, &minInput, &maxInput);
    std::cout<<"ORIGINAL IMAGE pixels values range (max,min) : "<<maxInput<<", "<<minInput<<std::endl;

    if (firstTimeread)
    {
        /* the first time, get the pixel values range and rougthly update scaling value
        in order to center values around 128 and getting a range close to [0-255],
        => actually using a little less in order to let some more flexibility in range evolves...
        */
        double maxInput1, minInput1;
        minMaxLoc(inputImage, &minInput1, &maxInput1);
        std::cout<<"FIRST IMAGE pixels values range (max,min) : "<<maxInput1<<", "<<minInput1<<std::endl;
        globalRescalefactor=(float)(50.0/(maxInput1-minInput1)); // less than 255 for flexibility... experimental value to be carefull about
        double channelOffset = -1.5*minInput;
        globalOffset= cv::Scalar(channelOffset, channelOffset, channelOffset, channelOffset);
    }
    // call the generic input image rescaling callback
    callBack_rescaleGrayLevelMat(1,NULL);
}




#endif /* HDR_H_ */
