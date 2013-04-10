/*

 Copyright (c) 2013, Simon Lynen, ASL, ETH Zurich, Switzerland
 You can contact the author at <slynen at ethz dot ch>

 All rights reserved.

 Redistribution and use in source and binary forms, with or without
 modification, are permitted provided that the following conditions are met:
 * Redistributions of source code must retain the above copyright
 notice, this list of conditions and the following disclaimer.
 * Redistributions in binary form must reproduce the above copyright
 notice, this list of conditions and the following disclaimer in the
 documentation and/or other materials provided with the distribution.
 * Neither the name of ETHZ-ASL nor the
 names of its contributors may be used to endorse or promote products
 derived from this software without specific prior written permission.

 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 DISCLAIMED. IN NO EVENT SHALL ETHZ-ASL BE LIABLE FOR ANY
 DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

 */
#include <flir/falsecolor.h>
#include <cmath>
#include <vector>
#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <boost/lexical_cast.hpp>
#include <iomanip>
#include "opencv2/contrib/retina.hpp"
#include <ros/package.h>

#define DEG2RAD 0.01745329
palette GetPalette(palette::palettetypes pal)
{
  palette ret;

  int i, r, g, b;
  float f;

  switch (pal)
  {
    case palette::Linear_red_palettes:
      /*
       * Linear red palettes.
       */
      for (i = 0; i < 256; i++)
      {
        ret.colors[i].rgbBlue = 0;
        ret.colors[i].rgbGreen = 0;
        ret.colors[i].rgbRed = i;
      }
      break;
    case palette::GammaLog_red_palettes:
      /*
       * GammaLog red palettes.
       */
      for (i = 0; i < 256; i++)
      {
        f = log10(pow((i / 255.0), 1.0) * 9.0 + 1.0) * 255.0;
        ret.colors[i].rgbBlue = 0;
        ret.colors[i].rgbGreen = 0;
        ret.colors[i].rgbRed = f;
      }
      break;
    case palette::Inversion_red_palette:
      /*
       * Inversion red palette.
       */
      for (i = 0; i < 256; i++)
      {
        ret.colors[i].rgbBlue = 0;
        ret.colors[i].rgbGreen = 0;
        ret.colors[i].rgbRed = 255 - i;
      }
      break;
    case palette::Linear_palettes:
      /*
       * Linear palettes.
       */
      for (i = 0; i < 256; i++)
      {
        ret.colors[i].rgbBlue = ret.colors[i].rgbGreen = ret.colors[i].rgbRed = i;
      }
      break;
    case palette::GammaLog_palettes:
      /*
       * GammaLog palettes.
       */
      for (i = 0; i < 256; i++)
      {
        f = log10(pow((i / 255.0), 1.0) * 9.0 + 1.0) * 255.0;
        ret.colors[i].rgbBlue = ret.colors[i].rgbGreen = ret.colors[i].rgbRed = f;
      }
      break;
    case palette::Inversion_palette:
      /*
       * Inversion palette.
       */
      for (i = 0; i < 256; i++)
      {
        ret.colors[i].rgbBlue = ret.colors[i].rgbGreen = ret.colors[i].rgbRed = 255 - i;
      }
      break;
    case palette::False_color_palette1:
      /*
       * False color palette #1.
       */
      for (i = 0; i < 256; i++)
      {
        r = (sin((i / 255.0 * 360.0 - 120.0 > 0 ? i / 255.0 * 360.0 - 120.0 : 0) * DEG2RAD) * 0.5 + 0.5) * 255.0;
        g = (sin((i / 255.0 * 360.0 + 60.0) * DEG2RAD) * 0.5 + 0.5) * 255.0;
        b = (sin((i / 255.0 * 360.0 + 140.0) * DEG2RAD) * 0.5 + 0.5) * 255.0;
        ret.colors[i].rgbBlue = b;
        ret.colors[i].rgbGreen = g;
        ret.colors[i].rgbRed = r;
      }
      break;
    case palette::False_color_palette2:
      /*
       * False color palette #2.
       */
      for (i = 0; i < 256; i++)
      {
        r = (sin((i / 255.0 * 360.0 + 120.0) * DEG2RAD) * 0.5 + 0.5) * 255.0;
        g = (sin((i / 255.0 * 360.0 + 240.0) * DEG2RAD) * 0.5 + 0.5) * 255.0;
        b = (sin((i / 255.0 * 360.0 + 0.0) * DEG2RAD) * 0.5 + 0.5) * 255.0;
        ret.colors[i].rgbBlue = b;
        ret.colors[i].rgbGreen = g;
        ret.colors[i].rgbRed = r;
      }
      break;
    case palette::False_color_palette3:
      /*
       * False color palette #3.
       */
      for (i = 0; i < 256; i++)
      {
        r = (sin((i / 255.0 * 360.0 + 240.0) * DEG2RAD) * 0.5 + 0.5) * 255.0;
        g = (sin((i / 255.0 * 360.0 + 0.0) * DEG2RAD) * 0.5 + 0.5) * 255.0;
        b = (sin((i / 255.0 * 360.0 + 120.0) * DEG2RAD) * 0.5 + 0.5) * 255.0;
        ret.colors[i].rgbBlue = b;
        ret.colors[i].rgbGreen = g;
        ret.colors[i].rgbRed = r;
      }
      break;

    case palette::False_color_palette4:
      /*
       * False color palette #4. Matlab JET
       */

      enum
      {
        nsep = 64, nvals = 192, n = 256
      };

      std::vector<double> vals;
      vals.resize(nvals, 0);

      int idx = 0;
      for (int i = 0; i < nsep; ++i)
      {
        vals.at(idx++) = (i / (double)nsep);
      }

      for (int i = 0; i < nsep; ++i){
        vals.at(idx + i) = 1.;
      }

      idx += nsep;
      for (int i = nsep - 1; i >= 0; --i)
      {
        vals.at(idx++) = i / (double)nsep;
      }

      std::vector<int> r;
      r.resize(nvals);
      std::vector<int> g;
      g.resize(nvals);
      std::vector<int> b;
      b.resize(nvals);
      for (std::size_t i = 0; i < nvals; ++i)
      {
        g.at(i) = ceil(nsep / 2) - 1 + i;
        r.at(i) = g.at(i) + nsep;
        b.at(i) = g.at(i) - nsep;
      }

      int idxr = 0;
      int idxg = 0;

      for (int i = 0; i < nvals; ++i)
      {
        if (r.at(i) >= 0 && r.at(i) < n)
          ret.colors[r.at(i)].rgbRed = vals.at(idxr++) * 255.;

        if (g.at(i) >= 0 && g.at(i) < n)
          ret.colors[g.at(i)].rgbGreen = vals.at(idxg++) * 255.;
      }

      int idxb = 0;
      int cntblue = 0;
      for (int i = 0; i < nvals; ++i)
      {
        if (b.at(i) >= 0 && b.at(i) < n)
          cntblue++;
      }

      for (int i = 0; i < nvals; ++i)
      {
        if (b.at(i) >= 0 && b.at(i) < n)
          ret.colors[b.at(i)].rgbBlue = vals.at(nvals - 1 - cntblue + idxb++) * 255.;
      }
      break;
  }
  return ret;
}
#undef DEG2RAD

void convertFalseColor(const cv::Mat& srcmat, cv::Mat& dstmat, palette::palettetypes paltype, bool drawlegend, double mintemp, double maxtemp)
{

  palette pal = GetPalette(paltype);

  dstmat.create(srcmat.rows, srcmat.cols, CV_8UC3);

  cv::Size sz = srcmat.size();
  const unsigned char* src = srcmat.data;
  unsigned char* dst = dstmat.data;


  if (srcmat.isContinuous() && dstmat.isContinuous())
  {
    sz.width *= sz.height;
    sz.height = 1;
  }

  for (int i = 0; i < sz.width; ++i)
  {
    for (int j = 0; j < sz.height; ++j)
    {
      int idx = j * sz.width + i;
      uint8_t val = src[idx];
      dst[idx * dstmat.channels() + 0] = pal.colors[val].rgbBlue;
      dst[idx * dstmat.channels() + 1] = pal.colors[val].rgbGreen;
      dst[idx * dstmat.channels() + 2] = pal.colors[val].rgbRed;
    }
  }

  //draw a legend if true Temperatures
  if(drawlegend){

    //get min max to scale the legend
    double max_val;
    double min_val;
    cv::minMaxIdx(srcmat, &min_val, &max_val);

    enum{
      legenddiscretization = 5,
      legendnumbers = 5,
      legendwidth = 5,
      x_0 = 20,
      y_0 = 10,
    };
    double stepsize;

    //    std::cout<<"mintemp "<<mintemp<<" maxtemp "<<maxtemp<<std::endl;

    //draw legend color bar
    for(int i = y_0 ; i < dstmat.rows - y_0 ; ++i){
      int py = dstmat.rows - i;
      int val = (i - y_0) / (double)(dstmat.rows - y_0 * 2) * 255.;
      cv::rectangle(dstmat, cv::Point(x_0, py), cv::Point(x_0 + legendwidth, py + 1),
                    CV_RGB(pal.colors[val].rgbRed, pal.colors[val].rgbGreen, pal.colors[val].rgbBlue ), -1);
    }

    //draw temp tick labels
    stepsize = (dstmat.rows - y_0 * 2) / (double)legendnumbers;
    for(int i = 0 ; i <= legendnumbers ; ++i){
      int py = y_0 + (legendnumbers - i) * stepsize + 5; //bottom up
      double tempval = (mintemp - 273.15) + i * (maxtemp - mintemp) / (double)legendnumbers;
      std::stringstream ss;
      ss<<std::setprecision(2)<<tempval<<" C";
      cv::putText(dstmat, ss.str(), cv::Point(x_0 + 20, py), CV_FONT_HERSHEY_SIMPLEX, 0.4, CV_RGB(255,255,255), 1);
    }

    //draw ticks into legends
    stepsize = (dstmat.rows - y_0 * 2) / (double)legenddiscretization;
    for(int i = 0 ; i <= legenddiscretization ; ++i){
      int py = y_0 + (legenddiscretization - i) * stepsize; //bottom up
      cv::line(dstmat, cv::Point(x_0 - 2, py), cv::Point(x_0 + legendwidth + 2, py), CV_RGB(255,255,255), 1);
    }
  }

}

converter_16_8::converter_16_8()
{
  min_ = std::numeric_limits<uint16_t>::max();
  max_ = 0;
  firstframe_ = true;
}

converter_16_8::~converter_16_8()
{
  delete inst_;
  inst_ = NULL;
}

double converter_16_8::getMin(){
  return min_;
}
double converter_16_8::getMax(){
  return max_;
}
void converter_16_8::toneMapping(const cv::Mat& img16, cv::Mat& img8){
  if(!retina_){
    retina_.reset(new cv::Retina(img16.size(), false));
    retina_->setup(ros::package::getPath("brisk") + "/include/flir/retina_params");
  }
  retina_->run(img16);
  retina_->getParvo(img8);
}

//adjustment -3 for slightly wrong fit
double powerToK4(double power){
  double slope = 2.58357167114001779457e-07;
  double y_0 = 2.26799217314804718626e+03;
return sqrt(sqrt(((double)power - y_0) / slope)) - 3;
}


void converter_16_8::convert_to8bit(const cv::Mat& img16, cv::Mat& img8, bool doTempConversion)
{
  if(img8.empty()){ //make an image if the user has provided nothing
    img8.create(cvSize(img16.cols, img16.rows), CV_8UC1);
  }

  double min = std::numeric_limits<uint16_t>::max();
  double max = 0;

  //make a histogram of intensities
  typedef std::map<double, int> hist_t;
  hist_t hist;

  double bucketwidth = 2.; //bucketwidth in degrees K

  for (int i = 0; i < img16.cols; ++i)
  {
    for (int j = 0; j < img16.rows; ++j)
    {
      double power = img16.at<uint16_t>(j, i);
      double temp;
      if(doTempConversion){
        temp = powerToK4(power);
      }else{
        temp = power;
      }
      temp = round(temp / bucketwidth) * bucketwidth;
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

  //  std::cout<<"min: "<<min-273.15<<" max: "<<max-273.15<<" sm: min: "<<min_-273.15<<" max: "<<max_-273.15<<std::endl;

  //exp smoothing
  double expsm = 0.95;
  min_ = expsm * min_ + (1. - expsm) * min;
  max_ = expsm * max_ + (1. - expsm) * max;

  for (int i = 0; i < img16.cols; ++i)
  {
    for (int j = 0; j < img16.rows; ++j)
    {
      double temp;
      if(doTempConversion){
        temp = powerToK4(img16.at<uint16_t>(j, i));
      }else{
        temp = (double)(img16.at<uint16_t>(j, i));
      }

      int val = (((temp - min_) / (max_ - min_)) * 255);

      val = val > std::numeric_limits<uint8_t>::max() ? std::numeric_limits<uint8_t>::max() : val < 0 ? 0 : val; //saturate
      img8.at<uint8_t>(j, i) = (uint8_t)val;
    }
  }

  firstframe_ = false;
}

converter_16_8* converter_16_8::inst_ = NULL;
