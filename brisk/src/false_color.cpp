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
#include <falsecolor.h>
#include <cmath>
#include <vector>
#include <iostream>
#include <opencv2/core/core.hpp>

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

void convertFalseColor(const cv::Mat& srcmat, cv::Mat& dstmat, palette::palettetypes paltype)
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

}

