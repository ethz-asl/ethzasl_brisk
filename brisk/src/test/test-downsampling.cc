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

#if HAVE_GLOG
#include <glog/logging.h>
#else
#include <brisk/glog_replace.h>
#endif
#include <gtest/gtest.h>

#include <opencv2/highgui/highgui.hpp>

#include <brisk/internal/image-down-sampling.h>

#ifndef TEST
#define TEST(a, b) int Test_##a##_##b()
#endif

void CheckImageSame(const unsigned char* lhs, const unsigned char* rhs,
                    size_t rows, size_t cols) {
  CHECK_NOTNULL(lhs);
  CHECK_NOTNULL(rhs);
  unsigned int errors = 0;
  for (size_t row = 0; row < rows; ++row) {
    for (size_t col = 0; col < rows; ++col) {
      EXPECT_EQ(lhs[row * cols + col], rhs[row * cols + col])
      << "Difference at row: " << row << " col: " << col;
      if (lhs[row * cols + col] != rhs[row * cols + col]) {
        ++errors;
        if (errors > 10) {
          ASSERT_TRUE(false) << "Too many errors";
        }
      }
    }
  }
}

void PlainHalfSample(const unsigned char* src, unsigned char* dst,
                     size_t src_rows, size_t src_cols) {
  CHECK_NOTNULL(src);
  CHECK_NOTNULL(dst);
  const size_t dst_rows = src_rows / 2;
  const size_t dst_cols = src_cols / 2;

  for (size_t row = 0; row < dst_rows; ++row) {
     for (size_t col = 0; col < dst_cols; ++col) {
       unsigned int value11 = src[(row * 2) * src_cols + (col * 2)];
       unsigned int value12 = src[(row * 2) * src_cols + (col * 2 + 1)];
       unsigned int value21 = src[(row * 2 + 1) * src_cols + (col * 2)];
       unsigned int value22 = src[(row * 2 + 1) * src_cols + (col * 2 + 1)];

       // Also do horizontal pairwise add like vectorized versions.
       // We also add the 1s to force rounding 'up'.
       dst[row * dst_cols + col] = std::min(
           ((value11 + 1 + value21) / 2 +
               (value12 + 1 + value22) / 2 + 1) / 2, 255u);
     }
  }
}

void PlainTwoThirdSample(const unsigned char* src, unsigned char* dst,
                         size_t src_rows, size_t src_cols) {
  CHECK_NOTNULL(src);
  CHECK_NOTNULL(dst);
  const size_t dst_rows = src_rows / 3 * 2;
  const size_t dst_cols = src_cols / 3 * 2;

  for (size_t row = 0; row < dst_rows; row += 2) {
     for (size_t col = 0; col < dst_cols; col += 2) {
       const uint16_t A1 = src[(row / 2 * 3 + 0) * src_cols +
                               (col / 2 * 3 + 0)];
       const uint16_t A2 = src[(row / 2 * 3 + 0) * src_cols +
                               (col / 2 * 3 + 1)];
       const uint16_t A3 = src[(row / 2 * 3 + 0) * src_cols +
                               (col / 2 * 3 + 2)];
       const uint16_t B1 = src[(row / 2 * 3 + 1) * src_cols +
                               (col / 2 * 3 + 0)];
       const uint16_t B2 = src[(row / 2 * 3 + 1) * src_cols +
                               (col / 2 * 3 + 1)];
       const uint16_t B3 = src[(row / 2 * 3 + 1) * src_cols +
                               (col / 2 * 3 + 2)];
       const uint16_t C1 = src[(row / 2 * 3 + 2) * src_cols +
                               (col / 2 * 3 + 0)];
       const uint16_t C2 = src[(row / 2 * 3 + 2) * src_cols +
                               (col / 2 * 3 + 1)];
       const uint16_t C3 = src[(row / 2 * 3 + 2) * src_cols +
                               (col / 2 * 3 + 2)];

       dst[row * dst_cols + col] = static_cast<unsigned char>(
               ((4 * A1 + 2 * (A2 + B1) + B2) / 9) & 0x00FF);
       dst[row * dst_cols + col + 1] = static_cast<unsigned char>(
               ((4 * A3 + 2 * (A2 + B3) + B2) / 9) & 0x00FF);
       dst[(row + 1) * dst_cols + col] = static_cast<unsigned char>(
           ((4 * C1 + 2 * (C2 + B1) + B2) / 9) & 0x00FF);
       dst[(row + 1) * dst_cols + col + 1] = static_cast<unsigned char>(
           ((4 * C3 + 2 * (C2 + B3) + B2) / 9) & 0x00FF);
     }
  }
}

TEST(Brisk, HalfSample) {
  std::string imagepath = "./src/test/test_data/img1.ppm";
  cv::Mat imgRGB = cv::imread(imagepath);
  cv::Mat src_img;
  cv::cvtColor(imgRGB, src_img, CV_BGR2GRAY);

  const static int source_cols = src_img.cols;
  const static int source_rows = src_img.rows;
  const static int dst_cols = source_cols / 2;
  const static int dst_rows = source_rows / 2;
  cv::Mat dst_img_a(dst_rows, dst_cols, CV_8UC1);
  cv::Mat dst_img_b(dst_rows, dst_cols, CV_8UC1);

  brisk::Halfsample8(src_img, dst_img_a);
  PlainHalfSample(src_img.data, dst_img_b.data, source_rows, source_cols);

  CheckImageSame(dst_img_a.data, dst_img_b.data, dst_rows, dst_cols);
}

TEST(Brisk, TwoThirdSample) {
  std::string imagepath = "./src/test/test_data/img1.ppm";
  cv::Mat imgRGB = cv::imread(imagepath);
  cv::Mat src_img;
  cv::cvtColor(imgRGB, src_img, CV_BGR2GRAY);

  const static int source_cols = src_img.cols;
  const static int source_rows = src_img.rows;
  const static int dst_cols = source_cols / 3 * 2;
  const static int dst_rows = source_rows / 3 * 2;
  cv::Mat dst_img_a(dst_rows, dst_cols, CV_8UC1);
  cv::Mat dst_img_b(dst_rows, dst_cols, CV_8UC1);

  brisk::Twothirdsample8(src_img, dst_img_a);
  PlainTwoThirdSample(src_img.data, dst_img_b.data, source_rows, source_cols);

  cv::namedWindow("DownSample", cv::WINDOW_AUTOSIZE);
  cv::imshow("DownSample", dst_img_b);
  cv::waitKey(0);

  CheckImageSame(dst_img_a.data, dst_img_b.data, dst_rows, dst_cols);
}
int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
