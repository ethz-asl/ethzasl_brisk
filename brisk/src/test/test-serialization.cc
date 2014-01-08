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

#include <limits>

#include <glog/logging.h>
#include <gtest/gtest.h>
#include <opencv2/core/core.hpp>
#include "./bench-ds.h"

#ifndef TEST
#define TEST(a, b) int Test_##a##_##b()
#endif

template<typename TYPE>
void SetRandom(
    TYPE* value,
    typename std::enable_if<std::is_integral<TYPE>::value>::type* = 0) {
  CHECK_NOTNULL(value);
  *value = rand() % std::numeric_limits<TYPE>::max();
}

template<typename TYPE>
void SetRandom(
    TYPE* value,
    typename std::enable_if<std::is_floating_point<TYPE>::value>::type* = 0) {
  CHECK_NOTNULL(value);
  *value = rand() / RAND_MAX;
}

template<typename TYPEA, typename TYPEB>
void SetRandom(std::pair<TYPEA, TYPEB>* value) {
  CHECK_NOTNULL(value);
  SetRandom(&value->first);
  SetRandom(&value->second);
}

template<typename TYPE>
void SetRandom(std::vector<TYPE>* value) {
  CHECK_NOTNULL(value);
  int size;
  SetRandom(&size);
  size %= 20;
  value->resize(size);
  for (TYPE& entry : *value) {
    SetRandom(entry);
  }
}

template<typename TYPEA, typename TYPEB>
void SetRandom(std::map<TYPEA, TYPEB>* value) {
  CHECK_NOTNULL(value);
  int size;
  SetRandom(&size);
  size %= 20;
  for (size_t i = 0; i < size; ++i) {
    std::pair<TYPEA, TYPEB> entry;
    SetRandom(entry);
    value->insert(entry);
  }
}

void SetRandom(cv::Mat* value) {
  CHECK_NOTNULL(value);
  int rows;
  SetRandom(&rows);
  rows %= 20;
  int cols;
  SetRandom(&cols);
  cols %= 20;
  value->create(rows, cols, CV_8UC1);
  // We assume using random memory is random enough as matrix content.
}

template<typename TYPE>
void SetRandom(cv::Point_<TYPE>* value) {
  CHECK_NOTNULL(value);
  SetRandom(&value->x);
  SetRandom(&value->y);
}

void SetRandom(cv::KeyPoint* value) {
  CHECK_NOTNULL(value);
  SetRandom(&value->angle);
  SetRandom(&value->class_id);
  SetRandom(&value->octave);
  SetRandom(&value->pt);
  SetRandom(&value->response);
  SetRandom(&value->size);
}

template<typename TYPE>
bool RunSerializationTest() {
  TYPE loaded_value;
  std::string filename = "serialization_file_" +
      std::string(typeid(TYPE).name());

}

TEST(Serialization, Integer) {
  RunSerializationTest<int>();
}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
