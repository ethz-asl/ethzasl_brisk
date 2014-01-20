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
#include <list>
#include <string>
#include <vector>

#include <brisk/brisk-opencv.h>
#if HAVE_GLOG
#include <glog/logging.h>
#else
#include <brisk/glog_replace.h>
#endif
#include <gtest/gtest.h>

#include "./bench-ds.h"
#include "./serialization.h"

#ifndef TEST
#define TEST(a, b) int Test_##a##_##b()
#endif

template<typename TYPE>
void SetRandom(
    TYPE* value, typename std::enable_if<std::is_integral<TYPE>::value>::type* =
        0) {
  CHECK_NOTNULL(value);
  *value = rand() % std::numeric_limits < TYPE > ::max();
}

template<typename TYPE>
void SetRandom(
    TYPE* value,
    typename std::enable_if<std::is_floating_point<TYPE>::value>::type* = 0) {
  CHECK_NOTNULL(value);
  *value = static_cast<TYPE>(rand()) / RAND_MAX;
}

void SetRandom(uint32_t* value) {
  CHECK_NOTNULL(value);
  *value = rand() % std::numeric_limits < uint32_t > ::max();
}

void SetRandom(std::string* value) {
  CHECK_NOTNULL(value);
  size_t size;
  SetRandom(&size);
  size = size % 20 + 1;
  *value = "0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz";
  int pos;
  while (value->size() != size) {
    pos = ((rand() % (value->size() - 1)));
    value->erase(pos, 1);
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
  // Just using the random memory which is contained in the matrix.
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
  size_t size;
  SetRandom(&size);
  size %= 20;
  value->resize(size);
  for (TYPE& entry : *value) {
    SetRandom(&entry);
  }
}

template<typename TYPE>
void SetRandom(std::list<TYPE>* value) {
  CHECK_NOTNULL(value);
  size_t size;
  SetRandom(&size);
  size %= 20;
  value->resize(size);
  for (TYPE& entry : *value) {
    SetRandom(&entry);
  }
}

template<typename TYPEA, typename TYPEB>
void SetRandom(std::map<TYPEA, TYPEB>* value) {
  CHECK_NOTNULL(value);
  size_t size;
  SetRandom(&size);
  size %= 20;
  for (size_t i = 0; i < size; ++i) {
    std::pair<TYPEA, TYPEB> entry;
    SetRandom(&entry);
    value->insert(entry);
  }
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
void AssertEqual(const TYPE& lhs, const TYPE& rhs) {
  ASSERT_EQ(lhs, rhs);
}

template<typename TYPE>
void AssertNotEqual(const TYPE& lhs, const TYPE& rhs) {
  ASSERT_NE(lhs, rhs);
}

template<>
void AssertEqual(const cv::Mat& lhs, const cv::Mat& rhs) {
  ASSERT_EQ(lhs.size(), rhs.size());
  for (int index = 0, size = lhs.rows * lhs.cols; index < size; ++index) {
    CHECK_EQ(lhs.at<unsigned char>(index), rhs.at<unsigned char>(index))
        << "Failed matrix equality for index " << index;
  }
}

template<>
void AssertNotEqual(const cv::Mat& lhs, const cv::Mat& rhs) {
  bool is_same = true;
  is_same = is_same && lhs.size() == rhs.size();
  if (is_same) {
    for (int index = 0, size = lhs.rows * lhs.cols; index < size; ++index) {
      if (lhs.at<unsigned char>(index) != rhs.at<unsigned char>(index)) {
        is_same = false;
      }
    }
  }
  ASSERT_FALSE(is_same);
}

template<typename TYPE>
void RunSerializationTest() {
  TYPE saved_value, loaded_value;
  std::string filename = "src/test/test_data/tmp/serialization_file_"
      + std::string(typeid(TYPE).name()) + "_tmp";
  {  // Scoping to flush and close file.
    std::ofstream ofs(filename);
    SetRandom(&saved_value);
    SetRandom(&loaded_value);
    AssertNotEqual(saved_value, loaded_value);
    serialization::Serialize(saved_value, &ofs);
  }
  std::ifstream ifs(filename);
  serialization::DeSerialize(&loaded_value, &ifs);
  AssertEqual(saved_value, loaded_value);
}

TEST(Serialization, Char) {
  RunSerializationTest<char>();
}

TEST(Serialization, UnsignedChar) {
  RunSerializationTest<unsigned char>();
}

TEST(Serialization, Integer) {
  RunSerializationTest<int>();
}

TEST(Serialization, UnsignedInteger) {
  RunSerializationTest<unsigned int>();
}

TEST(Serialization, Float) {
  RunSerializationTest<float>();
}

TEST(Serialization, Double) {
  RunSerializationTest<double>();
}

TEST(Serialization, String) {
  RunSerializationTest<std::string>();
}

TEST(Serialization, PairDoubleInt) {
  RunSerializationTest<std::pair<double, int> >();
}

TEST(Serialization, PairDoubleDouble) {
  RunSerializationTest<std::pair<double, double> >();
}

TEST(Serialization, PairStringDouble) {
  RunSerializationTest<std::pair<std::string, double> >();
}

TEST(Serialization, VectorDouble) {
  RunSerializationTest<std::vector<double> >();
}

TEST(Serialization, VectorString) {
  RunSerializationTest<std::vector<std::string> >();
}

TEST(Serialization, VectorPairStringDouble) {
  RunSerializationTest<std::vector<std::pair<std::string, double> > >();
}

TEST(Serialization, ListDouble) {
  RunSerializationTest<std::list<double> >();
}

TEST(Serialization, ListString) {
  RunSerializationTest<std::list<std::string> >();
}

TEST(Serialization, ListPairStringDouble) {
  RunSerializationTest<std::list<std::pair<std::string, double> > >();
}

TEST(Serialization, MapIntDouble) {
  RunSerializationTest<std::map<int, double> >();
}

TEST(Serialization, MapDoubleString) {
  RunSerializationTest<std::map<double, std::string> >();
}

TEST(Serialization, MapStringDouble) {
  RunSerializationTest<std::map<std::string, double> >();
}

TEST(Serialization, MapStringString) {
  RunSerializationTest<std::map<std::string, std::string> >();
}

TEST(Serialization, CvMat) {
  RunSerializationTest<cv::Mat>();
}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
