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

#ifndef BENCHDS_H_
#define BENCHDS_H_
#include <fstream>
#include <memory>
#include <type_traits>

#include <brisk/brisk.h>
#include <brisk/internal/hamming-sse.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

typedef unsigned char imagedata_T;
class Blob;
class DatasetEntry;

#define EXPECTSAMETHROW(THIS, OTHER, MEMBER) \
    do { if (THIS.MEMBER != OTHER.MEMBER) { \
      std::stringstream ss; \
      ss<<  "Failed on " << #MEMBER << ": "<< THIS.MEMBER \
      << " other " << OTHER.MEMBER << " at " << __PRETTY_FUNCTION__ \
      << " Line: " << __LINE__ << std::endl; \
      CHECK(false) << ss.str(); \
      return false;\
    } }\
    while (0);

#define CHECKCVKEYPOINTMEMBERSAME(THIS, OTHER, MEMBER, KEYPOINTIDX) \
    do { if (THIS.MEMBER != OTHER.MEMBER) { \
      std::stringstream ss; \
      ss <<  "For Keypoint " << KEYPOINTIDX << " failed on " \
      << #MEMBER << ": "<< THIS.MEMBER << \
      " other " << OTHER.MEMBER << \
      " at " << __PRETTY_FUNCTION__ << " Line: " << __LINE__ << std::endl; \
      ss << "this / other: "\
      << std::endl << "pt.x:\t" << THIS.pt.x << "\t" << OTHER.pt.x << std::endl\
      << std::endl << "pt.y:\t" << THIS.pt.y << "\t" << OTHER.pt.y << std::endl\
      << std::endl << "class_id:\t" << THIS.class_id << "\t" << OTHER.class_id \
      << std::endl \
      << std::endl << "octave:\t" << THIS.octave<<"\t" << OTHER.octave \
      << std::endl << "response:\t" << THIS.response << "\t" << OTHER.response \
      << std::endl << "size:\t" << THIS.size << "\t" << OTHER.size \
      << std::endl; \
      CHECK(false) << ss.str(); \
      return false;\
    } }\
    while (0);

#define CHECKCVKEYPOINTANGLESAME(THIS, OTHER, MEMBER, KEYPOINTIDX) \
    do { if ((std::abs(THIS.MEMBER - OTHER.MEMBER) > 180 ? \
      (360 - std::abs(THIS.MEMBER-OTHER.MEMBER)) : \
      std::abs(THIS.MEMBER - OTHER.MEMBER) ) > 0.01) { \
      std::stringstream ss; \
      ss <<  "For Keypoint " << KEYPOINTIDX \
      << " failed on angle" << ": "<< THIS.MEMBER\
      << " other " << OTHER.MEMBER \
      << " at " << __PRETTY_FUNCTION__ << " Line: " << __LINE__ << std::endl; \
      ss << "this / other: "\
      << std::endl << "pt.x:\t" << THIS.pt.x << "\t" << OTHER.pt.x \
      << std::endl << "pt.y:\t" << THIS.pt.y << "\t" << OTHER.pt.y \
      << std::endl << "class_id:\t" << THIS.class_id << "\t" << OTHER.class_id \
      << std::endl << "octave:\t" << THIS.octave << "\t" << OTHER.octave \
      << std::endl << "response:\t" << THIS.response<<"\t" << OTHER.response \
      << std::endl << "size:\t" << THIS.size << "\t" << OTHER.size \
      << std::endl;\
      CHECK(false) << ss.str(); \
      return false;\
    } }\
    while (0);

std::string descriptortoString(const __m128i * d, int num128Words) {
  std::stringstream ss;
  ss << "[";
  const unsigned int __attribute__ ((__may_alias__))* data =
      reinterpret_cast<const unsigned int __attribute__ ((__may_alias__))*>(d);
  for (int bit = 0; bit < num128Words * 128; ++bit) {
    ss << (data[bit >> 5] & (1 << (bit & 31)) ? "1 " : "0 ");
  };
  return ss.str();
}

struct Blob {
  friend void Serialize(const Blob& value, std::ofstream* out);
  friend void DeSerialize(Blob* value, std::ifstream* in);
 private:
  std::unique_ptr<unsigned char[]> current_data_;
  std::unique_ptr<unsigned char[]> verification_data_;
  uint32_t size_;
 public:
  Blob() {
    size_ = 0;
  }
  Blob(const Blob& other) {
    size_ = other.size_;
    if (size_) {
      verification_data_.reset(new unsigned char[size_]);
      memcpy(verification_data_.get(), other.verification_data_.get(), size_);
      if (other.current_data_) {
        current_data_.reset(new unsigned char[size_]);
        memcpy(current_data_.get(), other.current_data_.get(), size_);
      }
    }
  }

  uint32_t size() const {
    return size_;
  }

  int checkCurrentDataSameAsVerificationData() const {
    if (!size_) {
      return 0;
    }
    if (!current_data_) {  //if we don't have data to compare, we assume the user has not set it
      std::cout
          << "You asked me to verify userdata, but the current_data is not set"
          << std::endl;
      return 0;
    }
    return memcmp(current_data_.get(), verification_data_.get(), size_);
  }

  bool hasverificationData() {
    return static_cast<bool>(verification_data_);
  }

  void setVerificationData(const unsigned char* data, uint32_t size) {
    size_ = size;
    verification_data_.reset(new unsigned char[size_]);
    memcpy(verification_data_.get(), data, size_);
  }

  void setCurrentData(const unsigned char* data, uint32_t size) {
    if (size != size_) {
      CHECK(false) << "You set the current data to a different length than "
          "the verification data. This will fail the verification."
          "Use both times the same lenght.";
    }
    current_data_.reset(new unsigned char[size]);
    memcpy(current_data_.get(), data, size);
  }

  const unsigned char* verificationData() {
    if (verification_data_) {
      return verification_data_.get();
    } else {
      return NULL;
    }
  }

  const unsigned char* currentData() {
    if (current_data_) {
      return current_data_.get();
    } else {
      return NULL;
    }
  }
};

struct DatasetEntry {
  friend void DeSerialize(DatasetEntry* value, std::ifstream* in);
  friend void Serialize(const DatasetEntry& value, std::ofstream* out);
 private:
  std::map<std::string, Blob> userdata_;
  std::string path_;
  cv::Mat imgGray_;
  std::vector<cv::KeyPoint> keypoints_;
  cv::Mat descriptors_;

 public:
  DatasetEntry() = default;

  std::string shortname() {
    int idx = path_.find_last_of("/\\") + 1;
    return path_.substr(idx, path_.length() - idx);
  }

  const std::string& GetPath() const {
    return path_;
  }

  const cv::Mat& GetImage() const {
    return imgGray_;
  }

  const std::vector<cv::KeyPoint>& GetKeyPoints() const {
    return keypoints_;
  }

  const cv::Mat& GetDescriptors() const {
    return descriptors_;
  }

  std::string* GetPathMutable() {
    return &path_;
  }

  cv::Mat* GetImgMutable() {
    return &imgGray_;
  }

  std::vector<cv::KeyPoint>* GetKeyPointsMutable() {
    return &keypoints_;
  }

  cv::Mat* GetDescriptorsMutable() {
    return &descriptors_;
  }

  /*
   * especially do a deep copy of the cv::Mats
   */
  DatasetEntry(const DatasetEntry& other) {
    path_ = other.path_;
    imgGray_ = other.imgGray_.clone();
    keypoints_ = other.keypoints_;
    descriptors_ = other.descriptors_.clone();
    userdata_ = other.userdata_;
  }

  std::string path() const {
    return path_;
  }

  // Returns a blob of data belonging to this key.
  Blob& getBlob(std::string key) {
    return userdata_[key];
  }

  std::string listBlobs() const {
    std::stringstream ss;
    ss << "Userdata:" << std::endl;
    int i = 0;
    for (std::map<std::string, Blob>::const_iterator it = userdata_.begin(),
        end = userdata_.end(); it != end; ++it, ++i) {
      ss << "\t\t" << i << ": " << "" "" << it->first << "" " size: "
          << it->second.size() << std::endl;
    }
    return ss.str();
  }

  bool operator==(const DatasetEntry& other) const {

    EXPECTSAMETHROW((*this), other, path_);

    /**
     * CHECK IMAGE
     */
    bool doImageVerification = true;  //not really necessary
    if (doImageVerification) {
      //check rows
      if (this->imgGray_.rows != other.imgGray_.rows) {
        EXPECTSAMETHROW((*this), other, imgGray_.rows);
        return false;
      }
      //check cols
      if (this->imgGray_.cols != other.imgGray_.cols) {
        EXPECTSAMETHROW((*this), other, imgGray_.cols);
        return false;
      }
      //check type
      if (this->imgGray_.type() != other.imgGray_.type()) {
        EXPECTSAMETHROW((*this), other, imgGray_.type());
        return false;
      }
      //check pixel by pixel (very efficient!)
      for (int i = 0, size = this->imgGray_.rows * this->imgGray_.cols;
          i < size; ++i) {
        if (this->imgGray_.data[i] != other.imgGray_.data[i]) {
          std::stringstream ss;
          ss << "Failed on imgGray_.data at [" << i % this->imgGray_.cols
              << ", " << i / this->imgGray_.cols << "] "
              << static_cast<int>(this->imgGray_.data[i]) << " other "
              << static_cast<int>(other.imgGray_.data[i]) << " at "
              << __PRETTY_FUNCTION__ << " Line: " << __LINE__ << std::endl;
          CHECK(false) << ss.str();
          return false;
        }
      }
    }

    /**
     * CHECK KEYPOINTS
     */
    if (this->keypoints_.size() != other.keypoints_.size()) {
      EXPECTSAMETHROW((*this), other, keypoints_.size());
      return false;
    }

    //TODO(slynen): we might want to sort the keypoints and descriptors by
    // location to allow detection and description to be done with blocking type
    // optimizations.
    int kpidx = 0;
    for (std::vector<cv::KeyPoint>::const_iterator it_this = this->keypoints_
        .begin(), it_other = other.keypoints_.begin(), end_this = this
        ->keypoints_.end(), end_other = other.keypoints_.end();
        it_this != end_this && it_other != end_other;
        ++it_this, ++it_other, ++kpidx) {
      CHECKCVKEYPOINTANGLESAME((*it_this), (*it_other), angle, kpidx);
      CHECKCVKEYPOINTMEMBERSAME((*it_this), (*it_other), class_id, kpidx);
      CHECKCVKEYPOINTMEMBERSAME((*it_this), (*it_other), octave, kpidx);
      CHECKCVKEYPOINTMEMBERSAME((*it_this), (*it_other), pt.x, kpidx);
      CHECKCVKEYPOINTMEMBERSAME((*it_this), (*it_other), pt.y, kpidx);
      CHECKCVKEYPOINTMEMBERSAME((*it_this), (*it_other), response, kpidx);
      CHECKCVKEYPOINTMEMBERSAME((*it_this), (*it_other), size, kpidx);
    }

    // Check descriptors.
    if (this->descriptors_.rows != other.descriptors_.rows) {

      EXPECTSAMETHROW((*this), other, descriptors_.rows);
      return false;
    }
    if (this->descriptors_.cols != other.descriptors_.cols) {
      EXPECTSAMETHROW((*this), other, descriptors_.cols);
      return false;
    }

    uint32_t hammdisttolerance = 5;
    int numberof128Blocks = other.descriptors_.step * 8 / 128;
    for (int rowidx = 0; rowidx < this->descriptors_.rows; ++rowidx) {
      const __m128i* d1 = reinterpret_cast<const __m128i *>(
          this->descriptors_.data + this->descriptors_.step * rowidx);
      const __m128i* d2 = reinterpret_cast<const __m128i *>(
          other.descriptors_.data + other.descriptors_.step * rowidx);
      uint32_t hammdist = brisk::HammingSse::SSSE3PopcntofXORed(
          d1, d2, numberof128Blocks);
      if (hammdist > hammdisttolerance)
      {
        std::cout << "Failed on descriptor " << rowidx << ": Hammdist "
        << hammdist << " " << descriptortoString(d1, numberof128Blocks) <<
        " other " <<descriptortoString(d2, numberof128Blocks) << std::endl;
        return false;
      }
    }

    // Check user data.
    for (std::map<std::string, Blob>::const_iterator it = userdata_.begin(),
        end = userdata_.end(); it != end; ++it) {
      int diffbytes = it->second.checkCurrentDataSameAsVerificationData();
      if (diffbytes) {
        std::stringstream ss;
        ss << "For userdata " << it->first << " failed with " << diffbytes
            << " bytes difference. At " << __PRETTY_FUNCTION__ << " Line: "
            << __LINE__ << std::endl;
        CHECK(false) << ss.str();
      }
    }
    return true;
  }

  bool operator!=(const DatasetEntry& other) const {
    return !operator==(other);
  }

  // Echo some information about this entry.
  std::string print() const {
    std::stringstream ss;
    ss << "\t" << path_ << " [" << imgGray_.cols << "x" << imgGray_.rows << "]"
        << std::endl;
    if (!keypoints_.empty()) {
      ss << "\t Keypoints: " << keypoints_.size() << " Descriptors: "
          << descriptors_.rows << std::endl;
    }
    ss << "\t" << listBlobs();
    return ss.str();
  }

  /**
   * remove processing results so we can re-run the pipeline on this image
   */
  void clear_processed_data(bool clearDescriptors, bool clearKeypoints) {
    if (clearDescriptors) {
      descriptors_ = cv::Mat::zeros(0, 0, CV_8U);
    }
    if (clearKeypoints) {
      keypoints_.clear();
    }
  }

  /**
   * get the images from the path and convert to grayscale
   */
  void readImage(const std::string& path) {
    path_ = path;
    cv::Mat imgRGB = cv::imread(path_);  //do we want these to be at specific mem locations?
    cv::cvtColor(imgRGB, imgGray_, CV_BGR2GRAY);
  }

  /**
   * set the static image name to the current image
   */
  void setThisAsCurrentEntry() {
    current_entry = this;
  }

  /**
   * return the name of the image currently processed
   */
  static DatasetEntry* getCurrentEntry() {
    return current_entry;
  }

 private:
  static DatasetEntry* current_entry;  //a global tag which image is currently being processed
};

template<class TYPE>
void Serialize(
    const TYPE& value, std::ofstream* out,
    typename std::enable_if<std::is_integral<TYPE>::value>::type* = 0) {
  CHECK_NOTNULL(out);
  out->write(reinterpret_cast<const char*>(&value), sizeof(value));
}

template<class TYPE>
void DeSerialize(TYPE* value, std::ifstream* in,
                 typename std::enable_if<std::is_integral<TYPE>::value>::type* =
                     0) {
  CHECK_NOTNULL(value);
  CHECK_NOTNULL(in);
  in->read(reinterpret_cast<char*>(value), sizeof(*value));
}

template<class TYPE>
void Serialize(
    const TYPE& value, std::ofstream* out,
    typename std::enable_if<std::is_floating_point<TYPE>::value>::type* = 0) {
  CHECK_NOTNULL(out);
  out->write(reinterpret_cast<const char*>(&value), sizeof(value));
}

template<class TYPE>
void DeSerialize(
    TYPE* value, std::ifstream* in,
    typename std::enable_if<std::is_floating_point<TYPE>::value>::type* = 0) {
  CHECK_NOTNULL(value);
  CHECK_NOTNULL(in);
  in->read(reinterpret_cast<char*>(value), sizeof(*value));
}

void Serialize(const uint32_t& value, std::ofstream* out) {
  CHECK_NOTNULL(out);
  out->write(reinterpret_cast<const char*>(&value), sizeof(value));
}

void DeSerialize(uint32_t* value, std::ifstream* in) {
  CHECK_NOTNULL(value);
  CHECK_NOTNULL(in);
  in->read(reinterpret_cast<char*>(value), sizeof(*value));
}

void Serialize(const cv::Mat& mat, std::ofstream* out) {
  CHECK_NOTNULL(out);
  cv::Mat mat_cont;
  if (!mat.isContinuous()) {
    mat_cont = mat.clone();
  } else {
    mat_cont = mat;
  }
  int type = mat_cont.type();
  int element_size = mat_cont.elemSize();

  Serialize(mat_cont.rows, out);
  Serialize(mat_cont.cols, out);
  Serialize(type, out);
  Serialize(element_size, out);

  out->write(reinterpret_cast<char*>(mat_cont.data),
             element_size * mat_cont.rows * mat_cont.cols);
}

void DeSerialize(cv::Mat* mat, std::ifstream* in) {
  CHECK_NOTNULL(mat);
  CHECK_NOTNULL(in);
  int rows;
  int cols;
  int type;
  int element_size;
  DeSerialize(&rows, in);
  DeSerialize(&cols, in);
  DeSerialize(&type, in);
  DeSerialize(&element_size, in);
  mat->create(rows, cols, type);
  in->read(reinterpret_cast<char*>(mat->data), element_size * rows * cols);
}

void Serialize(const cv::Point2f& pt, std::ofstream* out) {
  CHECK_NOTNULL(out);
  Serialize(pt.x, out);
  Serialize(pt.y, out);
}

template<typename TYPE>
void DeSerialize(cv::Point_<TYPE>* pt, std::ifstream* in) {
  CHECK_NOTNULL(pt);
  CHECK_NOTNULL(in);
  DeSerialize(&pt->x, in);
  DeSerialize(&pt->y, in);
}

void Serialize(const cv::KeyPoint& pt, std::ofstream* out) {
  CHECK_NOTNULL(out);
  Serialize(pt.angle, out);
  Serialize(pt.class_id, out);
  Serialize(pt.octave, out);
  Serialize(pt.pt, out);
  Serialize(pt.response, out);
  Serialize(pt.size, out);
}

void DeSerialize(cv::KeyPoint* pt, std::ifstream* in) {
  CHECK_NOTNULL(pt);
  CHECK_NOTNULL(in);
  DeSerialize(&pt->angle, in);
  DeSerialize(&pt->class_id, in);
  DeSerialize(&pt->octave, in);
  DeSerialize(&pt->pt, in);
  DeSerialize(&pt->response, in);
  DeSerialize(&pt->size, in);
}

void Serialize(const std::string& value, std::ofstream* out) {
  CHECK_NOTNULL(out);
  size_t length = value.size();
  Serialize(length, out);
  out->write(reinterpret_cast<const char*>(value.data()),
             length * sizeof(value[0]));
}

void DeSerialize(std::string* value, std::ifstream* in) {
  CHECK_NOTNULL(value);
  CHECK_NOTNULL(in);
  size_t length;
  DeSerialize(&length, in);
  value->resize(length);
  std::unique_ptr<char[]> mem(new char[length + 1]);
  in->read(mem.get(), length * sizeof(mem.get()[0]));
  mem[length] = '\0';
  *value = std::string(mem.get());
}

template<typename TYPEA, typename TYPEB>
void Serialize(const std::pair<TYPEA, TYPEB>& value, std::ofstream* out) {
  CHECK_NOTNULL(out);
  Serialize(value.first, out);
  Serialize(value.second, out);
}

template<typename TYPEA, typename TYPEB>
void DeSerialize(std::pair<TYPEA, TYPEB>* value, std::ifstream* in) {
  CHECK_NOTNULL(value);
  CHECK_NOTNULL(in);
  DeSerialize(&value->first, in);
  DeSerialize(&value->second, in);
}

template<typename TYPEA, typename TYPEB>
void Serialize(const std::map<TYPEA, TYPEB>& value, std::ofstream* out) {
  CHECK_NOTNULL(out);
  size_t length = value.size();
  Serialize(length, out);
  for (const std::pair<TYPEA, TYPEB>& entry : value) {
    Serialize(entry, out);
  }
}

template<typename TYPEA, typename TYPEB>
void DeSerialize(std::map<TYPEA, TYPEB>* value, std::ifstream* in) {
  CHECK_NOTNULL(value);
  CHECK_NOTNULL(in);
  value->clear();
  size_t length;
  DeSerialize(&length, in);
  for (size_t i = 0; i < length; ++i) {
    std::pair<TYPEA, TYPEB> entry;
    DeSerialize(&entry, in);
    value->insert(entry);
  }
}

template<typename T>
void Serialize(const std::vector<T>& value, std::ofstream* out) {
  CHECK_NOTNULL(out);
  size_t length = value.size();
  Serialize(length, out);
  for (const T& entry : value) {
    Serialize(entry, out);
  }
}

template<typename T>
void DeSerialize(std::vector<T>* value, std::ifstream* in) {
  CHECK_NOTNULL(value);
  CHECK_NOTNULL(in);
  value->clear();
  size_t length;
  DeSerialize(&length, in);
  value->resize(length);
  for (size_t i = 0; i < length; ++i) {
    DeSerialize(&value->at(i), in);
  }
}

void Serialize(const Blob& value, std::ofstream* out) {
  CHECK_NOTNULL(out);
  Serialize(value.size_, out);
  out->write(reinterpret_cast<const char*>(value.verification_data_.get()),
             value.size_);
}

void DeSerialize(Blob* value, std::ifstream* in) {
  CHECK_NOTNULL(in);
  CHECK_NOTNULL(value);
  ::DeSerialize(&value->size_, in);
  value->verification_data_.reset(new unsigned char[value->size_]);
  in->read(reinterpret_cast<char*>(value->verification_data_.get()),
           value->size_);
}

void Serialize(const DatasetEntry& value, std::ofstream* out) {
  CHECK_NOTNULL(out);
  Serialize(value.path_, out);
  Serialize(value.imgGray_, out);
  Serialize(value.keypoints_, out);
  Serialize(value.descriptors_, out);
  Serialize(value.userdata_, out);
}

void DeSerialize(DatasetEntry* value, std::ifstream* in) {
  CHECK_NOTNULL(value);
  CHECK_NOTNULL(in);
  try {
    DeSerialize(&value->path_, in);
    DeSerialize(&value->imgGray_, in);
    DeSerialize(&value->keypoints_, in);
    DeSerialize(&value->descriptors_, in);
    DeSerialize(&value->userdata_, in);
  } catch (const std::ifstream::failure& e) {
    CHECK(false) << "Failed to load DatasetEntry " + std::string(e.what());
  }
}

#endif /* BENCHDS_H_ */
