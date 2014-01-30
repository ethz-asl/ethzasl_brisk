// Copyright 2013 Motorola Mobility LLC. Part of the Trailmix project.
// CONFIDENTIAL. AUTHORIZED USE ONLY. DO NOT REDISTRIBUTE.
#include <algorithm>
#include <ostream>  //NOLINT
#include <sstream>
#include <stdio.h>
#include <string>
#ifdef _MSC_VER
#include <boost/foreach.hpp>
#endif


#ifdef USE_RDTSC
#include <brisk/internal/rdtsc.h>
#endif
#include <brisk/internal/timer.h>

namespace brisk {
namespace timing {

const double kNumSecondsPerNanosecond = 1.e-9;

Timing& Timing::Instance() {
  static Timing t;
  return t;
}

Timing::Timing() : maxTagLength_(0) { }

Timing::~Timing() { }

// Static functions to query the timers:
size_t Timing::GetHandle(std::string const& tag) {
  // Search for an existing tag.
  map_t::iterator i = Instance().tagMap_.find(tag);
  if (i == Instance().tagMap_.end()) {
    // If it is not there, create a tag.
    size_t handle = Instance().timers_.size();
    Instance().tagMap_[tag] = handle;
    Instance().timers_.push_back(TimerMapValue());
    // Track the maximum tag length to help printing a table of timing values
    // later.
    Instance().maxTagLength_ = std::max(Instance().maxTagLength_, tag.size());
    return handle;
  } else {
    return i->second;
  }
}

std::string Timing::GetTag(size_t handle) {
  std::string tag = "";

  // Perform a linear search for the tag.
#ifdef _MSC_VER
  BOOST_FOREACH(map_t::value_type current_tag, Instance().tagMap_) {
#else
  for (typename map_t::value_type current_tag : Instance().tagMap_) {
#endif
    if (current_tag.second == handle) {
      return current_tag.first;
    }
  }
  return tag;
}

// Class functions used for timing.
Timer::Timer(size_t handle, bool construct_stopped)
    : timing_(false),
      handle_(handle) {
  if (!construct_stopped)
    Start();
}

Timer::Timer(std::string const& tag, bool construct_stopped)
    : timing_(false),
      handle_(Timing::GetHandle(tag)) {
  if (!construct_stopped)
    Start();
}

Timer::~Timer() {
  if (IsTiming())
    Stop();
}

void Timer::Start() {
  timing_ = true;
#if USE_RDTSC
  CPUID(); RDTSC(start_); RDTSC(end_);
  CPUID(); RDTSC(start_); RDTSC(end_);

  CPUID(); RDTSC(start_);
#else
#ifdef _MSC_VER
using boost::chrono::system_clock;
#else
using std::chrono::system_clock;
#endif  // _MSC_VER
  time_ = system_clock::now();
#endif  // USE_RDTSC
}

void Timer::Stop() {
#if USE_RDTSC
  RDTSC(end_); CPUID();
  double cycles = (static_cast<double>(COUNTER_DIFF(end_, start_)));
  Timing::Instance().AddCycles(handle_, cycles);
#else
#ifdef _MSC_VER
using boost::chrono::time_point;
using boost::chrono::system_clock;
using boost::chrono::duration_cast;
using boost::chrono::nanoseconds;
#else
using std::chrono::time_point;
using std::chrono::system_clock;
using std::chrono::duration_cast;
using std::chrono::nanoseconds;
#endif  // _MSC_VER
  time_point<system_clock> now = system_clock::now();
  double dt = static_cast<double>(duration_cast<nanoseconds>(now - time_).count())
      * kNumSecondsPerNanosecond;
  Timing::Instance().AddTime(handle_, dt);
#endif
  timing_ = false;
}

bool Timer::IsTiming() const {
  return timing_;
}

size_t Timer::GetHandle() const {
  return handle_;
}

void Timing::AddTime(size_t handle, double seconds) {
  timers_[handle].acc_.Add(seconds);
}

void Timing::AddCycles(size_t handle, double cycles) {
  timers_[handle].acc_.Add(cycles);
}

double Timing::GetTotalSeconds(size_t handle) {
  return Instance().timers_[handle].acc_.Sum();
}
double Timing::GetTotalSeconds(std::string const& tag) {
  return GetTotalSeconds(GetHandle(tag));
}
double Timing::GetMeanSeconds(size_t handle) {
  return Instance().timers_[handle].acc_.Mean();
}
double Timing::GetMeanSeconds(std::string const& tag) {
  return GetMeanSeconds(GetHandle(tag));
}
size_t Timing::GetNumSamples(size_t handle) {
  return Instance().timers_[handle].acc_.TotalSamples();
}
size_t Timing::GetNumSamples(std::string const& tag) {
  return GetNumSamples(GetHandle(tag));
}
double Timing::GetVarianceSeconds(size_t handle) {
  return Instance().timers_[handle].acc_.LazyVariance();
}
double Timing::GetVarianceSeconds(std::string const& tag) {
  return GetVarianceSeconds(GetHandle(tag));
}
double Timing::GetMinSeconds(size_t handle) {
  return Instance().timers_[handle].acc_.Min();
}
double Timing::GetMinSeconds(std::string const& tag) {
  return GetMinSeconds(GetHandle(tag));
}
double Timing::GetMaxSeconds(size_t handle) {
  return Instance().timers_[handle].acc_.Max();
}
double Timing::GetMaxSeconds(std::string const& tag) {
  return GetMaxSeconds(GetHandle(tag));
}

double Timing::GetHz(size_t handle) {
  return 1.0 / Instance().timers_[handle].acc_.RollingMean();
}

double Timing::GetHz(std::string const& tag) {
  return GetHz(GetHandle(tag));
}

#ifdef _MSC_VER
#define snprintf c99_snprintf

inline int c99_snprintf(char* str, size_t size, const char* format, ...) {
  int count;
  va_list ap;
  va_start(ap, format);
  count = c99_vsnprintf(str, size, format, ap);
  va_end(ap);
  return count;
}

inline int c99_vsnprintf(char* str, size_t size, const char* format, va_list ap) {
  int count = -1;
  if (size != 0)
    count = _vsnprintf_s(str, size, _TRUNCATE, format, ap);
  if (count == -1)
    count = _vscprintf(format, ap);
  return count;
}
#endif  // _MSC_VER

std::string Timing::SecondsToTimeString(double seconds) {
  double secs = fmod(seconds, 60);
  int minutes = (seconds / 60);
  int hours = (seconds / 3600);
  minutes -= (hours * 60);

char buffer[256];
snprintf(buffer, sizeof(buffer),
#ifdef BRISK_TIMING_SHOW_HOURS
"%02d:"
#endif
#ifdef BRISK_TIMING_SHOW_MINUTES
"%02d:"
#endif
"%09.6f",
#ifdef BRISK_TIMING_SHOW_HOURS
hours,
#endif
#ifdef BRISK_TIMING_SHOW_MINUTES
minutes,
#endif
secs);
return buffer;
}

void Timing::Print(std::ostream& out) {  //NOLINT
  map_t& tagMap = Instance().tagMap_;

  if (tagMap.empty()) {
    return;
  }
#ifdef USE_RDTSC
  out << "BRISK Timing - RDTSC cycles\n";
#else
  out << "BRISK Timing\n";
#endif
  out << "-----------\n";
  
#ifdef _MSC_VER
  BOOST_FOREACH(map_t::value_type t, tagMap) {
#else
  for (typename map_t::value_type t : tagMap) {
#endif  // _MSC_VER
    size_t i = t.second;
    out.width((std::streamsize) Instance().maxTagLength_);
    out.setf(std::ios::left, std::ios::adjustfield);
    out << t.first << "\t";
    out.width(7);

    out.setf(std::ios::right, std::ios::adjustfield);
    out << GetNumSamples(i) << "\t";
    if (GetNumSamples(i) > 0) {
#ifdef USE_RDTSC
      out << GetTotalSeconds(i) << "\t";
      double meansec = GetMeanSeconds(i);
      double stddev = sqrt(GetVarianceSeconds(i));
      out << "(" << meansec << " +- ";
      out << stddev << ")\t";

      double minsec = GetMinSeconds(i);
      double maxsec = GetMaxSeconds(i);

      // The min or max are out of bounds.
      out << "[" << minsec << "," << maxsec << "]";
#else
      out << SecondsToTimeString(GetTotalSeconds(i)) << "\t";
      double meansec = GetMeanSeconds(i);
      double stddev = sqrt(GetVarianceSeconds(i));
      out << "(" << SecondsToTimeString(meansec) << " +- ";
      out << SecondsToTimeString(stddev) << ")\t";

      double minsec = GetMinSeconds(i);
      double maxsec = GetMaxSeconds(i);

      // The min or max are out of bounds.
      out << "[" << SecondsToTimeString(minsec) << ","
          << SecondsToTimeString(maxsec) << "]";
#endif
    }
    out << std::endl;
  }
}
std::string Timing::Print() {
  std::stringstream ss;
  Print(ss);
  return ss.str();
}

void Timing::Reset() {
  Instance().tagMap_.clear();
}

}  // namespace timing
}  // namespace brisk
