/*
 Copyright (C) 2011  The Autonomous Systems Lab, ETH Zurich,
 Stefan Leutenegger, Simon Lynen and Margarita Chli.

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

#include <brisk/internal/rdtsc-wrapper.h>

namespace rdtsc {
namespace timing {

Timing & Timing::instance() {
  static Timing t;
  return t;
}

Timing::Timing()
    : m_maxTagLength(0) {
#ifdef SM_USE_HIGH_PERF_TIMER
  LARGE_INTEGER freq;
  BOOL returnCode = QueryPerformanceFrequency(&freq);
  SM_ASSERT_NE(TimerException,returnCode,0,"Unable to query the performance frequency");
  m_clockPeriod = 1.0 / freq.QuadPart;
#endif
}

Timing::~Timing() {

}

// Static funcitons to query the timers:
size_t Timing::getHandle(std::string const & tag) {
  // Search for an existing tag.
  map_t::iterator i = instance().m_tagMap.find(tag);
  if (i == instance().m_tagMap.end()) {
    // If it is not there, create a tag.
    size_t handle = instance().m_timers.size();
    instance().m_tagMap[tag] = handle;
    instance().m_timers.push_back(TimerMapValue());
    // Track the maximum tag length to help printing a table of timing values later.
    instance().m_maxTagLength = std::max(instance().m_maxTagLength, tag.size());
    return handle;
  } else {
    return i->second;
  }
}

std::string Timing::getTag(size_t handle) {
  std::string tag;
  bool found = false;

  // Perform a linear search for the tag.
  map_t::iterator i = instance().m_tagMap.begin();
  for (; i != instance().m_tagMap.end(); i++) {
    if (i->second == handle) {
      found = true;
      tag = i->first;
      break;
    }
  }
  if (!found)
    assert(found && "Unable to find the tag for a given handle.");
  return tag;
}

void Timing::addTime(size_t handle, double seconds) {
  m_timers[handle].m_acc(seconds);
}
void Timing::addCycles(size_t handle, double cycles) {
  m_timers[handle].m_acc(cycles);
}

double Timing::getTotalSeconds(size_t handle) {
  return boost::accumulators::extract::sum(instance().m_timers[handle].m_acc);
}
double Timing::getTotalSeconds(std::string const & tag) {
  return getTotalSeconds(getHandle(tag));
}
double Timing::getMeanSeconds(size_t handle) {
  return boost::accumulators::extract::mean(instance().m_timers[handle].m_acc);
}
double Timing::getMeanSeconds(std::string const & tag) {
  return getMeanSeconds(getHandle(tag));
}
size_t Timing::getNumSamples(size_t handle) {
  return boost::accumulators::extract::count(instance().m_timers[handle].m_acc);
}
size_t Timing::getNumSamples(std::string const & tag) {
  return getNumSamples(getHandle(tag));
}
double Timing::getVarianceSeconds(size_t handle) {
  return boost::accumulators::extract::variance(
      instance().m_timers[handle].m_acc);
}
double Timing::getVarianceSeconds(std::string const & tag) {
  return getVarianceSeconds(getHandle(tag));
}
double Timing::getMinSeconds(size_t handle) {
  return boost::accumulators::extract::min(instance().m_timers[handle].m_acc);
}
double Timing::getMinSeconds(std::string const & tag) {
  return getMinSeconds(getHandle(tag));
}
double Timing::getMaxSeconds(size_t handle) {
  return boost::accumulators::extract::max(instance().m_timers[handle].m_acc);
}
double Timing::getMaxSeconds(std::string const & tag) {
  return getMaxSeconds(getHandle(tag));
}

double Timing::getHz(size_t handle) {
  return 1.0
      / boost::accumulators::extract::rolling_mean(
          instance().m_timers[handle].m_acc);
}

double Timing::getHz(std::string const & tag) {
  return getHz(getHandle(tag));
}

std::string Timing::secondsToTimeString(double seconds) {

  double secs = fmod(seconds, 60);
  int minutes = (long) (seconds / 60);
  int hours = (long) (seconds / 3600);
  minutes = minutes - (hours * 60);

  char buffer[256];
#ifdef WIN32
  sprintf_s(buffer,256,"%02d:%02d:%09.6f",hours,minutes,secs);
#else
  sprintf(buffer, "%02d:%02d:%09.6f", hours, minutes, secs);
#endif
  return buffer;
}

void Timing::print(std::ostream & out) {
  map_t & tagMap = instance().m_tagMap;

#ifdef USE_RDTSC
  out << "SM Timing - RDTSC cycles\n";
#else
  out << "SM Timing\n";
#endif
  out << "-----------\n";
  map_t::iterator t = tagMap.begin();
  for (; t != tagMap.end(); t++) {
    size_t i = t->second;
    out.width((std::streamsize) instance().m_maxTagLength);
    out.setf(std::ios::left, std::ios::adjustfield);
    out << t->first << "\t";
    out.width(7);

    out.setf(std::ios::right, std::ios::adjustfield);
    out << getNumSamples(i) << "\t";
    if (getNumSamples(i) > 0) {
#ifdef USE_RDTSC
      // The same functions just that they return cycles.
      out << getTotalSeconds(i) << "\t";
      double meansec = getMeanSeconds(i);
      double stddev = sqrt(getVarianceSeconds(i));
      out << "(" << meansec << " +- ";
      out << stddev << ")\t";

      double minsec = getMinSeconds(i);
      double maxsec = getMaxSeconds(i);

      // The min or max are out of bounds.
      out << "[" << minsec << "," << maxsec << "]";
#else
      out << secondsToTimeString(getTotalSeconds(i)) << "\t";
      double meansec = getMeanSeconds(i);
      double stddev = sqrt(getVarianceSeconds(i));
      out << "(" << secondsToTimeString(meansec) << " +- ";
      out << secondsToTimeString(stddev) << ")\t";

      double minsec = getMinSeconds(i);
      double maxsec = getMaxSeconds(i);

      // The min or max are out of bounds.
      out << "[" << secondsToTimeString(minsec) << ","
          << secondsToTimeString(maxsec) << "]";
#endif
    }
    out << std::endl;
  }
}
std::string Timing::print() {
  std::stringstream ss;
  print(ss);
  return ss.str();
}

}  // namespace timing
}  // end namespace sm