#ifndef BRISK_TIMING_TIMER_H_
#define BRISK_TIMING_TIMER_H_

#include <algorithm>
#ifdef _MSC_VER
#include <boost/chrono.hpp>
#else
#include <chrono>
#endif  // _MSC_VER
#include <limits>
#include <map>
#include <string>
#include <vector>

namespace brisk {
namespace timing {

template<typename T, typename Total, int N>
class Accumulator {
 public:
  Accumulator()
      : window_samples_(0), totalsamples_(0),
        sum_(0), window_sum_(0),
        min_(std::numeric_limits <T> ::max()),
        max_(std::numeric_limits <T> ::min()) { }

  void Add(T sample) {
    if (window_samples_ < N) {
      samples_[window_samples_++] = sample;
      window_sum_ += sample;
    } else {
      T& oldest = samples_[window_samples_++ % N];
      window_sum_ += sample - oldest;
      oldest = sample;
    }
    sum_ += sample;
    ++totalsamples_;
    if (sample > max_) {
      max_ = sample;
    }
    if (sample < min_) {
      min_ = sample;
    }
  }

  int TotalSamples() const {
    return totalsamples_;
  }

  double Sum() const {
    return sum_;
  }

  double Mean() const {
    return sum_ / totalsamples_;
  }

  double RollingMean() const {
    return window_sum_ / std::min(window_samples_, N);
  }

  double Max() const {
    return max_;
  }

  double Min() const {
    return min_;
  }

  double LazyVariance() const {
    if (window_samples_ == 0) {
      return 0.0;
    }
    double var = 0;
    double mean = RollingMean();
    for (int i = 0; i < std::min(window_samples_, N); ++i) {
      var += (samples_[i] - mean) * (samples_[i] - mean);
    }
    var /= std::min(window_samples_, N);
    return var;
  }

 private:
  T samples_[N];
  int window_samples_;
  int totalsamples_;
  Total sum_;
  Total window_sum_;
  T min_;
  T max_;
};

struct TimerMapValue {
  TimerMapValue() { }

  // Create an accumulator with specified window size.
  Accumulator<double, double, 50> acc_;
};

// A class that has the timer interface but does nothing. Swapping this in in
// place of the Timer class (say with a typedef) should allow one to disable
// timing. Because all of the functions are inline, they should just disappear.
class DummyTimer {
 public:
  DummyTimer(size_t /*handle*/, bool construct_stopped = false) {
    static_cast<void>(construct_stopped);
  }
  DummyTimer(std::string const& /*tag*/, bool construct_stopped = false) {
    static_cast<void>(construct_stopped);
  }
  ~DummyTimer() { }

  void Start() { }
  void Stop() { }
  bool IsTiming() {
    return false;
  }
};

class Timer {
 public:
  Timer(size_t handle, bool construct_stopped = false);
  Timer(std::string const& tag, bool construct_stopped = false);
  ~Timer();

  void Start();
  void Stop();
  bool IsTiming() const;
  size_t GetHandle() const;
 private:
#ifdef _MSC_VER
  boost::chrono::time_point<boost::chrono::system_clock> time_;
#else
  std::chrono::time_point<std::chrono::system_clock> time_;
#endif
#if USE_RDTSC
  tsc_counter start_, end_;
#endif
  bool timing_;
  size_t handle_;
};

class Timing {
 public:
  typedef std::map<std::string, size_t> map_t;
  friend class Timer;
  // Definition of static functions to query the timers.
  static size_t GetHandle(std::string const& tag);
  static std::string GetTag(size_t handle);
  static double GetTotalSeconds(size_t handle);
  static double GetTotalSeconds(std::string const& tag);
  static double GetMeanSeconds(size_t handle);
  static double GetMeanSeconds(std::string const& tag);
  static size_t GetNumSamples(size_t handle);
  static size_t GetNumSamples(std::string const& tag);
  static double GetVarianceSeconds(size_t handle);
  static double GetVarianceSeconds(std::string const& tag);
  static double GetMinSeconds(size_t handle);
  static double GetMinSeconds(std::string const& tag);
  static double GetMaxSeconds(size_t handle);
  static double GetMaxSeconds(std::string const& tag);
  static double GetHz(size_t handle);
  static double GetHz(std::string const& tag);
  static void Print(std::ostream& out);  //NOLINT
  static std::string Print();
  static std::string SecondsToTimeString(double seconds);
  static void Reset();
  static const map_t& GetTimers() {return Instance().tagMap_;}

 private:
  void AddTime(size_t handle, double seconds);
  void AddCycles(size_t handle, double cycles);

  static Timing& Instance();

  Timing();
  ~Timing();

  typedef std::vector<TimerMapValue> list_t;

  list_t timers_;
  map_t tagMap_;
  size_t maxTagLength_;
};

#if ENABLE_BRISK_TIMING
typedef Timer DebugTimer;
#else
typedef DummyTimer DebugTimer;
#endif

}  // namespace timing
}  // namespace brisk
#endif  // BRISK_TIMING_TIMER_H_
