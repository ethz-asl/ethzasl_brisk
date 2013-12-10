#ifndef RTDSC_WRAPPER_H_
#define RTDSC_WRAPPER_H_

#ifdef USE_RDTSC
#include <brisk/rdtsc.h>
#endif

#ifndef BOOST_DATE_TIME_NO_LOCALE
#define BOOST_DATE_TIME_NO_LOCALE
#include <boost/date_time/posix_time/posix_time.hpp>
#undef BOOST_DATE_TIME_NO_LOCALE
#else
#include <boost/date_time/posix_time/posix_time.hpp>
#endif

#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics.hpp>
#include <boost/accumulators/statistics/rolling_mean.hpp>
#include <map>
#include <vector>

#ifdef _WIN32
#define SM_USE_HIGH_PERF_TIMER
#include <windows.h>
#endif

namespace rdtsc {
namespace timing {

struct TimerMapValue {
  // Initialize the window size for the rolling mean.
  TimerMapValue()
      : m_acc(boost::accumulators::tag::rolling_window::window_size = 50) {
  }
  boost::accumulators::accumulator_set<double,
      boost::accumulators::features<boost::accumulators::tag::lazy_variance,
          boost::accumulators::tag::sum, boost::accumulators::tag::min,
          boost::accumulators::tag::max, boost::accumulators::tag::rolling_mean,
          boost::accumulators::tag::mean> > m_acc;
};

// A class that has the timer interface but does nothing.
// Swapping this in in place of the Timer class (say with a
// typedef) should allow one to disable timing. Because all
// of the functions are inline, they should just disappear.
class DummyTimer {
 public:
  DummyTimer(size_t handle, bool constructStopped = false) {
  }
  DummyTimer(std::string const & tag, bool constructStopped = false) {
  }
  ~DummyTimer() { }

  void start() { }
  void stop() { }
  bool isTiming() {
    return false;
  }
};

class Timer;

class Timing {
 public:
  friend class Timer;
  // Static functions to query the timers:
  static size_t getHandle(std::string const & tag);
  static std::string getTag(size_t handle);
  static double getTotalSeconds(size_t handle);
  static double getTotalSeconds(std::string const & tag);
  static double getMeanSeconds(size_t handle);
  static double getMeanSeconds(std::string const & tag);
  static size_t getNumSamples(size_t handle);
  static size_t getNumSamples(std::string const & tag);
  static double getVarianceSeconds(size_t handle);
  static double getVarianceSeconds(std::string const & tag);
  static double getMinSeconds(size_t handle);
  static double getMinSeconds(std::string const & tag);
  static double getMaxSeconds(size_t handle);
  static double getMaxSeconds(std::string const & tag);
  static double getHz(size_t handle);
  static double getHz(std::string const & tag);
  static void print(std::ostream & out);
  static std::string print();
  static std::string secondsToTimeString(double seconds);
  void addTime(size_t handle, double seconds);
  void addCycles(size_t handle, double cycles);
 private:
  static Timing & instance();

  // Singleton design pattern.
  Timing();
  ~Timing();

  typedef std::map<std::string, size_t> map_t;
  typedef std::vector<TimerMapValue> list_t;

  // Static members.
  list_t m_timers;
  map_t m_tagMap;
  size_t m_maxTagLength;

};

class Timer {
 public:
  Timer(size_t handle, bool constructStopped = false)
      : m_timing(false),
        m_handle(handle)

  {
    if (!constructStopped)
      start();
  }
  Timer(std::string const & tag, bool constructStopped = false)
      : m_timing(false),
        m_handle(Timing::getHandle(tag)) {
    if (!constructStopped)
      start();
  }
  ~Timer() {
    if (isTiming())
      stop();
  }

  inline void start() {
    m_timing = true;
#ifdef SM_USE_HIGH_PERF_TIMER
    QueryPerformanceCounter(&m_time);
#elif USE_RDTSC
    CPUID(); RDTSC(start_); RDTSC(end_);
    CPUID(); RDTSC(start_); RDTSC(end_);

    CPUID(); RDTSC(start_);

#else
    m_time = boost::posix_time::microsec_clock::local_time();
#endif
  }

  inline void stop() {
#ifdef SM_USE_HIGH_PERF_TIMER
    double dt;
    LARGE_INTEGER end;
    QueryPerformanceCounter(&end);
    dt = (double)(end.QuadPart - m_time.QuadPart)*Timing::instance().m_clockPeriod;
    Timing::instance().addTime(m_handle, dt);
#elif USE_RDTSC
    RDTSC(end_); CPUID();
    double cycles = ((double)COUNTER_DIFF(end_, start_));
    Timing::instance().addCycles(m_handle, cycles);
#else
    double dt;
    boost::posix_time::ptime now =
        boost::posix_time::microsec_clock::local_time();
    boost::posix_time::time_duration t = now - m_time;
    dt = ((double) t.total_nanoseconds() * 1e-9);
    Timing::instance().addTime(m_handle, dt);
#endif
    m_timing = false;
  }
  inline bool isTiming() {
    return m_timing;
  }
 private:
#ifdef SM_USE_HIGH_PERF_TIMER
  LARGE_INTEGER m_time;
#elif USE_RDTSC
  tsc_counter start_, end_;
#else
  boost::posix_time::ptime m_time;
#endif
  bool m_timing;
  size_t m_handle;
};

}  // namespace timing
}  // namespace sm

#endif  // RTDSC_WRAPPER_H_
