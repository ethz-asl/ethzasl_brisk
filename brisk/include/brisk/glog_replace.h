#ifndef GLOG_REPLACE_H_
#define GLOG_REPLACE_H_

#if !HAVE_GLOG
#include <cassert>
#include <iosfwd>

struct nullstream {};
template <typename T>
inline nullstream & operator<<(nullstream& s, const T&) {
  return s;
}
inline nullstream & operator<<(nullstream& s, std::ostream&(std::ostream&)) {
  return s;
}
static nullstream logstream;

#define CHECK_NOTNULL(x) assert(x != nullptr);
#define CHECK_EQ(x, y) assert(x == y); logstream
#define CHECK_NE(x, y) assert(x != y); logstream
#define CHECK_GT(x, y) assert(x > y); logstream
#define CHECK_LT(x, y) assert(x < y); logstream
#define CHECK_GE(x, y) assert(x >= y); logstream
#define CHECK_LE(x, y) assert(x <= y); logstream
#define CHECK(x) assert(x); logstream

#endif  // !HAVE_GLOG
#endif  // GLOG_REPLACE_H_
