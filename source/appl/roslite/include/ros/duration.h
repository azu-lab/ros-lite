#pragma once

#if ROSLITE_TARGET_CLUSTER_ID == 0
  #include "roslite/include/ros/common.h"
  #include "roslite/include/ros/rate.h"
  #include "ros/duration.h"
#else
  #include "ros/common.h"
  #include "ros/rate.h"
#endif

#include <limits>
#include <cmath>
#include <stdexcept>

namespace ROSLITE_NAMESPACE
{

/**
 * \brief Base class for Duration implementations.  Provides storage, common functions and operator overloads.
 * This should not need to be used directly.
 */
template <class T>
class DurationBase
{
public:
    int32_t sec, nsec;
    DurationBase() : sec(0), nsec(0) { }
    DurationBase(int32_t _sec, int32_t _nsec);
    explicit DurationBase(double t){fromSec(t);};
    ~DurationBase() {}
    T operator+(const T &rhs) const;
    T operator-(const T &rhs) const;
    T operator-() const;
    T operator*(double scale) const;
    T& operator+=(const T &rhs);
    T& operator-=(const T &rhs);
    T& operator*=(double scale);
    bool operator==(const T &rhs) const;
    inline bool operator!=(const T &rhs) const { return !(*static_cast<const T*>(this) == rhs); }
    bool operator>(const T &rhs) const;
    bool operator<(const T &rhs) const;
    bool operator>=(const T &rhs) const;
    bool operator<=(const T &rhs) const;
    double toSec() const { return (double)sec + 1e-9*(double)nsec; };
    int64_t toNSec() const {return (int64_t)sec*1000000000ll + (int64_t)nsec;  };
    T& fromSec(double d)
    {
        int64_t sec64 = static_cast<int64_t>(floor(d));
        if (sec64 < std::numeric_limits<int32_t>::min() || sec64 > std::numeric_limits<int32_t>::max())
            throw std::runtime_error("Duration is out of dual 32-bit range");
        sec = static_cast<int32_t>(sec64);
        nsec = static_cast<int32_t>(std::round((d - sec) * 1e9));
        int32_t rollover = nsec / 1000000000ul;
        sec += rollover;
        nsec %= 1000000000ul;
        return *static_cast<T*>(this);
    }
    T& fromNSec(int64_t t);
    bool isZero() const;
    // boost::posix_time::time_duration toBoost() const;
};    

/**
 * \brief Duration representation for use with the Time class.
 *
 * ros::DurationBase provides most of its functionality.
 */
class Duration : public DurationBase<Duration>
{
public:
    Duration()
    : DurationBase<Duration>()
    { }

    Duration(int32_t _sec, int32_t _nsec)
    : DurationBase<Duration>(_sec, _nsec)
    {}

    explicit Duration(double t) { fromSec(t); }
    explicit Duration(const Rate&);
    /**
     * \brief sleep for the amount of time specified by this Duration.  If a signal interrupts the sleep, resleeps for the time remaining.
     */
    bool sleep() const;

#if ROSLITE_TARGET_CLUSTER_ID == 0
    static roslite::Duration FromRosMsg(const typename ros::Duration& ros_duration) {
        roslite::Duration roslite_duration;
        roslite_duration.sec = ros_duration.sec;
        roslite_duration.nsec = ros_duration.nsec;
        return roslite_duration;
    }

    ros::Duration ToRosMsg() const {
        ros::Duration ros_duration;
        ros_duration.sec = sec;
        ros_duration.nsec = nsec;
        return ros_duration;
    }
#endif
};

}  // namespace ROSLITE_NAMESPACE
