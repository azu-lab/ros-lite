#pragma once

#if ROSLITE_TARGET_CLUSTER_ID == 0
  #include "roslite/include/ros/common.h"
  #include "roslite/include/ros/duration.h"
#else
  #include "ros/common.h"
  #include "ros/duration.h"
#endif

namespace ROSLITE_NAMESPACE
{
/*********************************************************************
 ** Functions
*********************************************************************/

void normalizeSecNSec(uint64_t& sec, uint64_t& nsec);
void normalizeSecNSec(uint32_t& sec, uint32_t& nsec);

/*********************************************************************
 ** Time Classes
*********************************************************************/

/**
 * \brief Base class for Time implementations.  Provides storage, common functions and operator overloads.
 * This should not need to be used directly.
 */
template<class T, class D>
class TimeBase
{
    public:
    uint32_t sec, nsec;

    TimeBase() : sec(0), nsec(0) { }
    TimeBase(uint32_t _sec, uint32_t _nsec) : sec(_sec), nsec(_nsec)
    {
        normalizeSecNSec(sec, nsec);
    }
    // explicit TimeBase(double t) { fromSec(t); }
    ~TimeBase() {}
    D operator-(const T &rhs) const;
    T operator+(const D &rhs) const;
    T operator-(const D &rhs) const;
    T& operator+=(const D &rhs);
    T& operator-=(const D &rhs);
    bool operator==(const T &rhs) const;
    inline bool operator!=(const T &rhs) const { return !(*static_cast<const T*>(this) == rhs); }
    bool operator>(const T &rhs) const;
    bool operator<(const T &rhs) const;
    bool operator>=(const T &rhs) const;
    bool operator<=(const T &rhs) const;

    double toSec()  const { return (double)sec + 1e-9*(double)nsec; };
    // T& fromSec(double t) { sec = (uint32_t)floor(t); nsec = (uint32_t)boost::math::round((t-sec) * 1e9);  return *static_cast<T*>(this);}

    uint64_t toNSec() const {return (uint64_t)sec*1000000000ull + (uint64_t)nsec;  }
    T& fromNSec(uint64_t t);

    inline bool isZero() const { return sec == 0 && nsec == 0; }
    inline bool is_zero() const { return isZero(); }
    // boost::posix_time::ptime toBoost() const;

};

/**
 * \brief Time representation.  May either represent wall clock time or ROS clock time.
 *
 * ros::TimeBase provides most of its functionality.
 */
class Time : public TimeBase<Time, Duration>
{
public:
    Time()
    : TimeBase<Time, Duration>()
    {}

    Time(uint32_t _sec, uint32_t _nsec)
    : TimeBase<Time, Duration>(_sec, _nsec)
    {}

    // explicit Time(double t) { fromSec(t); }

    /**
     * \brief Retrieve the current time.  If ROS clock time is in use, this returns the time according to the
     * ROS clock.  Otherwise returns the current wall clock time.
     */
    static Time now();
    /**
     * \brief Sleep until a specific time has been reached.
     */
    static bool sleepUntil(const Time& end);

    static void init();
    static void shutdown();
    static void setNow(const Time& new_now);
    static bool useSystemTime();
    static bool isSimTime();
    static bool isSystemTime();

    /**
     * \brief Returns whether or not the current time is valid.  Time is valid if it is non-zero.
     */
    static bool isValid();
    /**
     * \brief Wait for time to become valid
     */
    static bool waitForValid();
    /**
     * \brief Wait for time to become valid, with timeout
     */
    // static bool waitForValid(const ros::WallDuration& timeout);

    // static Time fromBoost(const boost::posix_time::ptime& t);
    // static Time fromBoost(const boost::posix_time::time_duration& d);
};  


}  // namespace ROSLITE_NAMESPACE
