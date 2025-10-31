#ifndef MARINE_NAV_UTILITIES_GZ4D_ANGLES_HPP
#define MARINE_NAV_UTILITIES_GZ4D_ANGLES_HPP


#include <cmath>

namespace marine_nav_utilities
{
namespace gz4d
{ 

////////// Angles


// period units: ranges for periodic types, such as angles
namespace pu
{
  struct Degree
  {
    static constexpr double period = 360.0;
    static constexpr double half_period = 180.0;
      // template <typename T> static T period(){return 360.0;}
      // template <typename T> static T half_period(){return 180.0;}
  };

  struct Radian
  {
    static constexpr double half_period = M_PI;
    static constexpr double period = half_period*2.0;
      // template<typename T> static T half_period(){return 3.14159265358979323846;}
      // template<typename T> static T period(){return half_period<T>()*2.0;}
  };
}

// range types, where does a periodic value roll over
namespace rt
{
  // zero to period length
  struct PositivePeriod
  {
      template<typename T, typename PU> static T fix(T v)
      {
          if(v >= 0.0 && v < PU::period)
              return v;
          T ret = fmod(v,PU::period);
          if(ret < 0)
              ret += PU::period;
          return ret;
      }
  };

  // minus half period to plus half period
  struct ZeroCenteredPeriod
  {
      template<typename T, typename PU> static T fix(T v)
      {
          T half_period = PU::half_period;
          if(v <= half_period && v > -half_period)
              return v;
          T period = PU::period;
          T ret = fmod(v,period);
          if(ret > half_period)
              return ret-period;
          if(ret < -half_period)
              return ret+period;
          return ret;
      }
  };
  
  // unclamped, so doesn't roll over
  struct Unclamped
  {
      template<typename T, typename PU> static T fix(T v)
      {
        return v;
      }
  };

  /// cover half a circle, such as latitudes
  /// Throws out_of_range if necessary
  struct ZeroCenteredHalfPeriodChecked
  {
    template<typename T, typename PU> static T fix(T v)
    {
      static constexpr T quarter_period = PU::half_period/2.0;
      if(v < -quarter_period || v > quarter_period)
      {
        std::stringstream ss;
        ss << "Angle " << v << " out of range (" << -quarter_period << " to " << quarter_period << ")";
        throw(std::range_error(ss.str()));
      }
      return v;
    }
  };
}

// Angle class aware of its units and range so it automatically rolls over
// T is type of value
// PU is period units, degrees, radians or other
// RT is range type, such as zero centered, positive, or other
template<typename T, typename PU, typename RT = rt::PositivePeriod> class Angle
{
    T _value;

public:
    typedef T value_type;
    typedef PU period_units;
    typedef RT range_type;

    Angle():_value(RT::template fix<T, PU>(0.0)){}
    explicit Angle(T v):_value(RT::template fix<T, PU>(v)){}
    Angle(Angle<T,PU,RT> const &a):_value(a._value){}
    template <typename ORT> Angle(Angle<T,PU,ORT> const &a):_value(RT::template fix<T, PU>(a.value())){}
    template <typename OPU, typename ORT> Angle(Angle<T,OPU,ORT> const &a):_value(RT::template fix<T, PU>(a.normalized()*PU::period)){}

    T normalized() const{return _value/PU::period;}
    
    operator T() const {return _value;}
    T value() const {return _value;}

    // period aware operators, if the other value is more than half a period away, consider wrapping
    //bool operator<(Angle<T,PU,RT> o) const {return (o._value-_value > 0.0 && o._value-_value < PU::template half_period<T>()) || o._value-_value < -PU::template half_period<T>();}
    //bool operator==(Angle<T,PU,RT> o) const {return _value==o._value;}
    template<typename OPU, typename ORT> bool operator!=(Angle<T,OPU,ORT> o) const {return ! *this==o;}
    template<typename OPU, typename ORT> bool operator>(Angle<T,OPU,ORT> o) const {return o<*this;}
    template<typename OPU, typename ORT> bool operator<=(Angle<T,OPU,ORT> o) const {return !*this>o;}
    template<typename OPU, typename ORT> bool operator>=(Angle<T,OPU,ORT> o) const {return !*this<o;}

    Angle<T,PU,RT> const &operator-=(Angle<T,PU,RT> o) {_value = RT::template fix<T, PU>(_value-o._value); return *this;}
    Angle<T,PU,RT> const &operator+=(Angle<T,PU,RT> o) {_value = RT::template fix<T, PU>(_value+o._value); return *this;}
    Angle<T,PU,RT> const &operator*=(T o) {_value = RT::template fix<T, PU>(_value*o); return *this;}
    Angle<T,PU,RT> const &operator/=(T o) {_value = RT::template fix<T, PU>(_value/o); return *this;}

    Angle<T,PU,RT> operator+(Angle<T,PU,RT> o) const {return Angle<T,PU,RT>(*this)+= o;}
    Angle<T,PU,RT> operator-(Angle<T,PU,RT> o) const {return Angle<T,PU,RT>(*this)-= o;}
    Angle<T,PU,RT> operator*(T o) const {return Angle<T,PU,RT>(*this)*= o;}
    Angle<T,PU,RT> operator/(T o) const {return Angle<T,PU,RT>(*this)/= o;}
    
    Angle<T,PU,RT> operator-() const {return Angle<T,PU,RT>(-_value);}

    friend Angle<T, PU, RT> min(const Angle<T, PU, RT>& a1, const Angle<T, PU, RT>& a2)
    {
      if(a1 < a2)
        return a1;
      return a2;
    }

    friend Angle<T, PU, RT> max(const Angle<T, PU, RT>& a1, const Angle<T, PU, RT>& a2)
    {
      if(a1 > a2)
        return a1;
      return a2;
    }
};

// period aware < operator, if the other value is more than half a period away, consider wrapping
template<typename T, typename PU, typename RT> bool operator<(Angle<T,PU,RT> l, Angle<T,PU,RT> r)
{
  return (r.value()-l.value() > 0.0 && r.value()-l.value() < PU::half_period || r.value()-l.value() < -PU::half_period);
}

template<typename T, typename LPU, typename LRT, typename RPU, typename RRT> bool operator<(Angle<T,LPU,LRT> l, Angle<T,RPU,RRT> r)
{
  return l < Angle<T,LPU,LRT>(r);
}

template<typename T, typename PU, typename RT> bool operator<(Angle<T,PU,rt::Unclamped> l, Angle<T,PU,RT> r)
{
  return Angle<T,PU,rt::PositivePeriod>(l) < r;
}

template<typename T, typename PU, typename RT> bool operator==(Angle<T,PU,RT> l, Angle<T,PU,RT> r)
{
  return l._value==r._value;
}

template<typename T, typename LPU, typename LRT, typename RPU, typename RRT> bool operator==(Angle<T,LPU,LRT> l, Angle<T,RPU,RRT> r)
{
  return l == Angle<T,LPU,LRT>(r);
}

template<typename T, typename PU, typename RT> bool operator==(Angle<T,PU,rt::Unclamped> l, Angle<T,PU,RT> r)
{
  return Angle<T,PU,rt::PositivePeriod>(l) == r;
}

template<typename T, typename PU, typename RT> bool isnan(const Angle<T, PU, RT>& angle)
{
    return std::isnan(angle.value());
}

// interpolates period aware angles
// a and b are the input angles, p is proportion of a vs b to weight.
// 
template <typename T, typename PU, typename RT> Angle<T,PU,RT> interpolate(Angle<T,PU,RT> const &a, Angle<T,PU,RT> const &b, double p=0.5)
{
    Angle<T,PU,RT> ret;
    if(a<b)
        ret =  a+(b-a)*(1.0-p);
    else
        ret = b+(a-b)*p;
    return ret;
}

template <typename T, typename RT> inline T sin(Angle<T,pu::Radian,RT> const &a)
{
    return ::sin(a.value());
}

template <typename T, typename PU, typename RT> inline T sin(Angle<T,PU,RT> const &a)
{
    return sin(Angle<T,pu::Radian,RT>(a));
}

template <typename T, typename RT> inline T cos(Angle<T,pu::Radian,RT> const &a)
{
    return ::cos(a.value());
}

template <typename T, typename PU, typename RT> inline T cos(Angle<T,PU,RT> const &a)
{
    return cos(Angle<T,pu::Radian,RT>(a));
}

template <typename T> inline T cos(T val)
{
    return ::cos(val);
}

template <typename T> inline T sin(T val)
{
    return ::sin(val);
}

// angle types that do not wrap
using AngleDegrees = Angle<double, pu::Degree, rt::Unclamped>;
using AngleRadians = Angle<double, pu::Radian, rt::Unclamped>;

// angle types that wrap at +/- half a circle
using AngleDegreesZeroCentered = Angle<double, pu::Degree, rt::ZeroCenteredPeriod>;
using AngleRadiansZeroCentered = Angle<double, pu::Radian, rt::ZeroCenteredPeriod>;

// angle types that wrap at 0 and full circle
using AngleDegreesPositive = Angle<double, pu::Degree, rt::PositivePeriod>;
using AngleRadiansPositive = Angle<double, pu::Radian, rt::PositivePeriod>;

} // namespace gz4d
} // namespace marine_nav_utilities 

#endif
