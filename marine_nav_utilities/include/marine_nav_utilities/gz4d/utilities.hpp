#ifndef MARINE_NAV_UTILITIES_GZ4D_UTILITIES_HPP
#define MARINE_NAV_UTILITIES_GZ4D_UTILITIES_HPP

// Roland Arsenault
// Center for Coastal and Ocean Mapping
// University of New Hampshire
// Copyright 2017, All rights reserved.
//
// Condensed from libgz4d.




namespace marine_nav_utilities
{
namespace gz4d
{
  /// Weighted interpolation between two values.
  /// @param start First value.
  /// @param end Second value.
  /// @param p Proportion of b relative to a.
  /// @return Weighted interpolated value.
  template <typename T> inline T interpolate(T const &start, T const &end, double p)
  {
      return (1.0-p)*start + p*end;
  }

  /// Interpolates two angles, using the shorter distance between them.
  /// With a weight of 0, degree1 is essentially returned. degree2 is returned when weight is 1.
  /// A weight of 0.5 returns the average of the two angles, or the mid point between them.
  /// @param a First angle.
  /// @param b Second angle.
  /// @param p Proportion of b relative to a.
  /// @return Weighted interpolated value.
  template <typename T> inline T InterpolateDegrees(T a, T b, T p = .5)
  {
      while(a < b-180.0)
          a+=360.0;
      while(a > b+180.0)
          a-=360.0;

      return interpolate(a,b,p);
  }

  template <typename T> inline double ratio(T const &a, T const &b) {return a/b;}

  template <typename T> inline bool IsEven(T i)
  {
      return !(i%2);
  }

  /// Used by std::shared_ptr's to hold pointers it shouldn't auto-delete.
  struct NullDeleter
  {
      void operator()(void const *) const {}
  };

  template <typename T> inline T Radians(T degrees) {return degrees*0.01745329251994329577;}
  template <typename T> inline T Degrees(T radians) {return radians*57.2957795130823208768;}
    
  /// Use with lexical_cast to convert hex string to integer type.
  /// From: http://stackoverflow.com/questions/1070497/c-convert-hex-string-to-signed-integer
  /// Example: uint32_t value = boost::lexical_cast<HexTo<uint32_t> >("0x2a");
  template <typename ElemT>
  struct HexTo {
      ElemT value;
      operator ElemT() const {return value;}
      friend std::istream& operator>>(std::istream& in, HexTo& out) {
          in >> std::hex >> out.value;
          return in;
      }
  };



    

    

    template<typename vType, typename rType> class ValueScaler
    {
        double scale;
        double offset;
        public:
            ValueScaler():scale(1.0),offset(0.0){}
            ValueScaler(ValueScaler const &v):scale(v.scale),offset(v.offset){}
            ValueScaler(double s, double o):scale(s),offset(o){}
            vType Value(rType const &r) const {return r*scale+offset;}
            rType Representation(vType const &v) const {return (v-offset)/scale;}
    };


    template<typename T> class Interval
    {
        T start;
        T end;
        public:
            Interval():start(0.0),end(1.0){}
            Interval(Interval const &i):start(i.start),end(i.end){}
            Interval(T s, T e):start(s),end(e){}

            T GetRange() const {return end-start;}
            T GetStart() const {return start;}
            T GetEnd() const {return end;}
            T Map(double p) const {return start+p*GetRange();}
    };

}

}

#endif
