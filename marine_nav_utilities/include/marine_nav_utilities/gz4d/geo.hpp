#ifndef MARINE_NAV_UTILITIES_GZ4D_GEO_HPP
#define MARINE_NAV_UTILITIES_GZ4D_GEO_HPP

#include "marine_nav_utilities/gz4d/angles.hpp"

namespace marine_nav_utilities
{

namespace gz4d
{

namespace geo
{

template<typename PU>
using LatitudeType = Angle<double, PU, rt::ZeroCenteredHalfPeriodChecked>;

using LatitudeDegrees = LatitudeType<pu::Degree>;
using LatitudeRadians = LatitudeType<pu::Radian>;

template<typename PU>
using LatitudeSpan = Angle<double, PU, rt::Unclamped>;

using LatitudeSpanDegrees = LatitudeSpan<pu::Degree>;
using LatitudeSpanRadians = LatitudeSpan<pu::Radian>;

template<typename PU>
using LongitudeType = Angle<double, PU, rt::ZeroCenteredPeriod>;

using LongitudeDegrees = LongitudeType<pu::Degree>;
using LongitudeRadians = LongitudeType<pu::Radian>;

template<typename PU>
using LongitudeSpan = Angle<double, PU, rt::Unclamped>;

using LongitudeSpanDegrees = LongitudeSpan<pu::Degree>;
using LongitudeSpanRadians = LongitudeSpan<pu::Radian>;

// coordinate formats, defines order of components
namespace cf 
{
  struct LatLon
  {
      enum Coordinates
      {
          Latitude = 0, Longitude = 1, Height = 2
      };
  };

  struct LonLat
  {
      enum Coordinates
      {
          Longitude = 0, Latitude = 1, Height = 2
      };
  };

  struct XYZ
  {
      enum Coordinates
      {
          X = 0, Y = 1, Z = 2
      };
  };
}


/// 3D geographic point
/// T is numeric type of components
/// RF is the reference frame
template<typename T, typename RF> class Point: public gz4d::Point<T>
{
    public:

        Point(){}
        Point(T x, T y, T z=0):gz4d::Point<T>(x,y,z){}
        explicit Point(gz4d::Point<T> const &op):gz4d::Point<T>(op){}
        //using gz4d::Point<T>::operator=;

        //template<typename OT, typename ORF> explicit Point(Point<OT,ORF> const&op)
        template<typename OT, typename ORF> Point(Point<OT,ORF> const&op)
        {
            *this = typename RF::coordinate_type()(op);
        }
        
        T latitude() const {return gz4d::Point<T>::operator[](RF::coordinate_type::coordinate_format::Latitude);}
        T longitude() const {return gz4d::Point<T>::operator[](RF::coordinate_type::coordinate_format::Longitude);};
        T altitude() const {return gz4d::Point<T>::operator[](RF::coordinate_type::coordinate_format::Height);};
        
        T x() const {return gz4d::Point<T>::operator[](RF::coordinate_type::coordinate_format::X);}
        T y() const {return gz4d::Point<T>::operator[](RF::coordinate_type::coordinate_format::Y);}
        T z() const {return gz4d::Point<T>::operator[](RF::coordinate_type::coordinate_format::Z);}
        
        T &latitude() {return gz4d::Point<T>::operator[](RF::coordinate_type::coordinate_format::Latitude);}
        T &longitude() {return gz4d::Point<T>::operator[](RF::coordinate_type::coordinate_format::Longitude);};
        T &altitude() {return gz4d::Point<T>::operator[](RF::coordinate_type::coordinate_format::Height);};
        
        T &x() {return gz4d::Point<T>::operator[](RF::coordinate_type::coordinate_format::X);}
        T &y() {return gz4d::Point<T>::operator[](RF::coordinate_type::coordinate_format::Y);}
        T &z() {return gz4d::Point<T>::operator[](RF::coordinate_type::coordinate_format::Z);}

};
  
// define a reference frame
// CT is coordinate type
// ET is ellipsoid type
template <typename CT, typename ET> struct ReferenceFrame
{
    typedef CT coordinate_type;
    typedef ET ellipsoid_type;
};

// coordinate types, such as cartesian, geodetic (lat/long), etc.
namespace ct
{

template <typename CF> struct ECEF;

// Geodetic coordinate type
// CF determines order of lat and long
// PU is period units for angles
template <typename CF=cf::LatLon, typename PU=pu::Degree>
struct Geodetic
{
  typedef CF coordinate_format;
  typedef PU period_units;

  // convert from a different coordinate format
  template <typename T, typename OCF, typename ET> Point<T, ReferenceFrame<Geodetic<CF,PU>,ET> > operator()(Point<T, ReferenceFrame<Geodetic<OCF,PU>,ET> > const &p)
  {
      Point<T, ReferenceFrame<Geodetic<CF,PU>,ET> > ret;
      ret[CF::Latitude] = p[OCF::Latitude];
      ret[CF::Longitude] = p[OCF::Longitude];
      ret[CF::Height] = p[OCF::Height];
      return ret;
  }
  
  // convert from a different coordinate format and angular unit
  template <typename T, typename OCF, typename OPU, typename ET> Point<T, ReferenceFrame<Geodetic<CF,PU>,ET> > operator()(Point<T, ReferenceFrame<Geodetic<OCF,OPU>,ET> > const &p)
  {
      Point<T, ReferenceFrame<Geodetic<CF,PU>,ET> > ret;
      ret[CF::Latitude] = p[OCF::Latitude]*PU::period/OPU::period;
      ret[CF::Longitude] = p[OCF::Longitude]*PU::period/OPU::period;
      ret[CF::Height] = p[OCF::Height];
      return ret;
  }

  
  // convert from ECEF
  template <typename T, typename OCF, typename ET> Point<T, ReferenceFrame<Geodetic<CF,PU>,ET> > operator()(Point<T, ReferenceFrame<ECEF<OCF>,ET> > const &p) const
  {
    Point<T, ReferenceFrame<Geodetic<CF,PU>,ET> > ret;
    ET::ToGeodetic(p, ret);
    return ret;
  }
};

// Earth center earth fixed coordinate type
// CF determines order of cartesian coordinates
template <typename CF=cf::XYZ>
struct ECEF
{
    typedef CF coordinate_format;

    template <typename T, typename OCF, typename PU, typename ET> Point<T, ReferenceFrame<ECEF<CF>,ET> > operator()(Point<T, ReferenceFrame<Geodetic<OCF,PU>,ET> > const &p)
    {
        return ET::ToEarthCenteredEarthFixed(p);
    }
};

}


/// 2D geographic position
/// PU is period units, degrees, radians or other
/// ET is ellipsoid type
template<typename PU, typename ET>
struct Position
{
  using ellipsoid = ET;
  LatitudeType<PU> latitude;
  LongitudeType<PU> longitude;

  Position():
    latitude(std::nan("")), longitude(std::nan(""))
  {}

  Position(double latitude, double longitude):
    latitude(latitude), longitude(longitude)
  {}

  template <typename OPU>
  Position(const Position<OPU, ET>& other):
    latitude(other.latitude), longitude(other.longitude)
  {}

  Position(const Point<double, ReferenceFrame<ct::Geodetic<cf::LatLon, PU>, ET> > &point):
    latitude(point.latitude()), longitude(point.longitude())
  {}


  double distanceFrom(Position<PU, ET> other)
  {
    auto ad = ellipsoid::inverse(*this, other);
    return ad.second;
  }

  friend std::ostream& operator<< (std::ostream& stream, const Position<PU, ET>& p)
  {
    stream << "lat: " << p.latitude << " lon: " << p.longitude;
    return stream;
  }

  friend bool valid(const Position<PU, ET>& p)
  {
    return !std::isnan(p.latitude) && !std::isnan(p.longitude);
  }

  friend Position<PU, ET> min(const Position<PU, ET>& p1, const Position<PU, ET>& p2)
  {
    return Position<PU, ET>(min(p1.latitude, p2.latitude), min(p1.longitude, p2.longitude));
  }

  friend Position<PU, ET> max(const Position<PU, ET>& p1, const Position<PU, ET>& p2)
  {
    return Position<PU, ET>(max(p1.latitude, p2.latitude), max(p1.longitude, p2.longitude));
  }

};


/// @brief 2D bounding box for geographic coordinates
/// @tparam T Position type
template<typename T>
class Bounds
{
public:
  Bounds(){}

  Bounds(const T& p):
    min_(p), max_(p)
  {}

  Bounds(const T& p1, const T& p2):
    min_(p1), max_(p1)
  {
    expand(p2);
  }

  const T& minimum() const
  {
    return min_;
  }

  const T& maximum() const
  {
    return max_;
  }

  void expand(const T& position)
  {
    if(valid(position))
    {
      if(valid(*this))
      {
        min_ = min(min_, position);
        max_ = max(max_, position);
      }
      else
      {
        min_ = position;
        max_ = position;
      }
    }
  }

  static Bounds<T> radiusFromCenter(const T& center, double radius)
  {
    auto delta_lat = T::ellipsoid::latitudinalSpan(center.latitude, radius);
    auto delta_lon = T::ellipsoid::longitudinalSpan(center.latitude, radius);
    return Bounds<T>(T(center.latitude-delta_lat, center.longitude-delta_lon), T(center.latitude+delta_lat, center.longitude+delta_lon));
  }

  friend bool valid(const Bounds<T>& bounds)
  {
    return valid(bounds.min_) && valid(bounds.max_);
  }

  friend std::ostream& operator<< (std::ostream& stream, const Bounds<T>& bounds)
  {
    stream << "min: " << bounds.getMin() << " max: " << bounds.getMax();
    return stream;
  }
private:
  T min_;
  T max_;

};
  
template <typename ET>
class LocalENU
{
  Matrix<double, 4,4> transform;
  Matrix<double, 4,4> inverse;

  template <typename CF>
  void initialize(
    Point<double, ReferenceFrame<ct::Geodetic<CF, pu::Radian>, ET> > const &ref,
    Point<double, ReferenceFrame<ct::ECEF<>, ET> > const &refECEF
  )
  {
    double lat = ref[CF::Latitude];
    double lon = ref[CF::Longitude];

    transform(0,0) = -sin(lon);
    transform(0,1) = cos(lon);
    transform(0,2) = 0.0;
    transform(0,3) = 0.0;
    transform(1,0) = -sin(lat)*cos(lon);
    transform(1,1) = -sin(lat)*sin(lon);
    transform(1,2) = cos(lat);
    transform(1,3) = 0.0;
    transform(2,0) = cos(lat)*cos(lon);
    transform(2,1) = cos(lat)*sin(lon);
    transform(2,2) = sin(lat);
    transform(2,3) = 0.0;
    transform(3,0) = 0.0;
    transform(3,1) = 0.0;
    transform(3,2) = 0.0;
    transform(3,3) = 1.0;
    inverse = Translation<double>(refECEF).GetMatrix()*transpose(transform);
    transform = transform*Translation<double>(-refECEF).GetMatrix();
  }

public:
  using Ptr = std::shared_ptr<LocalENU<ET> >;

  LocalENU() = default;

  template <typename CF, typename PU>
  LocalENU(Point<double, ReferenceFrame<ct::Geodetic<CF, PU>, ET> > const &ref)
  {
    initialize(
      Point<double, ReferenceFrame<ct::Geodetic<CF, pu::Radian>, ET> >(ref),
      ET::ToEarthCenteredEarthFixed(ref)
    );
  }

  Matrix<double,4,4> GetMatrix() const
  {
      return transform;
  }

  Matrix<double,4,4> GetInverseMatrix() const
  {
      return inverse;
  }

  Point<double, ReferenceFrame<ct::ECEF<>, ET> > toECEF(gz4d::Point<double> const &p) const
  {
      Vector<double,4> t(p,0);
      t[3] = 1.0;
      t = inverse*t;
      return Point<double, ReferenceFrame<ct::ECEF<>, ET> >(Vector<double,3>(t,0));
  }

  Point<double, ReferenceFrame<ct::Geodetic<cf::LatLon, pu::Radian>, ET> > toLatLong(gz4d::Point<double> const &p) const
  {
      auto ecef = toECEF(p);
      Point<double, ReferenceFrame<ct::Geodetic<cf::LatLon, pu::Radian>, ET> > ret(ecef);
      return ret;
  }

  gz4d::Point<double> toLocal(Point<double, ReferenceFrame<ct::ECEF<>, ET> > const &p) const
  {
      Vector<double,4> t(p,0);
      t[3] = 1.0;
      t = transform*t;
      return Vector<double,3>(t,0);
  }
        
  template<typename CT>
  gz4d::Point<double> toLocal(Point<double, ReferenceFrame<CT, ET> > const &p) const
  {
    Point<double,ReferenceFrame<ct::ECEF<>, ET> > p_ecef(p);
    return toLocal(p_ecef);
  }

  std::vector<gz4d::Point<double> > toLocal(
    std::vector<Point<double, ReferenceFrame<ct::ECEF<>, ET> > > const &pv
  ) const
  {
    std::vector<gz4d::Point<double> > ret;
    for(const auto &p: pv)
      ret.push_back(toLocal(p));
    return ret;
  }
        
  Box2d toLonLatBox(const Box2d &local_box) const
  {
      Point<double, ReferenceFrame<ct::Geodetic<>, ET> > min(toECEF(Vector<double,3>(local_box.getMin(),0)));
      Point<double, ReferenceFrame<ct::Geodetic<>, ET> > max(toECEF(Vector<double,3>(local_box.getMax(),0)));
      return Box2d(Vector<double,2>(min[1],min[0]),Vector<double,2>(max[1],max[0]));
  }
};


} // namespace geo


} // namespace gz4d

} // namespace marine_nav_utilities

#endif
