#ifndef MARINE_NAV_UTILITIES_GZ4D_WGS84_HPP
#define MARINE_NAV_UTILITIES_GZ4D_WGS84_HPP

#include "marine_nav_utilities/gz4d/ellipsoid.hpp"

namespace marine_nav_utilities
{
namespace gz4d
{
namespace geo
{

namespace WGS84
{

struct EllipsoidSpecs
{
  static constexpr double a = 6378137.0;
  static constexpr double b = 6356752.3142;
  static constexpr double f = 1.0/298.257223563;
  static constexpr double w = 7292115e-11;
  static constexpr double e2 = 1.0-(b*b)/(a*a);
};

using Ellipsoid = geo::Ellipsoid<EllipsoidSpecs>;

using LatLonRadians = ReferenceFrame<ct::Geodetic<cf::LatLon, pu::Radian>, Ellipsoid>;
using LonLatRadians = ReferenceFrame<ct::Geodetic<cf::LonLat, pu::Radian>, Ellipsoid>;
using LatLonDegrees = ReferenceFrame<ct::Geodetic<cf::LatLon, pu::Degree>, Ellipsoid>;
using LonLatDegrees = ReferenceFrame<ct::Geodetic<cf::LonLat, pu::Degree>, Ellipsoid>;
using ECEF = ReferenceFrame<ct::ECEF<>, Ellipsoid>;

} // namespace WGS84

} // namespace geo

using WGS84 = geo::WGS84::Ellipsoid;

using GeoPointLatLongDegrees = geo::Point<double, geo::WGS84::LatLonDegrees>;
using GeoPointLatLongRadians = geo::Point<double, geo::WGS84::LatLonRadians>;
using GeoPointECEF = geo::Point<double, geo::WGS84::ECEF>;
using LocalENU = geo::LocalENU<WGS84>;

using PositionDegrees = geo::Position<pu::Degree, geo::WGS84::Ellipsoid>;
using PositionRadians = geo::Position<pu::Radian, geo::WGS84::Ellipsoid>;
using BoundsDegrees = geo::Bounds<PositionDegrees>;


} // namespace gz4d

} // namespace marine_nav_utilities

#endif
