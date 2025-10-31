#ifndef MARINE_NAV_UTILITIES_GZ4D_ELLIPSOID_HPP
#define MARINE_NAV_UTILITIES_GZ4D_ELLIPSOID_HPP


namespace marine_nav_utilities
{

namespace gz4d
{

namespace geo
{

// Static ellipsoid class, does not need to be instantiated to be used.
// EllipsoidSpecs determines the specs
template<typename EllipsoidSpecs>
class Ellipsoid
{
public:
  using EllipsoidSpecs = EllipsoidSpecs;

  using AzimuthType = Angle<double,pu::Radian,rt::PositivePeriod>;
  using AzimuthDistanceType = std::pair<AzimuthType, double>;

  /// Meridional radius of curvature.
  /// Radius of curvature in north-south direction.
  /// @param latitude Latitude in radians.
  static double M(double latitude)
  {
      return EllipsoidSpecs::a*(1.0-EllipsoidSpecs::e2)/pow((1.0-EllipsoidSpecs::e2)*pow(sin(latitude),2.0),3.0/2.0);
  }
  
  template<typename RT>
  static double M(Angle<double,pu::Radian,RT> latitude)
  {
    return M(latitude.value());
  }
  
  template<typename RT>
  static double M(Angle<double,pu::Degree,RT> latitude)
  {
    return M(Radians(latitude.value()));
  }

  /// Transverse radius of curvature.
  /// Radius of curvature in east-west direction.
  /// @param latitude Latitude in radians.
  static double N(double latitude)
  {
      if(EllipsoidSpecs::e2 == 0.0)
          return EllipsoidSpecs::a;
      return EllipsoidSpecs::a/sqrt(1-EllipsoidSpecs::e2*pow(sin(latitude),2.0));
  }
    
  template<typename RT>
  static double N(Angle<double,pu::Radian,RT> latitude)
  {
    return N(latitude.value());
  }

  template<typename RT>
  static double N(Angle<double,pu::Degree,RT> latitude)
  {
    return N(Radians(latitude.value()));
  }
    
  /// Calculate angle of longitude covered by distance in meters at given latitude in radians.
  /// From https://en.wikipedia.org/wiki/Longitude#Length_of_a_degree_of_longitude
  /// delta 1 long = (pi/180)a*cos(B) where tan(B) = (b/a)tan(phi) where B is reduced latitude
  static inline LongitudeSpanRadians longitudinalSpan(double latitude, double distance)
  {
    //U is 'reduced latitude'
    double tanU1 = (1.0-EllipsoidSpecs::f)*tan(latitude);
    double cosU1 = 1/sqrt(1+tanU1*tanU1);
    return LongitudeSpanRadians(distance/(EllipsoidSpecs::a*cosU1));
  }

  template<typename RT>
  static inline LongitudeSpanRadians longitudinalSpan(LatitudeRadians latitude, double distance)
  {
    return longitudinalSpan(latitude.value(), distance);
  }

  template<typename RT>
  static inline LongitudeSpanRadians longitudinalSpan(LatitudeDegrees latitude, double distance)
  {
    return longitudinalSpan(LatitudeRadians(latitude), distance);
  }

  /// Calculates approximate angle of latitude covered by distance in meters 
  /// along longitudinal lines at given latitude.
  /// https://en.wikipedia.org/wiki/Latitude#Length_of_a_degree_of_latitude
  /// The length of a small meridian arc is given by:
  /// delta m(phi) = M(phi)*delta phi = a(1-e2)((1-e2*sin(phi)^2)^(-3/2)) *delta phi
  static inline LatitudeSpanRadians latitudinalSpan(LatitudeRadians latitude, double distance)
  {
    return LatitudeSpanRadians(distance*pow(1.0-EllipsoidSpecs::e2*pow(sin(latitude),2),3.0/2.0)/(EllipsoidSpecs::a*(1-EllipsoidSpecs::e2)));
  }

  template<typename T, typename CF>
  static Point<double,ReferenceFrame<ct::ECEF<>, Ellipsoid<S> > > ToEarthCenteredEarthFixed(
    Point<T,ReferenceFrame<ct::Geodetic<CF, pu::Radian>, Ellipsoid<S> > > const &p
  )
  {
      double latr = p[CF::Latitude];
      double lonr = p[CF::Longitude];
      double height = p[CF::Height];
      double n = N(latr);
      return Point<double,ReferenceFrame<ct::ECEF<>, Ellipsoid<S> > >(
        (n+height)*cos(latr)*cos(lonr),
        (n+height)*cos(latr)*sin(lonr),
        (n*(1.0-EllipsoidSpecs::e2)+height)*sin(latr)
        );
  }

  template<typename T, typename CF, typename PU>
  static Point<double,ReferenceFrame<ct::ECEF<>, Ellipsoid<S> > > ToEarthCenteredEarthFixed
  (
    Point<T,ReferenceFrame<ct::Geodetic<CF, PU>, Ellipsoid<S> > > const &p
  )
  {
      return ToEarthCenteredEarthFixed(Point<T,ReferenceFrame<ct::Geodetic<CF, pu::Radian>, Ellipsoid<S> > >(p));
  }

  template <typename CF>
  static void ToGeodetic(
    Point<double,ReferenceFrame<ct::ECEF<>, Ellipsoid<S> > > const &p,
    Point<double,ReferenceFrame<ct::Geodetic<CF, pu::Degree>, Ellipsoid<S> > > &ret
  )
  {
    Point<double,ReferenceFrame<ct::Geodetic<CF, pu::Radian>, Ellipsoid<S> > > ret_rad;
    ToGeodetic(p, ret_rad);
    ret = ret_rad;
  }

  template <typename CF>
  static void ToGeodetic(
    Point<double,ReferenceFrame<ct::ECEF<>, Ellipsoid<S> > > const &p,
    Point<double,ReferenceFrame<ct::Geodetic<CF, pu::Radian>, Ellipsoid<S> > > &ret
  )
  {
    double ep2 = (EllipsoidSpecs::a*EllipsoidSpecs::a)/(EllipsoidSpecs::b*EllipsoidSpecs::b)-1.0;
    double r = sqrt(p[0]*p[0]+p[1]*p[1]);
    double E2 = EllipsoidSpecs::a*EllipsoidSpecs::a-EllipsoidSpecs::b*EllipsoidSpecs::b;
    double F = 54*EllipsoidSpecs::b*EllipsoidSpecs::b*p[2]*p[2];
    double G = r*r +(1.0-EllipsoidSpecs::e2)*p[2]*p[2]-EllipsoidSpecs::e2*E2;
    double C = (EllipsoidSpecs::e2*EllipsoidSpecs::e2*F*r*r)/(G*G*G);
    double s = pow(1.0+C+sqrt(C*C+2*C),1/3.0);
    double P = F/(3.0*pow((s+(1.0/s)+1.0),2.0)*G*G);
    double Q = sqrt(1.0+2.0*EllipsoidSpecs::e2*EllipsoidSpecs::e2*P);
    double r0 = (-(P*EllipsoidSpecs::e2*r)/(1.0+Q))+sqrt((1.0/2.0)*EllipsoidSpecs::a*EllipsoidSpecs::a*(1.0+1.0/Q)-((P*(1-EllipsoidSpecs::e2)*p[2]*p[2])/(Q*(1.0+Q)))-(1.0/2.0)*P*r*r);
    double U = sqrt(pow(r-EllipsoidSpecs::e2*r0,2.0)+p[2]*p[2]);
    double V = sqrt(pow(r-EllipsoidSpecs::e2*r0,2.0)+(1.0-EllipsoidSpecs::e2)*p[2]*p[2]);
    double Z0 = EllipsoidSpecs::b*EllipsoidSpecs::b*p[2]/(EllipsoidSpecs::a*V);

    ret[CF::Height] = U*(1.0-(EllipsoidSpecs::b*EllipsoidSpecs::b)/(EllipsoidSpecs::a*V));
    ret[CF::Latitude] = atan((p[2]+ep2*Z0)/r);
    ret[CF::Longitude] =atan2(p[1],p[0]);
  }

  /// Calculates the position P2 from azimuth and distance from P1 on the specified ellipsoid.
  /// @param p1 starting point
  /// @param azimuth clockwise angle in radians relative to north.
  /// @param distance distance in meters.
  template <typename CF>
  static Point<double,ReferenceFrame<ct::Geodetic<CF, pu::Radian>, Ellipsoid<S> > > direct(
    Point<double,ReferenceFrame<ct::Geodetic<CF, pu::Radian>,
    Ellipsoid<S> > > const &p1, Angle<double,pu::Radian,rt::Unclamped> azimuth, double distance
  )
  {
      double phi1 = p1[CF::Latitude];
      double alpha1 = static_cast<double>(azimuth);
      
      double epsilon = 1e-12;
      
      //U is 'reduced latitude'
      double tanU1 = (1.0-EllipsoidSpecs::f)*tan(phi1);
      double cosU1 = 1/sqrt(1+tanU1*tanU1);
      double sinU1 = tanU1*cosU1;

      double cosAlpha1 = cos(alpha1);
      double sinAlpha1 = sin(alpha1);

      double sigma1 = atan2(tanU1, cosAlpha1); // angular distance on sphere from equator to P1 along geodesic
      double sinAlpha = cosU1*sinAlpha1;
      double cos2Alpha = 1.0-sinAlpha*sinAlpha;
      
      double a = EllipsoidSpecs::a;
      double b = EllipsoidSpecs::b;
      double f = EllipsoidSpecs::f;

      double u2 = cos2Alpha*(a*a-b*b)/(b*b);

      double k1 = (sqrt(1.0+u2)-1.0)/(sqrt(1.0+u2)+1.0);
      double A = (1.0+k1*k1/4.0)/(1.0-k1);
      double B = k1*(1.0-3.0*k1*k1/8.0);

      double sigma = distance/(b*A);
      double last_sigma;
      double cos2Sigmam;
      while (true)
      {
          cos2Sigmam = cos(2.0*sigma1+sigma);
          double sinSigma = sin(sigma);
          double cosSigma = cos(sigma);

          double deltaSigma = B*sinSigma*(cos2Sigmam+.25*B*(cosSigma*(-1.0+2.0*cos2Sigmam*cos2Sigmam)-(B/6.0)*cos2Sigmam*(-3.0+4.0*sinSigma*sinSigma)*(-3.0+4.0*cos2Sigmam*cos2Sigmam)));
          last_sigma = sigma;
          sigma = (distance/(b*A))+deltaSigma;
          if (fabs(last_sigma-sigma) <= epsilon)
              break;
      }

      cos2Sigmam = cos(2.0*sigma1+sigma);

      double phi2 = atan2(sinU1*cos(sigma)+cosU1*sin(sigma)*cosAlpha1,(1-f)*sqrt(sinAlpha*sinAlpha+pow(sinU1*sin(sigma)-cosU1*cos(sigma)*cosAlpha1,2)));
      double l = atan2(sin(sigma)*sinAlpha1,cosU1*cos(sigma)-sinU1*sin(sigma)*cosAlpha1);
      double C = (f/16.0)*cos2Alpha*(4.0+f*(4.0-3.0*cos2Alpha));
      double L = l-(1.0-C)*f*sinAlpha*(sigma+C*sin(sigma)*(cos2Sigmam+C*cos(sigma)*(-1+2.0*cos2Sigmam*cos2Sigmam)));

      double lat2 = phi2;
      double lon2 = p1[CF::Longitude] + L;
      
      Point<double,ReferenceFrame<ct::Geodetic<CF, pu::Radian>, Ellipsoid<S> > > ret;
      ret[CF::Latitude] = lat2;
      ret[CF::Longitude] = lon2;
      ret[CF::Height] = p1[CF::Height];
      return ret;
  }

  template <typename CF, typename PU>
  static Point<double,ReferenceFrame<ct::Geodetic<CF, PU>, Ellipsoid<S> > > direct(
    Point<double,ReferenceFrame<ct::Geodetic<CF, PU>, Ellipsoid<S> > > const &p1,
    Angle<double,pu::Radian,rt::Unclamped> azimuth,
    double distance
  )
  {
    return direct(Point<double,ReferenceFrame<ct::Geodetic<CF, pu::Radian>, Ellipsoid<S> > >(p1), azimuth, distance);
  }


    /// Calculates the azimuth and distance from P1 to P2 on the WGS84 ellipsoid.
    /// @param p1: Position P1 in radians
    /// @param p2: Position P2 in radians
    /// @return: azimuth in radians, distance in meters
    template <typename CF>
    static azimuth_distance_type inverse(Point<double,ReferenceFrame<ct::Geodetic<CF, pu::Radian>, Ellipsoid<S> > > const &p1,Point<double,ReferenceFrame<ct::Geodetic<CF, pu::Radian>, Ellipsoid<S> > > const &p2)
    {
        if(p1 == p2)
            return std::pair<Angle<double,pu::Radian,rt::PositivePeriod>,double>(0.0,0.0);
        
        double a = EllipsoidSpecs::a;
        double b = EllipsoidSpecs::b;
        double f = EllipsoidSpecs::f;
        
        double epsilon = 1e-12;   
        
        double phi1 = p1[CF::Latitude];
        double phi2 = p2[CF::Latitude];
        
        double L = p2[CF::Longitude]-p1[CF::Longitude];

        double U1 = atan((1.0-f)*tan(phi1));
        double U2 = atan((1.0-f)*tan(phi2));
        double cosU1 = cos(U1);
        double cosU2 = cos(U2);
        double sinU1 = sin(U1);
        double sinU2 = sin(U2);

        double l = L;
        double last_l = std::nan("");
        double cosl;
        double sinl;
        double sinSigma;
        double cosSigma;
        double sigma;
        double cos2Alpha;
        double cos2Sigmam;

        while (true)
        {
            cosl = cos(l);
            sinl = sin(l);

            sinSigma = sqrt((pow((cosU2*sinl),2))+pow((cosU1*sinU2-sinU1*cosU2*cosl),2));
            cosSigma = sinU1*sinU2+cosU1*cosU2*cosl;
            sigma = atan2(sinSigma,cosSigma);
            double sinAlpha = (cosU1*cosU2*sinl)/sinSigma;

            cos2Alpha = 1-sinAlpha*sinAlpha;
            if (cos2Alpha == 0)
                cos2Sigmam = 0;
            else
                cos2Sigmam = cosSigma-((2.0*sinU1*sinU2)/cos2Alpha);

            if (!std::isnan(last_l) && fabs(last_l - l) <= epsilon)
                break;
            last_l = l;

            double C = (f/16.0)*cos2Alpha*(4.0+f*(4.0-3.0*cos2Alpha));
            l = L+(1.0-C)*f*sinAlpha*(sigma+C*sinSigma*(cos2Sigmam+C*cosSigma*(-1.0+2.0*pow(cos2Sigmam,2))));
        }

        double u2 = cos2Alpha*(a*a-b*b)/(b*b);
        double k1 = (sqrt(1.0+u2)-1.0)/(sqrt(1.0+u2)+1.0);
        double A = (1.0+k1*k1/4.0)/(1.0-k1);
        double B = k1*(1.0-3.0*k1*k1/8.0);
        double deltaSigma = B*sinSigma*(cos2Sigmam+.25*B*(cosSigma*(-1.0+2.0*cos2Sigmam*cos2Sigmam)-(B/6.0)*cos2Sigmam*(-3.0+4.0*sinSigma*sinSigma)*(-3.0+4.0*cos2Sigmam*cos2Sigmam)));
        double s = b*A*(sigma-deltaSigma);
        double alpha1 = atan2(cosU2*sinl,cosU1*sinU2-sinU1*cosU2*cosl);

        Angle<double,pu::Radian,rt::PositivePeriod> azimuth(alpha1);

        return std::pair<Angle<double,pu::Radian,rt::PositivePeriod>,double>(azimuth,s);
    }
    
    /// Calculates the azimuth and distance from P1 to P2 on the WGS84 ellipsoid.
    /// @param p1: Position P1
    /// @param p2: Position P2
    /// @return: azimuth in radians, distance in meters
    template <typename CF, typename PU>
    static azimuth_distance_type inverse(Point<double,ReferenceFrame<ct::Geodetic<CF, PU>, Ellipsoid<S> > > const &p1,Point<double,ReferenceFrame<ct::Geodetic<CF, PU>, Ellipsoid<S> > > const &p2)
    {
      return inverse(Point<double,ReferenceFrame<ct::Geodetic<CF, pu::Radian>, Ellipsoid<S> > >(p1), Point<double,ReferenceFrame<ct::Geodetic<CF, pu::Radian>, Ellipsoid<S> > >(p2));
    }

    static azimuth_distance_type inverse(const Position<pu::Radian, Ellipsoid<S> > &p1, const Position<pu::Radian, Ellipsoid<S> > &p2)
    {
      return inverse(Point<double,ReferenceFrame<ct::Geodetic<cf::LatLon, pu::Radian>, Ellipsoid<S> > >(p1.latitude, p1.longitude), Point<double,ReferenceFrame<ct::Geodetic<cf::LatLon, pu::Radian>, Ellipsoid<S> > >(p2.latitude, p2.longitude));
    }

  };



} // namespace geo


} // namespace gz4d

} // namespace marine_nav_utilities

#endif