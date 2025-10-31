#ifndef MARINE_NAV_UTILITIES_GZ4D_TRANSFORMS_HPP
#define MARINE_NAV_UTILITIES_GZ4D_TRANSFORMS_HPP

#include "marine_nav_utilities/gz4d/matrix.hpp"
#include "tf2/LinearMath/Quaternion.h"

namespace marine_nav_utilities
{

namespace gz4d
{

template<typename T>
class Translation: public Point<T>
{
public:
  Translation();
  Translation(Vector<T,3> const & v);
  Translation(T x, T y, T z);
  Matrix<T,4,4> GetMatrix() const;
  Matrix<T,4,4> GetInverseMatrix() const;
  using Point<T>::operator=;
};

template<typename T> inline Translation<T>::Translation()
{}

template<typename T> inline Translation<T>::Translation(Vector<T,3> const & v)
:Point<T>(v)
{}

template<typename T> inline Translation<T>::Translation(T x, T y, T z)
:Point<T>(x,y,z)
{}

template<typename T> inline Matrix<T,4,4> Translation<T>::GetMatrix() const
{
    Matrix<T,4,4> ret = Matrix<T,4,4>::Identity();
    ret(0,3) = (*this)[0];
    ret(1,3) = (*this)[1];
    ret(2,3) = (*this)[2];
    return ret;
}

template<typename T> inline Matrix<T,4,4> Translation<T>::GetInverseMatrix() const
{
    Matrix<T,4,4> ret = Matrix<T,4,4>::Identity();
    ret(0,3) = -(*this)[0];
    ret(1,3) = -(*this)[1];
    ret(2,3) = -(*this)[2];
    return ret;
}






template<typename T>
class Rotation: public tf2::Quaternion
{
    public:
        template<typename T2> Rotation(Rotation<T2> const &r)
        :tf2::Quaternion(r)
        {}

        template<typename T2> Rotation(tf2::Quaternion const &q)
        :tf2::Quaternion(q)
        {}

        template<typename AT, typename RT> Rotation(Angle<AT,pu::Degree,RT> angle, Point<T> const &axis)
        {
            Rotation<T>::Set(Angle<AT,pu::Radian,RT>(angle),axis);
        }

        template<typename AT, typename RT> Rotation(Angle<AT,pu::Radian,RT> angle, Point<T> const &axis)
        {
            Rotation<T>::Set(angle ,axis);
        }
        
        
        template<typename T2> Rotation(Point<T2> const &v1, Point<T2> const &v2)
        {
            T2 n = norm(v1)*norm(v2);
            if(n > 0.0)
            {
                Angle<T2,pu::Radian,rt::Unclamped> angle  = acos(v1.dot(v2)/n);
                Point<T2> axis = cross(v1,v2);
                Rotation<T>::Set(angle,axis);
            }
        }


        Angle<T, pu::Radian, rt::Unclamped> angle() const
        {
            return acos(w())*2.0;
        }

        /// Sets rotation using angle around axis
        template<typename T2, typename PU, typename RT> void Set(Angle<T2,PU,RT> angle, Point<T2> const &axis)
        {
            if(norm2(axis) > 0.0)
            {
                Point<T2> a = normalize(axis) * sin(angle/2.0);
                *this = tf2::Quaternion(a[0],a[1],a[2],cos(angle/2.0));
            }
        }

        Point<T> operator()(Point<T> const &vector) const
        {
            tf2::Quaternion v(0.0,vector[0],vector[1],vector[2]);
            v = (*this) * v * conj(*this);
            return Point<T>(v.x(),v.y(),v.z());
        }

        Rotation<T> Inverse() const
        {
            return conj(*this);
        }

        Matrix<T,4,4> GetMatrix() const
        {
            Matrix<T,4,4> ret = Matrix<T,4,4>::Identity();

            T a = x();
            T b = y();
            T c = z();
            T d = w();
            T a2 = a*a;
            T b2 = b*b;
            T c2 = c*c;
            T d2 = d*d;

            ret(0,0) = a2 + b2 - c2 - d2;
            ret(1,0) = 2.0*(a*d + b*c);
            ret(2,0) = 2.0*(b*d - a*c);

            ret(0,1) = 2.0*(b*c - a*d);
            ret(1,1) = a2 - b2 + c2 - d2;
            ret(2,1) = 2.0*(a*b + c*d);

            ret(0,2) = 2.0*(a*c + b*d);
            ret(1,2) = 2.0*(c*d - a*b);
            ret(2,2) = a2 - b2 - c2 + d2;
            return ret;
        }

        Matrix<T,4,4> GetInverseMatrix() const
        {
            return Inverse().GetMatrix();
        }
};



}

}

#endif