#ifndef MARINE_NAV_UTILITIES_GZ4D_BOX_HPP
#define MARINE_NAV_UTILITIES_GZ4D_BOX_HPP

#include "marine_nav_utilities/gz4d/vector.hpp"

namespace marine_nav_utilities
{

namespace gz4d
{

template <typename T> class Box
{
public:
    typedef T value_type;
private:
    value_type min_;
    value_type max_;
public:
    Box(Box const &b):min_(b.min_),max_(b.max_){}
    Box(){min_ +=1;}
    //:min(T(typename T::value_type(1))),max(typename T::value_type(0)){}

    Box(T const &min, T const &max)
    :min_(min),max_(max){}

    bool operator!=(Box<T> const &other) const
    {
        return min_ != other.min_ || max_ != other.max_;
    }
    
    bool operator==(Box<T> const &other) const
    {
        return !((*this)==other);
    }
    
    T const &getMin() const {return min_;}
    T const &getMax() const {return max_;}

    void setMin(T const &m) {min_ = m;}
    void setMax(T const &m) {max_ = m;}

    bool empty() const
    {
        return std::isnan(min_[0]) || min_[0] > max_[0];
    }

    T getCenter() const
    {
        if(empty())
            return min_;
        T ret;
        for(std::size_t i = 0; i < T::size(); ++i)
            ret[i] = min_[i] + (max_[i] - min_[i]) *0.5;
        return ret;
    }

    T getSizes() const
    {
        //if(empty())
        //    throw(Exception("Empty box doesn't have a size"));

        T ret;
        for(std::size_t i = 0; i < T::size(); ++i)
            ret[i] = max_[i] - min_[i];
        return ret;
    }

    typename T::value_type getMaxLength() const
    {
        //if(empty())
        //    throw(Exception("Empty box doesn't have a length"));

        typename T::value_type ret = max_[0]-min_[0];
        for(std::size_t i = 1; i < T::size(); ++i)
            ret = std::max(ret,max_[i] - min_[i]);
        return ret;
    }

    typename T::value_type getMinLength() const
    {
        //if(empty())
        //    throw(Exception("Empty box doesn't have a length"));

        typename T::value_type ret = max_[0]-min_[0];
        for(std::size_t i = 1; i < T::size(); ++i)
            ret = std::min(ret,max_[i] - min_[i]);
        return ret;
    }
    
    typename T::value_type getVolume() const
    {
        //if(empty())
        //    throw(Exception("Empty box doesn't have a volume"));
        return getSizes().volume();
    }
    

    Box &expand(T const &p)
    {
        if(empty())
        {
            min_ = p;
            max_ = p;
        }
        else
            for(std::size_t i = 0; i < T::size(); ++i)
            {
                if(p[i] < min_[i])
                {
                    if(p[i] > max_[i])  // for types that can wrap, such as Angles
                    {
                        if(min_[i]-p[i]<p[i]-min_[i])
                            min_[i] = p[i];
                        else
                            max_[i] = p[i];
                    }
                    else
                        min_[i] = p[i];
                }
                else if(p[i] > max_[i])
                    max_[i] = p[i];
            }
        return *this;
    }

    Box &expand(Box<T> const &other)
    {
        expand(other.min_);
        expand(other.max_);
    }

    typename T::value_type distance(T const &p) const
    {
        if(contains(p))
            return 0;
        typename T::value_type d2 = 0;
        for(std::size_t i = 0; i < T::size(); ++i)
        {
            if(p[i] > max_[i])
                d2 += (p[i]-max_[i])*(p[i]-max_[i]);
            else if (p[i] < min_[i])
                d2 += (min_[i]-p[i])*(min_[i]-p[i]);
        }
        return sqrt(d2);
    }

    template <typename OT> bool contains(OT const &p) const
    {
        assert(T::size() == OT::size());
        if(empty())
            return false;
        for(std::size_t i = 0; i < T::size(); ++i)
            if(p[i] < min_[i] || p[i] > max_[i])
                return false;
        return true;
    }

    bool contains(Box<T> const &other) const
    {
        if(other.empty())
            return false;
        return contains(other.min_) && contains(other.max_);
    }

    bool intersects(Box<T> const &other) const
    {
        for(std::size_t i = 0; i < T::size(); ++i)
            if(min_[i] > other.max_[i] || max_[i] < other.min_[i])
                return false;
        return true;
    }

    void setSizesFromCenter(T const &s)
    {
        T c = getCenter();
        min_ = c-s/2.0;
        max_ = c+s/2.0;
    }

    void setSizesFromMin(T const &s)
    {
        if(empty())
            min_ = T(0);
        max_ = min_ + s;
    }
    
    Box operator&(Box const &o) const
    {
        if(intersects(o))
        {
            T newMin, newMax;
            for(std::size_t i = 0; i < T::size(); ++i)
            {
                newMin[i] = std::max(min_[i],o.min_[i]);
                newMax[i] = std::min(max_[i],o.max_[i]);
            }
            return Box(newMin,newMax);
        }
        return Box();
    }
    
    Box operator|(Box const &o) const
    {
        return Box(*this).expand(o);
    }
    
    Box &operator+=(const value_type &v)
    {
        min_ += v;
        max_ += v;
        return *this;
    }
    
    Box &operator-=(const value_type &v)
    {
        return this->operator+=(-v);
    }
    
    Box operator+(const value_type &v) const
    {
        return Box(*this)+=v;
    }

    Box operator-(const value_type &v) const
    {
        return Box(*this)-=v;
    }
};


using Box3f = Box< Vector<float,3> >;
using Box3d = Box< Vector<double,3> >;
using Box2f = Box< Vector<float,2> >;
using Box2d = Box< Vector<double,2> >;


} // namespace gz4d

} // namespace marine_nav_utilities

#endif