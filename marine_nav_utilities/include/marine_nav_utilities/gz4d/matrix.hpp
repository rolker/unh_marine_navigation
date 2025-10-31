#ifndef MARINE_NAV_UTILITIES_GZ4D_MATRIX_HPP
#define MARINE_NAV_UTILITIES_GZ4D_MATRIX_HPP

#include "marine_nav_utilities/gz4d/vector.hpp"

namespace marine_nav_utilities
{
namespace gz4d
{ 

/// Column-major matrix with M rows and N columns.
/// \ingroup base
template <typename T, std::size_t M, std::size_t N> class Matrix
{
    Vector<T,M*N> values;
    Matrix(Vector<T,M*N> const &v):values(v){}
    public:
        Matrix(){}
        Matrix(Matrix<T,M,N> const &m):values(m.values){}
        explicit Matrix(T val):values(val){}
        template <typename VI> Matrix(const std::pair<VI,VI> &iterators):values(iterators){}
        template <std::size_t CM, std::size_t CN> Matrix(Matrix<T,CM,CN> const &cm, std::size_t row, std::size_t col);

        static Matrix<T,M,N> Identity();

        T const &operator()(std::size_t row, std::size_t col) const {return values[col*M+row];}
        T &operator()(std::size_t row, std::size_t col) {return values[col*M+row];}

        template <std::size_t P> Matrix<T,M,P> operator*(Matrix<T,N,P> const &rvalue) const;
        Vector<T,M> operator*(Vector<T,N> const &rvalue) const;

        T &front() {return values.front();}
        T const &front() const {return values.front();}

        Matrix<T,M,N> operator-(Matrix const &o) const {return Matrix<T,M,N>(values-o.values);}
        Matrix<T,M,N> operator+(Matrix const &o) const {return Matrix<T,M,N>(values+o.values);}

        Matrix<T,M,N> operator-() const {return Matrix<T,M,N>(-values);}

        /// Element-wise arithmetic
        Matrix<T,M,N> &operator*=(T value) { values*=value; return *this;}
        Matrix<T,M,N> operator*(T value) const { return Matrix<T,M,N>(*this)*=value;}

        Matrix<T,M,N> &operator/=(T value) { values/=value; return *this;}
        Matrix<T,M,N> operator/(T value) const { return Matrix<T,M,N>(*this)/=value;}

        Matrix<T,M,N> &operator+=(T value) { values+=value; return *this;}
        Matrix<T,M,N> operator+(T value) const { return Matrix<T,M,N>(*this)+=value;}
        
        Matrix<T,M,N> &operator-=(T value) { values-=value; return *this;}
        Matrix<T,M,N> operator-(T value) const { return Matrix<T,M,N>(*this)-=value;}

        bool operator!=(Matrix const &o) const {return values != o.values;}
        bool operator==(Matrix const &o) const {return !(*this != o);}
};

template <typename T, std::size_t M, std::size_t N> inline Matrix<T,M,N> Matrix<T,M,N>::Identity()
{
    Matrix<T,M,N> ret;
    for(std::size_t i = 0; i < N && i < M; ++i)
        ret.values[i*M+i] = 1;
    return ret;
}

template <typename T, std::size_t M, std::size_t N> template <std::size_t P> inline Matrix<T,M,P> Matrix<T,M,N>::operator*(Matrix<T,N,P> const &rvalue) const 
{
    Matrix<T,M,P> ret;
    for(std::size_t r = 0; r < M; ++r)
        for(std::size_t c = 0; c < P; ++c)
            for(std::size_t i = 0; i < N; ++i)
                ret(r,c) += operator()(r,i)*rvalue(i,c);
    return ret;
}

template <typename T, std::size_t M, std::size_t N> inline Vector<T,M> Matrix<T,M,N>::operator*(Vector<T,N> const &rvalue) const
{
    Vector<T,M> ret;
    for(std::size_t r = 0; r < M; ++r)
        for(std::size_t c = 0; c < N; ++c)
            ret[r] += operator()(r,c)*rvalue[c];
    return ret;
}

template <typename T, std::size_t M, std::size_t N> template <std::size_t CM, std::size_t CN> inline Matrix<T,M,N>::Matrix(Matrix<T,CM,CN> const &cm, std::size_t row, std::size_t col)
{
    for(std::size_t r = 0; r < M; ++r)
        for(std::size_t c = 0; c < N; ++c)
            values[c*M+r] = cm(row+r,col+c);
}

template <typename T> static Matrix<T,4,4> Frustum(T l, T r, T b, T t, T n, T f=std::numeric_limits<T>::infinity())
{
    Matrix<T,4,4> ret(0);
    T w = r-l;
    T h = t - b;
    ret(0,0) = 2.0*n/w;
    ret(0,2) = (r+l)/w;
    ret(1,1) = 2.0*n/h;
    ret(1,2) = (t+b)/h;
    ret(2,2) = -(f+n)/(f-n);
    //ret(2,2) = -1.0;
    ret(2,3) = -2.0*f*n/(f-n);
    //ret(2,3) = -2.0*n;
    ret(3,2) = -1.0;
    return ret;
}

template <typename T, std::size_t M, std::size_t N> Matrix<T,N,M> transpose(Matrix<T,M,N> const &m)
{
    Matrix<T,N,M> ret;
    for(std::size_t r = 0; r < M; ++r)
        for(std::size_t c = 0; c < N; ++c)
            ret(c,r) = m(r,c);
    return ret;
}

template <typename T> inline Matrix<T,1,1> inverse(Matrix<T,1,1> const &m)
{
    return Matrix<T,1,1>(1.0/m(0,0));
}

template <typename T> inline T determinant(Matrix<T,2,2> const &m)
{
    return m(0,0)*m(1,1)-m(0,1)*m(1,0);
}

template <typename T> inline T determinant(Matrix<T,3,3> const &m)
{
    return m(0,0)*m(1,1)*m(2,2)+m(0,1)*m(1,2)*m(2,0)+m(0,2)*m(1,0)*m(2,1)-m(0,0)*m(1,2)*m(2,1)-m(0,1)*m(1,0)*m(2,2)-m(0,2)*m(1,1)*m(2,0);
}

template <typename T, std::size_t N> inline T determinant(Matrix<T,N,N> const &m)
{
    T ret = 0;
    for(std::size_t i = 0; i < N; ++i)
        ret += m(i,0)*cofactor(m,i,0);
    return ret;
}

/// Calculates Cij of matrix m.
/// http://en.wikipedia.org/wiki/Cofactor_(linear_algebra)
/// Cij=(-1)^(i+j)*Mij
/// Where Mij is determinant of the submatrix obtained by removing from m its i-th row and j-th column.
template <typename T, std::size_t N> inline T cofactor(Matrix<T,N,N> const &m, std::size_t i, std::size_t j)
{
    Matrix<T,N-1,N-1> minor_matrix;
    std::size_t mi = 0;
    for(std::size_t in_row = 0; in_row < N && mi < N-1; ++in_row)
    {
        std::size_t mj = 0;
        for(std::size_t in_col = 0; in_col < N && mj < N-1; ++in_col)
        {
            minor_matrix(mi,mj)=m(in_row,in_col);
            if(in_col != j)
                ++mj;
        }
        if (in_row != i)
            ++mi;
    }
    T ret = determinant(minor_matrix);
    if((i+j)%2==1)
        return -ret;
    return ret;
}

/// Transpose of the cofactor matrix.
/// http://en.wikipedia.org/wiki/Adjugate_matrix
template <typename T, std::size_t N> inline Matrix<T,N,N> adjugate(Matrix<T,N,N> const &m)
{
    Matrix<T,N,N> ret;
    for(std::size_t i = 0; i < N; ++i)
        for(std::size_t j = 0; j < N; ++j)
            ret(i,j)=cofactor(m,j,i); // i and j reversed so we get transpose of cofactor matrix
    return ret;
}


template <typename T> inline Matrix<T,2,2> inverse(Matrix<T,2,2> const &m)
{
    Matrix<T,2,2> ret;
    T a = m(0,0);
    T b = m(0,1);
    T c = m(1,0);
    T d = m(1,1);
    ret(0,0) = d/(a*d-b*c);
    ret(0,1) = -b/(a*d-b*c);
    ret(1,0) = -c/(a*d-b*c);
    ret(1,1) = a/(a*d-b*c);
    return ret;
}

/// Invert a matrix using Cramer's rule.
/// http://en.wikipedia.org/wiki/Invertible_matrix
/// inverse of A is:
/// (1/det(A))*transpose(C)
/// where C is matrix of cofactors. 
template <typename T, std::size_t N> inline Matrix<T,N,N> inverse_cramer(Matrix<T,N,N> const &m)
{
    return adjugate(m)/determinant(m);
}

template <typename T> inline Matrix<T,4,4> inverse(Matrix<T,4,4> const &m)
{
    // Attempt Blockwise inversion first
    Matrix<T,2,2> A(m,0,0);
    Matrix<T,2,2> B(m,0,2);
    Matrix<T,2,2> C(m,2,0);
    Matrix<T,2,2> D(m,2,2);
    
    if(determinant(A) != 0.0)
    {

        Matrix<T,2,2> A_inv = inverse(A);

        Matrix<T,2,2> tmp = D-C*A_inv*B;
        if(determinant(tmp) != 0.0)
        {
            Matrix<T,2,2> D_ret = inverse(tmp);

            Matrix<T,2,2> A_ret = A_inv+A_inv*B*D_ret*C*A_inv;
            Matrix<T,2,2> B_ret = -A_inv*B*D_ret;
            Matrix<T,2,2> C_ret = -D_ret*C*A_inv;

            Matrix<T,4,4> ret;
            for(std::size_t r = 0; r < 2; ++r)
                for(std::size_t c = 0; c < 2; ++c)
                {
                    ret(r,c) = A_ret(r,c);
                    ret(r,c+2) = B_ret(r,c);
                    ret(r+2,c) = C_ret(r,c);
                    ret(r+2,c+2) = D_ret(r,c);
                }
            return ret;
        }
    }
    
    // Blockwise didn't work, try Cramer's rule
    return inverse_cramer(m);
}


} // namespace gz4d
} // namespace marine_nav_utilities



#endif