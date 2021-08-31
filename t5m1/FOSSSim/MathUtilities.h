#ifndef __MATH_UTILITIES_H__
#define __MATH_UTILITIES_H__

#include <Eigen/Core>
#include <Eigen/LU>
#include "MathDefines.h"

#define CLAMP(x, a, b)  ((x) < (a) ? (a) : ((x) > (b) ? (b) : (x)))

namespace mathutils
{
  // Determines if a scalar is a NAN or INF
  template <typename scalartype> inline
  bool isNAN( const scalartype& a )
  {
    return a != a;
  }

  template <typename scalartype> inline
  bool isINF( const scalartype& a )
  {
    return a == SCALAR_INFINITY || a == -SCALAR_INFINITY;
  }

  // Check for NaNs and INFs in eigen types
  template <typename Derived> inline
  bool containsNANs( const Eigen::MatrixBase<Derived>& a )
  {
    return (a.array()!=a.array()).any();
  }

  template <typename Derived> inline
  bool containsINFs( const Eigen::MatrixBase<Derived>& a )
  {
    return (a.array()==SCALAR_INFINITY).any() || (a.array()==-SCALAR_INFINITY).any();
  }

  template <typename Derived> inline
  bool containsNANs( const Eigen::ArrayBase<Derived>& a )
  {
    return (a!=a).any();
  }

  template <typename Derived> inline
  bool containsINFs( const Eigen::ArrayBase<Derived>& a )
  {
    return (a==SCALAR_INFINITY).any() || (a==-SCALAR_INFINITY).any();
  }

  // Check for NaNs and INFs in arrays
  template <typename scalartype>
  bool containsNANs( const int& N, const scalartype* a )
  {
    for( int i = 0; i < N; ++i ) if( isNAN(a[i]) ) return true;
    return false;
  }

  template <typename scalartype>
  bool containsINFs( const int& N, const scalartype* a )
  {
    for( int i = 0; i < N; ++i ) if( isINF(a[i]) ) return true;
    return false;
  }


  // Rounds a scalar to the nearest integer
  template <typename scalartype> inline
  int round( const scalartype& a )
  {
    return floor(a+0.5);
  }


  // Determine if two floats are within eps of one another
  template <typename scalartype> inline
  bool approxEqual( const scalartype& a, const scalartype& b, const scalartype& eps )
  {
    // Less than or equal so it reduces to equality if eps == 0.0
    return fabs(a-b) <= eps;
  }

//  bool approxSymmetric( const MatrixXs& A, const scalar& eps );


//template <typename Derived> inline
//bool approxEqual( const Eigen::MatrixBase<Derived>& a, const Eigen::MatrixBase<Derived>& b, const typename Derived::Scalar& eps )
//{
//  assert( a.rows() == b.rows() );
//  assert( a.cols() == b.cols() );
//  assert( eps >= 0.0 );
//
//  // Less than or equal so it reduces to equality if eps == 0.0
//  return ((a-b).array().abs()<=eps).all();
//}
//
//
//// NOTE: THIS METHOD IS SLOW
//bool isSingular( const MatrixXs& A );
//
//
//
//inline
//scalar crossTwoD( const Vector2s& a, const Vector2s& b )
//{
//  return a.x()*b.y()-a.y()*b.x();
//}
//
//inline
//Vector2s rotateCounterClockwise90degrees( const Vector2s& x )
//{
//  return Vector2s(-x.y(),x.x());
//}

}

#endif
