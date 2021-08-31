#ifndef __MATH_UTILITIES_H__
#define __MATH_UTILITIES_H__

#include <Eigen/Core>
#include <Eigen/LU>
#include "MathDefs.h"

#define PI 3.1415926535897932384626433832795028841971693993751058209749445923078164062862089986280348253421170679821480865132823066470938446095505822317253594081284811174502841027019385211055596446229489549303819644288109756659334461284756482337867831652712019091456485669234603486104543266482133936072602491412737245870066063155881748815209209628292540917153643678925903600113305305488204665213841469519415116094330572703657595919530921861173819326117931051185480744623799627495673518857527248912279381830119491298336733624406566430860213949463952247371907021798609437027705392171762931767523846748184676694051320005681271452635608277857713427577896091736371787214684409012249534301465495853710507922796892589235420199561121290219608640344181598136297747713099605187072113499999983729780499510597317328160963185950244594553469083026425223082533446850352619311881710100031378387528865875332083814206171776691473035982534904287554687311595628638823537875937519577818577805321712268066130019278766111959092164201989

namespace mathutils
{

bool approxSymmetric( const MatrixXs& A, const scalar& eps );

template <typename scalartype> inline
bool approxEqual( const scalartype& a, const scalartype& b, const scalartype& eps )
{
  // Less than or equal so it reduces to equality if eps == 0.0
  return fabs(a-b) <= eps;
}

template <typename Derived> inline
bool approxEqual( const Eigen::MatrixBase<Derived>& a, const Eigen::MatrixBase<Derived>& b, const typename Derived::Scalar& eps )
{
  assert( a.rows() == b.rows() );
  assert( a.cols() == b.cols() );
  assert( eps >= 0.0 );

  // Less than or equal so it reduces to equality if eps == 0.0
  return ((a-b).array().abs()<=eps).all();
}

template <typename Derived> inline
bool containsNANs( const Eigen::MatrixBase<Derived>& a )
{
  return (a.array()!=a.array()).any();
}

template <typename Derived> inline
bool containsINFs( const Eigen::MatrixBase<Derived>& a )
{
  return (a.array()==std::numeric_limits<typename Derived::Scalar>::infinity()).any();
}

// NOTE: THIS METHOD IS SLOW
bool isSingular( const MatrixXs& A );



inline
scalar crossTwoD( const Vector2s& a, const Vector2s& b )
{
  return a.x()*b.y()-a.y()*b.x();
}

inline
Vector2s rotateCounterClockwise90degrees( const Vector2s& x )
{
  return Vector2s(-x.y(),x.x());
}

}

#endif
