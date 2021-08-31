#ifndef __RIGID_BODY_COLLISION_H__
#define __RIGID_BODY_COLLISION_H__

#include "FOSSSim/MathDefs.h"
#include "FOSSSim/MathUtilities.h"

using mathutils::approxEqual;

class RigidBodyCollision
{
public:
  
  RigidBodyCollision( const int& i0, const int& i1, const Vector2s& r0, const Vector2s& r1, const Vector2s& nhat, const scalar& eqleps = 1.0e-9 )
  : i0(i0<i1?i0:i1) // Enforce that i0 is always the lower indexed body
  , i1(i0<i1?i1:i0)
  , r0(i0<i1?r0:r1)
  , r1(i0<i1?r1:r0)
  , nhat(i0<i1?nhat:(Vector2s)-nhat)
  , m_eps(eqleps)
  {}

  // Define == and < operators so we can insert collisions into sets

  bool operator==( const RigidBodyCollision& rhs ) const 
  {
    
    return i0==rhs.i0 && i1==rhs.i1 && 
           approxEqual(r0.x(),  rhs.r0.x(),m_eps)   && approxEqual(r0.y(),  rhs.r0.y(),m_eps) && 
           approxEqual(r1.x(),  rhs.r1.x(),m_eps)   && approxEqual(r1.y(),  rhs.r1.y(),m_eps) && 
           approxEqual(nhat.x(),rhs.nhat.x(),m_eps) && approxEqual(nhat.y(),rhs.nhat.y(),m_eps);
  }

  bool operator<( const RigidBodyCollision& rhs ) const 
  {
    if( i0 != rhs.i0 ) return i0 < rhs.i0;
    if( i1 != rhs.i1 ) return i1 < rhs.i1;
    if( !approxEqual(r0.x(),rhs.r0.x(),m_eps) )     return r0.x() < rhs.r0.x();
    if( !approxEqual(r0.y(),rhs.r0.y(),m_eps) )     return r0.y() < rhs.r0.y();
    if( !approxEqual(r1.x(),rhs.r1.x(),m_eps) )     return r1.x() < rhs.r1.x();
    if( !approxEqual(r1.y(),rhs.r1.y(),m_eps) )     return r1.y() < rhs.r1.y();
    if( !approxEqual(nhat.x(),rhs.nhat.x(),m_eps) ) return nhat.x() < rhs.nhat.x();
    if( !approxEqual(nhat.y(),rhs.nhat.y(),m_eps) ) return nhat.y() < rhs.nhat.y();
    return false;
  }  
  
  // Indices of the two rigid bodies 
  int i0;
  int i1;
  // Position of collision relative to center of mass on each rigid body
  Vector2s r0;
  Vector2s r1;
  // Collision normal
  Vector2s nhat;

  // Threshold to define to objects as equal
  scalar m_eps;
  
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

#endif
