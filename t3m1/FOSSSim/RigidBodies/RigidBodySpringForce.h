#ifndef __RIGID_BODY_SPRING_FORCE_H__
#define __RIGID_BODY_SPRING_FORCE_H__

#include <Eigen/Core>
#include <iostream>
#include "FOSSSim/MathDefs.h"
#include "RigidBodyForce.h"
#include "RigidBody.h"

class RigidBodySpringForce : public RigidBodyForce
{

public:

  // Inputs:
  //   k: scalar spring stiffness
  //   rb0: integer index of first rigid body
  //   vc0: body-space location of endpoint on first rigid body
  //   rb1: integer index of second rigid body
  //   vc1: body-space location of endpoint on second rigid body
  RigidBodySpringForce( const scalar& k, const scalar& l0, int rb0, const Vector2s& vc0, int rb1, const Vector2s& vc1 );
  
  virtual ~RigidBodySpringForce();

  // True if this force is conservative.
  virtual bool isConservative() const;

  // Returns the index of the first rigid body this force acts on, or -1 if the first endpoint is fixed in 'world space'.
  int getFirstRigidBody() const;
  // Returns the index of the second rigid body this force acts on, or -1 if the second endpoint is fixed in 'world space'.
  int getSecondRigidBody() const;

  // Returns the point in 'body space' the first endpoint is attached to, or the 'world space' position if the first endpoint is fixed.
  const Vector2s& getFirstEndpoint() const;
  // Returns the point in 'body space' the second endpoint is attached to, or the 'world space' position if the second endpoint is fixed.
  const Vector2s& getSecondEndpoint() const;
  
  // True if the first endpoint is fixed in space, false if attached to a rigid body
  bool firstEndpointIsFixed() const;
  
  // True if the second endpoint is fixed in space, false if attached to a rigid body
  bool secondEndpointIsFixed() const;

  // Computes the position of the first endpoint in 'world space'
  Vector2s computeFirstEndpoint( const std::vector<RigidBody>& rbs ) const;

  // Computes the position of the second endpoint in 'world space'
  Vector2s computeSecondEndpoint( const std::vector<RigidBody>& rbs ) const;

  // If this force is conservative, computes the potential energy given all rigid bodies in the scene.
  virtual scalar computePotentialEnergy( const std::vector<RigidBody>& rbs );
  
  // Adds this force's contribution to the total force and torque acting on each rigid body.
  virtual void computeForceAndTorque( std::vector<RigidBody>& rbs );

  // Creates a new copy of this force.
  virtual RigidBodyForce* createNewCopy();

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:

  // Stiffness of the spring
  scalar m_k;
  // Rest length of the spring
  scalar m_l0;
  // If -1, indicates that the first endpoint is fixed in space. Otherwise, 
  //  the index of the first rigid body the spring is attached to
  int m_rb0;
  // If -1, indicates that the second endpoint is fixed in space. Otherwise, 
  //  the index of the second rigid body the spring is attached to
  int m_rb1;
  // Either the position in 'world space' the endpoint is attached to if fixed,
  //  or the position in 'body space' on the first rigid body the spring is attached to
  Vector2s m_vc0;
  // Either the position in 'world space' the endpoint is attached to if fixed,
  //  or the position in 'body space' on the first rigid body the spring is attached to
  Vector2s m_vc1;

};

#endif
