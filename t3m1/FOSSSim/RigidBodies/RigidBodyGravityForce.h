#ifndef __RIGID_BODY_GRAVITY_FORCE_H__
#define __RIGID_BODY_GRAVITY_FORCE_H__

#include <Eigen/Core>
#include <iostream>
#include "FOSSSim/MathDefs.h"
#include "RigidBodyForce.h"

class RigidBodyGravityForce : public RigidBodyForce
{

public:

  // Inputs:
  //   g: Two-dimensional gravity force
  RigidBodyGravityForce( const Vector2s& g );

  virtual ~RigidBodyGravityForce();

  // If this force is conservative, computes the potential energy given all rigid bodies in the scene.
  virtual scalar computePotentialEnergy( const std::vector<RigidBody>& rbs );

  // Adds this force's contribution to the total force and torque acting on each rigid body.
  virtual void computeForceAndTorque( std::vector<RigidBody>& rbs );

  // True if this force is conservative.
  virtual bool isConservative() const;

  // Creates a new copy of this force.
  virtual RigidBodyForce* createNewCopy();

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:

  Vector2s m_g;

};

#endif
