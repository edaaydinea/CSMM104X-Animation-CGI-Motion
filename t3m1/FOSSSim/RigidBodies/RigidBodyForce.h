#ifndef __RIGID_BODY_FORCE_H__
#define __RIGID_BODY_FORCE_H__

#include <Eigen/Core>
#include <iostream>
#include "FOSSSim/MathDefs.h"
#include "RigidBody.h"

class RigidBodyForce
{
public:

  virtual ~RigidBodyForce();

  // If this force is conservative, computes the potential energy given all rigid bodies in the scene. 
  virtual scalar computePotentialEnergy( const std::vector<RigidBody>& rbs ) = 0;

  // Adds this force's contribution to the total force and torque acting on each rigid body.
  virtual void computeForceAndTorque( std::vector<RigidBody>& rbs ) = 0;

  // True if this force is conservative.
  virtual bool isConservative() const = 0;

  // Creates a new copy of this force.
  virtual RigidBodyForce* createNewCopy() = 0;

};

#endif
