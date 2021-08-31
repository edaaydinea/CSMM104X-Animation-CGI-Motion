#ifndef __RIGID_BODY_VELOCITY_PROJECTION_COLLISION_RESOLVER_H__
#define __RIGID_BODY_VELOCITY_PROJECTION_COLLISION_RESOLVER_H__

#include "RigidBodyCollisionResolver.h"

#include <iostream>
#include <vector>
#include <iomanip>
#include "FOSSSim/MathDefs.h"

#include "LCPSolver/ODELCPSolver.h"
#include <Eigen/Dense>
#include "FOSSSim/quadprog/QuadProg++.hh"

class RigidBodyVelocityProjectionCollisionResolver : public RigidBodyCollisionResolver
{
public:

  virtual void resolveCollisions( std::vector<RigidBody>& rbs, const std::set<RigidBodyCollision>& rbcs );

private:

  virtual std::string getName() const;
};

#endif
