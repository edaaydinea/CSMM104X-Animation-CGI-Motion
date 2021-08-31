#ifndef __RIGID_BODY_WIND_FORCE_H__
#define __RIGID_BODY_WIND_FORCE_H__

#include <Eigen/Core>
#include <iostream>
#include "FOSSSim/MathDefs.h"
#include "RigidBodyForce.h"

class RigidBodyWindForce : public RigidBodyForce
{

public:

  // Inputs:
  //   num_quad_points: number of points to break edges into when computing the force
  //   beta:            scalar to scale the magnitude of the force
  //   wind:            wind force
  RigidBodyWindForce( int num_quad_points, const scalar& beta, const Vector2s& wind );

  virtual ~RigidBodyWindForce();

  // True if this force is conservative.
  virtual bool isConservative() const;
  
  // If this force is conservative, computes the potential energy given all rigid bodies in the scene.
  virtual scalar computePotentialEnergy( const std::vector<RigidBody>& rbs );
  
  // Adds this force's contribution to the total force and torque acting on each rigid body.
  virtual void computeForceAndTorque( std::vector<RigidBody>& rbs );

  // Creates a new copy of this force.
  virtual RigidBodyForce* createNewCopy();
  
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:

  int m_num_quadrature_points;
  scalar m_beta;
  Vector2s m_wind;

};

#endif
