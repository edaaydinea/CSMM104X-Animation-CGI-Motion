#ifndef __RIGID_BODY_SCENE_H__
#define __RIGID_BODY_SCENE_H__

#include "FOSSSim/MathDefs.h"
#include <iostream>

#include "RigidBody.h"
#include "RigidBodyForce.h"

class RigidBodyScene
{

public:

  RigidBodyScene();

  RigidBodyScene( const RigidBodyScene& otherscene );

  ~RigidBodyScene();

  RigidBodyScene& operator=( const RigidBodyScene& rhs );

  // Adds a new force to the system
  void insertForce( RigidBodyForce* force );

  // Adds a new rigid body to the system
  void addRigidBody( const RigidBody& rb );

  // Returns a vector containing all forces in the system
  std::vector<RigidBodyForce*>& getForces();

  // Returns a vector containing all rigid bodies in the system
  const std::vector<RigidBody>& getRigidBodies() const;
  std::vector<RigidBody>& getRigidBodies();

  // Computes the total momentum of the system
  Vector2s computeTotalMomentum() const;

  // Computes the total angular momentum of the system with respect to the origin
  scalar computeTotalAngularMomentum() const;

  // Computes the kinetic energy using center of mass momentum and angular momentum
  scalar computeKineticEnergy() const;

  // Computes the total potential energy of the system. Note that if the system includes
  //  non-conservative forces, these are not included
  scalar computeTotalPotentialEnergy();

  // Writes the state of this simulation to the provided outputstream
  void serialize( std::ofstream& outputstream ) const;

  // Loads the state of this simulation from the provided outputstream
  void deserialize( std::ifstream& inputstream );

private:

  // Rigid bodies contained in the scene.
  std::vector<RigidBody> m_rbs;

  // Forces acting on rigid bodies. This class inherits responsibility for deleting forces.
  std::vector<RigidBodyForce*> m_forces;

};

#endif
