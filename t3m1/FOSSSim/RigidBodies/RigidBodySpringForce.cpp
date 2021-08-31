#include "RigidBodySpringForce.h"

scalar RigidBodySpringForce::computePotentialEnergy( const std::vector<RigidBody>& rbs )
{
  assert( m_rb0 >= -1 ); assert( m_rb0 < (int) rbs.size() );
  assert( m_rb1 >= -1 ); assert( m_rb1 < (int) rbs.size() );
  
  // Your code goes here!
  
  Vector2s rb0 = (firstEndpointIsFixed) ? m_vc0: rbs[m_rb0].computeWorldSpacePosition(m_vc0);
  Vector2s rb1 = (secondEndpointIsFixed)? m_vc1: rbs[m_rb1].computeWorldSpacePosition(m_vc1);
  Vector2s spring_vector = rb1 - rb0;

  scalar potential_energy = 0.5 * m_k * pow(spring_vector.norm() - m_l0, 2.0);

  return potential_energy;
}

void RigidBodySpringForce::computeForceAndTorque( std::vector<RigidBody>& rbs )
{
  assert( m_rb0 >= -1 ); assert( m_rb0 < (int) rbs.size() );
  assert( m_rb1 >= -1 ); assert( m_rb1 < (int) rbs.size() );
  
  // Your code goes here!
  // for all rigid bodies i rbs[i].getForce()  += ... some force you compute ...
  //                        rbs[i].getTorque() += ... some torque you compute ...

  Vector2s rb0 = (firstEndpointIsFixed)? m_vc0: rbs[m_rb0].rotateIntoWorldSpace(m_vc0);
  Vector2s rb1 = (secondEndpointIsFixed)? m_vc1: rbs[m_rb1].rotateIntoWorldSpace(m_vc1);  
  Vector2s spring_vector = rb1 - rb0;

  if(!firstEndpointIsFixed)
      spring_vector -= rbs[m_rb0].getX();

  if(!secondEndpointIsFixed)
      spring_vector += rbs[m_rb1].getX();
  
  // compute the length
  scalar length = spring_vector.norm();
  
  // no force
  if(m_l0 == 0.0 && length == 0.0)
      return;
  assert(length != 0.0);

  // compute the force
  Vector2s n_hat = spring_vector/length;

  Vector2s spring_force = m_k * (length - m_l0) * n_hat;

  if(!firstEndpointIsFixed) 
  {
      rbs[m_rb0].getForce() += spring_force;
      rbs[m_rb0].getTorque() += rb0.x() * spring_force.y() - rb0.y() * spring_force.x();
  }
  if(!secondEndpointIsFixed)
  {
      rbs[m_rb1].getForce() -= spring_force;
      rbs[m_rb1].getTorque() += rb1.y() * spring_force.x() - rb1.x() * spring_force.y();
  }
  return;

  return;
  
}
