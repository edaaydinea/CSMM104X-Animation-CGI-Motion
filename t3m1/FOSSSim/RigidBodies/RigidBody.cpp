#include "RigidBody.h"

Vector2s RigidBody::computeTotalMomentum() const
{
  // Your code goes here!
  return m_M * m_V;
}

scalar RigidBody::computeCenterOfMassAngularMomentum() const
{
  // Your code goes here!
  Vector2s total_momentum = computeTotalMomentum();
  return m_X(0) * total_momentum(1) - m_X(1) * total_momentum(0);
}

scalar RigidBody::computeSpinAngularMomentum() const
{
  // Your code goes here!
  return m_I * m_omega;
}


scalar RigidBody::computeCenterOfMassKineticEnergy() const
{
  // Your code goes here!
  return 0.5 * m_M * m_V.squaredNorm();
}

scalar RigidBody::computeSpinKineticEnergy() const
{
  // Your code goes here!
  return 0.5 * m_I * m_omega * m_omega;
}

scalar RigidBody::computeTotalMass( const VectorXs& masses ) const
{
    // Your code goes here!
    scalar total_mass = 0;
    
    for (int i = 0; i <masses.size(); ++i)
    {
        total_mass = total_mass + masses(i);
    }
    
    return total_mass;
}

Vector2s RigidBody::computeCenterOfMass( const VectorXs& vertices, const VectorXs& masses ) const
{
    // Your code goes here!
    assert(vertices.size() % 2 == 0);
    assert(2 * masses.size() == vertices.size());
    
    //scalar total_mass = 0.0;
    //Vector2s center_of_mass = Vector2s::Zero();
    Vector2s center_of_mass;
    center_of_mass.setZero();
    
    for (int i = 0; i < masses.size(); ++i)
    {
        center_of_mass = center_of_mass + vertices.segment<2>(2 * i) * masses(i);
        //total_mass = total_mass + masses(i);
    }
    
    return center_of_mass * (1.0 / m_M);
}

scalar RigidBody::computeMomentOfInertia( const VectorXs& vertices, const VectorXs& masses ) const
{
    assert( vertices.size() % 2 == 0 );
    assert( 2*masses.size() == vertices.size() );

    // Your code goes here!
    scalar inertia = 0.0;
    //Vector2s center_of_mass = computeCenterOfMass(vertices, masses);
    
    for (int i = 0; i < masses.size(); ++i)
    {
      //Vector2s temporary = vertices.segment<2>(2 * i) - center_of_mass;
      //inertia = inertia + masses(i) * temporary.dot(temporary);
      inertia = inertia + masses(i)*(m_X - vertices.segment<2>(2*i)).squaredNorm();
    }
    
    return inertia;
}
