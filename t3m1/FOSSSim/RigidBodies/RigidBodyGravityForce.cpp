#include "RigidBodyGravityForce.h"

scalar RigidBodyGravityForce::computePotentialEnergy( const std::vector<RigidBody>& rbs )
{
    // Your code goes here!
    scalar U_body = 0.0;
    for (std::vector<RigidBody>::size_type i = 0 ; i < rbs.size(); ++i)
    {
        U_body -= rbs[i].getM() * m_g.dot(rbs[i].getX());
    }

    return U_body;
    
}

void RigidBodyGravityForce::computeForceAndTorque( std::vector<RigidBody>& rbs )
{
    // Your code goes here!
    // for all rigid bodies i rbs[i].getForce()  += ... some force you compute ...
    //                        rbs[i].getTorque() += ... some torque you compute ...

    for (std::vector<RigidBody>::size_type i = 0; i < rbs.size(); ++i)
    {
        rbs[i].getForce() += rbs[i].getM() * m_g;

        // There is no torque because gradient of angular velocity with respect to velocity is zero. 
        // We don't calculate this force.
    }

    return;
}
