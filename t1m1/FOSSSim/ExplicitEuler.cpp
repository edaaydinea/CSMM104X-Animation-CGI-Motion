#include "ExplicitEuler.h"

bool ExplicitEuler::stepScene( TwoDScene& scene, scalar dt )
{
    // Your code goes here!
    
    // Some tips on getting data from TwoDScene:
    // A vector containing all of the system's position DoFs. x0, y0, x1, y1, ...
    //VectorXs& x = scene.getX();
    // A vector containing all of the system's velocity DoFs. v0, v0, v1, v1, ...
    //VectorXs& v = scene.getV();
    // A vector containing the masses associated to each DoF. m0, m0, m1, m1, ...
    //const VectorXs& m = scene.getM();
    // Determine if the ith particle is fixed
    // if( scene.isFixed(i) )
  
  // x means system's position vector
  VectorXs& x = scene.getX();
  // v means system's velocity
  VectorXs& v = scene.getV();
  // m means system's masses
  const VectorXs& m = scene.getM();
  
  // f means Force vector (same size as x and v)
  VectorXs F(x.size());
  
  // initial value of Force vector to be zero
  F.setZero();
  scene.accumulateGradU(F);
  
  // The energy gradient
  F = F * -1.0;
  
  // Determine if the ith particle is fixed
  for(int i=0; i<scene.getNumParticles(); i++)
  {
    if(scene.isFixed(i))
    {
      F.segment<2>(2*i).setZero();
    }
  }
  
  // New position for the next step
  x = x + (dt * v);
  
  // Normalize the Force vector
  F.array() = F.array() / m.array();
  
  // New velocity for the next step
  v = v + (dt * F);
  
    return true;
}

