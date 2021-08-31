#include "SymplecticEuler.h"

bool SymplecticEuler::stepScene( TwoDScene& scene, scalar dt )
{
  // Your code goes here!
    VectorXs& qn = scene.getX();
    VectorXs& qdot = scene.getV();
    VectorXs M = scene.getM();    
    
    int numberOfDimensions = 2;
    int numberOfParticles = qn.rows() / numberOfDimensions;  
    
    VectorXs deltaU(numberOfDimensions * numberOfParticles);
    deltaU.setZero();
    
    scene.accumulateGradU(deltaU);
    
    for(int i = 0; i < numberOfParticles; i++) {
        if(!scene.isFixed(i)) {
            int ix = i * numberOfDimensions + 0;
            int iy = i * numberOfDimensions + 1;       
            
            scalar MX = M[ix];        
            scalar MY = M[iy];            

            qdot[ix] += (-deltaU[ix] / MX) * dt;     
            qdot[iy] += (-deltaU[iy] / MY) * dt;   
            
            qn[ix] += qdot[ix] * dt;
            qn[iy] += qdot[iy] * dt;            
        }
    }            
  
  return true;
}





