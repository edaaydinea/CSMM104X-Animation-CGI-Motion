#include "LinearizedImplicitEuler.h"

bool LinearizedImplicitEuler::stepScene( TwoDScene& scene, scalar dt )
{
    VectorXs& x = scene.getX();
    VectorXs& v = scene.getV();
    const VectorXs& m = scene.getM();
    assert(x.size() == v.size());
    assert(x.size() == m.size());

    // Your code goes here!
    
  int dof = x.size();
  int number_of_particles = scene.getNumParticles();
    
  VectorXs dx = dt * v;
  VectorXs dv = VectorXs::Zero(dof);
  
  VectorXs b = VectorXs::Zero(dof);
  scene.accumulateGradU(b, dx, dv);
  b = b * -dt;
    

  for( int i = 0; i < number_of_particles; ++i ) 
      if(scene.isFixed(i))
          b.segment<2>(2*i).setZero();


  MatrixXs df_dq = MatrixXs::Zero(dof,dof);
  scene.accumulateddUdxdx(df_dq, dx, dv);
    
  MatrixXs df_dqdot = MatrixXs::Zero(dof,dof);
  scene.accumulateddUdxdv(df_dqdot, dx, dv);
  
  MatrixXs M = m.asDiagonal();
  MatrixXs A = M - ((dt * dt * -df_dq) + (dt * -df_dqdot));
    
  for( int i = 0; i < number_of_particles; ++i )
  {
      if( scene.isFixed(i) )
      {
          A.row(2*i).setZero();
          A.row(2*i+1).setZero();
          A.col(2*i).setZero();
          A.col(2*i+1).setZero();

          A(2*i,2*i) = 1.0;
          A(2*i+1,2*i+1) = 1.0;
      } 
   }
    

   VectorXs dqdot = A.fullPivLu().solve(b); 
   v = v + dqdot;
   x = x + dt * v;
    
   return true;
}
