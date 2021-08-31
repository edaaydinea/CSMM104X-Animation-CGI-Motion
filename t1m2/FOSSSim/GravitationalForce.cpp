#include "GravitationalForce.h"

void GravitationalForce::addEnergyToTotal( const VectorXs& x, const VectorXs& v, const VectorXs& m, scalar& E )
{
  assert( x.size() == v.size() );
  assert( x.size() == m.size() );
  assert( x.size()%2 == 0 );
  assert( m_particles.first >= 0 );  assert( m_particles.first < x.size()/2 );
  assert( m_particles.second >= 0 ); assert( m_particles.second < x.size()/2 );

  // Your code goes here!
}

void GravitationalForce::addGradEToTotal( const VectorXs& x, const VectorXs& v, const VectorXs& m, VectorXs& gradE )
{
  assert( x.size() == v.size() );
  assert( x.size() == m.size() );
  assert( x.size() == gradE.size() );
  assert( x.size()%2 == 0 );
  assert( m_particles.first >= 0 );  assert( m_particles.first < x.size()/2 );
  assert( m_particles.second >= 0 ); assert( m_particles.second < x.size()/2 );

  // Your code goes here!
  // Getting the particles for calculation
  
  int iX1 = m_particles.first * 2;
  int iY1 = iX1 + 1;
    
  int iX2 = m_particles.second * 2;
  int iY2 = iX2 + 1;
    
  scalar x1 = x[iX1];
  scalar y1 = x[iY1];
                  
  scalar x2 = x[iX2];
  scalar y2 = x[iY2];  
    
  scalar m1 = m[m_particles.first * 2];
  scalar m2 = m[m_particles.second * 2];

  // qdot = velocity
  scalar qdot_X = x2-x1;
  scalar qdot_Y = y2-y1;
    
  scalar qdot_length = sqrt((qdot_X * qdot_X)+(qdot_Y * qdot_Y));
       
  scalar nx = qdot_X / qdot_length;
  scalar ny = qdot_Y / qdot_length;      
    
  scalar gradientX = ((m_G * m1 * m2) / (qdot_length * qdot_length)) * nx;
  scalar gradientY = ((m_G * m1 * m2) / (qdot_length * qdot_length)) * ny;       
    
  gradE[iX1] -= gradientX;
  gradE[iY1] -= gradientY;
    
  gradE[iX2] += gradientX;
  gradE[iY2] += gradientY; 
  
 
}
