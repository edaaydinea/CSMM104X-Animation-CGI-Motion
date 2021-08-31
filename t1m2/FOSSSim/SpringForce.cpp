#include "SpringForce.h"

void SpringForce::addEnergyToTotal( const VectorXs& x, const VectorXs& v, const VectorXs& m, scalar& E )
{
  assert( x.size() == v.size() );
  assert( x.size()%2 == 0 );
  assert( m_endpoints.first >= 0 );  assert( m_endpoints.first < x.size()/2 );
  assert( m_endpoints.second >= 0 ); assert( m_endpoints.second < x.size()/2 );

  // Your code goes here!
}

void SpringForce::addGradEToTotal( const VectorXs& x, const VectorXs& v, const VectorXs& m, VectorXs& gradE )
{
  assert( x.size() == v.size() );
  assert( x.size() == gradE.size() );
  assert( x.size()%2 == 0 );
  assert( m_endpoints.first >= 0 );  assert( m_endpoints.first < x.size()/2 );
  assert( m_endpoints.second >= 0 ); assert( m_endpoints.second < x.size()/2 );

  // Your code goes here!
  int iX1 = m_endpoints.first * 2;
  int iY1 = iX1 + 1;
    
  int iX2 = m_endpoints.second * 2;
  int iY2 = iX2 + 1;
    
  scalar x1 = x[iX1];
  scalar y1 = x[iY1];
                  
  scalar x2 = x[iX2];
  scalar y2 = x[iY2];  
    
  scalar qdot_X1 = v[iX1];
  scalar qdot_Y1 = v[iY1];
    
  scalar qdot_X2 = v[iX2];
  scalar qdot_Y2 = v[iY2];
    
  scalar qdot_X = x2-x1;
  scalar qdot_Y = y2-y1;
    
  scalar qdot_length = sqrt((qdot_X * qdot_X)+(qdot_Y * qdot_Y));   
    
  scalar energy = 0.5 * m_k * ((qdot_length - m_l0) * (qdot_length - m_l0));         
    
  scalar nx = qdot_X / qdot_length;
  scalar ny = qdot_Y / qdot_length;  
    
  scalar gradientX = m_k * (qdot_length - m_l0) * nx;
  scalar gradientY = m_k * (qdot_length - m_l0) * ny;   
  
  scalar velocityDeltaX = (qdot_X1 - qdot_X2);
  scalar velocityDeltaY = (qdot_Y1 - qdot_Y2);
    
  scalar dotted = (m_b * nx * velocityDeltaX) + (m_b * ny * velocityDeltaY);
            
  scalar dampingX = nx * dotted;  
  scalar dampingY = ny * dotted;
  
  gradE[iX1] -= (gradientX - dampingX);
  gradE[iY1] -= (gradientY - dampingY);
    
  gradE[iX2] += (gradientX - dampingX);
  gradE[iY2] += (gradientY - dampingY);   
}