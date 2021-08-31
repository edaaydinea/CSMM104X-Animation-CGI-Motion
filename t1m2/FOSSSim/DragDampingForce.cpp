#include "DragDampingForce.h"

void DragDampingForce::addGradEToTotal( const VectorXs& x, const VectorXs& v, const VectorXs& m, VectorXs& gradE )
{
  assert( x.size() == v.size() );
  assert( x.size() == m.size() );
  assert( x.size() == gradE.size() );
  assert( x.size()%2 == 0 );
  
  // Your code goes here!
  int numPositions = x.size()/2;   
    
  for(int i = 0; i < numPositions; i++) {
      int iX = i * 2;
      int iY = iX + 1;
      
      scalar forceX = -(v[iX] * m_b);
      scalar forceY = -(v[iY] * m_b);
      
      gradE[iX] -= forceX;
      gradE[iY] -= forceY;
  }
}
