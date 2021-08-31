#include "SimpleGravityForce.h"

void SimpleGravityForce::addEnergyToTotal( const VectorXs& x, const VectorXs& v, const VectorXs& m, scalar& E )
{
    assert( x.size() == v.size() );
    assert( x.size() == m.size() );
    assert( x.size()%2 == 0 );
    
    // Your code goes here!
  for (int i = 0; i < x.size()/2; ++i)
  {
    E = E - VectorXs(m.segment<2>(2 * i).array()*m_gravity.array()).dot(x.segment<2>(2 * i));
  }
}

void SimpleGravityForce::addGradEToTotal( const VectorXs& x, const VectorXs& v, const VectorXs& m, VectorXs& gradE )
{
    assert( x.size() == v.size() );
    assert( x.size() == m.size() );
    assert( x.size() == gradE.size() );
    assert( x.size()%2 == 0 );
    
    // Your code goes here!
  
  for(int i = 0; i < x.size()/2; ++i ) 
  {
    gradE.segment<2>(2 * i) = gradE.segment<2>(2 * i) -  VectorXs(m.segment<2>(2 * i).array() * m_gravity.array());
  }
}

