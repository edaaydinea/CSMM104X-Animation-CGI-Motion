#include "ElasticBodySpringForce.h"
#include <assert.h>

void ElasticBodySpringForce::addEnergyToTotal( const VectorXs& x, const VectorXs& v, const VectorXs& m, scalar& E )
{
    assert( x.size() == v.size() );
    assert( x.size()%2 == 0 );
    assert( m_idx1 >= 0 );  assert( m_idx1 < x.size()/2 );
    assert( m_idx2 >= 0 );  assert( m_idx2 < x.size()/2 );

}

void ElasticBodySpringForce::addGradEToTotal( const VectorXs& x, const VectorXs& v, const VectorXs& m, VectorXs& gradE )
{
    assert( x.size() == v.size() );
    assert( x.size() == gradE.size() );
    assert( x.size()%2 == 0 );
    assert( m_idx1 >= 0 );  assert( m_idx1 < x.size()/2 );
    assert( m_idx2 >= 0 );  assert( m_idx2 < x.size()/2 );

    // Your code goes here!
}

void ElasticBodySpringForce::addHessXToTotal( const VectorXs& x, const VectorXs& v, const VectorXs& m, MatrixXs& hessE )
{
    assert( x.size() == v.size() );
    assert( x.size() == m.size() );
    assert( x.size() == hessE.rows() );
    assert( x.size() == hessE.cols() );
    assert( x.size()%2 == 0 );
    assert( m_idx1 >= 0 );  assert( m_idx1 < x.size()/2 );
    assert( m_idx2 >= 0 );  assert( m_idx2 < x.size()/2 );
    
    // Your code goes here!
}

void ElasticBodySpringForce::addHessVToTotal( const VectorXs& x, const VectorXs& v, const VectorXs& m, MatrixXs& hessE )
{
    assert( x.size() == v.size() );
    assert( x.size() == m.size() );
    assert( x.size() == hessE.rows() );
    assert( x.size() == hessE.cols() );
    assert( x.size()%2 == 0 );
    assert( m_idx1 >= 0 );  assert( m_idx1 < x.size()/2 );
    assert( m_idx2 >= 0 );  assert( m_idx2 < x.size()/2 );
    
    // You do not need to implement this.
}
