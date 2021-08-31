#include "SpringForce.h"

void SpringForce::addHessXToTotal( const VectorXs& x, const VectorXs& v, const VectorXs& m, MatrixXs& hessE )
{
    assert( x.size() == v.size() );
    assert( x.size() == m.size() );
    assert( x.size() == hessE.rows() );
    assert( x.size() == hessE.cols() );
    assert( x.size()%2 == 0 );
    assert( m_endpoints.first >= 0 );  assert( m_endpoints.first < x.size()/2 );
    assert( m_endpoints.second >= 0 ); assert( m_endpoints.second < x.size()/2 );
    
    // Implement force Jacobian here!
    // // Contribution from elastic component

    int index_i = 2 * m_endpoints.first;
    int index_j = 2 * m_endpoints.second;
  
    int dof = v.size();

    scalar mi = m(index_i);
    scalar mj = m(index_j);

    Vector2s xi = x.segment(index_i, 2);
    Vector2s xj = x.segment(index_j, 2);

    Vector2s vi = v.segment(index_i, 2);
    Vector2s vj = v.segment(index_j, 2);

    MatrixXs J = MatrixXs::Zero(4,4);
    Matrix2s K = MatrixXs::Zero(2,2);

    Vector2s n_hat = xj - xi;
    scalar length = n_hat.norm();
    n_hat = n_hat / length;

    K = (-m_k) * (n_hat * n_hat.transpose() + ((length- m_l0)/length * (Matrix2s::Identity() - n_hat * n_hat.transpose())));

    J.block<2,2>(0,0) = J.block<2,2>(0,0) + K;
    J.block<2,2>(2,0) = J.block<2,2>(2,0) + (-K);
    J.block<2,2>(0,2) = J.block<2,2>(0,2) + (-K);
    J.block<2,2>(2,2) = J.block<2,2>(2,2) + K;

    Vector2s deltaV = vi - vj;
    Vector2s deltaV_transpose = deltaV.transpose();

    K  = -(m_b/length) *(n_hat.dot(deltaV) * Matrix2s::Identity() + n_hat * deltaV.transpose()) *(Matrix2s::Identity() - n_hat*n_hat.transpose());

    J.block<2,2>(0,0) = J.block<2,2>(0,0) + K;
    J.block<2,2>(2,0) = J.block<2,2>(2,0) + (-K);
    J.block<2,2>(0,2) = J.block<2,2>(0,2) + (-K);
    J.block<2,2>(2,2) = J.block<2,2>(2,2) + K;

    hessE.block<2,2>(index_i,index_i) = hessE.block<2,2>(index_i,index_i) + (-J.block<2,2>(0,0));
    hessE.block<2,2>(index_i,index_j) = hessE.block<2,2>(index_i,index_j) + (-J.block<2,2>(2,0));
    hessE.block<2,2>(index_j,index_i) = hessE.block<2,2>(index_j,index_i) + (-J.block<2,2>(0,2));
    hessE.block<2,2>(index_j,index_j) = hessE.block<2,2>(index_j,index_j) + (-J.block<2,2>(2,2));
}

void SpringForce::addHessVToTotal( const VectorXs& x, const VectorXs& v, const VectorXs& m, MatrixXs& hessE )
{
    assert( x.size() == v.size() );
    assert( x.size() == m.size() );
    assert( x.size() == hessE.rows() );
    assert( x.size() == hessE.cols() );
    assert( x.size()%2 == 0 );
    assert( m_endpoints.first >= 0 );  assert( m_endpoints.first < x.size()/2 );
    assert( m_endpoints.second >= 0 ); assert( m_endpoints.second < x.size()/2 );

    // Implement force Jacobian here!
    int index_i = m_endpoints.first * 2;
    int index_j = m_endpoints.second * 2;

    Vector2s xi = x.segment(index_i, 2);
    Vector2s xj = x.segment(index_j, 2);
      
    MatrixXs J = MatrixXs::Zero(4,4); 
    Matrix2s B = MatrixXs::Zero(2,2);
      
    Vector2s n_hat = xj - xi; 
    scalar length = n_hat.norm(); 
    n_hat = n_hat / length;
    // Contribution from damping

    B = m_b * n_hat * n_hat.transpose();

    J.block<2,2>(0,0) = -B;
    J.block<2,2>(2,0) = B;
    J.block<2,2>(0,2) = B;
    J.block<2,2>(2,2) = -B;

    hessE.block<2,2>(index_i,index_i) = hessE.block<2,2>(index_i,index_i) + (-J.block<2,2>(0,0));
    hessE.block<2,2>(index_i,index_j) = hessE.block<2,2>(index_i,index_j) + (-J.block<2,2>(2,0));
    hessE.block<2,2>(index_j,index_i) = hessE.block<2,2>(index_j,index_i) + (-J.block<2,2>(0,2));
    hessE.block<2,2>(index_j,index_j) = hessE.block<2,2>(index_j,index_j) + (-J.block<2,2>(2,2));

}
