#include "GravitationalForce.h"

void GravitationalForce::addHessXToTotal( const VectorXs& x, const VectorXs& v, const VectorXs& m, MatrixXs& hessE )
{
    assert( x.size() == v.size() );
    assert( x.size() == m.size() );
    assert( x.size() == hessE.rows() );
    assert( x.size() == hessE.cols() );
    assert( x.size()%2 == 0 );

    // Compute the force Jacobian here!
    int index_i = 2 * m_particles.first;
    int index_j = 2 * m_particles.second;
    int dof = x.size();

    scalar mi = m(index_i);
    scalar mj = m(index_j);

    Vector2s xi = x.segment(index_i, 2);
    Vector2s xj = x.segment(index_j, 2);

    Vector2s n_hat = xj - xi;
    scalar length = n_hat.norm();
    n_hat = n_hat / length;

    Matrix2s K = Matrix2s::Identity() - (3.0 * n_hat * n_hat.transpose());
    K = K * (-m_G * mi * mj) / (length * length * length);

    MatrixXs Jacobian = MatrixXs::Zero(4,4);
    Jacobian.block<2,2>(0,0) = K;
    Jacobian.block<2,2>(2,0) = -K;
    Jacobian.block<2,2>(0,2) = -K;
    Jacobian.block<2,2>(2,2) = K;

    hessE.block<2,2>(index_i,index_i) = hessE.block<2,2>(index_i,index_i) + (-Jacobian.block<2,2>(0,0));
    hessE.block<2,2>(index_i,index_j) = hessE.block<2,2>(index_i,index_j) + (-Jacobian.block<2,2>(2,0));
    hessE.block<2,2>(index_j,index_i) = hessE.block<2,2>(index_j,index_i) + (-Jacobian.block<2,2>(0,2));
    hessE.block<2,2>(index_j,index_j) = hessE.block<2,2>(index_j,index_j) + (-Jacobian.block<2,2>(2,2));
}

void GravitationalForce::addHessVToTotal( const VectorXs& x, const VectorXs& v, const VectorXs& m, MatrixXs& hessE )
{
    assert( x.size() == v.size() );
    assert( x.size() == m.size() );
    assert( x.size() == hessE.rows() );
    assert( x.size() == hessE.cols() );
    assert( x.size()%2 == 0 );
    // Nothing to do.
}
