#include "PenaltyForce.h"
#include "TwoDScene.h"

void PenaltyForce::addEnergyToTotal( const VectorXs& x, const VectorXs& v, const VectorXs& m, scalar& E )
{
    // Feel free to implement if you feel like doing so.
}

// Adds the gradient of the penalty potential (-1 * force) for a pair of 
// particles to the total.
// Read the positions of the particles from the input variable x. Radii can
// be obtained from the member variable m_scene, the penalty force stiffness 
// from member variable m_k, and penalty force thickness from member variable
// m_thickness.
// Inputs:
//   x:    The positions of the particles in the scene. 
//   idx1: The index of the first particle, i.e. the position of this particle
//         is ( x[2*idx1], x[2*idx1+1] ).
//   idx2: The index of the second particle.
// Outputs:
//   gradE: The total gradient of penalty force. *ADD* the particle-particle
//          gradient to this total gradient.
void PenaltyForce::addParticleParticleGradEToTotal(const VectorXs &x, int idx1, int idx2, VectorXs &gradE)
{
    VectorXs x1 = x.segment<2>(2*idx1);
    VectorXs x2 = x.segment<2>(2*idx2);
    
    double r1 = m_scene.getRadius(idx1);
    double r2 = m_scene.getRadius(idx2);
    
    // Your code goes here!
    VectorXs n = x2 - x1;
    VectorXs n_hat = n;
    n_hat.normalize();
    
    if(n.norm() < r1 + r2 + m_thickness)
    {
        gradE.segment<2>(2*idx1) -= m_k * (n.norm() - r1 - r2 - m_thickness) * n_hat;
        gradE.segment<2>(2*idx2) += m_k * (n.norm() - r1 - r2 - m_thickness) * n_hat;
    }
}

// Adds the gradient of the penalty potential (-1 * force) for a particle-edge
// pair to the total.
// Read the positions of the particle and edge endpoints from the input
// variable x.
// Inputs:
//   x:    The positions of the particles in the scene.
//   vidx: The index of the particle.
//   eidx: The index of the edge, i.e. the indices of the particle making up the
//         endpoints of the edge are given by m_scene.getEdge(eidx).first and 
//         m_scene.getEdges(eidx).second.
// Outputs:
//   gradE: The total gradient of penalty force. *ADD* the particle-edge
//          gradient to this total gradient.
void PenaltyForce::addParticleEdgeGradEToTotal(const VectorXs &x, int vidx, int eidx, VectorXs &gradE)
{
    VectorXs x1 = x.segment<2>(2*vidx);
    VectorXs x2 = x.segment<2>(2*m_scene.getEdge(eidx).first);
    VectorXs x3 = x.segment<2>(2*m_scene.getEdge(eidx).second);
    
    double r1 = m_scene.getRadius(vidx);
    double r2 = m_scene.getEdgeRadii()[eidx];
    
    // Your code goes here!
    // Calculation the alpha for n
    double alpha = (x2-x1).dot(x3-x2)/(x3-x2).dot(x3-x2);
    // We have to clamp alpha to the range [0,1]
    if(alpha < 0)
        alpha = 0;
    if(alpha > 1)
        alpha = 1;
    
    //Calculation of n
    VectorXs n = x2 + alpha*(x3-x2) - x1;
    VectorXs n_hat = n;
    n_hat.normalize();
    
    if (n.norm() < r1 + r2 + m_thickness)
    {
        gradE.segment<2>(2*vidx) -= m_k*(n.norm() - r1 - r2 - m_thickness)*n_hat;
        gradE.segment<2>(2*m_scene.getEdge(eidx).first) += m_k*(1-alpha)*(n.norm() - r1 - r2 - m_thickness)*n_hat;
        gradE.segment<2>(2*m_scene.getEdge(eidx).second) += m_k*alpha*(n.norm() - r1 - r2 - m_thickness)*n_hat;
    }
}

// Adds the gradient of the penalty potential (-1 * force) for a particle-
// half-plane pair to the total.
// Read the positions of the particle from the input variable x.
// Inputs:
//   x:    The positions of the particles in the scene.
//   vidx: The index of the particle.
//   pidx: The index of the half-plane, i.e. the position and normal vectors
//         for the half-plane can be retrieved by calling
//         m_scene.getHalfplane(pidx).
// Outputs:
//   gradE: The total gradient of the penalty force. *ADD* the particle-
//          half-plane gradient to this total gradient.
void PenaltyForce::addParticleHalfplaneGradEToTotal(const VectorXs &x, int vidx, int pidx, VectorXs &gradE)
{
    VectorXs x1 = x.segment<2>(2*vidx);
    VectorXs nh = m_scene.getHalfplane(pidx).second;
    
    // Your code goes here!
    // Calculation of n
    VectorXs n = (m_scene.getHalfplane(pidx).first - x1).dot(nh)/(nh.dot(nh))*nh;
    VectorXs n_hat = n;
    n_hat.normalize();
    
    double r = m_scene.getRadius(vidx);
    
    if(n.norm() < r + m_thickness)
    {
        gradE.segment<2>(2*vidx) -= m_k*(n.norm() - r - m_thickness)*n_hat.dot(nh)/(nh.dot(nh))*nh;
    }
    
}
