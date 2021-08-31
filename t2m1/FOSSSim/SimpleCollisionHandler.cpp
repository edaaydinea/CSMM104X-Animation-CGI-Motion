#include "SimpleCollisionHandler.h"
#include <iostream>
#include <set>

// BEGIN STUDENT CODE //


// Detects whether two particles are overlapping (including the radii of each)
// and approaching.
// If the two particles overlap and are approaching, returns true and sets 
// the vector n to be the vector between the first and second particle.
// Inputs:
//   scene: The scene data structure. The positions and radii of the particles
//          can be obtained from here.
//   idx1:  The index of the first particle. (Ie, the degrees of freedom
//          corresponding to this particle are entries 2*idx1 and 2*idx1+1 in
//          scene.getX().
//   idx2:  The index of the second particle.
// Outputs:
//   n: The vector between the two particles.
//   Returns true if the two particles overlap and are approaching.
bool SimpleCollisionHandler::detectParticleParticle(TwoDScene &scene, int idx1, int idx2, Vector2s &n)
{
    VectorXs x1 = scene.getX().segment<2>(2*idx1);
    VectorXs x2 = scene.getX().segment<2>(2*idx2);
    
    // Your code goes here!
    n = x2 - x1;
    // overlapping means that radius1 + radius2 > ||x_2 - x_1||
    if (n.norm() < scene.getRadius(idx1) + scene.getRadius(idx2))
    {
        double relative_velocity = (scene.getV().segment<2>(2*idx1) - scene.getV().segment<2>(2*idx2)).dot(n);
        // Checking if the relative velocity along n is positive
        if (relative_velocity > 0 )
            return true;
    }
    return false;
}

// Detects whether a particle and an edge are overlapping (including the radii 
// of both) and are approaching.
// If the two objects overlap and are approaching, returns true and sets the 
// vector n to be the shortest vector between the particle and the edge.
// Inputs:
//   scene: The scene data structure.
//   vidx:  The index of the particle.
//   eidx:  The index of the edge. (Ie, the indices of particle with index e are
//          scene.getEdges()[e].first and scene.getEdges()[e].second.)
// Outputs:
//   n: The shortest vector between the particle and the edge.
//   Returns true if the two objects overlap and are approaching.
bool SimpleCollisionHandler::detectParticleEdge(TwoDScene &scene, int vidx, int eidx, Vector2s &n)
{
    VectorXs x1 = scene.getX().segment<2>(2*vidx);
    VectorXs x2 = scene.getX().segment<2>(2*scene.getEdges()[eidx].first);
    VectorXs x3 = scene.getX().segment<2>(2*scene.getEdges()[eidx].second);
    
    // Your code goes here!
   
    //Calculate alpha using the formula
    double alpha = (x1-x2).dot(x3-x2) / (x3-x2).dot(x3-x2);
    // We have to clamp alpha to the range [0,1]
    alpha = std::min(1.0, std::max(0.0, alpha));
    // Calculate the point x(alpha)
    VectorXs x_alpha = x2 + alpha*(x3-x2);
    // The vector we need is then n = x(alpha) - x1
    n = x_alpha - x1;
    
    // Checking if this vector n is less than the combined radius of your particle and edge to confirm it is overlapping.
    if (n.norm() < scene.getRadius(vidx) + scene.getEdgeRadii()[eidx])
    {
        // looking at the velocities in the n direction of the particle and the closest point on the edge
        VectorXs v1 = scene.getV().segment<2>(2*vidx);
        VectorXs v2 = scene.getV().segment<2>(2*scene.getEdges()[eidx].first);
        VectorXs v3 = scene.getV().segment<2>(2*scene.getEdges()[eidx].second);
        
        // Caclulation of the velocity of the closest point x(alpha)
        double relative_velocity = (v1 - v2 - alpha*(v3-v2)).dot(n);
        // The objects are approaching if this scalar is positive.
        if (relative_velocity > 0)
            return true;
    }
    
    return false;
}

// Detects whether a particle and a half-plane are overlapping (including the 
// radius of the particle) and are approaching.
// If the two objects overlap and are approaching, returns true and sets the 
// vector n to be the shortest vector between the particle and the half-plane.
// Inputs:
//   scene: The scene data structure.
//   vidx:  The index of the particle.
//   pidx:  The index of the halfplane. The vectors (px, py) and (nx, ny) can
//          be retrieved by calling scene.getHalfplane(pidx).
// Outputs:
//   n: The shortest vector between the particle and the half-plane.
//   Returns true if the two objects overlap and are approaching.
bool SimpleCollisionHandler::detectParticleHalfplane(TwoDScene &scene, int vidx, int pidx, Vector2s &n)
{
    VectorXs x1 = scene.getX().segment<2>(2*vidx);
    VectorXs px = scene.getHalfplane(pidx).first;
    VectorXs pn = scene.getHalfplane(pidx).second;
    
    // Your code goes here!
    pn.normalize();
    
    // Calculation of the distance
    n = (px - x1).dot(pn)*pn;
    
   
    if (n.norm() < scene.getRadius(vidx))
    {
        // Calculation of velocity
        double relative_velocity = scene.getV().segment<2>(2*vidx).dot(n);
        // If the quantity is positive, the particle is approaching the half-plane.
        if (relative_velocity > 0)
            return true;
    }
    return false;
}


// Responds to a collision detected between two particles by applying an impulse
// to the velocities of each one.
// You can get the COR of the simulation by calling getCOR().
// Inputs:
//   scene: The scene data structure.
//   idx1:  The index of the first particle.
//   idx2:  The index of the second particle.
//   n:     The vector between the first and second particle.
// Outputs:
//   None.
void SimpleCollisionHandler::respondParticleParticle(TwoDScene &scene, int idx1, int idx2, const Vector2s &n)
{
    const VectorXs &M = scene.getM();
    VectorXs &v = scene.getV();
    
    // Your code goes here!
    // Caclulation of the collision - response between two particles
    
    // Denote by n_hat the unit vector in direction n.
    VectorXs n_hat = n;
    n_hat.normalize();
    
    double cfactor = (1.0 + getCOR()) / 2.0;
    double m1 = scene.isFixed(idx1) ? std::numeric_limits<double>::infinity() : M[2*idx1];
    double m2 = scene.isFixed(idx2) ? std::numeric_limits<double>::infinity() : M[2*idx2];

    double numerator = 2  * cfactor *  (v.segment<2>(2*idx2) - v.segment<2>(2*idx1)).dot(n_hat);
    double denominator1 = 1 + m1/m2;
    double denominator2 = m2/m1 + 1;
    
    // Updating velocities when a collision is detected are:
    
    if(!scene.isFixed(idx1))
      v.segment<2>(2*idx1) += numerator/denominator1 * n_hat;
    if(!scene.isFixed(idx2))
      v.segment<2>(2*idx2) -= numerator/denominator2 * n_hat;
    
}

// Responds to a collision detected between a particle and an edge by applying
// an impulse to the velocities of each one.
// Inputs:
//   scene: The scene data structure.
//   vidx:  The index of the particle.
//   eidx:  The index of the edge.
//   n:     The shortest vector between the particle and the edge.
// Outputs:
//   None.
void SimpleCollisionHandler::respondParticleEdge(TwoDScene &scene, int vidx, int eidx, const Vector2s &n)
{
    const VectorXs &M = scene.getM();
    
    int eidx1 = scene.getEdges()[eidx].first;
    int eidx2 = scene.getEdges()[eidx].second;
    
    VectorXs x1 = scene.getX().segment<2>(2*vidx);
    VectorXs x2 = scene.getX().segment<2>(2*eidx1);
    VectorXs x3 = scene.getX().segment<2>(2*eidx2);
    
    VectorXs v1 = scene.getV().segment<2>(2*vidx);
    VectorXs v2 = scene.getV().segment<2>(2*eidx1);
    VectorXs v3 = scene.getV().segment<2>(2*eidx2);
    
    // Your code goes here!
    VectorXs n_hat = n;
    n_hat.normalize();
    
    //Calculate alpha using the formula
    double alpha = (x1-x2).dot(x3-x2)/(x3-x2).dot(x3-x2);
    // We have to clamp alpha to the range [0,1]
    alpha = std::min(1.0, std::max(0.0, alpha));
    
    // Calculation of the velocity_edge
    VectorXs velocity_edge = v2 + alpha*(v3-v2);
    
    double cfactor = (1.0 + getCOR()) / 2.0;
    
    double m1 = scene.isFixed(vidx) ? std::numeric_limits<double>::infinity() : M[2*vidx];
    double m2 = scene.isFixed(eidx1) ? std::numeric_limits<double>::infinity() : M[2*eidx1];
    double m3 = scene.isFixed(eidx2) ? std::numeric_limits<double>::infinity() : M[2*eidx2];
    
    double numerator = 2  * cfactor * (velocity_edge - v1).dot(n_hat);
    double denominator1 = 1.0 + (1-alpha)*(1-alpha)*m1/m2 + alpha*alpha*m1/m3;
    double denominator2 = m2/m1 + (1-alpha)*(1-alpha) + alpha*alpha*m2/m3;
    double denominator3 = m3/m1 + (1-alpha)*(1-alpha)*m3/m2 + alpha*alpha;
    
    if (!scene.isFixed(vidx))
        scene.getV().segment<2>(2*vidx) += numerator / denominator1 * n_hat;
    if(!scene.isFixed(eidx1))
        scene.getV().segment<2>(2*eidx1) -= (1.0-alpha)*numerator/denominator2 * n_hat;
    if(!scene.isFixed(eidx2))
        scene.getV().segment<2>(2*eidx2) -= alpha * numerator/denominator3 * n_hat;
    
}


// Responds to a collision detected between a particle and a half-plane by 
// applying an impulse to the velocity of the particle.
// Inputs:
//   scene: The scene data structure.
//   vidx:  The index of the particle.
//   pidx:  The index of the half-plane.
//   n:     The shortest vector between the particle and the half-plane.
// Outputs:
//   None.
void SimpleCollisionHandler::respondParticleHalfplane(TwoDScene &scene, int vidx, int pidx, const Vector2s &n)
{
    VectorXs n_hat = n;
    // Your code goes here!
    n_hat.normalize();
    
    double cfactor = (1.0 + getCOR()) / 2.0;
    
    scene.getV().segment<2>(2*vidx) -=2*cfactor*scene.getV().segment<2>(2*vidx).dot(n_hat)*n_hat;
    
}
