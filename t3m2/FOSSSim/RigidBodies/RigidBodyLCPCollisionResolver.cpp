#include "RigidBodyLCPCollisionResolver.h"

void RigidBodyLCPCollisionResolver::resolveCollisions( std::vector<RigidBody>& rbs, const std::set<RigidBodyCollision>& rbcs )
{
  // Your code goes here!

  // Example of using the lcp solver
  MatrixXs A(4,4);
  A << 0.828869, 0.337798, -0.28125, -0.21875,
       0.337798, 0.828869, -0.21875, -0.28125,
       -0.28125, -0.21875, 0.828869, 0.337798,
       -0.21875, -0.28125, 0.337798, 0.828869;

  VectorXs b(4);
  b << -1, -1, -1, -1;

  VectorXs lambda(4);
  lcputils::solveLCPwithODE( A, b, lambda );
}
