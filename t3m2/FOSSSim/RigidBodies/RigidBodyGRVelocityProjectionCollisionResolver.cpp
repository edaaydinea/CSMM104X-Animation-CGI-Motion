#include "RigidBodyGRVelocityProjectionCollisionResolver.h"


void RigidBodyGRVelocityProjectionCollisionResolver::resolveCollisions( std::vector<RigidBody>& rbs, const std::set<RigidBodyCollision>& rbcs )
{
  // Your code goes here!

  // Example of using QuadProg++
  // For detailed documentation, please see FOSSSim/quadprog/QuadProg++.hh
 
  // Matrix in quadratic form of objective
  QuadProgPP::Matrix<scalar> G;
  // Equality constraints
  QuadProgPP::Matrix<scalar> CE;
  // Inequality constraints
  QuadProgPP::Matrix<scalar> CI;
  
  QuadProgPP::Vector<scalar> g0;
  QuadProgPP::Vector<scalar> ce0;
  QuadProgPP::Vector<scalar> ci0;
  QuadProgPP::Vector<scalar> x;

  // M = 16  0 0
  //      0 16 0
  //      0  0 289.75
  G.resize(3,3);
  for(int i=0; i< 3; i++) for(int j=0; j< 3; j++) G[i][j]=0;
  G[0][0] = 16; 
  G[1][1] = 16;
  G[2][2] = 289.75;

  // -M \dot q = -0
  //             139.52
  //            -0
  g0.resize(3);
  g0[0] = 0;
  g0[1] = 139.52;
  g0[2] = 0;

  // No equality constraints, currently
  CE.resize(3,0);
  ce0.resize(0);

  // Compute the number of inequality constraints
  CI.resize(3,24);

  MatrixXs tempN(3,24);
  tempN << 0,  0,  0,  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
           1,  1,  1,  1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
          -5, -4, -3, -2, 2, 3, 4, 5, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;

  for( int i = 0; i < 3; ++i ) for( int j = 0; j < 24; ++j ) CI[i][j] = tempN(i,j);

  // Constant term added to inequality constraints
  ci0.resize(24);
  for(int i=0; i< 24; i++) ci0[i] = 0;

  // Solution
  x.resize(3);

  solve_quadprog(G, g0, CE, ce0, CI, ci0, x);
}


