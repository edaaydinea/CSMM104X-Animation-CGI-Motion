#ifndef __RIGID_BODY_H__
#define __RIGID_BODY_H__

#include <Eigen/Core>
#include "FOSSSim/MathDefs.h"

#include <iostream>
#include <fstream>

class RigidBody
{
public:

  // Inputs:
  //   v:         Velocity of the center of mass.
  //   omega:     Angular velocity of the rigid body.
  //   vertices:  Flat vector containing the vertices of the rigid body. The hull is defined
  //              in clockwise order by the vertices. Indices 2*i and 2*i+1 define the ith vertex.
  //              vertices.size() == 2*N
  //   masses:    Flat vector containing all masses associated with vertices. masses.size() == N
  //   radius:    Inflated radius of the rigid body for rendering and collision detection. These
  //              rigid bodies are minkowski sums of the poly-line hulls and circles. 
  RigidBody( const Vector2s& v, const scalar& omega, const VectorXs& vertices, const VectorXs& masses, const scalar& radius );

  // Update quantities computed from 'core' state (e.g. rotation matrix derived from theta)
  void updateDerivedQuantities();

  // Returns the center of mass
  Vector2s& getX();
  const Vector2s& getX() const;

  // Returns the orientation (an angle in radians)
  scalar& getTheta();
  const scalar& getTheta() const;
  
  // Returns the velocity of the center of mass
  Vector2s& getV();
  const Vector2s& getV() const;
  
  // Returns the angular velocity
  scalar& getOmega();
  const scalar& getOmega() const;

  // Returns the total mass of the rigid body
  const scalar& getM() const;

  // Returns the moment of inertia of the rigid body
  const scalar& getI() const;

  // Returns the radius of the rigid body
  const scalar& getRadius() const;

  // Returns the number of vertices in the rigid body
  int getNumVertices() const;

  // Returns the number of edges that compose the boundary of this rigid body
  int getNumEdges() const;



  // Returns the position of vertex i in 'world space', taking into account the body's orientation and position
  // Inputs:
  //   i:       Index of a vertex. Valid range is 0...N-1 where N is the number of vertices. 
  // Outputs:
  //   Position of a vertex in 'world space'
  Vector2s getWorldSpaceVertex( int i ) const;
  
  // Rotates a 'body space' vector into 'world space', taking into account the body's orientation
  // Inputs:
  //   bodyvec: A vector in 'body space'   
  // Outputs:
  //   The input 'body space' vector rotated by the body's orientation, but not translated. 
  Vector2s rotateIntoWorldSpace( const Vector2s& bodyvec ) const;
  
  // Returns the position of 'body space' vector bodyvec in 'world space', taking into account the body's orientation and position
  // Inputs:
  //   bodyvec: A vector in 'body space'
  // Outputs:
  //   The input 'body space' vector transformed into 'world space' 
  Vector2s computeWorldSpacePosition( const Vector2s& bodyvec ) const;
  
  // Given a 'world space' vector, computes the velocity assuming that point is on the rigid body
  // Inputs:
  //   worldposition: A vector in 'world space'
  // Outputs:
  //   The velocity of the input point assuming it was attached to the rigid body 
  Vector2s computeWorldSpaceVelocity( const Vector2s& worldposition ) const;
  
  // Computes the i'th edge of the rigid body in 'world space'
  // Inputs:
  //   i:       Index of an edge. Valid range is 0...N-1 where N is the number of vertices. 
  // Outputs:
  //   ith edge of the rigid body in 'world space'
  Vector2s computeWorldSpaceEdge( int i ) const;



  // Returns a reference to a Vector2s containing the total force acting on the rigid body
  Vector2s& getForce();

  // Returns a reference to a scalar containing the total torque acting on the rigid body
  scalar& getTorque();



  // Computes the total momentum of this rigid body
  Vector2s computeTotalMomentum() const;

  // Computes the contribution to angular momentum from the center of mass' velocity
  scalar computeCenterOfMassAngularMomentum() const;
    
  // Computes the contribution to angular momentum from the angular velocity computed about the center of mass
  scalar computeSpinAngularMomentum() const;
  
  // Computes the total angular momentum of this rigid body
  scalar computeTotalAngularMomentum() const;

  // Computes the contribution to kinetic energy from the center of mass' velocity
  scalar computeCenterOfMassKineticEnergy() const;
  
  // Computes the contribution to kinetic energy from the angular velocity computed about the center of mass
  scalar computeSpinKineticEnergy() const;
  
  // Computes the total kinetic energy of the rigid body
  scalar computeKineticEnergy() const;



  // Writes the state of this rigid body to the provided stream. Only writes state that should change
  //  (center of mass, orientation, center of mass' velocity, angular velocity)
  void serialize( std::ofstream& outputstream ) const;

  // Loads the state of this rigid body from the provided outputstream. Only loads state that should change
  void deserialize( std::ifstream& inputstream );

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:

  // Computes the total mass of the rigid body.
  // Inputs:
  //   masses: A flat vector containing the masses of each vertex of the rigid body.
  // Outputs:
  //   Total mass of the rigid body.
  scalar computeTotalMass( const VectorXs& masses ) const;

  // Computes the center of mass of the rigid body.
  // Inputs:
  //   vertices:  Flat vector containing the vertices of the rigid body. The hull is defined
  //              in clockwise order by the vertices. Indices 2*i and 2*i+1 define the ith vertex.
  //              vertices.size() == 2*N
  //   masses:    Flat vector containing all masses associated with vertices. masses.size() == N
  // Outputs:
  //   Center of mass of the rigid body. 
  Vector2s computeCenterOfMass( const VectorXs& vertices, const VectorXs& masses ) const;

  // Computes the moment of inertia of the rigid body
  // Inputs:
  //   vertices:  Flat vector containing the vertices of the rigid body. The hull is defined
  //              in clockwise order by the vertices. Indices 2*i and 2*i+1 define the ith vertex.
  //              vertices.size() == 2*N
  //   masses:    Flat vector containing all masses associated with vertices. masses.size() == N
  // Outputs:
  //   Moment of inertia of the rigid body.
  scalar computeMomentOfInertia( const VectorXs& vertices, const VectorXs& masses ) const;
    
  
  /////////////////////////////////////////////////////////////////////////////
  // Constant variables

  // Total mass of the rigid body
  scalar m_M;
  // Masses of each point composing the rigid body (not required, but useful for debugging)
  //VectorXs m_masses;
  // Moment of inertia of the rigid body
  scalar m_I;

  // Array containing body-space coordinates of vertices that compose the rigid body.
  VectorXs m_vertices;
  // Thickness of the rigid body. Rigid body is Minkowski sum of circle of this radius and 'centerline' of rigid body.
  scalar m_r;


  /////////////////////////////////////////////////////////////////////////////
  // State variables

  // Center of mass of the rigid body.
  Vector2s m_X;
  // Orientation of the rigid body. Simply an angle in 2d. 
  scalar m_theta;
  // Velocity of the center of mass
  Vector2s m_V;
  // Angular velocity of the rigid body
  scalar m_omega;

  
  /////////////////////////////////////////////////////////////////////////////
  // Derived variables (cached mainly for efficency)
  
  // Orientation encoded as a 2x2 rotation matrix
  Matrix2s m_R;

  /////////////////////////////////////////////////////////////////////////////
  // Total force and torque

  Vector2s m_F;
  scalar m_tau;
  
};

#endif
