#include "RigidBodyAllPairsCollisionDetector.h"

void RigidBodyAllPairsCollisionDetector::detectCollisions( const std::vector<RigidBody>& rbs, std::set<RigidBodyCollision>& collisions )
{
  // Your code goes here! 
  // Compute all vertex-edge collisions between all pairs of rigid bodies.

  collisions.clear();
  Vector2s n = Vector2s(0,-1);

  for (int i = 0; i < (int) rbs.size(); i++)
  {
    
  }


}
