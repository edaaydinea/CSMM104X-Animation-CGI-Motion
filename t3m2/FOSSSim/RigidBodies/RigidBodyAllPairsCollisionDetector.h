#ifndef __RIGID_BODY_ALL_PAIRS_COLLISION_DETECTOR_H__
#define __RIGID_BODY_ALL_PAIRS_COLLISION_DETECTOR_H__

#include <iostream>
#include <vector>
#include "RigidBodyCollisionDetector.h"
#include "FOSSSim/MathDefs.h"

class RigidBodyAllPairsCollisionDetector : public RigidBodyCollisionDetector
{
public:

  // Detects all collisions between all rigid bodies in the scene and returns them in the collisions set.
  virtual void detectCollisions( const std::vector<RigidBody>& rbs, std::set<RigidBodyCollision>& collisions );

  std::string getName() const;
};

#endif

