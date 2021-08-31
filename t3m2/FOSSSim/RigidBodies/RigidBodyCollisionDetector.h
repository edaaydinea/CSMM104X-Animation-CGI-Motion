#ifndef __RIGID_BODY_COLLISION_DETECTOR_H__
#define __RIGID_BODY_COLLISION_DETECTOR_H__

#include <set>
#include <string>
#include "RigidBody.h"
#include "RigidBodyCollision.h"

class RigidBodyCollisionDetector
{
public:

  virtual void detectCollisions( const std::vector<RigidBody>& rbs, std::set<RigidBodyCollision>& collisions ) = 0;

  virtual std::string getName() const = 0;
};

#endif
