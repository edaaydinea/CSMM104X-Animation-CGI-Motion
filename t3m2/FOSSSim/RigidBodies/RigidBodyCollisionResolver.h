#ifndef __RIGID_BODY_COLLISION_RESOLVER_H__
#define __RIGID_BODY_COLLISION_RESOLVER_H__

#include <set>
#include <string>
#include "RigidBody.h"
#include "RigidBodyCollision.h"

class RigidBodyCollisionResolver
{
public:

  virtual void resolveCollisions( std::vector<RigidBody>& rbs, const std::set<RigidBodyCollision>& rbcs ) = 0;

  virtual std::string getName() const = 0;
};

#endif
