#ifndef __RIGID_BODY_EXPLICIT_EULER_H__
#define __RIGID_BODY_EXPLICIT_EULER_H__

#include "RigidBodyStepper.h"

class RigidBodyExplicitEuler : public RigidBodyStepper
{
public:

  virtual ~RigidBodyExplicitEuler();

  virtual bool stepScene( RigidBodyScene& scene, scalar dt );

  virtual std::string getName() const;

};

#endif
