#ifndef __RIGID_BODY_STEPPER_H__
#define __RIGID_BODY_STEPPER_H__

#include <Eigen/Core>
#include <iostream>
#include "FOSSSim/MathDefs.h"
#include "RigidBodyScene.h"

class RigidBodyStepper
{
public:

  virtual ~RigidBodyStepper();

  virtual bool stepScene( RigidBodyScene& scene, scalar dt ) = 0;

  virtual std::string getName() const = 0;

};

#endif
