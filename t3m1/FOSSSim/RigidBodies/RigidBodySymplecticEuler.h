#ifndef __RIGID_BODY_SYMPLECTIC_EULER_H__
#define __RIGID_BODY_SYMPLECTIC_EULER_H__

#include "RigidBodyStepper.h"

class RigidBodySymplecticEuler : public RigidBodyStepper
{
public:

  virtual ~RigidBodySymplecticEuler();

  virtual bool stepScene( RigidBodyScene& scene, scalar dt );

  virtual std::string getName() const;

};

#endif
