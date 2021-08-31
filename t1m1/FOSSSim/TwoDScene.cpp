#include "TwoDScene.h"

scalar TwoDScene::computeKineticEnergy() const
{
  // Your code goes here!
  // KineticEnergy of the system is T = 1/2 * mv * v
  return (1.0 / 2) * m_v.dot(VectorXs(m_m.array() * m_v.array()));
}

