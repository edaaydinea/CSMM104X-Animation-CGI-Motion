#include "RigidBodyWindForce.h"

void RigidBodyWindForce::computeForceAndTorque( std::vector<RigidBody>& rbs )
{
  // Your code goes here!
  // for all rigid bodies i rbs[i].getForce()  += ... some force you compute ...
  //                        rbs[i].getTorque() += ... some torque you compute ...

  int num_quad_points = m_num_quadrature_points;
  scalar division_by_quad = 1.0 / num_quad_points;
  scalar beta_by_quad = m_beta * division_by_quad;

  for (std::vector<RigidBody>::size_type index = 0; index < rbs.size(); ++index)
  {
    int num_edge_points = rbs[index].getNumEdges();
    Vector2s center_of_mass = rbs[index].getX();

    for (int i = 0; i < num_edge_points; ++i)
    {
      Vector2s vertex = rbs[index].getWorldSpaceVertex(i);
      Vector2s edge = rbs[index].computeWorldSpaceEdge(i);
      Vector2s n_hat(-edge.y(),edge.x());
      scalar length =n_hat.norm();
      assert(length != 0.0);
      n_hat = n_hat / length;

      for (int j = 0;j < num_quad_points; ++j)
      {
        Vector2s j_position = vertex + edge * ((j + 0.5) * division_by_quad);
        Vector2s j_velocity = rbs[index].computeWorldSpaceVelocity(j_position);

        // center of mass
        Vector2s relative_position = j_position - center_of_mass;
        // wind
        Vector2s relative_velocity = n_hat * (m_wind - j_velocity).dot(n_hat);

        Vector2s wind_force = (length * beta_by_quad) * relative_velocity;

        rbs[index].getForce() += wind_force;
        rbs[index].getTorque() += relative_position.x() + wind_force.y() - relative_position.y() * wind_force.x();
      }
    }
  }

  return;
}
