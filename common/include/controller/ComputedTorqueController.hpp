#ifndef COMPUTED_TORQUE_CONTROLLER_HPP_
#define COMPUTED_TORQUE_CONTROLLER_HPP_

#include <mujoco/mujoco.h>

#include "eigen_types.hpp"

template <typename T>
class ComputedTorqueController
{
private:
  Eigen::Matrix<T, 2, 1> kp_;  // proportional gain
  Eigen::Matrix<T, 2, 1> kd_;  // derviative gain

public:
  ComputedTorqueController(const Eigen::Matrix<T, 2, 1> & kp, const Eigen::Matrix<T, 2, 1> & kd);

  void applyDesiredTorqueCommand(mjData * d);
};

#endif  // COMPUTED_TORQUE_CONTROLLER_HPP_