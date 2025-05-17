#ifndef COMPUTED_TORQUE_CONTROLLER_HPP_
#define COMPUTED_TORQUE_CONTROLLER_HPP_

#include <mutex>

#include <mujoco/mujoco.h>

#include "DoublePendulumModel.hpp"
#include "TrajectoryGenerator.hpp"

template <typename T>
class ComputedTorqueController
{
private:
  DoublePendulumModel<T> robot_;
  TrajectoryGenerator<T> planner_;

  std::mutex state_mutex_;
  std::mutex command_mutex_;

  Vec2<T> q_mes_;
  Vec2<T> dq_mes_;

  Vec2<T> kp_;  // proportional gain
  Vec2<T> kd_;  // derviative gain

  Vec2<T> tau_des_;  // desired torque command

public:
  ComputedTorqueController();

  void init();
  void update(const mjModel * m, mjData * d);

  void setControlParameters(const Vec2<T> & kp, const Vec2<T> & kd);
};

#endif  // COMPUTED_TORQUE_CONTROLLER_HPP_