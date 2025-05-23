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

  Vec2<T> p_mes_;  // end-effector Cartesian position
  Vec2<T> v_mes_;  // end-effector Cartesian velocity

  Vec2<T> tau_des_;  // desired torque command

  Vec2<T> kp_;  // proportional gain
  Vec2<T> kd_;  // derviative gain

public:
  ComputedTorqueController();

  ComputedTorqueController(const ComputedTorqueController &) = delete;
  ComputedTorqueController & operator=(const ComputedTorqueController &) = delete;
  ComputedTorqueController(ComputedTorqueController &&) = delete;
  ComputedTorqueController & operator=(ComputedTorqueController &&) = delete;

  static ComputedTorqueController & getInstance()
  {
    static ComputedTorqueController controller;
    return controller;
  }

  static void update(const mjModel * m, mjData * d);
  void updateCallback(const mjModel * m, mjData * d);
};

#endif  // COMPUTED_TORQUE_CONTROLLER_HPP_