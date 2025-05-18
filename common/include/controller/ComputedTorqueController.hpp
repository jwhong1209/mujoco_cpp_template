#ifndef COMPUTED_TORQUE_CONTROLLER_HPP_
#define COMPUTED_TORQUE_CONTROLLER_HPP_

#include <memory>
#include <mutex>

#include <mujoco/mujoco.h>

#include "DoublePendulumModel.hpp"
#include "TrajectoryGenerator.hpp"

template <typename T>
class ComputedTorqueController
{
private:
  std::unique_ptr<DoublePendulumModel<T>> robot_;
  std::unique_ptr<TrajectoryGenerator<T>> planner_;

  // TODO: Check whether these mutexs are necessary
  // std::mutex state_mtx_;
  // std::mutex command_mtx_;
  std::mutex logging_mtx_;

  int tick_ = 0;
  T time_ = 0.0;
  T traj_time_ = 0.0;

  int dof_;

  Vec2<T> q_mes_;   // joint position
  Vec2<T> dq_mes_;  // joint velocity

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
  void updateImpl(const mjModel * m, mjData * d);  // * Implement controller here

  /**
   * @brief Get MuJoCo sensor data in MJCF into local member variables
   */
  void getSensorData(const mjModel * m, mjData * d);

  // TODO: Implement data logging in separated thread
  // void dataLoggingLoop();
};

#endif  // COMPUTED_TORQUE_CONTROLLER_HPP_