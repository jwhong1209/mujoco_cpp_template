#ifndef DOUBLE_PENDULUM_MODEL_HPP_
#define DOUBLE_PENDULUM_MODEL_HPP_

#include <mujoco/mujoco.h>
#include <eigen3/Eigen/Dense>

template <typename T>
class DoublePendulumModel
{
private:
  T l1_, l2_;  // link length
  T d1_, d2_;  // distance from joint axis to link's CoM
  T m1_, m2_;  // link's mass
  T I1_, I2_;  // link's momentum of inertia w.r.t CoM
  T J1_, J2_;  // link's momentum of inertia w.r.t axis

  Eigen::Matrix<T, 2, 1> q_;    // joint pos
  Eigen::Matrix<T, 2, 1> dq_;   // joint vel
  Eigen::Matrix<T, 2, 1> ddq_;  // joint acc

  Eigen::Matrix<T, 2, 1> p_;  // Cartesian position
  Eigen::Matrix<T, 2, 1> v_;  // Cartesian velocity
  // Eigen::Matrix<T, 2, 1> a_;  // Cartesian acceleration

  Eigen::Matrix<T, 2, 2> M_;
  Eigen::Matrix<T, 2, 2> C_;
  Eigen::Matrix<T, 2, 1> tau_c_;
  Eigen::Matrix<T, 2, 1> tau_g_;

  Eigen::Matrix<T, 2, 2> R_;  // rotation matrix
  Eigen::Matrix<T, 2, 2> J_;  // Jacobian matrix

public:
  DoublePendulumModel(const T & l1, const T & l2,   //
                      const T & d1, const T & d2,   //
                      const T & m1, const T & m2,   //
                      const T & I1, const T & I2);  //

  //* ----- SETTERS --------------------------------------------------------------------------------
  void setJointStates(const Eigen::Matrix<T, 2, 1> & q, const Eigen::Matrix<T, 2, 1> & dq);

  //* ----- GETTERS --------------------------------------------------------------------------------
  /* Kinematics */
  Eigen::Matrix<T, 2, 1> position();
  Eigen::Matrix<T, 2, 2> orientatoin();
  Eigen::Matrix<T, 2, 2> jacobian();

  /* Dynamics */
  Eigen::Matrix<T, 2, 2> inertia();
  Eigen::Matrix<T, 2, 1> coriolis();
  Eigen::Matrix<T, 2, 1> gravity();

  //* ----- PRINTER --------------------------------------------------------------------------------
};

#endif  // DOUBLE_PENDULUM_MODEL_HPP_