#ifndef DOUBLE_PENDULUM_MODEL_HPP_
#define DOUBLE_PENDULUM_MODEL_HPP_

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
  Eigen::Matrix<T, 2, 1> a_;  // Cartesian acceleration

  Eigen::Matrix<T, 2, 2> M_;
  Eigen::Matrix<T, 2, 2> C_;
  Eigen::Matrix<T, 2, 1> tau_c_;
  Eigen::Matrix<T, 2, 1> tau_g_;

  Eigen::Matrix<T, 2, 2> R_;  // rotation matrix
  Eigen::Matrix<T, 2, 2> J_, dJ_;

public:
  DoublePendulumModel(const T & l1, const T & l2, const T & d1, const T & d2,  //
                      const T & m1, const T & m2, const T & I1, const T & I2);

  void set_joint_states(const Eigen::Matrix<T, 2, 1> & q, const Eigen::Matrix<T, 2, 1> & dq);
  void compute_dynamics();

  Eigen::Matrix<T, 2, 1> get_end_effector_position();

  // Eigen::Matrix<T, 2, 2>  get_rotation_matrix();
  Eigen::Matrix<T, 2, 2> get_jacobian();
  // Eigen::Matrix<T, 2, 2>  get_jacobian_derivative();

  Eigen::Matrix<T, 2, 2> get_inertia_matrix()
  {
    return M_;
  }

  Eigen::Matrix<T, 2, 2> get_coriolis_matrix()
  {
    return C_;
  }

  Eigen::Matrix<T, 2, 1> get_coriolis_torque()
  {
    return tau_c_;
  }

  Eigen::Matrix<T, 2, 1> get_gravity_torque()
  {
    return tau_g_;
  }
};

#endif  // DOUBLE_PENDULUM_MODEL_HPP_