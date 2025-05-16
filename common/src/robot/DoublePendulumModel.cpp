#include "DoublePendulumModel.hpp"

#include <cmath>

template <typename T>
DoublePendulumModel<T>::DoublePendulumModel(const T & l1, const T & l2,  //
                                            const T & d1, const T & d2,  //
                                            const T & m1, const T & m2,  //
                                            const T & I1, const T & I2)
  : l1_(l1), l2_(l2), d1_(d2), d2_(d2), m1_(m1), m2_(m2), I1_(I1), I2_(I2)
{
  /* compute links' momentum of inertia w.r.t rotating axis */
  J1_ = I1_ + m1_ * (d1_ * d1_);
  J2_ = I2_ + m2_ * (d2_ * d2_);
}

//* ----- SETTERS ----------------------------------------------------------------------------------
template <typename T>
void DoublePendulumModel<T>::setJointStates(const Eigen::Matrix<T, 2, 1> & q,
                                            const Eigen::Matrix<T, 2, 1> & dq)
{
  q_ = q;
  dq_ = dq;
}

//* ----- GETTERS ----------------------------------------------------------------------------------
/* Kinematics */
template <typename T>
Eigen::Matrix<T, 2, 1> DoublePendulumModel<T>::position()
{
  const T q1 = q_(0);
  const T q12 = q_(0) + q_(1);

  p_(0) = l1_ * std::cos(q1) + l2_ * std::cos(q12);  // x-coordinate
  p_(1) = l1_ * std::sin(q1) + l2_ * std::sin(q12);  // y-coordinate
  return p_;
}

template <typename T>
Eigen::Matrix<T, 2, 1> DoublePendulumModel<T>::velocity()
{
  v_ = J_ * dq_;
  return v_;
}

template <typename T>
Eigen::Matrix<T, 2, 2> DoublePendulumModel<T>::orientatoin()
{
  const T q1 = q_(1);
  const T q2 = q_(1);

  R_ << std::sin(q1), std::cos(q2),  //
    -std::cos(q2), std::sin(q1);

  return R_;
}

template <typename T>
Eigen::Matrix<T, 2, 2> DoublePendulumModel<T>::jacobian()
{
  const T q1 = q_(0);
  const T q12 = q_(0) + q_(1);

  J_(0, 0) = -l1_ * std::sin(q1) - l2_ * std::sin(q12);
  J_(0, 1) = -l2_ * std::sin(q12);
  J_(1, 0) = l1_ * std::cos(q1) + l2_ * std::cos(q12);
  J_(1, 1) = l2_ * std::cos(q12);

  return J_;
}

/* Dynamics */
template <typename T>
Eigen::Matrix<T, 2, 2> DoublePendulumModel<T>::inertia()
{
  const T q2 = q_(1);

  M_(0, 0) = J1_ + J2_ + m2_ * (l2_ * l2_) + 2 * m2_ * d2_ * l2_ * std::cos(q2);
  M_(0, 1) = J2_ + m2_ * d2_ * l2_ * std::cos(q2);
  M_(1, 0) = J2_ + m2_ * d2_ * l2_ * std::cos(q2);
  M_(1, 1) = J2_;

  return M_;
}

template <typename T>
Eigen::Matrix<T, 2, 1> DoublePendulumModel<T>::coriolis()
{
  const T q1 = q_(0);
  const T q2 = q_(1);

  tau_c_(0) = -m2_ * d2_ * l2_ * std::sin(q2) * (q2 * q2 + q1 * q2);
  tau_c_(1) = m2_ * d2_ * l2 * std::sin(q2) * (q1 * q1);

  return tau_c_;
}

template <typename T>
Eigen::Matrix<T, 2, 1> DoublePendulumModel<T>::gravity()
{
  const T g = 9.81;
  const T q1 = q_(0);
  const T q12 = q_(0) + q_(1);

  tau_g_(0) = g * (m1_ * d1_ + m2_ * l1_) * std::cos(q1) + g * m2_ * d2_ * std::cos(q12);
  tau_g_(1) = g * m2_ * d2_ * std::cos(q12);

  return tau_g_;
}
