#include "DoublePendulumModel.hpp"

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
  return p_;
}

template <typename T>
Eigen::Matrix<T, 2, 1> DoublePendulumModel<T>::velocity()
{
  return v_;
}

template <typename T>
Eigen::Matrix<T, 2, 2> DoublePendulumModel<T>::orientatoin()
{
  return R_;
}

template <typename T>
Eigen::Matrix<T, 2, 2> DoublePendulumModel<T>::jacobian()
{
  return J_;
}

/* Dynamics */
template <typename T>
Eigen::Matrix<T, 2, 2> DoublePendulumModel<T>::inertia()
{
  return M_;
}

template <typename T>
Eigen::Matrix<T, 2, 1> DoublePendulumModel<T>::coriolis()
{
  // C_ = ;
  return C_ * dq_;
}

template <typename T>
Eigen::Matrix<T, 2, 1> DoublePendulumModel<T>::gravity()
{
  return g_;
}
