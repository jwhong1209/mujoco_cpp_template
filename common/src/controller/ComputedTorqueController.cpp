#include "ComputedTorqueController.hpp"

#include <iostream>

using namespace std;

template <typename T>
ComputedTorqueController<T>::ComputedTorqueController()
{
  this->init();
  cout << "Controller object is created" << endl;
}

template <typename T>
void ComputedTorqueController<T>::init()
{
  tau_des_.setZero();

  /* set control parameters */
}

template <typename T>
void ComputedTorqueController<T>::update(const mjModel * m, mjData * d)
{
  cout << "Enter control loop" << endl;

  int dof = q_mes_.size();

  //* update states *//
  /* compute kinematics */
  /* compute dynamics */
  Mat2<T> M;
  Vec2<T> tau_c;
  Vec2<T> tau_g;

  //* set desired trajectory *//

  //* control law *//

  //* send command *//
  for (int i = 0; i < dof; ++i)
  {
    d->ctrl[i] = tau_des_[i];
  }
}

template <typename T>
void ComputedTorqueController<T>::setControlParameters(const Vec2<T> & kp, const Vec2<T> & kd)
{
  kp_ = kp;
  kd_ = kd;
}

template class ComputedTorqueController<float>;
template class ComputedTorqueController<double>;