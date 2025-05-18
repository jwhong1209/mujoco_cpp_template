#include "ComputedTorqueController.hpp"

#include <iostream>

using namespace std;

template <typename T>
ComputedTorqueController<T>::ComputedTorqueController()
{
  /* initialize states */
  q_mes_.setZero();
  dq_mes_.setZero();
  p_mes_.setZero();
  v_mes_.setZero();
  tau_des_.setZero();

  /* set control parameters */
  kp_ << 10.0, 10.0;
  kd_ << 0.1, 0.1;

  cout << "ComputedTorqueController (CTC) object is created and initialized" << endl;
}

template <typename T>
void ComputedTorqueController<T>::update(const mjModel * m, mjData * d)
{
  getInstance().updateCallback(m, d);
}

template <typename T>
void ComputedTorqueController<T>::updateCallback(const mjModel * m, mjData * d)
{
  const int DOF = m->nv;  // degree of freedom

  //* update states *//
  /* compute kinematics */
  /* compute dynamics */
  Mat2<T> M;
  Vec2<T> tau_c;
  Vec2<T> tau_g;

  //* set desired trajectory *//

  //* control law *//

  //* send command *//
  for (int i = 0; i < DOF; ++i)
  {
    d->ctrl[i] = tau_des_[i];
  }
}

template class ComputedTorqueController<float>;
template class ComputedTorqueController<double>;