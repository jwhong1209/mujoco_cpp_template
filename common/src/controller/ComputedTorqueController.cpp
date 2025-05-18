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
  F_ext_local_.setZero();
  F_ext_world_.setZero();
  tau_des_.setZero();

  /* set control parameters */
  kp_ << 10.0, 10.0;
  kd_ << 1, 1;

  /* create model and planner objects */
  robot_ = std::make_unique<DoublePendulumModel<T>>();
  if (!robot_)
  {
    cout << "Error: Double Pendulum Model is not created!" << endl;
  }
  else
  {
    cout << "Double Pendulum Model is created" << endl;
  }

  planner_ = std::make_unique<TrajectoryGenerator<T>>();
  if (!planner_)
  {
    cout << "Error: Trajectory Generator is not created!" << endl;
  }
  else
  {
    cout << "Trajectory Generator is created" << endl;
  }

  cout << "ComputedTorqueController (CTC) object is created and initialized" << endl;
}

template <typename T>
void ComputedTorqueController<T>::update(const mjModel * m, mjData * d)
{
  getInstance().updateImpl(m, d);
}

template <typename T>
void ComputedTorqueController<T>::updateImpl(const mjModel * m, mjData * d)
{
  // cout << "Tick:\t" << tick_ << " Time:\t" << d->time << endl;

  dof_ = m->nv;  // degree of freedom
  // cout << "DoF:\t" << dof_ << endl;

  //* update states *//
  this->getSensorData(m, d);
  robot_->setJointStates(q_mes_, dq_mes_);

  /* compute kinematics */
  Vec2<T> p = robot_->position();
  // cout << "EE position (calculated):\t" << p.transpose() << endl;
  // cout << "EE position (measured):\t" << p_mes_.transpose() << endl << endl;

  Mat2<T> J = robot_->jacobian();
  // cout << "Jacobian (computed):\n" << J << endl;

  // int site_id = mj_name2id(m, mjOBJ_SITE, "ee_site");
  // mjtNum jacp[3 * dof_] = { 0 };
  // mjtNum jacr[3 * dof_] = { 0 };
  // mj_jacSite(m, d, jacp, jacr, site_id);
  // Mat2<T> J_mj = Mat2<T>::Zero();
  // J_mj << jacp[2], jacp[3],  //
  //   jacp[4], jacp[5];        //
  // cout << "Jacobian (API):\n" << J_mj << endl;
  // cout << jacp[0] << ", " << jacp[1] << endl;
  // cout << jacp[2] << ", " << jacp[3] << endl;
  // cout << jacp[4] << ", " << jacp[5] << endl << endl;

  Vec2<T> v = robot_->velocity();
  // cout << "EE velocity (calculated):\t" << v.transpose() << endl;
  // cout << "EE velocity (measured):\t" << v_mes_.transpose() << endl;

  /* compute dynamics */
  Mat2<T> M = robot_->inertia();
  // cout << "Inertia (computed):\n" << M << endl;
  // double M_mj_arr[dof_ * dof_] = { 0 };
  // mj_fullM(m, M_mj_arr, d->qM);
  // Mat2<T> M_mj = Mat2<T>::Zero();
  // M_mj << M_mj_arr[0], M_mj_arr[1], M_mj_arr[2], M_mj_arr[3];
  // cout << "Inertia (API):\n" << M_mj << endl << endl;

  Vec2<T> tau_c = robot_->coriolis();
  // cout << "coriolis:\t" << tau_c.transpose() << endl;

  Vec2<T> tau_g = robot_->gravity();
  // cout << "gravity:\t" << tau_g.transpose() << endl;

  // Vec2<T> tau_bias = Vec2<T>::Zero();
  // for (int i = 0; i < dof_; ++i)
  // {
  //   tau_bias(i) = d->qfrc_bias[i];  // ! looks like it doesn't include coriolis
  // }
  // cout << "coriolis+gravity:\t" << (tau_c + tau_g).transpose() << endl;
  // cout << "coriolis+gravity:\t" << tau_g.transpose() << endl;
  // cout << "tau_bias:\t" << tau_bias.transpose() << endl << endl;

  //* set desired trajectory *//

  //* control law *//
  Vec2<T> q_des;
  q_des << -0.7854, 1.5708;

  tau_des_ = kp_.cwiseProduct(q_des - q_mes_) + kd_.cwiseProduct(-dq_mes_);
  tau_des_ += tau_g;

  //* send command *//
  for (int i = 0; i < dof_; ++i)
  {
    d->ctrl[i] = tau_des_[i];
  }

  tick_++;
}

/**
 * @brief Get MuJoCo sensor data in MJCF into local member variables
 */
template <typename T>
void ComputedTorqueController<T>::getSensorData(const mjModel * m, mjData * d)
{
  for (int i = 0; i < dof_; ++i)
  {
    q_mes_(i) = d->sensordata[i];
    dq_mes_(i) = d->sensordata[i + dof_];
  }

  p_mes_(0) = d->sensordata[5];        // y direction
  p_mes_(1) = d->sensordata[6] - 1.5;  // z direction (1.5 is initial height)

  v_mes_(0) = d->sensordata[12];  // y direction
  v_mes_(1) = d->sensordata[13];  // z direction (1.5 is initial height)
}

template class ComputedTorqueController<float>;
template class ComputedTorqueController<double>;