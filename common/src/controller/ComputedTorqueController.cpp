#include "ComputedTorqueController.hpp"

#include <iostream>

using namespace std;

template <typename T>
ComputedTorqueController<T>::ComputedTorqueController()
{
  /* initialize states */
  // q_mes_.setZero();
  q_mes_ << -0.7854, 1.5708;
  dq_mes_.setZero();
  p_init_.setZero();
  p_des_.setZero();
  p_mes_.setZero();
  v_des_.setZero();
  v_mes_.setZero();
  a_des_.setZero();
  F_ext_local_.setZero();
  F_ext_world_.setZero();
  tau_des_.setZero();

  /* set control parameters */
  kp_ << 50.0, 50.0;
  kd_ << 5, 5;

  /* create model and planner objects */
  robot_ = std::make_unique<DoublePendulumModel<T>>();
  if (!robot_)
  {
    cout << "Error: Double Pendulum Model is not created!" << endl;
  }
  else
  {
    cout << "Double Pendulum Model is created" << endl;
    robot_->setJointStates(q_mes_, dq_mes_);
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
  // cout << "Iteration:\t" << iter_ << " Time:\t" << d->time << endl;

  dof_ = m->nv;  // degree of freedom

  //* update states *//
  this->getSensorData(m, d);
  robot_->setJointStates(q_mes_, dq_mes_);

  /* compute kinematics */
  Vec2<T> p = robot_->position();
  // cout << "EE position (calculated):\t" << p.transpose() << endl;
  // cout << "EE position (measured):\t" << p_mes_.transpose() << endl << endl;

  Mat2<T> J = robot_->jacobian();
  // cout << "Jacobian (computed):\n" << J << endl;
  Mat2<T> J_t = J.transpose();  // Jacobian transpose
  Mat2<T> J_inv = J.inverse();
  Mat2<T> J_t_inv = J_t.inverse();

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
  if (M.determinant() < 1e-10)
  {
    std::cerr << "Warning: M is nearly singular !" << std::endl;
  }

  Mat2<T> Mo = J_t_inv * M * J_inv;  // operational-space inertia
  if (!Mo.allFinite())
  {
    std::cerr << "Warning: Mo contains NaN or Inf !" << std::endl;
    std::exit(EXIT_FAILURE);
  }
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
  const T traj_start_time(3);  // trajectory starts at 3 sec
  const T traj_end_time(9);    // trajectory ends at 9 sec
  const bool b_traj_running_ = (traj_start_time <= d->time && d->time < traj_end_time);

  if (b_traj_running_)
  {
    if (!b_traj_start_)
    {
      p_init_ = p_mes_;
      b_traj_start_ = true;
    }
    else
    {
      switch (traj_type_)
      {
      case TrajectoryType::CUBIC: {
        Vec2<T> p_goal = p_init_ + Vec2<T>::Constant(0.2);  // move EE 10 cm

        p_des_ = planner_->cubic(d->time, traj_start_time, traj_end_time, p_init_, p_goal,
                                 Vec2<T>::Zero());
        a_des_.setZero();
        break;
      }
      case TrajectoryType::CIRCULAR: {
        const int repeat(3);
        T circle_radius(0.1);
        T circle_freq = repeat / (traj_end_time - traj_start_time);

        p_des_ = planner_->circular(d->time, traj_start_time, circle_radius, circle_freq, p_init_);
        a_des_ = planner_->circularDDot(d->time, traj_start_time, circle_radius, circle_freq);
        break;
      }
      default:
        p_des_ = p_mes_;
        a_des_.setZero();

        break;
      }
    }
  }
  else
  {
    p_des_ = p_mes_;
    a_des_.setZero();
    b_traj_start_ = false;
  }

  //* control law *//
  /* task-space control */
  // Vec2<T> Fc = kp_.cwiseProduct(p_des_ - p_mes_) + kd_.cwiseProduct(-v_mes_);
  Vec2<T> Fc = Mo * a_des_ + kp_.cwiseProduct(p_des_ - p_mes_) + kd_.cwiseProduct(-v_mes_);
  tau_des_ = J_t * Fc;

  /* joint-space control */
  tau_des_ += tau_g;

  //* send command *//
  for (int i = 0; i < dof_; ++i)
  {
    d->ctrl[i] = tau_des_[i];
  }

  iter_++;
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
    // q_mes_(i) = d->qpos[i];
    // dq_mes_(i) = d->qvel[i];
  }

  p_mes_(0) = d->sensordata[5];        // y direction
  p_mes_(1) = d->sensordata[6] - 1.5;  // z direction (1.5 is initial height)

  v_mes_(0) = d->sensordata[12];  // y direction
  v_mes_(1) = d->sensordata[13];  // z direction (1.5 is initial height)
}

template class ComputedTorqueController<float>;
template class ComputedTorqueController<double>;