#ifndef COMPUTED_TORQUE_CONTROLLER_HPP_
#define COMPUTED_TORQUE_CONTROLLER_HPP_

#include <eigen3/Eigen/Dense>

template <typename T>
class ComputedTorquecontroller
{
private:
  /* data */
public:
  ComputedTorquecontroller(/* args */);
  ~ComputedTorquecontroller();

  //* ----- SETTERS --------------------------------------------------------------------------------
  void setControlParameters(const Eigen::Matrix<T, 2, 1> & kp, const Eigen::Matrix<T, 2, 1> & kd);

  //* ----- GETTERS --------------------------------------------------------------------------------
  Eigen::Matrix<T, 2, 1> getDesiredTorqueCommand();
};

template <typename T>
ComputedTorquecontroller<T>::ComputedTorquecontroller(/* args */)
{
}

template <typename T>
ComputedTorquecontroller<T>::~ComputedTorquecontroller()
{
}

#endif  // COMPUTED_TORQUE_CONTROLLER_HPP_