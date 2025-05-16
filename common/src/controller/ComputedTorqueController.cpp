#include "ComputedTorqueController.hpp"

template <typename T>
ComputedTorqueController<T>::ComputedTorqueController(const Eigen::Matrix<T, 2, 1> & kp,
                                                      const Eigen::Matrix<T, 2, 1> & kd)
  : kp_(kp), kd_(kd){};

template <typename T>
void ComputedTorqueController<T>::applyDesiredTorqueCommand(mjData * d)
{
}

template class ComputedTorqueController<double>;