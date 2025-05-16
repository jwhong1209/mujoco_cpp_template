#include "ComputedTorqueController.hpp"

template <typename T>
ComputedTorqueController<T>::ComputedTorqueController(const Eigen::Matrix<T, 2, 1> & kp,
                                                      const Eigen::Matrix<T, 2, 1> & kd)
  : kp_(kp), kd_(kd){};

template <typename T>
void ComputedTorqueController<T>::applyDesiredTorqueCommand(mjData * d)
{
  // for (int i = 0;)
}

template class ComputedTorqueController<float>;
template class ComputedTorqueController<double>;