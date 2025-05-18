#ifndef TRAJECTORY_GENERATOR_HPP_
#define TRAJECTORY_GENERATOR_HPP_

// TODO: Add simple trajectory e.g. sinusoidal or circular

template <typename T>
class TrajectoryGenerator
{
private:
  T traj_time_;  //

public:
  TrajectoryGenerator(){};

  void step();
  void sinusoidal();

  void cubic();
  void circular();
};

#endif  // TRAJECTORY_GENERATOR_HPP_