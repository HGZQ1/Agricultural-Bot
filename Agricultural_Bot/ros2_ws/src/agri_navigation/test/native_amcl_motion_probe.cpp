// Exercise the installed AMCL motion library on particles without a ROS node,
// laser updates, DDS, reinitializing localization or changing runtime state.
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

#include "nav2_amcl/motion_model/differential_motion_model.hpp"
#include "nav2_amcl/motion_model/omni_motion_model.hpp"

template<class Model>
void probe(bool lateral, bool noise)
{
  constexpr int count = 20000;
  std::vector<pf_sample_t> samples(count);
  pf_t filter{};
  filter.current_set = 0;
  filter.sets[0].sample_count = count;
  filter.sets[0].samples = samples.data();
  pf_vector_t pose{}, delta{};
  delta.v[lateral ? 1 : 0] = 0.05;
  pose = delta;
  Model model;
  const double alpha = noise ? 0.2 : 0.0;
  model.initialize(alpha, alpha, alpha, alpha, noise ? 0.1 : 0.0);
  srand48(42);
  model.odometryUpdate(&filter, pose, delta);
  double mean[3]{}, variance[3]{};
  for (const auto & sample : samples) {
    for (int axis = 0; axis < 3; ++axis) {mean[axis] += sample.pose.v[axis] / count;}
  }
  for (const auto & sample : samples) {
    for (int axis = 0; axis < 3; ++axis) {
      const double error = sample.pose.v[axis] - mean[axis];
      variance[axis] += error * error / count;
    }
  }
  std::cout.precision(17);
  std::cout << "{\"particles\":" << count << ",\"mean_xyyaw\":["
            << mean[0] << ',' << mean[1] << ',' << mean[2]
            << "],\"std_xyyaw\":[" << std::sqrt(variance[0]) << ','
            << std::sqrt(variance[1]) << ',' << std::sqrt(variance[2]) << "]}\n";
}

int main(int argc, char ** argv)
{
  if (argc != 4) {throw std::invalid_argument("diff|omni lateral|forward noisy|exact");}
  const bool lateral = std::string(argv[2]) == "lateral";
  const bool noise = std::string(argv[3]) == "noisy";
  if (std::string(argv[1]) == "diff") {
    probe<nav2_amcl::DifferentialMotionModel>(lateral, noise);
  } else if (std::string(argv[1]) == "omni") {
    probe<nav2_amcl::OmniMotionModel>(lateral, noise);
  } else {
    throw std::invalid_argument("unknown model");
  }
}
