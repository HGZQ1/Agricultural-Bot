// Exercise the installed Jazzy generator library without rclcpp::init, a node,
// DDS, a costmap or motion.  The subclasses only supply the parameters normally
// set by initialize(); sampling and trajectory integration use installed code.
#include <algorithm>
#include <cmath>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <type_traits>

#include "dwb_plugins/limited_accel_generator.hpp"
#include "dwb_plugins/xy_theta_iterator.hpp"
#include "dwb_critics/obstacle_footprint.hpp"
#include "dwb_core/exceptions.hpp"
#include "nav2_costmap_2d/cost_values.hpp"

class Parameters : public dwb_plugins::KinematicParameters
{
public:
  explicit Parameters(double acceleration)
  {
    min_vel_x_ = -0.12; max_vel_x_ = 0.12;
    min_vel_y_ = -0.08; max_vel_y_ = 0.08;
    max_vel_theta_ = 0.05;
    min_speed_xy_ = 0.0; max_speed_xy_ = 0.15;
    min_speed_theta_ = 0.0;
    min_speed_xy_sq_ = 0.0; max_speed_xy_sq_ = 0.15 * 0.15;
    acc_lim_x_ = acceleration; acc_lim_y_ = acceleration;
    decel_lim_x_ = -0.05; decel_lim_y_ = -0.05;
    acc_lim_theta_ = 0.1; decel_lim_theta_ = -0.1;
  }
};

class Handler : public dwb_plugins::KinematicsHandler
{
public:
  explicit Handler(double acceleration) {update_kinematics(Parameters(acceleration));}
};

class Iterator : public dwb_plugins::XYThetaIterator
{
public:
  explicit Iterator(const dwb_plugins::KinematicsHandler::Ptr & handler)
  {
    kinematics_handler_ = handler;
    vx_samples_ = 11; vy_samples_ = 11; vtheta_samples_ = 5;
  }
};

template<class Generator>
class ConfiguredGenerator : public Generator
{
public:
  explicit ConfiguredGenerator(double acceleration)
  {
    this->kinematics_handler_ = std::make_shared<Handler>(acceleration);
    this->velocity_iterator_ = std::make_shared<Iterator>(this->kinematics_handler_);
    this->sim_time_ = 1.5;
    this->discretize_by_time_ = false;
    this->time_granularity_ = 0.5;
    this->linear_granularity_ = 0.025;
    this->angular_granularity_ = 0.01;
    this->include_last_point_ = true;
    this->limit_vel_cmd_in_traj_ = false;
    if constexpr (std::is_same_v<Generator, dwb_plugins::LimitedAccelGenerator>) {
      this->acceleration_time_ = 0.05;
    }
  }
};

template<class Generator>
void probe(double acceleration)
{
  ConfiguredGenerator<Generator> generator(acceleration);
  nav_2d_msgs::msg::Twist2D stopped;
  geometry_msgs::msg::Pose2D origin;
  generator.startNewIteration(stopped);
  size_t samples = 0, forward_cell_changes = 0, lateral_cell_changes = 0;
  double max_x = 0.0, max_y = 0.0;
  double forward_endpoint = 0.0, lateral_endpoint = 0.0;
  double returned_forward = 0.0, returned_lateral = 0.0;
  while (generator.hasMoreTwists()) {
    const auto target = generator.nextTwist();
    const auto trajectory = generator.generateTrajectory(origin, stopped, target);
    ++samples;
    max_x = std::max(max_x, target.x);
    max_y = std::max(max_y, target.y);
    if (std::abs(target.theta) > 1e-12) {continue;}
    const auto endpoint = trajectory.poses.back();
    if (target.x > 1e-12 && endpoint.x >= 0.05) {++forward_cell_changes;}
    if (target.y > 1e-12 && endpoint.y >= 0.05) {++lateral_cell_changes;}
    if (std::abs(target.y) < 1e-12 && endpoint.x > forward_endpoint) {
      forward_endpoint = endpoint.x;
      returned_forward = trajectory.velocity.x;
    }
    if (std::abs(target.x) < 1e-12 && endpoint.y > lateral_endpoint) {
      lateral_endpoint = endpoint.y;
      returned_lateral = trajectory.velocity.y;
    }
  }
  std::cout.precision(17);
  std::cout << "{\"samples\":" << samples
            << ",\"max_target_x\":" << max_x
            << ",\"max_target_y\":" << max_y
            << ",\"forward_endpoint_m\":" << forward_endpoint
            << ",\"lateral_endpoint_m\":" << lateral_endpoint
            << ",\"returned_forward_command\":" << returned_forward
            << ",\"returned_lateral_command\":" << returned_lateral
            << ",\"forward_cell_changes\":" << forward_cell_changes
            << ",\"lateral_cell_changes\":" << lateral_cell_changes << "}\n";
}

class FootprintCritic : public dwb_critics::ObstacleFootprintCritic
{
public:
  explicit FootprintCritic(double scale)
  : grid_(80, 80, 0.05, -2.0, -2.0, nav2_costmap_2d::FREE_SPACE)
  {
    name_ = "ObstacleFootprint";
    scale_ = scale;
    costmap_ = &grid_;
    sum_scores_ = false;
    for (const auto & xy : {std::pair{0.495, 0.395}, std::pair{0.495, -0.395},
                            std::pair{-0.535, -0.395}, std::pair{-0.535, 0.395}}) {
      geometry_msgs::msg::Point point;
      point.x = xy.first; point.y = xy.second;
      footprint_spec_.push_back(point);
    }
  }
  void edgeCost(unsigned char cost)
  {
    unsigned int x, y;
    if (!grid_.worldToMap(0.0, 0.395, x, y)) {throw std::runtime_error("bad test cell");}
    grid_.setCost(x, y, cost);
  }
private:
  nav2_costmap_2d::Costmap2D grid_;
};

void probeCritic(double scale)
{
  FootprintCritic critic(scale);
  dwb_msgs::msg::Trajectory2D stationary;
  stationary.poses.emplace_back();
  critic.edgeCost(nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE);
  const double legal_cost = critic.scoreTrajectory(stationary);
  int rejected = 0;
  for (const auto cost : {nav2_costmap_2d::LETHAL_OBSTACLE,
                          nav2_costmap_2d::NO_INFORMATION}) {
    critic.edgeCost(cost);
    try {(void)critic.scoreTrajectory(stationary);}
    catch (const dwb_core::IllegalTrajectoryException &) {++rejected;}
  }
  critic.edgeCost(nav2_costmap_2d::FREE_SPACE);
  stationary.poses.front().x = 1.8;  // center on-grid, front footprint off-grid
  try {(void)critic.scoreTrajectory(stationary);}
  catch (const dwb_core::IllegalTrajectoryException &) {++rejected;}
  std::cout.precision(17);
  std::cout << "{\"inflated_raw_score\":" << legal_cost
            << ",\"effective_scale\":" << critic.getScale()
            << ",\"weighted_inflated_score\":" << legal_cost * critic.getScale()
            << ",\"hard_rejections\":" << rejected << "}\n";
}

int main(int argc, char ** argv)
{
  if (argc != 3) {throw std::invalid_argument("generator-name acceleration");}
  const double acceleration = std::stod(argv[2]);
  if (std::string(argv[1]) == "limited") {
    probe<dwb_plugins::LimitedAccelGenerator>(acceleration);
  } else if (std::string(argv[1]) == "standard") {
    probe<dwb_plugins::StandardTrajectoryGenerator>(acceleration);
  } else if (std::string(argv[1]) == "footprint") {
    probeCritic(acceleration);
  } else {
    throw std::invalid_argument("unknown generator");
  }
}
