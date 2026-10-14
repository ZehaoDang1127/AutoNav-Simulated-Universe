// Cost aggregation follows Navigation2 1.1.20 CostCritic (Apache-2.0).
// Copyright (c) 2022 Samsung Research America, @artofnothingness Alexey Budyakov
// Copyright (c) 2023 Open Navigation LLC
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy at http://www.apache.org/licenses/LICENSE-2.0
// Unless required by applicable law or agreed to in writing, software is
// distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY
// KIND, either express or implied. See the License for permissions and limits.

#include "autonav_filled_cost_critic/filled_cost_critic.hpp"
#include "nav2_costmap_2d/footprint.hpp"
#include <cmath>
#include <stdexcept>
#include "pluginlib/class_list_macros.hpp"

namespace mppi::critics
{
void FilledCostCritic::initialize()
{
  CostCritic::initialize();
  auto getParam = parameters_handler_->getParamGetter(name_);
  getParam(rollout_footprint_margin_, "rollout_footprint_margin", 0.0);
  getParam(rollout_clearance_cost_, "rollout_clearance_cost", 3000.0);
  if (!std::isfinite(rollout_footprint_margin_) || rollout_footprint_margin_ < 0 ||
    !std::isfinite(rollout_clearance_cost_) || rollout_clearance_cost_ < 0)
  {
    throw std::invalid_argument("MPPI rollout margin and clearance cost must be finite and nonnegative");
  }
  filled_checker_ = std::make_unique<nav2_smac_planner::GridCollisionChecker>(
    costmap_, 72u, parent_.lock());
}

void FilledCostCritic::score(CriticData & data)
{
  if (!enabled_) {return;}
  if (!consider_footprint_) {CostCritic::score(data); return;}
  if (!filled_checker_) {
    filled_checker_ = std::make_unique<nav2_smac_planner::GridCollisionChecker>(
      costmap_, 72u, parent_.lock());
  }
  filled_checker_->setCostmap(costmap_);
  auto footprint = costmap_ros_->getRobotFootprint();
  // The native center-cost shortcut is calculated for the live footprint.
  // It can skip a collision in our larger reserve, or in a merged layer with
  // incomplete inflation. The occupied-row index supplies the safe shortcut.
  filled_checker_->setFootprint(footprint, false, 0.0);
  // MPPI holds the local costmap lock during scoring. Rebuild the occupied
  // index on this stable map every cycle, including after rolling/clearing.
  filled_checker_->cacheCollisionBounds();
  if (rollout_footprint_margin_ > 0.0) {
    if (!clearance_checker_) {
      clearance_checker_ = std::make_unique<nav2_smac_planner::GridCollisionChecker>(
        costmap_, 72u, parent_.lock());
    }
    clearance_checker_->setCostmap(costmap_);
    nav2_costmap_2d::padFootprint(footprint, rollout_footprint_margin_);
    clearance_checker_->setFootprint(footprint, false, 0.0);
    clearance_checker_->cacheCollisionBounds();
  }
  const bool traverse_unknown = costmap_ros_->getLayeredCostmap()->isTrackingUnknown();
  const bool near_goal = utils::withinPositionGoalTolerance(
    near_goal_distance_, data.state.pose.pose, data.path);
  auto repulsive_cost = xt::xtensor<float, 1>::from_shape({data.costs.shape(0)});
  repulsive_cost.fill(0.0);
  const auto & traj = data.trajectories;
  const size_t traj_len = traj.x.shape(1);
  bool all_collide = true;
  const double resolution = costmap_->getResolution();
  for (size_t i = 0; i < traj.x.shape(0); ++i) {
    bool collide = false;
    for (size_t j = 0; j < traj_len; ++j) {
      double yaw = std::fmod(traj.yaws(i,j), 2.0*M_PI);
      if (yaw < 0.0) {yaw += 2.0*M_PI;}
      if (filled_checker_->inCollision(
          (traj.x(i,j)-costmap_->getOriginX())/resolution-.5,
          (traj.y(i,j)-costmap_->getOriginY())/resolution-.5,
          yaw*72.0/(2.0*M_PI), traverse_unknown))
      {
        collide = true;
        break;
      }
      const float center_cost = filled_checker_->getCost();
      // Extra clearance is a preference beyond the live hard body. A fresh
      // observation inside that reserve must permit departure rather than
      // making every rollout invalid at its initial pose.
      if (rollout_footprint_margin_ > 0.0 && clearance_checker_->inCollision(
          (traj.x(i,j)-costmap_->getOriginX())/resolution-.5,
          (traj.y(i,j)-costmap_->getOriginY())/resolution-.5,
          yaw*72.0/(2.0*M_PI), traverse_unknown))
      {
        repulsive_cost[i] += rollout_clearance_cost_;
      }
      if (center_cost >= nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE) {
        repulsive_cost[i] += critical_cost_;
      } else if (!near_goal) {
        repulsive_cost[i] += center_cost;
      }
    }
    if (collide) {repulsive_cost[i] = collision_cost_;}
    else {all_collide = false;}
  }
  data.costs += xt::pow((weight_ * repulsive_cost / traj_len), power_);
  data.fail_flag = all_collide;
}
}  // namespace mppi::critics

PLUGINLIB_EXPORT_CLASS(mppi::critics::FilledCostCritic, mppi::critics::CriticFunction)
