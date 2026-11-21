#include "nav2_smac_planner/node_lattice.hpp"
#include <cassert>
#include <cmath>
#include <iostream>

int main()
{
  using namespace nav2_smac_planner;
  NodeLattice parent(0), child(1);
  auto & table = NodeLattice::motion_table;
  table.lattice_metadata.grid_resolution = .05;
  table.cost_penalty = 4;
  table.travel_distance_reward = 1;
  table.non_straight_penalty = 1;
  table.change_penalty = 0;
  table.reverse_penalty = 2;
  table.rotation_penalty = 5;
  MotionPrimitive motion{};
  motion.trajectory_length = .1;
  motion.arc_length = 0;
  motion.poses = {MotionPose(0,0,0), MotionPose(.05,0,0), MotionPose(.1,0,0)};
  parent.pose = NodeLattice::Coordinates(-1,0,0);
  child.setMotionPrimitive(&motion);
  child.getCost() = 0;
  lane_transition_model.dividers.clear();
  const auto baseline = parent.getTraversalCost(&child);
  lane_transition_model.dividers = autonav_lane::segments({0,-10,0,10});
  lane_transition_model.cost_grid = .25/.05;
  // The first primitive must also pay the change penalty.
  assert(std::abs(parent.getTraversalCost(&child)-baseline-5) < 1e-5);
  parent.setMotionPrimitive(&motion);
  assert(std::abs(parent.getTraversalCost(&child)-baseline-5) < 1e-5);
  child.backwards(true);
  assert(std::abs(parent.getTraversalCost(&child)-baseline*2-5) < 1e-5);
  child.backwards(false);
  motion.trajectory_length = 0;
  motion.poses = {MotionPose(0,0,0), MotionPose(0,0,.1)};
  assert(std::abs(parent.getTraversalCost(&child)-5) < 1e-5);
  std::cout << "Smac native transition scoring passed\n";
}
