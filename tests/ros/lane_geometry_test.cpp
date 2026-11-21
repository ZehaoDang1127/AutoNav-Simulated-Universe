#include "../../integrations/ros_lane_costmap/lane_semantics.hpp"
#include <cassert>
#include <iostream>

int main()
{
  using namespace autonav_lane;
  const auto divider = segments({0, -10, 0, 10});
  assert(crossings({-1, 0}, {1, 0}, divider) == 1);
  assert(crossings({1, 0}, {-1, 0}, divider) == 1);
  assert(crossings({-1, 0}, {-1, 8}, divider) == 0);
  assert(crossings({-1, 20}, {1, 20}, divider) == 0);
  assert(crossings({0, -1}, {0, 1}, divider) == 0);
  // Sampling a crossing into many primitive subposes must not multiply cost.
  for (int count : {2, 10, 100, 1000}) {
    unsigned changes = 0;
    for (int i = 0; i < count; ++i) {
      changes += crossings({-1+2.0*i/count, 0}, {-1+2.0*(i+1)/count, 0}, divider);
    }
    assert(changes == 1);
    for (double resolution : {.05, .1, .25}) {
      const double cost_grid = .25/resolution;
      assert(std::abs(changes*cost_grid*resolution-.25) < 1e-9);
    }
  }
  const auto joined = segments({0,-10,0,0, 0,0,0,10});
  assert(crossings({-1,0}, {1,0}, joined) == 1);
  const auto hard = segments({-5,0,5,0});
  assert(crossable({0,3}, divider, hard, .12));
  assert(!crossable({0,0}, divider, hard, .12));
  assert(!crossable({2,2}, divider, hard, .12));
  // Paint consists of separate dashes, while the transition centreline spans
  // their gap. Crossing that gap is still a single lane change.
  const auto dashed_paint = segments({0,-2,0,-1, 0,1,0,2});
  const auto dashed_centerline = segments({0,-2,0,2});
  assert(crossable({0,-1.5}, dashed_paint, {}, .12));
  assert(!crossable({0,0}, dashed_paint, {}, .12));
  assert(crossings({-1,0}, {1,0}, dashed_centerline) == 1);
  assert(crossings({-1,0}, {0,0}, dashed_centerline) +
    crossings({0,0}, {1,0}, dashed_centerline) == 1);
  const auto crossing_boundary = segments({-1,-1.5,1,-1.5});
  assert(!crossable({0,-1.5}, dashed_paint, crossing_boundary, .12));
  assert(quadratic_cost(0,.05,.8) == 254);
  assert(quadratic_cost(.04,.05,.8) == 253);
  assert(quadratic_cost(.25,.05,.8) > quadratic_cost(.55,.05,.8));
  assert(quadratic_cost(.85,.05,.8) == 0);
  for (const auto & bad : std::vector<std::vector<double>>{{1}, {0,0,0,0}, {0,0,INFINITY,1}}) {
    bool rejected = false;
    try {segments(bad);} catch (const std::invalid_argument &) {rejected = true;}
    assert(rejected);
  }
  std::cout << "Lane geometry: crossings, sample/resolution invariance, hard priority, clearance and validation passed\n";
}
