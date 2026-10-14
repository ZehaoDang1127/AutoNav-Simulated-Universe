#include "nav2_smac_planner/node_lattice.hpp"
#include "nav2_smac_planner/lattice_start.hpp"
#include "nav2_smac_planner/lattice_goal.hpp"
#include <algorithm>
#include <cmath>
#include <iostream>
#include <random>
#include <stdexcept>

void require(bool condition, const char * message) {
  if (!condition) throw std::runtime_error(message);
}

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  try {
    nav2_costmap_2d::Costmap2D map(200, 200, .05, -3., -3., 200);
    auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("smac_collision_regression");
    nav2_smac_planner::GridCollisionChecker checker(&map, 72u, node);
    nav2_costmap_2d::Footprint footprint;
    for (const auto & xy : {std::pair<double,double>{.92,.56}, {.92,-.56}, {-.47,-.56}, {-.47,.56}}) {
      geometry_msgs::msg::Point point;
      point.x = xy.first; point.y = xy.second;
      footprint.push_back(point);
    }
    checker.setFootprint(footprint, false, 36.);
    auto clear = [&]() {std::fill(map.getCharMap(), map.getCharMap()+40000, 200);};
    auto mark = [&](double x, double y, unsigned char cost = 254) {
      unsigned int mx, my;
      require(map.worldToMap(x,y,mx,my), "Test mark outside map");
      map.setCost(mx,my,cost);
    };
    require(!checker.inCollision(60.,60.,0.,true), "Clear padded body rejected");
    mark(.725,.225);
    require(checker.inCollision(60.,60.,0.,true), "Interior obstacle accepted");
    clear();
    mark(.975,.025);
    require(checker.inCollision(60.9,60.,0.,true), "Fractional translation discarded");
    clear();
    map.setCost(61,60,253);
    require(checker.inCollision(60.9,60.,0.,true), "Inscribed center checked in the wrong cell");
    clear();
    mark(.825,.625);
    require(checker.inCollision(60.,60.,4.9/5.,true), "Fractional yaw discarded");
    clear();
    mark(.925,.575,253);
    require(!checker.inCollision(60.,60.,0.,true), "Inflated footprint edge treated as a hard collision");
    require(checker.getCost() == 200, "Footprint-edge inflation replaced center traversal cost");
    clear();
    // The top edge is at y=.565, below this cell's center at y=.575.
    // Rasterizing the outline visits that cell although its center lies
    // outside the filled padded body used by the active route guard.
    mark(.025,.575);
    require(!checker.inCollision(59.6,59.6,0.,true), "Rasterized edge rejected an exterior cell center");
    require(checker.inCollision(60.,60.,0.,true), "Snapped start failed to reproduce interior collision");
    unsigned int start_heading = 99;
    require(nav2_smac_planner::selectCollisionFreeLatticeStart(
      checker,59.6,59.6,0.,{0.,float(M_PI/2)},true,start_heading), "Continuous start rejected");
    require(start_heading == 0, "Clear closest heading was not selected");
    mark(.025,.525);
    require(checker.inCollision(59.6,59.6,0.,true), "Filled edge-adjacent lethal cell accepted");
    require(!nav2_smac_planner::selectCollisionFreeLatticeStart(
      checker,59.6,59.6,0.,{0.,float(M_PI/2)},true,start_heading), "Occupied measured start accepted");
    clear();
    mark(.875,-.425);
    require(checker.inCollision(60.,60.,0.,true), "Nearest-heading obstacle fixture is clear");
    require(nav2_smac_planner::selectCollisionFreeLatticeStart(
      checker,60.,60.,.6,{0.,float(M_PI/2)},true,start_heading), "Safe alternate start heading rejected");
    require(start_heading == 1, "Colliding closest heading selected");
    clear();
    unsigned int goal_heading=99;
    require(nav2_smac_planner::selectCollisionFreeLatticeGoal(
      checker,60.,60.,0.,{0.,float(M_PI)},true,goal_heading) && goal_heading==0,
      "Clear requested goal heading was not preserved");
    mark(.725,.225);
    require(checker.inCollision(60.,60.,0.,true),"Blocked goal-heading fixture is clear");
    require(nav2_smac_planner::selectCollisionFreeLatticeGoal(
      checker,60.,60.,0.,{0.,float(M_PI)},true,goal_heading) && goal_heading==1,
      "Position goal could not select a clear arrival heading");
    map.setCost(60,60,253);goal_heading=99;
    require(!nav2_smac_planner::selectCollisionFreeLatticeGoal(
      checker,60.,60.,0.,{0.,float(M_PI)},true,goal_heading) && goal_heading==99,
      "Fully blocked goal claimed a clear heading or changed the fallback");
    unsigned int goal_x=60,goal_y=60;
    require(nav2_smac_planner::selectCollisionFreeLatticeGoalWithinTolerance(
      checker,map,.025,.025,0.,{0.,float(M_PI)},true,.35,goal_x,goal_y,goal_heading),
      "Blocked exact goal could not select a clear arrival within existing tolerance");
    double arrival_x,arrival_y;map.mapToWorld(goal_x,goal_y,arrival_x,arrival_y);
    require(std::hypot(arrival_x-.025,arrival_y-.025)<=.35 &&
      !checker.inCollision(goal_x,goal_y,goal_heading*36.,true),
      "Tolerant goal selection left the allowed disk or selected a blocked body");
    // Isolate center inflation from the preceding asymmetric lethal obstacle.
    clear();map.setCost(60,60,253);
    goal_x=60;goal_y=60;goal_heading=99;
    require(!nav2_smac_planner::selectCollisionFreeLatticeGoalWithinTolerance(
      checker,map,.029,.025,0.,{0.,float(M_PI)},true,.045,goal_x,goal_y,goal_heading) &&
      goal_x==60 && goal_y==60 && goal_heading==99,
      "Goal tolerance used a snapped origin or changed the fully blocked fallback");
    require(nav2_smac_planner::selectCollisionFreeLatticeGoalWithinTolerance(
      checker,map,.029,.025,0.,{0.,float(M_PI)},true,.05,goal_x,goal_y,goal_heading) &&
      goal_x==61 && goal_y==60 && goal_heading==0,
      "Nearest clear goal inside the requested world-position tolerance was not selected");
    clear();goal_x=60;goal_y=60;goal_heading=99;
    require(nav2_smac_planner::selectCollisionFreeLatticeGoalWithinTolerance(
      checker,map,.025,.025,0.,{0.,float(M_PI)},true,.35,goal_x,goal_y,goal_heading) &&
      goal_x==60 && goal_y==60 && goal_heading==0,
      "Tolerant arrival selector moved a clear requested goal");
    for (int y=50;y<=70;++y) {
      for (int x=50;x<=70;++x) {map.setCost(x,y,254);}
    }
    goal_x=60;goal_y=60;goal_heading=99;
    require(!nav2_smac_planner::selectCollisionFreeLatticeGoalWithinTolerance(
      checker,map,.025,.025,0.,{0.,float(M_PI)},true,.35,goal_x,goal_y,goal_heading) &&
      goal_x==60 && goal_y==60 && goal_heading==99,
      "Tolerant arrival selector accepted a completely occupied arrival region");
    clear();
    mark(.675,.675);
    using namespace nav2_smac_planner;
    NodeLattice::motion_table.lattice_metadata.grid_resolution = .05;
    NodeLattice::motion_table.lattice_metadata.heading_angles = {0., float(M_PI/2)};
    NodeLattice robot(0);
    robot.setPose(NodeLattice::Coordinates(60.,60.,1.));
    MotionPrimitive rotation{};
    rotation.poses = {MotionPose(0.,0.,0.), MotionPose(0.,0.,M_PI/4), MotionPose(0.,0.,M_PI/2)};
    require(!robot.isNodeValid(true, &checker, &rotation, false), "Rotation sweep obstacle skipped");
    clear();
    mark(.025,.625,255);
    mark(.725,.225);
    require(checker.inCollision(60.,60.,0.,true), "Allowed unknown masked lethal interior");
    clear();
    mark(.025,.625,255);
    require(!checker.inCollision(60.,60.,0.,true), "Allowed unknown rejected");
    require(checker.inCollision(200.,60.,0.,true), "Map upper bound accepted");
    clear();
    checker.cacheCollisionBounds();
    require(!checker.inCollision(60.,60.,0.,true), "Cached clear bounds rejected");
    mark(.725,.225);
    checker.cacheCollisionBounds();
    require(checker.inCollision(60.,60.,0.,true), "Search cache failed to refresh occupied cells");
    clear();
    mark(.975,-.675);
    checker.cacheCollisionBounds();
    require(!checker.inCollision(60.,60.,9.,true), "Bounding box treated as filled body");
    clear();
    std::mt19937 random(42);
    for (int i = 0; i < 200; ++i) {
      map.setCost(35+random()%70,35+random()%70,i%4 == 0 ? 255 : 254);
    }
    nav2_smac_planner::GridCollisionChecker uncached(&map,72u,node);
    uncached.setFootprint(footprint,false,36.);
    checker.cacheCollisionBounds();
    for (int i = 0; i < 512; ++i) {
      const float x=35+float(random()%700)/10, y=35+float(random()%700)/10;
      const float yaw=float(random()%7200)/100;
      for (bool unknown : {false,true}) {
        require(checker.inCollision(x,y,yaw,unknown) == uncached.inCollision(x,y,yaw,unknown),
          "Occupied-row optimization changed a filled-body collision decision");
      }
    }
    // A continuous query can lie almost half a cell from the inflation sample.
    // Reproduce a real inflated field whose sampled center is below the
    // shortcut threshold although a lethal cell is inside the body corner.
    nav2_costmap_2d::Footprint current_body;
    for (const auto & xy : {std::pair<double,double>{.87,.51}, {.87,-.51}, {-.42,-.51}, {-.42,.51}}) {
      geometry_msgs::msg::Point point;point.x=xy.first;point.y=xy.second;
      current_body.push_back(point);
    }
    for (unsigned int y=0;y<200;++y) for (unsigned int x=0;x<200;++x) {
      const double distance=.05*std::hypot(double(x)-77,double(y)-72);
      const unsigned char cost=distance==0 ? 254 : distance<=.42 ? 253 :
        distance<=1.25 ? static_cast<unsigned char>(252*std::exp(-3.2*(distance-.42))) : 0;
      map.setCost(x,y,cost);
    }
    nav2_smac_planner::GridCollisionChecker corner_checker(&map,72u,node);
    corner_checker.setFootprint(current_body,false,252*std::exp(-3.2*(std::hypot(.87,.51)-.42)));
    corner_checker.cacheCollisionBounds();
    require(corner_checker.inCollision(60.499,60.499,.078*72/(2*M_PI),true),
      "Inflation center shortcut accepted a lethal continuous body corner");
    require(!corner_checker.inCollision(58.,58.,0.,true),
      "Full filled-body checking rejected the neighboring clear pose");
    std::cout << "PASS: filled body, fractional translation/yaw, continuous start and safe heading connection, rotation sweep, unknown handling, cache refresh, map bounds, inflated-field corner\n";
  } catch (const std::exception & error) {
    std::cerr << "FAIL: " << error.what() << '\n';
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
