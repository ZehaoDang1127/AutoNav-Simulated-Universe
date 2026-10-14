#include "autonav_filled_cost_critic/filled_cost_critic.hpp"
#include "pluginlib/class_loader.hpp"
#include <iostream>
#include <stdexcept>

void require(bool condition, const char * message)
{
  if (!condition) {throw std::runtime_error(message);}
}

class TestCostmap : public nav2_costmap_2d::Costmap2DROS
{
public:
  TestCostmap() : Costmap2DROS("filled_costmap_regression")
  {
    layered_costmap_ = std::make_unique<nav2_costmap_2d::LayeredCostmap>("map",false,true);
    layered_costmap_->resizeMap(200,200,.05,-3.,-3.);
    getCostmap()->setDefaultValue(200);
    getCostmap()->resetMap(0,0,200,200);
    nav2_costmap_2d::Footprint body;
    for (const auto & xy : {std::pair<double,double>{.92,.56},{.92,-.56},{-.47,-.56},{-.47,.56}}) {
      geometry_msgs::msg::Point p; p.x=xy.first; p.y=xy.second; body.push_back(p);
    }
    footprint_padding_=0;
    setRobotFootprint(body);
  }
};

template<typename Base>
class TestCritic : public Base
{
public:
  void reserve(double margin) {this->rollout_footprint_margin_=margin;}
  void cacheLiveThreshold(float cost) {
    this->circumscribed_radius_=this->costmap_ros_->getLayeredCostmap()->getCircumscribedRadius();
    this->circumscribed_cost_=cost;
  }
  void prepare(std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node, std::shared_ptr<TestCostmap> map)
  {
    this->parent_=node;
    this->costmap_ros_=map;
    this->costmap_=map->getCostmap();
    this->collision_checker_.setCostmap(this->costmap_);
    this->enabled_=true;
    this->consider_footprint_=true;
    this->power_=1;
    this->weight_=1.6/254.;
    this->critical_cost_=300;
    this->collision_cost_=1e6;
    this->near_goal_distance_=.6;
    this->circumscribed_radius_=-1;
    this->possibly_inscribed_cost_=-1;
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc,argv);
  try {
    pluginlib::ClassLoader<mppi::critics::CriticFunction> loader(
      "nav2_mppi_controller","mppi::critics::CriticFunction");
    auto loaded=loader.createUniqueInstance("mppi::critics::FilledCostCritic");
    require(loaded != nullptr,"Filled critic plugin is not discoverable by MPPI");
    auto node=std::make_shared<rclcpp_lifecycle::LifecycleNode>("filled_critic_regression");
    auto map=std::make_shared<TestCostmap>();
    TestCritic<mppi::critics::CostCritic> original;
    TestCritic<mppi::critics::FilledCostCritic> filled;
    original.prepare(node,map); filled.prepare(node,map);
    mppi::models::State state;
    state.pose.pose.orientation.w=1;
    mppi::models::Trajectories trajectories;
    trajectories.reset(1,1);
    trajectories.x(0,0)=trajectories.y(0,0)=.025;
    mppi::models::Path path;
    path.reset(2); path.x(1)=path.y(1)=3.;
    auto costs=xt::xtensor<float,1>::from_shape({1});
    float dt=.05;
    mppi::CriticData data{state,trajectories,path,costs,dt,false,nullptr,nullptr,std::nullopt,std::nullopt};
    const auto score=[&](mppi::critics::CostCritic & critic) {
        costs.fill(0); data.fail_flag=false; critic.score(data); return data.fail_flag;
      };
    map->getCostmap()->setCost(60,60,253);
    require(!score(original),"Original inscribed-center mismatch was not reproduced");
    require(score(filled),"Filled critic accepted an inscribed center");
    map->getCostmap()->resetMap(0,0,200,200);
    map->getCostmap()->setCost(74,64,254);
    require(!score(original),"Original interior-obstacle mismatch was not reproduced");
    require(score(filled),"Filled critic accepted a lethal interior obstacle");
    trajectories.reset(2,1);
    trajectories.x(0,0)=trajectories.y(0,0)=.025;
    trajectories.x(1,0)=trajectories.y(1,0)=-1.5;
    costs=xt::xtensor<float,1>::from_shape({2});
    require(!score(filled),"One safe alternative incorrectly failed every trajectory");
    require(costs(0)>1000*costs(1),"Colliding candidate was not penalized over a safe alternative");
    map->getCostmap()->resetMap(0,0,200,200);
    require(!score(filled) && costs(0)<10,"Occupied index did not refresh after a map clear");
    trajectories.x(0,0)=trajectories.y(0,0)=.005;
    map->getCostmap()->setCost(60,71,254);
    require(!score(filled) && costs(0)<10,"Exterior cell center was treated as filled-body overlap");
    filled.reserve(.1);
    require(!score(filled) && costs(0)>10*costs(1),
      "Clearance cost failed to prefer a safe alternative over a near-edge candidate");
    trajectories.reset(1,1);
    trajectories.x(0,0)=trajectories.y(0,0)=.005;
    costs=xt::xtensor<float,1>::from_shape({1});
    require(!score(filled) && costs(0)>10,
      "Extra clearance either failed to penalize an occupied reserve or blocked every safe departure");
    map->getCostmap()->setDefaultValue(0);
    map->getCostmap()->resetMap(0,0,200,200);
    map->getCostmap()->setCost(60,71,254);
    filled.cacheLiveThreshold(50);
    require(!score(filled) && costs(0)>10,
      "Live-footprint center-cost shortcut skipped the extra clearance cost");
    map->getCostmap()->setCost(60,70,254);
    require(score(filled),"Reserve admitted a lethal live-body overlap");
    std::cout << "PASS: filled body, safe alternatives, map refresh, reserve, low-cost shortcut, live guard\n";
  } catch (const std::exception & error) {
    std::cerr << "FAIL: " << error.what() << '\n';
    rclcpp::shutdown(); return 1;
  }
  rclcpp::shutdown(); return 0;
}
