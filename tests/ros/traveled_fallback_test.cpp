#include "nav2_smac_planner/smac_planner_lattice.hpp"
#include <iostream>
#include <stdexcept>
class TestMap : public nav2_costmap_2d::Costmap2DROS {
public:
  TestMap():Costmap2DROS("traveled_fallback_costmap") {
    global_frame_="odom";
    layered_costmap_=std::make_unique<nav2_costmap_2d::LayeredCostmap>("odom",false,true);
    layered_costmap_->resizeMap(240,240,.05,0,0);
    getCostmap()->setDefaultValue(0);getCostmap()->resetMap(0,0,240,240);
    std::vector<geometry_msgs::msg::Point> body;
    for(auto xy:{std::pair<double,double>{.87,.51},{.87,-.51},{-.42,-.51},{-.42,.51}}) {
      geometry_msgs::msg::Point p;p.x=xy.first;p.y=xy.second;body.push_back(p);
    }
    footprint_padding_=0;setRobotFootprint(body);
    for(unsigned int y=64;y<=136;++y) for(unsigned int x=96;x<=104;++x) getCostmap()->setCost(x,y,254);
  }
};
class TestPlanner : public nav2_smac_planner::SmacPlannerLattice {
public:
  void seed() {
    for(int leg=0;leg<3;++leg) for(int i=0;i<=90;++i) {
      _traveled_corridor.record(leg%2 ? 10-i*.1 : 1+i*.1,5,"odom");
    }
  }
  void seedGlobal() {
    // map -> odom is (+2, -1). Store the U-shaped route in map coordinates.
    for(int i=0;i<=90;++i) _traveled_corridor.record(-1+i*.1,4,"map");
    for(int i=1;i<=70;++i) _traveled_corridor.record(8,4+i*.1,"map");
    for(int i=1;i<=90;++i) _traveled_corridor.record(8-i*.1,11,"map");
  }
  std::string historyFrame() const {return _traveled_corridor.frame();}
};
int main(int argc,char ** argv) {
  rclcpp::init(argc,argv);
  try {
    std::vector<rclcpp::Parameter> params{
      rclcpp::Parameter("GridBased.prefer_traveled_return",true),
      rclcpp::Parameter("GridBased.allow_goal_heading_change",true),
      rclcpp::Parameter("GridBased.max_planning_time",5.0),
      rclcpp::Parameter("GridBased.smooth_path",false),
      rclcpp::Parameter("GridBased.lattice_filepath",std::string("/opt/ros/humble/share/nav2_smac_planner/sample_primitives/5cm_resolution/0.5m_turning_radius/diff/output.json"))};
    auto node=std::make_shared<rclcpp_lifecycle::LifecycleNode>("traveled_fallback_test",rclcpp::NodeOptions().parameter_overrides(params));
    auto map=std::make_shared<TestMap>();TestPlanner planner;
    planner.configure(node,"GridBased",nullptr,map);planner.seed();
    geometry_msgs::msg::PoseStamped start,goal;start.header.frame_id=goal.header.frame_id="odom";
    start.pose.position.x=10;start.pose.position.y=5;start.pose.orientation.z=1;
    goal.pose.position.x=1;goal.pose.position.y=5;goal.pose.orientation.z=1;
    auto plan=planner.createPlan(start,goal);
    if(plan.poses.empty()) throw std::runtime_error("Blocked learned corridor did not fall back to normal planning");
    bool outside=false;
    for(const auto & p:plan.poses) {
      outside|=std::abs(p.pose.position.y-5)>1.5;
      unsigned int x,y;
      if(!map->getCostmap()->worldToMap(p.pose.position.x,p.pose.position.y,x,y) || map->getCostmap()->getCost(x,y)>=253)
        throw std::runtime_error("Fallback crossed the sensed blocking wall");
    }
    if(!outside) throw std::runtime_error("Fallback retained the impossible learned corridor");
    planner.cleanup();
    map->getCostmap()->resetMap(0,0,240,240);
    auto tf=std::make_shared<tf2_ros::Buffer>(node->get_clock());
    geometry_msgs::msg::TransformStamped transform;
    transform.header.frame_id="odom";transform.child_frame_id="map";
    transform.transform.translation.x=2;transform.transform.translation.y=-1;
    transform.transform.rotation.w=1;
    tf->setTransform(transform,"test",true);
    TestPlanner global_planner;
    global_planner.configure(node,"GridBased",tf,map);global_planner.seedGlobal();
    start.pose.position.x=1;start.pose.position.y=10;
    start.pose.orientation.z=0;start.pose.orientation.w=1;
    goal.pose.position.x=1;goal.pose.position.y=3;
    plan=global_planner.createPlan(start,goal);
    if(plan.poses.empty()) throw std::runtime_error("Global history TF produced no learned route");
    bool known_turn=false;
    for(const auto & p:plan.poses) known_turn|=p.pose.position.x>8;
    if(!known_turn) throw std::runtime_error("Global history was lost; planner crossed untraveled shortcut");
    if(global_planner.historyFrame()!="map") throw std::runtime_error("Odom start reset global history");
    global_planner.cleanup();
    std::cout<<"PASS: new wall invalidates learned route; unrestricted planner finds a safe detour\n";
  } catch(const std::exception & e) {std::cerr<<e.what()<<'\n';rclcpp::shutdown();return 1;}
  rclcpp::shutdown();return 0;
}
