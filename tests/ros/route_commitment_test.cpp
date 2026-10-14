#include "behaviortree_cpp_v3/bt_factory.h"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include <chrono>
#include <iostream>
#include <stdexcept>
#include <thread>

class RunningController : public BT::ActionNodeBase {
public:
  using BT::ActionNodeBase::ActionNodeBase;
  static BT::PortsList providedPorts() {return {};}
  BT::NodeStatus tick() override {return BT::NodeStatus::RUNNING;}
  void halt() override {setStatus(BT::NodeStatus::IDLE);}
};

void require(bool value,const char * message) {
  if (!value) throw std::runtime_error(message);
}

nav_msgs::msg::Path route(std::initializer_list<std::pair<double,double>> points,
                         const std::string & frame="odom") {
  nav_msgs::msg::Path out;out.header.frame_id=frame;
  for(const auto & xy:points) {
    geometry_msgs::msg::PoseStamped p;p.header.frame_id=frame;
    p.pose.position.x=xy.first;p.pose.position.y=xy.second;p.pose.orientation.w=1;
    out.poses.push_back(p);
  }
  return out;
}

int main(int argc,char ** argv) {
  rclcpp::init(argc,argv);
  try {
    auto node=std::make_shared<rclcpp::Node>("route_commitment_regression",rclcpp::NodeOptions().parameter_overrides({
      rclcpp::Parameter("path_significantly_changed.preserve_shorter_route",true),
      rclcpp::Parameter("path_significantly_changed.force_update_period_s",.001)}));
    auto board=BT::Blackboard::create();board->set("node",node);
    BT::BehaviorTreeFactory factory;factory.registerFromPlugin(argv[1]);
    factory.registerNodeType<RunningController>("RunningController");
    auto tree=factory.createTreeFromText(R"(<root main_tree_to_execute="Test"><BehaviorTree ID="Test">
      <PathSignificantlyChanged path="{candidate}" goal="{mission_goal}" filtered_path="{accepted}"><RunningController/>
      </PathSignificantlyChanged></BehaviorTree></root>)",board);
    auto tick=[&](nav_msgs::msg::Path path) {
      board->set("candidate",path);std::this_thread::sleep_for(std::chrono::milliseconds(3));
      require(tree.tickRoot()==BT::NodeStatus::RUNNING,"Controller stopped during route handoff");
      return board->get<nav_msgs::msg::Path>("accepted");
    };
    // A return corner moves farther away on each replan. The incumbent suffix
    // remains shorter even though the original route was longer than the new one.
    const auto original=route({{0,0},{0,6},{-2,6},{-2,0}});
    tick(original);
    auto detour=route({{0,4},{0,8},{-2,8},{-2,0}});
    auto accepted=tick(detour);
    require(accepted.poses[1].pose.position.y==6,"Periodic refresh replaced a shorter remaining route");
    detour=route({{0,5},{0,9},{-2,9},{-2,0}});
    accepted=tick(detour);
    require(accepted.poses[1].pose.position.y==6,"Repeated replan moved the committed turn");
    // Comparable or improved routes, a new goal, and a pose away from the
    // incumbent are eligible for replacement without waiting for recovery.
    accepted=tick(route({{0,4},{0,5},{-2,5},{-2,0}}));
    require(accepted.poses[1].pose.position.y==5,"Shorter candidate was suppressed");
    accepted=tick(route({{0,4},{0,9},{-3,9},{-3,0}}));
    require(accepted.poses.back().pose.position.x==-3,"New goal retained the old mission route");
    accepted=tick(route({{1,4},{1,10},{-3,10},{-3,0}}));
    require(accepted.poses.front().pose.position.x==1,"Off-route pose could not accept a new route");
    accepted=tick(route({{1,4},{1,12},{-3,12},{-3,0}},"map"));
    require(accepted.header.frame_id=="map","Routes from different frames were compared");
    // A footprint rejection halts this decorator through FollowPathRecovery.
    // After halt, even a longer replacement must be admitted immediately.
    tree.haltTree();
    accepted=tick(route({{1,4},{1,15},{-3,15},{-3,0}},"map"));
    require(accepted.poses[1].pose.position.y==15,"Recovery did not release route commitment");
    node->set_parameter(rclcpp::Parameter("path_significantly_changed.preserve_shorter_route",false));
    accepted=tick(route({{1,4},{1,18},{-3,18},{-3,0}},"map"));
    require(accepted.poses[1].pose.position.y==18,"Opt-out changed the upstream replacement behavior");
    // Two arrivals inside a 0.35 m planner tolerance can be 0.60 m apart.
    // Compare the original request so this does not repeatedly release a turn.
    node->set_parameter(rclcpp::Parameter("path_significantly_changed.preserve_shorter_route",true));
    tree.haltTree();
    geometry_msgs::msg::PoseStamped goal;goal.header.frame_id="map";
    goal.pose.position.x=-2;goal.pose.orientation.w=1;
    board->set("mission_goal",goal);
    tick(route({{0,0},{0,6},{-2,6},{-2,-.3}}));
    accepted=tick(route({{0,4},{0,9},{-2,9},{-2,.3}}));
    require(accepted.poses[1].pose.position.y==6,"Arrival tolerance was mistaken for a changed goal");
    goal.pose.position.x=-2.1;board->set("mission_goal",goal);
    accepted=tick(route({{0,4},{0,10},{-2,10},{-2,.3}}));
    require(accepted.poses[1].pose.position.y==10,"Changed request with an identical endpoint was suppressed");
    std::cout<<"PASS: remaining route, moving turns, improvements, goal identity, arrival tolerance, deviation, frames, recovery, opt-out\n";
  } catch(const std::exception & e) {
    std::cerr<<"FAIL: "<<e.what()<<'\n';rclcpp::shutdown();return 1;
  }
  rclcpp::shutdown();return 0;
}
