#include "behaviortree_cpp_v3/bt_factory.h"
#include "nav2_msgs/msg/costmap.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h"
#include <chrono>
#include <iostream>
#include <stdexcept>
#include <thread>

int controller_ticks = 0;
int controller_halts = 0;
class FakeFollowPath : public BT::ActionNodeBase {
public:
  using BT::ActionNodeBase::ActionNodeBase;
  static BT::PortsList providedPorts() { return {}; }
  BT::NodeStatus tick() override { ++controller_ticks; return BT::NodeStatus::RUNNING; }
  void halt() override { ++controller_halts; setStatus(BT::NodeStatus::IDLE); }
};

void require(bool condition, const char * message) {
  if (!condition) { throw std::runtime_error(message); }
}

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  try {
    require(argc > 1, "Pass the built PathFootprintSafe plugin library");
    auto node = std::make_shared<rclcpp::Node>("path_handoff_regression");
    auto buffer = std::make_shared<tf2_ros::Buffer>(node->get_clock());
    auto blackboard = BT::Blackboard::create();
    blackboard->set("node", node);
    blackboard->set("tf_buffer", buffer);
    geometry_msgs::msg::TransformStamped robot;
    robot.header.frame_id = "map";
    robot.child_frame_id = "nav_center";
    robot.transform.rotation.w = 1.0;
    buffer->setTransform(robot, "test", true);
    BT::BehaviorTreeFactory factory;
    factory.registerFromPlugin(argv[1]);
    factory.registerNodeType<FakeFollowPath>("FakeFollowPath");
    auto candidate = factory.createTreeFromText(R"(
      <root main_tree_to_execute="Test"><BehaviorTree ID="Test">
        <PathFootprintSafe path="{candidate}" validated_path="{accepted}"
          costmap_topic="/test_costmap" max_costmap_age_ms="10000"/>
      </BehaviorTree></root>)", blackboard);
    auto controller = factory.createTreeFromText(R"(
      <root main_tree_to_execute="Test"><BehaviorTree ID="Test">
        <ReactiveSequence>
          <PathFootprintSafe path="{accepted}" validate_remaining="true"
            costmap_topic="/test_costmap" max_costmap_age_ms="10000"/>
          <FakeFollowPath/>
        </ReactiveSequence>
      </BehaviorTree></root>)", blackboard);
    auto publisher = node->create_publisher<nav2_msgs::msg::Costmap>(
      "/test_costmap", rclcpp::QoS(1).transient_local().reliable());
    nav2_msgs::msg::Costmap map;
    map.header.frame_id = "map";
    map.metadata.resolution = 0.05;
    map.metadata.size_x = map.metadata.size_y = 160;
    map.metadata.origin.position.x = map.metadata.origin.position.y = -3.0;
    map.metadata.origin.orientation.w = 1.0;
    map.data.resize(160 * 160, 0);
    nav_msgs::msg::Path path;
    path.header.frame_id = "map";
    for (int i = 0; i <= 40; ++i) {
      geometry_msgs::msg::PoseStamped pose;
      pose.header.frame_id = "map";
      pose.pose.position.x = i * 0.05;
      pose.pose.orientation.w = 1.0;
      path.poses.push_back(pose);
    }
    blackboard->set("candidate", path);
    auto publish = [&]() {
      map.header.stamp = node->now();
      publisher->publish(map);
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    };
    auto mark = [&](double x, double y) {
      int mx = static_cast<int>((x + 3.0) / map.metadata.resolution);
      int my = static_cast<int>((y + 3.0) / map.metadata.resolution);
      map.data[my * 160 + mx] = 254;
    };
    // Initialize subscription discovery and assert a successful handoff.
    candidate.tickRoot();
    publish();
    require(candidate.tickRoot() == BT::NodeStatus::SUCCESS, "Safe candidate rejected");
    require(blackboard->get<nav_msgs::msg::Path>("accepted").poses.size() == 41,
      "Validated candidate not committed");
    controller.tickRoot();
    publish();
    require(controller.tickRoot() == BT::NodeStatus::RUNNING, "Safe controller route rejected");
    const int started_ticks = controller_ticks;
    // A rejected new candidate must not replace the accepted controller route.
    path.poses.back().pose.position.y = 1.0;
    blackboard->set("candidate", path);
    mark(1.5, 0.0);
    publish();
    require(candidate.tickRoot() == BT::NodeStatus::FAILURE, "Lethal candidate accepted");
    require(blackboard->get<nav_msgs::msg::Path>("accepted").poses.back().pose.position.y == 0.0,
      "Rejected candidate leaked to accepted path");
    require(controller.tickRoot() == BT::NodeStatus::FAILURE, "Unsafe active route continued");
    require(controller_ticks == started_ticks && controller_halts == 1,
      "Unsafe route did not halt FollowPath before its next tick");
    // Traveled path obstacles must not prevent following the remaining route.
    std::fill(map.data.begin(), map.data.end(), 0);
    mark(0.0, 0.0);
    robot.transform.translation.x = 1.4;
    buffer->setTransform(robot, "test", true);
    publish();
    require(controller.tickRoot() == BT::NodeStatus::RUNNING, "Traveled path was not pruned");
    // Check measured robot pose even when it has deviated from the route.
    std::fill(map.data.begin(), map.data.end(), 0);
    robot.transform.translation.y = 0.6;
    buffer->setTransform(robot, "test", true);
    mark(1.4, 1.0);
    publish();
    require(controller.tickRoot() == BT::NodeStatus::FAILURE, "Unsafe measured footprint missed");
    require(controller_halts == 2, "Measured footprint failure did not halt controller");
    // A rolling local costmap must validate the near route rather than reject
    // the distant goal because it lies outside the rolling window.
    robot.transform.translation.y = 0.0;
    buffer->setTransform(robot, "test", true);
    std::fill(map.data.begin(), map.data.end(), 0);
    path = blackboard->get<nav_msgs::msg::Path>("accepted");
    auto distant = path.poses.back();
    distant.pose.position.x = 20.0;
    path.poses.push_back(distant);
    blackboard->set("accepted", path);
    auto local_gate = factory.createTreeFromText(R"(
      <root main_tree_to_execute="Test"><BehaviorTree ID="Test">
        <PathFootprintSafe path="{accepted}" validate_remaining="true"
          validation_distance_m="1.5" costmap_topic="/test_costmap"
          max_costmap_age_ms="10000"/>
      </BehaviorTree></root>)", blackboard);
    local_gate.tickRoot();
    publish();
    require(local_gate.tickRoot() == BT::NodeStatus::SUCCESS, "Local lookahead checked distant poses");
    // A newly observed distant hazard must leave safe motion running, then
    // halt the controller before a tick once that hazard enters its horizon.
    path.poses.pop_back();
    for (int i = 41; i <= 400; ++i) {
      auto next = distant;
      next.pose.position.x = i * 0.05;
      path.poses.push_back(next);
    }
    blackboard->set("accepted", path);
    auto bounded_controller = factory.createTreeFromText(R"(
      <root main_tree_to_execute="Test"><BehaviorTree ID="Test">
        <ReactiveSequence>
          <PathFootprintSafe path="{accepted}" validate_remaining="true"
            validation_distance_m="2.0" costmap_topic="/test_costmap"
            max_costmap_age_ms="10000"/>
          <FakeFollowPath/>
        </ReactiveSequence>
      </BehaviorTree></root>)", blackboard);
    mark(4.5, 0.0);
    bounded_controller.tickRoot();
    publish();
    require(bounded_controller.tickRoot() == BT::NodeStatus::RUNNING,
      "Distant hazard cancelled safe near-route motion");
    const int bounded_ticks = controller_ticks;
    const int bounded_halts = controller_halts;
    robot.transform.translation.x = 2.5;
    buffer->setTransform(robot, "test", true);
    publish();
    require(bounded_controller.tickRoot() == BT::NodeStatus::FAILURE,
      "Approaching hazard escaped bounded validation");
    require(controller_ticks == bounded_ticks && controller_halts == bounded_halts + 1,
      "Approaching hazard did not halt FollowPath before its next tick");
    std::fill(map.data.begin(), map.data.end(), 0);
    // Driving about the axle must retain the body overhang and its margin.
    auto drive_body_gate = factory.createTreeFromText(R"(
      <root main_tree_to_execute="Test"><BehaviorTree ID="Test">
        <PathFootprintSafe path="{drive_pose}" robot_base_frame="base_link"
          footprint="[[0.77, 0.41], [0.77, -0.41], [-0.32, -0.41], [-0.32, 0.41]]"
          footprint_padding="0.15" costmap_topic="/test_costmap"
          max_costmap_age_ms="10000"/>
      </BehaviorTree></root>)", blackboard);
    nav_msgs::msg::Path drive_pose;
    drive_pose.header.frame_id = "map";
    drive_pose.poses.push_back(path.poses.front());
    blackboard->set("drive_pose", drive_pose);
    drive_body_gate.tickRoot();
    publish();
    require(drive_body_gate.tickRoot() == BT::NodeStatus::SUCCESS, "Clear axle-centered body rejected");
    mark(.9, 0.0);
    publish();
    require(drive_body_gate.tickRoot() == BT::NodeStatus::FAILURE,
      "Drive-axle planning lost the padded forward body overhang");
    std::cout << "PASS: handoff, cancellation, pruning, measured body, bounded hazard approach, axle overhang\n";
  } catch (const std::exception & error) {
    std::cerr << "FAIL: " << error.what() << '\n';
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
