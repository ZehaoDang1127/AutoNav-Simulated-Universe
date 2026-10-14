#include "line_layer/line_layer.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "tf2_ros/static_transform_broadcaster.h"
#include <chrono>
#include <iostream>
#include <stdexcept>
#include <thread>

void require(bool condition, const char * message) {
  if (!condition) throw std::runtime_error(message);
}

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  try {
    auto options = rclcpp::NodeOptions().parameter_overrides({
      rclcpp::Parameter("lines.line_topic", "/global_line_regression/input"),
      rclcpp::Parameter("lines.clearing", true),
      rclcpp::Parameter("lines.rolling_window", false),
      rclcpp::Parameter("lines.publish_costmap", false),
      rclcpp::Parameter("lines.preserve_unknown_space", true),
      rclcpp::Parameter("lines.clearing_frame", "regression_line_sensor"),
      rclcpp::Parameter("lines.observation_persistence_ms", 100),
      rclcpp::Parameter("lines.clear_lines_only_in_view", true),
      rclcpp::Parameter("lines.line_clear_range_max_m", 6.),
      rclcpp::Parameter("lines.line_clear_range_min_m", .2),
      rclcpp::Parameter("lines.line_clear_angle_min_rad", -.95),
      rclcpp::Parameter("lines.line_clear_angle_max_rad", .95)});
    auto node = std::make_shared<nav2_util::LifecycleNode>("global_line_regression", "", options);
    tf2_ros::Buffer tf(node->get_clock());
    geometry_msgs::msg::TransformStamped sensor;
    sensor.header.frame_id = "map";
    sensor.child_frame_id = "regression_line_sensor";
    sensor.transform.translation.x = .6598;
    sensor.transform.rotation.w = 1.;
    tf.setTransform(sensor,"test",true);
    nav2_costmap_2d::LayeredCostmap global("map", false, true);
    global.resizeMap(320,320,.05,-8.,-8.);
    auto lines = std::make_shared<line_layer::LineLayer>();
    global.addPlugin(lines);
    lines->initialize(&global, "lines", &tf, node,
      node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive));
    auto pub_node = std::make_shared<rclcpp::Node>("global_line_regression_input");
    tf2_ros::StaticTransformBroadcaster broadcaster(pub_node);
    broadcaster.sendTransform(sensor);
    auto publisher = pub_node->create_publisher<autonav_interfaces::msg::LinePoints>(
      "/global_line_regression/input", rclcpp::QoS(1).reliable());
    nav2_costmap_2d::Costmap2D local(120,120,.05,-3.,-3.);
    autonav_interfaces::msg::LinePoints message;
    message.header.frame_id = "map";
    for (double x : {4.5,-4.5}) {
      geometry_msgs::msg::Vector3 point;
      point.x = x;
      message.points.push_back(point);
    }
    geometry_msgs::msg::Vector3 blind;
    blind.x = .425; blind.y = .225;
    message.points.push_back(blind);
    auto update = [&]() {
      rclcpp::spin_some(node->get_node_base_interface());
      double min_x=0,min_y=0,max_x=0,max_y=0;
      global.getCostmap()->resetMap(0,0,320,320);
      global.getCostmap()->setCost(170,170,254); // An independent obstacle.
      lines->updateBounds(0,0,0,&min_x,&min_y,&max_x,&max_y);
      lines->updateCosts(*global.getCostmap(),0,0,320,320);
    };
    auto send = [&]() {
      message.header.stamp = node->now();
      publisher->publish(message);
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
      update();
    };
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    send();
    unsigned int mx,my;
    require(!local.worldToMap(4.5,0,mx,my), "Far tape unexpectedly fits rolling window");
    require(global.getCostmap()->getCost(250,160) == 254, "Far front observation clipped");
    require(global.getCostmap()->getCost(70,160) == 254, "Far rear observation clipped");
    require(global.getCostmap()->getCost(160,260) == 255, "Line layer predeclared unseen canvas free");
    message.points.clear();
    std::this_thread::sleep_for(std::chrono::milliseconds(250));
    send();
    require(global.getCostmap()->getCost(250,160) == 255, "Visible stale far mark retained");
    require(global.getCostmap()->getCost(70,160) == 254, "Unseen far boundary erased");
    require(global.getCostmap()->getCost(168,164) == 254, "Blind area authorized tape clearing");
    require(global.getCostmap()->getCost(170,170) == 254, "Line clearing erased another obstacle");
    std::cout << "PASS: far line observations, visible clearing, unseen retention, safe merge\n";
  } catch (const std::exception & error) {
    std::cerr << "FAIL: " << error.what() << '\n';
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
