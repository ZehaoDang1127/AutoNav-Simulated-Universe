#include "local_mirror_layer/local_mirror_layer.hpp"
#include "nav2_costmap_2d/layered_costmap.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/static_transform_broadcaster.h"
#include <chrono>
#include <iostream>
#include <stdexcept>
#include <thread>

void require(bool condition, const char * message) {
  if (!condition) { throw std::runtime_error(message); }
}

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  try {
    auto options = rclcpp::NodeOptions().parameter_overrides({
      rclcpp::Parameter("mirror.allow_decrease", true),
      rclcpp::Parameter("mirror.decrease_only_in_front", true),
      rclcpp::Parameter("mirror.decrease_nonzero_outside_view", true),
      rclcpp::Parameter("mirror.exclude_topics", std::vector<std::string>{"/mirror_line_mask"}),
      rclcpp::Parameter("mirror.exclude_threshold", 100),
      rclcpp::Parameter("obstacles.allow_decrease", true),
      rclcpp::Parameter("obstacles.decrease_only_in_front", true),
      rclcpp::Parameter("obstacles.decrease_nonzero_outside_view", false),
      rclcpp::Parameter("obstacles.min_occupied_value_to_mirror", 100),
      rclcpp::Parameter("obstacles.source_topic", "/mirror_input"),
      rclcpp::Parameter("observed.source_topic", "/mirror_input"),
      rclcpp::Parameter("observed.allow_decrease", true),
      rclcpp::Parameter("observed.decrease_only_in_front", true),
      rclcpp::Parameter("observed.mirror_free_space", true),
      rclcpp::Parameter("observed.unknown_space_cost", 200),
      rclcpp::Parameter("observed.clearing_frame", "regression_lidar"),
      rclcpp::Parameter("observed.min_occupied_value_to_mirror", 100),
      rclcpp::Parameter("observed.exclude_topics", std::vector<std::string>{"/mirror_line_mask"}),
      rclcpp::Parameter("observed.exclude_threshold", 100),
      rclcpp::Parameter("controller_memory.source_topic", "/mirror_input"),
      rclcpp::Parameter("controller_memory.retain_source_history", false),
      rclcpp::Parameter("controller_memory.allow_decrease", true),
      rclcpp::Parameter("controller_memory.decrease_only_in_front", false),
      rclcpp::Parameter("controller_memory.min_occupied_value_to_mirror", 100),
      rclcpp::Parameter("mirror.source_topic", "/mirror_input")});
    auto node = std::make_shared<nav2_util::LifecycleNode>("mirror_memory_regression", "", options);
    auto pub_node = std::make_shared<rclcpp::Node>("mirror_memory_input");
    tf2_ros::StaticTransformBroadcaster broadcaster(pub_node);
    geometry_msgs::msg::TransformStamped sensor_tf;
    sensor_tf.header.stamp = pub_node->now();
    sensor_tf.header.frame_id = "map";
    sensor_tf.child_frame_id = "regression_lidar";
    sensor_tf.transform.translation.x = .6598;
    // The actual lidar mount rolls by pi; its forward yaw still stays zero.
    sensor_tf.transform.rotation.x = 1.0;
    broadcaster.sendTransform(sensor_tf);
    auto publisher = pub_node->create_publisher<nav_msgs::msg::OccupancyGrid>("/mirror_input", 1);
    auto mask_publisher = pub_node->create_publisher<nav_msgs::msg::OccupancyGrid>("/mirror_line_mask", 1);
    tf2_ros::Buffer buffer(node->get_clock());
    nav2_costmap_2d::LayeredCostmap layered("map", false, true);
    layered.resizeMap(80, 80, 0.05, -2.0, -2.0);
    auto mirror = std::make_shared<local_mirror_layer::LocalMirrorLayer>();
    layered.addPlugin(mirror);
    mirror->initialize(&layered, "mirror", &buffer, node,
      node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive));
    nav2_costmap_2d::LayeredCostmap obstacle_layered("map", false, true);
    obstacle_layered.resizeMap(80, 80, 0.05, -2.0, -2.0);
    auto obstacles = std::make_shared<local_mirror_layer::LocalMirrorLayer>();
    obstacle_layered.addPlugin(obstacles);
    obstacles->initialize(&obstacle_layered, "obstacles", &buffer, node,
      node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive));
    nav2_costmap_2d::LayeredCostmap observed_layered("map", false, true);
    observed_layered.resizeMap(80, 80, 0.05, -2.0, -2.0);
    auto observed = std::make_shared<local_mirror_layer::LocalMirrorLayer>();
    observed_layered.addPlugin(observed);
    observed->initialize(&observed_layered, "observed", &buffer, node,
      node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive));
    nav2_costmap_2d::LayeredCostmap controller_layered("map", true, true);
    controller_layered.resizeMap(80, 80, 0.05, -2.0, -2.0);
    auto controller_memory = std::make_shared<local_mirror_layer::LocalMirrorLayer>();
    controller_layered.addPlugin(controller_memory);
    controller_memory->initialize(&controller_layered, "controller_memory", &buffer, node,
      node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive));
    nav_msgs::msg::OccupancyGrid map;
    map.header.frame_id = "map";
    map.info.resolution = 0.05;
    map.info.width = map.info.height = 80;
    map.info.origin.position.x = map.info.origin.position.y = -2.0;
    map.info.origin.orientation.w = 1.0;
    map.data.resize(80 * 80, -1);
    const int rear = 40 * 80 + 20;  // (-0.975, 0.025): outside the front clearing view.
    const int front = 40 * 80 + 60;
    auto update = [&]() {
      map.header.stamp = pub_node->now();
      publisher->publish(map);
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
      rclcpp::spin_some(node->get_node_base_interface());
      double min_x = 0, min_y = 0, max_x = 0, max_y = 0;
      mirror->updateBounds(0, 0, 0, &min_x, &min_y, &max_x, &max_y);
      mirror->updateCosts(*layered.getCostmap(), 0, 0, 80, 80);
      obstacles->updateBounds(0, 0, 0, &min_x, &min_y, &max_x, &max_y);
      obstacles->updateCosts(*obstacle_layered.getCostmap(), 0, 0, 80, 80);
      observed_layered.getCostmap()->resetMap(0, 0, 80, 80);
      observed->updateBounds(0, 0, 0, &min_x, &min_y, &max_x, &max_y);
      observed->updateCosts(*observed_layered.getCostmap(), 0, 0, 80, 80);
      controller_layered.getCostmap()->resetMap(0, 0, 80, 80);
      controller_memory->updateBounds(0, 0, 0, &min_x, &min_y, &max_x, &max_y);
      controller_memory->updateCosts(*controller_layered.getCostmap(), 0, 0, 80, 80);
    };
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    map.data[rear] = map.data[front] = 100;
    const int blind_side = 51 * 80 + 47;  // In body's cone, behind the lidar's cone.
    map.data[blind_side] = 100;
    map.data[42 * 80 + 55] = map.data[42 * 80 + 25] = 0;
    update();
    require(observed_layered.getCostmap()->getCost(55, 42) == 0,
      "View-gated FREE did not fill UNKNOWN master space");
    require(observed->getCost(25, 42) == 255,
      "Rear rolling-window FREE was treated as observed space");
    require(observed_layered.getCostmap()->getCost(25, 42) == 200,
      "Unobserved space did not retain its finite exploration penalty");
    require(obstacles->getCost(55, 42) == 255,
      "FREE mirroring was enabled without its opt-in parameter");
    require(obstacle_layered.getCostmap()->getCost(55, 42) == 255,
      "Unknown-space planning cost was enabled without its opt-in parameter");
    require(observed_layered.getCostmap()->getCost(60, 40) == 254,
      "Exploration penalty replaced a lethal obstacle");
    require(mirror->getCost(20, 40) == 254, "Lethal observation not retained");
    require(obstacles->getCost(20, 40) == 254, "Obstacle lethal observation not retained");
    require(controller_layered.getCostmap()->getCost(20, 40) == 254,
      "Controller did not receive a remembered hazard behind the sensor view");
    map.data[rear] = 96;
    update();
    require(mirror->getCost(20, 40) < 253 && mirror->getCost(20, 40) > 0,
      "Positive lower halo did not correct stale lethal memory");
    require(layered.getCostmap()->getCost(20, 40) == 254,
      "A soft mirror halo erased another layer's lethal obstacle");
    require(obstacles->getCost(20, 40) == 254,
      "A neighbor's halo erased an unobserved obstacle outside the sensor view");
    require(controller_memory->getCost(20, 40) == 255,
      "Authoritative source correction did not retire this controller mirror's old mark");
    require(obstacles->getCost(60, 40) == 254,
      "Updating a stale mark erased a different live obstacle");
    map.data[front] = 96;
    update();
    require(obstacles->getCost(60, 40) == 255,
      "A lower observation in the sensor view did not retire stale obstacle memory");
    map.data[front] = 100;
    map.data[rear] = 100;
    update();
    map.data[rear] = map.data[front] = 0;
    map.data[blind_side] = 0;
    update();
    require(observed->getCost(47, 51) == 254,
      "Body-centered clearing erased a cone outside the offset lidar view");
    require(mirror->getCost(20, 40) == 254, "Unobserved rear FREE erased obstacle memory");
    require(mirror->getCost(60, 40) == 255, "Observed front FREE did not clear memory");
    require(obstacles->getCost(20, 40) == 254,
      "Unobserved rear FREE erased obstacle memory");
    require(observed_layered.getCostmap()->getCost(60, 40) == 0,
      "Observed FREE did not clear this mirror's previous obstacle");
    observed_layered.getCostmap()->setCost(60, 40, 254);
    observed->updateCosts(*observed_layered.getCostmap(), 0, 0, 80, 80);
    require(observed_layered.getCostmap()->getCost(60, 40) == 254,
      "FREE opinion erased another layer's lethal obstacle");
    map.data[front] = map.data[42 * 80 + 55] = -1;
    map.data[rear] = -1;
    update();
    require(observed_layered.getCostmap()->getCost(60, 40) == 0,
      "UNKNOWN source discarded previously observed FREE space");
    require(mirror->getCost(20, 40) == 254, "Unknown source erased memory");
    require(obstacles->getCost(20, 40) == 254, "Unknown source erased obstacle memory");
    auto mask = map;
    mask.data[rear] = 50;
    mask_publisher->publish(mask);
    map.data[rear] = 100;
    update();
    require(mirror->getCost(20, 40) == 254, "Soft tape halo masked a real obstacle");
    mask.data[rear] = 100;
    mask_publisher->publish(mask);
    update();
    require(mirror->getCost(20, 40) == 255, "Exact tape cell was not excluded from obstacle mirror");
    mask.data[front] = 100;
    mask_publisher->publish(mask);
    map.data[front] = 0;
    update();
    require(observed->getCost(60, 40) == 255,
      "FREE mirroring ignored an exact tape exclusion mask");
    // A rolling master moves before its layers update. Its mirror memory
    // must stay at the source world position rather than follow array indices.
    controller_layered.getCostmap()->updateOrigin(-1.5,-2.5);
    controller_layered.getCostmap()->resetMap(0,0,80,80);
    double rolling_min_x=0,rolling_min_y=0,rolling_max_x=0,rolling_max_y=0;
    controller_memory->updateBounds(.5,-.5,0,&rolling_min_x,&rolling_min_y,&rolling_max_x,&rolling_max_y);
    controller_memory->updateCosts(*controller_layered.getCostmap(),0,0,80,80);
    require(controller_layered.getCostmap()->getCost(10,50)==254,
      "Rolling mirror moved a remembered hazard away from its world position");
    require(controller_layered.getCostmap()->getCost(20,40)!=254,
      "Rolling mirror painted a ghost obstacle at the old array index");
    controller_layered.getCostmap()->updateOrigin(-2.,-2.);
    controller_layered.getCostmap()->resetMap(0,0,80,80);
    controller_memory->updateBounds(0,0,0,&rolling_min_x,&rolling_min_y,&rolling_max_x,&rolling_max_y);
    controller_memory->updateCosts(*controller_layered.getCostmap(),0,0,80,80);
    require(controller_layered.getCostmap()->getCost(20,40)==254,
      "Reverse rolling shift discarded remembered hazard");
    // A new full-source projection need not stamp every old destination cell.
    // Coarser centers reproduce those holes without depending on TF timing.
    map.info.resolution=.1;
    map.info.width=map.info.height=40;
    map.data.assign(40*40,-1);
    map.data[20*40+12]=100;
    update();
    require(controller_memory->getCost(20,40)==255,
      "Full-source snapshot retained an obsolete mark in a projection hole");
    require(controller_memory->getCost(25,41)==254,
      "Full-source snapshot discarded the newly projected real hazard");
    require(obstacles->getCost(20,40)==254,
      "Snapshot opt-in changed the default persistent observation memory");
    controller_layered.getCostmap()->setCost(60,50,254);
    controller_memory->updateCosts(*controller_layered.getCostmap(),0,0,80,80);
    require(controller_layered.getCostmap()->getCost(60,50)==254,
      "Snapshot cache cleared a different master's hazard");
    observed_layered.resizeMap(100,100,.05,-2.5,-2.5);
    require(observed->getCost(65,52)==0,
      "Resize discarded positively observed FREE memory");
    std::cout << "PASS: halo updates, view-gated FREE memory, safe merging, tape exclusion, rolling world alignment\n";
  } catch (const std::exception & error) {
    std::cerr << "FAIL: " << error.what() << '\n';
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
