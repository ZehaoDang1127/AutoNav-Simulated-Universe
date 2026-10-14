#include "nav2_costmap_2d/obstacle_layer.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include <cmath>
#include <iostream>
#include <stdexcept>

void require(bool ok,const char * message) {
  if (!ok) throw std::runtime_error(message);
}

void checkFootprintClearing(bool clear) {
  auto options=rclcpp::NodeOptions().parameter_overrides({
    rclcpp::Parameter("track_unknown_space",true),
    rclcpp::Parameter("obstacles.footprint_clearing_enabled",clear)});
  auto node=std::make_shared<nav2_util::LifecycleNode>(
    clear ? "body_clear_fixture" : "body_preserve_fixture","",options);
  node->declare_parameter("track_unknown_space",true);
  node->declare_parameter("transform_tolerance",.1);
  tf2_ros::Buffer tf(node->get_clock());
  nav2_costmap_2d::LayeredCostmap map("map",false,true);
  map.resizeMap(100,100,.05,-2.5,-2.5);
  std::vector<geometry_msgs::msg::Point> body;
  for (auto xy : {std::pair<double,double>{.92,.56},{.92,-.56},{-.47,-.56},{-.47,.56}}) {
    geometry_msgs::msg::Point point;point.x=xy.first;point.y=xy.second;
    body.push_back(point);
  }
  map.setFootprint(body);
  auto layer=std::make_shared<nav2_costmap_2d::ObstacleLayer>();
  map.addPlugin(layer);
  layer->initialize(&map,"obstacles",&tf,node,
    node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive));
  unsigned int mx,my;
  require(layer->worldToMap(.8,.5,mx,my),"body fixture outside map");
  layer->setCost(mx,my,254); // A real mark inside the extra safety padding.
  for (int repeat=0;repeat<2;++repeat) {
    double min_x=0,min_y=0,max_x=0,max_y=0;
    layer->updateBounds(0,0,0,&min_x,&min_y,&max_x,&max_y);
    layer->updateCosts(*map.getCostmap(),0,0,100,100);
    require(map.getCostmap()->getCost(mx,my)==(clear ? 0 : 254),
      "Footprint clearing reproduction or padded-body obstacle preservation failed");
  }
}

int main(int argc,char ** argv) {
  rclcpp::init(argc,argv);
  try {
    checkFootprintClearing(true);
    checkFootprintClearing(false);
    auto options=rclcpp::NodeOptions().parameter_overrides({
      rclcpp::Parameter("track_unknown_space",true),
      rclcpp::Parameter("far.footprint_clearing_enabled",false)});
    auto node=std::make_shared<nav2_util::LifecycleNode>("far_free_regression","",options);
    node->declare_parameter("track_unknown_space",true);
    node->declare_parameter("transform_tolerance",.1);
    tf2_ros::Buffer tf(node->get_clock());
    nav2_costmap_2d::LayeredCostmap global("map",false,true);
    global.resizeMap(400,400,.05,-10.,-10.);
    global.getCostmap()->setDefaultValue(nav2_costmap_2d::NO_INFORMATION);
    global.getCostmap()->resetMap(0,0,400,400);
    auto layer=std::make_shared<nav2_costmap_2d::ObstacleLayer>();
    global.addPlugin(layer);
    layer->initialize(&global,"far",&tf,node,
      node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive));
    geometry_msgs::msg::Point origin;
    origin.x=.6598;
    sensor_msgs::msg::PointCloud2 cloud;
    sensor_msgs::PointCloud2Modifier modifier(cloud);
    modifier.setPointCloud2FieldsByString(1,"xyz");
    modifier.resize(2);
    sensor_msgs::PointCloud2Iterator<float> x(cloud,"x"),y(cloud,"y"),z(cloud,"z");
    *x=8.6598; *y=0.; *z=.2; ++x; ++y; ++z;
    *x=2.6598; *y=2.; *z=.2; // A nearer obstacle terminates this ray.
    nav2_costmap_2d::Observation observation(origin,cloud,8.,.3,6.,0.);
    layer->addStaticObservation(observation,true,true);
    auto cost=[&](double wx,double wy) {
      unsigned int mx,my;
      require(global.getCostmap()->worldToMap(wx,wy,mx,my),"test point outside map");
      return global.getCostmap()->getCost(mx,my);
    };
    auto update=[&]() {
      auto * master=global.getCostmap();
      master->resetMap(0,0,400,400);
      unsigned int mx,my;
      master->worldToMap(1.5,0.,mx,my);
      master->setCost(mx,my,254); // Another layer's live obstacle.
      double min_x=0,min_y=0,max_x=0,max_y=0;
      layer->updateBounds(0,0,0,&min_x,&min_y,&max_x,&max_y);
      layer->updateCosts(*master,0,0,400,400);
    };
    update();
    require(cost(4.5,0.)==0,"observed free ray beyond the rolling map stayed unknown");
    require(cost(8.,0.)==255,"ray tracing cleared beyond the six metre sensor horizon");
    require(cost(-4.5,0.)==255,"unseen space behind the sensor was cleared");
    require(cost(3.6598,3.)==255,"free ray continued through an occluding obstacle");
    require(cost(2.6598,2.)==254,"nearer obstacle return was not marked");
    require(cost(1.5,0.)==254,"free ray lowered another layer's obstacle");
    layer->clearStaticObservations(true,true);
    update();
    require(cost(4.5,0.)==0,"fixed-map free observations disappeared after leaving view");
    std::cout<<"Far free rays, occlusion, unknown bounds, persistence, safe merging, and body-mark preservation passed\n";
  } catch(const std::exception & ex) {
    std::cerr<<ex.what()<<'\n'; rclcpp::shutdown(); return 1;
  }
  rclcpp::shutdown();
  return 0;
}
