#include "nav2_mppi_controller/calibrated_diff_drive.hpp"
#include "nav2_mppi_controller/optimizer.hpp"
#include <iostream>
#include <stdexcept>

void require(bool condition,const char * message)
{
  if (!condition) {throw std::runtime_error(message);}
}

class TestMap : public nav2_costmap_2d::Costmap2DROS
{
public:
  TestMap() : Costmap2DROS("optimal_control_costmap")
  {
    layered_costmap_=std::make_unique<nav2_costmap_2d::LayeredCostmap>("map",false,true);
    layered_costmap_->resizeMap(200,200,.05,-3,-3);
    getCostmap()->setDefaultValue(200);
    getCostmap()->resetMap(0,0,200,200);
    std::vector<geometry_msgs::msg::Point> body;
    for (auto xy : {std::pair<double,double>{.92,.56},{.92,-.56},{-.47,-.56},{-.47,.56}}) {
      geometry_msgs::msg::Point p;p.x=xy.first;p.y=xy.second;body.push_back(p);
    }
    footprint_padding_=0;
    setRobotFootprint(body);
  }
};

class TestOptimizer : public mppi::Optimizer
{
public:
  using mppi::Optimizer::validateControlSequence;
  using mppi::Optimizer::validateEmergencyStop;
  using mppi::Optimizer::selectCollisionFreeControl;
  using mppi::Optimizer::getControlFromSequenceAsTwist;
  void setup(std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node,
    std::shared_ptr<TestMap> map,mppi::ParametersHandler * handler)
  {
    parent_=node;costmap_ros_=map;parameters_handler_=handler;
    settings_.model_dt=.05f;settings_.time_steps=56;
    settings_.constraints.vx_max=.5f;settings_.constraints.vx_min=-.25f;
    settings_.constraints.wz=1.0f;
    settings_.shift_control_sequence=true;
    motion_model_=std::make_shared<mppi::CalibratedDiffDriveMotionModel>(settings_);
    critic_manager_.on_configure(node,"FollowPath",map,handler);
    state_.reset(2,56);
    state_.pose.pose.orientation.w=1;
    state_.speed.linear.x=.5;
    state_.cvx.fill(.5);
    xt::view(state_.cwz,0,xt::all())=1.0f;
    xt::view(state_.cwz,1,xt::all())=-1.0f;
    path_.reset(2);path_.x(1)=5;
    costs_=xt::xtensor<float,1>{1,2};
    control_sequence_.reset(56);control_sequence_.vx.fill(.5);
    for (auto & control : control_history_) {control={.5f,0.f,0.f};}
  }
  void turn(float yaw_rate) {control_sequence_.wz.fill(yaw_rate);}
  void brakingFixture() {
    state_.speed.angular.z=.3;
    state_.cwz.fill(1);
    control_sequence_.vx.fill(.5);
    control_sequence_.wz.fill(1);
  }
  bool plannedTurnPreserved() const {
    return xt::all(xt::equal(control_sequence_.vx,.5f)) &&
           xt::all(xt::equal(control_sequence_.wz,1.0f));
  }
  bool issuedHistoryUnchanged() const {
    for (const auto & control : control_history_) {
      if (control.vx!=.5f || control.vy!=0 || control.wz!=0) {return false;}
    }
    return true;
  }
};

int main(int argc,char ** argv)
{
  rclcpp::init(argc,argv);
  try {
    auto node=std::make_shared<rclcpp_lifecycle::LifecycleNode>("optimal_control_regression",
      rclcpp::NodeOptions().parameter_overrides({
        rclcpp::Parameter("FollowPath.critics",std::vector<std::string>{"FilledCostCritic"}),
        rclcpp::Parameter("FollowPath.FilledCostCritic.consider_footprint",true),
        rclcpp::Parameter("FollowPath.FilledCostCritic.rollout_footprint_margin",.1)}));
    auto map=std::make_shared<TestMap>();
    map->getCostmap()->setCost(100,60,254); // (2.025,0.025), outside initial body.
    mppi::ParametersHandler handler(node);
    TestOptimizer optimizer;
    optimizer.setup(node,map,&handler);
    optimizer.turn(1);
    require(optimizer.validateControlSequence(),"Left sample was not safe in the fixture");
    optimizer.turn(-1);
    require(optimizer.validateControlSequence(),"Right sample was not safe in the fixture");
    optimizer.turn(0);
    require(!optimizer.validateControlSequence(),"Unsafe mean of safe turns was accepted");
    require(optimizer.selectCollisionFreeControl(),"Safe sampled fallback was not selected");
    builtin_interfaces::msg::Time stamp;
    auto command=optimizer.getControlFromSequenceAsTwist(stamp).twist;
    require(command.angular.z>0 && command.angular.z<1 && command.linear.x<=.5,
      "Best validated sample bypassed the temporal filter or physical bounds");
    require(optimizer.issuedHistoryUnchanged(),
      "Testing a filtered candidate committed a command that was never issued");
    require(optimizer.validateControlSequence(),"Selected fallback was not validated");
    require(optimizer.selectCollisionFreeControl(),"Already-safe chosen control was rejected");
    map->getCostmap()->setCost(60,60,254);
    require(!optimizer.selectCollisionFreeControl(),"Blocked current body still produced a safe-control claim");
    map->getCostmap()->resetMap(0,0,200,200);
    map->getCostmap()->setCost(86,56,254); // (1.325,-0.175), outside the initial body.
    optimizer.brakingFixture();
    require(optimizer.validateControlSequence(),"Planned continuing turn was not safe in braking fixture");
    require(!optimizer.validateEmergencyStop(),"Unsafe smoothed braking path was accepted");
    require(optimizer.plannedTurnPreserved(),"Braking validation replaced the requested sequence");
    require(optimizer.selectCollisionFreeControl(),"No braking-safe filtered alternative was selected");
    require(optimizer.validateControlSequence() && optimizer.validateEmergencyStop(),
      "Selected alternative lacks continuing or braking clearance");
    command=optimizer.getControlFromSequenceAsTwist(stamp).twist;
    require(std::abs(command.angular.z-1)>1e-5,
      "Unsafe continuing turn bypassed the braking check");
    std::cout << "PASS: filtered fallback, history preservation, blocked rejection, braking clearance\n";
  } catch (const std::exception & error) {
    std::cerr << "FAIL: " << error.what() << '\n';
    rclcpp::shutdown();return 1;
  }
  rclcpp::shutdown();return 0;
}
