#include "nav2_mppi_controller/calibrated_diff_drive.hpp"
#include "nav2_mppi_controller/optimizer.hpp"
#include "nav2_smac_planner/collision_checker.hpp"
#include <iostream>
#include <stdexcept>

void require(bool condition, const char * message)
{
  if (!condition) {throw std::runtime_error(message);}
}

class CommandFixture : public mppi::Optimizer
{
public:
  using mppi::Optimizer::setMotionModel;
  using mppi::Optimizer::getControlFromSequenceAsTwist;
  void prepare()
  {
    settings_.model_dt=.05f;
    settings_.shift_control_sequence=true;
    costmap_ros_=std::make_shared<nav2_costmap_2d::Costmap2DROS>("command_offset_regression");
    parameters_handler_=nullptr;
    control_sequence_.reset(3);
    control_sequence_.vx(0)=.1f;
    control_sequence_.vx(1)=.4f;
    control_sequence_.wz(0)=.2f;
    control_sequence_.wz(1)=.8f;
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc,argv);
  try {
    CommandFixture command;
    command.prepare();
    builtin_interfaces::msg::Time stamp;
    command.setMotionModel("DiffDrive");
    require(std::abs(command.getControlFromSequenceAsTwist(stamp).twist.linear.x-.4)<1e-6,
      "Original command offset changed");
    command.setMotionModel("CalibratedDiffDrive");
    auto chosen=command.getControlFromSequenceAsTwist(stamp).twist;
    require(std::abs(chosen.linear.x-.1)<1e-6 && std::abs(chosen.angular.z-.2)<1e-6,
      "Calibrated command did not match the first scored action");
    mppi::models::OptimizerSettings settings;
    settings.model_dt=.05f;
    settings.constraints.vx_max=.5f;
    settings.constraints.vx_min=-.25f;
    settings.constraints.wz=1.0f;
    mppi::CalibratedDiffDriveMotionModel model(settings);
    mppi::models::State state;
    state.reset(3,56);
    for (size_t i=0;i<3;++i) {
      state.vx(i,0)=.2f;
      state.wz(i,0)=-.4f;
      for (size_t j=0;j<56;++j) {
        state.cvx(i,j)=i==0 ? .5f : i==1 ? -5.0f : 0.0f;
        state.cwz(i,j)=i==0 ? 1.0f : i==1 ? -5.0f : 0.0f;
      }
    }
    auto instant=state;
    mppi::DiffDriveMotionModel original;
    original.predict(instant);
    model.predict(state);
    require(std::abs(instant.vx(0,1)-.5f)<1e-6f,"Original instantaneous response not reproduced");
    require(std::abs(state.vx(0,1)-.2f)<1e-6f,"Command moved before its acquisition latency");
    require(std::abs(state.wz(0,1)+.4f)<1e-6f,"Turn moved before its acquisition latency");
    require(state.vx(0,2)>.2f && state.vx(0,2)<.25f,"Delayed linear response did not begin gradually");
    require(state.wz(0,2)>-.4f && state.wz(0,2)<-.3f,"Delayed angular response did not begin gradually");
    for (size_t i=0;i<3;++i) for (size_t j=1;j<56;++j) {
      require(std::abs(state.vx(i,j)-state.vx(i,j-1))<=.050001f,"Linear acceleration exceeded calibration");
      require(std::abs(state.wz(i,j)-state.wz(i,j-1))<=.100001f,"Angular acceleration exceeded calibration");
      require(state.vx(i,j)>=-.25f && state.vx(i,j)<=.5f,"Predicted linear response escaped calibrated limits");
      require(std::abs(state.wz(i,j))<=1,"Predicted turn response escaped calibrated limits");
    }
    require(state.vx(0,55)>.499f && state.wz(0,55)>.999f,"Forward response did not converge");
    require(state.vx(1,55)<-.249f && state.wz(1,55)<-.999f,"Reverse response did not converge");
    require(std::abs(state.vx(2,55))<.001f && std::abs(state.wz(2,55))<.001f,"Stop response did not converge");
    require(state.cvx(1,55)==-5 && state.cwz(1,55)==-5,
      "Gaussian perturbations were clipped before the MPPI weighted update");
    settings.constraints.vx_max=.3f;
    model.predict(state);
    require(state.vx(0,55)<.301f,"Changed speed limit was not applied to physical prediction");

    // A rapid left-to-right command looks safe instantaneously, but inertia
    // continues the left turn into a cell just outside the initial body.
    settings.constraints.vx_max=.5f;
    mppi::models::State turn;
    turn.reset(1,20);
    turn.vx(0,0)=.5f; turn.wz(0,0)=1.0f;
    turn.cvx.fill(.5f); turn.cwz.fill(-1.0f);
    auto nominal=turn;
    original.predict(nominal); model.predict(turn);
    nav2_costmap_2d::Costmap2D map(200,200,.05,-3,-3,200);
    map.setCost(77,73,254); // cell center (0.875,0.675)
    auto node=std::make_shared<rclcpp_lifecycle::LifecycleNode>("calibrated_collision_regression");
    nav2_smac_planner::GridCollisionChecker checker(&map,72,node);
    nav2_costmap_2d::Footprint body;
    for (auto xy : {std::pair<double,double>{.92,.56},{.92,-.56},{-.47,-.56},{-.47,.56}}) {
      geometry_msgs::msg::Point p; p.x=xy.first; p.y=xy.second; body.push_back(p);
    }
    checker.setFootprint(body,false,-1);
    checker.cacheCollisionBounds();
    auto collides=[&](const mppi::models::State & trajectory) {
        double x=0,y=0,yaw=0;
        for (size_t j=0;j<20;++j) {
          x+=trajectory.vx(0,j)*std::cos(yaw)*.05;
          y+=trajectory.vx(0,j)*std::sin(yaw)*.05;
          yaw+=trajectory.wz(0,j)*.05;
          double angle=std::fmod(yaw+2*M_PI,2*M_PI);
          if (checker.inCollision((x+3)/.05-.5,(y+3)/.05-.5,angle*72/(2*M_PI),true)) return true;
        }
        return false;
      };
    require(!collides(nominal),"Instantaneous-turn safety mismatch not reproduced");
    require(collides(turn),"Calibrated turn failed to expose the inertial body overlap");
    // A queued positive turn still acts during transport latency while the
    // next requested turn reverses. Measured rate alone hides that motion.
    mppi::CalibratedDiffDriveMotionModel feedback_model(settings);
    mppi::models::State queued;
    queued.reset(1,20);queued.vx(0,0)=.5f;queued.wz(0,0)=0.f;
    queued.cvx.fill(.5f);queued.cwz.fill(-1.f);
    auto unobserved=queued;
    model.predict(unobserved);
    feedback_model.setCommandFeedback({{9.85,.5f,1.f},{9.98,.5f,1.f}},10.,20.);
    feedback_model.predict(queued);
    require(queued.wz(0,1)>.09f && std::abs(unobserved.wz(0,1))<1e-6f,
      "Issued turn already in flight was replaced with measured velocity");
    require(queued.wz(0,4)>0.f && unobserved.wz(0,4)<0.f,
      "Normal smoother ramp was skipped during a requested turn reversal");
    std::fill(map.getCharMap(),map.getCharMap()+40000,200);
    map.setCost(79,71,254); // cell center (0.975,0.575), outside the initial body
    checker.cacheCollisionBounds();
    require(!collides(unobserved) && collides(queued),
      "Queued-command body hazard was not exposed by actual input feedback");
    auto stale=unobserved;
    stale.vx.fill(.5f);stale.wz.fill(0.f);stale.cvx.fill(.5f);stale.cwz.fill(0.f);
    feedback_model.setCommandFeedback({{0.,.5f,1.f}},10.,20.);
    feedback_model.predict(stale);
    require(std::abs(stale.wz(0,19))<1e-6f,"Stale command feedback invented an active turn");
    auto ramp=stale;ramp.vx.fill(0.f);ramp.wz.fill(0.f);ramp.cvx.fill(.5f);ramp.cwz.fill(1.f);
    auto step=ramp;model.predict(step);
    feedback_model.setCommandFeedback({{9.95,0.f,0.f},{10.,0.f,0.f}},10.,20.);
    feedback_model.predict(ramp);
    require(ramp.wz(0,4)<step.wz(0,4) && ramp.vx(0,4)<step.vx(0,4),
      "Normal command smoothing did not precede the calibrated actuator response");
    require(ramp.cvx(0,19)==.5f && ramp.cwz(0,19)==1.f,
      "Feedback prediction changed sampled command perturbations");
    mppi::CalibratedDiffDriveMotionModel history_model(settings);
    auto history=ramp;history.vx.fill(.5f);history.wz.fill(0.f);history.cvx.fill(.5f);history.cwz.fill(0.f);
    history_model.setCommandFeedback({{9.85,.5f,1.f},{9.98,.5f,-1.f}},10.,20.);
    history_model.predict(history);
    require(history.wz(0,1)>0.f,"Transport latency used the newest target instead of its time-matched history");
    std::cout << "PASS: command latency, response bounds, Gaussian sampling, reverse/stop, changed limit, inertial collision\n";
  } catch (const std::exception & error) {
    std::cerr << "FAIL: " << error.what() << '\n';
    rclcpp::shutdown(); return 1;
  }
  rclcpp::shutdown(); return 0;
}
