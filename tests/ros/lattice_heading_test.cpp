#include "nav2_mppi_controller/critics/path_angle_critic.hpp"
#include <iostream>
#include <stdexcept>

void require(bool condition, const char * message)
{
  if (!condition) {throw std::runtime_error(message);}
}

class TestCritic : public mppi::critics::PathAngleCritic
{
public:
  void setup(bool rotations) {
    enabled_=true; consider_lattice_rotations_=rotations;
    threshold_to_consider_=.5; max_angle_to_furthest_=0;
    offset_from_furthest_=4; power_=1; weight_=6;
    forward_preference_=false; reversing_allowed_=true;
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc,argv);
  try {
    mppi::models::State state;
    state.pose.pose.orientation.w=1;
    mppi::models::Trajectories trajectories;
    trajectories.reset(3,4);
    xt::view(trajectories.yaws,0,xt::all())=0;
    xt::view(trajectories.yaws,1,xt::all())=.8f;
    xt::view(trajectories.yaws,2,xt::all())=.8f-M_PI;
    mppi::models::Path path;
    path.reset(5);
    path.x(3)=1; path.x(4)=2;
    path.yaws(1)=.4; path.yaws(2)=.8;
    auto costs=xt::xtensor<float,1>::from_shape({3});
    float dt=.1;
    mppi::CriticData data{state,trajectories,path,costs,dt,false,nullptr,nullptr,
                         std::nullopt,std::optional<size_t>{0}};
    TestCritic critic;
    const auto score=[&](bool rotations) {critic.setup(rotations);costs.fill(0);critic.score(data);};
    score(false);
    require(costs(0)<costs(1),"Original position bearing mismatch was not reproduced");
    score(true);
    require(costs(1)<1e-5 && costs(0)>4 && costs(2)>18,
      "Lattice rotation did not prefer its body heading over bearing or reverse equivalence");
    require(path.x.shape(0)==5 && path.x(4)==2,"Heading score truncated forward lookahead");
    path.yaws(1)=.05;path.yaws(2)=.10;
    score(true);
    require(costs(0)<1e-5 && costs(1)>4,
      "Small lattice-bin alignment suppressed the following translating bearing");
    path.yaws.fill(0);
    score(true);
    require(costs(0)<1e-5 && costs(1)>4,"Unchanged repeated position became a required turn");
    path.x(0)=path.x(1)=path.x(2)=1;
    path.x(3)=1.5;path.x(4)=2;
    path.yaws(1)=.4;path.yaws(2)=.8;
    score(true);
    require(costs(0)<1e-5,"Distant rotation displaced the current translating bearing");
    path.x(0)=path.x(1)=path.x(2)=0;
    path.yaws(0)=3;path.yaws(1)=-3.1;path.yaws(2)=-2.8;
    xt::view(trajectories.yaws,1,xt::all())=-2.8f;
    score(true);
    require(costs(1)<1e-5,"Wrapped rotation heading was not retained");
    path.reset(3);path.yaws(1)=.4;path.yaws(2)=.8;
    xt::view(trajectories.yaws,1,xt::all())=.8f;
    score(true);
    require(costs(1)<1e-5 && costs(0)>4,"Near-goal skip hid a required stationary heading");
    std::cout << "PASS: rotation heading, asymmetric reverse, lookahead, duplicates, distant and wrapped turns\n";
  } catch (const std::exception & error) {
    std::cerr << "FAIL: " << error.what() << '\n';
    rclcpp::shutdown();return 1;
  }
  rclcpp::shutdown();return 0;
}
