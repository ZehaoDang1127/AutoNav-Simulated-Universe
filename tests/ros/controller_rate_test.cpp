#include <iostream>
#include <stdexcept>
#include "rcl/time.h"
#include "nav2_controller/ros_clock_rate.hpp"

void require(bool condition, const char * message)
{
  if (!condition) {throw std::runtime_error(message);}
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto clock = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
  require(rcl_enable_ros_time_override(clock->get_clock_handle()) == RCL_RET_OK,
    "could not enable ROS clock override");
  const auto set_time = [&clock](int64_t ns) {
      require(rcl_set_ros_time_override(clock->get_clock_handle(), ns) == RCL_RET_OK,
        "could not set ROS clock override");
    };
  set_time(100000000000LL);
  nav2_controller::ROSClockRate rate(clock, 10.0);
  int polls = 0;
  rate.sleep([&]() {return ++polls < 4;});
  require(polls == 4, "paused clock returned before cancellation");

  polls = 0;
  require(rate.sleep([&]() {
      ++polls;
      set_time(polls < 3 ? 100099999999LL : 100100000000LL);
      return polls < 8;
    }), "first clock interval was treated as an overrun");
  require(polls == 3, "control interval ended before its ROS deadline");

  set_time(100500000000LL);
  require(!rate.sleep([]() {return true;}), "large clock jump did not report a missed interval");
  polls = 0;
  rate.sleep([&]() {return ++polls < 4;});
  require(polls == 4, "clock jump caused a burst of stale control intervals");

  polls = 0;
  require(rate.sleep([&]() {
      ++polls;
      set_time(polls == 1 ? 95000000000LL :
        (polls == 2 ? 95099999999LL : 95100000000LL));
      return polls < 8;
    }), "rewound clock was treated as an overrun");
  require(polls == 3, "rewind did not reset the control deadline");
  rclcpp::shutdown();
  std::cout << "ROS-clock pause, intervals, overrun and rewind passed\n";
}
