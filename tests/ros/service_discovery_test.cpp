#include <chrono>
#include <iostream>
#include <thread>
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "rcl/service.h"
#include "rcl/publisher.h"
#include "std_msgs/msg/bool.hpp"
#include "rmw_fastrtps_shared_cpp/custom_service_info.hpp"
#include "rmw_fastrtps_shared_cpp/custom_publisher_info.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("service_discovery_regression");
  auto service = node->create_service<std_srvs::srv::Trigger>(
    "/navigate_to_waypoint/_action/send_goal",
    [](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
       std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
      response->success = true;
    });
  const auto * handle = rcl_service_get_rmw_handle(service->get_service_handle().get());
  auto * info = static_cast<CustomServiceInfo *>(handle->data);
  const auto duration = info->response_writer_->get_qos().reliability().max_blocking_time;
  const auto window = std::chrono::seconds(duration.seconds) +
    std::chrono::nanoseconds(duration.nanosec);
  auto publisher = node->create_publisher<std_msgs::msg::Bool>("/cmd_gate_probe", 10);
  auto * topic_info = static_cast<CustomPublisherInfo *>(
    rcl_publisher_get_rmw_handle(publisher->get_publisher_handle().get())->data);
  const auto topic_duration = topic_info->data_writer_->get_qos().reliability().max_blocking_time;
  if (topic_duration.seconds != 0 || topic_duration.nanosec != 100000000) {
    std::cerr << "Service discovery profile changed ordinary topic blocking time\n";
    return 1;
  }
  std::cout << "Native service reply discovery window: " <<
    std::chrono::duration<double>(window).count() << " seconds\n";

  // Exercise the installed middleware's discovery gate with a reader that
  // appears 300 ms after its request writer. This exceeds the old 100 ms.
  ServicePubListener listener(info);
  eprosima::fastrtps::rtps::GUID_t reader, writer;
  reader.guidPrefix.value[0] = writer.guidPrefix.value[0] = 17;
  reader.entityId.value[3] = 4;
  writer.entityId.value[3] = 3;
  listener.endpoint_add_reader_and_writer(reader, writer);
  std::thread discovery([&]() {
    std::this_thread::sleep_for(std::chrono::milliseconds(300));
    eprosima::fastdds::dds::PublicationMatchedStatus matched;
    matched.current_count_change = 1;
    matched.last_subscription_handle = reader;
    listener.on_publication_matched(nullptr, matched);
  });
  const auto discovered = listener.check_for_subscription(reader, window);
  discovery.join();
  if (discovered != client_present_t::YES) {
    std::cerr << "Reply discovery timed out after accepting a request\n";
    return 1;
  }
  if (window != std::chrono::seconds(5)) {
    std::cerr << "Service profile did not apply the bounded five-second window\n";
    return 1;
  }
  // Confirm actual ROS requests/replies still complete with the profile.
  // These use the same service endpoint mapping as action acknowledgements.
  auto client_node = std::make_shared<rclcpp::Node>("service_discovery_client");
  auto client = client_node->create_client<std_srvs::srv::Trigger>(
    "/navigate_to_waypoint/_action/send_goal");
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  executor.add_node(client_node);
  if (!client->wait_for_service(std::chrono::seconds(5))) {
    std::cerr << "Native service discovery failed\n";
    return 1;
  }
  auto reply = client->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>());
  if (executor.spin_until_future_complete(reply, std::chrono::seconds(5)) !=
    rclcpp::FutureReturnCode::SUCCESS || !reply.get()->success)
  {
    std::cerr << "Native service request/reply did not complete\n";
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
