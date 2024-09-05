#include <chrono>
#include <cinttypes>
#include <fstream>

#include "example_interfaces/srv/add_two_ints.hpp"
#include "rclcpp/rclcpp.hpp"

#include "realtime_node/command_line_options.hpp"
#include "realtime_node/sched_utils.hpp"
#include "realtime_node/rusage_utils.hpp"
#include "realtime_node/memory_lock_utils.hpp"

using namespace std::chrono_literals;
using AddTwoInts = example_interfaces::srv::AddTwoInts;
using Clock = std::chrono::steady_clock;

class MinimalServer : public rclcpp::Node {
public:
  MinimalServer()
  : Node("minimal_server")
  {
    minimal_server_ = this->create_service<AddTwoInts>(
      "add_two_ints",
      std::bind(&MinimalServer::handle_service, this, std::placeholders::_1, std::placeholders::_2)
    );

    RCLCPP_INFO(this->get_logger(), "Minimal Server is ready!");
  }

  void handle_service(
    const std::shared_ptr<AddTwoInts::Request> request,
    const std::shared_ptr<AddTwoInts::Response> response)
  {
    RCLCPP_INFO(this->get_logger(), "Requset is comming!");
    response->sum = request->a + request->b;
  }

private:
  std::string log_file_;
  std::ofstream log_stream_;
  rclcpp::Service<AddTwoInts>::SharedPtr minimal_server_;
};

int main(int argc, char ** argv)
{
  // Force flush of the stdout buffer
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);

  // Read Scheduling Option in Command Line
  auto options_reader = SchedOptionsReader();
  if(!options_reader.read_options(argc, argv)){
      options_reader.print_usage();
      return 0;
  }

  // Get Scheduling Option
  auto options = options_reader.get_options();

  set_thread_scheduling(pthread_self(), options.policy, options.priority);

  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalServer>());
  rclcpp::shutdown();
  return 0;
}