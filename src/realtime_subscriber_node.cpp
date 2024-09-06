#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include "realtime_node/command_line_options.hpp"
#include "realtime_node/sched_utils.hpp"
#include "realtime_node/rusage_utils.hpp"
#include "realtime_node/memory_lock_utils.hpp"

using namespace std::chrono_literals;

class MinimalSubscriber : public rclcpp::Node
{
public:
    MinimalSubscriber()
    : Node("minimal_subscriber")
    {
       auto topic_callback =
      [this](std_msgs::msg::String::UniquePtr msg) -> void {
        RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());
      };
    subscriber_ =
      this->create_subscription<std_msgs::msg::String>("topic", 10, topic_callback);
    }

private:
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscriber_;
};

int main(int argc, char * argv[])
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

    rclcpp::spin(std::make_shared<MinimalSubscriber>());
    rclcpp::shutdown();
    return 0;
}