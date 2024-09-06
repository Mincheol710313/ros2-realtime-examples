#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/strategies/allocator_memory_strategy.hpp"

#include "tlsf_cpp/tlsf.hpp"

#include "std_msgs/msg/string.hpp"

#include "rttest/rttest.h"

#include "realtime_node/rtt_executor.hpp"
#include "realtime_node/command_line_options.hpp"
#include "realtime_node/sched_utils.hpp"
#include "realtime_node/rusage_utils.hpp"
#include "realtime_node/memory_lock_utils.hpp"

using namespace std::chrono_literals;
using rclcpp::memory_strategies::allocator_memory_strategy::AllocatorMemoryStrategy;

template<typename  T = void>
using TLSFAllocator = tlsf_heap_allocator<T>;

class MinimalPublisher : public rclcpp::Node
{
public:
    MinimalPublisher()
    : Node("minimal_publisher"), count_(0)
    {   
        auto qos = rclcpp::QoS(10).deadline(std::chrono::milliseconds(1)).best_effort();
        publisher_ = this->create_publisher<std_msgs::msg::String>("topic", qos);
        timer_ = this->create_wall_timer(std::chrono::nanoseconds(1000000), std::bind(&MinimalPublisher::timer_callback, this));
    }

private:
    void timer_callback(){
        auto message = std_msgs::msg::String();
        message.data = "Hello world!" + std::to_string(this->count_);

        this->publisher_->publish(message);
        this->count_++;
    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    int count_;
    std::chrono::_V2::steady_clock::time_point prev_start;
};

int main(int argc, char * argv[])
{   
    // Force flush of the stdout buffer
    // setvbuf(stdout, NULL, _IONBF, BUFSIZ);

    // // Read Scheduling Option in Command Line
    // auto options_reader = SchedOptionsReader();
    // if(!options_reader.read_options(argc, argv)){
    //     options_reader.print_usage();
    //     return 0;
    // }

    // // Get Scheduling Option
    // auto options = options_reader.get_options();

    // set_thread_scheduling(pthread_self(), options.policy, options.priority);

    // // Lock Memoty
    // lock_memory();
    // preallocate_memory(100 * 1024 * 1024);
    rttest_read_args(argc, argv);

    rclcpp::init(argc, argv);
    auto node = std::make_shared<MinimalPublisher>();

    // Executor Option
    rclcpp::ExecutorOptions options;
    rclcpp::memory_strategy::MemoryStrategy::SharedPtr memory_strategy =
        std::make_shared<AllocatorMemoryStrategy<TLSFAllocator<void>>>();
    options.memory_strategy = memory_strategy;

    auto executor = std::make_shared<pendulum_control::RttExecutor>(options);

    executor->add_node(node);

    executor->spin();
    rclcpp::shutdown();
    return 0;
}