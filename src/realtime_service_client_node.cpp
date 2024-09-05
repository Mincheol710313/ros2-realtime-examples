#include <chrono>
#include <cinttypes>
#include <memory>

#include "example_interfaces/srv/add_two_ints.hpp"
#include "rclcpp/rclcpp.hpp"

#include "realtime_node/command_line_options.hpp"
#include "realtime_node/sched_utils.hpp"
#include "realtime_node/rusage_utils.hpp"
#include "realtime_node/memory_lock_utils.hpp"

using namespace std::chrono_literals;
using AddTwoInts = example_interfaces::srv::AddTwoInts;

static ContextSwitchesCounter context_switches_counter_rt(RUSAGE_THREAD);

class MinimalClient : public rclcpp::Node {
public:
    MinimalClient() 
    : Node("minimal_client"), count_(0), loop_count_(1000), cycle_time_ms_(1)
    {   
        this->declare_parameter<int>("loop_count", 1000);
        this->get_parameter("loop_count", loop_count_);
        minimal_client_ = this->create_client<AddTwoInts>("add_two_ints");
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(cycle_time_ms_),
            std::bind(&MinimalClient::request_result, this)
        );
    }

private:
    void request_result() {


        if (!minimal_client_->wait_for_service(1s)) {
            RCLCPP_WARN(this->get_logger(), "Waiting for service to be available...");
            return;
        }

        if(this->count_ < this->loop_count_)
        {  
            auto request = std::make_shared<AddTwoInts::Request>();
            request->a = 0;
            request->b = this->count_;

            using ServiceResponseFuture = rclcpp::Client<AddTwoInts>::SharedFuture;
            auto response_received_callback = [this, start=std::chrono::steady_clock::now()](ServiceResponseFuture future) {
                try {
                    auto result = future.get();
                    auto end = std::chrono::steady_clock::now();

                    auto latency = std::chrono::duration_cast<std::chrono::nanoseconds>(end-start);
                    

                    if(this->count_ == 0)
                        RCLCPP_INFO(this->get_logger(), "Latency : %ld ns, Cycle Time : 0 ns, Result : %ld", latency, result->sum);
                    else{
                        auto cycle_time = std::chrono::duration_cast<std::chrono::nanoseconds>(start-this->prev_start);
                        auto jitter = std::chrono::duration_cast<std::chrono::nanoseconds>(cycle_time - 1ms);
                        RCLCPP_INFO(this->get_logger(), "Latency : %ld ns, Cycle Time : %ld ns, Jitter : %ld ns, Result : %ld", latency, cycle_time, jitter, result->sum);
                    }
                    this->prev_start = start;
                    this->count_++;
                } 
                catch (const std::exception &e) {
                    RCLCPP_ERROR(this->get_logger(), "Service call failed: %s", e.what());
                }
            };

            minimal_client_->async_send_request(request, response_received_callback);
        }
        else{
            rclcpp::shutdown();
        }
    }

    rclcpp::Client<AddTwoInts>::SharedPtr minimal_client_;
    rclcpp::TimerBase::SharedPtr timer_;
    int count_;
    int loop_count_;
    size_t cycle_time_ms_;
    std::chrono::_V2::steady_clock::time_point prev_start;
};

int main(int argc, char * argv[]){
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
    auto node = std::make_shared<MinimalClient>();
    auto spin_thread = std::thread(
        [&](){
            rclcpp::spin(node);
    });
    spin_thread.join();
    rclcpp::shutdown();
    return 0;
}



