
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include <rosgraph_msgs/msg/clock.hpp>

using std::placeholders::_1;

class TimeRepublish : public rclcpp::Node
{
    protected:
        rclcpp::Clock clock_;
        rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr pub_;
        rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_;
        double last_sim_time_;
        rclcpp::Time last_print_;

        void timeCallback(std_msgs::msg::Float32::SharedPtr msg) {
            rosgraph_msgs::msg::Clock clock;
            if (last_sim_time_ < 0) {
                last_sim_time_ = msg->data;
            }
            clock.clock.sec = int(msg->data);
            clock.clock.nanosec = (msg->data - clock.clock.sec)*1e9;
            pub_->publish(clock);
            rclcpp::Time now = clock_.now();
            if ((now - last_print_).seconds() > 5.0) {
                double ratio = 100.*(msg->data-last_sim_time_)/(now-last_print_).seconds();
                if (ratio > 1) {
                    // Skipping, initial data
                    RCLCPP_INFO(this->get_logger(),"Real-time ratio: %.0f%%",ratio);
                }
                last_print_ = now;
                last_sim_time_ = msg->data;
            }
        }

    public:
        TimeRepublish() : rclcpp::Node("time_republish"), last_sim_time_(-1) {
            last_print_ = clock_.now();
            pub_ = this->create_publisher<rosgraph_msgs::msg::Clock>("/clock",1);
            sub_ = this->create_subscription<std_msgs::msg::Float32>("/vrep/simTime",1,std::bind(&TimeRepublish::timeCallback,this,std::placeholders::_1));
        }
};

int main(int argc, char * argv[]) {

    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TimeRepublish>());
    rclcpp::shutdown();
    return 0;
}


