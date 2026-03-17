#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "robot_msgs/msg/target_position.hpp"

using namespace std::chrono_literals;

class ArmController : public rclcpp::Node{

        public:
                ArmController() : Node("arm_controller"){

                        position_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
                                "/position_controller/commands", 10
                        );

                        target_position_sub_ = this->create_subscription<robot_msgs::msg::TargetPosition>(
                                "/target_position", 10,
                                std::bind(&ArmController::target_position_callback, this, std::placeholders::_1)
                        );

                        RCLCPP_INFO(this->get_logger(), "Launched without error");
                }


        private:

                void target_position_callback(const robot_msgs::msg::TargetPosition::SharedPtr target_msg){
                        double x_target = target_msg->x_target;
                        double y_target = target_msg->y_target;
                        double z_target = target_msg->z_target;

                        RCLCPP_INFO(this->get_logger(), "x_target=%.2f, y_target=%.2f z_target=%.2f", x_target, y_target, z_target);

                }

                rclcpp::Subscription<robot_msgs::msg::TargetPosition>::SharedPtr target_position_sub_;

                rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr position_pub_;
};


int main(int argc, char * argv[]){
        rclcpp::init(argc, argv);
        rclcpp::spin(std::make_shared<ArmController>());
        rclcpp::shutdown();
        return 0;
}