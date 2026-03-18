#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "robot_msgs/msg/target_position.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "cmath"

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

                        position_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
                                "/position_controller/commands", 10,
                                std::bind(&ArmController::position_controller_callback, this, std::placeholders::_1)
                        );

                        RCLCPP_INFO(this->get_logger(), "Launched without error");
                }


        private:

                void compute_3D_position(double theta_1, double theta_2, double theta_3, double theta_4){
                        
                        double x = 0.0;
                        double y = 0.0;
                        double z = 0.0;

                        z = ARM_HEIGHT*(sin(theta_2) + sin(theta_3) + sin(theta_4));

                        double value = ARM_HEIGHT*(cos(theta_2) + cos(theta_3) + cos(theta_4));
                        x = value*cos(theta_1);
                        y = value*sin(theta_1);

                        RCLCPP_INFO(this->get_logger(), "x=%f, y=%f, z=%f", x, y, z);
                        
                }

                void position_controller_callback(const std_msgs::msg::Float64MultiArray::SharedPtr position_msg){
                        double first_arm_joint_position  = position_msg->data[0];
                        double second_arm_joint_position = position_msg->data[1];
                        double third_arm_joint_position  = position_msg->data[2];
                        double fourth_arm_joint_position = position_msg->data[3];

                        compute_3D_position(first_arm_joint_position, second_arm_joint_position, third_arm_joint_position, fourth_arm_joint_position);
                }

                void target_position_callback(const robot_msgs::msg::TargetPosition::SharedPtr target_msg){
                        double x_target = target_msg->x_target;
                        double y_target = target_msg->y_target;
                        double z_target = target_msg->z_target;

                        RCLCPP_INFO(this->get_logger(), "x_target=%.2f, y_target=%.2f z_target=%.2f", x_target, y_target, z_target);

                }

                rclcpp::Subscription<robot_msgs::msg::TargetPosition>::SharedPtr target_position_sub_;
                rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr position_sub_;
                rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr position_pub_;

                double ARM_HEIGHT = 3.0;
};


int main(int argc, char * argv[]){
        rclcpp::init(argc, argv);
        rclcpp::spin(std::make_shared<ArmController>());
        rclcpp::shutdown();
        return 0;
}