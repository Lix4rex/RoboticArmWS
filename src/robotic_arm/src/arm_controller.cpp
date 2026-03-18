#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "robot_msgs/msg/target_position.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "cmath"
#include <Eigen/Dense>

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

                        current_position = Eigen::Vector3d::Zero();
                        current_angle = Eigen::Vector4d::Zero();

                        this->declare_parameter<int>("max_iter");
                        MAX_ITER = this->get_parameter("max_iter").as_int();

                        this->declare_parameter<double>("alpha");
                        alpha = this->get_parameter("alpha").as_double();

                        RCLCPP_INFO(this->get_logger(), "Launched without error");
                }


        private:

                void maj_J(double theta_1, double theta_2, double theta_3, double theta_4){

                        double value = ARM_HEIGHT*(sin(theta_2) + sin(theta_2 + theta_3) + sin(theta_2 + theta_3 + theta_4));

                        double value_derivate_theta_2 = ARM_HEIGHT*(cos(theta_2) + cos(theta_2 + theta_3) + cos(theta_2 + theta_3 + theta_4));
                        double value_derivate_theta_3 = ARM_HEIGHT*(cos(theta_2 + theta_3) + cos(theta_2 + theta_3 + theta_4));
                        double value_derivate_theta_4 = ARM_HEIGHT*(cos(theta_2 + theta_3 + theta_4));

                        J(0,0) = -value*sin(theta_1);
                        J(0,1) = value_derivate_theta_2*cos(theta_1);
                        J(0,2) = value_derivate_theta_3*cos(theta_1);
                        J(0,3) = value_derivate_theta_4*cos(theta_1);

                        J(1,0) = value*cos(theta_1);
                        J(1,1) = value_derivate_theta_2*sin(theta_1);
                        J(1,2) = value_derivate_theta_3*sin(theta_1);
                        J(1,3) = value_derivate_theta_4*sin(theta_1);

                        J(2,0) = 0;
                        J(2,1) = -ARM_HEIGHT*(sin(theta_2) + sin(theta_2 + theta_3) + sin(theta_2 + theta_3 + theta_4));
                        J(2,2) = -ARM_HEIGHT*(sin(theta_2 + theta_3) + sin(theta_2 + theta_3 + theta_4));
                        J(2,3) = -ARM_HEIGHT*(sin(theta_2 + theta_3 + theta_4)); 
                }

                void compute_3D_position(double theta_1, double theta_2, double theta_3, double theta_4){

                        double value = ARM_HEIGHT*(sin(theta_2) + sin(theta_2 + theta_3) + sin(theta_2 + theta_3 + theta_4));
                        
                        current_position(0) = value*cos(theta_1);
                        current_position(1) = value*sin(theta_1);
                        current_position(2) = ARM_HEIGHT*(1 + cos(theta_2) + cos(theta_2 + theta_3) + cos(theta_2 + theta_3 + theta_4));

                        maj_J(theta_1, theta_2, theta_3, theta_4);

                        // RCLCPP_INFO(this->get_logger(), "x=%f, y=%f, z=%f", current_position(0), current_position(1), current_position(2));
                }

                void compute_inverse_kinematic(Eigen::Vector3d target){

                        double alpha = 0.1;
                        current_angle = Eigen::Vector4d::Zero();
                        for (int i=0; i<MAX_ITER; i++){
                                compute_3D_position(current_angle(0), current_angle(1), current_angle(2), current_angle(3));

                                Eigen::Vector3d error = target - current_position;
                                if(error.norm() < 1e-3)
                                        break;

                                Eigen::Matrix3d I = Eigen::Matrix3d::Identity();
                                double lambda = 0.01;
                                Eigen::MatrixXd J_pinv = J.transpose() * (J * J.transpose() + lambda * I).inverse();

                                current_angle += alpha * J_pinv * error;
                        }
                        
                        compute_3D_position(current_angle(0), current_angle(1), current_angle(2), current_angle(3));
                        pub_position_commands(current_angle(0), current_angle(1), current_angle(2), current_angle(3));

                        RCLCPP_INFO(this->get_logger(), "Angles: %.2f %.2f %.2f %.2f", current_angle(0), current_angle(1), current_angle(2), current_angle(3));
                }

                void pub_position_commands(double theta_1, double theta_2, double theta_3, double theta_4){
                        auto msg_input = std_msgs::msg::Float64MultiArray();

                        msg_input.data.resize(4);
                        msg_input.data[0] = theta_1;
                        msg_input.data[1] = theta_2;
                        msg_input.data[2] = theta_3;
                        msg_input.data[3] = theta_4;

                        position_pub_->publish(msg_input);
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

                        Eigen::Vector3d target;
                        target(0) = x_target;
                        target(1) = y_target;
                        target(2) = z_target;

                        compute_inverse_kinematic(target);

                        RCLCPP_INFO(this->get_logger(), "x_target=%.2f, y_target=%.2f z_target=%.2f", x_target, y_target, z_target);

                }

                rclcpp::Subscription<robot_msgs::msg::TargetPosition>::SharedPtr target_position_sub_;
                rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr position_sub_;
                rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr position_pub_;

                double ARM_HEIGHT = 3.0;


                Eigen::Matrix<double, 3, 4> J;

                Eigen::Vector3d current_position;
                Eigen::Vector4d current_angle;

                double alpha;
                int MAX_ITER;

};


int main(int argc, char * argv[]){
        rclcpp::init(argc, argv);
        rclcpp::spin(std::make_shared<ArmController>());
        rclcpp::shutdown();
        return 0;
}