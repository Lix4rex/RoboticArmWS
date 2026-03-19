        #include "ros_hardware/ros_hardware_interface.hpp"
        #include "pluginlib/class_list_macros.hpp"

        namespace ros_hardware
        {

                // on_init : lecture du URDF, initialisation des vecteurs
                hardware_interface::CallbackReturn RosHardwareInterface::on_init(const hardware_interface::HardwareInfo & info){
                        if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS){
                                return hardware_interface::CallbackReturn::ERROR;
                        }

                        cmd_topic_   = info_.hardware_parameters.count("cmd_topic")
                                        ? info_.hardware_parameters.at("cmd_topic")
                                        : "/arm_commands";

                        joint_names_.resize(info_.joints.size());
                        hw_commands_position_.resize(info_.joints.size(), 0.0);
                        hw_states_position_.resize(info_.joints.size(), 0.0);

                        for (size_t i = 0; i < info_.joints.size(); ++i) {
                                joint_names_[i] = info_.joints[i].name;
                                RCLCPP_INFO(rclcpp::get_logger("RosHardwareInterface"),
                                "Joint enregistré : %s", joint_names_[i].c_str());
                        }

                        return hardware_interface::CallbackReturn::SUCCESS;
                }

                // on_configure : création du node ROS2, publisher et subscriber
                hardware_interface::CallbackReturn RosHardwareInterface::on_configure(const rclcpp_lifecycle::State & /*previous_state*/){
                        node_ = rclcpp::Node::make_shared("ros_hardware_interface");

                        // Publisher → ESP32 : vitesses cibles de chaque roue
                        angle_cmd_pub_ = node_->create_publisher<std_msgs::msg::Float64MultiArray>(
                                cmd_topic_, rclcpp::QoS(10)
                        );

                        RCLCPP_INFO(rclcpp::get_logger("RosHardwareInterface"),
                                "Configuré — cmd: %s",
                                cmd_topic_.c_str()
                        );

                        return hardware_interface::CallbackReturn::SUCCESS;
                }

                // on_activate / on_deactivate
                hardware_interface::CallbackReturn RosHardwareInterface::on_activate(const rclcpp_lifecycle::State & /*previous_state*/){
                        // Reset des commandes à zéro à l'activation
                        std::fill(hw_commands_position_.begin(), hw_commands_position_.end(), 0.0);
                        RCLCPP_INFO(rclcpp::get_logger("RosHardwareInterface"), "Hardware activé.");
                        return hardware_interface::CallbackReturn::SUCCESS;
                }

                hardware_interface::CallbackReturn RosHardwareInterface::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/){
                        // Envoyer zéro à l'ESP32 avant de désactiver
                        std_msgs::msg::Float64MultiArray msg;
                        msg.data.resize(4);
                        msg.data[0] = 0.f;
                        msg.data[1] = 0.f;
                        msg.data[2] = 0.f;
                        msg.data[3] = 0.f;
                        angle_cmd_pub_->publish(msg);
                        RCLCPP_INFO(rclcpp::get_logger("RosHardwareInterface"), "Hardware désactivé.");
                        return hardware_interface::CallbackReturn::SUCCESS;
                }

                // export_state_interfaces : position + vitesse pour chaque joint
                std::vector<hardware_interface::StateInterface>RosHardwareInterface::export_state_interfaces(){
                        std::vector<hardware_interface::StateInterface> state_interfaces;

                        for (size_t i = 0; i < joint_names_.size(); ++i) {
                                state_interfaces.emplace_back(
                                joint_names_[i],
                                hardware_interface::HW_IF_POSITION,
                                &hw_states_position_[i]
                                );
                        }

                        return state_interfaces;
                }

                // export_command_interfaces : vitesse à donner pour chaque roues
                std::vector<hardware_interface::CommandInterface>RosHardwareInterface::export_command_interfaces(){
                        std::vector<hardware_interface::CommandInterface> command_interfaces;

                        for (size_t i = 0; i < joint_names_.size(); ++i) {
                                command_interfaces.emplace_back(
                                        joint_names_[i],
                                        hardware_interface::HW_IF_POSITION,
                                        &hw_commands_position_[i]
                                );
                        }

                        return command_interfaces;
                }

                // read() : récupère les encodeurs de l'ESP32 → state interfaces
                hardware_interface::return_type RosHardwareInterface::read(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/){

                        for (size_t i = 0; i < hw_states_position_.size(); ++i) {
                                hw_states_position_[i] = hw_commands_position_[i];
                        }
                        return hardware_interface::return_type::OK;
                }

                // write() : envoie les commandes de vitesse vers l'ESP32
                hardware_interface::return_type RosHardwareInterface::write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/){
                        std_msgs::msg::Float64MultiArray msg;
                        msg.data.resize(4);
                        for (int i=0; i<4; i++){
                                msg.data[i] = hw_commands_position_[i];
                        }

                        angle_cmd_pub_->publish(msg);

                        return hardware_interface::return_type::OK;
                }

        }

        PLUGINLIB_EXPORT_CLASS(
                ros_hardware::RosHardwareInterface,
                hardware_interface::SystemInterface
        )