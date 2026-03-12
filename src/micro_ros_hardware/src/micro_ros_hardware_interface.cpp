#include "micro_ros_hardware/micro_ros_hardware_interface.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace micro_ros_hardware
{

        // on_init : lecture du URDF, initialisation des vecteurs
        hardware_interface::CallbackReturn MicroRosHardwareInterface::on_init(const hardware_interface::HardwareInfo & info){
                if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS){
                        return hardware_interface::CallbackReturn::ERROR;
                }

                cmd_topic_   = info_.hardware_parameters.count("cmd_topic")
                                ? info_.hardware_parameters.at("cmd_topic")
                                : "/wheel_commands";
                state_topic_ = info_.hardware_parameters.count("state_topic")
                                ? info_.hardware_parameters.at("state_topic")
                                : "/micro_controller/joint_states";

                joint_names_.resize(info_.joints.size());
                hw_states_position_.resize(info_.joints.size(), 0.0);
                hw_states_velocity_.resize(info_.joints.size(), 0.0);
                hw_commands_velocity_.resize(info_.joints.size(), 0.0);

                for (size_t i = 0; i < info_.joints.size(); ++i) {
                        joint_names_[i] = info_.joints[i].name;
                        RCLCPP_INFO(rclcpp::get_logger("MicroRosHardwareInterface"),
                        "Joint enregistré : %s", joint_names_[i].c_str());
                }

                return hardware_interface::CallbackReturn::SUCCESS;
        }

        // on_configure : création du node ROS2, publisher et subscriber
        hardware_interface::CallbackReturn MicroRosHardwareInterface::on_configure(const rclcpp_lifecycle::State & /*previous_state*/){
                node_ = rclcpp::Node::make_shared("micro_ros_hardware_interface");

                // Publisher → ESP32 : vitesses cibles de chaque roue
                wheel_cmd_pub_ = node_->create_publisher<robot_messages::msg::WheelsCommand>(
                        cmd_topic_, rclcpp::QoS(10)
                );
                
                // Subscriber ← ESP32 : positions et vitesses encodeurs
                joint_state_sub_ = node_->create_subscription<sensor_msgs::msg::JointState>(
                        state_topic_, 
                        rclcpp::QoS(10),
                        [this](const sensor_msgs::msg::JointState::SharedPtr msg) {
                                std::lock_guard<std::mutex> lock(joint_state_mutex_);
                                latest_joint_states_ = *msg;
                                joint_states_received_ = true;
                        }
                );

                RCLCPP_INFO(rclcpp::get_logger("MicroRosHardwareInterface"),
                        "Configuré — cmd: %s | states: %s",
                        cmd_topic_.c_str(), state_topic_.c_str()
                );

                return hardware_interface::CallbackReturn::SUCCESS;
        }

        // on_activate / on_deactivate
        hardware_interface::CallbackReturn MicroRosHardwareInterface::on_activate(const rclcpp_lifecycle::State & /*previous_state*/){
                // Reset des commandes à zéro à l'activation
                std::fill(hw_commands_velocity_.begin(), hw_commands_velocity_.end(), 0.0);
                RCLCPP_INFO(rclcpp::get_logger("MicroRosHardwareInterface"), "Hardware activé.");
                return hardware_interface::CallbackReturn::SUCCESS;
        }

        hardware_interface::CallbackReturn MicroRosHardwareInterface::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/){
                // Envoyer zéro à l'ESP32 avant de désactiver
                robot_messages::msg::WheelsCommand msg;
                msg.lb_joint = 0.f;
                msg.rb_joint = 0.f;
                msg.lf_joint = 0.f;
                msg.rf_joint = 0.f;
                wheel_cmd_pub_->publish(msg);
                RCLCPP_INFO(rclcpp::get_logger("MicroRosHardwareInterface"), "Hardware désactivé.");
                return hardware_interface::CallbackReturn::SUCCESS;
        }

        // export_state_interfaces : position + vitesse pour chaque joint
        std::vector<hardware_interface::StateInterface>MicroRosHardwareInterface::export_state_interfaces(){
                std::vector<hardware_interface::StateInterface> state_interfaces;

                for (size_t i = 0; i < joint_names_.size(); ++i) {
                        state_interfaces.emplace_back(
                                joint_names_[i],
                                hardware_interface::HW_IF_POSITION,
                                &hw_states_position_[i]
                        );

                        state_interfaces.emplace_back(
                                joint_names_[i],
                                hardware_interface::HW_IF_VELOCITY,
                                &hw_states_velocity_[i]
                        );
                }

                return state_interfaces;
        }

        // export_command_interfaces : vitesse à donner pour chaque roues
        std::vector<hardware_interface::CommandInterface>MicroRosHardwareInterface::export_command_interfaces(){
                std::vector<hardware_interface::CommandInterface> command_interfaces;

                for (size_t i = 0; i < joint_names_.size(); ++i) {
                        command_interfaces.emplace_back(
                                joint_names_[i],
                                hardware_interface::HW_IF_VELOCITY,
                                &hw_commands_velocity_[i]
                        );
                }

                return command_interfaces;
        }

        // read() : récupère les encodeurs de l'ESP32 → state interfaces
        hardware_interface::return_type MicroRosHardwareInterface::read(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/){
                rclcpp::spin_some(node_);

                if (!joint_states_received_) {
                        return hardware_interface::return_type::OK;
                }

                std::lock_guard<std::mutex> lock(joint_state_mutex_);

                for (size_t i = 0; i < joint_names_.size(); ++i) {
                        for (size_t j = 0; j < latest_joint_states_.name.size(); ++j) {
                                if (latest_joint_states_.name[j] == joint_names_[i]) {
                                        if (j < latest_joint_states_.position.size()) {
                                                hw_states_position_[i] = latest_joint_states_.position[j];
                                        }
                                        if (j < latest_joint_states_.velocity.size()) {
                                                hw_states_velocity_[i] = latest_joint_states_.velocity[j];
                                        }
                                        break;
                                }
                        }
                }

                return hardware_interface::return_type::OK;
        }

        // write() : envoie les commandes de vitesse vers l'ESP32
        hardware_interface::return_type MicroRosHardwareInterface::write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/){
                robot_messages::msg::WheelsCommand msg;

                msg.lf_joint = hw_commands_velocity_[0];
                msg.rf_joint = hw_commands_velocity_[1];
                msg.lb_joint = hw_commands_velocity_[2];
                msg.rb_joint = hw_commands_velocity_[3];

                wheel_cmd_pub_->publish(msg);

                return hardware_interface::return_type::OK;
        }

}

PLUGINLIB_EXPORT_CLASS(
  micro_ros_hardware::MicroRosHardwareInterface,
  hardware_interface::SystemInterface
)