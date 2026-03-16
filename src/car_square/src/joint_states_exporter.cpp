#include "rclcpp/rclcpp.hpp"
#include "control_msgs/msg/dynamic_joint_state.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

using namespace std::chrono_literals;

class JointStatesExport : public rclcpp::Node {

        public:
        // This node allows to export de joint_state value coming from gazebo (published in /dynamic_joint_states) in the defined topic for IRL (/micro_controller/joint_state)
        // Thus, we can use the same car_controller node for both simulation and IRL
                JointStatesExport() : Node("car_controller"){
                        dynamic_joint_states_sub = this->create_subscription<control_msgs::msg::DynamicJointState>(
                               "/dynamic_joint_states", 10,
                                std::bind(&JointStatesExport::dynamic_joint_states_callback, this, std::placeholders::_1)
                        );

                        joint_states_pub = this->create_publisher<sensor_msgs::msg::JointState>(
                                "/micro_controller/joint_states", 10
                        );
                }

        private: 
                
                void dynamic_joint_states_callback(const control_msgs::msg::DynamicJointState::SharedPtr msg){
                        auto pub_msg = sensor_msgs::msg::JointState();

                        pub_msg.header.stamp = this->now();
                        pub_msg.header.frame_id = "base_link";

                        pub_msg.name.push_back("lf_joint");
                        pub_msg.name.push_back("rf_joint");
                        pub_msg.name.push_back("rb_joint");
                        pub_msg.name.push_back("lb_joint");

                        pub_msg.position.resize(4);
                        pub_msg.velocity.resize(4);

                        pub_msg.position[0] = msg->interface_values[0].values[0];
                        pub_msg.position[1] = msg->interface_values[1].values[0];
                        pub_msg.position[2] = msg->interface_values[2].values[0];
                        pub_msg.position[3] = msg->interface_values[3].values[0];


                        pub_msg.velocity[0] = msg->interface_values[0].values[1];
                        pub_msg.velocity[1] = msg->interface_values[1].values[1];
                        pub_msg.velocity[2] = msg->interface_values[2].values[1];
                        pub_msg.velocity[3] = msg->interface_values[3].values[1];

                        joint_states_pub->publish(pub_msg);
                }

                rclcpp::Subscription<control_msgs::msg::DynamicJointState>::SharedPtr dynamic_joint_states_sub;
                rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_states_pub;
};


int main(int argc, char * argv[]){
        rclcpp::init(argc, argv);
        rclcpp::spin(std::make_shared<JointStatesExport>());
        rclcpp::shutdown();
        return 0;
}
