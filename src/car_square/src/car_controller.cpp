#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "control_msgs/msg/dynamic_joint_state.hpp"
#include <cmath>

using namespace std::chrono_literals;

class CarController : public rclcpp::Node {

        public:
                CarController() : Node("car_controller"){

                        joy_sub = this->create_subscription<sensor_msgs::msg::Joy>(
                                "/joy", 10,
                                std::bind(&CarController::joy_callback, this, std::placeholders::_1)
                        );

                        cmd_vel_pub = this->create_publisher<geometry_msgs::msg::TwistStamped>(
                                "/omni_wheel_drive_controller/cmd_vel", 10
                        );

                        dynamic_joint_states_sub = this->create_subscription<control_msgs::msg::DynamicJointState>(
                                "/dynamic_joint_states", 10,
                                std::bind(&CarController::dynamic_joint_states_callback, this, std::placeholders::_1)
                        );
                        last_time_ = this->now();

                }

        private: 
        
                void joy_callback(const sensor_msgs::msg::Joy::SharedPtr joy) {
                        auto msg = geometry_msgs::msg::TwistStamped();

                        msg.header.stamp = this->now();
                        msg.header.frame_id = "base_link";

                        msg.twist.linear.x  = -joy->axes[0] * 1.0;
                        msg.twist.linear.y  = -joy->axes[1] * 1.0;
                        msg.twist.angular.z = joy->axes[3] * 1.0;

                        cmd_vel_pub->publish(msg);
                }

                void dynamic_joint_states_callback(const control_msgs::msg::DynamicJointState::SharedPtr msg){
                        // Position (rad) - values[0]
                        double lf_pos = msg->interface_values[0].values[0];
                        double rf_pos = msg->interface_values[1].values[0];
                        double rb_pos = msg->interface_values[2].values[0];
                        double lb_pos = msg->interface_values[3].values[0];

                        // Velocity (rad/s) - values[1]
                        double lf_vel = msg->interface_values[0].values[1];
                        double rf_vel = msg->interface_values[1].values[1];
                        double rb_vel = msg->interface_values[2].values[1];
                        double lb_vel = msg->interface_values[3].values[1];

                        // Temps écoulé depuis le dernier appel
                        auto now = this->now();
                        double dt = (now - last_time_).seconds();
                        last_time_ = now;

                        if (dt <= 0.0 || dt > 1.0) return;

                        // Forward kinematics omni 4 roues
                        // vx, vy en m/s dans le repère base_link
                        double vx    = (lf_vel - rf_vel + rb_vel - lb_vel) * WHEEL_RADIUS / 4.0;
                        double vy = (lf_vel + rf_vel - rb_vel - lb_vel) * WHEEL_RADIUS / 4.0;
                        double omega = (lf_vel + rf_vel + rb_vel + lb_vel) * WHEEL_RADIUS / (4.0 * ROBOT_RADIUS);

                        // Intégration dans le repère world (odom)
                        double delta_x     = (vx * cos(theta_) - vy * sin(theta_)) * dt;
                        double delta_y     = -(vx * sin(theta_) + vy * cos(theta_)) * dt;
                        double delta_theta = omega * dt;

                        x_     += delta_x;
                        y_     += delta_y;
                        theta_ += delta_theta;

                        RCLCPP_INFO(this->get_logger(), "lf_vel=%.3f rf_vel=%.3f rb_vel=%.3f lb_vel=%.3f", lf_vel, rf_vel, rb_vel, lb_vel);
                        RCLCPP_INFO(this->get_logger(), "dx=%.3f dy=%.3f dtheta=%.3f", delta_x, delta_y, delta_theta);
                        RCLCPP_INFO(this->get_logger(), "x=%.3f y=%.3f theta=%.3f", x_, y_, theta_);
                }

                rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub;
                rclcpp::Subscription<control_msgs::msg::DynamicJointState>::SharedPtr dynamic_joint_states_sub;
                rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_vel_pub;

                double x_ = 0.0, y_ = 0.0, theta_ = 0.0;
                rclcpp::Time last_time_;
                const double WHEEL_RADIUS = 0.29;
                const double ROBOT_RADIUS = 2.32;

};


int main(int argc, char * argv[]){
        rclcpp::init(argc, argv);
        rclcpp::spin(std::make_shared<CarController>());
        rclcpp::shutdown();
        return 0;
}
