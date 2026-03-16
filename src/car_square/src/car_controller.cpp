#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "robot_messages/msg/target_position.hpp"
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

                        joint_states_sub = this->create_subscription<sensor_msgs::msg::JointState>(
                                "/micro_controller/joint_states", 10,
                                std::bind(&CarController::joint_states_callback, this, std::placeholders::_1)
                        );

                        position_sub = this->create_subscription<robot_messages::msg::TargetPosition>(
                                "/position_target", 10,
                                std::bind(&CarController::position_target_callback, this, std::placeholders::_1)
                        );

                        last_time_ = this->now();

                        this->declare_parameter<double>("speed");
                        this->declare_parameter<double>("Kp");
                        this->declare_parameter<double>("wheel_radius");
                        this->declare_parameter<double>("robot_radius");
                        this->declare_parameter<double>("accel");
                        this->declare_parameter<bool>("controlable");
                        speed = this->get_parameter("speed").as_double();
                        Kp = this->get_parameter("Kp").as_double();
                        wheel_radius = this->get_parameter("wheel_radius").as_double();
                        robot_radius = this->get_parameter("robot_radius").as_double();
                        accel = this->get_parameter("accel").as_double();
                        controlable = this->get_parameter("controlable").as_bool();

                }

        private: 
                void joy_callback(const sensor_msgs::msg::Joy::SharedPtr joy) {

                        auto msg = geometry_msgs::msg::TwistStamped();

                        msg.header.stamp = this->now();
                        msg.header.frame_id = "base_link";

                        msg.twist.linear.x = joy->axes[0] * speed;
                        msg.twist.linear.y = joy->axes[1] * speed;
                        msg.twist.angular.z = joy->axes[3] * speed / 2.0;

                        if (controlable) cmd_vel_pub->publish(msg);
                }

                void position_target_callback(const robot_messages::msg::TargetPosition::SharedPtr msg){
                        x_target = msg->x_target;
                        y_target = msg->y_target;
                        theta_target = msg->theta_target;
                }

                double ramp(double current, double target, double max_step)
                {
                        double diff = target - current;

                        if (diff > max_step) diff = max_step;
                        if (diff < -max_step) diff = -max_step;

                        return current + diff;
                }

                void joint_states_callback(const sensor_msgs::msg::JointState::SharedPtr msg){
                        
                        // Position (rad) - values[0]
                        double lf_pos = msg->position[0];
                        double rf_pos = msg->position[1];
                        double rb_pos = msg->position[2];
                        double lb_pos = msg->position[3];

                        // Velocity (rad/s) - values[1]
                        double lf_vel = msg->velocity[0];
                        double rf_vel = msg->velocity[1];
                        double rb_vel = msg->velocity[2];
                        double lb_vel = msg->velocity[3];

                        // Temps écoulé depuis le dernier appel
                        auto now = this->now();
                        double dt = (now - last_time_).seconds();
                        last_time_ = now;

                        if (dt <= 0.0 || dt > 1.0) return;

                        // Forward kinematics omni 4 roues
                        // vx, vy en m/s dans le repère base_link
                        double vx    = (-lf_vel - rf_vel + rb_vel + lb_vel) * wheel_radius / 4.0;
                        double vy = (lf_vel - rf_vel - rb_vel + lb_vel) * wheel_radius / 4.0;
                        double omega = (lf_vel + rf_vel + rb_vel + lb_vel) * wheel_radius / (4.0 * robot_radius);

                        // Intégration dans le repère world (odom)
                        double delta_x     = (vx * cos(theta_) - vy * sin(theta_)) * dt;
                        double delta_y     = -(vx * sin(theta_) + vy * cos(theta_)) * dt;
                        double delta_theta = omega * dt;

                        x_     += delta_x;
                        y_     += delta_y;
                        theta_ += delta_theta;                        

                        double x_error = x_target - x_;
                        double y_error = y_target - y_;
                        double theta_error = theta_ - theta_target;

                        double x_error_robot = -x_error * cos(theta_) + y_error * sin(theta_);
                        double y_error_robot = -x_error * sin(theta_) - y_error * cos(theta_);

                        RCLCPP_INFO(this->get_logger(), "x=%f y=%f ; x_target=%f y_target=%f theta_target=%f", x_, y_, x_target, y_target, theta_target);

                        auto msg_input = geometry_msgs::msg::TwistStamped();

                        msg_input.header.stamp = this->now();
                        msg_input.header.frame_id = "base_link";

                        double target_vx = std::clamp(Kp * y_error_robot, -speed, speed);
                        double target_vy = std::clamp(Kp * x_error_robot, -speed, speed);
                        double target_wz = std::clamp(Kp * 0.2 * theta_error, -speed, speed);

                        vx = ramp(vx, target_vx, accel);
                        vy = ramp(vy, target_vy, accel);
                        wz = ramp(wz, target_wz, accel);

                        msg_input.twist.linear.x  = vx;
                        msg_input.twist.linear.y  = vy;
                        msg_input.twist.angular.z = wz;

                        if (!controlable) cmd_vel_pub->publish(msg_input);
                }

                rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub;
                rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_states_sub;
                rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_vel_pub;
                rclcpp::Subscription<robot_messages::msg::TargetPosition>::SharedPtr position_sub;

                double x_ = 0.0, y_ = 0.0, theta_ = 0.0;
                rclcpp::Time last_time_;

                double vx = 0.0;
                double vy = 0.0;
                double wz = 0.0;


                double x_target;
                double y_target;
                double theta_target;

                double speed;
                double accel;
                double Kp;
                double wheel_radius;
                double robot_radius;
                bool controlable;
};


int main(int argc, char * argv[]){
        rclcpp::init(argc, argv);
        rclcpp::spin(std::make_shared<CarController>());
        rclcpp::shutdown();
        return 0;
}
