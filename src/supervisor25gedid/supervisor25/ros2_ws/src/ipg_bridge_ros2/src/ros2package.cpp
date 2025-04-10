#include <string>
#include "vehiclecontrol_msgs/msg/vehicle_control.hpp"
#include "PID.h"
#include "nav_msgs/msg/odometry.hpp"
 
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
 
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1; // Create a placeholder for the first argument of the function

#define PID_KP 0.5f
#define PID_KI 0.05f
#define PID_KD 0.015f
#define PID_TAU 0.02f
#define PID_LIM_MIN -1.0f
#define PID_LIM_MAX 1.0f
#define PID_LIM_MIN_INT -5.0f
#define PID_LIM_MAX_INT 5.0f
#define SAMPLE_TIME_S 0.1f

/* This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer. */

class MinimalPublisher : public rclcpp::Node
{
public:
    PIDController pid = {PID_KP, PID_KI, PID_KD,
                         PID_TAU,
                         PID_LIM_MIN, PID_LIM_MAX,
                         PID_LIM_MIN_INT, PID_LIM_MAX_INT,
                         SAMPLE_TIME_S};

    float Throttle = 0;
    float Brake = 0;
    float velocity_setpoint = 0.0f;
    float desired_steering_angle = 0;
    float current_velocity = 0.0f;
    vehiclecontrol_msgs::msg::VehicleControl Control_msg;
    MinimalPublisher()
        : Node("minimal_publisher"), count_(0)
    {
        publisher_ = this->create_publisher<vehiclecontrol_msgs::msg::VehicleControl>("/carmaker/VehicleControl", 10);
        timer_ = this->create_wall_timer(
            10ms, std::bind(&MinimalPublisher::timer_callback, this));
        subscription_ = this->create_subscription<ackermann_msgs::msg::AckermannDriveStamped>(
            "ackr", 10, std::bind(&MinimalPublisher::Bridge_CB, this, _1));
        subscription_odom_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/carmaker/Odometry", 10, std::bind(&MinimalPublisher::Bridge_odom_CB, this, _1));
    }

private:
    void Bridge_CB(const ackermann_msgs::msg::AckermannDriveStamped &msg)
    {
        velocity_setpoint = msg.drive.speed;
        desired_steering_angle = msg.drive.steering_angle;
    }
    void Bridge_odom_CB(const nav_msgs::msg::Odometry &msg)
    {
        current_velocity = sqrt(pow(msg.twist.twist.linear.x, 2) + pow(msg.twist.twist.linear.y, 2));
    }
    void timer_callback()
    {
        Control_msg.use_vc = 1;
        PIDController_Update(&pid, velocity_setpoint, current_velocity);
        if (pid.out > 0)
        {
            Brake = 0;
            Throttle = pid.out;
            Control_msg.use_vc = 1;
            Control_msg.gas = Throttle;
            Control_msg.brake = Brake;
        }
        else
        {
            Throttle = 0;
            Brake = -1 * pid.out;
            Control_msg.use_vc = 1;
            Control_msg.gas = Throttle;
            Control_msg.brake = Brake;
        }
        Control_msg.steer_ang = desired_steering_angle * 6.66666666;
        publisher_->publish(Control_msg);
        //RCLCPP_INFO(this->get_logger(), "Publishing Control");
        velocity_setpoint=0;
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<vehiclecontrol_msgs::msg::VehicleControl>::SharedPtr publisher_;
    rclcpp::Subscription<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr subscription_; // The subscriber object.
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_odom_; // The subscriber object.

    size_t count_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MinimalPublisher>());
    rclcpp::shutdown();
    return 0;
}
