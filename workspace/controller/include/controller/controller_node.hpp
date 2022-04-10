#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <controller/memory_manager.hpp>

// Example controller node for running TWIP model
class ControllerNode : public rclcpp::Node
{
public:
    ControllerNode();
    
    ~ControllerNode();

private:
    void control_loop();

    // ROS2 interface
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_motor_state_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_;
    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg);
    void motor_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg);
    void motor_command_publish();

    // Msgs/buffers
    sensor_msgs::msg::Imu::SharedPtr imu_state_;
    float * imu_quaternion_ = nullptr;
    sensor_msgs::msg::JointState::SharedPtr motor_state_;
    float * motor_position_ = nullptr;
    sensor_msgs::msg::JointState command_;
    float * motor_command_ = nullptr;

    std::chrono::time_point<std::chrono::high_resolution_clock> last_time_;
    rclcpp::TimerBase::SharedPtr control_loop_timer_;

    MemoryManager memory_manager_;
    std::shared_ptr<BaseController> model_controller_;
};
