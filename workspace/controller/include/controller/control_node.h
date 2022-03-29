#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/empty.hpp>
#include <onnxruntime_cxx_api.h>
#include <boost/signals2.hpp>

#include <thread>
#include <mutex>

class OnnxController;

class ControlNode : public rclcpp::Node
{
public:
    enum State {
        IDLE,
        KICKUP,
        KICKUP_WAIT,
        BALANCE
    };

    ControlNode();
    ~ControlNode();

private:
    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg);
    void motor_callback(const sensor_msgs::msg::JointState::SharedPtr msg);
    void start_callback(const std_msgs::msg::Empty::SharedPtr msg);
    void end_callback(const std_msgs::msg::Empty::SharedPtr msg);

    void control_loop();
    void kickup();

    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr motor_sub_;
    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr start_sub_;
    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr end_sub_;

    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr motor_pub_;

    sensor_msgs::msg::Imu::SharedPtr imu_state_;
    sensor_msgs::msg::JointState::SharedPtr motor_state_;
    sensor_msgs::msg::JointState command_;

    rclcpp::TimerBase::SharedPtr control_loop_timer_;

    boost::signals2::signal<void()> kickup_signal_;
    boost::signals2::signal<void()> control_step_signal_;

    std::unique_ptr<OnnxController> controller_;

    State state_;
    mutable std::mutex m_;
};