#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <JetsonGPIO.h>

class MotorDriver : public rclcpp::Node
{
public:
    MotorDriver(int pwm_motor_l, int pwm_motor_r, int encoder_l_a, int encoder_l_b, int encoder_r_a, int encoder_r_b);

    ~MotorDriver();

private:
    void command_callback(const sensor_msgs::msg::JointState::SharedPtr msg);
    void control_loop();
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_error_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_state_;

    sensor_msgs::msg::JointState::SharedPtr command_;
    sensor_msgs::msg::JointState rpm_error_;
    sensor_msgs::msg::JointState motor_state_;

    int encoder_l_pin_a_;
    int encoder_r_pin_a_;
    int encoder_l_pin_b_;
    int encoder_r_pin_b_;
    volatile int encoder_l_count_ = 0;
    volatile int encoder_r_count_ = 0;
    volatile bool read_a_l_ = false;
    volatile bool read_b_l_ = false;
    volatile bool read_a_r_ = false;
    volatile bool read_b_r_ = false;

    std::unique_ptr<GPIO::PWM> pwm_l_;
    std::unique_ptr<GPIO::PWM> pwm_r_;
    std::chrono::time_point<std::chrono::high_resolution_clock> last_time_;
    rclcpp::TimerBase::SharedPtr control_loop_timer_;

    class CounterCallback
    {
    public:
        CounterCallback(int &pin, bool a_or_b, volatile int &counter, volatile bool &read_a, volatile bool &read_b, bool one_channel=false) : counter_(counter), pin_(pin), a_or_b_(a_or_b), read_a_(read_a), read_b_(read_b), one_channel_(one_channel) {}
        CounterCallback(const CounterCallback &) = default; // Copy-constructible

        void operator()(const std::string &channel) // Callable
        {
            if (one_channel_){
                counter_++;
            } else{
                if (a_or_b_)
                {
                    read_b_ = GPIO::input(pin_);
                    counter_ += (read_a_ == read_b_) ? +1 : -1;
                }
                else
                {
                    read_a_ = GPIO::input(pin_);
                    counter_ += (read_a_ != read_b_) ? +1 : -1;
                }
            }
        }

        bool operator==(const CounterCallback &other) const // Equality-comparable
        {
            return counter_ == other.counter_ & pin_ == other.pin_ & a_or_b_ == other.a_or_b_;
        }

    private:
        volatile int &counter_;
        int &pin_;
        bool a_or_b_;
        volatile bool &read_a_;
        volatile bool &read_b_;
        bool one_channel_;
    };

    std::unique_ptr<MotorDriver::CounterCallback> cb_l_a_, cb_l_b_, cb_r_a_, cb_r_b_;

    struct PID
    {
        float Kp = 0.1;
        float Ki = 0.;
        float Kd = 0.;
        float last_error = 0.;
        float sum_error = 0.;

        float send(float error, float dt)
        {
            float rate_error = error - last_error;
            sum_error += error * dt;
            float pid_out = Kp * error + Ki * sum_error + Kd * rate_error/dt;
            last_error = error;
            return pid_out;
        }
    };

    PID pid_l_;
    PID pid_r_;
};