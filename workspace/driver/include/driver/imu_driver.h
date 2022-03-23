#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include <chrono>

class IMU;
class Callback;
struct DataIMU;

class IMUDriver : public rclcpp::Node
{
public:
    IMUDriver(int interrupt_pin, int sda_pin, int scl_pin, int address);

    ~IMUDriver();

private:
    typedef geometry_msgs::msg::Quaternion Quaternion;

    void data_callback(const std::string &channel);
    void madgwickAHRS();

    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr publisher_;
    std::shared_ptr<IMU> imu_;

    DataIMU *data_;
    Callback *cb_;
    Quaternion q_;
    double beta_;
    double sample_freq_;
};