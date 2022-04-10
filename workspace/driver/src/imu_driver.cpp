#include "driver/imu_driver.hpp"

#include <JetsonGPIO.h>
#include <device/imu.h>

#include <boost/bind.hpp>
#include <boost/function.hpp>

#include <sched.h>

#define BETA_IMU 0.75l // Change to IMU gyro error
#define SAMPLE_RATE 1e3l
#define DEG_TO_RAD 0.01745329251l

class Callback
{
public:
    typedef boost::function<void(const std::string &)> Handle;

    Callback(Handle fxn, const std::string &name)
        : fxn_(fxn), name_(name)
    {
    }

    Callback(const Callback &) = default;

    void operator()(const std::string &channel)
    {
        fxn_(channel);
    }

    bool operator==(const Callback &other) const
    {
        return name_ == other.name_;
    }

private:
    boost::function<void(const std::string &)> fxn_;
    std::string name_;
};

IMUDriver::IMUDriver(int interrupt_pin, int sda_pin, int scl_pin, int address)
    : Node("imu_driver"), imu_(new IMU(address, sda_pin, scl_pin)), data_(new DataIMU()), beta_(BETA_IMU), sample_freq_(SAMPLE_RATE)
{
    publisher_ = create_publisher<sensor_msgs::msg::Imu>("imu_data", 10);
    cb_ = new Callback(boost::bind(&IMUDriver::data_callback, this, _1), "data_callback");
    imu_->init();
    imu_->configure(INT_ENABLE, 0x01);
    imu_->configure(SMPLRT_DIV, 0x07);
    imu_->configure(INT_PIN_CFG, 0x00);
    imu_->setGyroRange(IMU::DEG250);
    imu_->setAccelRange(IMU::G2);

    q_.w = 1.0;
    q_.x = 0.0;
    q_.y = 0.0;
    q_.z = 0.0;

    GPIO::setmode(GPIO::BOARD);
    GPIO::setup(interrupt_pin, GPIO::IN);

    GPIO::add_event_detect(interrupt_pin, GPIO::RISING, *cb_, 1);
}

IMUDriver::~IMUDriver()
{
    delete data_;
    GPIO::cleanup();
}

void IMUDriver::data_callback(const std::string &channel)
{
    imu_->getData(data_);

    sensor_msgs::msg::Imu msg;
    msg.angular_velocity.x = data_->gyro().x() * DEG_TO_RAD;
    msg.angular_velocity.y = data_->gyro().y() * DEG_TO_RAD;
    msg.angular_velocity.z = data_->gyro().z() * DEG_TO_RAD;
    msg.linear_acceleration.x = data_->accel().x();
    msg.linear_acceleration.y = data_->accel().y();
    msg.linear_acceleration.z = data_->accel().z();

    madgwickAHRS();

    msg.header.stamp = get_clock()->now();

    msg.orientation = q_;

    publisher_->publish(msg);
}

// https://x-io.co.uk/open-source-imu-and-ahrs-algorithms/
void IMUDriver::madgwickAHRS()
{
    double gx = data_->gyro().x() * DEG_TO_RAD;
    double gy = data_->gyro().y() * DEG_TO_RAD;
    double gz = data_->gyro().z() * DEG_TO_RAD;
    double ax = data_->accel().x();
    double ay = data_->accel().y();
    double az = data_->accel().z();
    double _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2, _8q1, _8q2, q0q0, q1q1, q2q2, q3q3;
    double s0, s1, s2, s3;
    double recip_norm;
    Quaternion qDot;

    // Quaternion derivative using gyro data
    qDot.w = 0.5 * (-q_.x * gx - q_.y * gy - q_.z * gz);
    qDot.x = 0.5 * (q_.w * gx + q_.y * gz - q_.z * gy);
    qDot.y = 0.5 * (q_.w * gy - q_.x * gz + q_.z * gx);
    qDot.z = 0.5 * (q_.w * gz + q_.x * gy - q_.y * gx);

    // http://en.wikipedia.org/wiki/Fast_inverse_square_root
    auto invSqrt = [](const double &x)
    {
        double halfx = 0.5 * x;
        double y = x;
        long i = *(long *)&y;
        i = 0x5fe6eb50c7b537a9 - (i >> 1); // https://stackoverflow.com/questions/11644441/fast-inverse-square-root-on-x64
        y = *(double *)&i;
        y = y * (1.5 - (halfx * y * y));
        return y;
    };

    if (!((ax == 0.0) && (ay == 0.0) && (az == 0.0)))
    {

        // Normalize accelerometer data
        recip_norm = invSqrt(ax * ax + ay * ay + az * az);
        ax *= recip_norm;
        ay *= recip_norm;
        az *= recip_norm;

        // Constants
        _2q0 = 2.0 * q_.w;
        _2q1 = 2.0 * q_.x;
        _2q2 = 2.0 * q_.y;
        _2q3 = 2.0 * q_.z;
        _4q0 = 4.0 * q_.w;
        _4q1 = 4.0 * q_.x;
        _4q2 = 4.0 * q_.y;
        _8q1 = 8.0 * q_.x;
        _8q2 = 8.0 * q_.y;
        q0q0 = q_.w * q_.w;
        q1q1 = q_.x * q_.x;
        q2q2 = q_.y * q_.y;
        q3q3 = q_.z * q_.z;

        // Gradient decent
        s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
        s1 = _4q1 * q3q3 - _2q3 * ax + 4.0l * q0q0 * q_.x - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
        s2 = 4.0l * q0q0 * q_.y + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
        s3 = 4.0l * q1q1 * q_.z - _2q1 * ax + 4.0l * q2q2 * q_.z - _2q2 * ay;
        recip_norm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3);
        s0 *= recip_norm;
        s1 *= recip_norm;
        s2 *= recip_norm;
        s3 *= recip_norm;

        // Apply feedback
        qDot.w -= beta_ * s0;
        qDot.x -= beta_ * s1;
        qDot.y -= beta_ * s2;
        qDot.z -= beta_ * s3;
    }

    // Integrate quaternion derivative to get update
    q_.w += qDot.w * (1.0 / sample_freq_);
    q_.x += qDot.x * (1.0 / sample_freq_);
    q_.y += qDot.y * (1.0 / sample_freq_);
    q_.z += qDot.z * (1.0 / sample_freq_);

    // Normalize final quaternion
    recip_norm = invSqrt(q_.w * q_.w + q_.x * q_.x + q_.y * q_.y + q_.z * q_.z);
    q_.w *= recip_norm;
    q_.x *= recip_norm;
    q_.y *= recip_norm;
    q_.z *= recip_norm;
}