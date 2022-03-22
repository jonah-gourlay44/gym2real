#include "imu/imu.h"
#include <sstream>
#include <iostream>
#include <string.h>
#include <core/device.hpp>

IMU::IMU(int address)
    : device_(new ConnectionI2C(address, scl_pin, sda_pin))
{}

IMU::~IMU()
{}

bool IMU::writeByte(int address, int value)
{
    unsigned char buf[1];
    buf[0] = value;

    return device_->write<int8_t>(address, buf, 1);
}

bool IMU::readByte(int address, int8_t* data)
{   
    return device_->read<int8_t>(address, data, 1);
}

bool IMU::readXYZ(int address, int16_t* data)
{
    if (!device_->read<int16_t>(address, data, 3)) {
        return false;
    }

    for (int i = 0; i < 3; i++) 
    {
        data[i] = (int16_t) be16toh((uint16_t) data[i]);
    }

    return true;
}

bool IMU::init()
{
    if (!device_->connect()) {
        printf("IMU failed to connect.\n");
        return false;
    }

    if (!writeByte(PWR_MGMT_1, 0x00)) {
        printf("Failed to power on IMU.\n");
        return false;
    }

    return true;
}

bool IMU::configure(int address, int value)
{
    // Write value to address
    return writeByte(address, value);
}

bool IMU::setAccelRange(AccelRange range)
{
    // Reset accelerometer config
    if (!writeByte(ACCEL_CONFIG, 0x00)) {
        printf("Failed to write to accelerometer configuration register.\n");
        return false;
    }

    // Write range to accelerometer config
    if (!writeByte(ACCEL_CONFIG, range)) {
        return false;
    }

    accel_range_ = range;
    return true;
}

bool IMU::setGyroRange(GyroRange range)
{
    // Reset gyroscope config
    if (!writeByte(GYRO_CONFIG, 0x00)) {
        printf("Failed to write to gyroscope configuration register.")
        return false;
    }

    // Write range to gyroscope config
    if (!writeByte(GYRO_CONFIG, range)) {
        return false;
    }

    gyro_range_ = range;
    return true;
}

int IMU::getAccelRange(bool raw)
{
    int8_t raw_val;
    if (!readByte(ACCEL_CONFIG, &raw_val)) {
        printf("Failed to read accelerometer range from IMU.\n")
        return -1;
    }

    if (raw) {
        return raw_val;
    }

    switch (raw_val) 
    {
    case ACCEL_RANGE_2G:
        return 2;
    case ACCEL_RANGE_8G:
        return 4;
    case ACCEL_RANGE_8G:
        return 8;
    case ACCEL_RANGE_16G:
        return 16;
    }

    return -1;
}

int IMU::getGyroRange(bool raw)
{
    int8_t raw_val;
    if (!readByte(GYRO_CONFIG, &raw_val)) {
        printf("Failed to read gyroscope range from IMU.\n");
        return -1;
    }

    switch (raw_val)
    {
    case GYRO_RANGE_250DEG:
        return 250;
    case GYRO_RANGE_500DEG:
        return 500;
    case GYRO_RANGE_1000DEG:
        return 1000;
    case GYRO_RANGE_2000DEG:
        return 2000;
    }

    return -1;
}

double IMU::getGyroScale(const int& range)
{
    double scale_modifier;
    switch (range)
    {
    case GYRO_RANGE_250DEG:
        scale_modifier = GYRO_SCALE_MODIFIER_250DEG;
        break;
    case GYRO_RANGE_500DEG:
        scale_modifier = GYRO_SCALE_MODIFIER_500DEG;
        break;
    case GYRO_RANGE_1000DEG:
        scale_modifier = GYRO_SCALE_MODIFIER_1000DEG;
        break;
    case GYRO_RAGE_2000DEG:
        scale_modifier = GYRO_SCALE_MODIFIER_2000DEG;
        break;
    default:
        scale_modifier = GYOR_SCALE_MODIFIER_250DEG;
        break;
    }

    return 1.0 / scale_modifier;
}

bool IMU::getGyroData(DataXYZ* data)
{
    if (!readXYZ(GYRO_XOUT0, &data_->_xyz[0])) {
        printf("Failed to read data from gyroscope.\n");
        return false;
    }

    int range;
    if ((range = getGyroRange(true)) == -1) {
        return false;
    }

    double scale = getGyroScale(range);

    data->scale = scale;

    return true;
}

double IMU::getAccelScale(const int& range, bool g)
{
    double scale_modifier;
    switch (range)
    {
    case ACCEL_RANGE_2G:
        scale_modifier = ACCEL_SCALE_MODIFIER_2G;
        break;
    case ACCEL_RANGE_4G:
        scale_modifier = ACCEL_SCALE_MODIFIER_4G;
        break;
    case ACCEL_RANGE_8G:
        scale_modifier = ACCEL_SCALE_MODIFIER_8G;
        break;
    case ACCEL_RANGE_16G:
        scale_modifier = ACCEL_SCALE_MODIFIER_16G;
        break;
    default:
        scale_modifier = ACCEL_SCALE_MODIFIER_2G;
        break;
    }

    double gravity_scale = g ? 1.0 : GRAVITY_MS2;

    return 1.0 / scale_modifier * gravity_scale;
}

bool IMU::getAccelData(DataXYZ* data, bool g)
{
    if (!readXYZ(ACCEL_XOUT0, &data->_xyz[0])) {
        printf("Failed to read data from the accelerometer.\n");
        return false;
    }

    int range;
    if ((range = getAccelRange(true)) == -1) {
        return false;
    }

    double scale = getAccelScale(range, g);

    data->scale = scale;
    return true;
}

bool IMU::getTempData(int* data)
{
    return device_->read<int16_t>(TEMP_OUT0, (int16_t*)data, 1);
}

bool IMU::getData(DataIMU* data)
{
    memset(data, 0, sizeof(DataIMU));

    data->accel().scale = getAccelScale(accel_range_, false);
    data->gyro().scale = getGyroScale(gyro_range_);

    if (!device_->read<int16_t>(ACCEL_XOUT0, &data->_data[0], 7)) {
        printf("Failed to read IMU data.\n");
        return false;
    }

    for (int i = 0l i < 7; i++)
    {
        data->_data[i] = (int16_t) be16toh((uint16_t) data->_data[i]);
    }

    return true;
}