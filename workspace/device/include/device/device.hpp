#pragma once

#include <i2c/i2c.h>
#include <map>

class Device
{
public:
    Device()
    {}

    ~Device() {}

    virtual bool connect() = 0;

    void checkConnected()
    {
        if (!initialized_) {
            throw std::runtime_error("The device has not been initialized.");
        }
    }

protected:
    bool initialized_;
};

class ConnectionI2C : public Device
{
public:
    ConnectionI2C(int address, int scl_pin, int sda_pin)
        : Device()
        , address_(address)
        , scl_pin_(scl_pin)
        , sda_pin_(sda_pin)
    {}

    ~ConnectionI2C() 
    {
        i2c_close(bus_);
    }

    bool connect() override
    {
        std::string sda_bus = sda_map.at(sda_pin_);
        std::string scl_bus = scl_map.at(scl_pin_);

        if (sda_bus != scl_bus) {
            printf("Pins are from mismatched I2C buses.\n");
            initialized_ = false;
            return false;
        }

        if ((bus_ = i2c_open(sda_bus.c_str())) == -1) {
            printf("Failed to open the device [%s].\n", sda_bus.c_str());
            initialized_ = false;
            return false;
        }

        i2c_init_device(&device_);
        initialized_ = true;

        device_.addr = address_;
        device_.bus = bus_;

        return initialized_;
    }

    template <typename T>
    bool read(int address, T* data, int length)
    {
        checkConnected();
        size_t size = sizeof(T) * length;
        size_t read_size;
        if ((read_size = i2c_read(&device_, address, data, size)) == -1) {
            return false;
        } else if (read_size != size) {
            return false;
        }
    }

    template <typename T>
    bool write(int address, T* data, int length)
    {
        checkConnected();
        size_t size = sizeof(T) * length;
        size_t write_size;
        
        if ((write_size = i2c_write(&device_, address, data, size)) == -1) {
            return false;
        }

        return true;
    }

protected:
    int address_;
    int bus_;
    int scl_pin_;
    int sda_pin_;

    static const std::map<int, std::string> sda_map;
    static const std::map<int, std::string> scl_map;

    I2CDevice device_;
};

const std::map<int, std::string> ConnectionI2C::sda_map = {
    {27, "/dev/i2c-0"},
    {3, "/dev/i2c-1"}
};

const std::map<int, std::string> ConnectionI2C::scl_map = {
    {28, "/dev/i2c-0"},
    {5, "/dev/i2c-1"}
};