#pragma once

#include <array>
#include <atomic>
#include <chrono>
#include <memory>
#include <mutex>
#include <shared_mutex>
#include <string>
#include <thread>

#include "serial_port.h"
#include "imu_parser.h"

namespace IMU
{

struct ImuSnapshot
{
    std::array<double, 3> accel{};
    std::array<double, 3> gyro{};
    std::array<double, 3> euler{};
    std::array<double, 4> quat{};
    std::array<double, 3> projected_gravity{};
    bool valid{false};
};

class Imu
{
public:
    Imu(std::string device = "/dev/ttyACM0", int baudrate = 921600);
    ~Imu();

    Imu(const Imu&) = delete;
    Imu& operator=(const Imu&) = delete;

    bool start();
    void stop();

    ImuSnapshot get_snapshot() const;

    void get_acceleration(double* acc) const;
    void get_angular_velocity(double* angvel) const;
    void get_projected_gravity(double* gravity) const;

    bool is_connected() const { return connected_.load(std::memory_order_acquire); }

private:
    void io_loop();

    std::string device_;
    int baudrate_;

    SerialPort sp_;
    IMUParser parser_;

    mutable std::shared_mutex data_mutex_;
    ImuSnapshot snapshot_;

    std::atomic<bool> running_{false};
    std::atomic<bool> connected_{false};
    std::thread io_thread_;

};

} // namespace IMU
