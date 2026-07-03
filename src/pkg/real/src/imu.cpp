#include "real/imu.h"

#include <Eigen/Dense>
#include <cstdio>
#include <cstring>

namespace IMU
{

Imu::Imu(std::string device, int baudrate)
    : device_(std::move(device)), baudrate_(baudrate)
{
}

Imu::~Imu()
{
    stop();
}

bool Imu::start()
{
    if (running_.load(std::memory_order_acquire))
        return true;

    if (!sp_.open(device_, baudrate_)) {
        fprintf(stderr, "[IMU] Failed to open %s at %d baud\n",
                device_.c_str(), baudrate_);
        connected_.store(false, std::memory_order_release);
        return false;
    }

    connected_.store(true, std::memory_order_release);
    fprintf(stdout, "[IMU] Opened %s at %d baud\n", device_.c_str(), baudrate_);

    running_.store(true, std::memory_order_release);
    io_thread_ = std::thread(&Imu::io_loop, this);
    return true;
}

void Imu::stop()
{
    if (!running_.load(std::memory_order_acquire))
        return;

    running_.store(false, std::memory_order_release);
    if (io_thread_.joinable())
        io_thread_.join();

    sp_.close();
    connected_.store(false, std::memory_order_release);
    fprintf(stdout, "[IMU] Stopped\n");
}


ImuSnapshot Imu::get_snapshot() const
{
    std::shared_lock lock(data_mutex_);
    return snapshot_;
}

void Imu::get_acceleration(double* acc) const
{
    std::shared_lock lock(data_mutex_);
    acc[0] = snapshot_.accel[0];
    acc[1] = snapshot_.accel[1];
    acc[2] = snapshot_.accel[2];
}

void Imu::get_angular_velocity(double* angvel) const
{
    std::shared_lock lock(data_mutex_);
    angvel[0] = snapshot_.gyro[0];
    angvel[1] = snapshot_.gyro[1];
    angvel[2] = snapshot_.gyro[2];
}

void Imu::get_projected_gravity(double* gravity) const
{
    std::shared_lock lock(data_mutex_);
    gravity[0] = snapshot_.projected_gravity[0];
    gravity[1] = snapshot_.projected_gravity[1];
    gravity[2] = snapshot_.projected_gravity[2];
}

void Imu::io_loop()
{
    uint8_t buf[4096];
    fprintf(stdout, "[IMU] Reading data...\n");

    while (running_.load(std::memory_order_acquire)) {
        int n = sp_.read(buf, sizeof(buf));
        if (n < 0) {
            fprintf(stderr, "[IMU] Serial read error\n");
            connected_.store(false, std::memory_order_release);
            break;
        }
        if (n > 0) {
            parser_.feed(buf, n);

            const IMUData* raw = parser_.getIMUData();
            {
                std::unique_lock lock(data_mutex_);
                snapshot_.accel = {raw->accel.x, raw->accel.y, raw->accel.z};
                snapshot_.gyro  = {raw->gyro.x,  raw->gyro.y,  raw->gyro.z};
                snapshot_.euler = {raw->euler.x, raw->euler.y, raw->euler.z};
                snapshot_.quat  = {raw->quat.w,  raw->quat.x,  raw->quat.y,  raw->quat.z};

                Eigen::Quaterniond q(raw->quat.w, raw->quat.x, raw->quat.y, raw->quat.z);
                Eigen::Vector3d g = q.conjugate() * Eigen::Vector3d(0.0, 0.0, -1.0);
                snapshot_.projected_gravity = {g.x(), g.y(), g.z()};
                snapshot_.valid = true;
            }
        }
    }

    sp_.close();
    connected_.store(false, std::memory_order_release);
    fprintf(stdout, "[IMU] I/O loop exited\n");
}

} // namespace IMU
