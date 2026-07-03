#pragma once

#include <atomic>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>
#include <chrono>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/int32.hpp>

#include "Types_.h"
#include "gamepad/gamepad.h"
#include "imu.h"
#include "motor.h"

#ifdef CONFIG_PATH
#define SENSOR_CONFIG CONFIG_PATH "/sensor.yaml"
#else
#define SENSOR_CONFIG ""
#endif


class Real
{
public:
    Real(std::string node_name = "real_node");
    ~Real();

    Real(const Real&) = delete;
    Real& operator=(const Real&) = delete;

    void start();

private:
    void LoadConfig();
    void PubCallback();
    void ControlCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg);
    void ButtonStateCallback(const std_msgs::msg::Int32::SharedPtr msg);
    void reset();

    std::string config_path_{SENSOR_CONFIG};

    std::unique_ptr<IMU::Imu> imu_;
    std::vector<std::shared_ptr<MotorDriver>> motor_;
    std::unique_ptr<GAMEPAD::Gamepad> gamepad_;

    rclcpp::Node::SharedPtr node_;

    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr subscriber_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr button_state_sub_;
    rclcpp::TimerBase::SharedPtr timer_;

    int obs_dim_{0};
    int num_dof_{0};
    double frequency_{50.0};

    std::vector<MOTOR::LroMotor_75_8462::Config> motor_configs_;
    std::vector<float> pos_limits_low_;
    std::vector<float> pos_limits_high_;
    std::vector<float> kp_;
    std::vector<float> kd_;

    std::string imu_device_;
    int imu_baudrate_{921600};

    GAMEPAD::GamepadMapping gamepad_mapping_;

    std::atomic<bool> running_{true};
};
