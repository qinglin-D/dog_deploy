#include "real/real.h"

#include <yaml-cpp/yaml.h>
#include <algorithm>
#include <cassert>
#include <cmath>
#include <iostream>
#include <cstring>
#include <csignal>
#include <filesystem>
#include <fstream>
#include <dirent.h>
#include <sys/types.h>

Real::Real(std::string node_name)
{
    node_ = rclcpp::Node::make_shared(node_name);
    LoadConfig();

    publisher_ = node_->create_publisher<std_msgs::msg::Float64MultiArray>("observation", 10);
    subscriber_ = node_->create_subscription<std_msgs::msg::Float32MultiArray>(
        "actions", 10,
        std::bind(&Real::ControlCallback, this, std::placeholders::_1));

    button_state_sub_ = node_->create_subscription<std_msgs::msg::Int32>(
        "button_state", 10,
        std::bind(&Real::ButtonStateCallback, this, std::placeholders::_1));

    RCLCPP_INFO(node_->get_logger(), "Initializing real node (obs_dim=%d, dof=%d, freq=%.1f Hz)",
        obs_dim_, num_dof_, frequency_);

    imu_ = std::make_unique<IMU::Imu>(imu_device_, imu_baudrate_);

    gamepad_ = std::make_unique<GAMEPAD::Gamepad>(gamepad_mapping_);
    gamepad_->make_subscription(node_);

    for (const auto& item : motor_configs_)
    {
        motor_.push_back(MotorDriver::create_motor(item.motor_id, item.interface_type, 
                                                    item.can_interface, "LROCAN", 2));
        int err = motor_.back()->init_motor();
        if (err != 0) {
            std::string msg = "init_motor failed, error_id: " + std::to_string(err);
            write_log(msg.c_str(), "ERROR");
        }
    }
    

    auto period = std::chrono::milliseconds(static_cast<int>(1000.0 / frequency_));
    timer_ = node_->create_wall_timer(period, std::bind(&Real::PubCallback, this));

}


Real::~Real()
{
    running_.store(false, std::memory_order_release);
}


void Real::LoadConfig()
{
    try {
        YAML::Node config = YAML::LoadFile(config_path_);

        obs_dim_     = config["sensorDataSize"].as<int>();
        num_dof_     = config["actionsSize"].as<int>();
        frequency_   = config["frequency"].as<double>();

        imu_device_   = config["imu"]["port"].as<std::string>();
        imu_baudrate_ = config["imu"]["baudrate"].as<int>();

        gamepad_mapping_.axis_x = config["gamepad"]["axis_x"].as<int>();
        gamepad_mapping_.axis_y = config["gamepad"]["axis_y"].as<int>();
        gamepad_mapping_.axis_yaw = config["gamepad"]["axis_yaw"].as<int>();
        gamepad_mapping_.axis_trigger = config["gamepad"]["axis_trigger"].as<int>();
        gamepad_mapping_.button_go_on = config["gamepad"]["button_go_on"].as<int>();
        gamepad_mapping_.button_pause = config["gamepad"]["button_pause"].as<int>();
        gamepad_mapping_.button_reset = config["gamepad"]["button_reset"].as<int>();
        gamepad_mapping_.button_terminate = config["gamepad"]["button_terminate"].as<int>();
        gamepad_mapping_.button_policy_1 = config["gamepad"]["button_policy_1"].as<int>();
        gamepad_mapping_.button_policy_2 = config["gamepad"]["button_policy_2"].as<int>();
        gamepad_mapping_.button_policy_3 = config["gamepad"]["button_policy_3"].as<int>();

        std::vector<std::string> joint_names =
            config["actuator"]["control_order"].as<std::vector<std::string>>();

        motor_configs_.clear();
        motor_configs_.reserve(joint_names.size());
        kp_.reserve(joint_names.size());
        kd_.reserve(joint_names.size());
        pos_limits_low_.reserve(joint_names.size());
        pos_limits_high_.reserve(joint_names.size());

        for (const auto& name : joint_names) {
            MOTOR::LroMotor_75_8462::Config cfg;
            cfg.motor_id      = config["actuator"]["motor_ID"][name].as<uint8_t>();
            cfg.can_interface = config["actuator"]["interface"][name].as<std::string>();
            cfg.interface_type = config["actuator"]["interface_type"].as<std::string>();
            cfg.ctrl_mode      = MotorDriver::MIT;
            motor_configs_.push_back(cfg);

            kp_.push_back(config["actuator"]["stiffness"][name].as<double>());
            kd_.push_back(config["actuator"]["damping"][name].as<double>());
            pos_limits_low_.push_back(config["actuator"]["position_limit"][name][0].as<float>());
            pos_limits_high_.push_back(config["actuator"]["position_limit"][name][1].as<float>());
        }

        std::string msg = "Loaded config: " + std::to_string(motor_configs_.size()) + " motors loaded ";
        write_log(msg.c_str(), "INFO");

    } catch (const YAML::Exception& e) {
        RCLCPP_FATAL(node_->get_logger(), "Failed to load config: %s", e.what());
        write_log("Failed to load config (real package)", "ERROR");
        exit(EXIT_FAILURE);
    }
}

void Real::PubCallback()
{
    if (!running_.load(std::memory_order_acquire)) {
        std::this_thread::sleep_for(std::chrono::milliseconds(int(1000.0 / frequency_)-2));
        return;
    }

    std_msgs::msg::Float64MultiArray msg;
    msg.data.resize(obs_dim_);
    double* data = msg.data.data();

    // raw axes (4): x, y, yaw_stick, trigger
    gamepad_->get_axes(data);
    data += 4;

    // angular velocity (3)
    imu_->get_angular_velocity(data);
    data += 3;

    // projected gravity (3)
    imu_->get_projected_gravity(data);
    std::cout << "imu: " << data[0] << ", " << data[1] << ", " << data[2] << std::endl;
    data += 3;

    // joint positions (num_dof_)
    for(const auto& item : motor_)
    {
        *data = item->get_motor_pos();
        data++;
    }

    // // joint velocities (num_dof_)
    for(const auto& item : motor_)
    {
        *data = item->get_motor_spd();
        data++;
    }

    // for (int i = 0; i < num_dof_*2; i++) {
    //     data[i] = 0.0;
    // }

    publisher_->publish(msg);

}


void Real::ControlCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
{
    if (!running_.load(std::memory_order_acquire)) {
        for (const auto& item : motor_)
        {
            item->motor_mit_cmd(item->get_motor_pos(), 0.0f, 0.1, 0.1, 0.0f);
        }
        return;
    }

    int i = 0;
    for (const auto& item : motor_)
    {
        std::clamp(msg->data[i], pos_limits_low_[i], pos_limits_high_[i]);
        item->motor_mit_cmd(msg->data[i], 0.0f, kp_[i], kd_[i], 0.0f);
        i++;
    }
}


void Real::ButtonStateCallback(const std_msgs::msg::Int32::SharedPtr msg)
{
    auto state = static_cast<GAMEPAD::Gamepad::States>(msg->data);

    switch (state)
    {
    case GAMEPAD::Gamepad::TERMINATE:
        running_.store(false, std::memory_order_release);
        if (!timer_->is_canceled()) {
            timer_->cancel();
        }
        for (auto item : motor_) {
            item->motor_mit_cmd(item->get_motor_pos(), 0.0f, 0.0f, 0.0f, 0.0f);
        }
        rclcpp::shutdown();
        return;

    case GAMEPAD::Gamepad::PAUSE:
        if (running_.load(std::memory_order_acquire)) {
            running_.store(false, std::memory_order_release);
            if (!timer_->is_canceled()) {
                timer_->cancel();
            }
            for (auto item : motor_) {
                item->motor_mit_cmd(item->get_motor_pos(), 0.0f, 0.0f, 0.0f, 0.0f);
            }
        }
        break;

    case GAMEPAD::Gamepad::GO_ON:
        if (!running_.load(std::memory_order_acquire)) {
            running_.store(true, std::memory_order_release);
            if (timer_->is_canceled()) {
                timer_->reset();
            }
        }
        break;

    case GAMEPAD::Gamepad::RESET:
        if (running_.load(std::memory_order_acquire)) {
            running_.store(false, std::memory_order_release);
            if (!timer_->is_canceled()) {
                timer_->cancel();
            }
            for (auto item : motor_) {
                item->motor_mit_cmd(item->get_motor_pos(), 0.0f, 0.0f, 0.0f, 0.0f);
            }
            RCLCPP_INFO(node_->get_logger(), "RESET: releasing motors and timer...");
            reset();
        }
        break;

    default:
        break;
    }
}


void Real::reset()
{
    RCLCPP_INFO(node_->get_logger(), "Releasing motor objects...");
    motor_.clear();
    std::this_thread::sleep_for(std::chrono::milliseconds(5));

    RCLCPP_INFO(node_->get_logger(), "Reinitializing motor objects...");
    for (const auto& item : motor_configs_)
    {
        motor_.push_back(MotorDriver::create_motor(item.motor_id, item.interface_type,
                                                    item.can_interface, "LROCAN", 2));
        int err = motor_.back()->init_motor();
        if (err != 0) {
            std::string msg = "init_motor failed in reset(), error_id: " + std::to_string(err);
            write_log(msg.c_str(), "ERROR");
        }
    }


    RCLCPP_INFO(node_->get_logger(), "Recreating timer...");
    auto period = std::chrono::milliseconds(static_cast<int>(1000.0 / frequency_));
    timer_ = node_->create_wall_timer(period, std::bind(&Real::PubCallback, this));
    running_.store(true, std::memory_order_release);

    RCLCPP_INFO(node_->get_logger(), "Reset complete");
}


void Real::start()
{
    if (!imu_->start()) {
        RCLCPP_ERROR(node_->get_logger(), "Failed to start IMU");
        write_log("Failed to start IMU", "ERROR");
        return;
    }

    if (!imu_->is_connected()) {
        write_log("imu not connected", "ERROR");
    }
    if (!gamepad_->is_connected()) {
        write_log("gamepad not connected", "WARNING");
    }

    rclcpp::executors::MultiThreadedExecutor executor(
        rclcpp::ExecutorOptions{}, 5);
    executor.add_node(node_);
    executor.spin();

    RCLCPP_INFO(node_->get_logger(), "Real node shut down");
}




pid_t find_pid_by_name(const std::string& process_name) {
    for (const auto& entry : std::filesystem::directory_iterator("/proc")) {
        if (!entry.is_directory()) continue;
        std::string dirname = entry.path().filename().string();
        if (dirname.find_first_not_of("0123456789") != std::string::npos) continue;

        std::ifstream cmd_file(entry.path() / "comm");
        if (!cmd_file.is_open()) continue;
        std::string comm;
        std::getline(cmd_file, comm);
        if (comm == process_name) {
            return std::stoi(dirname);
        }

        std::ifstream cmdline_file(entry.path() / "cmdline");
        if (!cmdline_file.is_open()) continue;
        std::string cmdline;
        std::getline(cmdline_file, cmdline);
        if (cmdline.rfind(process_name, 0) == 0 ||
            cmdline.find("/" + process_name) != std::string::npos) {
            return std::stoi(dirname);
        }
    }
    return -1;
}

bool kill_process(pid_t pid, int signal = SIGTERM) {
    if (kill(pid, signal) == 0) {
        std::cout << "Sent signal " << signal << " to PID " << pid << std::endl;
        return true;
    }
    std::cerr << "Failed to kill PID " << pid << ": " << strerror(errno) << std::endl;
    return false;
}

bool kill_process(const std::string& name, int signal = SIGTERM) {
    pid_t pid = find_pid_by_name(name);
    if (pid <= 0) {
        std::cerr << "Process '" << name << "' not found." << std::endl;
        return false;
    }
    return kill_process(pid, signal);
}

int main(int argc, char** argv)
{
    write_log("********************", "INFO");
    rclcpp::init(argc, argv);
    Real real;
    real.start();
    write_log("Finished", "INFO");

    return 0;
}

