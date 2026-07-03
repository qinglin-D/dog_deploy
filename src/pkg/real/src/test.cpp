// #include "real/real.h"

// #include <yaml-cpp/yaml.h>
// #include <algorithm>
// #include <cassert>
// #include <cmath>
// #include <iostream>
// #include <cstring>
// #include <csignal>
// #include <filesystem>
// #include <fstream>
// #include <dirent.h>
// #include <sys/types.h>

// Real::Real(std::string node_name)
// {
//         std::cout << "testXXXXXXXXXXX" << std::endl;

//     // node_ = rclcpp::Node::make_shared(node_name);
//     LoadConfig();

//     // publisher_ = node_->create_publisher<std_msgs::msg::Float64MultiArray>("observation", 10);
//     // subscriber_ = node_->create_subscription<std_msgs::msg::Float32MultiArray>(
//     //     "actions", 10,
//     //     std::bind(&Real::ControlCallback, this, std::placeholders::_1));

//     // RCLCPP_INFO(node_->get_logger(), "Initializing real node (obs_dim=%d, dof=%d, freq=%.1f Hz)",
//     //     obs_dim_, num_dof_, frequency_);

//     // imu_ = std::make_unique<IMU::Imu>(imu_device_, imu_baudrate_);
//     std::cout << "testXXXXXXXXXXX" << std::endl;

//     // motor_ = std::make_unique<MOTOR::LroMotor_8462>();
//     // motor_->init(motor_configs_);
//     for (auto item : motor_configs_)
//     {
//         motor_.push_back(MotorDriver::create_motor(item.motor_id, item.interface_type, item.can_interface, "LROCAN", 2));
//         int err = motor_.back()->init_motor();
//         std::cout << err << std::endl;
//     }

//     // shutdown_thread_ = std::thread(std::bind(&Real::Shutdown, this));
//     // running_.store(true, std::memory_order_release
    

//     // gamepad_ = std::make_unique<GAMEPAD::Gamepad>(
//     //     cmd_x_min_, cmd_x_max_, cmd_y_min_, cmd_y_max_,
//     //     cmd_yaw_min_, cmd_yaw_max_, gamepad_mapping_);
//     // gamepad_->make_subscription(node_);

//     // auto period = std::chrono::milliseconds(static_cast<int>(1000.0 / frequency_));
//     // timer_ = node_->create_wall_timer(std::chrono::milliseconds(50), std::bind(&Real::PubCallback, this));

// }

// Real::~Real()
// {
//     // over_.store(true, std::memory_order_release);
//     running_.store(false, std::memory_order_release);
//     if (shutdown_thread_.joinable())
//         shutdown_thread_.join();
// }

// void Real::LoadConfig()
// {
//     try {
//         YAML::Node config = YAML::LoadFile(config_path_);

//         obs_dim_     = config["sensorDataSize"].as<int>();
//         num_dof_     = config["actionsSize"].as<int>();
//         frequency_   = config["frequency"].as<double>();

//         imu_device_   = config["imu"]["port"].as<std::string>();
//         imu_baudrate_ = config["imu"]["baudrate"].as<int>();

//         cmd_x_min_   = config["gamepad"]["x"][0].as<double>();
//         cmd_x_max_   = config["gamepad"]["x"][1].as<double>();
//         cmd_y_min_   = config["gamepad"]["y"][0].as<double>();
//         cmd_y_max_   = config["gamepad"]["y"][1].as<double>();
//         cmd_yaw_min_ = config["gamepad"]["yaw"][0].as<double>();
//         cmd_yaw_max_ = config["gamepad"]["yaw"][1].as<double>();
//         gamepad_mapping_.axis_x = config["gamepad"]["axis_x"].as<int>();
//         gamepad_mapping_.axis_y = config["gamepad"]["axis_y"].as<int>();
//         gamepad_mapping_.axis_yaw = config["gamepad"]["axis_yaw"].as<int>();
//         gamepad_mapping_.axis_trigger = config["gamepad"]["axis_trigger"].as<int>();
//         gamepad_mapping_.button_go_on = config["gamepad"]["button_go_on"].as<int>();
//         gamepad_mapping_.button_pause = config["gamepad"]["button_pause"].as<int>();
//         gamepad_mapping_.button_reset = config["gamepad"]["button_reset"].as<int>();
//         gamepad_mapping_.button_terminate = config["gamepad"]["button_terminate"].as<int>();

//         std::vector<std::string> joint_names =
//             config["actuator"]["control_order"].as<std::vector<std::string>>();

//         motor_configs_.clear();
//         motor_configs_.reserve(joint_names.size());
//         kp_.reserve(joint_names.size());
//         kd_.reserve(joint_names.size());
//         motor_.reserve(joint_names.size());

//         for (const auto& name : joint_names) {
//             MOTOR::LroMotor_8462::Config cfg;
//             cfg.motor_id      = config["actuator"]["motor_ID"][name].as<uint8_t>();
//             cfg.can_interface = config["actuator"]["interface"][name].as<std::string>();
//             cfg.interface_type = config["actuator"]["interface_type"].as<std::string>();
//             cfg.ctrl_mode      = MotorDriver::MIT;
//             motor_configs_.push_back(cfg);

//             kp_.push_back(config["actuator"]["stiffness"][name].as<double>());
//             kd_.push_back(config["actuator"]["damping"][name].as<double>());
//         }
//         std::cout << "motor_configs_: " << motor_configs_.size() << std::endl;
//         std::cout << "motor_configs_: " << (int)motor_configs_[0].motor_id << std::endl;
//         std::cout << "motor_configs_: " << motor_configs_[0].ctrl_mode << std::endl;
//         std::cout << "motor_configs_: " << motor_configs_[0].can_interface << std::endl;
//         std::cout << "motor_configs_: " << motor_configs_[0].interface_type << std::endl;

//         // RCLCPP_INFO(node_->get_logger(), "Loaded config: %zu motors on %s @ %d baud",
//         //             motor_configs_.size(), imu_device_.c_str(), imu_baudrate_);

//     } catch (const YAML::Exception& e) {
//         RCLCPP_FATAL(node_->get_logger(), "Failed to load config: %s", e.what());
//         exit(EXIT_FAILURE);
//     }
// }

// void Real::PubCallback()
// {
//     // Health check logging (throttled)
//     static int health_counter = 0;
//     if (++health_counter >= static_cast<int>(frequency_)) {
//         health_counter = 0;
//         if (!imu_->is_connected()) {
//             RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000,
//                                  "IMU disconnected");
//             return;
//         } else if (!imu_->is_healthy()) {
//             RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000,
//                                  "IMU data stale");
//             return;
//         }
//         if (!gamepad_->is_connected()) {
//             RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000,
//                                  "Gamepad disconnected");
//             return;
//         }
//     }

//     if (!running_.load(std::memory_order_acquire)) {
//         std::this_thread::sleep_for(std::chrono::milliseconds(80));
//         return;
//     }

//     std_msgs::msg::Float64MultiArray msg;
//     msg.data.resize(obs_dim_);
//     double* data = msg.data.data();

//     // command (3)
//     gamepad_->get_command(data);
//     // std::cout << "cmd: " << data[0] << ", " << data[1] << ", " << data[2] << std::endl;
//     data += 3;
    
//     // angular velocity (3)
//     imu_->get_angular_velocity(data);
//     data += 3;

//     // projected gravity (3)
//     imu_->get_projected_gravity(data);
//     data += 3;

//     // joint positions (num_dof_)
//     // motor_->get_motors_position_s(data, num_dof_);
//     // std::cout << "motor: " << data[0] << ", " << data[1] << std::endl;
//     // data += 2;

//     // joint velocities (num_dof_)
//     // motor_->get_motors_speed_s(data, num_dof_);
//     // data += num_dof_;

//     // last actions (num_dof_) — use ctrl_data_ from previous cycle
//     for (int i = 0; i < num_dof_*2 -2; ++i) {
//         data[i] = 0.0; // static_cast<double>(ctrl_data_[i]);
//     }

//     publisher_->publish(msg);

// }

// void Real::ControlCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
// {
//     if (!running_.load(std::memory_order_acquire)) {
//         // std::vector<float> zero_data;
//         // std::vector<float> kp_;
//         // std::vector<float> kd_;
//         // zero_data.resize(num_dof_);
//         // kp_.resize(num_dof_);
//         // kd_.resize(num_dof_);
//         // motor_->motor_mit_cmd_s(zero_data.data(), kp_.data(), kd_.data(), num_dof_);
//         return;
//     }
//     // if (msg->data.size() < static_cast<size_t>(num_dof_)) {
//     //     RCLCPP_ERROR(node_->get_logger(),
//     //                  "Received actions size %zu < expected %d",
//     //                  msg->data.size(), num_dof_);
//     //     return;
//     // }

//     // motor_->motor_mit_cmd_s(msg->data.data(), kp_.data(), kd_.data(), num_dof_);
// }


// void Real::reset()
// {
//     LoadConfig();
//     RCLCPP_INFO(node_->get_logger(), "Initializing real node (obs_dim=%d, dof=%d, freq=%.1f Hz)",
//         obs_dim_, num_dof_, frequency_);

//     gamepad_.reset();
//     gamepad_ = std::make_unique<GAMEPAD::Gamepad>(
//         cmd_x_min_, cmd_x_max_, cmd_y_min_, cmd_y_max_,
//         cmd_yaw_min_, cmd_yaw_max_, gamepad_mapping_);
//     gamepad_->make_subscription(node_);    
    
//     // motor_.reset();
//     // motor_ = std::make_unique<MOTOR::LroMotor_8462>();
//     // motor_->init(motor_configs_);

//     timer_->cancel();
//     auto period = std::chrono::milliseconds(static_cast<int>(1000.0 / frequency_));
//     timer_ = node_->create_wall_timer(period, std::bind(&Real::PubCallback, this));
// }


// void Real::start()
// {
//     if (!imu_->start()) {
//         RCLCPP_ERROR(node_->get_logger(), "Failed to start IMU");
//         return;
//     }

//     shutdown_thread_ = std::thread([this](){
//         while (rclcpp::ok())
//             {
//                 std::this_thread::sleep_for(std::chrono::milliseconds(400));
//                 auto state = gamepad_->get_states();
//                 switch (state)
//                 {
//                 case GAMEPAD::Gamepad::TERMINATE:
//                     rclcpp::shutdown();
//                     return;

//                 case GAMEPAD::Gamepad::PAUSE:
//                     running_.store(false, std::memory_order_release);
//                     if (!timer_->is_canceled()){
//                         timer_->cancel();
//                     }
//                     break;

//                 case GAMEPAD::Gamepad::GO_ON:
//                     running_.store(true, std::memory_order_release);
//                     if (timer_->is_canceled()){
//                         timer_->reset();
//                     }
//                     break;

//                 case GAMEPAD::Gamepad::RESET:
//                     if (running_.load(std::memory_order_acquire)){                        
//                         running_.store(false, std::memory_order_release);
//                         reset();
//                         running_.store(true, std::memory_order_release);
//                     }
//                     break;
//                 default:
//                     break;
//             }
//         }
//     });

//     rclcpp::executors::MultiThreadedExecutor executor(
//         rclcpp::ExecutorOptions{}, 5);
//     executor.add_node(node_);
//     executor.spin();

//     RCLCPP_INFO(node_->get_logger(), "Real node shut down");
// }




// void Real::test(){
//     // std::cout << "testXXXXXXXXXXX" << motor_->motor_count() << std::endl;
//     // double pos[2];
//     // float pos_[2] = {0.0, 0.0};
//     // float kp[2] = {10., 10.};
//     // float kd[2] = {1., 1.};
//     // std::this_thread::sleep_for(std::chrono::seconds(3));
//     // for(int i = 0; i < 10; ++i){
//     //     motor_->get_motors_position_s(pos, 1);
//     //     std::cout << "motor: " << pos[0] << ", " << pos[1] << std::endl;
//     //     pos_[0] = pos[0] + 0.1;
//     //     pos_[1] = pos[1] + 0.1;
//     //     motor_->motor_mit_cmd_s(pos_, kp, kd, 1);
//     //     std::this_thread::sleep_for(std::chrono::milliseconds(500));
//     // }
//     // motor_->motor_count();
// }

// int main(int argc, char* argv[])
// {
//     // rclcpp::init(argc, argv);
//     // auto node = rclcpp::Node::make_shared("real");
//     // std::cout << "test" << std::endl;
//     // rclcpp::spin(node);
//     // rclcpp::shutdown();

//     // Real real;
//     // real.test();

//     std::vector<std::shared_ptr<MotorDriver>> motors;
//     motors.resize(2);
//     motors[0] = MotorDriver::create_motor(2, "can","can0", "LROCAN", 2);
//     std::cout << "motor: " << (int)motors[0]->init_motor() << std::endl;
//     motors[1] = MotorDriver::create_motor(1, "can","can0", "LROCAN", 2);
//     std::cout << "motor: " << (int)motors[1]->init_motor() << std::endl;

//     // auto motor = MotorDriver::create_motor(2, "can","can0", "LROCAN", 2);
//     float tar_pos1 = 0., tar_pos2 = 0.;

//     std::this_thread::sleep_for(std::chrono::milliseconds(3000));


//     // for (int i = 0; i < 500; ++i){
//     //     std::this_thread::sleep_for(std::chrono::milliseconds(10));
        
//     //     motors[0]->motor_mit_cmd(0., 0, 5, 1, 0);
//     //     // std::this_thread::sleep_for(std::chrono::milliseconds(10));
//     //     motors[1]->motor_mit_cmd(0., 0, 5, 1, 0);
//     // }
//     tar_pos1 = motors[0]->get_motor_pos();
//     tar_pos2 = motors[1]->get_motor_pos();
//     for (int i = 0; i < 1000; ++i){
        
//         std::this_thread::sleep_for(std::chrono::milliseconds(10));
//         tar_pos1 += 0.01;
//         tar_pos2 += 0.01;
//         motors[0]->motor_mit_cmd(tar_pos1, 1, 10, 1, 0);
//         // std::this_thread::sleep_for(std::chrono::milliseconds(10));
//         motors[1]->motor_mit_cmd(tar_pos2, 1, 10, 1, 0);
//     }

//     return 0;
// }





