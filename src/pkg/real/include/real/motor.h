#pragma once

#include <memory>
#include <string>
#include <vector>
#include <stdexcept>

#include "motor_driver.hpp"

namespace MOTOR
{

class LroMotor_75_8462
{
public:
    struct Config
    {
        uint16_t motor_id;
        std::string can_interface;       // e.g. "can0"
        std::string interface_type;       // "canfd" or "can"
        MotorDriver::MotorControlMode_e ctrl_mode = MotorDriver::MIT;
    };

    LroMotor_75_8462() = default;
    ~LroMotor_75_8462();

    LroMotor_75_8462(const LroMotor_75_8462&) = delete;
    LroMotor_75_8462& operator=(const LroMotor_75_8462&) = delete;

    void init(const std::vector<Config>& configs);
    bool init_single(int motor_id,
                     const std::string& interface_type = "canfd",
                     const std::string& can_interface = "can0",
                     MotorDriver::MotorControlMode_e ctrl_mode = MotorDriver::MIT);

    void motor_mit_cmd(uint16_t id,
                       float target_position = 0.0f,
                       float target_speed = 0.0f,
                       float kp = 0.0f,
                       float kd = 0.0f,
                       float target_torque = 0.0f);

    void motor_mit_cmd_s(const float* target_position,
                         const float* kp,
                         const float* kd,
                         size_t count);

    float get_motor_position(uint16_t id) const;
    float get_motor_speed(uint16_t id) const;

    void get_motors_position_s(double* pos, size_t count) const;
    void get_motors_speed_s(double* spd, size_t count) const;

    size_t motor_count() const { return motors_.size(); }

private:
    size_t index_of(uint16_t id) const;

    std::vector<std::shared_ptr<MotorDriver>> motors_;
    std::vector<uint16_t> id_lut_;
};

} // namespace MOTOR
