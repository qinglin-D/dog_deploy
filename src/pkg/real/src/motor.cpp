#include "real/motor.h"

#include <algorithm>
#include <cassert>
#include <cstdio>
#include <stdexcept>
#include <unordered_map>

namespace MOTOR
{

LroMotor_75_8462::~LroMotor_75_8462()
{
    for (auto& m : motors_) {
        if (m) {
            m->deinit_motor();
            m.reset();
        }
    }
    motors_.clear();
    id_lut_.clear();
}

void LroMotor_75_8462::init(const std::vector<Config>& configs)
{
    motors_.clear();
    id_lut_.clear();
    motors_.reserve(configs.size());
    id_lut_.reserve(configs.size());

    for (const auto& cfg : configs) {
        if (!init_single(cfg.motor_id, cfg.interface_type,
                         cfg.can_interface, cfg.ctrl_mode)) {
            fprintf(stderr, "[Motor] Failed to init motor ID=%d\n", cfg.motor_id);
        }
    }
    fprintf(stdout, "[Motor] Initialized %zu / %zu motors\n",
            motors_.size(), configs.size());
}

bool LroMotor_75_8462::init_single(int motor_id,
                           const std::string& interface_type,
                           const std::string& can_interface,
                           MotorDriver::MotorControlMode_e ctrl_mode)
{
    std::cout << motor_id << interface_type << can_interface << std::endl;
    auto motor = MotorDriver::create_motor(motor_id, interface_type, can_interface, "LROCAN", 2);
    int a = !motor->init_motor();
    if (!motor || a != 0) {
        if (!motor)
            fprintf(stderr, "[Motor] !!motor\n");
        else
            fprintf(stderr, "[Motor] !!motor->init_motor(), error_id: %d\n", a);
        return false;
    }
    motor->set_motor_control_mode(ctrl_mode);
    motors_.push_back(std::move(motor));
    id_lut_.push_back(motor_id);
    return true;
}

size_t LroMotor_75_8462::index_of(uint16_t id) const
{
    auto it = std::find(id_lut_.begin(), id_lut_.end(), id);
    if (it == id_lut_.end()) {
        fprintf(stderr, "[Motor] Motor ID=%d not found\n", id);
        return static_cast<size_t>(-1);
    }
    return static_cast<size_t>(std::distance(id_lut_.begin(), it));
}

void LroMotor_75_8462::motor_mit_cmd(uint16_t id,
                             float target_position,
                             float target_speed,
                             float kp, float kd, float target_torque)
{
    size_t idx = index_of(id);
    if (idx < motors_.size()) {
        motors_[idx]->motor_mit_cmd(target_speed, target_position, kp, kd, target_torque);
    }
}

void LroMotor_75_8462::motor_mit_cmd_s(const float* target_position,
                               const float* kp, const float* kd,
                               size_t count)
{
    assert(target_position && kp && kd);
    count = std::min(count, motors_.size());
    for (size_t i = 0; i < count; ++i) {
        motors_[i]->motor_mit_cmd(0.0f, target_position[i], kp[i], kd[i], 0.0f);
    }
}

float LroMotor_75_8462::get_motor_position(uint16_t id) const
{
    size_t idx = index_of(id);
    if (idx < motors_.size()) {
        return motors_[idx]->get_motor_pos();
    }
    return 0.0f;
}

float LroMotor_75_8462::get_motor_speed(uint16_t id) const
{
    size_t idx = index_of(id);
    if (idx < motors_.size()) {
        return motors_[idx]->get_motor_spd();
    }
    return 0.0f;
}

void LroMotor_75_8462::get_motors_position_s(double* pos, size_t count) const
{
    assert(pos);
    count = std::min(count, motors_.size());
    for (size_t i = 0; i < count; ++i) {
        pos[i] = static_cast<double>(motors_[i]->get_motor_pos());
    }
}

void LroMotor_75_8462::get_motors_speed_s(double* spd, size_t count) const
{
    assert(spd);
    count = std::min(count, motors_.size());
    for (size_t i = 0; i < count; ++i) {
        spd[i] = static_cast<double>(motors_[i]->get_motor_spd());
    }
}

} // namespace MOTOR
