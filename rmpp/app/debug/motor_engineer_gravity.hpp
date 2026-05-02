#pragma once

#include "bsp/bsp.hpp"
#include "motor/CyberDogMotor.hpp"

inline constexpr Angle<deg> J2_TARGET = -47.5f * deg;
inline constexpr Angle<deg> J3_TARGET = 82.0f * deg;

inline constexpr float GRAVITY_C2 = -13.00f;
inline constexpr float GRAVITY_C3 = -4.64f;
inline constexpr float GRAVITY_COUPLING = 0.055f;
inline constexpr UnitFloat<> GRAVITY_TORQUE_LIMIT = 12.0f * Nm;

inline PID::config_t gravity_j2_speed_pid = {
    .kp = (7.0f * Nm) / (100.0f * deg_s),
    .ki = (3.0f * Nm) / (60.0f * deg),
    .max_i = 1.0f * Nm,
    .max_out = 10.0f * Nm,
    .fc = 5.0f * Hz,
};

inline PID::config_t gravity_j2_angle_pid = {
    .kp = 30.0f * default_unit,
    .kd = 0.0f * default_unit,
    .max_out = 240.0f * deg_s,
};

inline CyberDogMotor gravity_j2({
    .can_port = 2,
    .master_id = 0x05,
    .slave_id = 0x03,
    .timeout = 2000.0f * ms,
    .is_limit = true,
    .limit_min = -47.5f * deg,
    .limit_max = 45.0f * deg,
    .control_mode = Motor::ANGLE_SPEED_MODE,
    .pid_out_type = Motor::TORQUE_OUTPUT,
    .speed_pid_config = &gravity_j2_speed_pid,
    .angle_pid_config = &gravity_j2_angle_pid,
});

inline PID::config_t gravity_j3_speed_pid = {
    .kp = (3.0f * Nm) / (100.0f * deg_s),
    .ki = (0.0f * Nm) / (60.0f * deg),
    .max_i = 2.0f * Nm,
    .max_out = 10.0f * Nm,
    .fc = 2.0f * Hz,
};

inline PID::config_t gravity_j3_angle_pid = {
    .kp = 38.0f * default_unit,
    .max_out = 240.0f * deg_s,
};

inline CyberDogMotor gravity_j3({
    .can_port = 2,
    .master_id = 0x06,
    .slave_id = 0x04,
    .timeout = 2000.0f * ms,
    .is_invert = true,
    .offset = 31.0f * deg,
    .is_limit = true,
    .limit_min = 0.0f * deg,
    .limit_max = 175.0f * deg,
    .control_mode = Motor::ANGLE_SPEED_MODE,
    .pid_out_type = Motor::TORQUE_OUTPUT,
    .speed_pid_config = &gravity_j3_speed_pid,
    .angle_pid_config = &gravity_j3_angle_pid,
});
