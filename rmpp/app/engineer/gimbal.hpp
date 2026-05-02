#pragma once

#include "arm/Arm.hpp"
#include "gripper/gripper.hpp"
#include "motor/CustomMotor.hpp"
#include "motor/CyberDogMotor.hpp"
#include "motor/DM4310.hpp"
#include "motor/M2006.hpp"
#include "module/gimbal/Gimbal_Engineer.hpp"

namespace Arm {

const DHLink dh_links[DOF] = {
    {60.0f, 0.0f, -M_PI_2, 0.0f},
    {0.0f, 390.0f, 0.0f, -M_PI_2},
    {65.0f, 0.0f, M_PI_2, M_PI_2},
    {391.0f, 0.0f, -M_PI_2, 0.0f},
    {0.0f, 0.0f, M_PI_2, 0.0f},
    {0.0f, 0.0f, 0.0f, 0.0f},
};

const Robot6DOF robot = {
    .links = {
        {60.0f, 0.0f, -M_PI_2, 0.0f},
        {0.0f, 390.0f, 0.0f, -M_PI_2},
        {65.0f, 0.0f, M_PI_2, M_PI_2},
        {391.0f, 0.0f, -M_PI_2, 0.0f},
        {0.0f, 0.0f, M_PI_2, 0.0f},
        {0.0f, 0.0f, 0.0f, 0.0f},
    },
    .q_min = {
        -180.0f * DEG2RAD,
        -49.0f * DEG2RAD,
        0.0f * DEG2RAD,
        -180.0f * DEG2RAD,
        -180.0f * DEG2RAD,
        -180.0f * DEG2RAD,
    },
    .q_max = {
        180.0f * DEG2RAD,
        45.0f * DEG2RAD,
        177.0f * DEG2RAD,
        180.0f * DEG2RAD,
        180.0f * DEG2RAD,
        180.0f * DEG2RAD,
    },
};

} // namespace Arm

inline PID::config_t j1_speed_pid = {
    .kp = (1 * A) / (10 * deg_s),
    .ki = (0 * A) / (100 * deg),
    .kd = (0 * A) / (10 * deg_s),
    .max_i = 0.5 * A,
    .max_out = 7.5 * A,
    .fc = 10 * Hz,
};

inline PID::config_t j1_angle_pid = {
    .kp = 8 * default_unit,
    .kd = 0.2 * default_unit,
    .max_out = 200 * deg_s,
};

inline CustomMotor j1_motor({
    .can_port = 2,
    .master_id = 0x205,
    .slave_id = 0x1FF,
    .timeout = 1000 * ms,
    .is_invert = true,
    .offset = 129.05f * deg,
    .control_mode = Motor::ANGLE_SPEED_MODE,
    .pid_out_type = Motor::CURRENT_OUTPUT,
    .speed_pid_config = &j1_speed_pid,
    .angle_pid_config = &j1_angle_pid,
});

inline PID::config_t j2_speed_pid = {
    .kp = (7 * Nm) / (100 * deg_s),
    .ki = (3 * Nm) / (60 * deg),
    .max_i = 1 * Nm,
    .max_out = 10 * Nm,
    .fc = 5 * Hz,
};

inline PID::config_t j2_angle_pid = {
    .kp = 30 * default_unit,
    .kd = 0 * default_unit,
    .max_out = 240 * deg_s,
};

inline CyberDogMotor j2({
    .can_port = 2,
    .master_id = 0x05,
    .slave_id = 0x03,
    .timeout = 2000 * ms,
    .is_limit = true,
    .limit_min = -47.5 * deg,
    .limit_max = 45 * deg,
    .control_mode = Motor::ANGLE_SPEED_MODE,
    .pid_out_type = Motor::TORQUE_OUTPUT,
    .speed_pid_config = &j2_speed_pid,
    .angle_pid_config = &j2_angle_pid,
});

inline PID::config_t j3_speed_pid = {
    .kp = (3 * Nm) / (100 * deg_s),
    .ki = (0 * Nm) / (60 * deg),
    .max_i = 2 * Nm,
    .max_out = 10 * Nm,
    .fc = 2 * Hz,
};

inline PID::config_t j3_angle_pid = {
    .kp = 38 * default_unit,
    .max_out = 240 * deg_s,
};

inline CyberDogMotor j3({
    .can_port = 2,
    .master_id = 0x06,
    .slave_id = 0x04,
    .timeout = 2000 * ms,
    .is_invert = true,
    .offset = 31.0 * deg,
    .is_limit = true,
    .limit_min = 0 * deg,
    .limit_max = 175 * deg,
    .control_mode = Motor::ANGLE_SPEED_MODE,
    .pid_out_type = Motor::TORQUE_OUTPUT,
    .speed_pid_config = &j3_speed_pid,
    .angle_pid_config = &j3_angle_pid,
});

inline PID::config_t j4_speed_pid = {
    .kp = (1.0 * Nm) / (100 * deg_s),
    .ki = (3 * Nm) / (60 * deg),
    .max_i = 2 * Nm,
    .max_out = 3 * Nm,
    .fc = 10 * Hz,
};

inline PID::config_t j4_angle_pid = {
    .kp = 9 * default_unit,
    .max_out = 180 * deg_s,
};

inline DM4310 j4({
    .can_port = 2,
    .master_id = 0x10,
    .slave_id = 0x11,
    .is_invert = true,
    .offset = 31.52 * deg,
    .control_mode = Motor::ANGLE_SPEED_MODE,
    .pid_out_type = Motor::TORQUE_OUTPUT,
    .speed_pid_config = &j4_speed_pid,
    .angle_pid_config = &j4_angle_pid,
});

inline PID::config_t j5_speed_pid = {
    .kp = (5 * A) / (100 * deg_s),
    .ki = (10 * A) / (60 * deg),
    .max_i = 8 * A,
    .max_out = 8 * A,
    .fc = 10 * Hz,
};

inline PID::config_t j5_angle_pid = {
    .kp = 10 * default_unit,
    .max_out = 600 * deg_s,
};

inline M2006 j5({
    .can_port = 1,
    .master_id = 0x202,
    .slave_id = 0x200,
    .is_limit = true,
    .limit_min = -2160 * deg,
    .limit_max = 2160 * deg,
    .control_mode = Motor::ANGLE_SPEED_MODE,
    .pid_out_type = Motor::CURRENT_OUTPUT,
    .speed_pid_config = &j5_speed_pid,
    .angle_pid_config = &j5_angle_pid,
});

inline PID::config_t j6_speed_pid = {
    .kp = (5 * A) / (100 * deg_s),
    .ki = (10 * A) / (60 * deg),
    .max_i = 8 * A,
    .max_out = 8 * A,
    .fc = 10 * Hz,
};

inline PID::config_t j6_angle_pid = {
    .kp = 10 * default_unit,
    .max_out = 600 * deg_s,
};

inline M2006 j6({
    .can_port = 1,
    .master_id = 0x201,
    .slave_id = 0x200,
    .is_limit = true,
    .limit_min = -2160 * deg,
    .limit_max = 2160 * deg,
    .control_mode = Motor::ANGLE_SPEED_MODE,
    .pid_out_type = Motor::CURRENT_OUTPUT,
    .speed_pid_config = &j6_speed_pid,
    .angle_pid_config = &j6_angle_pid,
});

inline Gripper gripper({
    .can_port = 2,
    .cmd_id = 0x20,
    .feedback_id = 0x21,
});

inline Gimbal_Engineer::config_t gimbal_config = {
    .j2_init = -47.50 * deg,
    .j3_init = 82.00 * deg,
    .pitch_init = 90.0f,
    .roll_init = 0.0f,
    .pitch_factor = 3.0f,
    .roll_factor = 6.0f,
    .pitch_speed_max = 90.0f,
    .roll_speed_max = 180.0f,

    .gravity_c2 = -13.00f,
    .gravity_c3 = -4.64f,
    .torque_limit = 12 * Nm,

    .cartesian_speed = 200.0f,
    .cartesian_max_jump = 10.0f * M_PI / 180.0f,
};

inline Gimbal_Engineer gimbal(gimbal_config, {j1_motor, j2, j3, j4, j5, j6, gripper});
