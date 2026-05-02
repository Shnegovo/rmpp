#pragma once

#include "bsp/bsp.hpp"
#include "motor/M2006.hpp"

inline PID::config_t motor_speed_pid = {
    .kp = (0.8f * A) / (100.0f * deg_s),
    .ki = (0.0f * A) / (60.0f * deg),
    .kd = (0.0f * A) / (100.0f * deg_s),
    .max_i = 0.2f * A,
    .max_out = 1.0f * A,
    .fc = 10.0f * Hz,
};

inline PID::config_t motor_angle_pid_current_out = {
    .kp = (0.4f * A) / (10.0f * deg),
    .ki = (0.0f * A) / (60.0f * deg),
    .kd = (0.02f * A) / (10.0f * deg_s),
    .max_i = 0.0f * A,
    .max_out = 1.0f * A,
    .fc = 10.0f * Hz,
};

inline PID::config_t motor_angle_pid_speed_out = {
    .kp = 4.0f * default_unit,
    .ki = 0.0f * default_unit,
    .kd = 0.1f * default_unit,
    .max_i = 0.0f * deg_s,
    .max_out = 90.0f * deg_s,
    .fc = 10.0f * Hz,
};

inline M2006 motor({
    .can_port = 1,
    .master_id = 0x201,
    .slave_id = 0x200,
    .is_invert = false,
    .is_limit = true,
    .limit_min = -180.0f * deg,
    .limit_max = 180.0f * deg,

    .control_mode = Motor::OPEN_LOOP_MODE, // Motor::SPEED_MODE, Motor::ANGLE_MODE, Motor::ANGLE_SPEED_MODE
    .pid_out_type = Motor::CURRENT_OUTPUT,

    .speed_pid_config = &motor_speed_pid,
    .angle_pid_config = &motor_angle_pid_current_out, // ANGLE_SPEED_MODE: &motor_angle_pid_speed_out
});
