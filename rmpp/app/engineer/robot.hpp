#pragma once

#include "robot/Engineer.hpp"

#include "main.h"
#include "misc.hpp"
#include "chassis.hpp"
#include "gimbal.hpp"

inline bool is_calib_button_pressed() {
    return HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) == GPIO_PIN_RESET;
}

inline Engineer::config_t robot_config = {
    .vxy_max = 1.5 * m_s,
    .wr_max = 30 * rpm,
    .power_limit = 60 * W,
};

inline Engineer robot(robot_config,
                      {
                          led,
                          buzzer,

                          rc,
                          referee,
                          ui,

                          chassis,
                          gimbal,

                          is_calib_button_pressed,
                      });
