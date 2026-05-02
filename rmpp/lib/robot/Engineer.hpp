#pragma once

#include "misc/Buzzer.hpp"
#include "misc/LED.hpp"
#include "module/chassis/Chassis.hpp"
#include "module/gimbal/Gimbal_Engineer.hpp"
#include "rc/RC.hpp"
#include "referee/Referee.hpp"
#include "ui/UI.hpp"

class Engineer {
public:
    struct config_t {
        UnitFloat<m_s> vxy_max;
        UnitFloat<rpm> wr_max;
        UnitFloat<W> power_limit;
    } config;

    struct device_t {
        LED& led;
        Buzzer& buzzer;

        RC& rc;
        Referee& referee;
        UI& ui;

        Chassis& chassis;
        Gimbal_Engineer& gimbal;

        bool (*is_calib_button_pressed)() = nullptr;
    } device;

    Engineer(const config_t& config, const device_t& device);

    void Init();

    void OnLoop();

private:
    bool calib_fn_last = false;
    bool calib_btn_last = false;
    bool gripper_last = false;

    Angle<deg> chassis_yaw = 0 * deg;

    void handleConnect();

    void handleRC();

    void handleReferee();

    void handleChassis();

    void handleArm();

    void handleGripper();

    void handleCalibration();

    void handleUI();
};
