#include "Engineer.hpp"

Engineer::Engineer(const config_t& config, const device_t& device) : config(config), device(device) {}

void Engineer::Init() {
    device.buzzer.Play(Buzzer::C5D5G5);
}

void Engineer::OnLoop() {
    device.led.OnLoop();
    device.buzzer.OnLoop();

    handleConnect();

    handleRC();
    handleReferee();

    handleChassis();
    handleArm();
    handleUI();
}

void Engineer::handleConnect() {
    const bool en = (device.referee.game.game_progress == Referee::GAMING) || device.rc.is_enable;

    device.chassis.SetEnable(en);
    device.gimbal.SetEnable(en);
}

void Engineer::handleRC() {
    device.rc.OnLoop();
}

void Engineer::handleReferee() {
    device.referee.SetYaw(chassis_yaw, chassis_yaw);
    device.referee.OnLoop();
}

void Engineer::handleChassis() {
    auto& rc = device.rc;
    auto& chassis = device.chassis;
    auto& referee = device.referee;

    if (rc.is_enable && !rc.is_fn) {
        chassis.SetSpeed(rc.x * config.vxy_max,
                         rc.y * config.vxy_max,
                         rc.yaw * config.wr_max);
    } else {
        chassis.SetSpeed(0 * default_unit, 0 * default_unit, 0 * default_unit);
    }

    chassis.SetGimbalYaw(device.gimbal.GetJ1Angle());

    UnitFloat<W> power_limit = referee.chassis.power_limit;
    if (power_limit > config.power_limit || power_limit == 0 * W) {
        power_limit = config.power_limit;
    }
    chassis.SetPowerLimit(power_limit);
    chassis.SetBufferEnergy(referee.chassis.buffer_energy);
    chassis.OnLoop();
}

void Engineer::handleArm() {
    auto& rc = device.rc;
    const auto z_axis = unit::clamp(rc.pitch + rc.shoot, 1 * ratio);

    if (rc.is_enable && rc.is_fn) {
        if (rc.is_auto_aim) {
            device.gimbal.SetPolarControl(rc.x, rc.y, z_axis, rc.yaw, 0 * ratio);
        } else {
            device.gimbal.SetJointControl(rc.x, rc.y);
        }
    } else {
        device.gimbal.SetJointControl(0 * ratio, 0 * ratio);
    }

    handleGripper();
    device.gimbal.OnLoop();
    handleCalibration();
}

void Engineer::handleGripper() {
    auto& rc = device.rc;

    if (rc.is_shoot && !gripper_last) {
        device.gimbal.ToggleGripper(20 * deg_s, 700 * mA);
    }
    gripper_last = rc.is_shoot;
}

void Engineer::handleCalibration() {
    if (!device.rc.is_enable) {
        const bool fn_now = device.rc.is_calib;
        const bool btn_now = device.is_calib_button_pressed != nullptr && device.is_calib_button_pressed();
        const bool fn_triggered = calib_fn_last && !fn_now;
        const bool btn_triggered = calib_btn_last && !btn_now;

        if (fn_triggered || btn_triggered) {
            device.gimbal.CalibrateJ23();
            device.buzzer.Play(Buzzer::C5D5G5);
        }

        calib_fn_last = fn_now;
        calib_btn_last = btn_now;
    } else {
        calib_fn_last = false;
        calib_btn_last = false;
    }
}

void Engineer::handleUI() {
    auto& chassis = device.chassis;
    auto& ui = device.ui;

    const float wr_dps = chassis.wr.measure.toFloat(rpm) * 6.0f;
    chassis_yaw += wr_dps * 0.001f * deg;

    ui.yaw_ecd = chassis_yaw;
    ui.yaw = chassis_yaw;
    ui.pitch = 0 * deg;
    ui.is_detect = false;
    ui.cap = 0 * ratio;
    ui.wr = chassis.wr.measure;
    ui.bullet = 0 * Hz;
    ui.shoot = 0 * A;

    chassis.UpdateUI(ui);
    device.gimbal.UpdateUI(ui);
    ui.robot.referee = device.referee.is_connect ? UI::GREEN : UI::PINK;
    ui.robot.aim = UI::PINK;

    ui.OnLoop();
}
