#include "Gimbal_Engineer.hpp"

#include <cmath>

Gimbal_Engineer::Gimbal_Engineer(const config_t& config, const motor_t& motor)
    : config(config),
      motor(motor),
      j2_target(config.j2_init),
      j3_target(config.j3_init),
      pitch_target(config.pitch_init),
      roll_target(config.roll_init) {}

void Gimbal_Engineer::SetEnable(const bool is_enable) {
    if (this->is_enable == is_enable) return;
    this->is_enable = is_enable;

    motor.j1.SetEnable(is_enable);
    motor.j2.SetEnable(is_enable);
    motor.j3.SetEnable(is_enable);
    motor.j4.SetEnable(is_enable);
    motor.j5.SetEnable(is_enable);
    motor.j6.SetEnable(is_enable);
    motor.gripper.SetEnable(is_enable);

    if (!is_enable) {
        resetTarget();
    }
}

void Gimbal_Engineer::SetJointControl(const UnitFloat<>& j2_axis, const UnitFloat<>& j4_axis) {
    control_mode = JOINT_CONTROL;
    joint_control.j2 = unit::clamp(j2_axis, 1 * ratio);
    joint_control.j4 = unit::clamp(j4_axis, 1 * ratio);
}

void Gimbal_Engineer::SetPolarControl(const UnitFloat<>& r_axis,
                                      const UnitFloat<>& theta_axis,
                                      const UnitFloat<>& z_axis,
                                      const UnitFloat<>& pitch_axis,
                                      const UnitFloat<>& roll_axis) {
    control_mode = POLAR_CONTROL;
    polar_control.r = unit::clamp(r_axis, 1 * ratio);
    polar_control.theta = unit::clamp(theta_axis, 1 * ratio);
    polar_control.z = unit::clamp(z_axis, 1 * ratio);
    polar_control.pitch = unit::clamp(pitch_axis, 1 * ratio);
    polar_control.roll = unit::clamp(roll_axis, 1 * ratio);
}

void Gimbal_Engineer::SetCartesianMove(const int pose) {
    if (pose < 0) return;
    control_mode = CARTESIAN_CONTROL;
    cartesian_pose = pose;
}

void Gimbal_Engineer::SetCustomController(const bool is_connect, const bool lock, const Angle<deg> joint[Arm::DOF]) {
    if (!is_connect || lock) return;
    control_mode = CUSTOM_CONTROLLER_CONTROL;
    custom_controller.is_connect = is_connect;
    custom_controller.lock = lock;
    for (int i = 0; i < Arm::DOF; i++) {
        custom_controller.joint[i] = joint[i];
    }
}

void Gimbal_Engineer::AddRoll(const Angle<>& delta) {
    roll_target += delta.toFloat(deg);
}

void Gimbal_Engineer::OpenGripper(const UnitFloat<>& speed) {
    motor.gripper.Open(speed);
}

void Gimbal_Engineer::CloseGripper(const UnitFloat<>& speed, const UnitFloat<>& current) {
    motor.gripper.Close(speed, current);
}

void Gimbal_Engineer::ToggleGripper(const UnitFloat<>& speed, const UnitFloat<>& current) {
    if (motor.gripper.is_close) {
        OpenGripper(speed);
    } else {
        CloseGripper(speed, current);
    }
}

void Gimbal_Engineer::CalibrateJ23() {
    motor.j2.config.offset = motor.j2.raw.angle;
    motor.j3.config.offset = motor.j3.raw.angle;
}

Angle<deg> Gimbal_Engineer::GetJ1Angle() const {
    return motor.j1.angle.measure;
}

void Gimbal_Engineer::UpdateUI(UI& ui) {
    ui.robot.yaw1 = motor.j1.is_connect ? UI::GREEN : UI::PINK;
    ui.robot.yaw2 = motor.j2.is_connect ? UI::GREEN : UI::PINK;
    ui.robot.pitch = motor.j3.is_connect ? UI::GREEN : UI::PINK;
    ui.robot.rub1 = motor.j4.is_connect ? UI::GREEN : UI::PINK;
    ui.robot.rub2 = motor.j5.is_connect ? UI::GREEN : UI::PINK;
    ui.robot.rub3 = motor.j6.is_connect ? UI::GREEN : UI::PINK;
    ui.robot.rub4 = motor.gripper.is_connect ? UI::GREEN : UI::PINK;
}

void Gimbal_Engineer::OnLoop() {
    if (!is_enable) {
        resetTarget();
    } else {
        initJ4Target();

        switch (control_mode) {
            case CARTESIAN_CONTROL:
                handleCartesianMove();
                break;
            case CUSTOM_CONTROLLER_CONTROL:
                handleCustomController();
                break;
            case POLAR_CONTROL:
                handlePolarControl();
                break;
            case JOINT_CONTROL:
            default:
                handleJointControl();
                break;
        }
    }

    applyArmOutput();
    motor.gripper.OnLoop();
    resetControlInput();
}

void Gimbal_Engineer::resetTarget() {
    j1_target = 0 * deg;
    j2_target = config.j2_init;
    j3_target = config.j3_init;
    j4_inited = false;
    end_inited = false;
    pitch_target = config.pitch_init;
    roll_target = config.roll_init;
    cc_roll_inited = false;
}

void Gimbal_Engineer::resetControlInput() {
    control_mode = JOINT_CONTROL;
    joint_control = {};
    polar_control = {};
    cartesian_pose = -1;
    custom_controller.is_connect = false;
    custom_controller.lock = false;
}

void Gimbal_Engineer::initJ4Target() {
    if (j4_inited || !motor.j4.is_connect) return;

    const float cur = motor.j4.angle.measure.toFloat(deg);
    j4_target = roundf(cur / 360.0f) * 360.0f * deg;
    j4_inited = true;
}

void Gimbal_Engineer::handleCartesianMove() {
    if (cartesian_pose < 0) return;

    const float dt = ARM_DT.toFloat(s);

    float motor_q[Arm::DOF] = {
        j1_target.toFloat(rad),
        j2_target.toFloat(rad),
        j3_target.toFloat(rad),
        j4_target.toFloat(rad),
        pitch_target * Arm::DEG2RAD,
        roll_target * Arm::DEG2RAD,
    };
    float dh_q[Arm::DOF];
    Arm::MotorToDH(motor_q, dh_q);
    Arm::Mat4 target = Arm::FK(dh_q);

    float dir[3];
    Arm::ComputeLiftDirection(target, cartesian_pose, dir);
    for (int i = 0; i < 3; i++) {
        target.m[i][3] += dir[i] * config.cartesian_speed * dt;
    }

    Arm::IK6sResult ik_result;
    Arm::IKine6s(target, false, ik_result);

    float cur_q[Arm::DOF] = {
        j1_target.toFloat(rad),
        j2_target.toFloat(rad),
        j3_target.toFloat(rad),
        j4_target.toFloat(rad),
        0.0f,
        0.0f,
    };

    if (ik_result.num_solutions <= 0) return;

    const int best = Arm::SelectBestSolution(ik_result, cur_q);
    if (best < 0) return;

    const float* sol = ik_result.solutions[best];
    float max_jump = 0.0f;
    for (int i = 0; i < 4; i++) {
        float diff = fabsf(sol[i] - cur_q[i]);
        if (diff > M_PI) diff = 2.0f * M_PI - diff;
        if (diff > max_jump) max_jump = diff;
    }

    if (max_jump < config.cartesian_max_jump) {
        j1_target = sol[0] * rad;
        j2_target = sol[1] * rad;
        j3_target = sol[2] * rad;
        j4_target = sol[3] * rad;
    }
}

void Gimbal_Engineer::handlePolarControl() {
    static constexpr float VR_MAX = 500.0f;
    static constexpr float VTHETA_MAX = 4.0f;
    static constexpr float VZ_MAX = 500.0f;
    static constexpr float Z_MIN = -200.0f;
    static constexpr float Z_MAX = 600.0f;
    static constexpr float MAX_JOINT_JUMP = 30.0f * M_PI / 180.0f;

    if (!end_inited) {
        float init_q[Arm::DOF] = {
            0.0f,
            config.j2_init.toFloat(rad),
            config.j3_init.toFloat(rad),
            0.0f,
            0.0f,
            0.0f,
        };
        float dh_q[Arm::DOF];
        Arm::MotorToDH(init_q, dh_q);
        Arm::Mat4 T = Arm::FK(dh_q);
        float ix, iy, iz;
        Arm::GetPosition(T, ix, iy, iz);
        end_r = sqrtf(ix * ix + iy * iy);
        end_theta = atan2f(iy, ix);
        end_z = iz;
        end_inited = true;
    }

    const float dt = ARM_DT.toFloat(s);
    const float prev_r = end_r;
    const float prev_theta = end_theta;
    const float prev_z = end_z;

    end_r += polar_control.r.toFloat(ratio) * VR_MAX * dt;
    end_theta += polar_control.theta.toFloat(ratio) * VTHETA_MAX * dt;
    end_z += polar_control.z.toFloat(ratio) * VZ_MAX * dt;

    if (end_r < 50.0f) end_r = 50.0f;
    if (end_r > 750.0f) end_r = 750.0f;
    if (end_z < Z_MIN) end_z = Z_MIN;
    if (end_z > Z_MAX) end_z = Z_MAX;

    end_x = end_r * cosf(end_theta);
    end_y = end_r * sinf(end_theta);

    Arm::Mat4 target_pose = Arm::BuildPose(end_x, end_y, end_z, 0.0f, 0.0f, 0.0f);
    Arm::IK6sResult ik_result;
    Arm::IKine6s(target_pose, false, ik_result);

    float cur_q[Arm::DOF] = {
        j1_target.toFloat(rad),
        j2_target.toFloat(rad),
        j3_target.toFloat(rad),
        j4_target.toFloat(rad),
        0.0f,
        0.0f,
    };

    bool ik_accepted = false;
    if (ik_result.num_solutions > 0) {
        const int best = Arm::SelectBestSolution(ik_result, cur_q);
        if (best >= 0) {
            const float* sol = ik_result.solutions[best];
            float max_jump = 0.0f;
            for (int i = 0; i < 4; i++) {
                float diff = fabsf(sol[i] - cur_q[i]);
                if (diff > M_PI) diff = 2.0f * M_PI - diff;
                if (diff > max_jump) max_jump = diff;
            }
            if (max_jump < MAX_JOINT_JUMP) {
                j1_target = sol[0] * rad;
                j2_target = sol[1] * rad;
                j3_target = sol[2] * rad;
                j4_target = sol[3] * rad;
                ik_accepted = true;
            }
        }
    }

    if (!ik_accepted) {
        end_r = prev_r;
        end_theta = prev_theta;
        end_z = prev_z;
        end_x = end_r * cosf(end_theta);
        end_y = end_r * sinf(end_theta);
    }

    pitch_target += polar_control.pitch.toFloat(ratio) * config.pitch_speed_max * dt;
    roll_target += polar_control.roll.toFloat(ratio) * config.roll_speed_max * dt;
}

void Gimbal_Engineer::handleCustomController() {
    if (!custom_controller.is_connect || custom_controller.lock) {
        cc_roll_inited = false;
        return;
    }

    float dh_q[Arm::DOF];
    for (int i = 0; i < Arm::DOF; i++) {
        dh_q[i] = custom_controller.joint[i].toFloat(rad);
    }

    float motor_q[Arm::DOF];
    Arm::DHToMotor(dh_q, motor_q);

    j1_target = motor_q[0] * rad;
    j2_target = motor_q[1] * rad;
    j3_target = motor_q[2] * rad;
    j4_target = motor_q[3] * rad;
    pitch_target = motor_q[4] * Arm::RAD2DEG;

    const float cc_roll = motor_q[5] * Arm::RAD2DEG;
    if (cc_roll_inited) {
        roll_target += cc_roll - cc_roll_prev;
    } else {
        cc_roll_inited = true;
    }
    cc_roll_prev = cc_roll;
}

void Gimbal_Engineer::handleJointControl() {
    cc_roll_inited = false;
    j2_target += joint_control.j2 * J23_MAX_SPEED * ARM_DT;
    j4_target += joint_control.j4 * J4_MAX_SPEED * ARM_DT;
}

void Gimbal_Engineer::applyArmOutput() {
    static constexpr Angle<deg> J2_MIN = -47.5 * deg;
    static constexpr Angle<deg> J2_MAX = 45 * deg;
    if (j2_target < J2_MIN) j2_target = J2_MIN;
    if (j2_target > J2_MAX) j2_target = J2_MAX;
    motor.j2.SetAngle(j2_target);

    motor.j1.SetAngle(j1_target);
    motor.j4.SetAngle(j4_target);

    if (pitch_target < -90.0f) pitch_target = -90.0f;
    if (pitch_target > 90.0f) pitch_target = 90.0f;

    const float dp = (pitch_target - config.pitch_init) * config.pitch_factor;
    const float dr = (roll_target - config.roll_init) * config.roll_factor;
    motor.j5.SetAngle((dp + dr) / 2.0f * deg);
    motor.j6.SetAngle((dr - dp) / 2.0f * deg);

    static constexpr Angle<deg> J3_ABS_MIN = 0 * deg;
    Angle<deg> j3_max = +config.j3_init - config.j2_init + motor.j2.angle.measure;
    if (j3_target < J3_ABS_MIN) j3_target = J3_ABS_MIN;
    if (j3_target > j3_max) j3_target = j3_max;
    motor.j3.SetAngle(j3_target);

    static constexpr float COUPLING = 0.055f;
    if (++motor_loop_cnt >= 2) {
        motor_loop_cnt = 0;
        motor.j1.OnLoop();
        motor.j2.OnLoop();
        motor.j3.OnLoop();
        motor.j4.OnLoop();
        motor.j5.OnLoop();
        motor.j6.OnLoop();

        const float sin_j2 = sinf(motor.j2.angle.measure.toFloat(rad));
        const float sin_j3 = sinf(motor.j3.angle.measure.toFloat(rad));
        motor.j2.SetTorque(unit::clamp(
            motor.j2.torque.ref + (config.gravity_c2 * sin_j2 + COUPLING * config.gravity_c3 * sin_j3) * Nm,
            config.torque_limit));
        motor.j3.SetTorque(unit::clamp(
            motor.j3.torque.ref + (config.gravity_c3 * sin_j3) * Nm,
            config.torque_limit));
    }
}
