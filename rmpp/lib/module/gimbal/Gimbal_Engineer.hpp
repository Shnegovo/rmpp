#pragma once

#include "arm/Arm.hpp"
#include "gripper/gripper.hpp"
#include "motor/Motor.hpp"
#include "ui/UI.hpp"

class Gimbal_Engineer {
public:
    struct config_t {
        Angle<deg> j2_init, j3_init;
        float pitch_init = 0.0f;
        float roll_init = 0.0f;
        float pitch_factor = 1.0f;
        float roll_factor = 1.0f;
        float pitch_speed_max = 0.0f;
        float roll_speed_max = 0.0f;

        float gravity_c2 = 0.0f;
        float gravity_c3 = 0.0f;
        UnitFloat<Nm> torque_limit;

        float cartesian_speed = 0.0f;
        float cartesian_max_jump = 0.0f;
    } config;

    struct motor_t {
        Motor &j1, &j2, &j3, &j4, &j5, &j6;
        Gripper& gripper;
    } motor;

    Gimbal_Engineer(const config_t& config, const motor_t& motor);

    void SetEnable(bool is_enable);

    void SetJointControl(const UnitFloat<>& j2_axis, const UnitFloat<>& j4_axis);

    void SetPolarControl(const UnitFloat<>& r_axis,
                         const UnitFloat<>& theta_axis,
                         const UnitFloat<>& z_axis,
                         const UnitFloat<>& pitch_axis,
                         const UnitFloat<>& roll_axis);

    void SetCartesianMove(int pose);

    void SetCustomController(bool is_connect, bool lock, const Angle<deg> joint[Arm::DOF]);

    void AddRoll(const Angle<>& delta);

    void OpenGripper(const UnitFloat<>& speed);

    void CloseGripper(const UnitFloat<>& speed, const UnitFloat<>& current);

    void ToggleGripper(const UnitFloat<>& speed, const UnitFloat<>& current);

    void CalibrateJ23();

    Angle<deg> GetJ1Angle() const;

    void UpdateUI(UI& ui);

    void OnLoop();

private:
    static constexpr UnitFloat<ms> ARM_DT = 1 * ms;
    static constexpr UnitFloat<deg_s> J23_MAX_SPEED = 120 * deg_s;
    static constexpr UnitFloat<deg_s> J4_MAX_SPEED = 180 * deg_s;

    enum control_mode_e {
        JOINT_CONTROL,
        POLAR_CONTROL,
        CARTESIAN_CONTROL,
        CUSTOM_CONTROLLER_CONTROL,
    } control_mode = JOINT_CONTROL;

    bool is_enable = false;

    Angle<deg> j1_target = 0 * deg;
    Angle<deg> j2_target;
    Angle<deg> j3_target;
    Angle<deg> j4_target = 0 * deg;
    bool j4_inited = false;

    float pitch_target = 0.0f;
    float roll_target = 0.0f;

    float end_x = 0.0f;
    float end_y = 0.0f;
    float end_z = 0.0f;
    float end_r = 0.0f;
    float end_theta = 0.0f;
    bool end_inited = false;

    float cc_roll_prev = 0.0f;
    bool cc_roll_inited = false;

    uint8_t motor_loop_cnt = 0;

    struct {
        UnitFloat<ratio> j2;
        UnitFloat<ratio> j4;
    } joint_control;

    struct {
        UnitFloat<ratio> r;
        UnitFloat<ratio> theta;
        UnitFloat<ratio> z;
        UnitFloat<ratio> pitch;
        UnitFloat<ratio> roll;
    } polar_control;

    int cartesian_pose = -1;

    struct {
        bool is_connect = false;
        bool lock = false;
        Angle<deg> joint[Arm::DOF];
    } custom_controller;

    void resetTarget();

    void resetControlInput();

    void initJ4Target();

    void handleCartesianMove();

    void handlePolarControl();

    void handleCustomController();

    void handleJointControl();

    void applyArmOutput();
};
