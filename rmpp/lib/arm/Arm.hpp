#pragma once

#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif
#ifndef M_PI_2
#define M_PI_2 1.57079632679489661923f
#endif

namespace Arm {

static constexpr int DOF = 6;
static constexpr int MAX_IK_SOLUTIONS = 8;
static constexpr float DEG2RAD = M_PI / 180.0f;
static constexpr float RAD2DEG = 180.0f / M_PI;

// ---- 数据结构 ----

struct DHLink {
    float d;
    float a;
    float alpha;
    float offset;
};

struct Mat4 {
    float m[4][4];
    Mat4 operator*(const Mat4& rhs) const;
    static Mat4 Identity();
};

struct Mat6 {
    float m[6][6];
};

struct Vec6 {
    float d[6];
};

struct Vec3 {
    float d[3];
};

struct Robot6DOF {
    DHLink links[DOF];
    float q_min[DOF];
    float q_max[DOF];
};

struct IK6sResult {
    float solutions[MAX_IK_SOLUTIONS][DOF];
    int num_solutions;
};

struct IKResult {
    float joint_angles[DOF];
    bool success;
    int iterations;
    float final_error;
};

struct IKConfig {
    int max_iterations = 100;
    int max_searches = 50;
    float tolerance = 1e-4f;
    float lambda = 1.0f;
};

// ---- 全局常量（由应用层定义） ----

extern const DHLink dh_links[DOF];
extern const Robot6DOF robot;

// ---- 正运动学 ----

Mat4 DHTransform(const DHLink& link, float theta);
Mat4 FK(const float theta[DOF]);
void FK_all(const float theta[DOF], Mat4 T_links[DOF]);
void GetPosition(const Mat4& T, float& x, float& y, float& z);
Mat4 BuildPose(float x, float y, float z, float roll_deg, float pitch_deg, float yaw_deg);

// ---- 雅可比 ----

void ComputeJacobian(const float q[DOF], Mat6& J);

// ---- 逆运动学 ----

void IKine6s(const Mat4& target, bool check_limits, IK6sResult& result);
void IKineLM(const Mat4& target, const float* initial_guess, const IKConfig& cfg, IKResult& result);
int IKineVel(const float q[DOF], const Vec6& xdot, float damping, float qdot[DOF]);

// ---- 电机角度 <-> DH 角度转换 ----

void MotorToDH(const float motor[DOF], float dh[DOF]);
void DHToMotor(const float dh[DOF], float motor[DOF]);

// ---- 选解 ----

int SelectBestSolution(const IK6sResult& result, const float current_motor_q[DOF]);

// ---- 工具 ----

bool CheckLimits(const float motor_q[DOF]);
void ComputeLiftDirection(const Mat4& T_ee, int pose, float dir[3]);

} // namespace Arm
