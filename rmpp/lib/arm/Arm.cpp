#include "Arm.hpp"
#include <cstring>

namespace Arm {

// ============================================================
//  Mat4 实现
// ============================================================

Mat4 Mat4::operator*(const Mat4& rhs) const {
    Mat4 result = {};
    for (int i = 0; i < 4; i++)
        for (int j = 0; j < 4; j++) {
            float sum = 0.0f;
            for (int k = 0; k < 4; k++)
                sum += m[i][k] * rhs.m[k][j];
            result.m[i][j] = sum;
        }
    return result;
}

Mat4 Mat4::Identity() {
    Mat4 I = {};
    I.m[0][0] = 1.0f;
    I.m[1][1] = 1.0f;
    I.m[2][2] = 1.0f;
    I.m[3][3] = 1.0f;
    return I;
}

// ============================================================
//  static 矩阵/向量工具
// ============================================================

static float normalize_angle(float a) {
    while (a >  M_PI) a -= 2.0f * M_PI;
    while (a < -M_PI) a += 2.0f * M_PI;
    return a;
}

static void mat6_multiply(const Mat6& A, const Mat6& B, Mat6& C) {
    Mat6 tmp;
    for (int i = 0; i < 6; i++)
        for (int j = 0; j < 6; j++) {
            float s = 0.0f;
            for (int k = 0; k < 6; k++)
                s += A.m[i][k] * B.m[k][j];
            tmp.m[i][j] = s;
        }
    memcpy(C.m, tmp.m, sizeof(C.m));
}

static void mat6_transpose(const Mat6& A, Mat6& At) {
    for (int i = 0; i < 6; i++)
        for (int j = 0; j < 6; j++)
            At.m[j][i] = A.m[i][j];
}

static void mat6_multiply_vec(const Mat6& M, const Vec6& v, Vec6& out) {
    Vec6 tmp;
    for (int i = 0; i < 6; i++) {
        float s = 0.0f;
        for (int j = 0; j < 6; j++)
            s += M.m[i][j] * v.d[j];
        tmp.d[i] = s;
    }
    memcpy(out.d, tmp.d, sizeof(out.d));
}

static int mat6_inverse(const Mat6& M, Mat6& inv) {
    float aug[6][12];
    for (int i = 0; i < 6; i++)
        for (int j = 0; j < 6; j++) {
            aug[i][j] = M.m[i][j];
            aug[i][j + 6] = (i == j) ? 1.0f : 0.0f;
        }

    for (int i = 0; i < 6; i++) {
        int max_row = i;
        for (int k = i + 1; k < 6; k++)
            if (fabsf(aug[k][i]) > fabsf(aug[max_row][i]))
                max_row = k;
        if (max_row != i)
            for (int j = 0; j < 12; j++) {
                float tmp = aug[i][j];
                aug[i][j] = aug[max_row][j];
                aug[max_row][j] = tmp;
            }
        if (fabsf(aug[i][i]) < 1e-10f)
            return -1;
        float pivot = aug[i][i];
        for (int j = 0; j < 12; j++)
            aug[i][j] /= pivot;
        for (int k = 0; k < 6; k++) {
            if (k == i) continue;
            float factor = aug[k][i];
            for (int j = 0; j < 12; j++)
                aug[k][j] -= factor * aug[i][j];
        }
    }

    for (int i = 0; i < 6; i++)
        for (int j = 0; j < 6; j++)
            inv.m[i][j] = aug[i][j + 6];
    return 0;
}

static int mat6_pseudo_inverse(const Mat6& A, Mat6& A_pinv, float damping) {
    if (damping < 1e-10f) damping = 1e-6f;

    Mat6 At, AtA, damped, damped_inv;
    mat6_transpose(A, At);
    mat6_multiply(At, A, AtA);

    for (int i = 0; i < 6; i++)
        for (int j = 0; j < 6; j++)
            damped.m[i][j] = AtA.m[i][j] + ((i == j) ? damping : 0.0f);

    if (mat6_inverse(damped, damped_inv) != 0)
        return -1;

    mat6_multiply(damped_inv, At, A_pinv);
    return 0;
}

static void vec3_cross(const Vec3& a, const Vec3& b, Vec3& out) {
    out.d[0] = a.d[1] * b.d[2] - a.d[2] * b.d[1];
    out.d[1] = a.d[2] * b.d[0] - a.d[0] * b.d[2];
    out.d[2] = a.d[0] * b.d[1] - a.d[1] * b.d[0];
}

static void vec3_sub(const Vec3& a, const Vec3& b, Vec3& out) {
    out.d[0] = a.d[0] - b.d[0];
    out.d[1] = a.d[1] - b.d[1];
    out.d[2] = a.d[2] - b.d[2];
}

static float position_error(const Mat4& T1, const Mat4& T2) {
    float dx = T1.m[0][3] - T2.m[0][3];
    float dy = T1.m[1][3] - T2.m[1][3];
    float dz = T1.m[2][3] - T2.m[2][3];
    return sqrtf(dx * dx + dy * dy + dz * dz);
}

static void compute_pose_error(const Mat4& Te, const Mat4& Tep, Vec6& e) {
    e.d[0] = Tep.m[0][3] - Te.m[0][3];
    e.d[1] = Tep.m[1][3] - Te.m[1][3];
    e.d[2] = Tep.m[2][3] - Te.m[2][3];

    float R[3][3];
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++) {
            R[i][j] = 0.0f;
            for (int k = 0; k < 3; k++)
                R[i][j] += Tep.m[i][k] * Te.m[j][k];
        }

    float li[3];
    li[0] = R[2][1] - R[1][2];
    li[1] = R[0][2] - R[2][0];
    li[2] = R[1][0] - R[0][1];

    float li_norm = sqrtf(li[0] * li[0] + li[1] * li[1] + li[2] * li[2]);
    float R_tr = R[0][0] + R[1][1] + R[2][2];

    if (li_norm < 1e-6f) {
        if (R_tr > 0) {
            e.d[3] = e.d[4] = e.d[5] = 0.0f;
        } else {
            e.d[3] = M_PI_2 * (R[0][0] + 1.0f);
            e.d[4] = M_PI_2 * (R[1][1] + 1.0f);
            e.d[5] = M_PI_2 * (R[2][2] + 1.0f);
        }
    } else {
        float ang = atan2f(li_norm, R_tr - 1.0f);
        float s = ang / li_norm;
        e.d[3] = s * li[0];
        e.d[4] = s * li[1];
        e.d[5] = s * li[2];
    }
}

// ============================================================
//  正运动学
// ============================================================

Mat4 DHTransform(const DHLink& link, float theta) {
    const float q = theta + link.offset;
    const float cq = cosf(q);
    const float sq = sinf(q);
    const float ca = cosf(link.alpha);
    const float sa = sinf(link.alpha);

    Mat4 T = {};
    T.m[0][0] = cq;       T.m[0][1] = -sq * ca;  T.m[0][2] =  sq * sa;  T.m[0][3] = link.a * cq;
    T.m[1][0] = sq;       T.m[1][1] =  cq * ca;  T.m[1][2] = -cq * sa;  T.m[1][3] = link.a * sq;
    T.m[2][0] = 0.0f;     T.m[2][1] =  sa;       T.m[2][2] =  ca;       T.m[2][3] = link.d;
    T.m[3][0] = 0.0f;     T.m[3][1] =  0.0f;     T.m[3][2] =  0.0f;     T.m[3][3] = 1.0f;
    return T;
}

Mat4 FK(const float theta[DOF]) {
    Mat4 T = Mat4::Identity();
    for (int i = 0; i < DOF; i++)
        T = T * DHTransform(dh_links[i], theta[i]);
    return T;
}

void FK_all(const float theta[DOF], Mat4 T_links[DOF]) {
    Mat4 T = Mat4::Identity();
    for (int i = 0; i < DOF; i++) {
        T = T * DHTransform(dh_links[i], theta[i]);
        T_links[i] = T;
    }
}

void GetPosition(const Mat4& T, float& x, float& y, float& z) {
    x = T.m[0][3];
    y = T.m[1][3];
    z = T.m[2][3];
}

Mat4 BuildPose(float x, float y, float z,
               float roll_deg, float pitch_deg, float yaw_deg) {
    float r = roll_deg  * DEG2RAD;
    float p = pitch_deg * DEG2RAD;
    float w = yaw_deg   * DEG2RAD;

    float cr = cosf(r), sr = sinf(r);
    float cp = cosf(p), sp = sinf(p);
    float cw = cosf(w), sw = sinf(w);

    Mat4 T = {};
    T.m[0][0] = cw * cp;
    T.m[0][1] = cw * sp * sr - sw * cr;
    T.m[0][2] = cw * sp * cr + sw * sr;
    T.m[0][3] = x;
    T.m[1][0] = sw * cp;
    T.m[1][1] = sw * sp * sr + cw * cr;
    T.m[1][2] = sw * sp * cr - cw * sr;
    T.m[1][3] = y;
    T.m[2][0] = -sp;
    T.m[2][1] = cp * sr;
    T.m[2][2] = cp * cr;
    T.m[2][3] = z;
    T.m[3][0] = 0.0f; T.m[3][1] = 0.0f; T.m[3][2] = 0.0f; T.m[3][3] = 1.0f;
    return T;
}

// ============================================================
//  电机角度 <-> DH 角度转换
// ============================================================

void MotorToDH(const float motor[DOF], float dh[DOF]) {
    for (int i = 0; i < DOF; i++) dh[i] = motor[i];
    dh[2] = motor[2] - motor[1];
}

void DHToMotor(const float dh[DOF], float motor[DOF]) {
    for (int i = 0; i < DOF; i++) motor[i] = dh[i];
    motor[2] = dh[2] + dh[1];
}

// ============================================================
//  选解
// ============================================================

static const float joint_weights[DOF] = {
    3.0f, 3.0f, 3.0f, 1.0f, 1.0f, 1.0f
};

int SelectBestSolution(const IK6sResult& result, const float current_motor_q[DOF]) {
    if (result.num_solutions <= 0) return -1;

    int best = 0;
    float best_cost = 1e30f;

    for (int s = 0; s < result.num_solutions; s++) {
        float cost = 0.0f;
        for (int j = 0; j < DOF; j++) {
            float diff = normalize_angle(result.solutions[s][j] - current_motor_q[j]);
            cost += joint_weights[j] * diff * diff;
        }
        if (cost < best_cost) {
            best_cost = cost;
            best = s;
        }
    }
    return best;
}

// ============================================================
//  关节限位检查
// ============================================================

bool CheckLimits(const float motor_q[DOF]) {
    for (int i = 0; i < DOF; i++) {
        if (i == 2) {
            if (motor_q[2] < robot.q_min[2] - 1e-6f)
                return false;
            float rel = motor_q[2] - motor_q[1];
            if (rel < robot.q_max[2] - 1e-6f)
                return false;
            continue;
        }
        if (motor_q[i] < robot.q_min[i] - 1e-6f || motor_q[i] > robot.q_max[i] + 1e-6f)
            return false;
    }
    return true;
}

// ============================================================
//  雅可比矩阵
// ============================================================

void ComputeJacobian(const float q[DOF], Mat6& J) {
    Mat4 T_links[DOF];
    FK_all(q, T_links);

    Vec3 p_end;
    p_end.d[0] = T_links[DOF - 1].m[0][3];
    p_end.d[1] = T_links[DOF - 1].m[1][3];
    p_end.d[2] = T_links[DOF - 1].m[2][3];

    for (int i = 0; i < DOF; i++) {
        Vec3 z_i, p_i, p_diff, jv;

        if (i == 0) {
            z_i = {0.0f, 0.0f, 1.0f};
            p_i = {0.0f, 0.0f, 0.0f};
        } else {
            z_i.d[0] = T_links[i - 1].m[0][2];
            z_i.d[1] = T_links[i - 1].m[1][2];
            z_i.d[2] = T_links[i - 1].m[2][2];
            p_i.d[0] = T_links[i - 1].m[0][3];
            p_i.d[1] = T_links[i - 1].m[1][3];
            p_i.d[2] = T_links[i - 1].m[2][3];
        }

        vec3_sub(p_end, p_i, p_diff);
        vec3_cross(z_i, p_diff, jv);

        J.m[0][i] = jv.d[0];
        J.m[1][i] = jv.d[1];
        J.m[2][i] = jv.d[2];
        J.m[3][i] = z_i.d[0];
        J.m[4][i] = z_i.d[1];
        J.m[5][i] = z_i.d[2];
    }
}

// ============================================================
//  IKine6s — 解析法逆解
// ============================================================

void IKine6s(const Mat4& target, bool check_limits, IK6sResult& result) {
    result.num_solutions = 0;

    const float d1 = robot.links[0].d;
    const float a2 = robot.links[1].a;
    const float d3 = robot.links[2].d;
    const float d4 = robot.links[3].d;

    const float Px = target.m[0][3];
    const float Py = target.m[1][3];
    const float Pz = target.m[2][3];

    float R_sq = Px * Px + Py * Py;
    if (R_sq < d3 * d3) return;

    float R   = sqrtf(R_sq);
    float phi = atan2f(Py, Px);
    float psi = asinf(d3 / R);

    float q1_list[2];
    q1_list[0] = normalize_angle(phi - psi);
    q1_list[1] = normalize_angle(phi + psi + M_PI);

    for (int iq1 = 0; iq1 < 2; iq1++) {
        float q1 = q1_list[iq1];
        float x1 = Px * cosf(q1) + Py * sinf(q1);
        float y1 = d1 - Pz;

        float D_sq = x1 * x1 + y1 * y1;
        float cos3 = (D_sq - a2 * a2 - d4 * d4) / (2.0f * a2 * d4);

        if (fabsf(cos3) > 1.0f + 1e-6f) continue;
        if (cos3 >  1.0f) cos3 =  1.0f;
        if (cos3 < -1.0f) cos3 = -1.0f;

        float q3_list[2];
        q3_list[0] =  acosf(cos3);
        q3_list[1] = -acosf(cos3);

        for (int iq3 = 0; iq3 < 2; iq3++) {
            float q3 = q3_list[iq3];

            float K1 = a2 + d4 * cosf(q3);
            float K2 = d4 * sinf(q3);
            float s2 = (K1 * x1 + K2 * y1) / D_sq;
            float c2 = (K2 * x1 - K1 * y1) / D_sq;
            float q2 = atan2f(s2, c2);

            Mat4 T01 = DHTransform(robot.links[0], q1);
            Mat4 T12 = DHTransform(robot.links[1], q2);
            Mat4 T23 = DHTransform(robot.links[2], q3);
            Mat4 T02 = T01 * T12;
            Mat4 T03 = T02 * T23;

            float R36[3][3];
            for (int i = 0; i < 3; i++)
                for (int j = 0; j < 3; j++) {
                    R36[i][j] = 0.0f;
                    for (int k = 0; k < 3; k++)
                        R36[i][j] += T03.m[k][i] * target.m[k][j];
                }

            float cos5 = R36[2][2];
            if (cos5 >  1.0f) cos5 =  1.0f;
            if (cos5 < -1.0f) cos5 = -1.0f;

            float q5_list[2];
            q5_list[0] =  acosf(cos5);
            q5_list[1] = -acosf(cos5);

            for (int iq5 = 0; iq5 < 2; iq5++) {
                float q5 = q5_list[iq5];
                float q4, q6;
                float s5 = sinf(q5);

                if (fabsf(s5) < 1e-6f) {
                    q4 = 0.0f;
                    q6 = atan2f(-R36[0][1], R36[0][0]);
                } else {
                    q4 = atan2f(R36[1][2] / s5, R36[0][2] / s5);
                    q6 = atan2f(R36[2][1] / s5, -R36[2][0] / s5);
                }

                float dh_sol[DOF];
                dh_sol[0] = normalize_angle(q1);
                dh_sol[1] = normalize_angle(q2);
                dh_sol[2] = normalize_angle(q3);
                dh_sol[3] = normalize_angle(q4);
                dh_sol[4] = normalize_angle(q5);
                dh_sol[5] = normalize_angle(q6);

                Mat4 Tcheck = FK(dh_sol);
                if (position_error(Tcheck, target) > 1.0f)
                    continue;

                float motor_sol[DOF];
                DHToMotor(dh_sol, motor_sol);

                if (check_limits && !CheckLimits(motor_sol))
                    continue;

                if (result.num_solutions < MAX_IK_SOLUTIONS) {
                    memcpy(result.solutions[result.num_solutions], motor_sol, sizeof(float) * DOF);
                    result.num_solutions++;
                }
            }
        }
    }
}

// ============================================================
//  IKineLM — 数值法逆解
// ============================================================

static const float lm_perturbations[6] = {
    0.5f, -0.8f, 1.2f, -0.3f, 0.9f, -1.1f
};

void IKineLM(const Mat4& target, const float* initial_guess,
             const IKConfig& cfg, IKResult& result) {
    float q[DOF];
    result.success = false;
    result.iterations = 0;
    result.final_error = 1e30f;

    if (initial_guess) {
        MotorToDH(initial_guess, q);
    } else {
        for (int i = 0; i < DOF; i++) q[i] = 0.0f;
    }

    for (int search = 0; search <= cfg.max_searches; search++) {
        for (int iter = 0; iter < cfg.max_iterations; iter++) {
            result.iterations++;

            Mat4 Te = FK(q);
            Vec6 e;
            compute_pose_error(Te, target, e);

            float E = 0.0f;
            for (int i = 0; i < 6; i++) E += e.d[i] * e.d[i];
            E *= 0.5f;
            result.final_error = E;

            if (E < cfg.tolerance) {
                for (int i = 0; i < DOF; i++)
                    q[i] = normalize_angle(q[i]);
                result.success = true;
                goto done;
            }

            Mat6 J, Jt, JtJ, A, A_inv;
            ComputeJacobian(q, J);
            mat6_transpose(J, Jt);
            mat6_multiply(Jt, J, JtJ);

            float d = cfg.lambda * E;
            for (int i = 0; i < 6; i++)
                for (int j = 0; j < 6; j++)
                    A.m[i][j] = JtJ.m[i][j] + ((i == j) ? d : 0.0f);

            Vec6 g, dq;
            mat6_multiply_vec(Jt, e, g);

            if (mat6_inverse(A, A_inv) != 0)
                break;
            mat6_multiply_vec(A_inv, g, dq);

            for (int i = 0; i < DOF; i++)
                q[i] += dq.d[i];
        }

        for (int i = 0; i < DOF; i++)
            q[i] = lm_perturbations[(search + i) % 6] * (float)(search + 1) * 0.3f;
    }

    for (int i = 0; i < DOF; i++)
        q[i] = normalize_angle(q[i]);

done:
    DHToMotor(q, result.joint_angles);
}

// ============================================================
//  IKineVel — 速度逆解
// ============================================================

int IKineVel(const float motor_q[DOF], const Vec6& xdot, float damping,
             float motor_qdot[DOF]) {
    float dh_q[DOF];
    MotorToDH(motor_q, dh_q);

    Mat6 J, J_pinv;
    ComputeJacobian(dh_q, J);

    if (damping < 1e-12f) damping = 1e-6f;

    if (mat6_pseudo_inverse(J, J_pinv, damping) != 0) {
        for (int i = 0; i < DOF; i++) motor_qdot[i] = 0.0f;
        return -1;
    }

    Vec6 dh_qdot;
    mat6_multiply_vec(J_pinv, xdot, dh_qdot);

    DHToMotor(dh_qdot.d, motor_qdot);
    return 0;
}

// ============================================================
//  抬升方向计算
// ============================================================

void ComputeLiftDirection(const Mat4& T_ee, int pose, float dir[3]) {
    float sign = 1.0f;
    int col = 2;
    if (pose == 0) {
        col = 1;                // 0°  → FK Y+
    } else if (pose == 1) {
        col = 2;                // 左90° → FK Z
    } else if (pose == 2) {
        col = 1; sign = -1.0f;  // 右90° → FK -Y
    } else if (pose == 3) {
        col = 0;                // 末端X+
    } else {
        col = 0; sign = -1.0f;  // 末端X-
    }

    float len = 0.0f;
    for (int i = 0; i < 3; i++) {
        dir[i] = sign * T_ee.m[i][col];
        len += dir[i] * dir[i];
    }
    len = sqrtf(len);
    if (len > 1e-6f)
        for (int i = 0; i < 3; i++) dir[i] /= len;
}

} // namespace Arm
