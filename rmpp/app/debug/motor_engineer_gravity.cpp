#include "motor_engineer_gravity.hpp"

#include <cmath>

void send_can_cmd() {
    gravity_j2.SendCanCmd();
    BSP::Dwt::Delay(216 * us);
    gravity_j3.SendCanCmd();
}

void setup() {
    BSP::Init();
    gravity_j2.SetEnable(true);
    gravity_j3.SetEnable(true);
}

void loop() {
    gravity_j2.SetEnable(true);
    gravity_j3.SetEnable(true);

    gravity_j2.SetAngle(J2_TARGET);
    gravity_j3.SetAngle(J3_TARGET);

    gravity_j2.OnLoop();
    gravity_j3.OnLoop();

    const float sin_j2 = sinf(gravity_j2.angle.measure.toFloat(rad));
    const float sin_j3 = sinf(gravity_j3.angle.measure.toFloat(rad));

    const UnitFloat<> j2_gravity = (GRAVITY_C2 * sin_j2 + GRAVITY_COUPLING * GRAVITY_C3 * sin_j3) * Nm;
    const UnitFloat<> j3_gravity = (GRAVITY_C3 * sin_j3) * Nm;

    // gravity_j2.SetTorque(unit::clamp(gravity_j2.torque.ref + j2_gravity, GRAVITY_TORQUE_LIMIT));
    // gravity_j3.SetTorque(unit::clamp(gravity_j3.torque.ref + j3_gravity, GRAVITY_TORQUE_LIMIT));

    // Gravity only test
    gravity_j2.SetTorque(unit::clamp(j2_gravity, GRAVITY_TORQUE_LIMIT));
    gravity_j3.SetTorque(unit::clamp(j3_gravity, GRAVITY_TORQUE_LIMIT));

    send_can_cmd();
}

extern "C" void rmpp_main() {
    setup();

    BSP::Dwt dwt;
    while (true) {
        if (dwt.PollTimeout(1 * ms)) {
            loop();
        }
    }
}
