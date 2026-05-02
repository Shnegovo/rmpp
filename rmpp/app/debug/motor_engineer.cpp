#include "motor_engineer.hpp"

void send_can_cmd() {
    const int16_t cmd = motor.GetCanCmd();
    uint8_t data[8];

   //j6
    data[0] = cmd >> 8;
    data[1] = cmd;
    data[2] = 0;
    data[3] = 0;
    data[4] = 0;
    data[5] = 0;
    data[6] = 0;
    data[7] = 0;


    BSP::CAN::Transmit(1, 0x200, data, 8);
}

void setup() {
    BSP::Init();
    motor.SetEnable(true);
}

void loop() {
    motor.SetEnable(true);

    // OPEN_LOOP_MODE:
    motor.SetCurrent(0.10f * A);
    // motor.SetTorque(0.02f * Nm);

    // SPEED_MODE:
    // motor.SetSpeed(60.0f * deg_s);

    // ANGLE_MODE:
    // motor.SetAngle(30.0f * deg);

    // ANGLE_SPEED_MODE:
    // motor.SetAngle(30.0f * deg, 0.0f * deg_s);

    motor.OnLoop();
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
