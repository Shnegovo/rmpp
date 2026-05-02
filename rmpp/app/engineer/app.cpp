#include "misc.hpp"
#include "chassis.hpp"
#include "gimbal.hpp"
#include "robot.hpp"

void send_can_cmd() {
    ui.SendCanCmd();
    BSP::Dwt::Delay(108 * us);

    uint8_t data[8];

    const int16_t cmd_j6 = j6.GetCanCmd();
    const int16_t cmd_j5 = j5.GetCanCmd();
    data[0] = cmd_j6 >> 8;
    data[1] = cmd_j6;
    data[2] = cmd_j5 >> 8;
    data[3] = cmd_j5;
    data[4] = 0;
    data[5] = 0;
    data[6] = 0;
    data[7] = 0;
    BSP::CAN::Transmit(1, 0x200, data);

    BSP::Dwt::Delay(108 * us);

    static uint8_t gripper_cnt = 0;
    if (++gripper_cnt >= 100) {
        gripper_cnt = 0;
        gripper.SendCanCmd();
        BSP::Dwt::Delay(108 * us);
    }

    static uint16_t can_slot = 0;
    can_slot++;

    if (can_slot % 2 == 0) {
        const int16_t cmd1 = w1.GetCanCmd();
        const int16_t cmd2 = w2.GetCanCmd();
        const int16_t cmd3 = w3.GetCanCmd();
        const int16_t cmd4 = w4.GetCanCmd();
        data[0] = cmd1 >> 8;
        data[1] = cmd1;
        data[2] = cmd2 >> 8;
        data[3] = cmd2;
        data[4] = cmd3 >> 8;
        data[5] = cmd3;
        data[6] = cmd4 >> 8;
        data[7] = cmd4;
        BSP::CAN::Transmit(2, 0x200, data);

        BSP::Dwt::Delay(216 * us);
        j2.SendCanCmd();

        if (can_slot % 4 == 0) {
            BSP::Dwt::Delay(216 * us);
            const int16_t cmd_j1 = j1_motor.GetCanCmd();
            data[0] = cmd_j1 >> 8;
            data[1] = cmd_j1;
            data[2] = 0;
            data[3] = 0;
            data[4] = 0;
            data[5] = 0;
            data[6] = 0;
            data[7] = 0;
            BSP::CAN::Transmit(2, 0x1FF, data);
        }
    } else {
        BSP::Dwt::Delay(216 * us);
        j3.SendCanCmd();
        BSP::Dwt::Delay(216 * us);
        j4.SendCanCmd();
    }
}

void setup() {
    BSP::Init();
    robot.Init();
}

void loop() {
    robot.OnLoop();
    send_can_cmd();
}

extern "C" void rmpp_main() {
    setup();

    static UnitFloat<pct> cpu_usage;
    BSP::Dwt dwt;
    while (true) {
        if (dwt.PollTimeout(1 * ms)) {
            loop();
            const UnitFloat<ms> running_time = dwt.GetDT();
            cpu_usage = running_time / (1 * ms);
        }
    }
}
