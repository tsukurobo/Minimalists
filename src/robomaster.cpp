#include "robomaster.hpp"

// 4つのモーターへの電流指令値を梱包して送信する関数
bool send_motor_currents(MCP25625& can, int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4) {
    struct can_frame tx_frame;

    // CAN IDとデータ長を設定
    tx_frame.can_id = CAN_ID_CONTROL_MOTORS_1_4;
    tx_frame.can_dlc = 8;

    // 各モーターの電流指令値をデータペイロードに梱包
    // 上位8ビット、下位8ビットの順で格納
    tx_frame.data[0] = (uint8_t)(motor1 >> 8);
    tx_frame.data[1] = (uint8_t)(motor1);
    tx_frame.data[2] = (uint8_t)(motor2 >> 8);
    tx_frame.data[3] = (uint8_t)(motor2);
    tx_frame.data[4] = (uint8_t)(motor3 >> 8);
    tx_frame.data[5] = (uint8_t)(motor3);
    tx_frame.data[6] = (uint8_t)(motor4 >> 8);
    tx_frame.data[7] = (uint8_t)(motor4);

    // CANライブラリを使って送信
    return can.send_can_message(&tx_frame);
}

// 受信したCANフレームを解釈する関数
bool parse_motor_feedback(const struct can_frame& frame, RoboMotorFeedback& feedback_data) {
    // 受信したIDがモーターのフィードバックIDの範囲内か確認
    if (frame.can_id < CAN_ID_FEEDBACK_MOTOR1 || frame.can_id > CAN_ID_FEEDBACK_MOTOR4) {
        return false;
    }

    // データペイロードから各値を解釈して構造体に格納
    // 上位8ビットと下位8ビットを結合
    feedback_data.angle = (uint16_t)(frame.data[0] << 8 | frame.data[1]);
    feedback_data.speed = (int16_t)(frame.data[2] << 8 | frame.data[3]);
    feedback_data.current = (int16_t)(frame.data[4] << 8 | frame.data[5]);
    feedback_data.temperature = frame.data[6];

    return true;
}