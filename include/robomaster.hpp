#ifndef ROBOMASTER_CAN_H
#define ROBOMASTER_CAN_H

#include "mcp25625.hpp"  // MCP25625ライブラリをインクルード

// Robomasterモーターの標準的なCAN ID
#define CAN_ID_CONTROL_MOTORS_1_4 0x200
#define CAN_ID_CONTROL_MOTORS_5_8 0x1FF
#define CAN_ID_FEEDBACK_MOTOR1 0x201
#define CAN_ID_FEEDBACK_MOTOR2 0x202
#define CAN_ID_FEEDBACK_MOTOR3 0x203
#define CAN_ID_FEEDBACK_MOTOR4 0x204

// モーターからのフィードバックデータを格納する構造体
typedef struct {
    uint16_t angle;       // メカニカル角度 (0-8191)
    int16_t speed;        // 回転数 (RPM)
    int16_t current;      // 実電流
    uint8_t temperature;  // 温度 (°C)
} RoboMotorFeedback;

/**
 * @brief 4つのモーターに電流指令値を送信する
 * @param can MCP25625のインスタンス
 * @param motor1 モーター1の電流指令値 (-16384 ~ 16384)
 * @param motor2 モーター2の電流指令値
 * @param motor3 モーター3の電流指令値
 * @param motor4 モーター4の電流指令値
 * @return 送信に成功すればtrue、失敗すればfalse
 */
bool send_motor_currents(MCP25625& can, int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4);

/**
 * @brief 受信したCANフレームをモーターのフィードバックデータに解釈する
 * @param frame 受信したCANフレーム
 * @param feedback_data 解釈結果を格納する構造体への参照
 * @return 解釈に成功すればtrue、フィードバックフレームでなければfalse
 */
bool parse_motor_feedback(const struct can_frame& frame, RoboMotorFeedback& feedback_data);

#endif  // ROBOMASTER_CAN_H