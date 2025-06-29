#include <SPI.h>
#include <mcp_can.h>

#include "dynamics.hpp"
#include "pid_controller.hpp"
#include "robomaster_motor.hpp"
#include "trajectory.hpp"

// 制御関連の定数
constexpr double MAX_CURRENT = 1.0;  // (A)

double g_start_time;  // 開始時刻（秒）

long unsigned int rxId;
unsigned char len = 0;
unsigned char rxBuf[8];
byte txBuf[] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
long Pre_millis;
MCP_CAN CAN0(53);

// ベースのモータから出力軸までのギア比
double gear_ratio_R = 3591.0 / 187.0 * 3.0;  // M3508(3591.0/187.0) * M3508出力軸からベース根本(3.0)
// RoboMasterモータインスタンス（モータID: 0x201-0x200, ギア比）
robomaster_motor_t motor_R(0x201, gear_ratio_R);

dynamics_t dynamics_R(
    0.024371,                 // 等価慣性モーメント
    0.036437,                 // 等価粘性摩擦係数
    0.3 * gear_ratio_R * 0.7  // 等価トルク定数（M3508のトルク定数xギア比x伝達効率）(Nm/A)
);

trajectory_t trajectory_R(
    6.0,      // 最大速度 (rad/s)
    20.0,     // 最大加速度 (rad/s^2)
    0.0,      // 開始位置 (rad)
    2 * M_PI  // 目標位置 (rad)
);

pid_controller_t pid_R_vel(
    1,    // Kp
    0.2,  // Ki
    0.0,  // Kd
    1,    // integral_max 積分値の上限
    -1,   // integral_min
    20,   // output_max 最大電流指令値
    -20   // output_min
);

void setup() {
    Serial.begin(115200);
    if (CAN0.begin(MCP_ANY, CAN_1000KBPS, MCP_8MHZ) == CAN_OK) {
        Serial.println("CAN0: Init OK!");
        CAN0.setMode(MCP_NORMAL);
    } else {
        Serial.println("CAN0: Init Fail!");
    }

    // 軌道計算のパラメータを初期化
    trajectory_R.calculate_trapezoidal_params();

    Pre_millis = millis();
}

void loop() {
    static bool is_started = false;
    static bool auto_repeat = false;   // 自動往復フラグ
    static bool target_is_180 = true;  // 現在の目標が180度かどうか
    double total_current = 0.0;        // 総電流制御量
    // 軌道生成による目標値の計算
    static double target_pos = 0, target_vel = 0, target_accel = 0;
    if (Serial.available() > 0) {
        char mode = Serial.read();
        double val = Serial.readStringUntil('\n').toDouble();

        if (mode == 'c') {  // valueを角度として解釈
            is_started = true;
            auto_repeat = false;                             // 単発動作
            g_start_time = millis() / 1000.0;                // 開始時刻（秒）
            trajectory_R.set_start_pos(0.0);                 // 開始位置を0度に設定
            trajectory_R.set_end_pos(val * (M_PI / 180.0));  // 目標位置を設定(ラジアン)
            trajectory_R.calculate_trapezoidal_params();     // 軌道計算のパラメータを再計算
            Serial.print("Single move to ");
            Serial.print(val);
            Serial.println(" degrees");
        } else if (mode == 'r') {  // 往復動作開始
            is_started = true;
            auto_repeat = true;                           // 往復動作フラグを設定
            target_is_180 = true;                         // 最初は180度を目標にする
            g_start_time = millis() / 1000.0;             // 開始時刻（秒）
            trajectory_R.set_start_pos(0.0);              // 開始位置を0度に設定
            trajectory_R.set_end_pos(M_PI);               // 目標位置を180度に設定
            trajectory_R.calculate_trapezoidal_params();  // 軌道計算のパラメータを再計算
            Serial.println("Starting reciprocating motion: 0° <-> 180°");
        } else if (mode == 's') {  // 停止
            is_started = false;
            auto_repeat = false;
            target_vel = 0;
            Serial.println("Motion stopped");
        } else if (mode == 'a') {
            total_current = val;
        } else if (mode == 'v') {
            target_vel = val;
        }
    }

    int16_t angle_raw;
    int16_t rpm;
    int16_t current_raw;
    int8_t temp;

    // Receive
    if (CAN0.checkReceive() == CAN_MSGAVAIL) {
        CAN0.readMsgBuf(&rxId, &len, rxBuf);

        // RoboMasterモータクラスを使用してCANデータを解析
        motor_R.parse_can_data(rxBuf, &angle_raw, &rpm, &current_raw, &temp);

        // エンコーダ値を連続角度に変換
        motor_R.update_encoder_angle(angle_raw);

        // RPMを角速度に変換
        motor_R.rpm_to_angular_velocity(rpm);
    }

    double ff_current = 0;

    if (is_started) {
        double current_time = millis() / 1000.0 - g_start_time;  // 経過時間（秒）

        trajectory_R.get_trapezoidal_state(current_time, &target_pos, &target_vel, &target_accel);

        // フィードフォワード制御量の計算
        double ff_control = dynamics_R.calculate_feedforward_control(target_vel, target_accel);
        ff_current = dynamics_R.convert_to_current_command(ff_control);

        // 目標値に着いたら
        if (current_time >= trajectory_R.get_total_time()) {
            ff_current = 0.0;  // 目標位置に到達したら電流をゼロにする

            if (auto_repeat) {
                // 往復動作：次の目標位置を設定
                if (target_is_180) {
                    // 現在180度なので、次は0度へ
                    trajectory_R.set_start_pos(M_PI);  // 現在位置（180度）
                    trajectory_R.set_end_pos(0.0);     // 目標位置（0度）
                    target_is_180 = false;
                    Serial.println("Moving to 0 degrees...");
                } else {
                    // 現在0度なので、次は180度へ
                    trajectory_R.set_start_pos(0.0);  // 現在位置（0度）
                    trajectory_R.set_end_pos(M_PI);   // 目標位置（180度）
                    target_is_180 = true;
                    Serial.println("Moving to 180 degrees...");
                }

                // 新しい軌道のパラメータを計算
                trajectory_R.calculate_trapezoidal_params();

                // 開始時刻をリセット
                g_start_time = millis() / 1000.0;

                Serial.print("New trajectory total time: ");
                Serial.println(trajectory_R.get_total_time());
            } else {
                is_started = false;  // 動作を停止
                Serial.println("Target position reached. Stopping motor.");
            }
        }
    }

    // PIDフィードバック制御量の計算
    double fb_current = pid_R_vel.calculate(target_vel, motor_R.get_angular_velocity(), 0.004);

    // 総制御量（フィードフォワード + フィードバック）
    total_current = ff_current + fb_current;

    // 電流値を制限（安全のため）
    total_current = constrain(total_current, -MAX_CURRENT, MAX_CURRENT);

    // CAN通信用にデータを変換（RoboMasterモータクラスを使用）
    int16_t motor_output_current_Byte = motor_R.current_to_raw(total_current);
    txBuf[0] = (motor_output_current_Byte >> 8) & 0xFF;  // 上位バイト
    txBuf[1] = motor_output_current_Byte & 0xFF;         // 下位バイト

    // Send
    if (millis() - Pre_millis > 20) {  // Period: 20ms
        CAN0.sendMsgBuf(0x200, 0, 8, txBuf);
        Pre_millis = millis();
    }

    // デバッグ出力
    Serial.print("Angle(raw): ");
    Serial.print(angle_raw);
    Serial.print(", Turns: ");
    Serial.print(motor_R.get_encoder_turns());
    Serial.print(", Current(A): ");
    Serial.print(motor_R.raw_to_current(current_raw), 2);

    Serial.print(", Omega_R(rad/s): ");
    Serial.print(motor_R.get_angular_velocity(), 3);

    Serial.print(", TotalCurrent: ");
    Serial.println(total_current, 3);
    Serial.println(target_vel - motor_R.get_angular_velocity(), 3);

    static long prevTime = 0;
    unsigned long nowTime;
    unsigned long dt;
    const int us = 4000;
    nowTime = micros();
    dt = nowTime - prevTime;

    if (dt < us) {
        delayMicroseconds((unsigned int)us - dt);
    }
    prevTime = micros();
}