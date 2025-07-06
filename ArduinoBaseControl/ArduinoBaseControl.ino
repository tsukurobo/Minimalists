#include <SPI.h>
#include <mcp_can.h>

#include "dynamics.hpp"
#include "pid_controller.hpp"
#include "robomaster_motor.hpp"
#include "trajectory.hpp"

// 制御関連の定数
constexpr double MAX_CURRENT = 20.0;  // (A)

double g_start_time;  // 開始時刻（秒）

long unsigned int rxId;
unsigned char len = 0;
unsigned char rxBuf[8];
byte txBuf[] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
long Pre_millis;
MCP_CAN CAN0(53);

// ベースのモータから出力軸までのギア比
double gear_ratio_R = 3591.0 / 187.0 * 3.0;  // M3508(3591.0/187.0) * M3508出力軸からベース根本(3.0)
double gear_ratio_P = 36.0;                  // M2006 P36のギア比
double gear_radius_P = 0.025;                // ギアの半径 (m) - M2006の出力軸からラックまでの距離が25mm

// RoboMasterモータインスタンス
robomaster_motor_t motor_R(0x201, gear_ratio_R);  // ベース回転用M3508
robomaster_motor_t motor_P(0x202, gear_ratio_P);  // アーム直動用M2006

// R軸（ベース回転）の動力学パラメータ
dynamics_t dynamics_R(
    0.024371,                 // 等価慣性モーメント (kg·m^2)
    0.036437,                 // 等価粘性摩擦係数 (N·m·s/rad)
    0.3 * gear_ratio_R * 0.7  // 等価トルク定数（M3508のトルク定数xギア比x伝達効率）(Nm/A)
);

// P軸（アーム直動）の動力学パラメータ
dynamics_t dynamics_P(
    0.3,                                        // 等価慣性モーメント (kg·m^2)
    0.002651,                                   // 粘性摩擦係数 (N·m·s/rad)
    0.18 * gear_ratio_P * 0.66 / gear_radius_P  // 力定数（M2006のトルク定数xギア比x伝達効率/ギアの半径）(N/A)
);

// R軸（ベース回転）の軌道生成パラメータ
trajectory_t trajectory_R(
    6.0,      // 最大角速度 (rad/s)
    20.0,     // 最大角加速度 (rad/s^2)
    0.0,      // 開始位置 (rad)
    2 * M_PI  // 目標位置 (rad)
);

// P軸（アーム直動）の軌道生成パラメータ
trajectory_t trajectory_P(
    0.5 / gear_radius_P,  // 最大速度 (m/s) / ギアの円周(m) * 2pi = 最大速度 / ギアの半径 = 最大角速度 (rad/s)
    2.0 / gear_radius_P,  // 最大角加速度 (rad/s^2)
    0.0 / gear_radius_P,  // 開始位置 (rad)
    0.3 / gear_radius_P   // 目標位置 (rad) - 300mm移動
);

// R軸速度制御用速度型PIDコントローラ
velocity_pid_controller_t pid_R_vel(
    10.0,  // Kp
    1.0,   // Ki
    0.1,   // Kd
    20.0,  // output_max 最大電流指令値
    -20.0  // output_min
);

// P軸速度制御用速度型PIDコントローラ
velocity_pid_controller_t pid_P_vel(
    0.1,   // Kp - 直動軸なので少し高めに設定
    0.01,  // Ki
    0.0,   // Kd
    10.0,  // output_max 最大電流指令値（M2006は小型なので低めに設定）
    -10.0  // output_min
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
    trajectory_P.calculate_trapezoidal_params();

    Pre_millis = millis();
}

void loop() {
    static bool is_started_R = false, is_started_P = false;
    static bool auto_repeat_R = false, auto_repeat_P = false;    // 自動往復フラグ
    static bool target_is_180_R = true, target_is_out_P = true;  // 現在の目標状態
    static bool manual_mode_R = false, manual_mode_P = false;    // 手動制御フラグ
    double total_current_R = 0.0, total_current_P = 0.0;         // 総電流制御量
    // 軌道生成による目標値の計算
    static double target_pos_R = 0, target_vel_R = 0, target_accel_R = 0;
    static double target_pos_P = 0, target_vel_P = 0, target_accel_P = 0;
    static char mode = '\0';  // 現在のコマンドモードを保持

    if (Serial.available() > 0) {
        mode = Serial.read();
        double val = Serial.readStringUntil('\n').toDouble();

        if (mode == 'r') {   // R軸：回転軸コマンド（角度指定）
            if (val == 0) {  // r0: R軸往復動作開始
                is_started_R = true;
                auto_repeat_R = true;
                manual_mode_R = false;  // 手動制御モードを解除
                target_is_180_R = true;
                g_start_time = millis() / 1000.0;
                trajectory_R.set_start_pos(0.0);
                trajectory_R.set_end_pos(M_PI);
                trajectory_R.calculate_trapezoidal_params();
                pid_R_vel.reset();  // 速度型PIDをリセット
                Serial.println("R-axis: Starting reciprocating motion: 0° <-> 180°");
            } else {  // r<角度>: R軸単発動作
                is_started_R = true;
                auto_repeat_R = false;
                manual_mode_R = false;  // 手動制御モードを解除
                g_start_time = millis() / 1000.0;
                trajectory_R.set_start_pos(0.0);
                trajectory_R.set_end_pos(val * (M_PI / 180.0));
                trajectory_R.calculate_trapezoidal_params();
                pid_R_vel.reset();  // 速度型PIDをリセット
                Serial.print("R-axis: Single move to ");
                Serial.print(val);
                Serial.println(" degrees");
            }
        } else if (mode == 'p') {  // P軸：直動軸コマンド（距離指定）
            if (val == 0) {        // p0: P軸往復動作開始
                is_started_P = true;
                auto_repeat_P = true;
                manual_mode_P = false;  // 手動制御モードを解除
                target_is_out_P = true;
                g_start_time = millis() / 1000.0;
                trajectory_P.set_start_pos(0.0);
                trajectory_P.set_end_pos(0.3 / gear_radius_P);  // 300mm伸縮
                trajectory_P.calculate_trapezoidal_params();
                pid_P_vel.reset();  // 速度型PIDをリセット
                Serial.println("P-axis: Starting reciprocating motion: 0mm <-> 300mm");
            } else {  // p<距離>: P軸単発動作（mm単位）
                is_started_P = true;
                auto_repeat_P = false;
                manual_mode_P = false;  // 手動制御モードを解除
                g_start_time = millis() / 1000.0;
                trajectory_P.set_start_pos(0.0);
                trajectory_P.set_end_pos(val / 1000.0 / gear_radius_P);  // mmをmに変換
                trajectory_P.calculate_trapezoidal_params();
                pid_P_vel.reset();  // 速度型PIDをリセット
                Serial.print("P-axis: Single move to ");
                Serial.print(val);
                Serial.println(" mm");
            }
        } else if (mode == 's') {  // 停止
            is_started_R = false;
            is_started_P = false;
            auto_repeat_R = false;
            auto_repeat_P = false;
            manual_mode_R = false;
            manual_mode_P = false;
            target_vel_R = 0;
            target_vel_P = 0;
            // 速度型PIDコントローラをリセット
            pid_R_vel.reset();
            pid_P_vel.reset();
            Serial.println("All axes stopped");
        } else if (mode == 'a') {  // 手動電流指令（デバッグ用）
            is_started_R = false;  // 軌道制御を停止
            manual_mode_R = true;  // 手動制御モードに設定
            total_current_R = val;
            // 手動制御時は速度型PIDに現在の出力値を設定
            pid_R_vel.set_current_output(val);
            Serial.print("Manual current R: ");
            Serial.println(val);
        } else if (mode == 'v') {  // 手動速度指令（デバッグ用）
            is_started_R = false;  // 軌道制御を停止
            manual_mode_R = true;  // 手動制御モードに設定
            target_vel_R = val;
            Serial.print("Manual velocity R: ");
            Serial.println(val);
        } else if (mode == 'A') {  // 手動電流指令（P軸デバッグ用）
            is_started_P = false;  // 軌道制御を停止
            manual_mode_P = true;  // 手動制御モードに設定
            total_current_P = val;
            // 手動制御時は速度型PIDに現在の出力値を設定
            pid_P_vel.set_current_output(val);
            Serial.print("Manual current P: ");
            Serial.println(val);
        } else if (mode == 'V') {  // 手動速度指令（P軸デバッグ用）
            is_started_P = false;  // 軌道制御を停止
            manual_mode_P = true;  // 手動制御モードに設定
            target_vel_P = val;
            Serial.print("Manual velocity P: ");
            Serial.println(val);
        } else if (mode == 'g') {  // PIDゲイン変更コマンド
            // gR,Kp,Ki,Kd でR軸、gP,Kp,Ki,Kd でP軸
            String param = Serial.readStringUntil('\n');
            char axis = param.charAt(0);
            param = param.substring(1);  // "R,3.0,0.5,0.0" など
            double kp = param.substring(1, param.indexOf(',', 1)).toDouble();
            int idx2 = param.indexOf(',', 1);
            double ki = param.substring(idx2 + 1, param.indexOf(',', idx2 + 1)).toDouble();
            int idx3 = param.indexOf(',', idx2 + 1);
            double kd = param.substring(idx3 + 1).toDouble();

            if (axis == 'R') {
                pid_R_vel.set_kp(kp);
                pid_R_vel.set_ki(ki);
                pid_R_vel.set_kd(kd);
                Serial.print("R-axis PID gains set: Kp=");
                Serial.print(kp);
                Serial.print(", Ki=");
                Serial.print(ki);
                Serial.print(", Kd=");
                Serial.println(kd);
            } else if (axis == 'P') {
                pid_P_vel.set_kp(kp);
                pid_P_vel.set_ki(ki);
                pid_P_vel.set_kd(kd);
                Serial.print("P-axis PID gains set: Kp=");
                Serial.print(kp);
                Serial.print(", Ki=");
                Serial.print(ki);
                Serial.print(", Kd=");
                Serial.println(kd);
            } else {
                Serial.println("Invalid axis for PID gain setting.");
            }
        }
    }

    int16_t angle_raw_R = 0, angle_raw_P = 0;
    int16_t rpm_R = 0, rpm_P = 0;
    int16_t current_raw_R = 0, current_raw_P = 0;
    int8_t temp_R = 0, temp_P = 0;

    // 複数のCANメッセージを処理するためのループ
    while (CAN0.checkReceive() == CAN_MSGAVAIL) {
        CAN0.readMsgBuf(&rxId, &len, rxBuf);

        if (rxId == 0x201) {  // R軸モータ（M3508）
            motor_R.parse_can_data(rxBuf, &angle_raw_R, &rpm_R, &current_raw_R, &temp_R);
            motor_R.update_encoder_angle(angle_raw_R);
            motor_R.rpm_to_angular_velocity(rpm_R);
        } else if (rxId == 0x202) {  // P軸モータ（M2006）
            motor_P.parse_can_data(rxBuf, &angle_raw_P, &rpm_P, &current_raw_P, &temp_P);
            motor_P.update_encoder_angle(angle_raw_P);
            motor_P.rpm_to_angular_velocity(rpm_P);
        }
        // 他のモータIDがある場合はここに追加
    }

    // R軸（回転軸）制御
    double ff_current_R = 0;
    if (is_started_R) {
        double current_time = millis() / 1000.0 - g_start_time;
        trajectory_R.get_trapezoidal_state(current_time, &target_pos_R, &target_vel_R, &target_accel_R);

        double ff_control_R = dynamics_R.calculate_feedforward_control(target_vel_R, target_accel_R);
        ff_current_R = dynamics_R.convert_to_current_command(ff_control_R);

        if (current_time >= trajectory_R.get_total_time()) {
            ff_current_R = 0.0;

            if (auto_repeat_R) {
                if (target_is_180_R) {
                    trajectory_R.set_start_pos(M_PI);
                    trajectory_R.set_end_pos(0.0);
                    target_is_180_R = false;
                    Serial.println("R-axis: Moving to 0 degrees...");
                } else {
                    trajectory_R.set_start_pos(0.0);
                    trajectory_R.set_end_pos(M_PI);
                    target_is_180_R = true;
                    Serial.println("R-axis: Moving to 180 degrees...");
                }
                trajectory_R.calculate_trapezoidal_params();
                pid_R_vel.reset();  // 軌道切り替え時にPIDリセット
                g_start_time = millis() / 1000.0;
            } else {
                is_started_R = false;
                Serial.println("R-axis: Target position reached. Stopping motor.");
            }
        }
    }

    // P軸（直動軸）制御
    double ff_current_P = 0;
    if (is_started_P) {
        double current_time = millis() / 1000.0 - g_start_time;
        trajectory_P.get_trapezoidal_state(current_time, &target_pos_P, &target_vel_P, &target_accel_P);

        double ff_control_P = dynamics_P.calculate_feedforward_control(target_vel_P, target_accel_P);
        ff_current_P = dynamics_P.convert_to_current_command(ff_control_P);

        if (current_time >= trajectory_P.get_total_time()) {
            ff_current_P = 0.0;

            if (auto_repeat_P) {
                if (target_is_out_P) {
                    trajectory_P.set_start_pos(0.3 / gear_radius_P);  // 300mm位置
                    trajectory_P.set_end_pos(0.0);                    // 0mm位置
                    target_is_out_P = false;
                    Serial.println("P-axis: Retracting to 0mm...");
                } else {
                    trajectory_P.set_start_pos(0.0);                // 0mm位置
                    trajectory_P.set_end_pos(0.3 / gear_radius_P);  // 300mm位置
                    target_is_out_P = true;
                    Serial.println("P-axis: Extending to 300mm...");
                }
                trajectory_P.calculate_trapezoidal_params();
                pid_P_vel.reset();  // 軌道切り替え時にPIDリセット
                g_start_time = millis() / 1000.0;
            } else {
                is_started_P = false;
                Serial.println("P-axis: Target position reached. Stopping motor.");
            }
        }
    }

    // PIDフィードバック制御量の計算（速度型PID使用）
    double fb_current_R = 0.0, fb_current_P = 0.0;

    // R軸の制御量計算
    if (is_started_R) {
        // 自動軌道制御
        // 速度型PIDでフィードバック制御量を計算
        double pid_output_R = pid_R_vel.calculate(target_vel_R, motor_R.get_angular_velocity(), 0.02);

        // フィードフォワード制御量を追加（PIDの出力制限内で）
        double combined_current = pid_output_R + ff_current_R;
        if (combined_current > 20.0) {
            total_current_R = 20.0;
        } else if (combined_current < -20.0) {
            total_current_R = -20.0;
        } else {
            total_current_R = combined_current;
        }
    } else if (manual_mode_R) {
        // 手動制御モード
        if (mode == 'v') {
            // 手動速度制御
            total_current_R = pid_R_vel.calculate(target_vel_R, motor_R.get_angular_velocity(), 0.02);
        }
        // 手動電流制御（mode == 'a'）の場合はtotal_current_Rは既に設定済み
    } else {
        // 停止状態
        total_current_R = 0.0;
        pid_R_vel.set_current_output(0.0);
    }

    // P軸の制御量計算
    if (is_started_P) {
        // 自動軌道制御
        // 速度型PIDでフィードバック制御量を計算
        double pid_output_P = pid_P_vel.calculate(target_vel_P, motor_P.get_angular_velocity(), 0.02);

        // フィードフォワード制御量を追加（PIDの出力制限内で）
        double combined_current = pid_output_P + ff_current_P;
        if (combined_current > 10.0) {
            total_current_P = 10.0;
        } else if (combined_current < -10.0) {
            total_current_P = -10.0;
        } else {
            total_current_P = combined_current;
        }
    } else if (manual_mode_P) {
        // 手動制御モード
        if (mode == 'V') {
            // 手動速度制御
            total_current_P = pid_P_vel.calculate(target_vel_P, motor_P.get_angular_velocity(), 0.02);
        }
        // 手動電流制御（mode == 'A'）の場合はtotal_current_Pは既に設定済み
    } else {
        // 停止状態
        total_current_P = 0.0;
        pid_P_vel.set_current_output(0.0);
    }

    // 電流値を制限（安全のため）
    total_current_R = constrain(total_current_R, -MAX_CURRENT, MAX_CURRENT);
    total_current_P = constrain(total_current_P, -MAX_CURRENT * 0.5, MAX_CURRENT * 0.5);  // M2006は小型なので半分に制限

    // CAN通信用にデータを変換
    int16_t motor_output_current_R = motor_R.current_to_raw(total_current_R);
    int16_t motor_output_current_P = motor_P.current_to_raw(total_current_P);

    // 0x200番のCANメッセージ（モータID 0x201-0x204用）
    txBuf[0] = (motor_output_current_R >> 8) & 0xFF;  // R軸上位バイト
    txBuf[1] = motor_output_current_R & 0xFF;         // R軸下位バイト
    txBuf[2] = (motor_output_current_P >> 8) & 0xFF;  // P軸上位バイト
    txBuf[3] = motor_output_current_P & 0xFF;         // P軸下位バイト
    txBuf[4] = 0x00;                                  // モータ3（未使用）
    txBuf[5] = 0x00;
    txBuf[6] = 0x00;  // モータ4（未使用）
    txBuf[7] = 0x00;

    // Send
    if (millis() - Pre_millis > 20) {  // Period: 20ms
        CAN0.sendMsgBuf(0x200, 0, 8, txBuf);
        Pre_millis = millis();
    }

    // デバッグ出力
    // Serial.print("R-axis: Angle(raw)=");
    // Serial.print(angle_raw_R);
    // Serial.print(", Turns=");
    // Serial.print(motor_R.get_encoder_turns());
    // Serial.print(", Current(A)=");
    // Serial.print(motor_R.raw_to_current(current_raw_R), 2);
    // Serial.print(", Omega(rad/s)=");
    // Serial.print(motor_R.get_angular_velocity(), 3);
    // Serial.print(", TotalCurrent=");
    // Serial.print(total_current_R, 3);

    // Serial.print(" | P-axis: Angle(raw)=");
    // Serial.print(angle_raw_P);
    // Serial.print(", Turns=");
    // Serial.print(motor_P.get_encoder_turns());
    // Serial.print(", Current(A)=");
    // Serial.print(motor_P.raw_to_current(current_raw_P), 2);
    // Serial.print(", Omega(rad/s)=");
    // Serial.print(motor_P.get_angular_velocity(), 3);
    // Serial.print(", TotalCurrent=");
    // Serial.println(total_current_P, 3);

    // Serial.print("R-axis: TargetVel=");
    Serial.print(target_vel_R, 3);
    // Serial.print(", Omega=");
    Serial.print(", ");
    Serial.print(motor_R.get_angular_velocity(), 3);
    // Serial.print(", Error=");
    Serial.print(", ");
    Serial.println(target_vel_R - motor_R.get_angular_velocity(), 3);

    // Serial.print("P-axis: TargetVel=");
    // Serial.print(target_vel_P, 3);
    // Serial.print(", Omega=");
    // Serial.print(motor_P.get_angular_velocity(), 3);
    // Serial.print(", Error=");
    // Serial.println(target_vel_P - motor_P.get_angular_velocity(), 3);

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