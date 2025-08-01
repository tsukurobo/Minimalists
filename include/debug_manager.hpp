#ifndef DEBUG_MANAGER_HPP
#define DEBUG_MANAGER_HPP

#include <cmath>
#include <cstdarg>

#include "control_timing.hpp"

/**
 * @brief デバッグ出力レベル
 *
 * 出力するメッセージの重要度を定義します。
 * より低いレベルのメッセージは高いレベル設定時にも出力されます。
 */
enum class DebugLevel {
    OFF = 0,    ///< 出力なし
    ERROR = 1,  ///< エラーメッセージのみ
    WARN = 2,   ///< 警告メッセージまで
    INFO = 3,   ///< 情報メッセージまで
    DEBUG = 4,  ///< デバッグメッセージまで
    TRACE = 5   ///< 全てのメッセージ
};

/**
 * @brief 軌道デバッグ情報構造体
 *
 * 軌道制御に関するデバッグ情報をまとめた構造体です。
 */
struct TrajectoryDebugInfo {
    float final_target_pos;       ///< 最終目標位置 [rad]
    float trajectory_target_pos;  ///< 軌道目標位置 [rad]
    float trajectory_target_vel;  ///< 軌道目標速度 [rad/s]
    float current_pos;            ///< 現在位置 [rad]
    float current_vel;            ///< 現在速度 [rad/s]
    float final_target_vel;       ///< 最終目標速度 [rad/s]
    bool trajectory_active;       ///< 軌道実行中フラグ

    // 単位変換用パラメータ
    float gear_radius;      ///< ギア半径 [m] (P軸の場合のみ使用)
    const char* unit_name;  ///< 単位名 ("rad" or "mm")
    const char* axis_name;  ///< 軸名 ("R" or "P")
};

/**
 * @brief システム状態デバッグ情報構造体
 *
 * システム全体の状態に関するデバッグ情報をまとめた構造体です。
 */
struct SystemDebugInfo {
    int timing_violations;   ///< 制御タイミング違反回数
    int can_errors;          ///< CAN通信エラー回数
    led_mode_t led_status;   ///< LED状態
    bool encoder_r_valid;    ///< R軸エンコーダデータ有効性
    bool encoder_p_valid;    ///< P軸エンコーダデータ有効性
    float target_torque_R;   ///< R軸目標トルク [Nm]
    float target_torque_P;   ///< P軸目標トルク [Nm]
    float target_current_R;  ///< R軸目標電流 [A]
    float target_current_P;  ///< P軸目標電流 [A]

    // エンコーダ詳細情報
    int16_t encoder_p_turn_count;      ///< P軸エンコーダ回転回数
    float encoder_p_single_angle_deg;  ///< P軸エンコーダ単回転角度 [deg]
    float encoder_r_angle_deg;         ///< R軸エンコーダ角度 [deg]
};

/**
 * @brief デバッグ管理クラス
 *
 * システム全体のデバッグ出力を統一的に管理するクラスです。
 * レベル別のログ出力、システム状態の監視などの機能を提供します。
 */
class DebugManager {
   private:
    DebugLevel current_level;       ///< 現在のデバッグレベル
    float status_output_interval;   ///< ステータス出力間隔 [s]
    float last_status_output_time;  ///< 最後のステータス出力時刻 [s]

    // 軌道状態追跡用
    bool prev_trajectory_active_R;  ///< 前回のR軸軌道実行状態
    bool prev_trajectory_active_P;  ///< 前回のP軸軌道実行状態
    bool limits_displayed;          ///< 制限値表示済みフラグ

    float time_counter;  ///< 時間カウンタ [s]

   public:
    /**
     * @brief コンストラクタ
     * @param level デバッグレベル
     * @param status_interval ステータス出力間隔 [s]
     */
    DebugManager(DebugLevel level = DebugLevel::INFO, float status_interval = 1.0f);

    // === 基本ログ出力機能 ===

    /**
     * @brief レベル指定ログ出力
     * @param level デバッグレベル
     * @param format printfスタイルのフォーマット文字列
     * @param ... 可変引数
     */
    void log(DebugLevel level, const char* format, ...);

    /**
     * @brief エラーメッセージ出力
     * @param format printfスタイルのフォーマット文字列
     * @param ... 可変引数
     */
    void error(const char* format, ...);

    /**
     * @brief 警告メッセージ出力
     * @param format printfスタイルのフォーマット文字列
     * @param ... 可変引数
     */
    void warn(const char* format, ...);

    /**
     * @brief 情報メッセージ出力
     * @param format printfスタイルのフォーマット文字列
     * @param ... 可変引数
     */
    void info(const char* format, ...);

    /**
     * @brief デバッグメッセージ出力
     * @param format printfスタイルのフォーマット文字列
     * @param ... 可変引数
     */
    void debug(const char* format, ...);

    // === 軌道デバッグ機能 ===

    /**
     * @brief 軌道状態変化の検出と出力
     * @param traj_active_R R軸軌道実行中フラグ
     * @param traj_active_P P軸軌道実行中フラグ
     * @param current_pos_R R軸現在位置 [rad]
     * @param current_pos_P P軸現在位置 [rad]
     * @param final_target_pos_R R軸最終目標位置 [rad]
     * @param final_target_pos_P P軸最終目標位置 [rad]
     * @param gear_radius_P P軸ギア半径 [m]
     */
    void check_trajectory_state_changes(bool traj_active_R, bool traj_active_P,
                                        float current_pos_R, float current_pos_P,
                                        float final_target_pos_R, float final_target_pos_P,
                                        float gear_radius_P);

    /**
     * @brief 軌道制限値の表示（初回のみ）
     * @param max_vel_R R軸最大速度 [rad/s]
     * @param max_accel_R R軸最大加速度 [rad/s^2]
     * @param max_vel_P P軸最大速度 [rad/s]
     * @param max_accel_P P軸最大加速度 [rad/s^2]
     * @param gear_radius_P P軸ギア半径 [m]
     */
    void print_trajectory_limits(float max_vel_R, float max_accel_R,
                                 float max_vel_P, float max_accel_P,
                                 float gear_radius_P);

    /**
     * @brief 軌道ステータスの詳細表示
     * @param r_info R軸軌道デバッグ情報
     * @param p_info P軸軌道デバッグ情報
     */
    void print_trajectory_status(const TrajectoryDebugInfo& r_info,
                                 const TrajectoryDebugInfo& p_info);

    // === システム状態デバッグ機能 ===

    /**
     * @brief システム状態の表示
     * @param sys_info システム状態デバッグ情報
     */
    void print_system_status(const SystemDebugInfo& sys_info);

    /**
     * @brief 異常値の検出と警告
     * @param traj_target_pos_P P軸軌道目標位置 [rad]
     * @param gear_radius_P P軸ギア半径 [m]
     */
    void check_abnormal_values(float traj_target_pos_P, float gear_radius_P);

    // === ユーティリティ機能 ===

    /**
     * @brief 定期ステータス出力の判定
     * @param current_time 現在時刻 [s]
     * @return 出力すべき場合true
     */
    bool should_output_status(float current_time);

    /**
     * @brief 軌道進行状況のデバッグ出力
     * @param current_pos_R 現在のR軸位置 [rad]
     * @param current_pos_P 現在のP軸位置 [rad]
     * @param trajectory_final_target_R 最終目標R軸位置 [rad]
     * @param trajectory_final_target_P 最終目標P軸位置 [rad]
     * @param trajectory_position_reached 位置到達フラグ
     * @param trajectory_current_index 現在の軌道インデックス
     * @param trajectory_point_count 軌道点総数
     * @param gear_radius_P P軸のギア半径 [m]
     */
    void print_trajectory_progress(float current_pos_R, float current_pos_P,
                                   float trajectory_final_target_R, float trajectory_final_target_P,
                                   bool trajectory_position_reached,
                                   int trajectory_current_index, int trajectory_point_count,
                                   float gear_radius_P);

    /**
     * @brief デバッグレベルの設定
     * @param level 新しいデバッグレベル
     */
    void set_debug_level(DebugLevel level) { current_level = level; }

    /**
     * @brief 時間カウンタの更新
     * @param delta_time 増分時間 [s]
     */
    void update_time_counter(float delta_time) { time_counter += delta_time; }

    /**
     * @brief 時間カウンタの取得
     * @return 現在の時間カウンタ [s]
     */
    float get_time_counter() const { return time_counter; }

   private:
    /**
     * @brief レベル付きメッセージの出力
     * @param level デバッグレベル
     * @param format フォーマット文字列
     * @param args 可変引数リスト
     */
    void print_with_level_prefix(DebugLevel level, const char* format, va_list args);

    /**
     * @brief デバッグレベルの文字列取得
     * @param level デバッグレベル
     * @return レベル文字列
     */
    const char* get_level_string(DebugLevel level);
};

#endif  // DEBUG_MANAGER_HPP
