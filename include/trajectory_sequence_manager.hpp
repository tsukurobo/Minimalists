#ifndef TRAJECTORY_SEQUENCE_MANAGER_HPP
#define TRAJECTORY_SEQUENCE_MANAGER_HPP

#include "debug_manager.hpp"
#include "pico/mutex.h"

/**
 * @brief 軌道ウェイポイント構造体
 *
 * 3次元空間での目標位置を表現する構造体
 */
struct TrajectoryWaypoint {
    float position_R;          // R軸目標位置 [rad]
    float position_P;          // P軸目標位置 [rad]（距離をradに変換済み）
    float end_effector_angle;  // 手先角度 [rad]（現在未使用、ダミーデータ）

    /**
     * @brief コンストラクタ
     */
    TrajectoryWaypoint(float pos_R = 0.0f, float pos_P = 0.0f, float end_angle = 0.0f)
        : position_R(pos_R), position_P(pos_P), end_effector_angle(end_angle) {}
};

/**
 * @brief 軌道シーケンス管理クラス
 *
 * 複数のウェイポイントを順次実行する軌道シーケンスを管理するクラス
 */
class TrajectorySequenceManager {
   private:
    static constexpr int MAX_WAYPOINTS = 20;  // 最大ウェイポイント数

    TrajectoryWaypoint waypoints[MAX_WAYPOINTS];
    int waypoint_count;
    int current_waypoint_index;
    bool sequence_active;
    bool sequence_complete;
    float wait_duration;  // 各点での待機時間 [s]

    mutex_t sequence_mutex;
    DebugManager* debug_manager;

   public:
    /**
     * @brief コンストラクタ
     * @param debug_mgr デバッグマネージャのポインタ
     */
    explicit TrajectorySequenceManager(DebugManager* debug_mgr);

    /**
     * @brief デストラクタ
     */
    ~TrajectorySequenceManager() = default;

    /**
     * @brief シーケンスの初期化
     */
    void initialize();

    /**
     * @brief ウェイポイントの追加
     * @param pos_R R軸位置 [rad]
     * @param pos_P P軸位置 [rad]
     * @param end_effector_angle 手先角度 [rad] (デフォルト: 0.0)
     * @return 追加成功時true
     */
    bool add_waypoint(float pos_R, float pos_P, float end_effector_angle = 0.0f);

    /**
     * @brief テスト用軌道シーケンスの設定
     * @param gear_radius_P P軸のギア半径 [m]
     */
    void setup_test_sequence(float gear_radius_P);

    /**
     * @brief シーケンスの開始
     */
    void start_sequence();

    /**
     * @brief 次のウェイポイントの取得
     * @param target_R R軸目標位置 [rad] (出力)
     * @param target_P P軸目標位置 [rad] (出力)
     * @return 次のウェイポイントが存在する場合true
     */
    bool get_next_waypoint(float& target_R, float& target_P);

    /**
     * @brief 次のウェイポイントに進む
     */
    void advance_to_next_waypoint();

    /**
     * @brief シーケンスがアクティブかチェック
     * @return アクティブな場合true
     */
    bool is_sequence_active();

    /**
     * @brief シーケンスが完了したかチェック
     * @return 完了した場合true
     */
    bool is_sequence_complete();

    /**
     * @brief 現在のウェイポイントインデックスを取得
     * @return 現在のインデックス
     */
    int get_current_waypoint_index();

    /**
     * @brief 総ウェイポイント数を取得
     * @return 総ウェイポイント数
     */
    int get_waypoint_count();

    /**
     * @brief 待機時間の設定
     * @param duration 待機時間 [s]
     */
    void set_wait_duration(float duration);
};

#endif  // TRAJECTORY_SEQUENCE_MANAGER_HPP
