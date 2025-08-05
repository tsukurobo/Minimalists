#ifndef TRAJECTORY_SEQUENCE_MANAGER_HPP
#define TRAJECTORY_SEQUENCE_MANAGER_HPP

#include <cmath>

#include "debug_manager.hpp"

/**
 * @brief 軌道ウェイポイント構造体
 *
 * 3次元空間での目標位置を表現する構造体
 */
struct trajectory_waypoint_t {
    float position_R;               // R軸目標位置 [rad]
    float position_P;               // P軸目標位置 [rad]
    float end_effector_angle;       // 手先角度 [rad]（現在未使用、ダミーデータ）
    float intermediate_position_R;  // 中継位置 R軸 [rad]（未定義時はNAN）
    float intermediate_position_P;  // 中継位置 P軸 [rad]（未定義時はNAN）

    /**
     * @brief コンストラクタ
     */
    trajectory_waypoint_t(float pos_R = 0.0f, float pos_P = 0.0f, float end_angle = 0.0f,
                          float inter_pos_R = NAN, float inter_pos_P = NAN)
        : position_R(pos_R), position_P(pos_P), end_effector_angle(end_angle), intermediate_position_R(inter_pos_R), intermediate_position_P(inter_pos_P) {}
};

/**
 * @brief 軌道シーケンス管理クラス
 *
 * 複数のウェイポイントを順次実行する軌道シーケンスを管理するクラス
 */
class TrajectorySequenceManager {
   private:
    static constexpr int MAX_WAYPOINTS = 100;  // 最大ウェイポイント数

    trajectory_waypoint_t waypoints[MAX_WAYPOINTS];
    int waypoint_count;
    int current_waypoint_index;
    bool sequence_active;
    bool sequence_complete;
    float wait_duration;  // 各点での待機時間 [s]

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
     * @brief ウェイポイント配列から軌道シーケンスを設定
     * @param waypoint_array ウェイポイント配列
     * @param count ウェイポイントの数
     */
    void setup_sequence(const trajectory_waypoint_t* waypoint_array, int count);

    /**
     * @brief 次のウェイポイントの取得
     * @param target_position 目標位置 [rad] (出力)
     * @param intermediate_position 中継位置 [rad] (出力)
     * @return 次のウェイポイントが存在する場合true
     */
    bool get_next_waypoint(float target_position[2], float intermediate_position[2]);

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
