#include "config.hpp"

// core0 から core1にhand実行時の監視
bool g_hand_requested = false;
mutex_t g_hand_mutex;
// work　の保持状態
bool g_has_work = false;
// 把持状態のstate
hand_state_t g_hand_state = HAND_IDLE;
absolute_time_t g_hand_timer;

uint pwm0_slice_num;
uint pwm1_slice_num;
uint chan;

constexpr int SHUTDOWN_PIN = 27;  // シャットダウン用のGPIOピン

// 共有データ構造体
typedef struct
{
    int motor_speed;
    int sensor_value;
} robot_state_t;

// 共有状態とミューテックス
static robot_state_t g_robot_state;
static mutex_t g_state_mutex;

void init() {
    std::cout << "gpio initialized" << std::endl;
    gpio_init(SHUTDOWN_PIN);
    gpio_set_dir(SHUTDOWN_PIN, GPIO_OUT);
    gpio_put(SHUTDOWN_PIN, 0);
    // ポンプの設定
    gpio_init(PUMP_PIN1);
    gpio_set_dir(PUMP_PIN1, GPIO_OUT);
    gpio_init(PUMP_PIN2);
    gpio_set_dir(PUMP_PIN2, GPIO_OUT);
    gpio_init(SOLENOID_PIN1);
    gpio_set_dir(SOLENOID_PIN1, GPIO_OUT);

    gpio_init(SOLENOID_PIN2);
    gpio_set_dir(SOLENOID_PIN2, GPIO_OUT);

    sleep_ms(1000);  // GPIO初期化後の安定化待ち

    // Dynamixelの設定
    printf("Initializing Dynamixels (Daisy Chain on UART0)...\n");
    init_crc();
    configure_uart(&UART0, BAUD_RATE);
    sleep_ms(1000);
    write_statusReturnLevel(&UART0, DXL_ID1, 0x00);
    write_statusReturnLevel(&UART0, DXL_ID2, 0x00);
    sleep_ms(1000);
    write_dxl_led(&UART0, DXL_ID1, true);
    write_dxl_led(&UART0, DXL_ID2, true);
    sleep_ms(1000);
    write_dxl_led(&UART0, DXL_ID1, false);
    write_dxl_led(&UART0, DXL_ID2, false);
    sleep_ms(1000);
    write_torqueEnable(&UART0, DXL_ID1, false);
    write_torqueEnable(&UART0, DXL_ID2, false);
    sleep_ms(1000);
    write_operatingMode(&UART0, DXL_ID1, false);  // false : 位置制御, true : 拡張位置制御(マルチターン)
    write_operatingMode(&UART0, DXL_ID2, false);
    sleep_ms(1000);
    write_torqueEnable(&UART0, DXL_ID1, true);
    write_torqueEnable(&UART0, DXL_ID2, true);
}

// 　ハンドの動作実行
void hand_tick(hand_state_t* hand_state, bool* has_work, bool* hand_requested, absolute_time_t* hand_timer) {
    switch (*hand_state) {
        case HAND_IDLE:
            if (*hand_requested) {
                printf("hand requested\n");
                if (*has_work == false) {
                    *hand_requested = false;
                    *hand_state = HAND_LOWERING;
                    *hand_timer = make_timeout_time_ms(10);  // 降ろす時間
                    gpio_put(PUMP_PIN1, 1);                  // PWM デューティ設定

                    std::cout << "Hand lowering..." << std::endl;
                    // Dynamixel　降下処理
                    control_position(&UART0, DXL_ID2, DOWN_ANGLE);

                } else {
                    *hand_requested = false;
                    *hand_state = HAND_RELEASE;
                    *hand_timer = make_timeout_time_ms(10);
                    gpio_put(SOLENOID_PIN1, 1);  // ソレノイドを非吸着状態にする
                }
            }
            break;

        case HAND_LOWERING:
            if (absolute_time_diff_us(get_absolute_time(), *hand_timer) <= 0) {
                *hand_state = HAND_SUCTION_WAIT;
                *hand_timer = make_timeout_time_ms(10);  // 吸着待ち
                printf("Hand suction wait...\n");
            }
            break;

        case HAND_SUCTION_WAIT:
            if (absolute_time_diff_us(get_absolute_time(), *hand_timer) <= 0) {
                *hand_state = HAND_RAISING;
                *hand_timer = make_timeout_time_ms(10);
                // Dynamixel 昇降処理
                control_position(&UART0, DXL_ID2, UP_ANGLE);
                printf("Hand raising...\n");
            }
            break;

        case HAND_RAISING:
            if (absolute_time_diff_us(get_absolute_time(), *hand_timer) <= 0) {
                *has_work = true;
                control_position(&UART0, DXL_ID1, RELEASE_ANGLE);
                printf("Hand raised, work done.\n");
                *hand_state = HAND_IDLE;
            }
            break;

        case HAND_RELEASE:
            if (absolute_time_diff_us(get_absolute_time(), *hand_timer) <= 0) {
                *has_work = false;
                *hand_state = HAND_IDLE;
                gpio_put(SOLENOID_PIN1, 0);  // ソレノイドを吸着状態にする
                control_position(&UART0, DXL_ID1, GRAB_ANGLE);
                printf("Hand released\n");
            }
            break;
    }
}

// Core 1: 通信・デバッグ出力担当
void core1_entry(void) {
    gpio_put(SOLENOID_PIN1, 0);  // ソレノイドを吸着状態にする
    gpio_put(PUMP_PIN1, 1);
    printf("hand initialized\n");
    sleep_ms(500);
    control_position(&UART0, DXL_ID1, START_HAND_ANGLE);
    sleep_ms(500);
    control_position(&UART0, DXL_ID2, START_UP_ANGLE);
    while (1) {
        absolute_time_t next_time = make_timeout_time_ms(500);
        mutex_enter_blocking(&g_state_mutex);
        int speed = g_robot_state.motor_speed;
        int sensor = g_robot_state.sensor_value;
        mutex_exit(&g_state_mutex);
        // デバッグ出力
        // printf("[DEBUG] speed=%d, sensor=%d\n", speed, sensor);
        // 通信処理（例: USB出力やUART送信など）ここに追加可能
        mutex_enter_blocking(&g_hand_mutex);
        hand_tick(&g_hand_state, &g_has_work, &g_hand_requested, &g_hand_timer);
        mutex_exit(&g_hand_mutex);
        busy_wait_until(next_time);
    }
}

void request_hand_action() {
    mutex_enter_blocking(&g_hand_mutex);
    g_hand_requested = true;
    mutex_exit(&g_hand_mutex);
}

int main(void) {
    stdio_init_all();  // UARTなど初期化
    sleep_ms(2000);    // シリアル接続待ち
    // 初期化関数の実行
    init();
    sleep_ms(2000);

    // LEDのGPIO初期化
    gpio_init(PICO_DEFAULT_LED_PIN);
    gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);
    // ミューテックス初期化
    mutex_init(&g_hand_mutex);
    mutex_init(&g_state_mutex);
    g_robot_state.motor_speed = 0;
    g_robot_state.sensor_value = 0;
    // Core1で実行する関数を起動
    multicore_launch_core1(core1_entry);

    int counter = 0;  // デバッグ用カウンタ
    while (1) {
        absolute_time_t next_time = make_timeout_time_ms(500);  // 今から500ms後
        // LEDを点滅させる
        static bool led_on = false;
        led_on = !led_on;
        gpio_put(PICO_DEFAULT_LED_PIN, led_on);
        int new_sensor = rand() % 100;
        int new_speed = new_sensor * 2;
        // 状態を更新（排他制御あり）
        mutex_enter_blocking(&g_state_mutex);
        g_robot_state.motor_speed = new_speed;
        g_robot_state.sensor_value = new_sensor;
        mutex_exit(&g_state_mutex);

        if (counter % 10 == 0) {
            request_hand_action();
            printf("request_hand_action() called at counter=%d\n", counter);
        }
        counter++;

        busy_wait_until(next_time);  // 500ms待機
    }

    // gpio_put(SOLENOID_PIN1, 0);  // ソレノイドを非吸着状態にする
    gpio_put(PUMP_PIN1, 0);  // ポンプを停止
    return 0;
}
