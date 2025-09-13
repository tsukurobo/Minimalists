#include "config.hpp"
// core0 から core1にhand実行時の監視
bool g_hand_requested = false;
mutex_t g_hand_mutex;
// 把持状態のstate
QuickArm_state_t g_quickarm_state = HAND_STANDBY;
absolute_time_t g_hand_timer;

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
    gpio_init(PUMP_PIN_SUB);
    gpio_set_dir(PUMP_PIN_SUB, GPIO_OUT);
    gpio_init(SOLENOID_PIN_SUB);
    gpio_set_dir(SOLENOID_PIN_SUB, GPIO_OUT);
    sleep_ms(1000);  // GPIO初期化後の安定化待ち
    // Dynamixelの設定
    printf("Initializing Dynamixels (Daisy Chain on UART0)...\n");
    init_crc();
    configure_uart(&UART0, BAUD_RATE);
    sleep_ms(100);
    write_statusReturnLevel(&UART0, DXL_ID5, 0x00);
    write_statusReturnLevel(&UART0, DXL_ID6, 0x00);
    sleep_ms(100);
    write_dxl_led(&UART0, DXL_ID5, true);
    write_dxl_led(&UART0, DXL_ID6, true);
    sleep_ms(100);
    write_dxl_led(&UART0, DXL_ID5, false);
    write_dxl_led(&UART0, DXL_ID6, false);
    sleep_ms(100);
    write_torqueEnable(&UART0, DXL_ID5, false);
    write_torqueEnable(&UART0, DXL_ID6, false);
    sleep_ms(100);
    write_position_Dgain(&UART0, DXL_ID5, 16000);
    write_position_Pgain(&UART0, DXL_ID5, 1600);
    write_position_Dgain(&UART0, DXL_ID6, 1900);
    write_position_Pgain(&UART0, DXL_ID6, 2100);
    sleep_ms(100);
    write_dxl_current_limit(&UART0, DXL_ID5, 1400);  // ID=1, 電流制限=100mA
    write_dxl_current_limit(&UART0, DXL_ID6, 1400);  // ID=2, 電流制限=100mA
    sleep_ms(100);
    write_operatingMode(&UART0, DXL_ID5, false);
    write_operatingMode(&UART0, DXL_ID6, false);
    sleep_ms(100);
    write_torqueEnable(&UART0, DXL_ID5, true);
    write_torqueEnable(&UART0, DXL_ID6, true);
    sleep_ms(500);
    control_position_multiturn(&UART0, DXL_ID5, START_HAND_ANGLE);
    sleep_ms(500);
    control_position_multiturn(&UART0, DXL_ID6, START_UP_ANGLE);
    sleep_ms(100);
    gpio_put(SOLENOID_PIN_SUB, 0);  // ソレノイドを吸着状態にする
    gpio_put(PUMP_PIN_SUB, 1);
}

// 　最速アーム実行
void exe_QuickArm(QuickArm_state_t* hand_state, bool* hand_requested, absolute_time_t* state_start_time) {
    uint32_t elapsed_ms = absolute_time_diff_us(*state_start_time, get_absolute_time()) / 1000;
    switch (*hand_state) {
        case HAND_STANDBY:
            if (*hand_requested) {
                printf("hand requested\n");
                *hand_requested = false;
                *hand_state = CATCHING_POSITON;
                *state_start_time = get_absolute_time();
                gpio_put(PUMP_PIN_SUB, 1);
                control_position_multiturn(&UART0, DXL_ID5, CATCH_ANGLE);
            }
            break;

        case CATCHING_POSITON:
            if (elapsed_ms >= 490) {  // 500
                *hand_state = HAND_DROPPING;
                *state_start_time = get_absolute_time();
                printf("Hand dropping...\n");
                control_position_multiturn(&UART0, DXL_ID6, LOWER_ANGLE);
            }
            break;

        case HAND_DROPPING:
            if (elapsed_ms >= 300) {
                *hand_state = CATCHING_WAIT;
                *state_start_time = get_absolute_time();
            }
            break;

        case CATCHING_WAIT:
            if (elapsed_ms >= 100) {
                *hand_state = HAND_LIFTING;
                *state_start_time = get_absolute_time();
                control_position_multiturn(&UART0, DXL_ID6, UPPER_ANGLE);
                printf("Hand raising...\n");
            }
            break;

        case HAND_LIFTING:
            if (elapsed_ms >= 490) {
                control_position_multiturn(&UART0, DXL_ID5, SHOOTING_ANGLE);
                printf("Hand raised, work done.\n");
                *state_start_time = get_absolute_time();
                *hand_state = SHOOTING_POSITION;
            }
            break;

        case SHOOTING_POSITION:
            if (elapsed_ms >= 450) {  // 1000
                *hand_state = HAND_FINISH;
                *state_start_time = get_absolute_time();
                gpio_put(SOLENOID_PIN_SUB, 1);  // ソレノイドを非吸着状態にする
                printf("Hand in shooting position, pump off.\n");
            }
            break;

        case HAND_FOLD:
            if (elapsed_ms >= 100) {
                control_position_multiturn(&UART0, DXL_ID5, INTER_POINT);
                *hand_state = HAND_FINISH;
            }

        case HAND_FINISH:
            if (elapsed_ms >= 450) {
                control_position_multiturn(&UART0, DXL_ID5, FOLD_ANGLE);
            }
            break;
    }
}

// Core 1: 通信・デバッグ出力担当
void core1_entry(void) {
    gpio_put(SOLENOID_PIN_SUB, 0);  // ソレノイドを吸着状態にする
    gpio_put(PUMP_PIN_SUB, 1);
    printf("hand initialized\n");
    sleep_ms(500);
    while (1) {
        absolute_time_t next_time = make_timeout_time_ms(500);
        mutex_enter_blocking(&g_state_mutex);
        int speed = g_robot_state.motor_speed;
        int sensor = g_robot_state.sensor_value;
        mutex_exit(&g_state_mutex);
        mutex_enter_blocking(&g_hand_mutex);
        exe_QuickArm(&g_quickarm_state, &g_hand_requested, &g_hand_timer);
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

    int counter = 1;
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
        if (counter == 10) {
            request_hand_action();
            printf("request_hand_action() called at counter=%d\n", counter);
        } else {
            printf("Counter: %d\n", counter);
        }
        counter++;
        busy_wait_until(next_time);  // 500ms待機
    }

    // gpio_put(SOLENOID_PIN_SUB, 0);  // ソレノイドを非吸着状態にする
    gpio_put(PUMP_PIN_SUB, 0);  // ポンプを停止
    return 0;
}
