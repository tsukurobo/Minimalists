#include "config.hpp"

// core0 から core1にhand実行時の監視
volatile bool g_hand_requested = false;
mutex_t g_hand_mutex;
// work　の保持状態
volatile bool g_has_work = false;
// 把持状態のstate
hand_state_t g_hand_state = HAND_IDLE;
absolute_time_t g_hand_timer;

uint pwm0_slice_num;
uint pwm1_slice_num;
uint chan;

// 共有データ構造体
typedef struct
{
    int motor_speed;
    int sensor_value;
} robot_state_t;

// 共有状態とミューテックス
static robot_state_t g_robot_state;
static mutex_t g_state_mutex;

bool task = false;

void init() {
    // ポンプの設定
    pump_init(PUMP_PIN1);
    gpio_set_dir(PUMP_PIN1, GPIO_OUT);
    gpio_init(PUMP_PIN2);
    gpio_set_dir(PUMP_PIN2, GPIO_OUT);
    gpio_init(SOLENOID_PIN1);
    gpio_set_dir(SOLENOID_PIN1, GPIO_OUT);

    gpio_init(SOLENOID_PIN2);
    gpio_set_dir(SOLENOID_PIN2, GPIO_OUT);

    sleep_ms(1000);  // GPIO初期化後の安定化待ち

    // Dynamixelの設定
    init_crc();
    configure_uart(&UART0, BAUD_RATE);
    configure_uart(&UART1, BAUD_RATE);
    write_statusReturnLevel(&UART0, DXL_ID1, 0x00);
    write_statusReturnLevel(&UART1, DXL_ID2, 0x00);
    write_dxl_led(&UART0, DXL_ID1, true);
    write_dxl_led(&UART1, DXL_ID2, true);
    sleep_ms(1000);
    write_dxl_led(&UART0, DXL_ID1, false);
    write_dxl_led(&UART1, DXL_ID2, false);
    sleep_ms(1000);
    write_torqueEnable(&UART0, DXL_ID1, false);
    write_torqueEnable(&UART1, DXL_ID2, false);
    write_operatingMode(&UART0, DXL_ID1, true);  // false : 位置制御, true : 拡張位置制御(マルチターン)
    write_operatingMode(&UART1, DXL_ID2, false);
    write_torqueEnable(&UART0, DXL_ID1, true);
    write_torqueEnable(&UART1, DXL_ID2, true);
}

// 　ハンドの動作実行
void hand_tick(hand_state_t* hand_state, volatile bool* has_work, volatile bool* hand_requested, absolute_time_t* hand_timer) {
    switch (*hand_state) {
        case HAND_IDLE:
            if (*hand_requested) {
                if (*has_work == false) {
                    *hand_requested = false;
                    *hand_state = HAND_LOWERING;
                    *hand_timer = make_timeout_time_ms(1000);  // 降ろす時間
                    solenoid_input(1, SOLENOID_PIN1, SOLENOID_PIN2, pwm1_slice_num);
                    pwm_set_chan_level(pwm0_slice_num, chan, 149 * PUMP_PWM);  // PWM デューティ設定
                    gpio_put(PUMP_PIN2, true);                                 // ポンプの方向ピンを正転に設定

                    std::cout << "Hand lowering..." << std::endl;
                    // Dynamixel 手先制御
                    control_position(&UART1, DXL_ID1, GRAB_ANGLE);
                    // Dynamixel　降下処理
                    control_position(&UART0, DXL_ID2, DOWN_ANGLE);

                } else {
                    *hand_requested = false;
                    *hand_state = HAND_RELEASE;
                    *hand_timer = make_timeout_time_ms(2000);
                    solenoid_input(0, SOLENOID_PIN1, SOLENOID_PIN2, pwm1_slice_num);  // ソレノイドを非吸着状態にする
                }
            }
            break;

        case HAND_LOWERING:
            if (absolute_time_diff_us(get_absolute_time(), *hand_timer) <= 0) {
                *hand_state = HAND_SUCTION_WAIT;
                *hand_timer = make_timeout_time_ms(5000);  // 吸着待ち
                std::cout << "Hand suction wait..." << std::endl;
            }
            break;

        case HAND_SUCTION_WAIT:
            if (absolute_time_diff_us(get_absolute_time(), *hand_timer) <= 0) {
                *hand_state = HAND_RAISING;
                *hand_timer = make_timeout_time_ms(2000);
                // Dynamixel 昇降処理
                control_position(&UART1, DXL_ID1, UP_ANGLE);
                control_position(&UART1, DXL_ID2, RELEASE_ANGLE);

                std::cout << "Hand raising..." << std::endl;
            }
            break;

        case HAND_RAISING:
            if (absolute_time_diff_us(get_absolute_time(), *hand_timer) <= 0) {
                *has_work = true;
                *hand_state = HAND_IDLE;

                std::cout << "Hand raised, work done." << std::endl;
            }
            break;

        case HAND_RELEASE:
            if (absolute_time_diff_us(get_absolute_time(), *hand_timer) <= 0) {
                *has_work = false;
                *hand_state = HAND_IDLE;
                solenoid_input(1, SOLENOID_PIN1, SOLENOID_PIN2, pwm1_slice_num);  // ソレノイドを吸着状態にする
                task = true;                                                      // デバッグ用フラグ
                std::cout << "Hand released, task completed." << std::endl;
            }
            break;
    }
}

// Core 1: 通信・デバッグ出力担当
void core1_entry(void) {
    while (1) {
        absolute_time_t next_time = make_timeout_time_ms(250);
        mutex_enter_blocking(&g_state_mutex);
        int speed = g_robot_state.motor_speed;
        int sensor = g_robot_state.sensor_value;
        mutex_exit(&g_state_mutex);
        // デバッグ出力
        printf("[DEBUG] speed=%d, sensor=%d\n", speed, sensor);
        // 通信処理（例: USB出力やUART送信など）ここに追g_hand_requested加可能
        // hand_tick(g_hand_state, g_has_work, , g_hand_timer);
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
    static pwm_config pwm0_slice_config;
    chan = pwm_gpio_to_channel(PUMP_PIN1);
    gpio_set_function(PUMP_PIN1, GPIO_FUNC_PWM);
    gpio_set_function(SOLENOID_PIN1, GPIO_FUNC_PWM);
    pwm0_slice_num = pwm_gpio_to_slice_num(PUMP_PIN1);
    pwm0_slice_config = pwm_get_default_config();
    pwm_config_set_wrap(&pwm0_slice_config, 149);  // 周期設定
    pwm_config_set_clkdiv(&pwm0_slice_config, 1);
    pwm_init(pwm0_slice_num, &pwm0_slice_config, true);  // PWM初期化

    pwm1_slice_num = pwm_gpio_to_slice_num(SOLENOID_PIN1);
    pwm_init(pwm1_slice_num, &pwm0_slice_config, true);  // ソレノイド用PWM初期化
    solenoid_input(1, SOLENOID_PIN1, SOLENOID_PIN2, pwm1_slice_num);
    sleep_ms(2000);

    // LEDのGPIO初期化
    gpio_init(PICO_DEFAULT_LED_PIN);
    gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);
    // ミューテックス初期化
    mutex_init(&g_state_mutex);
    g_robot_state.motor_speed = 0;
    g_robot_state.sensor_value = 0;
    // Core1で実行する関数を起動
    multicore_launch_core1(core1_entry);

    int counter = 0;                                                  // デバッグ用カウンタ
    solenoid_input(1, SOLENOID_PIN1, SOLENOID_PIN2, pwm1_slice_num);  // ソレノイドを吸着状態にする
    while (1) {
        absolute_time_t next_time = make_timeout_time_ms(500);  // 今から500ms後
        // LEDを点滅させる
        static bool led_on = false;
        led_on = !led_on;
        gpio_put(PICO_DEFAULT_LED_PIN, led_on);

        control_position(&UART1, DXL_ID1, RELEASE_ANGLE);
        std::cout << "Dynamixel ID1 position set to RELEASE_ANGLE: " << RELEASE_ANGLE << std::endl;
        sleep_ms(2000);
        control_position(&UART1, DXL_ID2, DOWN_ANGLE);
        std::cout << "Dynamixel ID2 position set to DOWN_ANGLE: " << DOWN_ANGLE << std::endl;
        sleep_ms(2000);
        control_position(&UART1, DXL_ID1, GRAB_ANGLE);
        std::cout << "Dynamixel ID1 position set to GRAB_ANGLE: " << GRAB_ANGLE << std::endl;
        sleep_ms(2000);
        control_position(&UART1, DXL_ID2, UP_ANGLE);
        std::cout << "Dynamixel ID2 position set to UP_ANGLE: " << UP_ANGLE << std::endl;

        if (counter > 50) {
            std::cout << "Task executed." << std::endl;
            solenoid_input(0, SOLENOID_PIN1, SOLENOID_PIN2, pwm1_slice_num);  // ソレノイドを非吸着状態にする
            break;
        } else if (counter == 15) {
            std::cout << "Countr is 15" << std::endl;
            solenoid_input(0, SOLENOID_PIN1, SOLENOID_PIN2, pwm1_slice_num);  // ソレノイドを非吸着状態にする
        } else if (counter == 40) {
            std::cout << "Countr is 40" << std::endl;
            solenoid_input(1, SOLENOID_PIN1, SOLENOID_PIN2, pwm1_slice_num);  // ソレノイドを吸着状態にする
        } else {
            std::cout << "Counter: " << counter << std::endl;
            // std::cout << "Solenoid not activated." << std::endl;
            // solenoid_input(1, SOLENOID_PIN1, SOLENOID_PIN2, pwm1_slice_num);
        }
        counter++;
        // ループごとに点灯/消灯を切り替え（点滅周期は制御周期の2倍）
        // センサ読み取りや制御計算の疑似処理
        int new_sensor = rand() % 100;
        int new_speed = new_sensor * 2;
        // 状態を更新（排他制御あり）
        mutex_enter_blocking(&g_state_mutex);
        g_robot_state.motor_speed = new_speed;
        g_robot_state.sensor_value = new_sensor;
        mutex_exit(&g_state_mutex);
        // 実際のモータ制御などをここで行う（PWM制御など）
        // motor_set_speed(new_speed);

        busy_wait_until(next_time);  // 500ms待機
    }

    solenoid_input(0, SOLENOID_PIN1, SOLENOID_PIN2, pwm1_slice_num);
    pump_set_speed(PUMP_PIN1, 0);
    pump_set_speed(PUMP_PIN2, 0);
    return 0;
}
