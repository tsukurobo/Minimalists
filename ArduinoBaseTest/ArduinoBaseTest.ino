#include <SPI.h>
#include <mcp_can.h>

long unsigned int rxId;
unsigned char len = 0;
unsigned char rxBuf[8];
byte txBuf[] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
long Pre_millis;
MCP_CAN CAN0(53);

int16_t fmap(double x, double in_min, double in_max, int16_t out_min, int16_t out_max) {
    return (x - in_min) * ((double)(out_max - out_min)) / (in_max - in_min) + out_min;
}

void setup() {
    Serial.begin(115200);
    if (CAN0.begin(MCP_ANY, CAN_1000KBPS, MCP_8MHZ) == CAN_OK) {
        Serial.println("CAN0: Init OK!");
        CAN0.setMode(MCP_NORMAL);
    } else {
        Serial.println("CAN0: Init Fail!");
    }

    Pre_millis = millis();
}

void loop() {
    static bool is_started = false;
    static int count = 0;
    static double motor_output_current_A = 0.0;
    if (Serial.available() > 0) {
        char mode = Serial.read();
        double val = Serial.readStringUntil('\n').toDouble();

        if (mode == 'c')  // 瀬戸しおを取る
        {
            is_started = true;
        } else if (mode == 'a') {
            motor_output_current_A = val;
        }
    }

    if (is_started) {
        const int total_count = 10000;
        if (count < total_count) {
            // 3種類のsin波を合成した複雑な波形を生成
            // 基本波: 1000カウントで1周期、振幅1.0A
            double freq1 = 2.0 * PI / 1000.0;
            double wave1 = 1.0 * sin(freq1 * count);

            // 高調波1: 500カウントで1周期、振幅0.5A
            double freq2 = 2.0 * PI / 500.0;
            double wave2 = 0.5 * sin(freq2 * count);

            // 高調波2: 333カウントで1周期、振幅0.3A、位相90度シフト
            double freq3 = 2.0 * PI / 333.0;
            double wave3 = 0.3 * sin(freq3 * count + PI / 2);

            //   低周波: 2000カウントで1周期、振幅0.5A
            double freq4 = 2.0 * PI / 2000.0;
            double wave4 = 0.5 * sin(freq4 * count);

            //   低周波: 5000カウントで1周期、振幅0.5A、位相180度シフト
            double freq5 = 2.0 * PI / 5000.0;
            double wave5 = 0.3 * sin(freq5 * count + PI);

            // 5つの波を合成（合計振幅が約2.8Aになるので、3Aでクリップ）
            double composite_wave = wave1 + wave2 + wave3 + wave4 + wave5;

            // -3A～+3Aの範囲にクリップ
            if (composite_wave > 3.0) {
                motor_output_current_A = 3.0;
            } else if (composite_wave < -3.0) {
                motor_output_current_A = -3.0;
            } else {
                motor_output_current_A = composite_wave;
            }
        } else {
            motor_output_current_A = 0.0;
            is_started = false;
            count = -1;
        }
        count += 1;
    }

    // 行ったり来たりの動作
    // if(is_started){
    //     const int stop_start = 1000;
    //     const int stop_end = 2000;
    //     const int move_end = 3000;
    //     if(0<= count && count <stop_start){
    //       motor_output_current_A = 1.0;
    //     } else if (stop_start <= count && count < stop_end){
    //       motor_output_current_A = 0.0;
    //     }else if (stop_end <= count && count < move_end){
    //       motor_output_current_A = -1.0;
    //     }else if(move_end <= count){
    //       motor_output_current_A = 0.0;
    //       is_started = false;
    //       count = -1;
    //     }
    //     count += 1;
    //   }

    // int16_t motor_ouput_current_Byte = fmap(motor_output_current_A, -20, 20, -16384, 16384);//2バイトに変換
    int16_t motor_ouput_current_Byte = (int16_t)(motor_output_current_A * (16384.0 / 20.0));

    txBuf[0] = (motor_ouput_current_Byte >> 8) & 0xFF;  // 上位バイト
    txBuf[1] = motor_ouput_current_Byte & 0xFF;         // 下位バイト

    // Send
    if (millis() - Pre_millis > 20) {  // Period: 20ms
        CAN0.sendMsgBuf(0x200, 0, 8, txBuf);
        Pre_millis = millis();
    }

    // Receive
    if (CAN0.checkReceive() == CAN_MSGAVAIL) {
        CAN0.readMsgBuf(&rxId, &len, rxBuf);
        // Serial.print("Recive ID: ");
        // Serial.print(rxId, HEX);
        // Serial.print(" Data: ");

        int16_t angle = rxBuf[0] << 8 | rxBuf[1];
        int16_t rpm = rxBuf[2] << 8 | rxBuf[3];
        int16_t amp = rxBuf[4] << 8 | rxBuf[5];
        int8_t temp = rxBuf[6];

        static int log_count = 0;
        if (log_count % 10) {
            Serial.print(motor_output_current_A);
            Serial.print(", ");
            // Serial.print(angle);
            // Serial.print(", ");
            Serial.println(rpm);
            // Serial.print(", ");
            // Serial.print(amp);
            // Serial.print(", ");
            // Serial.println(temp);
        }
        log_count++;
    }

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