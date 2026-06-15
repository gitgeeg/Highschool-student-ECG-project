/**
 * ADS1292R ECG 采集主程序
 * 
 * 参考实现:
 *   - STM32 (ECG/ADS1292.C):  DRDY 外部中断下降沿触发, SPI 发 0x00 产生 SCLK
 *   - ESP32 (sketch_aug15a):  轮询 DRDY==LOW, SPI 发 0xFF 产生 SCLK
 *   - 当前: 轮询 + 下降沿去重, SPI 发 0xFF 产生 SCLK
 * 
 * SPI 读取原理 (RDATAC 模式):
 *   ADS1292R 将数据放在输出移位寄存器中等待。
 *   MCU 发送虚拟字节 (0xFF) 产生 SCLK 脉冲:
 *     每 8 个 SCLK → 移出 1 字节到 MISO
 *     9 字节 = 72 个 SCLK 脉冲
 *   发送什么值不重要, 关键是 SCLK 把数据"挤"出来。
 * 
 * 硬件连接 (ESP32标准版):
 *   ADS1292R  ->  ESP32
 *   CS        ->  GPIO5
 *   DRDY      ->  GPIO4
 *   RESET     ->  GPIO17
 *   START     ->  GPIO16
 *   SCLK      ->  GPIO18
 *   Dout(MISO)->  GPIO19
 *   Din(MOSI) ->  GPIO23
 * 
 * 串口输出格式:
 *   每行 18 个 hex 字符 (9 字节裸数据, PC 端解析)
 *   字节 0-2: 状态 (24位, 导联脱落标志)
 *   字节 3-5: CH1 (24位有符号)
 *   字节 6-8: CH2 (24位有符号)
 */

#include <Arduino.h>
#include <SPI.h>
#include "ads1292r.h"

// ========== 引脚定义 ==========
#define PIN_CS      5
#define PIN_DRDY    4
#define PIN_RESET   17
#define PIN_START   16

#define PIN_SCLK    18
#define PIN_MISO    19
#define PIN_MOSI    23

// ========== 全局对象 ==========
Ads1292R ads1292r;

// ========== 初始化 ==========
void setup() {
    Serial.begin(921600);
    delay(3000);

    Serial.println(F("=== ADS1292R ECG 采集 (参考STM32流程) ==="));
    Serial.println(F("引脚: CS=5 DRDY=4 RESET=17 START=16 SCLK=18 MISO=19 MOSI=23"));

    // 检查初始 DRDY 电平
    pinMode(PIN_DRDY, INPUT);
    Serial.print(F("[诊断] DRDY 初始电平: "));
    Serial.println(digitalRead(PIN_DRDY));

    // 调用库初始化
    Serial.println(F("[诊断] 调用 begin()..."));
    ads1292r.begin(PIN_CS, PIN_DRDY, PIN_RESET, PIN_START);

    // 读取设备 ID
    uint8_t id = ads1292r.readDeviceId();
    Serial.print(F("[诊断] 设备 ID = 0x"));
    if (id < 0x10) Serial.print("0");
    Serial.print(id, HEX);
    if (id == DEVICE_ID_ADS1292R) {
        Serial.println(F(" ✓ (ADS1292R)"));
    } else if ((id & 0xF0) == 0x70) {
        Serial.println(F(" (ADS1292R 系列, 非标准ID)"));
    } else {
        Serial.println(F(" ✗ (异常)"));
    }

    Serial.println(F("初始化完成，开始采集..."));
}

// ========== 主循环 ==========
void loop() {
    Ads1292rData data;

    // 读取样本（非阻塞，仅当DRDY拉低时返回数据）
    if (ads1292r.readSamples(&data)) {
        // 输出原始 9 字节为 hex 格式, PC 端解析
        // 格式: 9 个十六进制字节
        for (int i = 0; i < 9; i++) {
            if (data.raw[i] < 0x10) Serial.print('0');
            Serial.print(data.raw[i], HEX);
        }
        Serial.println();
    }
}