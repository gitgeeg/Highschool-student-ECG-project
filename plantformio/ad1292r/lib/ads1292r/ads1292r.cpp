#include "ads1292r.h"

// =====================================================================
// 构造函数
// =====================================================================
Ads1292R::Ads1292R()
    : _csPin(0), _drdyPin(0), _resetPin(0), _startPin(0), _lastDRDY(true) {
}

// =====================================================================
// 上电初始化 — 参考 STM32 ADS1292_PowerOnInit()
// 流程: START=0 → RESET=0(1秒) → RESET=1 → SDATAC → SPI RESET(0x06) → SDATAC
// =====================================================================
void Ads1292R::begin(int csPin, int drdyPin, int resetPin, int startPin) {
    _csPin = csPin;
    _drdyPin = drdyPin;
    _resetPin = resetPin;
    _startPin = startPin;

    // ---- 1. 配置引脚 ----
    pinMode(_csPin, OUTPUT);
    pinMode(_resetPin, OUTPUT);
    pinMode(_startPin, OUTPUT);
    pinMode(_drdyPin, INPUT);

    digitalWrite(_csPin, HIGH);
    digitalWrite(_resetPin, HIGH);
    digitalWrite(_startPin, LOW);   // 停止转换

    // ---- 2. 配置SPI接口 ----
    SPI.begin(18, 19, 23, csPin);
    SPI.setFrequency(1000000);
    SPI.setDataMode(SPI_MODE1);
    SPI.setHwCs(false);
    delay(10);

    // ---- 3. 硬件复位 (RESET引脚) ----
    // 拉低 RESET 保持 1 秒，确保 POR 完成且振荡器启动
    digitalWrite(_resetPin, LOW);
    delay(1000);
    digitalWrite(_resetPin, HIGH);
    delay(100);  // 等待稳定

    // ---- 4. SDATAC ----
    _sendCommand(ADS1292_CMD_SDATAC);
    delay(100);

    // ---- 5. SPI 复位命令 (0x06) ----
    // STM32 参考代码在 SDATAC 之后发送 RESET 命令
    _sendCommand(0x06);
    delay(1000);

    // ---- 6. SDATAC (复位后芯片回到 RDATAC，需再次退出) ----
    _sendCommand(ADS1292_CMD_SDATAC);
    delay(100);

    // ---- 7. 配置所有寄存器 ----
    _configureRegisters();

    // ---- 8. WAKEUP → RDATAC → START ----
    _sendCommand(ADS1292_CMD_WAKEUP);
    delay(10);
    _sendCommand(ADS1292_CMD_RDATAC);
    delay(10);
    digitalWrite(_startPin, HIGH);
    delay(10);
}

// =====================================================================
// 配置寄存器 — 参考 STM32 ADS1292_SET_REGBUFF() + ADS1292_WRITE_REGBUFF()
// =====================================================================
bool Ads1292R::_configureRegisters() {
    uint8_t regs[11];

    // ── CONFIG1 (0x01) ──────────────────────────────────────────────
    regs[0] = 0b00000101;   // CONFIG1: 4k SPS (per datasheet)
    // ── CONFIG2 (0x02) ──────────────────────────────────────────────
    regs[1] = 0b10100000;   // 
    // ── LOFF (0x03) ─────────────────────────────────────────────────
    regs[2] = 0b00010000;   // 默认
    // ── CH1SET (0x04) ──────────────────────────────────────────────
    regs[3] = 0b00100000;   // CH1 使能, 正常输入, 增益6
    // ── CH2SET (0x05) ──────────────────────────────────────────────
    regs[4] = 0b00110000;   // CH2 使能, 正常输入, 增益6
    regs[5] = 0b10101111;   // RLD缓冲使能, 内部参考
    regs[6] = 0b00000000;   // 关闭导联脱落检测
    // ── LOFF_STAT (0x08) ────────────────────────────────────────────
    regs[7] = 0b00000000;   // 默认 (只读)
    regs[8] = 0b11110010;   // 呼吸调制/解调使能, 64kHz
    regs[9] = 0b10000011;   // 呼吸相位配置, 校准关闭

    // ── GPIO (0x0B) ─────────────────────────────────────────────────
    //   0b 0000 0000
    //      |||| ||||
    //      |||| |||└─ GPI1 方向/数据
    //      |||| ||└── GPI2 方向/数据
    //      |||| │└─── 保留
    //      |||| └──── 保留
    //      ||│└────── 保留
    //      ││└─────── 保留
    //      │└──────── 保留
    //      └───────── 保留
    regs[10] = 0b00000000;  // 全部配置为输入，防止干扰

    // 写寄存器 1-11 (从CONFIG1开始, 共11个)
    _writeRegisterBurst(ADS1292_REG_CONFIG1, 11, regs);
    delay(10);

    // 回读验证
    uint8_t verify[12];
    _readRegisterBurst(ADS1292_REG_ID, 12, verify);

    bool ok = true;
    for (int i = 0; i < 11; i++) {
        // 跳过 ID(0), LOFF_STAT(8), GPIO(11) — 这些可能不匹配
        if (i == 0 || i == 7 || i == 10) continue;
        if (regs[i] != verify[i + 1]) {
            ok = false;
        }
    }
    return ok;
}

// =====================================================================
// 读取ECG和呼吸样本 — 呼吸使能时数据帧为27字节
// =====================================================================
// 读取原始 9 字节 (状态3 + CH1 24位 + CH2 24位)
// 返回裸字节，解析在 PC 端完成
// =====================================================================
// DRDY 下降沿检测:
//   每完成一次转换, ADS1292R 拉低 DRDY 表示数据就绪
//   SPI 读取 (发送 0xFF 产生 SCLK) 将数据移出, DRDY 自动恢复 HIGH
//   所以我们检测 HIGH→LOW 跳变来精准同步每个样本
// =====================================================================
bool Ads1292R::readSamples(Ads1292rData *data) {
    if (data == NULL) return false;

    // ──────────────────────────────────────────────────────────────
    //  DRDY 读取逻辑 (参考 STM32 + sketch_aug15a):
    //
    //  1. 先检查 DRDY 电平: LOW = 数据就绪 (参考 sketch_aug15a)
    //  2. 用 _lastDRDY 做下降沿去重: 防止同一个 DRDY 周期重复读取
    //
    //  原理:
    //    ADS1292R 完成一次转换后, /DRDY 拉低 → MCU 检测到 LOW
    //    → SPI 发送虚拟字节 (0xFF) 产生 SCLK 把数据"挤"出来
    //    → 读取完成后 /DRDY 自动恢复 HIGH
    //    → 等待下一次下降沿
    // ──────────────────────────────────────────────────────────────
    bool drdy = digitalRead(_drdyPin);

    if (drdy == LOW) {
        if (_lastDRDY) {
            // DRDY 从 HIGH→LOW (下降沿), 新数据就绪
            // 读 9 字节: 状态(3) + CH1(3) + CH2(3)
            _readRawData(data->raw, 9);
            _lastDRDY = false;  // 标记已读取, 等待 DRDY 恢复 HIGH
            return true;
        }
        // DRDY 仍为 LOW 但已读取过, 等待恢复 HIGH
        return false;
    } else {
        // DRDY = HIGH, 准备检测下一个下降沿
        _lastDRDY = true;
        return false;
    }
}

// =====================================================================
// 复位 — 拉低 RESET 引脚 (PWDN/RESET 低有效)
// =====================================================================
void Ads1292R::reset() {
    digitalWrite(_resetPin, LOW);
    delay(1000);  // STM32参考: 1秒
    digitalWrite(_resetPin, HIGH);
    delay(100);
}

// =====================================================================
// 读取设备ID
// =====================================================================
uint8_t Ads1292R::readDeviceId() {
    _sendCommand(ADS1292_CMD_SDATAC);
    delay(10);
    uint8_t id = _readRegister(ADS1292_REG_ID);
    _sendCommand(ADS1292_CMD_RDATAC);
    delay(10);
    return id;
}

// =====================================================================
// 设置采样率
// =====================================================================
void Ads1292R::setDataRate(uint8_t dr) {
    dr &= 0x07;
    _sendCommand(ADS1292_CMD_SDATAC);
    delay(10);
    _writeRegister(ADS1292_REG_CONFIG1, dr);
    delay(10);
    _sendCommand(ADS1292_CMD_RDATAC);
    delay(10);
}

// =====================================================================
// WAKEUP — 退出待机模式
// =====================================================================
void Ads1292R::wakeup() {
    _sendCommand(ADS1292_CMD_WAKEUP);
    delay(10);
}

// =====================================================================
// STANDBY — 进入待机模式
// =====================================================================
void Ads1292R::standby() {
    _sendCommand(ADS1292_CMD_STANDBY);
    delay(10);
}

// =====================================================================
// START/STOP 转换 (SPI命令方式)
// =====================================================================
void Ads1292R::startConv() {
    _sendCommand(ADS1292_CMD_START);
    delay(10);
}

void Ads1292R::stopConv() {
    _sendCommand(ADS1292_CMD_STOP);
    delay(10);
}

// =====================================================================
// 写单个寄存器
// =====================================================================
void Ads1292R::_writeRegister(uint8_t regAddr, uint8_t value) {
    digitalWrite(_csPin, LOW);
    delayMicroseconds(5);
    SPI.transfer(ADS1292_CMD_WREG | (regAddr & 0x1F));
    SPI.transfer(0x00);
    SPI.transfer(value);
    delayMicroseconds(5);
    digitalWrite(_csPin, HIGH);
}

// =====================================================================
// 读单个寄存器
// =====================================================================
uint8_t Ads1292R::_readRegister(uint8_t regAddr) {
    digitalWrite(_csPin, LOW);
    delayMicroseconds(5);
    SPI.transfer(ADS1292_CMD_RREG | (regAddr & 0x1F));
    SPI.transfer(0x00);
    uint8_t value = SPI.transfer(0xFF);
    delayMicroseconds(5);
    digitalWrite(_csPin, HIGH);
    return value;
}

// =====================================================================
// 突发写多个寄存器 (参考 STM32 ADS1292_WR_REGS)
// =====================================================================
void Ads1292R::_writeRegisterBurst(uint8_t startAddr, uint8_t count, uint8_t *data) {
    digitalWrite(_csPin, LOW);
    delayMicroseconds(5);
    SPI.transfer(ADS1292_CMD_WREG | (startAddr & 0x1F));
    SPI.transfer(count - 1);
    for (uint8_t i = 0; i < count; i++) {
        delayMicroseconds(5);
        SPI.transfer(data[i]);
    }
    delayMicroseconds(5);
    digitalWrite(_csPin, HIGH);
}

// =====================================================================
// 突发读多个寄存器
// =====================================================================
void Ads1292R::_readRegisterBurst(uint8_t startAddr, uint8_t count, uint8_t *data) {
    digitalWrite(_csPin, LOW);
    delayMicroseconds(5);
    SPI.transfer(ADS1292_CMD_RREG | (startAddr & 0x1F));
    SPI.transfer(count - 1);
    for (uint8_t i = 0; i < count; i++) {
        delayMicroseconds(5);
        data[i] = SPI.transfer(0xFF);
    }
    delayMicroseconds(5);
    digitalWrite(_csPin, HIGH);
}

// =====================================================================
// 发送SPI命令
// =====================================================================
void Ads1292R::_sendCommand(uint8_t cmd) {
    digitalWrite(_csPin, LOW);
    delayMicroseconds(5);
    SPI.transfer(cmd);
    delayMicroseconds(5);
    digitalWrite(_csPin, HIGH);
}

// =====================================================================
// 读取原始数据 — 发送虚拟字节 0xFF 产生 SCLK 时钟
//
// 在 RDATAC 模式下, ADS1292R 把数据放在输出移位寄存器中等待。
// MCU 发送虚拟字节 (0xFF 或 0x00, 任意值) 产生 SCLK 脉冲:
//   每 8 个 SCLK → 移出 1 字节数据到 MISO
//   读 9 字节需要发送 9 个虚拟字节 → 72 个 SCLK 脉冲
// 发送什么值不重要, 关键是 SCLK 把数据"挤"出来
//
// 参考:
//   STM32 (ADS1292.C):   SPI1_ReadWriteByte(0X00)  — 发 0x00
//   sketch_aug15a:       SPI.transfer(0xFF)         — 发 0xFF
//   当前:                SPI.transfer(0xFF)
// =====================================================================
void Ads1292R::_readRawData(uint8_t *buffer, uint8_t len) {
    digitalWrite(_csPin, LOW);
    delayMicroseconds(5);
    for (uint8_t i = 0; i < len; i++) {
        buffer[i] = SPI.transfer(0xFF);
    }
    delayMicroseconds(5);
    digitalWrite(_csPin, HIGH);
}

// =====================================================================
// 24位有符号转换
// =====================================================================
int32_t Ads1292R::_convert24BitToSigned(const uint8_t *data) {
    int32_t raw = ((int32_t)data[0] << 16) |
                  ((int32_t)data[1] << 8)  |
                  ((int32_t)data[2]);
    if (raw & 0x800000) {
        raw |= ~0xFFFFFF;
    }
    return raw;
}
