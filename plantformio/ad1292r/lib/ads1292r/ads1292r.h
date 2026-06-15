#ifndef ADS1292R_H
#define ADS1292R_H

#include <Arduino.h>
#include <SPI.h>

/*=========================================================================
    命令定义 — 参考 STM32 ADS1292.H
 *=======================================================================*/
#define ADS1292_CMD_WAKEUP    0x02
#define ADS1292_CMD_STANDBY   0x04
#define ADS1292_CMD_RESET     0x06
#define ADS1292_CMD_START     0x08
#define ADS1292_CMD_STOP      0x0A
#define ADS1292_CMD_RDATAC    0x10
#define ADS1292_CMD_SDATAC    0x11
#define ADS1292_CMD_RDATA     0x12
#define ADS1292_CMD_RREG      0x20
#define ADS1292_CMD_WREG      0x40
#define ADS1292_CMD_OFFSETCAL 0x1A

/*=========================================================================
    寄存器地址
 *=======================================================================*/
#define ADS1292_REG_ID        0
#define ADS1292_REG_CONFIG1   1
#define ADS1292_REG_CONFIG2   2
#define ADS1292_REG_LOFF      3
#define ADS1292_REG_CH1SET    4
#define ADS1292_REG_CH2SET    5
#define ADS1292_REG_RLDSENS   6
#define ADS1292_REG_LOFFSENS  7
#define ADS1292_REG_LOFFSTAT  8
#define ADS1292_REG_RESP1     9
#define ADS1292_REG_RESP2     10
#define ADS1292_REG_GPIO      11

/*=========================================================================
    ID 寄存器值
 *=======================================================================*/
#define DEVICE_ID_ADS1292     0x53
#define DEVICE_ID_ADS1292R    0x73

/*=========================================================================
    CONFIG1 — 采样率
 *=======================================================================*/
#define ADS1292_DR_125SPS     0x00
#define ADS1292_DR_250SPS     0x01
#define ADS1292_DR_500SPS     0x02
#define ADS1292_DR_1KSPS      0x03
#define ADS1292_DR_2KSPS      0x04
#define ADS1292_DR_4KSPS      0x05
#define ADS1292_DR_8KSPS      0x06
#define ADS1292_DR_16KSPS     0x07

/*=========================================================================
    CHnSET 字段定义 — 注意: 这些是直接写入寄存器的编码值
    GAIN编码: 000=6, 001=1, 010=2, 011=3, 100=4, 110=8, 111=12
 *=======================================================================*/
#define GAIN_CODE_6     0   // 增益6 (默认)
#define GAIN_CODE_1     1
#define GAIN_CODE_2     2
#define GAIN_CODE_3     3
#define GAIN_CODE_4     4
#define GAIN_CODE_8     6
#define GAIN_CODE_12    7

#define MUX_Normal_input    0   // 正常电极输入
#define MUX_input_shorted   1   // 输入短路
#define MUX_Test_signal     5   // 测试信号
#define MUX_VDD_signal      3   // VDD/2 信号

/*=========================================================================
    数据结构
 *=======================================================================*/
typedef struct {
    uint8_t raw[9];         // 原始 9 字节: 状态(3) + CH1(3) + CH2(3)
    int32_t channelData[2]; // 解析后的通道数据 (可选)
    int32_t respirationData;
    bool    leadOffDetected;
} Ads1292rData;

/*=========================================================================
    ADS1292R 驱动类
 *=======================================================================*/
class Ads1292R {
public:
    Ads1292R();

    void begin(int csPin, int drdyPin, int resetPin, int startPin);
    bool readSamples(Ads1292rData *data);
    void reset();
    void setDataRate(uint8_t dr);
    uint8_t readDeviceId();
    void wakeup();
    void standby();
    void startConv();
    void stopConv();

private:
    int _csPin;
    int _drdyPin;
    int _resetPin;
    int _startPin;
    bool _lastDRDY;          // 上一次 DRDY 电平, 用于下降沿检测

    bool _configureRegisters();
    void _writeRegister(uint8_t regAddr, uint8_t value);
    uint8_t _readRegister(uint8_t regAddr);
    void _writeRegisterBurst(uint8_t startAddr, uint8_t count, uint8_t *data);
    void _readRegisterBurst(uint8_t startAddr, uint8_t count, uint8_t *data);
    void _sendCommand(uint8_t cmd);
    void _readRawData(uint8_t *buffer, uint8_t len);
    int32_t _convert24BitToSigned(const uint8_t *data);
};

#endif // ADS1292R_H
