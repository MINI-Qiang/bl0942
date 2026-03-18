#ifndef _BL0942_H_
#define _BL0942_H_

#include "Arduino.h"
#include "SPI.h"


//Electrical parameter register (read only) 
#define BL0942_VREF 1.218   //芯片参考基准源电压

#define BL0942_V_RMS_LBS 73989
#define BL0942_V_R1  110000
#define BL0942_V_R2  28.7   

#define BL0942_I_RMS_LBS 305978
#define BL0942_I_R1  2      //电流采样系数
#define BL0942_I_Rt  2000   //电流环变比

// Energy pulse coefficient
#define BL0942_CF_CNT ((1638.4 * 256 * BL0942_VREF * BL0942_VREF * BL0942_V_R1) / (3600000.0 * 3537.0 * (BL0942_I_R1 * 1000.0 / BL0942_I_Rt) * BL0942_V_R2 * 1000.0))


#define BL0942_TIMEOUT                      200
#define BL0942_DEFAULT_BAUD_RATE            4800
#define BL0942_DEFAULT_PORT_CONFIG          SERIAL_8N1
#define BL0942_DEFAULT_RMS_UPDATE           400
#define BL0942_DEFAULT_AC_FREQUENCY         50

// Register addresses - Read only
#define BL0942_REG_I_WAVE       0x01
#define BL0942_REG_V_WAVE       0x02
#define BL0942_REG_I_RMS        0x03
#define BL0942_REG_V_RMS        0x04
#define BL0942_REG_I_FAST_RMS   0x05
#define BL0942_REG_WATT         0x06
#define BL0942_REG_CF_CNT       0x07
#define BL0942_REG_FREQ         0x08
#define BL0942_REG_STATUS       0x09

// Register addresses - Read/Write
#define BL0942_REG_I_RMSOS      0x12
#define BL0942_REG_WA_CREEP     0x14
#define BL0942_REG_I_FAST_TH    0x15
#define BL0942_REG_I_FAST_CYC   0x16
#define BL0942_REG_FREQ_CYC     0x17
#define BL0942_REG_OT_FUNX      0x18
#define BL0942_REG_MODE         0x19
#define BL0942_REG_GAIN_CR      0x1A
#define BL0942_REG_SOFT_RESET   0x1C
#define BL0942_REG_USR_WRPROT   0x1D

// Communication mode
enum BL0942_CommMode { BL0942_COMM_UART, BL0942_COMM_SPI };

// Packet data structure for batch read (UART packet mode)
struct BL0942_Data {
    uint32_t i_rms;
    uint32_t v_rms;
    uint32_t i_fast_rms;
    int32_t  watt;
    uint32_t cf_cnt;
    uint16_t freq;
    uint16_t status;
};


class bl0942
{
    public:
        bl0942();
        bl0942(uint8_t cs_io);
        void begin();                                              // SPI mode
        void begin(HardwareSerial* hwSerial, uint8_t deviceAddr = 0); // UART mode

        float getVoltage();       // 获取电压 (V)
        float getCurrent();       // 获取电流 (A)
        float getActivePower();   // 有功功率 (W)
        float getEnergy();        // 总电量 (kWh)
        uint32_t getEnergyCF();   // 获取脉冲数
        float getEnergy(uint32_t cf);
        float getFrequency();     // 获取频率 (Hz)

        // Batch read all parameters (UART packet mode)
        bool readAll(BL0942_Data &data);

        // Configuration & status
        uint8_t getWaCreep();
        uint16_t getStatus();
        bool writeRegister(uint8_t regAddress, uint32_t regValue);
        void softReset();

        // Low-level register access
        long readRegister(uint8_t regAddress);

        uint8_t crc(uint8_t *ReqData, uint8_t dataLen);
        bool crc(uint8_t Addr, uint8_t *ReqData, uint8_t dataLen, uint8_t _crc);

    private:
        long readRegister_uart(uint8_t regAddress);
        long readRegister_spi(uint8_t regAddress);
        bool writeRegister_uart(uint8_t regAddress, uint32_t regValue);
        bool writeRegister_spi(uint8_t regAddress, uint32_t regValue);
        bool unlockWrite();

        BL0942_CommMode _mode = BL0942_COMM_UART;
        HardwareSerial* _serial = nullptr;
        uint8_t _cs_io = 0xFF;       // 0xFF = not configured
        uint8_t _deviceAddr = 0;     // UART device address (0~3)
        uint8_t _cmdRead = 0x58;     // Read command byte
        uint8_t _cmdWrite = 0xA8;    // Write command byte
        uint32_t lastRcv = 0;
        uint32_t V_RMS_RAW = 0;      // Last valid raw V_RMS ADC value
        uint32_t I_RMS_RAW = 0;      // Last valid raw I_RMS ADC value
        int32_t  W_RMS_RAW = 0;      // Last valid raw WATT ADC value
        uint32_t W_CF_CNT = 0;       // Last valid CF_CNT value
};

#endif  //_BL0942_H_