#ifndef _BL0942_H_
#define _BL0942_H_

#include "Arduino.h"


//Electrical parameter register (read only) 
#define BL0942_I_FAST_RMS_REG_ADDR          0x00
#define BL0942_I_WAVE_REG_ADDR              0x01
#define BL0942_V_WAVE_REG_ADDR              0x03
#define BL0942_I_RMS_REG_ADDR               0x04
#define BL0942_V_RMS_REG_ADDR               0x06
#define BL0942_WATT_REG_ADDR                0x08
#define BL0942_CF_CNT_REG_ADDR              0x0A
#define BL0942_CORNER_REG_ADDR              0x0C
#define BL0942_TPS1_REG_ADDR                0x0E
#define BL0942_TPS2_REG_ADDR                0x0F
#define BL0942_READALL_REG_ADDR             0xAA

//User operated register (read and write)
#define BL0942_I_FAST_RMS_CTRL_REG_ADDR     0x10
#define BL0942_I_RMSOS_REG_ADDR             0x13
#define BL0942_WATTOS_REG_ADDR              0x15
#define BL0942_WA_CREEP_REG_ADDR            0x17
#define BL0942_MODE_REG_ADDR                0x18
#define BL0942_SOFT_RESET_REG_ADDR          0x19
#define BL0942_USR_WRPROT_REG_ADDR          0x1A
#define BL0942_TPS_CTRL_REG_ADDR            0x1B
#define BL0942_TPS2_A_REG_ADDR              0x1C
#define BL0942_TPS2_B_REG_ADDR              0x1D

#define BL0942_READ_CMD                     0x50    //0x58 in docs, bw-shp10 firmware send 0x50
#define BL0942_WRITE_CMD                    0xA0    //0xA8 in docs. Many thanks to Theo Arends @ https://github.com/arendst/ 
#define BL0942_UNLOCK_USER_REG              0x55

#define BL0942_FRM_HEAD_POS                 0x00
#define BL0942_FRM_ADDR_POS                 0x01
#define BL0942_FRM_SEND_MODE_HEAD_POS       0x02
#define BL0942_FRM_DATA_H_POS               0x04
#define BL0942_FRM_DATA_M_POS               0x03
#define BL0942_FRM_DATA_L_POS               0x02
#define BL0942_FRM_CRC_POS                  0x05

#define BL0942_IFRMS_FRM_POS                0x01
#define BL0942_IRMS_FRM_POS                 0x04
#define BL0942_VRMS_FRM_POS                 0x0A
#define BL0942_WATT_FRM_POS                 0x10
#define BL0942_CF_CNT_FRM_POS               0x16
#define BL0942_TPS1_FRM_POS                 0x1C
#define BL0942_TPS2_FRM_POS                 0x1F
#define BL0942_CRC_FRM_POS                  0x22

#define BL0942_IFRMS_HOLD_POS               0x00
#define BL0942_IRMS_HOLD_POS                0x03
#define BL0942_VRMS_HOLD_POS                0x06
#define BL0942_WATT_HOLD_POS                0x09
#define BL0942_CF_CNT_HOLD_POS              0x0C
#define BL0942_CORNER_HOLD_POS              0x0F
#define BL0942_TPS1_HOLD_POS                0x11
#define BL0942_TPS2_HOLD_POS                0x13

#define BL0942_MAX_REG_VALUE                0x1000000
#define BL0942_MAX_FRAME                    37
#define BL0942_SEND_MODE_FRAME_BYTES        35
#define BL0942_TIMEOUT                      50

#define BL0942_RMS_UPDATE_SEL_BITS          0b0000000100000000
#define BL0942_AC_FREQ_SEL_BIT              0b0000001000000000
#define BL0942_CF_UNABLE_BIT                0b0001000000000000
#define BL0942_RMS_REG_UPDATE_RATE_400MS    0b00000000
#define BL0942_RMS_REG_UPDATE_RATE_800MS    0b00000001
#define BL0942_AC_FREQ_50HZ                 0b00000000
#define BL0942_AC_FREQ_60HZ                 0b00000010
#define BL0942_CF_ENERGY_PULSE              0b00000000
#define BL0942_CF_ALARM_PULSE               0b00010000
#define BL0942_TEMPERATURE_SWITCH_BIT       0b1000000000000000
#define BL0942_ALARM_SWITCH_BIT             0b0100000000000000
#define BL0942_TEMPERATURE_SELECTOR_BIT     0b0011000000000000
#define BL0942_TEMPERATURE_INTERVAL_BIT     0b0000110000000000
#define BL0942_EXT_TEMPERATURE_THRHOLD_BIT  0b0000001111111111
#define BL0942_TEMPERATUTE_CTRL_ON          0b00000000
#define BL0942_TEMPERATUTE_CTRL_OFF         0b10000000
#define BL0942_TEMPERATURE_ALARM_ON         0b00000000
#define BL0942_OVERCURRENT_ALARM_ON         0b01000000
#define BL0942_TEMPERATURE_AUTO             0b00000000
#define BL0942_TEMPERATURE_SAME             0b00010000
#define BL0942_TEMPERATURE_INTERNAL         0b00100000
#define BL0942_TEMPERATURE_EXTERNAL         0b00110000
#define BL0942_TEMPERATURE_INTERVAL_50MS    0b00000000
#define BL0942_TEMPERATURE_INTERVAL_100MS   0b00000100
#define BL0942_TEMPERATURE_INTERVAL_200MS   0b00001000
#define BL0942_TEMPERATURE_INTERVAL_400MS   0b00001100

#define BL0942_TPS_CTRL_DEFAULT             0b0000011111111111      
                                                                    // Temperature measurement switch on
                                                                    // Temperature alarm ON - controled by MODE register
                                                                    // Automatic temperature measurement
                                                                    // 100mS Measurement interval
                                                                    // ~140°C External alarm threshold

#define BL0942_MODE_DEFAULT                 0b0000000000000000      // Energy pulse on CF pin
                                                                    // 50Hz AC frequency
                                                                    // 400mS RMS register update

#define BL0942_DEFAULT_BAUD_RATE            4800
#define BL0942_DEFAULT_PORT_CONFIG          SERIAL_8N1
#define BL0942_DEFAULT_RMS_UPDATE           400
#define BL0942_DEFAULT_AC_FREQUENCY         50




class bl0942
{
	public:
		bl0942();
		void begin(HardwareSerial* hwSerial);

        float getVoltage();  //获取电压
        float getCurrent();   //获取电流
        float getActivePower();  //有功功率
        float getReactivePower();  //无功功率
        float getApparentPower();   //视在功率
        float getEnergy();      //总功率
        float getEnergy(uint32_t cf);
        float getEnergyDelta();    //功率增量
        float getPhaseAngle();     //相位角
        float getPowerFactor(bool percentage=true);   //功率因数
        float getTemperature();    //温度
		
		/*
		void setRMSUpdate(uint8_t val = BL0942_RMS_REG_UPDATE_RATE_400MS);
        void setACFrequency(uint8_t val = BL0942_AC_FREQ_50HZ);
        void setCFPinMode(uint8_t val = BL0942_CF_ENERGY_PULSE);
        void setTSwitch(uint8_t val = BL0942_TEMPERATUTE_CTRL_ON);
        void setASwitch(uint8_t val = BL0942_TEMPERATURE_ALARM_ON);
        void setTSelector(uint8_t val = BL0942_TEMPERATURE_AUTO);
        void setTInterval(uint8_t val = BL0942_TEMPERATURE_INTERVAL_100MS);
		*/
		

        long readRegister(uint8_t regAddress);
        bool writeModeRegister();
        bool writeTpsRegister();
        bool writeRegister(uint8_t regAddress, uint32_t regValue);

	 private:
        HardwareSerial* _serial;
        uint8_t _rawHolder[21];
		
};

#endif  //_BL0942_H_