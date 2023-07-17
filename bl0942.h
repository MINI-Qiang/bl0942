#ifndef _BL0942_H_
#define _BL0942_H_

#include "Arduino.h"


//Electrical parameter register (read only) 
#define BL0942_VREF 1.218
#define BL0942_V_RMS_LBS 73989
#define BL0942_V_R1  110000
#define BL0942_V_R2  28.7

#define BL0942_I_RMS_LBS 305978
#define BL0942_I_R1  10



#define BL0942_TIMEOUT                     50
#define BL0942_DEFAULT_BAUD_RATE            4800
#define BL0942_DEFAULT_PORT_CONFIG          SERIAL_8N1
#define BL0942_DEFAULT_RMS_UPDATE           400
#define BL0942_DEFAULT_AC_FREQUENCY         50




struct frame_t 
{
    uint8_t Index;
    uint8_t Addr;
    uint8_t Payload[4];
    uint8_t Checksum;
};


class bl0942
{
	public:
		bl0942();
        void begin();
		void begin(HardwareSerial* hwSerial);

        float getVoltage();  //获取电压
        float getCurrent();   //获取电流
        float getActivePower();  //有功功率
        float getEnergy();      //总功率
        float getEnergy(uint32_t cf);
        float getFrequency();  //获取频率
     
		
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
        long readRegister(uint8_t regAddress,uint8_t deviceID);
        bool writeModeRegister();
        bool writeTpsRegister();
        bool writeRegister(uint8_t regAddress, uint32_t regValue);


        bool crc(uint8_t Addr ,uint8_t *ReqData, uint8_t dataLen,uint8_t _crc);
	 private:
        HardwareSerial* _serial;
        uint8_t _rawHolder[21];
        uint32_t lastRcv = 0;
        
		
};

#endif  //_BL0942_H_