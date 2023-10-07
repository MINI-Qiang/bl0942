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

//(1638.4 * 256 * BL0942_VREF * BL0942_VREF * BL0942_V_R1) / (3600000* 3537 * (BL0942_I_R1 * 1000 / BL0942_I_Rt) * BL0942_V_R2 * 1000);
#define BL0942_CF_CNT 0.00018729


#define BL0942_TIMEOUT                      200
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
		bl0942(uint8_t cs_io);
        void begin();
		void begin(HardwareSerial* hwSerial);

        float getVoltage();  //获取电压
        float getCurrent();   //获取电流
        float getActivePower();  //有功功率
        float getEnergy();      //总功率
        float getEnergy(uint32_t cf);
        float getFrequency();  //获取频率

        uint8_t WA_CREEP();
     
		
        long readRegister(uint8_t regAddress);
		long readRegister_spi(uint8_t regAddress);
        long readRegister(uint8_t regAddress,uint8_t deviceID);
        //bool writeModeRegister();
        //bool writeTpsRegister();
		
		bool writeRegister_spi(uint8_t regAddress, uint32_t regValue);
		
        //bool writeRegister(uint8_t regAddress, uint32_t regValue);

		uint8_t crc(uint8_t *ReqData, uint8_t dataLen);
        bool crc(uint8_t Addr ,uint8_t *ReqData, uint8_t dataLen,uint8_t _crc);
	 private:
        
        HardwareSerial* _serial;
		uint8_t _cs_io = 0;
        uint8_t _rawHolder[21];
        uint32_t lastRcv = 0;
        uint32_t V_RMS_ADC = 0;
        uint32_t I_RMS_ADC = 0;
        int32_t  W_RMS_ADC = 0;
        uint32_t W_CF_CNT = 0;
		
};

#endif  //_BL0942_H_