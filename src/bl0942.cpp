#include "bl0942.h"

bl0942::bl0942()
{
}

void bl0942::begin(HardwareSerial *hwSerial)
{
    _serial = hwSerial;
    _serial->begin(4800);
}

void bl0942::begin()
{
}


//获取电力数据


float bl0942::getVoltage()  //获取电压
{
    uint8_t Addr = 0x04;
    uint32_t get_reg_data = readRegister(Addr);

    float Data = (get_reg_data*BL0942_VREF*BL0942_V_R1) / (BL0942_V_RMS_LBS * BL0942_V_R2 *1000);


    //float Data =  get_reg_data * BL0942_VREF / BL0942_V_RMS_LBS;
    //Data =  BL0942_V_R1 / BL0942_V_R2 * Data / 1000.0;
    //Data = Data / BL0942_V_R2 * (220 / 2);
    return Data;
}



float bl0942::getCurrent()   //获取电流() 
{
    uint8_t Addr = 0x03;
    uint32_t get_reg_data = readRegister(Addr);
    if(get_reg_data != 0)
    {
        I_RMS_ADC = get_reg_data;
    }
    else
    {
        get_reg_data = I_RMS_ADC;
    }

    float Data =  (get_reg_data * BL0942_VREF) / (BL0942_I_RMS_LBS* (BL0942_I_R1 * 1000) /  BL0942_I_Rt);
    //Data =  Data * 2000 / BL0942_I_R1 ; 
    return Data;

}


float bl0942::getActivePower()  //有功功率
{
    uint8_t Addr = 0x06;
    int32_t get_reg_data = readRegister(Addr);

    if(get_reg_data != 0)
    {
        W_RMS_ADC = get_reg_data;
    }
    else
    {
        get_reg_data = W_RMS_ADC;
    }

    float Data = (get_reg_data * BL0942_VREF * BL0942_VREF * BL0942_V_R1) / (3537 * (BL0942_I_R1 * 1000 / BL0942_I_Rt) * BL0942_V_R2 * 1000);
    return Data;
}


float bl0942::getEnergy()      //总功率
{
    uint8_t Addr = 0x07;
    uint32_t get_reg_data = readRegister(Addr);

     if(get_reg_data != 0)
    {
        W_CF_CNT = get_reg_data;
    }
    else
    {
        get_reg_data = W_CF_CNT;
    }

    float Data = get_reg_data *  BL0942_CF_CNT;
    return Data;
}






float bl0942::getFrequency()  //获取频率
{
    uint8_t Addr = 0x08;
    long get_reg_data = readRegister(Addr);
    float Data = 2.0 * 500000 / get_reg_data;  //ADC管脚电压  mv
    return Data;
}





long bl0942::readRegister(uint8_t regAddress)
{
    lastRcv = millis(); // 记录超时时间
    uint16_t Bytes = 4; // 目标接收数据量
    uint16_t Index = 0; // 当前接收数据量
    long regValue = 0;

    uint8_t Start = 0x58; // 读指令
    uint8_t regAddr = regAddress;
    uint8_t packet[] = {Start, regAddr};
    _serial->write(packet, sizeof(packet));

    /*
    预计的数据包长度
    循环检查串口缓冲区,如果有数据则
    */
    uint8_t Payload[Bytes];

    while ((Index < Bytes) && !((millis() - lastRcv) > BL0942_TIMEOUT)) // 检查串口收到足够数据,或者超时
    {
        uint8_t bytesAvailable = _serial->available(); // 获取缓冲区数据
        if (bytesAvailable > 0)                // 如果有数据
        {
            for( uint16_t bytesIndex=0; bytesIndex<bytesAvailable; bytesIndex++)
            {
                Payload[Index++] = (uint8_t)_serial->read();  //数据获取至缓冲区
            }
            lastRcv = millis();         //更新超时计数器
        }
    }

    if( Index != Bytes )  //超时返回如果数据不够,则错误返回
    {
        //_state = BADFRAME_ERR;
        return 0;
    }
    //数据返回
    
    
    if( crc(regAddr,Payload,sizeof(Payload),Payload[3]) != true )  //超时返回如果数据不够,则错误返回
    {
        //_state = BADFRAME_ERR;
        return 0;
    }
    
    // 数据处理成整数输出
    long Data = (((uint32_t)Payload[2] << 24) + ((uint32_t)Payload[1] << 16) + ((uint32_t)Payload[0]<<8)) >>8;
    return Data;
}


bool bl0942::crc(uint8_t Addr ,uint8_t *ReqData, uint8_t dataLen,uint8_t _crc)
{
    // 校验值计算
    uint8_t startData = 0x58;
    uint8_t checkData = 0;
    for (uint8_t a = 0; a < dataLen-1; a++)
    {
        checkData = checkData + ReqData[a]; // 校验和计算
    }
    checkData = checkData + startData + Addr; // 求总和
    checkData = ~checkData;            // 取反
    if (_crc != checkData) // 校验值不通过
    {
        return false;
    }
    else
    {
        return true;
    }
}