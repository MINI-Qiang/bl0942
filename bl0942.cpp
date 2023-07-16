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
    
   
    Serial.print(Payload[0],HEX);
    Serial.print(",");
    Serial.print(Payload[1],HEX);
    Serial.print(",");
    Serial.print(Payload[2],HEX);
    Serial.print(",");
    Serial.print(Payload[3],HEX);
    Serial.println();
    
    return Payload[0];
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