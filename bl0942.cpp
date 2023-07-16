#include "bl0942.h"

bl0942::bl0942()
{

}


void bl0942::begin(HardwareSerial* hwSerial)
{
    _serial = hwSerial;
    _serial->begin(BL0942_DEFAULT_BAUD_RATE,BL0942_DEFAULT_PORT_CONFIG);
}





bool bl0942::crc(uint8_t *ReqData,uint8_t dataLen)
{
   //校验值计算
  uint8_t  startData = 0x58;
  uint8_t  checkData = 0;
  for(uint8_t a=0;a<dataLen;a++)
  {
    checkData = checkData + ReqData[a]; //校验和计算
  }
  checkData = checkData + startData;  //求总和
  checkData =  ~checkData;    //取反
  
  if (ReqData[22] != checkData) //校验值不通过
  {
    return false;
  }
  else
  {
    return true;
  }
}