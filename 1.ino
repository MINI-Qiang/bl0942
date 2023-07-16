#include "bl0942.h"
bl0942 Bl;

void setup()
{
  Bl.begin(&Serial1);
  Serial.begin(115200);
  

}

void loop() {
  uint8_t Data = Bl.readRegister(0x08);
  //Serial.println(Data,HEX);
  delay(1000);

}