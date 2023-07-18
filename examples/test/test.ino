#include "bl0942.h"
bl0942 Bl;

void setup()
{
  Bl.begin(&Serial1);   //test rp2040
  Serial.begin(115200);
  pinMode(D3,OUTPUT);
  digitalWrite(D3,HIGH);

}

void loop() {
  float Vol = Bl.getVoltage();
  float Cur = Bl.getCurrent();
  float AcPower = Bl.getActivePower();
  float Energy = Bl.getEnergy();
  Serial.print(Vol);    // RMS Voltage  (V)
  Serial.print(",");
  Serial.print(Cur,3);  // RMS Current  (A)
  Serial.print(",");
  Serial.print(AcPower); // RMS Power   (W/h)
  Serial.print(",");
  Serial.println(Energy,4);     // (W)
  delay(1000);

}