/**
 * BL0942 快速测试示例 (RP2040)
 * 
 * 功能：读取电压、电流、功率、电量、频率，CSV 格式输出
 * 接线：BL0942 UART 连接到 Serial1，D3 用作使能引脚
 */

#include "bl0942.h"

bl0942 Bl;

void setup()
{
  Bl.begin(&Serial1);   // UART mode, default address 0
  Serial.begin(115200);
  pinMode(D3, OUTPUT);
  digitalWrite(D3, HIGH);

  Serial.println("BL0942 Quick Test");
  Serial.println("Voltage(V),Current(A),Power(W),Energy(kWh),Freq(Hz)");
}

void loop() {
  float Vol     = Bl.getVoltage();
  float Cur     = Bl.getCurrent();
  float AcPower = Bl.getActivePower();
  float Energy  = Bl.getEnergy();
  float Freq    = Bl.getFrequency();

  Serial.print(Vol);          // RMS Voltage  (V)
  Serial.print(",");
  Serial.print(Cur, 3);      // RMS Current  (A)
  Serial.print(",");
  Serial.print(AcPower);     // Active Power (W)
  Serial.print(",");
  Serial.print(Energy, 4);   // Energy       (kWh)
  Serial.print(",");
  Serial.println(Freq, 1);   // Frequency    (Hz)
  delay(1000);
}