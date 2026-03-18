/**
 * BL0942 基础读取示例
 * 
 * 功能：逐个读取电压、电流、功率、电量、频率并通过串口输出
 * 接线：BL0942 UART TX/RX 连接到 Serial1
 */

#include "bl0942.h"

bl0942 sensor;

void setup() {
  Serial.begin(115200);
  sensor.begin(&Serial1);  // UART 模式，默认地址 0

  Serial.println("BL0942 Basic Reading Example");
  Serial.println("Voltage(V), Current(A), Power(W), Energy(kWh), Freq(Hz)");
}

void loop() {
  float voltage  = sensor.getVoltage();
  float current  = sensor.getCurrent();
  float power    = sensor.getActivePower();
  float energy   = sensor.getEnergy();
  float freq     = sensor.getFrequency();

  Serial.print(voltage, 1);
  Serial.print(",\t");
  Serial.print(current, 3);
  Serial.print(",\t");
  Serial.print(power, 1);
  Serial.print(",\t");
  Serial.print(energy, 4);
  Serial.print(",\t");
  Serial.println(freq, 1);

  delay(1000);
}
