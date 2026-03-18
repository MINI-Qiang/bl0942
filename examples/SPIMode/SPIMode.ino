/**
 * BL0942 SPI 通信模式示例
 * 
 * 功能：通过 SPI 接口读取电参数
 * 接线：
 *   BL0942 SCK  -> MCU SCK
 *   BL0942 MOSI -> MCU MOSI
 *   BL0942 MISO -> MCU MISO
 *   BL0942 CS   -> MCU D10 (可自定义)
 */

#include "bl0942.h"

#define CS_PIN 10

bl0942 sensor(CS_PIN);

void setup() {
  Serial.begin(115200);
  sensor.begin();  // SPI 模式

  Serial.println("BL0942 SPI Mode Example");
  Serial.println("---");
}

void loop() {
  float voltage = sensor.getVoltage();
  float current = sensor.getCurrent();
  float power   = sensor.getActivePower();
  float energy  = sensor.getEnergy();

  if (voltage >= 0) {
    Serial.print("Voltage: ");  Serial.print(voltage, 1);  Serial.println(" V");
    Serial.print("Current: ");  Serial.print(current, 3);  Serial.println(" A");
    Serial.print("Power:   ");  Serial.print(power, 1);    Serial.println(" W");
    Serial.print("Energy:  ");  Serial.print(energy, 4);   Serial.println(" kWh");
    Serial.println();
  } else {
    Serial.println("SPI read error - check wiring and CS pin");
  }

  delay(1000);
}
