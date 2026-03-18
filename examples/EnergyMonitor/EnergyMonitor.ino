/**
 * BL0942 电量监测示例
 * 
 * 功能：持续监测累计电量 (kWh)，定时输出统计信息
 *       包含运行时间、平均功率、功率因数估算
 * 适用场景：智能插座、电能计量、用电分析
 */

#include "bl0942.h"

bl0942 sensor;

unsigned long startTime;
unsigned long lastPrintTime;
const unsigned long PRINT_INTERVAL = 5000;  // 5秒打印一次

// 统计变量
float maxVoltage = 0, minVoltage = 9999;
float maxCurrent = 0;
float maxPower   = 0;
uint32_t sampleCount = 0;
float powerSum = 0;

void setup() {
  Serial.begin(115200);
  sensor.begin(&Serial1);

  startTime = millis();
  lastPrintTime = startTime;

  Serial.println("=== BL0942 Energy Monitor ===");
  Serial.println("Monitoring started...");
  Serial.println();
}

void loop() {
  float voltage = sensor.getVoltage();
  float current = sensor.getCurrent();
  float power   = sensor.getActivePower();
  float energy  = sensor.getEnergy();
  float freq    = sensor.getFrequency();

  // 更新统计（仅在有效读数时）
  if (voltage > 0) {
    if (voltage > maxVoltage) maxVoltage = voltage;
    if (voltage < minVoltage) minVoltage = voltage;
  }
  if (current > maxCurrent) maxCurrent = current;
  if (power > maxPower)     maxPower = power;
  powerSum += power;
  sampleCount++;

  // 定时输出
  if (millis() - lastPrintTime >= PRINT_INTERVAL) {
    lastPrintTime = millis();
    unsigned long elapsed = (millis() - startTime) / 1000;

    Serial.println("──────────────────────────────");
    Serial.print("Runtime: ");
    Serial.print(elapsed / 3600); Serial.print("h ");
    Serial.print((elapsed % 3600) / 60); Serial.print("m ");
    Serial.print(elapsed % 60); Serial.println("s");

    Serial.print("Voltage:  "); Serial.print(voltage, 1);  Serial.print(" V");
    Serial.print("  (min: ");   Serial.print(minVoltage, 1);
    Serial.print(", max: ");    Serial.print(maxVoltage, 1);
    Serial.println(")");

    Serial.print("Current:  "); Serial.print(current, 3);  Serial.print(" A");
    Serial.print("  (max: ");   Serial.print(maxCurrent, 3);
    Serial.println(")");

    Serial.print("Power:    "); Serial.print(power, 1);    Serial.print(" W");
    Serial.print("  (max: ");   Serial.print(maxPower, 1);
    Serial.println(")");

    float avgPower = (sampleCount > 0) ? (powerSum / sampleCount) : 0;
    Serial.print("Avg Power: "); Serial.print(avgPower, 1); Serial.println(" W");

    Serial.print("Energy:   "); Serial.print(energy, 4);   Serial.println(" kWh");
    Serial.print("Freq:     "); Serial.print(freq, 1);     Serial.println(" Hz");

    // 视在功率与功率因数估算
    float apparentPower = voltage * current;
    if (apparentPower > 0.1) {
      float pf = power / apparentPower;
      if (pf > 1.0) pf = 1.0;
      Serial.print("PF:       "); Serial.println(pf, 2);
    }

    Serial.println();
  }

  delay(500);
}
