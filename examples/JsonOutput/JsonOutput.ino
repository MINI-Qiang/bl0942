/**
 * BL0942 JSON 格式输出示例
 * 
 * 功能：将电参数以 JSON 格式通过串口输出
 * 适用场景：与上位机/Node-RED/MQTT/Home Assistant 对接
 *           每行一个完整 JSON 对象，便于解析
 */

#include "bl0942.h"

bl0942 sensor;

unsigned long lastSend = 0;
const unsigned long SEND_INTERVAL = 2000;  // 2秒发送一次

void setup() {
  Serial.begin(115200);
  sensor.begin(&Serial1);
}

void loop() {
  if (millis() - lastSend < SEND_INTERVAL) return;
  lastSend = millis();

  float voltage = sensor.getVoltage();
  float current = sensor.getCurrent();
  float power   = sensor.getActivePower();
  float energy  = sensor.getEnergy();
  float freq    = sensor.getFrequency();

  // 视在功率与功率因数
  float apparentPower = voltage * current;
  float pf = (apparentPower > 0.1) ? (power / apparentPower) : 0;
  if (pf > 1.0) pf = 1.0;

  // 输出 JSON（不依赖 ArduinoJson 库，手动拼接）
  Serial.print("{\"voltage\":");    Serial.print(voltage, 2);
  Serial.print(",\"current\":");    Serial.print(current, 4);
  Serial.print(",\"power\":");      Serial.print(power, 2);
  Serial.print(",\"energy\":");     Serial.print(energy, 4);
  Serial.print(",\"frequency\":");  Serial.print(freq, 2);
  Serial.print(",\"pf\":");         Serial.print(pf, 3);
  Serial.print(",\"apparent\":");   Serial.print(apparentPower, 2);
  Serial.print(",\"uptime\":");     Serial.print(millis() / 1000);
  Serial.println("}");
}
