/**
 * BL0942 过流/过压报警示例
 * 
 * 功能：设定电流和电压阈值，超限时触发报警（LED / 蜂鸣器 / 继电器断开）
 * 适用场景：电源保护、负载监控、安全告警
 */

#include "bl0942.h"

bl0942 sensor;

#define ALARM_PIN    LED_BUILTIN  // 报警输出引脚（LED / 蜂鸣器 / 继电器）
#define RELAY_PIN    4            // 继电器控制引脚（可选）

// 阈值设定
const float OVER_CURRENT_A   = 10.0;   // 过流阈值 (A)
const float OVER_VOLTAGE_V   = 260.0;  // 过压阈值 (V)
const float UNDER_VOLTAGE_V  = 180.0;  // 欠压阈值 (V)
const float OVER_POWER_W     = 2200.0; // 过功率阈值 (W)

bool alarmActive = false;

void setup() {
  Serial.begin(115200);
  sensor.begin(&Serial1);

  pinMode(ALARM_PIN, OUTPUT);
  pinMode(RELAY_PIN, OUTPUT);
  digitalWrite(ALARM_PIN, LOW);
  digitalWrite(RELAY_PIN, HIGH);  // 继电器默认闭合（通电）

  Serial.println("BL0942 Over-Current/Voltage Alert");
  Serial.print("Limits: I>");  Serial.print(OVER_CURRENT_A);
  Serial.print("A, V>");       Serial.print(OVER_VOLTAGE_V);
  Serial.print("V, V<");       Serial.print(UNDER_VOLTAGE_V);
  Serial.print("V, P>");       Serial.print(OVER_POWER_W);
  Serial.println("W");
}

void loop() {
  float voltage = sensor.getVoltage();
  float current = sensor.getCurrent();
  float power   = sensor.getActivePower();

  // 检测异常
  bool fault = false;
  if (current > OVER_CURRENT_A) {
    Serial.print("[ALARM] Over-current: ");
    Serial.print(current, 3);  Serial.println(" A");
    fault = true;
  }
  if (voltage > OVER_VOLTAGE_V) {
    Serial.print("[ALARM] Over-voltage: ");
    Serial.print(voltage, 1);  Serial.println(" V");
    fault = true;
  }
  if (voltage > 10 && voltage < UNDER_VOLTAGE_V) {
    Serial.print("[ALARM] Under-voltage: ");
    Serial.print(voltage, 1);  Serial.println(" V");
    fault = true;
  }
  if (power > OVER_POWER_W) {
    Serial.print("[ALARM] Over-power: ");
    Serial.print(power, 1);  Serial.println(" W");
    fault = true;
  }

  if (fault && !alarmActive) {
    alarmActive = true;
    digitalWrite(ALARM_PIN, HIGH);
    digitalWrite(RELAY_PIN, LOW);   // 断开继电器
    Serial.println(">>> RELAY OFF - Load disconnected <<<");
  }

  if (!fault) {
    // 正常输出
    Serial.print(voltage, 1);  Serial.print("V  ");
    Serial.print(current, 3);  Serial.print("A  ");
    Serial.print(power, 1);    Serial.println("W");
  }

  delay(500);
}
