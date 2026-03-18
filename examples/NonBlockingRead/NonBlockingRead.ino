/**
 * BL0942 非阻塞读取示例
 * 
 * 功能：使用 millis() 实现非阻塞定时读取，
 *       主循环中可同时处理其他任务（如按键、LED、通信）
 * 适用场景：需要同时运行多个任务的项目
 */

#include "bl0942.h"

bl0942 sensor;

// 定时器
unsigned long lastReadTime = 0;
const unsigned long READ_INTERVAL = 1000;  // 1秒读一次

// 示例：LED 闪烁任务
unsigned long lastBlinkTime = 0;
const unsigned long BLINK_INTERVAL = 200;
bool ledState = false;

// 存储最新读数
float voltage = 0, current = 0, power = 0;

void setup() {
  Serial.begin(115200);
  sensor.begin(&Serial1);
  pinMode(LED_BUILTIN, OUTPUT);

  Serial.println("BL0942 Non-Blocking Read Example");
}

void loop() {
  unsigned long now = millis();

  // 任务1：定时读取电参数
  if (now - lastReadTime >= READ_INTERVAL) {
    lastReadTime = now;

    voltage = sensor.getVoltage();
    current = sensor.getCurrent();
    power   = sensor.getActivePower();

    Serial.print(voltage, 1);  Serial.print("V  ");
    Serial.print(current, 3);  Serial.print("A  ");
    Serial.print(power, 1);    Serial.println("W");
  }

  // 任务2：LED 闪烁（演示非阻塞多任务）
  if (now - lastBlinkTime >= BLINK_INTERVAL) {
    lastBlinkTime = now;
    ledState = !ledState;
    digitalWrite(LED_BUILTIN, ledState);
  }

  // 任务3：串口命令处理
  if (Serial.available()) {
    char c = Serial.read();
    if (c == '?') {
      Serial.println("--- Current Readings ---");
      Serial.print("Voltage: "); Serial.print(voltage, 2); Serial.println(" V");
      Serial.print("Current: "); Serial.print(current, 4); Serial.println(" A");
      Serial.print("Power:   "); Serial.print(power, 2);   Serial.println(" W");
    }
  }
}
