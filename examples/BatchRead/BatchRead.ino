/**
 * BL0942 批量读取示例
 * 
 * 功能：使用 readAll() 一次性读取所有参数，效率更高
 * 说明：readAll() 仅支持 UART 模式，通过数据包一次获取全部测量值
 */

#include "bl0942.h"

bl0942 sensor;
BL0942_Data data;

void setup() {
  Serial.begin(115200);
  sensor.begin(&Serial1);

  Serial.println("BL0942 Batch Read Example");
  Serial.println("---");
}

void loop() {
  if (sensor.readAll(data)) {
    // 使用原始寄存器值手动换算，或直接用封装好的 get 函数
    float voltage = sensor.getVoltage();
    float current = sensor.getCurrent();
    float power   = sensor.getActivePower();
    float energy  = sensor.getEnergy();
    float freq    = sensor.getFrequency();

    Serial.print("V: ");    Serial.print(voltage, 1);   Serial.print(" V\t");
    Serial.print("I: ");    Serial.print(current, 3);   Serial.print(" A\t");
    Serial.print("P: ");    Serial.print(power, 1);     Serial.print(" W\t");
    Serial.print("E: ");    Serial.print(energy, 4);    Serial.print(" kWh\t");
    Serial.print("f: ");    Serial.print(freq, 1);      Serial.println(" Hz");

    // 也可以直接访问 raw 数据
    Serial.print("  [RAW] I_RMS=");  Serial.print(data.i_rms);
    Serial.print("  V_RMS=");        Serial.print(data.v_rms);
    Serial.print("  WATT=");         Serial.print(data.watt);
    Serial.print("  CF_CNT=");       Serial.print(data.cf_cnt);
    Serial.print("  STATUS=0x");     Serial.println(data.status, HEX);
  } else {
    Serial.println("readAll() failed - check wiring");
  }

  delay(1000);
}
