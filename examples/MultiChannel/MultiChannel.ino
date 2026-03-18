/**
 * BL0942 多通道（多地址）示例
 * 
 * 功能：同一 UART 总线上挂载多个 BL0942（最多 4 个，地址 0~3）
 * 接线：所有 BL0942 的 TX/RX 并联到同一个 Serial 端口
 *       通过 ADDR 引脚配置不同地址（见数据手册）
 * 
 * 注意：多芯片共用总线时，每次只能与一个地址通信
 */

#include "bl0942.h"

#define NUM_CHANNELS 3  // 实际使用的通道数（1~4）

bl0942 sensor[NUM_CHANNELS];

void setup() {
  Serial.begin(115200);

  // 初始化每个通道，指定不同的设备地址
  for (uint8_t i = 0; i < NUM_CHANNELS; i++) {
    sensor[i].begin(&Serial1, i);  // 地址 0, 1, 2
  }

  Serial.println("BL0942 Multi-Channel Example");
  Serial.print("Channels: "); Serial.println(NUM_CHANNELS);
  Serial.println("CH\tVoltage\tCurrent\tPower\tEnergy");
}

void loop() {
  for (uint8_t ch = 0; ch < NUM_CHANNELS; ch++) {
    float voltage = sensor[ch].getVoltage();
    float current = sensor[ch].getCurrent();
    float power   = sensor[ch].getActivePower();
    float energy  = sensor[ch].getEnergy();

    Serial.print("CH");    Serial.print(ch);     Serial.print("\t");
    Serial.print(voltage, 1);                     Serial.print("V\t");
    Serial.print(current, 3);                     Serial.print("A\t");
    Serial.print(power, 1);                       Serial.print("W\t");
    Serial.print(energy, 4);                      Serial.println("kWh");

    delay(50);  // 通道间短暂延时，避免总线冲突
  }

  Serial.println("---");
  delay(2000);
}
