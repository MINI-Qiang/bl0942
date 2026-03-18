/**
 * BL0942 寄存器直接访问示例
 * 
 * 功能：演示底层寄存器读写操作，包括：
 *   - 读取状态寄存器
 *   - 读取原始 ADC 波形数据
 *   - 配置寄存器写入
 *   - 软复位
 * 适用场景：调试、校准、高级配置
 */

#include "bl0942.h"

bl0942 sensor;

void setup() {
  Serial.begin(115200);
  sensor.begin(&Serial1);

  Serial.println("BL0942 Register Access Example");
  Serial.println("Commands via Serial:");
  Serial.println("  r  - Read all registers");
  Serial.println("  w  - Read waveform data");
  Serial.println("  s  - Read status");
  Serial.println("  x  - Soft reset");
  Serial.println("---");
}

void printRegHex(const char* name, uint8_t reg) {
  long val = sensor.readRegister(reg);
  Serial.print(name);
  Serial.print(" (0x");
  if (reg < 0x10) Serial.print("0");
  Serial.print(reg, HEX);
  Serial.print("): ");
  if (val < 0) {
    Serial.println("READ ERROR");
  } else {
    Serial.print("0x");
    if (val < 0x100000) Serial.print("0");
    if (val < 0x10000)  Serial.print("0");
    if (val < 0x1000)   Serial.print("0");
    if (val < 0x100)    Serial.print("0");
    if (val < 0x10)     Serial.print("0");
    Serial.print(val, HEX);
    Serial.print(" (");
    Serial.print(val);
    Serial.println(")");
  }
}

void readAllRegisters() {
  Serial.println("=== Read-only Registers ===");
  printRegHex("I_RMS      ", BL0942_REG_I_RMS);
  printRegHex("V_RMS      ", BL0942_REG_V_RMS);
  printRegHex("I_FAST_RMS ", BL0942_REG_I_FAST_RMS);
  printRegHex("WATT       ", BL0942_REG_WATT);
  printRegHex("CF_CNT     ", BL0942_REG_CF_CNT);
  printRegHex("FREQ       ", BL0942_REG_FREQ);
  printRegHex("STATUS     ", BL0942_REG_STATUS);

  Serial.println("=== Config Registers ===");
  printRegHex("I_RMSOS    ", BL0942_REG_I_RMSOS);
  printRegHex("WA_CREEP   ", BL0942_REG_WA_CREEP);
  printRegHex("I_FAST_TH  ", BL0942_REG_I_FAST_TH);
  printRegHex("FREQ_CYC   ", BL0942_REG_FREQ_CYC);
  printRegHex("OT_FUNX    ", BL0942_REG_OT_FUNX);
  printRegHex("MODE       ", BL0942_REG_MODE);
  printRegHex("GAIN_CR    ", BL0942_REG_GAIN_CR);
  Serial.println();
}

void readWaveform() {
  Serial.println("=== Waveform Samples ===");
  for (int i = 0; i < 10; i++) {
    long iWave = sensor.readRegister(BL0942_REG_I_WAVE);
    long vWave = sensor.readRegister(BL0942_REG_V_WAVE);
    Serial.print("I_WAVE="); Serial.print(iWave);
    Serial.print("\tV_WAVE="); Serial.println(vWave);
    delay(5);
  }
  Serial.println();
}

void loop() {
  if (Serial.available()) {
    char cmd = Serial.read();
    switch (cmd) {
      case 'r': readAllRegisters(); break;
      case 'w': readWaveform();     break;
      case 's':
        Serial.print("Status: 0x");
        Serial.println(sensor.getStatus(), HEX);
        break;
      case 'x':
        Serial.println("Performing soft reset...");
        sensor.softReset();
        Serial.println("Done.");
        break;
    }
  }
}
