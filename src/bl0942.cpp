#include "bl0942.h"

bl0942::bl0942()
{
}

bl0942::bl0942(uint8_t cs_io)
{
    _cs_io = cs_io;
}

void bl0942::begin(HardwareSerial *hwSerial, uint8_t deviceAddr)
{
    _mode = BL0942_COMM_UART;
    _serial = hwSerial;
    _deviceAddr = deviceAddr & 0x03;
    _cmdRead = 0x58 | _deviceAddr;
    _cmdWrite = 0xA8 | _deviceAddr;
    _serial->begin(BL0942_DEFAULT_BAUD_RATE, BL0942_DEFAULT_PORT_CONFIG);
}

void bl0942::begin()
{
    _mode = BL0942_COMM_SPI;
    if (_cs_io != 0xFF)
    {
        pinMode(_cs_io, OUTPUT);
        digitalWrite(_cs_io, HIGH);
        SPI.begin();
        SPI.beginTransaction(SPISettings(800000, MSBFIRST, SPI_MODE1));
    }
}

// Unified register read - routes to SPI or UART
long bl0942::readRegister(uint8_t regAddress)
{
    if (_mode == BL0942_COMM_SPI)
        return readRegister_spi(regAddress);
    else
        return readRegister_uart(regAddress);
}

// Unified register write with auto write-protect unlock
bool bl0942::writeRegister(uint8_t regAddress, uint32_t regValue)
{
    if (!unlockWrite()) return false;
    if (_mode == BL0942_COMM_SPI)
        return writeRegister_spi(regAddress, regValue);
    else
        return writeRegister_uart(regAddress, regValue);
}

bool bl0942::unlockWrite()
{
    if (_mode == BL0942_COMM_SPI)
        return writeRegister_spi(BL0942_REG_USR_WRPROT, 0x55);
    else
        return writeRegister_uart(BL0942_REG_USR_WRPROT, 0x55);
}

void bl0942::softReset()
{
    unlockWrite();
    if (_mode == BL0942_COMM_SPI)
        writeRegister_spi(BL0942_REG_SOFT_RESET, 0x5A5A5A);
    else
        writeRegister_uart(BL0942_REG_SOFT_RESET, 0x5A5A5A);
}

// -------- Measurement Functions --------

float bl0942::getVoltage()
{
    long raw = readRegister(BL0942_REG_V_RMS);
    if (raw < 0) return -1.0f;
    V_RMS_RAW = (uint32_t)raw;
    float Data = (V_RMS_RAW * BL0942_VREF * BL0942_V_R1) / (BL0942_V_RMS_LBS * BL0942_V_R2 * 1000.0);
    return Data;
}

float bl0942::getCurrent()
{
    long raw = readRegister(BL0942_REG_I_RMS);
    if (raw < 0) return -1.0f;
    uint32_t get_reg_data = (uint32_t)raw;

    if (get_reg_data != 0)
    {
        I_RMS_RAW = get_reg_data;
    }
    else
    {
        if (V_RMS_RAW < 100)
        {
            I_RMS_RAW = 0;
        }
        get_reg_data = I_RMS_RAW;
    }

    float Data = (get_reg_data * BL0942_VREF) / (BL0942_I_RMS_LBS * (BL0942_I_R1 * 1000.0) / BL0942_I_Rt);
    return Data;
}

float bl0942::getActivePower()
{
    long raw = readRegister(BL0942_REG_WATT);
    if (raw < 0) return 0.0f;
    int32_t get_reg_data = (int32_t)raw;

    if (get_reg_data != 0)
    {
        W_RMS_RAW = get_reg_data;
    }
    else
    {
        if (V_RMS_RAW < 100)
        {
            W_RMS_RAW = 0;
        }
        get_reg_data = W_RMS_RAW;
    }

    float Data = (get_reg_data * BL0942_VREF * BL0942_VREF * BL0942_V_R1) / (3537.0 * (BL0942_I_R1 * 1000.0 / BL0942_I_Rt) * BL0942_V_R2 * 1000.0);
    return Data;
}

float bl0942::getEnergy()
{
    long raw = readRegister(BL0942_REG_CF_CNT);
    if (raw < 0) return -1.0f;
    uint32_t get_reg_data = (uint32_t)raw;

    if (get_reg_data != 0)
    {
        W_CF_CNT = get_reg_data;
    }
    else
    {
        get_reg_data = W_CF_CNT;
    }

    float Data = get_reg_data * BL0942_CF_CNT;
    return Data;
}

uint32_t bl0942::getEnergyCF()
{
    long raw = readRegister(BL0942_REG_CF_CNT);
    if (raw < 0) return W_CF_CNT;
    uint32_t get_reg_data = (uint32_t)raw;

    if (get_reg_data != 0)
    {
        W_CF_CNT = get_reg_data;
    }
    else
    {
        get_reg_data = W_CF_CNT;
    }

    return get_reg_data;
}

float bl0942::getEnergy(uint32_t cf)
{
    float Data = cf * BL0942_CF_CNT;
    return Data;
}

float bl0942::getFrequency()
{
    long raw = readRegister(BL0942_REG_FREQ);
    if (raw <= 0) return -1.0f;
    float Data = 1000000.0 / raw;  // f = 1000000 / FREQ (datasheet §2.8)
    return Data;
}

uint8_t bl0942::getWaCreep()
{
    long raw = readRegister(BL0942_REG_WA_CREEP);
    if (raw < 0) return 0;
    return (uint8_t)raw;
}

uint16_t bl0942::getStatus()
{
    long raw = readRegister(BL0942_REG_STATUS);
    if (raw < 0) return 0;
    return (uint16_t)raw;
}



long bl0942::readRegister_spi(uint8_t regAddress)
{
    uint8_t Start = 0x58; // SPI read command (address always 0 for SPI)
    uint8_t Payload[4];

    digitalWrite(_cs_io, LOW);
    SPI.transfer(Start);
    SPI.transfer(regAddress);
    Payload[0] = SPI.transfer(0x00);  // Data_H (MSB first per datasheet §3.1.2)
    Payload[1] = SPI.transfer(0x00);  // Data_M
    Payload[2] = SPI.transfer(0x00);  // Data_L
    Payload[3] = SPI.transfer(0x00);  // Checksum
    digitalWrite(_cs_io, HIGH);

    // Verify checksum: ~(Start + Addr + Data_H + Data_M + Data_L)
    uint8_t Payload_OUT[5];
    Payload_OUT[0] = Start;
    Payload_OUT[1] = regAddress;
    Payload_OUT[2] = Payload[0];
    Payload_OUT[3] = Payload[1];
    Payload_OUT[4] = Payload[2];

    uint8_t crc_data = crc(Payload_OUT, 5);
    if (crc_data != Payload[3])
    {
        return -1;
    }

    // SPI is MSB first: Payload[0]=High, Payload[1]=Mid, Payload[2]=Low
    int32_t Data = (int32_t)((uint32_t)Payload[0] << 16 |
                              (uint32_t)Payload[1] << 8  |
                              (uint32_t)Payload[2]);
    Data = Data << 8;
    Data = Data >> 8;  // Sign extension for 24-bit signed values
    return Data;
}


long bl0942::readRegister_uart(uint8_t regAddress)
{
    if (_serial == nullptr) return -1;

    lastRcv = millis();
    const uint16_t Bytes = 4;  // Data_L + Data_M + Data_H + Checksum
    uint16_t Index = 0;

    uint8_t packet[] = {_cmdRead, regAddress};
    _serial->write(packet, sizeof(packet));

    uint8_t Payload[4];

    while ((Index < Bytes) && !((millis() - lastRcv) > BL0942_TIMEOUT))
    {
        uint8_t bytesAvailable = _serial->available();
        if (bytesAvailable > 0)
        {
            for (uint16_t bytesIndex = 0; bytesIndex < bytesAvailable && Index < Bytes; bytesIndex++)
            {
                Payload[Index++] = (uint8_t)_serial->read();
            }
            lastRcv = millis();
        }
    }

    // Drain any extra bytes in buffer
    while (_serial->available()) _serial->read();

    if (Index != Bytes)
    {
        return -1;
    }

    if (crc(regAddress, Payload, sizeof(Payload), Payload[3]) != true)
    {
        return -1;
    }

    // UART is LSB first: Payload[0]=Low, Payload[1]=Mid, Payload[2]=High
    int32_t Data = (int32_t)((uint32_t)Payload[2] << 16 |
                              (uint32_t)Payload[1] << 8  |
                              (uint32_t)Payload[0]);
    Data = Data << 8;
    Data = Data >> 8;  // Sign extension for 24-bit signed values
    return Data;
}


bool bl0942::writeRegister_spi(uint8_t regAddress, uint32_t regValue)
{
    uint8_t Start = 0xA8; // SPI write command

    uint8_t Payload[5];
    Payload[0] = Start;
    Payload[1] = regAddress;
    Payload[2] = regValue >> 16;  // Data_H
    Payload[3] = regValue >> 8;   // Data_M
    Payload[4] = regValue;        // Data_L

    uint8_t crc_data = crc(Payload, sizeof(Payload));

    digitalWrite(_cs_io, LOW);
    SPI.transfer(Payload[0]);
    SPI.transfer(Payload[1]);
    SPI.transfer(Payload[2]);
    SPI.transfer(Payload[3]);
    SPI.transfer(Payload[4]);
    SPI.transfer(crc_data);
    digitalWrite(_cs_io, HIGH);

    return true;
}


bool bl0942::writeRegister_uart(uint8_t regAddress, uint32_t regValue)
{
    if (_serial == nullptr) return false;

    uint8_t packet[6];
    packet[0] = _cmdWrite;
    packet[1] = regAddress;
    packet[2] = regValue & 0xFF;         // Data_L (UART low byte first)
    packet[3] = (regValue >> 8) & 0xFF;  // Data_M
    packet[4] = (regValue >> 16) & 0xFF; // Data_H

    // Checksum: ~(cmd + addr + data_l + data_m + data_h)
    uint8_t checkData = 0;
    for (uint8_t i = 0; i < 5; i++)
    {
        checkData += packet[i];
    }
    packet[5] = ~checkData;

    _serial->write(packet, sizeof(packet));
    return true;
}


// -------- Packet Batch Read (UART only, datasheet §3.2.6) --------

bool bl0942::readAll(BL0942_Data &data)
{
    if (_mode != BL0942_COMM_UART || _serial == nullptr) return false;

    uint8_t packet[] = {_cmdRead, 0xAA};
    _serial->write(packet, sizeof(packet));

    // Expect 23 bytes: HEAD(1) + I_RMS(3) + V_RMS(3) + I_FAST_RMS(3) +
    //                  WATT(3) + CF_CNT(3) + FREQ(3) + STATUS(3) + CHECKSUM(1)
    const uint16_t totalBytes = 23;
    uint8_t buf[23];
    uint16_t idx = 0;
    uint32_t startTime = millis();

    while ((idx < totalBytes) && !((millis() - startTime) > BL0942_TIMEOUT * 3))
    {
        uint8_t avail = _serial->available();
        if (avail > 0)
        {
            for (uint16_t i = 0; i < avail && idx < totalBytes; i++)
            {
                buf[idx++] = (uint8_t)_serial->read();
            }
            startTime = millis();
        }
    }

    while (_serial->available()) _serial->read();

    if (idx != totalBytes) return false;
    if (buf[0] != 0x55) return false;

    // Verify checksum
    uint8_t checkData = 0;
    for (uint16_t i = 0; i < totalBytes - 1; i++)
    {
        checkData += buf[i];
    }
    checkData += _cmdRead;
    checkData = ~checkData;
    if (checkData != buf[totalBytes - 1]) return false;

    // Parse data (UART: low byte first)
    data.i_rms      = (uint32_t)buf[3]  << 16 | (uint32_t)buf[2]  << 8 | buf[1];
    data.v_rms      = (uint32_t)buf[6]  << 16 | (uint32_t)buf[5]  << 8 | buf[4];
    data.i_fast_rms = (uint32_t)buf[9]  << 16 | (uint32_t)buf[8]  << 8 | buf[7];

    // WATT is signed 24-bit
    int32_t watt_raw = (int32_t)((uint32_t)buf[12] << 16 | (uint32_t)buf[11] << 8 | buf[10]);
    watt_raw = watt_raw << 8;
    watt_raw = watt_raw >> 8;
    data.watt = watt_raw;

    data.cf_cnt = (uint32_t)buf[15] << 16 | (uint32_t)buf[14] << 8 | buf[13];
    data.freq   = (uint16_t)buf[17] << 8 | buf[16];
    data.status = (uint16_t)buf[20] << 8 | buf[19];

    // Update cached values
    if (data.v_rms != 0)  V_RMS_RAW = data.v_rms;
    if (data.i_rms != 0)  I_RMS_RAW = data.i_rms;
    if (data.watt != 0)   W_RMS_RAW = data.watt;
    if (data.cf_cnt != 0) W_CF_CNT = data.cf_cnt;

    return true;
}


// -------- CRC Functions --------

uint8_t bl0942::crc(uint8_t *ReqData, uint8_t dataLen)
{
    uint8_t checkData = 0;
    for (uint8_t a = 0; a < dataLen; a++)
    {
        checkData += ReqData[a];
    }
    checkData = ~checkData;
    return checkData;
}


bool bl0942::crc(uint8_t Addr, uint8_t *ReqData, uint8_t dataLen, uint8_t _crc)
{
    uint8_t checkData = 0;
    for (uint8_t a = 0; a < dataLen - 1; a++)
    {
        checkData += ReqData[a];
    }
    checkData = checkData + _cmdRead + Addr;
    checkData = ~checkData;
    return (_crc == checkData);
}