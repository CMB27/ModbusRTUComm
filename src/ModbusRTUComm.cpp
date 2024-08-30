#include "ModbusRTUComm.h"

ModbusRTUComm::ModbusRTUComm(Stream& serial, int8_t dePin, int8_t rePin, unsigned long timeout) : _serial(serial) {
  _dePin = dePin;
  _rePin = rePin;
  _readTimeout = timeout;
}

void ModbusRTUComm::begin(unsigned long baud, ConfigType config, unsigned long preDelay, unsigned long postDelay) {
  _preDelay = preDelay;
  _postDelay = postDelay;
  unsigned long bitsPerChar;
  switch (config) {
    case SERIAL_8E2:
    case SERIAL_8O2:
      bitsPerChar = 12;
      break;
    case SERIAL_8N2:
    case SERIAL_8E1:
    case SERIAL_8O1:
      bitsPerChar = 11;
      break;
    case SERIAL_8N1:
    default:
      bitsPerChar = 10;
      break;
  }
  if (baud <= 19200) {
    _charTimeout = (bitsPerChar * 2500000) / baud;
    _frameTimeout = (bitsPerChar * 4500000) / baud;
  }
  else {
    _charTimeout = (bitsPerChar * 1000000) / baud + 750;
    _frameTimeout = (bitsPerChar * 1000000) / baud + 1750;
  }
  if (_dePin >= 0) {
    pinMode(_dePin, OUTPUT);
    digitalWrite(_dePin, LOW);
  }
  if (_rePin >= 0) {
    pinMode(_rePin, OUTPUT);
    digitalWrite(_rePin, LOW);
  }
  clearRxBuffer();
}

void ModbusRTUComm::setTimeout(unsigned long readTimeout) {
  _readTimeout = readTimeout;
}

ModbusRTUCommError ModbusRTUComm::readAdu(ModbusADU& adu) {
  adu.setRtuLen(0);
  unsigned long startMillis = millis();
  while (!_serial.available()) {
    if (millis() - startMillis >= _readTimeout) return MODBUS_RTU_COMM_TIMEOUT;
  }
  unsigned long startMicros = micros();
  do {
    if (_serial.available()) {
      startMicros = micros();
      adu.rtu[adu.getRtuLen()] = _serial.read();
      adu.setRtuLen(adu.getRtuLen() + 1);
    }
  } while (micros() - startMicros <= _charTimeout && adu.getRtuLen() < 256);
  
  while (micros() - startMicros < _frameTimeout);
  if (_serial.available()) {
    adu.setRtuLen(0);
    return MODBUS_RTU_COMM_FRAME_ERROR;
  }
  if (!adu.crcGood()) {
    adu.setRtuLen(0);
    return MODBUS_RTU_COMM_CRC_ERROR;
  }
  return MODBUS_RTU_COMM_SUCCESS;
}

void ModbusRTUComm::writeAdu(ModbusADU& adu) {
  adu.updateCrc();
  if (_dePin >= 0) digitalWrite(_dePin, HIGH);
  if (_rePin >= 0) digitalWrite(_rePin, HIGH);
  delayMicroseconds(_preDelay);
  _serial.write(adu.rtu, adu.getRtuLen());
  _serial.flush();
  delayMicroseconds(_postDelay);
  if (_dePin >= 0) digitalWrite(_dePin, LOW);
  if (_rePin >= 0) digitalWrite(_rePin, LOW);
}

void ModbusRTUComm::clearRxBuffer() {
  unsigned long startMicros = micros();
  do {
    if (_serial.available() > 0) {
      startMicros = micros();
      _serial.read();
    }
  } while (micros() - startMicros < _frameTimeout);
}
