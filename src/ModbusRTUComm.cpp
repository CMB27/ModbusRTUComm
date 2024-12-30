#include "ModbusRTUComm.h"

ModbusRTUComm::ModbusRTUComm(Stream& serial, int8_t dePin, int8_t rePin) : _serial(serial) {
  _dePin = dePin;
  _rePin = rePin;
}

void ModbusRTUComm::begin(unsigned long baud, uint32_t config) {
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
  _bytePeriod = (bitsPerChar * 1000000) / baud;
  _bitPeriod = 1000000 / baud;
  if (_dePin >= 0) {
    pinMode(_dePin, OUTPUT);
    digitalWrite(_dePin, LOW);
  }
  if (_rePin >= 0) {
    pinMode(_rePin, OUTPUT);
    digitalWrite(_rePin, LOW);
  }
  unsigned long startMicros = micros();
  do {
    if (_serial.available() > 0) {
      startMicros = micros();
      _serial.read();
    }
  } while (micros() - startMicros < _frameTimeout);
}

void ModbusRTUComm::setTimeout(unsigned long timeout) {
  _readTimeout = timeout;
}

ModbusRTUCommError ModbusRTUComm::readAdu(ModbusADU& adu) {
  adu.setRtuLen(0);
  unsigned long startMillis = millis();
  while (!_serial.available()) {
    if (millis() - startMillis >= _readTimeout) return MODBUS_RTU_COMM_TIMEOUT;
  }
  uint16_t len = 0;
  unsigned long startMicros = micros();
  do {
    if (_serial.available()) {
      startMicros = micros();
      adu.rtu[len] = _serial.read();
      len++;
    }
  } while (micros() - startMicros <= _charTimeout && len < 256);
  adu.setRtuLen(len);
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

/*
bool ModbusRTUComm::writeAdu(ModbusADU& adu) {
  uint16_t i = 0;
  uint16_t j = 0;
  bool verified = false;
  adu.updateCrc();
  uint16_t len = adu.getRtuLen();
  if (_dePin >= 0) digitalWrite(_dePin, HIGH);
  unsigned long startMicros = micros();
  while (true) {
    unsigned long microsNow = micros();
    if (i < len){
      _serial.write(adu.rtu[i]);
      delayMicroseconds(_bytePeriod);
      i++;
      if (i == len && _dePin >= 0) digitalWrite(_dePin, LOW);
    }
    if (_serial.available()) {
      startMicros = microsNow;
      uint8_t value = _serial.read();
      if (j == 0) verified = true;
      if (j < len && value != adu.rtu[j]) verified = false;
      j++;
    }
    if (i == len && (microsNow - startMicros) > _charTimeout) {
      if (j != len) verified = false;
      break;
    }
  }
  return verified;
}
*/

/*
bool ModbusRTUComm::writeAdu(ModbusADU& adu) {
  uint16_t i = 0;
  uint16_t j = 0;
  bool transmitting = true;
  bool verified = false;
  unsigned long microsNow;
  unsigned long txStartMicros;
  unsigned long rxStartMicros;
  adu.updateCrc();
  uint16_t len = adu.getRtuLen();
  if (_dePin >= 0) digitalWrite(_dePin, HIGH);
  while (true) {
    microsNow = micros();
    if (i == 0) {
      txStartMicros = microsNow + _bytePeriod;
      rxStartMicros = microsNow;
    }
    if (transmitting && microsNow - txStartMicros >= _bytePeriod) {
      txStartMicros = microsNow;
      if (i < len) {
        _serial.write(adu.rtu[i]);
        i++;
      }
      else {
        if (_dePin >= 0) digitalWrite(_dePin, LOW);
        transmitting = false;
      }
    }
    if (_serial.available()) {
      rxStartMicros = microsNow;
      uint8_t value = _serial.read();
      if (j == 0) verified = true;
      if (j < len && value != adu.rtu[j]) verified = false;
      j++;
    }
    if (i == len && (microsNow - rxStartMicros) > _charTimeout) {
      if (j != len) verified = false;
      break;
    }
  }
  return verified;
}
*/

/*
bool ModbusRTUComm::writeAdu(ModbusADU& adu) {
  uint16_t i = 0;
  uint16_t j = 0;
  bool verified = false;
  unsigned long microsNow;
  unsigned long txStartMicros;
  unsigned long rxStartMicros;
  adu.updateCrc();
  uint16_t len = adu.getRtuLen();
  if (_dePin >= 0) digitalWrite(_dePin, HIGH);
  while (true) {
    microsNow = micros();
    if (i == 0) {
      txStartMicros = microsNow + _bytePeriod;
      rxStartMicros = microsNow;
    }
    if (i < len && microsNow - txStartMicros >= _bytePeriod) {
      txStartMicros = microsNow;
      _serial.write(adu.rtu[i]);
      i++;
      if (i == len && _dePin >= 0) {
        delayMicroseconds(_bytePeriod);
        digitalWrite(_dePin, LOW);
      }
    }
    if (_serial.available()) {
      rxStartMicros = microsNow;
      uint8_t value = _serial.read();
      if (j == 0) verified = true;
      if (j < len && value != adu.rtu[j]) verified = false;
      j++;
    }
    if (i == len && (microsNow - rxStartMicros) > _charTimeout) {
      if (j != len) verified = false;
      break;
    }
  }
  return verified;
}
*/

bool ModbusRTUComm::writeAdu(ModbusADU& adu) {
  uint16_t i = 0;
  uint16_t j = 0;
  bool transmitting = true;
  bool verified = false;
  adu.updateCrc();
  uint16_t len = adu.getRtuLen();
  if (_dePin >= 0) digitalWrite(_dePin, HIGH);
  unsigned long microsNow= micros();
  unsigned long txStartMicros = microsNow;
  unsigned long rxStartMicros = microsNow;
  while (true) {
    microsNow = micros();
    if (transmitting) {
      if (i == 0 || (i < len && microsNow - txStartMicros >= _bytePeriod)) {
        txStartMicros = microsNow;
        _serial.write(adu.rtu[i]);
        i++;
      }
      if (i == len && microsNow - txStartMicros >= _bytePeriod + _bitPeriod) {
        if (_dePin >= 0) digitalWrite(_dePin, LOW);
        transmitting = false;
      }
    }
    if (_serial.available()) {
      rxStartMicros = microsNow;
      uint8_t value = _serial.read();
      if (j == 0) verified = true;
      if (j < len && value != adu.rtu[j]) verified = false;
      j++;
    }
    if (!transmitting && (microsNow - rxStartMicros) > _charTimeout) {
      if (j != len) verified = false;
      break;
    }
  }
  return verified;
}