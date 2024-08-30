#ifndef ModbusRTUComm_h
#define ModbusRTUComm_h

#include "Arduino.h"
#include "ModbusADU.h"

enum ModbusRTUCommError uint8_t {
  MODBUS_RTU_COMM_SUCCESS = 0,
  MODBUS_RTU_COMM_TIMEOUT = 1,
  MODBUS_RTU_COMM_FRAME_ERROR = 2,
  MODBUS_RTU_COMM_CRC_ERROR = 3
};

class ModbusRTUComm {
  public:
    ModbusRTUComm(Stream& serial, int8_t dePin = -1, int8_t rePin = -1, unsigned long timeout = 0);
    template <typename ConfigType>
    void begin(unsigned long baud, ConfigType config = SERIAL_8N1, unsigned long preDelay = 0, unsigned long postDelay = 0);
    void setTimeout(unsigned long readTimeout);
    ModbusRTUCommError readAdu(ModbusADU& adu);
    void writeAdu(ModbusADU& adu);
    void clearRxBuffer();

  private:
    Stream& _serial;
    int8_t _dePin;
    int8_t _rePin;
    unsigned long _preDelay;
    unsigned long _postDelay;
    unsigned long _charTimeout;
    unsigned long _frameTimeout;
    unsigned long _readTimeout = 0;
};

#endif