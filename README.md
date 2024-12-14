# ModbusRTUComm
This library provides some core functions for implementing Modbus RTU communication.
It is not a full implementation of Modbus RTU. Other libraries are available for that purpose: see [ModbusRTUSlave](https://github.com/CMB27/ModbusRTUSlave) and [ModbusRTUMaster](https://github.com/CMB27/ModbusRTUMaster).



## Compatibility
This library has been tested with the following boards and cores:

| Board Name                  | Core                                                                 | Works   |
| :-------------------------- | :------------------------------------------------------------------- | :-----: |
| Arduino Due                 | **Arduino SAM Boards (32-bits ARM Cortex-M3)** by Arduino `1.6.12`   | Yes     |
| Arduino Giga                | **Arduino Mbed OS GIGA Boards** by Arduino `4.1.5`                   | Yes     |
| Arduino Leonardo            | **Arduino AVR Boards** by Arduino `1.8.6`                            | Yes     |
| Arduino Make Your UNO       | **Arduino AVR Boards** by Arduino `1.8.6`                            | Yes     |
| Arduino Mega 2560           | **Arduino AVR Boards** by Arduino `1.8.6`                            | Yes     |
| Arduino Nano                | **Arduino AVR Boards** by Arduino `1.8.6`                            | Yes     |
| Arduino Nano 33 BLE         | **Arduino Mbed OS Nano Boards** by Arduino `4.1.5`                   | Yes     |
| Arduino Nano 33 IoT         | **Arduino SAMD Boards (32-bits ARM Cortex-M0+)** by Arduino `1.8.14` | Yes     |
| Arduino Nano ESP32          | **Arduino ESP32 Boards** by Arduino `2.0.13`                         | Yes     |
| Arduino Nano ESP32          | **esp32** by Espressif Systems `3.0.4`                               | Yes     |
| Arduino Nano Every          | **Arduino megaAVR Boards** by Arduino `1.8.8`                        | Yes     |
| Arduino Nano Matter         | **Silicon Labs** by Silicon Labs `2.1.0`                             | No [^1] |
| Arduino Nano RP2040 Connect | **Arduino Mbed OS Nano Boards** by Arduino `4.1.5`                   | No [^2] |
| Arduino Nano RP2040 Connect | **Raspberry Pi Pico/RP2040** by Earle F. Philhower, III `4.0.1`      | Yes     |
| Arduino UNO R3 SMD          | **Arduino AVR Boards** by Arduino `1.8.6`                            | Yes     |
| Arduino UNO R4 Minima       | **Arduino UNO R4 Boards** by Arduino `1.2.0`                         | Yes     |

[^1]: **Arduino Nano RP2040 Connect**  
This board has trouble receiving Modbus messages when using the **Arduino Mbed OS Nano Boards** core by Arduino.  
There seems to be some sort of timing issue.  
It can technically be made to work if you tell the library that it is operating at a lower baud rate than the serial port assigned to the library is actually operating at.
However, this would cause the library to operate with unknown timing tolerances, possibly well outside the Modbus specification.

[^2]: **Arduino Nano Matter**  
As of this writing (2024-09-07), `flush()` is not properly implemented with `Serial1` on this board.  
This library depends on `flush()` to know when to set the DE pin LOW after a message is sent.



## Methods



<details><summary id="modbusrtucomm-1"><strong>ModbusRTUComm()</strong></summary>
  <blockquote>

### Description
Creates a `ModbusRTUComm` object and sets the serial port to use for data transmission.
Optionally sets a driver enable pin. This pin will go `HIGH` when the library is transmitting. This is primarily intended for use with an RS-485 transceiver, but it can also be a handy diagnostic when connected to an LED.
A read enable pin can also be specified. This pin will be set `LOW`, always enabling reading.

### Syntax
- `ModbusRTUComm(serial)`
- `ModbusRTUComm(serial, dePin)`
- `ModbusRTUComm(serial, dePin, rePin)`

### Parameters
- `serial`: the `Stream` object to use for Modbus communication. Usually something like `Serial1`.
- `dePin`: the driver enable pin. This pin is set `HIGH` when transmitting. If this parameter is set to `-1`, this feature will be disabled. The default value is `-1`. Allowed data types are `int8_t` or `char`.
- `rePin`: the read enable pin. This pin is always set `LOW`. If this parameter is set to `-1`, this feature will be disabled. The default value is `-1`. Allowed data types are `int8_t` or `char`.

### Example
```C++
#include <ModbusRTUComm.h>

const int8_t dePin = A6;
const int8_t rePin = A5;

ModbusRTUComm rtuComm(Serial1, dePin, rePin);
```

  </blockquote>
</details>



<details><summary id="begin"><strong>begin()</strong></summary>
  <blockquote>
  
  ### Description
  Sets the data rate in bits per second (baud) for serial transmission.
  Optionally it also sets the data configuration. Note, there must be 8 data bits for Modbus RTU communication. The default configuration is 8 data bits, no parity, and one stop bit.

  ### Syntax
  ```C++
  rtuComm.begin(baud);
  rtuComm.begin(baud, config);
  ```

  ### Parameters
  - `rtuComm`: a `ModbusRTUComm` object.
  - `baud`: the baud rate to use for Modbus communication. Common values are: `1200`, `2400`, `4800`, `9600`, `19200`, `38400`, `57600`, and `115200`. Allowed data types: `unsigned long`.
  - `config`: the serial port configuration to use. Valid values are:  
  `SERIAL_8N1`: no parity (default)  
  `SERIAL_8N2`  
  `SERIAL_8E1`: even parity  
  `SERIAL_8E2`  
  `SERIAL_8O1`: odd parity  
  `SERIAL_8O2`

  *`begin()` for the serial port used with the modbus object must be run seperately.*

  ### Example
  ```C++
  void setup() {
    Serial1.begin(38400, SERIAL_8E1);
    rtuComm.begin(38400, SERIAL_8E1)
  }
  ```
  
  </blockquote>
</details>



<details><summary id="settimeout"><strong>setTimeout()</strong></summary>
  <blockquote>

### Description
Sets the maximum time in milliseconds that `readAdu()` will wait for a data to be received. The default value is 0.

### Syntax
- `rtuComm.setTimeout(timeout)`

### Parameters
- `rtuComm`: a `ModbusRTUComm` object.
- `timeout`: the timeout duration in milliseconds. Allowed data types: `unsigned long`.

  </blockquote>
</details>



<details><summary id="readadu"><strong>readAdu()</strong></summary>
  <blockquote>

### Description
Reads serial data to a `ModbusADU` object.

### Syntax
- `rtuComm.readAdu(adu)`

### Parameters
- `rtuComm`: a `ModbusRTUComm` object.
- `adu`: a `ModbusADU` object.

### Returns
Error code. Data type: `ModbusRTUCommError` or `uint8_t`.
- `0`: Success
- `1`: Timeout
- `2`: Frame error
- `3`: CRC error

### Example
```C++
ModbusADU adu;
uint8_t error = rtuComm.readAdu(adu);
```

  </blockquote>
</details>



<details><summary id="writeadu"><strong>writeAdu()</strong></summary>
  <blockquote>

### Description
Writes serial data from a `ModbusADU` object.

### Syntax
- `rtuComm.writeAdu(adu)`

### Parameters
- `rtuComm`: a `ModbusRTUComm` object.
- `adu`: a `ModbusADU` object.

### Returns
`1` or `true` when the transmitted message is verified as having been sent, `0` or `false` otherwise. Data type: `bool`.

*This will only return `true` if some sort of loopback mechanism is in place where all the transmitted data is also received.
This can easily be done with an RS-485 transceiver by connecting the RE pin to GND.*

*It is not essential that this value be read.*

### Example
```C++
ModbusADU adu;
adu.setUnitId(1);
adu.setFunctionCode(1);
adu.setDataRegister(0, 0);
adu.setDataRegister(2, 2);
adu.setDataLen(4);
bool success = rtuComm.writeAdu(adu);
```

  </blockquote>
</details>
