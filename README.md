# QuadMeUp Crossbow

_QuadMeUp Crossbow_ is a DIY project that gives 5km (at least) of RC link for UAV (airplanes and drones) for a price below $40. I uses SX1278 (LoRa 868MHz/915MHz) compatible (like HopeRF RFM95W) radio modules connected to Arduino compatible boards. It can be regular Arduino connected via SPI to SX1278 or dedicated board like _Adafruit Feather 32u4 RFM LoRa_ or _LoRa32u4 II_

# Current state

Development, ready for testing

# Protocol

| Byte                  | Description | Notes |
| ----                  | ----        | ---- |
| 1                     | Channel ID | channel used for comunication between TX and RX |
| 2                     | Frame type & Length | bits 7-5 defines frame, bits 4-0 current radio channel |
| 3 - 34                | Payload | lenghth defined by frame type |
| payload length + 3    | CRC | using crc8_dvb_s2 method |

## Frame types

| Value  | Value hex    | Description                      | Direction  | Payload length |
| ----   | ----         |----                              | ---- | ----            |
| 0000   | 0x0          | RC channels data `RC_DATA` | TX -> RX | 9 |
| 0001   | 0x1          | Receiver health and basic telemetry `RX_HEALTH` | RX -> TX | 6 |
| 0010   | 0x2          | Request receiver configuration | TX -> RX | no used |
| 0011   | 0x3          | Receiver configuration | RX -> TX | no used |
| 0100   | 0x4          | Set receiver configuration | TX -> RX | no used |
| 0101   | 0x5          | PING frame, uses 9 byte payload | TX -> RX | 4    |
| 0110   | 0x6          | PONG frame, the same payload as PING | RX -> TX | 4 |

### `RC_DATA` frame format

Protocol allows to send 10 RC channels in total encoded as following

* channels 1 to 4 encoded using 10 bits each (5 bytes)
* channels 5 to 6 encoded using 8 bits each (2 bytes)
* channels 7 to 10 encoded using 4 bits per channel (2 bytes)

Total length of `RC_DATA` payload is 9 bytes

### `RX_HEALTH` frame format

| Byte  | Description                           |
| ----  | ----                                  |
| 1     | RX RSSI                               |
| 2     | RX SNR                                |   
| 3     | RX supply volatage, sent in 0,1V      |
| 4     | RX analog input 1 sent in 0,1V        |
| 5     | RX analog input 2 sent in 0,1V        |
| 6     | Flags                                 |

#### Flags

| Bit   | Meaning                               |
| ----  | ----                                  |
| 00000001  | Device in Failsafe mode           |


### `PING` and `PONG` frames

`PING` and `PONG` frames are to determine packet roundrip between **TX** and **RX** module.
**TX** sends `PING` frame with curent `micros`. If **RX** receives `PING` frame, it respons
its payload as `PONG` frame. 

# TX module connection diagram

![Diagram](docs/TX_module_schem.png)

# RX module connection diagram

![Diagram](docs/RX_module_schem.png)