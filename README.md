# QuadMeUp_Crossbow
Cheap DIY RC link based on LoRa 868MHz modules

# Protocol

| Byte | Description | Notes |
| ---- | ---- | ---- |
| 0    | Preamble | "Q" 0x51 |
| 1   | Channel ID | channel used for comunication between TX and RX |
| 2    | Frame type & Payload Length | bits 7-5 defines frame type, bits 4-0 payload length |
| 3    | Packet ID | |
| 4 - 36 | Payload | 32 bytes max |
| payload length + 4 | CRC | XOR of all previous bytes |

## Frame types

| Value | Description                   | Direction                                       |
| ----  | ----                          | ---- |
| 000   | RC channels data `RC_DATA` | TX -> RX |
| 001   | Receiver health and basic telemetry `RX_HEALTH` | RX -> TX |
| 010   | Request receiver configuration | TX -> RX |
| 011   | Receiver configuration | RX -> TX |
| 100   | Set receiver configuration | TX -> RX |

### `RC_DATA` frame format

Protocol allows to send 10 RC channels in total encoded as following

* channels 1 to 4 encoded using 10 bits each (5 bytes)
* channels 5 to 6 encoded using 8 bits each (2 bytes)
* channels 7 to 10 encoded using 4 bits per channel (2 bytes)

Total length of `RC_DATA` payload is 9 bytes

### `RX_HEALTH` frame format

| Byte | Description                        |
| ---- | ----                               |
| 0    | RX RSSI                            |
| 1    | RX supply volatage, sent in 0,1V   |
| 2    | RX analog input 1 sent in 0,1V     |
| 3    | RX analog input 2 sent in 0,1V     |
