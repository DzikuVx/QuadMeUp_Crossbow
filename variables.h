#pragma once

//Only for UART connected radio modules
#define UART_SPEED 57600
#define E45_TTL_100_UART_DOWNTIME 30 

#define PPM_CHANNEL_COUNT 10

#define TX_RC_FRAME_RATE 5000 //ms

#define CHANNEL_ID 0x01
#define QSP_PREAMBLE 0x51
#define QSP_PAYLOAD_LENGTH 32

#define QSP_FRAME_RC_DATA 0x0
#define QSP_FRAME_RX_HEALTH 0x1

#define PIN_LED 13

enum dataStates {
    IDLE,
    PREAMBLE_RECEIVED,
    CHANNEL_RECEIVED,
    FRAME_TYPE_RECEIVED,
    PACKET_ID_RECEIVED,
    PAYLOAD_RECEIVED,
    CRC_RECEIVED
};

#define PPM_INPUT_PIN       2
#define PPM_INPUT_INTERRUPT 1 //For Pro Micro 1, For Pro Mini 0

#define PPM_CHANNEL_DEFAULT_VALUE 1500  //set the default servo value
#define PPM_FRAME_LENGTH 22500  //set the PPM frame length in microseconds (1ms = 1000µs)
#define PPM_PULSE_LENGTH 300  //set the pulse length
#define PPM_SIGNAL_POSITIVE_STATE 1  //set polarity of the pulses: 1 is positive, 0 is negative
#define PPM_OUTPUT_PIN 10  //set PPM signal output pin on the arduino

struct QspConfiguration_t {
    uint8_t protocolState = IDLE;
    uint8_t crc = 0;
    uint8_t payload[QSP_PAYLOAD_LENGTH] = {0};
    uint8_t payloadLength = 0;
    uint8_t frameToSend = 0;
    void (* hardwareWriteFunction)(uint8_t, QspConfiguration_t*);
};