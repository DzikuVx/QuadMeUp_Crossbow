#pragma once

#define SBUS_UPDATE_RATE 12 //ms
#define SBUS_PACKET_LENGTH 25

#define RC_CHANNEL_MIN 990
#define RC_CHANNEL_MAX 2010

//Only for UART connected radio modules
#define UART_SPEED 57600
#define E45_TTL_100_UART_DOWNTIME 30 

#define RX_RX_HEALTH_FRAME_RATE 1000
#define TX_RC_FRAME_RATE 500 //ms
#define RX_FAILSAFE_DELAY (TX_RC_FRAME_RATE * 8)

#define TX_PING_RATE 2000 

#define CHANNEL_ID 0x01
#define QSP_PREAMBLE 0x51
#define QSP_PAYLOAD_LENGTH 32

#define QSP_MAX_FRAME_DECODE_TIME 50 //max time that frame can be decoded in ms

#define QSP_FRAME_RC_DATA 0x0
#define QSP_FRAME_RX_HEALTH 0x1
#define QSP_FRAME_GET_RX_CONFIG 0x2
#define QSP_FRAME_RX_CONFIG 0x3
#define QSP_FRAME_SET_RX_CONFIG 0x4
#define QSP_FRAME_PING 0x5
#define QSP_FRAME_PONG 0x6
#define QSP_FRAME_COUNT 0x7

#define RX_ADC_PIN_1 A0
#define RX_ADC_PIN_2 A1
#define RX_ADC_PIN_3 A2

enum dataStates {
    QSP_STATE_IDLE,
    QSP_STATE_PREAMBLE_RECEIVED,
    QSP_STATE_CHANNEL_RECEIVED,
    QSP_STATE_FRAME_TYPE_RECEIVED,
    QSP_STATE_PACKET_ID_RECEIVED,
    QSP_STATE_PAYLOAD_RECEIVED,
    QSP_STATE_CRC_RECEIVED
};

enum deviceStates {
    DEVICE_STATE_OK,
    DEVICE_STATE_FAILSAFE
};

enum debugConfigFlags {
    DEBUG_FLAG_SERIAL   = 0b00000001,
    DEBUG_FLAG_LED      = 0b00000010
};

#define PPM_INPUT_PIN       0 // Has to be one of Interrupt pins
#define PPM_INPUT_INTERRUPT 2 // For Pro Micro 1, For Pro Mini 0

#define PPM_INPUT_CHANNEL_COUNT 10
#define PPM_OUTPUT_CHANNEL_COUNT 10

#define PPM_CHANNEL_DEFAULT_VALUE 1500  //set the default servo value
#define PPM_FRAME_LENGTH 30500  //set the PPM frame length in microseconds (1ms = 1000µs)
#define PPM_PULSE_LENGTH 300  //set the pulse length
#define PPM_OUTPUT_MULTIPLIER 1 //1 for 8MHz RX, 2 for 16MHz RX
#define PPM_SIGNAL_POSITIVE_STATE 1  //set polarity of the pulses: 1 is positive, 0 is negative
#define PPM_OUTPUT_PIN 10  //set PPM signal output pin on the arduino

struct QspConfiguration_t {
    uint8_t protocolState = QSP_STATE_IDLE;
    uint8_t crc = 0;
    uint8_t payload[QSP_PAYLOAD_LENGTH] = {0};
    uint8_t payloadLength = 0;
    uint8_t frameToSend = 0;
    uint32_t lastFrameReceivedAt[QSP_FRAME_COUNT] = {0};
    uint32_t lastFrameTransmitedAt[QSP_FRAME_COUNT] = {0};
    uint8_t deviceState = DEVICE_STATE_OK;
    void (* hardwareWriteFunction)(uint8_t, QspConfiguration_t*);
    uint8_t lastReceivedPacketId = 0;
    bool canTransmit = false;
    bool forcePongFrame = false;
    uint8_t debugConfig = 0;
    uint32_t frameDecodingStartedAt = 0;
};

struct RxDeviceState_t {
    int rssi = 0;
    float snr = 0;
    uint8_t rxVoltage = 0;
    uint8_t a1Voltage = 0;
    uint8_t a2Voltage = 0;
};