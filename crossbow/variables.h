#include "Arduino.h"

#pragma once

#define OLED_UPDATE_RATE 750

#define SBUS_UPDATE_RATE 15 //ms
#define SBUS_PACKET_LENGTH 25

#define RC_CHANNEL_MIN 990
#define RC_CHANNEL_MAX 2010

#define RX_TASK_HEALTH 200 //5Hz should be enough
#define RSSI_CHANNEL 11

#define TX_TRANSMIT_SLOT_RATE 67 //ms
#define RX_FAILSAFE_DELAY (TX_TRANSMIT_SLOT_RATE * 8)
#define TX_FAILSAFE_DELAY (RX_FAILSAFE_DELAY * 4)

#define CHANNEL_ID 0x01
#define QSP_PAYLOAD_LENGTH 32

#define QSP_MAX_FRAME_DECODE_TIME 10 //max time that frame can be decoded in ms

#define QSP_FRAME_RC_DATA 0x0
#define QSP_FRAME_RX_HEALTH 0x1
#define QSP_FRAME_GET_RX_CONFIG 0x2
#define QSP_FRAME_RX_CONFIG 0x3
#define QSP_FRAME_SET_RX_CONFIG 0x4
#define QSP_FRAME_PING 0x5
#define QSP_FRAME_PONG 0x6
#define QSP_FRAME_COUNT 0x7

static const uint8_t qspFrameLengths[QSP_FRAME_COUNT] = {
    9, //QSP_FRAME_RC_DATA
    6, //QSP_FRAME_RX_HEALTH
    0, //QSP_FRAME_GET_RX_CONFIG -> Not used
    0, //QSP_FRAME_RX_CONFIG -> Not used
    0, //QSP_FRAME_SET_RX_CONFIG -> Not used
    4, //QSP_FRAME_PING
    4, //QSP_FRAME_PONG
};

#define RX_ADC_PIN_1 A0
#define RX_ADC_PIN_2 A1
#define RX_ADC_PIN_3 A2

enum dataStates {
    QSP_STATE_IDLE,
    QSP_STATE_CHANNEL_RECEIVED,
    QSP_STATE_FRAME_TYPE_RECEIVED,
    QSP_STATE_PAYLOAD_RECEIVED,
    QSP_STATE_CRC_RECEIVED
};

enum deviceStates {
    DEVICE_STATE_OK,
    DEVICE_STATE_FAILSAFE,
    DEVICE_STATE_UNDETERMINED
};

enum debugConfigFlags {
    DEBUG_FLAG_SERIAL   = 0b00000001,
    DEBUG_FLAG_LED      = 0b00000010
};

#define PPM_INPUT_PIN       0 // Has to be one of Interrupt pins

#define PPM_INPUT_CHANNEL_COUNT 10
#define PPM_OUTPUT_CHANNEL_COUNT 10

#define TX_BUZZER_PIN A5

#define PPM_CHANNEL_DEFAULT_VALUE 1500  //set the default servo value
#define PPM_FRAME_LENGTH 30500  //set the PPM frame length in microseconds (1ms = 1000µs)
#define PPM_PULSE_LENGTH 300  //set the pulse length
#define PPM_OUTPUT_MULTIPLIER 1 //1 for 8MHz RX, 2 for 16MHz RX
#define PPM_SIGNAL_POSITIVE_STATE 1  //set polarity of the pulses: 1 is positive, 0 is negative
#define PPM_OUTPUT_PIN 10  //set PPM signal output pin on the arduino

#define MIN_PACKET_SIZE 3 //Min theorethical size of valid packet 
#define MAX_PACKET_SIZE 20 //Max theorethical size of valid packet

#define NO_DATA_TO_READ -1

#define RADIO_STATE_TX 1
#define RADIO_STATE_RX 2

struct RadioState_t {
    uint32_t frequency = 867000000;
    uint32_t loraBandwidth = 250000;
    uint8_t loraSpreadingFactor = 7;
    uint8_t loraCodingRate = 6;
    uint8_t loraTxPower = 17; // Defines output power of TX, defined in dBm range from 2-17
    int8_t bytesToRead = -1;
    uint8_t rssi = 0;
    uint8_t snr = 0;
    uint8_t deviceState = RADIO_STATE_RX;
    uint32_t nextTxCheckMillis = 0;
};

struct TxDeviceState_t {
    uint8_t flags = 0;
    uint32_t roundtrip = 0;
    bool isReceiving = false; //Indicates that TX module is receiving frames from RX module
};

struct RxDeviceState_t {
    uint8_t rssi = 0;
    uint8_t snr = 0;
    uint8_t rxVoltage = 0;
    uint8_t a1Voltage = 0;
    uint8_t a2Voltage = 0;
    uint8_t flags = 0;
    int16_t channels[16] = {};
};

struct QspConfiguration_t {
    uint8_t protocolState = QSP_STATE_IDLE;
    uint8_t crc = 0;
    uint8_t payload[QSP_PAYLOAD_LENGTH] = {0};
    uint8_t payloadLength = 0;
    uint8_t frameToSend = 0;
    uint8_t frameId = 0;
    uint32_t lastFrameReceivedAt[QSP_FRAME_COUNT] = {0};
    uint32_t anyFrameRecivedAt = 0;
    uint8_t deviceState = DEVICE_STATE_UNDETERMINED;
    void (* onSuccessCallback)(QspConfiguration_t*, TxDeviceState_t*, RxDeviceState_t*, volatile RadioState_t*);
    void (* onFailureCallback)(QspConfiguration_t*, TxDeviceState_t*, RxDeviceState_t*, volatile RadioState_t*);    
    bool canTransmit = false;
    bool forcePongFrame = false;
    uint8_t debugConfig = 0;
    uint32_t frameDecodingStartedAt = 0;
    uint32_t lastTxSlotTimestamp = 0;
    bool transmitWindowOpen = false;
};
