#include "Arduino.h"
#include "variables.h"
#include "sbus.h"

#define SBUS_MIN_OFFSET 173
#define SBUS_MID_OFFSET 992
#define SBUS_MAX_OFFSET 1811
#define SBUS_CHANNEL_NUMBER 16
#define SBUS_PACKET_LENGTH 25
#define SBUS_FRAME_HEADER 0x0f
#define SBUS_FRAME_FOOTER 0x00
#define SBUS_FRAME_FOOTER_V2 0x04
#define SBUS_STATE_FAILSAFE 0x08
#define SBUS_STATE_SIGNALLOSS 0x04

#define SBUS_IS_RECEIVING_THRESHOLD 125 //If there is no SBUS input for 125ms, assume connection is broken

/*
Precomputed mapping from 990-2010 to 173:1811
equivalent to 
map(channels[i], RC_CHANNEL_MIN, RC_CHANNEL_MAX, SBUS_MIN_OFFSET, SBUS_MAX_OFFSET);
*/
int mapChannelToSbus(int in) {
    return ((long) in * 1605l / 1000l) - 1417;
}

/*
Precomputed mapping from 173:1811 to 990-2010
*/
int mapSbusToChannel(int in) {
    return (((long) in - 173l) * 1020l / 1638l) + 990;
}

void sbusPreparePacket(uint8_t packet[], int16_t channels[], bool isSignalLoss, bool isFailsafe){
    
    static int output[SBUS_CHANNEL_NUMBER] = {0};
    
    /*
     * Map 1000-2000 with middle at 1500 chanel values to
     * 173-1811 with middle at 992 S.BUS protocol requires
     */
    for (uint8_t i = 0; i < SBUS_CHANNEL_NUMBER; i++) {
        output[i] = mapChannelToSbus(channels[i]);
    }

    uint8_t stateByte = 0x00;
    if (isSignalLoss) {
        stateByte |= SBUS_STATE_SIGNALLOSS;
    }
    if (isFailsafe) {
        stateByte |= SBUS_STATE_FAILSAFE;
    }
    packet[0] = SBUS_FRAME_HEADER; //Header
    
    packet[1] = (uint8_t) (output[0] & 0x07FF);
    packet[2] = (uint8_t) ((output[0] & 0x07FF)>>8 | (output[1] & 0x07FF)<<3);
    packet[3] = (uint8_t) ((output[1] & 0x07FF)>>5 | (output[2] & 0x07FF)<<6);
    packet[4] = (uint8_t) ((output[2] & 0x07FF)>>2);
    packet[5] = (uint8_t) ((output[2] & 0x07FF)>>10 | (output[3] & 0x07FF)<<1);
    packet[6] = (uint8_t) ((output[3] & 0x07FF)>>7 | (output[4] & 0x07FF)<<4);
    packet[7] = (uint8_t) ((output[4] & 0x07FF)>>4 | (output[5] & 0x07FF)<<7);
    packet[8] = (uint8_t) ((output[5] & 0x07FF)>>1);
    packet[9] = (uint8_t) ((output[5] & 0x07FF)>>9 | (output[6] & 0x07FF)<<2);
    packet[10] = (uint8_t) ((output[6] & 0x07FF)>>6 | (output[7] & 0x07FF)<<5);
    packet[11] = (uint8_t) ((output[7] & 0x07FF)>>3);
    packet[12] = (uint8_t) ((output[8] & 0x07FF));
    packet[13] = (uint8_t) ((output[8] & 0x07FF)>>8 | (output[9] & 0x07FF)<<3);
    packet[14] = (uint8_t) ((output[9] & 0x07FF)>>5 | (output[10] & 0x07FF)<<6);  
    packet[15] = (uint8_t) ((output[10] & 0x07FF)>>2);
    packet[16] = (uint8_t) ((output[10] & 0x07FF)>>10 | (output[11] & 0x07FF)<<1);
    packet[17] = (uint8_t) ((output[11] & 0x07FF)>>7 | (output[12] & 0x07FF)<<4);
    packet[18] = (uint8_t) ((output[12] & 0x07FF)>>4 | (output[13] & 0x07FF)<<7);
    packet[19] = (uint8_t) ((output[13] & 0x07FF)>>1);
    packet[20] = (uint8_t) ((output[13] & 0x07FF)>>9 | (output[14] & 0x07FF)<<2);
    packet[21] = (uint8_t) ((output[14] & 0x07FF)>>6 | (output[15] & 0x07FF)<<5);
    packet[22] = (uint8_t) ((output[15] & 0x07FF)>>3);
    
    packet[23] = stateByte; //Flags byte
    packet[24] = SBUS_FRAME_FOOTER; //Footer
}

SbusInput::SbusInput(HardwareSerial &serial) : _serial(serial)
{
}


void SbusInput::loop(void)
{
    if (_serial.available()) {
        sbusRead();
    }
}

void SbusInput::start(void)
{
    _serial.begin(100000, SERIAL_8N2);
}

void SbusInput::restart(void)
{   
    _serial.end();
    start();
}

/*
 * We prevent from frame decoding being stuck somewhere
 */
void SbusInput::recoverStuckFrames(void)
{
    if (
        _protocolState == SBUS_DECODING_STATE_IN_PROGRESS &&
        millis() - _frameDecodingStartedAt > 6
    ) {
        _protocolState = SBUS_DECODING_STATE_IDLE;
        _serial.flush();
    }
}

void sbusToChannels(volatile int16_t channels[], byte buffer[]) {
    channels[0]  = ((buffer[1]    |buffer[2]<<8)                 & 0x07FF);
    channels[1]  = ((buffer[2]>>3 |buffer[3]<<5)                 & 0x07FF);
    channels[2]  = ((buffer[3]>>6 |buffer[4]<<2 |buffer[5]<<10)  & 0x07FF);
    channels[3]  = ((buffer[5]>>1 |buffer[6]<<7)                 & 0x07FF);
    channels[4]  = ((buffer[6]>>4 |buffer[7]<<4)                 & 0x07FF);
    channels[5]  = ((buffer[7]>>7 |buffer[8]<<1 |buffer[9]<<9)   & 0x07FF);
    channels[6]  = ((buffer[9]>>2 |buffer[10]<<6)                & 0x07FF);
    channels[7]  = ((buffer[10]>>5|buffer[11]<<3)                & 0x07FF);
    channels[8]  = ((buffer[12]   |buffer[13]<<8)                & 0x07FF);
    channels[9]  = ((buffer[13]>>3|buffer[14]<<5)                & 0x07FF);
    channels[10] = ((buffer[14]>>6|buffer[15]<<2|buffer[16]<<10) & 0x07FF);
    channels[11] = ((buffer[16]>>1|buffer[17]<<7)                & 0x07FF);
    channels[12] = ((buffer[17]>>4|buffer[18]<<4)                & 0x07FF);
    channels[13] = ((buffer[18]>>7|buffer[19]<<1|buffer[20]<<9)  & 0x07FF);
    channels[14] = ((buffer[20]>>2|buffer[21]<<6)                & 0x07FF);
    channels[15] = ((buffer[21]>>5|buffer[22]<<3)                & 0x07FF);

    for (uint8_t channelIndex = 0; channelIndex < SBUS_CHANNEL_NUMBER; channelIndex++) {
        channels[channelIndex] = mapSbusToChannel(channels[channelIndex]);
    }
}

void SbusInput::sbusRead() {
    static byte buffer[25];
	static byte buffer_index = 0;

    static uint32_t _decoderErrorFrames;
    static uint32_t _goodFrames;

	while (_serial.available()) {
		byte rx = _serial.read();

        if (_protocolState == SBUS_DECODING_STATE_IDLE) {
            //We allow next frame to start only when previous frame eneded some time ago
            if (
                rx == SBUS_FRAME_HEADER &&
                millis() - _frameDecodingEndedAt > 5
            ) {
                //Header is correct
                _frameDecodingStartedAt = millis();
                buffer_index = 0;
                _protocolState = SBUS_DECODING_STATE_IN_PROGRESS;
            }
        } 

        buffer[buffer_index] = rx;
        buffer_index++;

        if (
            _protocolState == SBUS_DECODING_STATE_IN_PROGRESS && 
            buffer_index == 25 &&
            buffer[24] == SBUS_FRAME_FOOTER
        ) {
            //We have full frame now
            _frameDecodingEndedAt = millis();
            sbusToChannels(channels, buffer);
            _protocolState = SBUS_DECODING_STATE_IDLE;
        }
	}
}

bool SbusInput::isReceiving() {
    return  !(millis() - _frameDecodingEndedAt > SBUS_IS_RECEIVING_THRESHOLD);
}