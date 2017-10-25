#pragma once

#define CRSF_FRAME_SIZE_MAX 64
#define CRSF_TIME_NEEDED_PER_FRAME_US 3000
#define CRSF_CONNECTION_LOSS_INTERVAL_US = 12000
#define CRSF_CHANNEL_COUNT 16

enum frameLength_e {
    CRSF_FRAME_LENGTH_ADDRESS = 1, // length of ADDRESS field
    CRSF_FRAME_LENGTH_FRAMELENGTH = 1, // length of FRAMELENGTH field
    CRSF_FRAME_LENGTH_TYPE = 1, // length of TYPE field
    CRSF_FRAME_LENGTH_CRC = 1, // length of CRC field
    CRSF_FRAME_LENGTH_TYPE_CRC = 2, // length of TYPE and CRC fields combined
    CRSF_FRAME_LENGTH_NON_PAYLOAD = 4 // combined length of all fields except payload
};

enum crsfFrameType_e {
    // Broadcast Frames, range: 0x00 to 0x27
    CRSF_RAMETYPE_GPS = 0x02,
    CRSF_FRAMETYPE_BATTERY_SENSOR = 0x08,
    CRSF_FRAMETYPE_LINK_HEARTBEAT = 0x0B,
    CRSF_FRAMETYPE_LINK_STATISTICS = 0x14,
    CRSF_FRAMETYPE_RC_CHANNELS_PACKED = 0x16,
    CRSF_FRAMETYPE_RC_CHANNELS_VARIABLE_PACKED = 0x17,
    CRSF_FRAMETYPE_ATTITUDE = 0x1E,
    CRSF_FRAMETYPE_FLIGHT_MODE = 0x21,
    // Extended Header Frames, range: 0x28 to 0x96
    CRSF_FRAMETYPE_PARAMETER_PING = 0x28,
    CRSF_FRAMETYPE_PARAMETER_DEVICE_INFO = 0x29,
    CRSF_FRAMETYPE_PARAMETER_SETTINGS_ENTRY = 0x2B,
    CRSF_FRAMETYPE_PARAMETER_READ = 0x2C,
    CRSF_FRAMETYPE_PARAMETER_WRITE = 0x2D,
    CRSF_FRAMETYPE_COMMAND = 0x32
};

struct crsfPackedFrame_s {
    // 176 bits of data (11 bits per channel * 16 channels) = 22 bytes.
                            //bits   :bytes
    unsigned int ch0  : 11; //  0, 10: 1,2
    unsigned int ch1  : 11; // 11, 21: 2,3
    unsigned int ch2  : 11; // 22, 32: 3,4,5
    unsigned int ch3  : 11; // 33, 42: 5,6
    unsigned int ch4  : 11; // 44, 54: 6,7
    unsigned int ch5  : 11; // 55, 65: 7,8,9
    unsigned int ch6  : 11; // 66, 76: 9,10
    unsigned int ch7  : 11; // 77, 87:10,11
    unsigned int ch8  : 11; // 88, 11:12,13
    unsigned int ch9  : 11; // 99, 12:13,14,15
    unsigned int ch10 : 11; //110,120:15,16
    unsigned int ch11 : 11; //121,131:16,17
    unsigned int ch12 : 11; //132,142:17,18,19
    unsigned int ch13 : 11; //143,153:19,20
    unsigned int ch14 : 11; //154,164,20,21
    unsigned int ch15 : 11; //165,175:21,22
} __attribute__ ((__packed__));

struct CrsfState_t {
    uint32_t frameStartTimeUs = 0;
    uint32_t frameCompleteTimeUs;
    uint8_t frameType;
    uint8_t framePos;
    uint8_t fullFrameLength;
    uint8_t frameBuf[CRSF_FRAME_SIZE_MAX + 2]; // +2 for sync byte
    int rcData[CRSF_CHANNEL_COUNT];  // RC channels [-1024:+1024]
};

void crsfOnByteReceived(CrsfState_t *crsf, uint32_t currentTimeUs, const uint8_t c);

// #include <cstddef>
// #include <cstdint>

// #include "common/base.h"
// #include "drivers/serialPort.h"
// #include "io/rcProtocol.h"
/*
class rcCrsfReceiver_c : public rcProtocol_c {
public:

private:
    struct crsfPackedFrame_s {
        // 176 bits of data (11 bits per channel * 16 channels) = 22 bytes.
                                //bits   :bytes
        unsigned int ch0  : 11; //  0, 10: 1,2
        unsigned int ch1  : 11; // 11, 21: 2,3
        unsigned int ch2  : 11; // 22, 32: 3,4,5
        unsigned int ch3  : 11; // 33, 42: 5,6
        unsigned int ch4  : 11; // 44, 54: 6,7
        unsigned int ch5  : 11; // 55, 65: 7,8,9
        unsigned int ch6  : 11; // 66, 76: 9,10
        unsigned int ch7  : 11; // 77, 87:10,11
        unsigned int ch8  : 11; // 88, 11:12,13
        unsigned int ch9  : 11; // 99, 12:13,14,15
        unsigned int ch10 : 11; //110,120:15,16
        unsigned int ch11 : 11; //121,131:16,17
        unsigned int ch12 : 11; //132,142:17,18,19
        unsigned int ch13 : 11; //143,153:19,20
        unsigned int ch14 : 11; //154,164,20,21
        unsigned int ch15 : 11; //165,175:21,22
    } __attribute__ ((__packed__));

private:
    enum frameType_e {
        // Broadcast Frames, range: 0x00 to 0x27
        FRAMETYPE_GPS = 0x02,
        FRAMETYPE_BATTERY_SENSOR = 0x08,
        FRAMETYPE_LINK_HEARTBEAT = 0x0B,
        FRAMETYPE_LINK_STATISTICS = 0x14,
        FRAMETYPE_RC_CHANNELS_PACKED = 0x16,
        FRAMETYPE_RC_CHANNELS_VARIABLE_PACKED = 0x17,
        FRAMETYPE_ATTITUDE = 0x1E,
        FRAMETYPE_FLIGHT_MODE = 0x21,
        // Extended Header Frames, range: 0x28 to 0x96
        FRAMETYPE_PARAMETER_PING = 0x28,
        FRAMETYPE_PARAMETER_DEVICE_INFO = 0x29,
        FRAMETYPE_PARAMETER_SETTINGS_ENTRY = 0x2B,
        FRAMETYPE_PARAMETER_READ = 0x2C,
        FRAMETYPE_PARAMETER_WRITE = 0x2D,
        FRAMETYPE_COMMAND = 0x32
    };
    enum frameLength_e {
        FRAME_LENGTH_ADDRESS = 1, // length of ADDRESS field
        FRAME_LENGTH_FRAMELENGTH = 1, // length of FRAMELENGTH field
        FRAME_LENGTH_TYPE = 1, // length of TYPE field
        FRAME_LENGTH_CRC = 1, // length of CRC field
        FRAME_LENGTH_TYPE_CRC = 2, // length of TYPE and CRC fields combined
        FRAME_LENGTH_NON_PAYLOAD = 4 // combined length of all fields except payload
    };
    enum payloadSize_e {
        FRAME_GPS_PAYLOAD_SIZE = 15,
        FRAME_BATTERY_SENSOR_PAYLOAD_SIZE = 8,
        FRAME_LINK_STATISTICS_PAYLOAD_SIZE = 10,
        FRAME_RC_CHANNELS_PAYLOAD_SIZE = 22, // 11 bits per channel * 16 channels = 22 bytes.
        FRAME_ATTITUDE_PAYLOAD_SIZE = 6,
    };

    enum { FRAME_SIZE_MAX = 64 }; // 62 bytes frame plus 2 bytes frame header(<length><type>)
    enum { CHANNEL_COUNT = 16 };
    enum { TIME_NEEDED_PER_FRAME_US = 1500, CONNECTION_LOSS_INTERVAL_US = 12000 };  // connection is considered lost if 3 consecutive updates are missed
    enum { CRSF_BAUD_RATE = 420000 };
    // enum { CRSF_BAUD_RATE = 9600 };
    enum { CRSF_PORT_OPTIONS = serialPort_c::STOPBITS_1 | serialPort_c::PARITY_NONE | serialPort_c::INVERTED };
    // enum { CRSF_PORT_OPTIONS = serialPort_c::STOPBITS_1 | serialPort_c::PARITY_NONE };

private:
    timeUs_t frameStartTimeUs;
    timeUs_t frameCompleteTimeUs;
    uint8_t frameType;
    uint8_t framePos;
    uint8_t fullFrameLength;
    uint8_t frameBuf[FRAME_SIZE_MAX + 2]; // +2 for sync byte

    int16_t             rcData[CHANNEL_COUNT];  // RC channels [-1024:+1024]
    serialPort_c *      serialPort;

    void onByteReceived(timeUs_t currentTimeUs, const uint8_t c);
    
public:
    bool init(serialPort_c::id_e serialPortId);

    virtual linkStatus_e getLinkStatus(void) const override;
    virtual void setLinkStatus(linkStatus_e _linkStatus) override;
    virtual int16_t getChannel(unsigned _ch) const override;
    virtual void setChannel(unsigned _ch, int16_t _value) override;
    virtual void run(timeUs_t currentTimeUs) override;

    inline uint8_t getChannelCount(void) { return channelCount; }
};*/