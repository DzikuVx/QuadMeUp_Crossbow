// #include "build/debug.h"
// #include "common/math.h"
// #include "common/crc.h"
#include "Arduino.h"
#include "variables.h"
#include "crsfReceiver.h"

// bool rcCrsfReceiver_c::init(serialPort_c::id_e serialPortId)
// {
//     // Set channel count for reference
//     rcProtocol_c::init(CHANNEL_COUNT);

//     // Set all channels to zero
//     for (auto ch = 0; ch < channelCount; ch++) {
//         setChannel(ch, 0);
//     }

//     // Start in failsafe mode
//     setLinkStatus(rcProtocol_c::LINK_FAILSAFE);

//     // Open serial port
//     const serialPort_c::options_e portOptions = static_cast<serialPort_c::options_e>(CRSF_PORT_OPTIONS);

//     serialPort = serialPortOpen(serialPortId, nullptr, CRSF_BAUD_RATE, serialPort_c::MODE_RXTX, portOptions);
//     if (serialPort == nullptr) {
//         return false;
//     }

//     return true;
// }

// rcProtocol_c::linkStatus_e rcCrsfReceiver_c::getLinkStatus(void) const
// {
//     return linkStatus;
// }

// void rcCrsfReceiver_c::setLinkStatus(rcProtocol_c::linkStatus_e _linkStatus)
// {
//     linkStatus = _linkStatus;
// }

// int16_t rcCrsfReceiver_c::getChannel(unsigned _ch) const
// {
//     if (_ch < CHANNEL_COUNT) {
//         return rcData[_ch];
//     }
//     else {
//         return 0;
//     }
// }

// void rcCrsfReceiver_c::setChannel(unsigned _ch, int16_t _value)
// {
//     if (_ch < CHANNEL_COUNT) {
//         rcData[_ch] = constrain(_value, -1024, 1024);
//     }
// }

void crsfOnByteReceived(CrsfState_t *crsf, uint32_t currentTimeUs, const uint8_t c)
{
    uint32_t deltaTimeUs = currentTimeUs - crsf->frameStartTimeUs;
    if (deltaTimeUs > CRSF_TIME_NEEDED_PER_FRAME_US) {
        // We've received a character after max time needed to complete a frame,
        // so this must be the start of a new frame.
        crsf->framePos = 0;
    }

    if (crsf->framePos == 0) {
        crsf->frameStartTimeUs = currentTimeUs;
    }
    
    if (crsf->framePos < CRSF_FRAME_SIZE_MAX) {
        crsf->frameBuf[crsf->framePos] = c;
    }

    switch (crsf->framePos) {
        case 1:
            // full frame length includes the length of the address and framelength fields
            crsf->fullFrameLength = crsf->frameBuf[crsf->framePos] + CRSF_FRAME_LENGTH_ADDRESS + CRSF_FRAME_LENGTH_FRAMELENGTH;
            break;
        case 2:
            crsf->frameType = crsf->frameBuf[crsf->framePos];
            break;
    }

    ++crsf->framePos;
    
    if (crsf->framePos >= crsf->fullFrameLength && crsf->fullFrameLength > 0) {
        crsf->framePos = 0;
        
        if (crsf->frameType == CRSF_FRAMETYPE_RC_CHANNELS_PACKED) {
            // const uint8_t crc = crc8::dvbS2(0, &frameBuf[2], FRAME_LENGTH_TYPE + FRAME_RC_CHANNELS_PAYLOAD_SIZE);
            // if (crc != frameBuf[FRAME_RC_CHANNELS_PAYLOAD_SIZE + FRAME_LENGTH_NON_PAYLOAD - 1]) { // crc is last byte
                // return;
            // }

            // unpack the channels
            const crsfPackedFrame_s* crsfPackedFrame = reinterpret_cast<crsfPackedFrame_s*>(&crsf->frameBuf[3]);
            crsf->rcData[0] = constrain(((int32_t)crsfPackedFrame->ch0 - 992) * 2048 / (1811 - 172), -1024, 1024);
            crsf->rcData[1] = constrain(((int32_t)crsfPackedFrame->ch1 - 992) * 2048 / (1811 - 172), -1024, 1024);
            crsf->rcData[2] = constrain(((int32_t)crsfPackedFrame->ch2 - 992) * 2048 / (1811 - 172), -1024, 1024);
            crsf->rcData[3] = constrain(((int32_t)crsfPackedFrame->ch3 - 992) * 2048 / (1811 - 172), -1024, 1024);
            crsf->rcData[4] = constrain(((int32_t)crsfPackedFrame->ch4 - 992) * 2048 / (1811 - 172), -1024, 1024);
            crsf->rcData[5] = constrain(((int32_t)crsfPackedFrame->ch5 - 992) * 2048 / (1811 - 172), -1024, 1024);
            crsf->rcData[6] = constrain(((int32_t)crsfPackedFrame->ch6 - 992) * 2048 / (1811 - 172), -1024, 1024);
            crsf->rcData[7] = constrain(((int32_t)crsfPackedFrame->ch7 - 992) * 2048 / (1811 - 172), -1024, 1024);
            crsf->rcData[8] = constrain(((int32_t)crsfPackedFrame->ch8 - 992) * 2048 / (1811 - 172), -1024, 1024);
            crsf->rcData[9] = constrain(((int32_t)crsfPackedFrame->ch9 - 992) * 2048 / (1811 - 172), -1024, 1024);
            crsf->rcData[10] = constrain(((int32_t)crsfPackedFrame->ch10 - 992) * 2048 / (1811 - 172), -1024, 1024);
            crsf->rcData[11] = constrain(((int32_t)crsfPackedFrame->ch11 - 992) * 2048 / (1811 - 172), -1024, 1024);
            crsf->rcData[12] = constrain(((int32_t)crsfPackedFrame->ch12 - 992) * 2048 / (1811 - 172), -1024, 1024);
            crsf->rcData[13] = constrain(((int32_t)crsfPackedFrame->ch13 - 992) * 2048 / (1811 - 172), -1024, 1024);
            crsf->rcData[14] = constrain(((int32_t)crsfPackedFrame->ch14 - 992) * 2048 / (1811 - 172), -1024, 1024);
            crsf->rcData[15] = constrain(((int32_t)crsfPackedFrame->ch15 - 992) * 2048 / (1811 - 172), -1024, 1024);

            crsf->frameCompleteTimeUs = currentTimeUs;
        }
        else {
            // Nothing, skip
        }
    }
}

// void rcCrsfReceiver_c::run(timeUs_t currentTimeUs)
// {
//     if (!serialPort) {
//         return;
//     }

//     while (serialPort->rxWaitingCount() > 0) {
//         const uint8_t c = serialPort->read();
//         onByteReceived(currentTimeUs, c);
//     }

//     if (cmpTimeUs(currentTimeUs, frameStartTimeUs) > CONNECTION_LOSS_INTERVAL_US) {
//         linkStatus = LINK_FAILSAFE;
//     }
//     else {
//         linkStatus = LINK_OK;
//     }
// }