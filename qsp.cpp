#include "Arduino.h"
#include "variables.h"
#include <PPMReader.h>

void qspDecodeRcDataFrame(uint8_t payload[], int output[]) {
    int temporaryPpmOutput[PPM_CHANNEL_COUNT] = {0};
    //TODO fix it, baby :)

    temporaryPpmOutput[0] = (uint16_t) (((uint16_t) payload[0] << 2) & 0x3fc) | ((payload[1] >> 6) & 0x03);
    temporaryPpmOutput[1] = (uint16_t) (((uint16_t) payload[1] << 4) & 0x3f0) | ((payload[2] >> 4) & 0x0F);
    temporaryPpmOutput[2] = (uint16_t) (((uint16_t) payload[2] << 6) & 0x3c0) | ((payload[3] >> 2) & 0x3F);
    temporaryPpmOutput[3] = (uint16_t) (((uint16_t) payload[3] << 8) & 0x300) | ((payload[4] >> 2) & 0xFF);
    temporaryPpmOutput[4] = payload[5];
    temporaryPpmOutput[5] = payload[6];
    temporaryPpmOutput[6] = (payload[7] >> 4) & 0b00001111;
    temporaryPpmOutput[7] = payload[7] & 0b00001111;
    temporaryPpmOutput[8] = (payload[8] >> 4) & 0b00001111;
    temporaryPpmOutput[9] = payload[8] & 0b00001111;

    //10bit channels
    temporaryPpmOutput[0] = map(temporaryPpmOutput[0], 0, 1000, 1000, 2000);
    temporaryPpmOutput[1] = map(temporaryPpmOutput[1], 0, 1000, 1000, 2000);
    temporaryPpmOutput[2] = map(temporaryPpmOutput[2], 0, 1000, 1000, 2000);
    temporaryPpmOutput[3] = map(temporaryPpmOutput[3], 0, 1000, 1000, 2000);

    //8bit channels
    temporaryPpmOutput[4] = map(temporaryPpmOutput[4], 0, 0xff, 1000, 2000);
    temporaryPpmOutput[5] = map(temporaryPpmOutput[5], 0, 0xff, 1000, 2000);

    //4bit channels
    temporaryPpmOutput[6] = map(temporaryPpmOutput[6], 0, 0x0f, 1000, 2000);
    temporaryPpmOutput[7] = map(temporaryPpmOutput[7], 0, 0x0f, 1000, 2000);
    temporaryPpmOutput[8] = map(temporaryPpmOutput[8], 0, 0x0f, 1000, 2000);
    temporaryPpmOutput[9] = map(temporaryPpmOutput[9], 0, 0x0f, 1000, 2000);

    /*
     * Copy tremporary to real output
     */
    for (uint8_t i = 0; i < PPM_CHANNEL_COUNT; i++) {
        output[i] = temporaryPpmOutput[i];
    }
}

uint8_t get10bitHighShift(uint8_t channel) {
    return ((channel % 4) * 2) + 2;
}

uint8_t get10bitLowShift(uint8_t channel) {
    return 8 - get10bitHighShift(channel);
}

void qspComputeCrc(uint8_t *crc, uint8_t dataByte)
{
    *crc ^= dataByte;
}

/**
 * Encode 10 RC channels 
 */
void encodeRcDataPayload(PPMReader *ppmSource, uint8_t noOfChannels, uint8_t payload[], uint8_t *payloadLength)
{
    for (uint8_t i = 0; i < noOfChannels; i++)
    {
        uint16_t channelValue10 = map(ppmSource->get(i), 1000, 2000, 0, 1000) & 0x03ff;
        uint8_t channelValue8 = map(ppmSource->get(i), 1000, 2000, 0, 255) & 0xff;
        uint8_t channelValue4 = map(ppmSource->get(i), 1000, 2000, 0, 15) & 0x0f;

        if (i < 4)
        {
            /*
             * First 4 channels encoded with 10 bits
             */
            uint8_t bitIndex = i + (i / 4);
            payload[bitIndex] |= (channelValue10 >> get10bitHighShift(i)) & (0x3ff >> get10bitHighShift(i));
            payload[bitIndex + 1] |= (channelValue10 << get10bitLowShift(i)) & 0xff << (8 - get10bitHighShift(i));
        }
        else if (i == 4 || i == 5)
        {
            /*
             * Next 2 with 8 bits
             */
            payload[i + 1] |= channelValue8;
        }
        else if (i == 6)
        {
            /*
             * And last 4 with 4 bits per channel
             */
            payload[7] |= (channelValue4 << 4) & B11110000;
        }
        else if (i == 7)
        {
            payload[7] |= channelValue4 & B00001111;
        }
        else if (i == 8)
        {
            payload[8] |= (channelValue4 << 4) & B11110000;
        }
        else if (i == 9)
        {
            payload[8] |= channelValue4 & B00001111;
        }
    }

    *payloadLength = 9;
}