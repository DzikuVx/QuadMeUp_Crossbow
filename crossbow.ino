#include <LoRa.h>
#include "variables.h"

// #define LORA_HARDWARE_SERIAL
#define LORA_HARDWARE_SPI

#define DEVICE_MODE_TX
// #define DEVICE_MODE_RX

/*
 * Main defines for device working in TX mode
 */
#ifdef DEVICE_MODE_TX

#include <PPMReader.h>

#define PPM_INPUT_PIN       2
#define PPM_INPUT_INTERRUPT 1 //For Pro Micro 1, For Pro Mini 0

PPMReader ppmReader(PPM_INPUT_PIN, PPM_INPUT_INTERRUPT);
#endif

/*
 * Main defines for device working in RX mode
 */
#ifdef DEVICE_MODE_RX

#include <Adafruit_SSD1306.h>

#define OLED_RESET 4
Adafruit_SSD1306 display(OLED_RESET);

#endif

/*
 * Start of QSP protocol implementation
 */
static uint8_t protocolState = IDLE;
static uint8_t packetId = 0;
static uint8_t qspCrc = 0;
static uint8_t qspPayload[QSP_PAYLOAD_LENGTH] = {0};
static uint8_t qspPayloadLength = 0;
static uint8_t qspFrameToSend = 0;

uint8_t qspGetPacketId() {
    return packetId++;
}

void qspClearPayload() {
    for (uint8_t i = 0; i < QSP_PAYLOAD_LENGTH; i++) {
        qspPayload[i] = 0;
    }
    qspPayloadLength = 0;
}

int ppmOutput[PPM_CHANNEL_COUNT] = {0};

void qspDecodeRcDataFrame() {
    //TODO fix it, baby :)

    ppmOutput[0] = (uint16_t) (((uint16_t) qspPayload[0] << 2) & 0x3fc) | ((qspPayload[1] >> 6) & 0x03);
    ppmOutput[1] = (uint16_t) (((uint16_t) qspPayload[1] << 4) & 0x3f0) | ((qspPayload[2] >> 4) & 0x0F);
    ppmOutput[2] = (uint16_t) (((uint16_t) qspPayload[2] << 6) & 0x3c0) | ((qspPayload[3] >> 2) & 0x3F);
    ppmOutput[3] = (uint16_t) (((uint16_t) qspPayload[3] << 8) & 0x300) | ((qspPayload[4] >> 2) & 0xFF);
    ppmOutput[4] = qspPayload[5];
    ppmOutput[5] = qspPayload[6];
    ppmOutput[6] = (qspPayload[7] >> 4) & 0b00001111;
    ppmOutput[7] = qspPayload[7] & 0b00001111;
    ppmOutput[8] = (qspPayload[8] >> 4) & 0b00001111;
    ppmOutput[9] = qspPayload[8] & 0b00001111;

    //10bit channels
    ppmOutput[0] = map(ppmOutput[0], 0, 1000, 1000, 2000);
    ppmOutput[1] = map(ppmOutput[1], 0, 1000, 1000, 2000);
    ppmOutput[2] = map(ppmOutput[2], 0, 1000, 1000, 2000);
    ppmOutput[3] = map(ppmOutput[3], 0, 1000, 1000, 2000);

    //8bit channels
    ppmOutput[4] = map(ppmOutput[4], 0, 0xff, 1000, 2000);
    ppmOutput[5] = map(ppmOutput[5], 0, 0xff, 1000, 2000);

    //4bit channels
    ppmOutput[6] = map(ppmOutput[6], 0, 0x0f, 1000, 2000);
    ppmOutput[7] = map(ppmOutput[7], 0, 0x0f, 1000, 2000);
    ppmOutput[8] = map(ppmOutput[8], 0, 0x0f, 1000, 2000);
    ppmOutput[9] = map(ppmOutput[9], 0, 0x0f, 1000, 2000);
}

void qspDecodeIncomingFrame(uint8_t incomingByte) {
    static uint8_t frameId;
    static uint8_t payloadLength;
    static uint8_t receivedPayload;
    static uint8_t packetId; //TODO move this to global scope maybe?

    if (protocolState == IDLE && incomingByte == QSP_PREAMBLE) {
        //If in IDLE and correct preamble comes, start to decode frame
        protocolState = PREAMBLE_RECEIVED;
        qspCrc = 0 ^ incomingByte;
    } else if (protocolState == PREAMBLE_RECEIVED) {
        // Check if incomming channel ID is the same as receiver
        if (incomingByte == CHANNEL_ID) {
            protocolState = CHANNEL_RECEIVED;
            qspCrc ^= incomingByte;

            for (uint8_t i = 0; i < QSP_PAYLOAD_LENGTH; i++) {
                qspPayload[i] = 0x00;
            }

            receivedPayload = 0;
            packetId = 0;
        } else {
            protocolState = IDLE;
        }
    } else if (protocolState == CHANNEL_RECEIVED) {
        //Frame ID and payload length
        qspCrc ^= incomingByte;

        frameId = (incomingByte >> 4) & 0x0f;
        payloadLength = incomingByte & 0x0f;

        protocolState = FRAME_TYPE_RECEIVED;

    } else if (protocolState == FRAME_TYPE_RECEIVED) {
        qspCrc ^= incomingByte;
        packetId = incomingByte;
        protocolState = PACKET_ID_RECEIVED;
    } else if (protocolState == PACKET_ID_RECEIVED) {

        //Now it's time for payload
        qspCrc ^= incomingByte;
        qspPayload[receivedPayload] = incomingByte;

        receivedPayload++;

        if (receivedPayload == payloadLength) {
            protocolState = PAYLOAD_RECEIVED;    
        }

    } else if (protocolState == PAYLOAD_RECEIVED) {

        if (qspCrc == incomingByte) {
            //CRC is correct

            switch (frameId) {
                case QSP_FRAME_RC_DATA:
                    qspDecodeRcDataFrame();
                    break;

                default:
                    //Unknown frame
                    //TODO do something in this case
                    break;
            }

        } else {
            //CRC failed, frame has to be rejected
            //TODO do something in this case or something
        }

        // In both cases switch to listening for next preamble
        protocolState = IDLE;
    }

}

void qspEncodeFrame(uint8_t frameId, uint8_t length, uint8_t *payload) {
    //Zero CRC
    qspCrc = 0;
    
    //Write preamble
    writeToRadio(QSP_PREAMBLE);
    //Write CHANNEL_ID
    writeToRadio(CHANNEL_ID);

    //Write frame type and length
    uint8_t data = length & 0x0f;
    data |= (frameId << 4) & 0xf0;
    writeToRadio(data);

    //Write packet ID
    writeToRadio(qspGetPacketId());

    //Write payload
    for (uint8_t i = 0; i < length; i++) {
        writeToRadio(payload[i]);
    }

    //Finally write CRC
    writeToRadio(qspCrc);
}

/*
 * End of QSP protocol implementation
 */

static uint32_t lastRcFrameTransmit = 0;

uint8_t get10bitHighShift(uint8_t channel) {
    return ((channel % 4) * 2) + 2;
}

uint8_t get10bitLowShift(uint8_t channel) {
    return 8 - get10bitHighShift(channel);
}

void computeCrc(uint8_t dataByte) {
    qspCrc ^= dataByte;
}

/*
 * Serial port used to send data
 */
#ifdef LORA_HARDWARE_SERIAL

void radioPacketStart(void) {

}

void radioPacketEnd(void) {
    Serial.end();
    delay(E45_TTL_100_UART_DOWNTIME);
    Serial.begin(UART_SPEED);
}

void writeToRadio(uint8_t dataByte) {
    //Compute CRC
    computeCrc(dataByte);

    //Write to radio
    Serial.write(dataByte);
}

#endif

#ifdef LORA_HARDWARE_SPI

void radioPacketStart(void) {
    LoRa.beginPacket();
}

void radioPacketEnd(void) {
    LoRa.endPacket();
}

void writeToRadio(uint8_t dataByte) {
    //Compute CRC
    computeCrc(dataByte);

    //Write to radio
    LoRa.write(dataByte);
}

#endif

/*
display.clearDisplay();
display.setCursor(0,0);
display.print("Lat:");
display.print(remoteData.latitude);
display.display();
*/

void setup(void) {

#ifdef LORA_HARDWARE_SERIAL
    Serial.begin(UART_SPEED);
#endif

#ifdef LORA_HARDWARE_SPI
    if (!LoRa.begin(868E6)) {
        Serial.println("LoRa init failed. Check your connections.");
        while (true);
    }
    LoRa.onReceive(onReceive);
    LoRa.receive();
#endif

#ifdef DEVICE_MODE_RX
    pinMode(PIN_LED, OUTPUT);

    /*
     * Initialize OLED display
     */
    display.begin(SSD1306_SWITCHCAPVCC, 0x3C);  // initialize with the I2C addr 0x3C (for the 128x32)
    display.setTextSize(1);
    display.setTextColor(WHITE);
    display.clearDisplay();
    display.display();

#endif
}

#ifdef DEVICE_MODE_TX

/**
 * Encode 10 RC channels 
 */
void encodeRcDataPayload(PPMReader* ppmSource, uint8_t noOfChannels) {
    for (uint8_t i = 0; i < noOfChannels; i++) {
        uint16_t channelValue10 = map(ppmSource->get(i), 1000, 2000, 0, 1000) & 0x03ff;
        uint8_t channelValue8 = map(ppmSource->get(i), 1000, 2000, 0, 255) & 0xff;
        uint8_t channelValue4 = map(ppmSource->get(i), 1000, 2000, 0, 15) & 0x0f;

        if (i < 4) {
            /*
             * First 4 channels encoded with 10 bits
             */
            uint8_t bitIndex = i + (i / 4);
            qspPayload[bitIndex] |= (channelValue10 >> get10bitHighShift(i)) & (0x3ff >> get10bitHighShift(i));
            qspPayload[bitIndex + 1] |= (channelValue10 << get10bitLowShift(i)) & 0xff << (8 - get10bitHighShift(i));
        } else if (i == 4 || i == 5) {
            /*
             * Next 2 with 8 bits
             */ 
            qspPayload[i + 1] |= channelValue8;
        } else if (i == 6) {
            /*
             * And last 4 with 4 bits per channel
             */
            qspPayload[7] |= (channelValue4 << 4) & B11110000; 
        } else if (i == 7) {
            qspPayload[7] |= channelValue4 & B00001111;
        } else if (i == 8) {
            qspPayload[8] |= (channelValue4 << 4) & B11110000;
        } else if (i == 9) {
            qspPayload[8] |= channelValue4 & B00001111;
        }
    }

    qspPayloadLength = 9;
}

#endif

void loop(void) {

    bool transmitPayload = false;

#ifdef DEVICE_MODE_TX

    uint32_t currentMillis = millis();

    //TODO It should be only possible to transmit when radio is not receiveing 
    /*
     * RC_DATA QSP frame
     */
    if (currentMillis - lastRcFrameTransmit > TX_RC_FRAME_RATE && !transmitPayload && protocolState == IDLE) {
        lastRcFrameTransmit = currentMillis; 

        qspClearPayload();
        encodeRcDataPayload(&ppmReader, PPM_CHANNEL_COUNT);
        qspFrameToSend = QSP_FRAME_RC_DATA;

        transmitPayload = true;
    }

#endif

#ifdef LORA_HARDWARE_SERIAL 
    if (Serial.available()) {
        qspDecodeIncomingFrame(Serial.read());
    }
#endif

    if (transmitPayload) {
        transmitPayload = false;

        radioPacketStart();
        qspEncodeFrame(qspFrameToSend, qspPayloadLength, qspPayload);
        radioPacketEnd();
    }

}

#ifdef LORA_HARDWARE_SPI
void onReceive(int packetSize) {
    if (packetSize == 0) return;

    while (LoRa.available()) {
        qspDecodeIncomingFrame(LoRa.read());
    }
}
#endif