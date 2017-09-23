// #define DEVICE_MODE_TX
#define DEVICE_MODE_RX

#ifdef DEVICE_MODE_TX

/*
 * Main defines for device working in TX mode
 */

#include <PPMReader.h>

#define PPM_INPUT_PIN       2
#define PPM_INPUT_INTERRUPT 0

PPMReader ppmReader(PPM_INPUT_PIN, PPM_INPUT_INTERRUPT);

#endif

#ifdef DEVICE_MODE_RX

#include <Adafruit_SSD1306.h>

#define OLED_RESET 4
Adafruit_SSD1306 display(OLED_RESET);

#endif

static uint8_t protocolState = IDLE;
static uint8_t packetId = 0;
static uint8_t qspCrc = 0;
static uint8_t qspPayload[QSP_PAYLOAD_LENGTH] = {0};

void writeToRadio(uint8_t dataByte) {
    //Compute CRC
    qspCrc ^= dataByte;

    //Write to radio
    Serial.write(dataByte);
}

void decodeIncomingQspFrame(uint8_t incomingByte) {

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
            display.clearDisplay();
            display.setCursor(0, 0);
            display.print(qspPayload[0]); 
        
            display.setCursor(0, 10);
            display.print(qspPayload[1]); 
        
            display.setCursor(0, 20);
            display.print(qspPayload[2]); 
        
            display.setCursor(0, 30);
            display.print(qspPayload[3]); 
        
            display.display(); 
        } else {

            display.clearDisplay();
            display.setCursor(0, 0);
            display.print("CRC failed"); 
            display.display(); 

        }

        protocolState = IDLE;
    }

}

uint8_t getPacketId() {
    return packetId++;
}

void encodeQspFrame(uint8_t frameId, uint8_t length, uint8_t *payload) {
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
    writeToRadio(getPacketId());

    //Write payload
    for (uint8_t i = 0; i < length; i++) {
        writeToRadio(payload[i]);
    }

    //Finally write CRC
    writeToRadio(qspCrc);
}

void setup(void) {
    Serial.begin(UART_SPEED);

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

void loop(void) {

#ifdef DEVICE_MODE_TX
    uint8_t pp[4] = {0x41, 0x42, 0x43, 0x44};

    encodeQspFrame(0x01, 0x04, pp);

    Serial.end();
    delay(30);
    Serial.begin(UART_SPEED);
    delay(1000);
#endif

#ifdef DEVICE_MODE_RX
    
    if (Serial.available()) {
        decodeIncomingQspFrame(Serial.read());
    }

#endif
}