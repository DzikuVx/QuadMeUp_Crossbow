#include <LoRa.h>
#include "variables.h"
#include "qsp.h"

// #define LORA_HARDWARE_SERIAL
#define LORA_HARDWARE_SPI

#define DEVICE_MODE_TX
// #define DEVICE_MODE_RX

int ppm[PPM_CHANNEL_COUNT] = {0};

/*
 * Main defines for device working in TX mode
 */
#ifdef DEVICE_MODE_TX

#include <PPMReader.h>
PPMReader ppmReader(PPM_INPUT_PIN, PPM_INPUT_INTERRUPT);

bool canTransmit = true;

#endif

/*
 * Main defines for device working in RX mode
 */
#ifdef DEVICE_MODE_RX

#include <Adafruit_SSD1306.h>

#define OLED_RESET 4
Adafruit_SSD1306 display(OLED_RESET);

bool canTransmit = false;

#endif

/*
 * Start of QSP protocol implementation
 */
static uint8_t protocolState = IDLE;
static uint8_t qspCrc = 0;
static uint8_t qspPayload[QSP_PAYLOAD_LENGTH] = {0};
static uint8_t qspPayloadLength = 0;
static uint8_t qspFrameToSend = 0;

uint8_t qspGetPacketId()
{
    static uint8_t packetId = 0;

    return packetId++;
}

void qspClearPayload()
{
    for (uint8_t i = 0; i < QSP_PAYLOAD_LENGTH; i++)
    {
        qspPayload[i] = 0;
    }
    qspPayloadLength = 0;
}

void qspDecodeIncomingFrame(uint8_t incomingByte)
{
    static uint8_t frameId;
    static uint8_t payloadLength;
    static uint8_t receivedPayload;
    static uint8_t packetId; //TODO move this to global scope maybe?

    if (protocolState == IDLE && incomingByte == QSP_PREAMBLE)
    {
        //If in IDLE and correct preamble comes, start to decode frame
        protocolState = PREAMBLE_RECEIVED;
        qspCrc = 0 ^ incomingByte;
    }
    else if (protocolState == PREAMBLE_RECEIVED)
    {
        // Check if incomming channel ID is the same as receiver
        if (incomingByte == CHANNEL_ID)
        {
            protocolState = CHANNEL_RECEIVED;
            qspCrc ^= incomingByte;

            for (uint8_t i = 0; i < QSP_PAYLOAD_LENGTH; i++)
            {
                qspPayload[i] = 0x00;
            }

            receivedPayload = 0;
            packetId = 0;
        }
        else
        {
            protocolState = IDLE;
        }
    }
    else if (protocolState == CHANNEL_RECEIVED)
    {
        //Frame ID and payload length
        qspCrc ^= incomingByte;

        frameId = (incomingByte >> 4) & 0x0f;
        payloadLength = incomingByte & 0x0f;

        protocolState = FRAME_TYPE_RECEIVED;
    }
    else if (protocolState == FRAME_TYPE_RECEIVED)
    {
        qspCrc ^= incomingByte;
        packetId = incomingByte;
        protocolState = PACKET_ID_RECEIVED;
    }
    else if (protocolState == PACKET_ID_RECEIVED)
    {

        //Now it's time for payload
        qspCrc ^= incomingByte;
        qspPayload[receivedPayload] = incomingByte;

        receivedPayload++;

        if (receivedPayload == payloadLength)
        {
            protocolState = PAYLOAD_RECEIVED;
        }
    }
    else if (protocolState == PAYLOAD_RECEIVED)
    {

        if (qspCrc == incomingByte)
        {
//CRC is correct

#ifdef DEVICE_MODE_RX
            //If devide received a valid frame, that means it can start to talk
            canTransmit = true;
#endif

            switch (frameId)
            {
            case QSP_FRAME_RC_DATA:
                qspDecodeRcDataFrame(qspPayload, ppm);
                break;

            default:
                //Unknown frame
                //TODO do something in this case
                break;
            }
        }
        else
        {
            //CRC failed, frame has to be rejected
            //TODO do something in this case or something
        }

        // In both cases switch to listening for next preamble
        protocolState = IDLE;
    }
}

void qspEncodeFrame(uint8_t frameId, uint8_t length, uint8_t payload[])
{
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
    for (uint8_t i = 0; i < length; i++)
    {
        writeToRadio(payload[i]);
    }

    //Finally write CRC
    writeToRadio(qspCrc);
}

/*
 * End of QSP protocol implementation
 */

static uint32_t lastRcFrameTransmit = 0;

/*
 * Serial port used to send data
 */
#ifdef LORA_HARDWARE_SERIAL

int getRadioRssi(void)
{
    return 0;
}

float getRadioSnr(void)
{
    return 0;
}

void radioPacketStart(void)
{
}

void radioPacketEnd(void)
{
    Serial.end();
    delay(E45_TTL_100_UART_DOWNTIME);
    Serial.begin(UART_SPEED);
}

void writeToRadio(uint8_t dataByte)
{
    //Compute CRC
    qspComputeCrc(&qspCrc, dataByte);

    //Write to radio
    Serial.write(dataByte);
}

#endif

#ifdef LORA_HARDWARE_SPI

int getRadioRssi(void)
{
    return LoRa.packetRssi();
}

float getRadioSnr(void)
{
    return LoRa.packetSnr();
}

void radioPacketStart(void)
{
    LoRa.beginPacket();
}

void radioPacketEnd(void)
{
    LoRa.endPacket();
}

void writeToRadio(uint8_t dataByte)
{
    //Compute CRC
    qspComputeCrc(&qspCrc, dataByte);

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

void setup(void)
{

#ifdef LORA_HARDWARE_SERIAL
    Serial.begin(UART_SPEED);
#endif

#ifdef LORA_HARDWARE_SPI
    if (!LoRa.begin(868E6))
    {
        Serial.println("LoRa init failed. Check your connections.");
        while (true)
            ;
    }
    LoRa.onReceive(onReceive);
    LoRa.receive();
#endif

#ifdef DEVICE_MODE_RX
    pinMode(PIN_LED, OUTPUT);

    /*
     * Initialize OLED display
     */
    display.begin(SSD1306_SWITCHCAPVCC, 0x3C); // initialize with the I2C addr 0x3C (for the 128x32)
    display.setTextSize(1);
    display.setTextColor(WHITE);
    display.clearDisplay();
    display.display();

    //initiallize default ppm values
    for (int i = 0; i < PPM_CHANNEL_COUNT; i++)
    {
        ppm[i] = PPM_CHANNEL_DEFAULT_VALUE;
    }

    pinMode(PPM_OUTPUT_PIN, OUTPUT);
    digitalWrite(PPM_OUTPUT_PIN, !PPM_SIGNAL_POSITIVE_STATE); //set the PPM signal pin to the default state (off)

    cli();
    TCCR1A = 0; // set entire TCCR1 register to 0
    TCCR1B = 0;

    OCR1A = 100;             // compare match register, change this
    TCCR1B |= (1 << WGM12);  // turn on CTC mode
    TCCR1B |= (1 << CS11);   // 8 prescaler: 0,5 microseconds at 16mhz
    TIMSK1 |= (1 << OCIE1A); // enable timer compare interrupt
    sei();

#endif
}

#ifdef DEVICE_MODE_RX

ISR(TIMER1_COMPA_vect)
{ //leave this alone
    static boolean state = true;

    TCNT1 = 0;

    if (state)
    { //start pulse
        digitalWrite(PPM_OUTPUT_PIN, PPM_SIGNAL_POSITIVE_STATE);
        OCR1A = PPM_PULSE_LENGTH * 2;
        state = false;
    }
    else
    { //end pulse and calculate when to start the next pulse
        static byte cur_chan_numb;
        static unsigned int calc_rest;

        digitalWrite(PPM_OUTPUT_PIN, !PPM_SIGNAL_POSITIVE_STATE);
        state = true;

        if (cur_chan_numb >= PPM_CHANNEL_COUNT)
        {
            cur_chan_numb = 0;
            calc_rest = calc_rest + PPM_PULSE_LENGTH; //
            OCR1A = (PPM_FRAME_LENGTH - calc_rest) * 2;
            calc_rest = 0;
        }
        else
        {
            OCR1A = (ppm[cur_chan_numb] - PPM_PULSE_LENGTH) * 2;
            calc_rest = calc_rest + ppm[cur_chan_numb];
            cur_chan_numb++;
        }
    }
}

#endif

void loop(void)
{

    bool transmitPayload = false;

#ifdef DEVICE_MODE_TX

    uint32_t currentMillis = millis();

    //TODO It should be only possible to transmit when radio is not receiveing
    /*
     * RC_DATA QSP frame
     */
    if (currentMillis - lastRcFrameTransmit > TX_RC_FRAME_RATE && !transmitPayload && protocolState == IDLE)
    {
        lastRcFrameTransmit = currentMillis;

        qspClearPayload();
        encodeRcDataPayload(&ppmReader, PPM_CHANNEL_COUNT, qspPayload, &qspPayloadLength);
        qspFrameToSend = QSP_FRAME_RC_DATA;

        transmitPayload = true;
    }

#endif

#ifdef LORA_HARDWARE_SERIAL
    if (Serial.available())
    {
        qspDecodeIncomingFrame(Serial.read());
    }
#endif

    if (canTransmit && transmitPayload)
    {
        transmitPayload = false;

        radioPacketStart();
        qspEncodeFrame(qspFrameToSend, qspPayloadLength, qspPayload);
        radioPacketEnd();
    }
}

#ifdef LORA_HARDWARE_SPI
void onReceive(int packetSize)
{
    if (packetSize == 0)
        return;

    while (LoRa.available())
    {
        qspDecodeIncomingFrame(LoRa.read());
    }
}
#endif