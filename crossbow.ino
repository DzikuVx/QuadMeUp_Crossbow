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
QspConfiguration_t qsp = {};

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

void writeToRadio(uint8_t dataByte, QspConfiguration_t *qsp)
{
    //Compute CRC
    qspComputeCrc(qsp, dataByte);

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

void writeToRadio(uint8_t dataByte, QspConfiguration_t *qsp)
{
    //Compute CRC
    qspComputeCrc(qsp, dataByte);

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
    qsp.hardwareWriteFunction = writeToRadio;

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
    if (currentMillis - lastRcFrameTransmit > TX_RC_FRAME_RATE && !transmitPayload && qsp.protocolState == QSP_STATE_IDLE)
    {
        lastRcFrameTransmit = currentMillis;

        qspClearPayload(&qsp);
        encodeRcDataPayload(&qsp, &ppmReader, PPM_CHANNEL_COUNT);
        qsp.frameToSend = QSP_FRAME_RC_DATA;

        transmitPayload = true;
    }

#endif

#ifdef LORA_HARDWARE_SERIAL
    if (Serial.available())
    {
        qspDecodeIncomingFrame(&qsp, Serial.read(), ppm);
    }
#endif

    if (canTransmit && transmitPayload)
    {
        transmitPayload = false;

        radioPacketStart();
        qspEncodeFrame(&qsp);
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
        qspDecodeIncomingFrame(&qsp, LoRa.read(), ppm);
    }
}
#endif