// #define LORA_HARDWARE_SERIAL
#define LORA_HARDWARE_SPI

#define DEVICE_MODE_TX
// #define DEVICE_MODE_RX

#define DEBUG_SERIAL
// #define DEBUG_PING_PONG
// #define DEBUG_LED
// #define WAIT_FOR_SERIAL

#include <LoRa.h>
// #include <PinChangeInterrupt.h>
#include "variables.h"
#include "qsp.h"

// LoRa32u4 ports
#define LORA32U4_SS_PIN     8
#define LORA32U4_RST_PIN    4
#define LORA32U4_DI0_PIN    7

int ppm[PPM_CHANNEL_COUNT] = {0};

/*
 * Main defines for device working in TX mode
 */
#ifdef DEVICE_MODE_TX

// #define OLED_RESET -1
#include <PPMReader.h>
// #include <Adafruit_SSD1306.h>

PPMReader ppmReader(PPM_INPUT_PIN, PPM_INPUT_INTERRUPT);
// PPMReader ppmReader(11, 2, MODE_PIN_CHANGE_INTERRUPT);
// Adafruit_SSD1306 display(OLED_RESET);

#endif

/*
 * Main defines for device working in RX mode
 */
#ifdef DEVICE_MODE_RX

#endif

/*
 * Start of QSP protocol implementation
 */
QspConfiguration_t qsp = {};
RxDeviceState_t rxDeviceState = {};

/*
 * End of QSP protocol implementation
 */

/*
 * Serial port used to send data
 */
#ifdef LORA_HARDWARE_SERIAL

unint8_t getRadioRssi(void)
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

uint8_t getRadioRssi(void)
{
    //Map from -164:0 to 0:100
    return map(constrain(LoRa.packetRssi() * -1, 0, 164), 0, 164, 100, 0);
}

float getRadioSnr(void)
{
    return (uint8_t) constrain(LoRa.packetSnr() * 10, 0, 255);
}

void radioPacketStart(void)
{
    LoRa.beginPacket();
}

void radioPacketEnd(void)
{
    LoRa.endPacket();
    //After ending packet, put device into receive mode again
    LoRa.receive();
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

#ifdef DEVICE_MODE_RX
    qsp.deviceState = DEVICE_STATE_FAILSAFE;
#else 
    qsp.deviceState = DEVICE_STATE_OK;
#endif

#ifdef LORA_HARDWARE_SERIAL
    Serial.begin(UART_SPEED);
#endif

#ifdef LORA_HARDWARE_SPI
    Serial.begin(115200);

#ifdef WAIT_FOR_SERIAL
    while (!Serial) {
        ; // wait for serial port to connect. Needed for native USB
    }
#endif

#ifdef DEBUG_SERIAL
    Serial.println("Start");
#endif

    /*
     * Setup hardware
     */
    LoRa.setPins(
        LORA32U4_SS_PIN,
        LORA32U4_RST_PIN,
        LORA32U4_DI0_PIN
    );

#ifdef DEBUG_SERIAL 
    Serial.println("Pins Set");
#endif
    
    if (!LoRa.begin(868E6))
    {
    #ifdef DEBUG_SERIAL
        Serial.println("LoRa init failed. Check your connections.");
    #endif
        while (true);
    }

    LoRa.setSignalBandwidth(250E3);
    LoRa.setSpreadingFactor(7);
    LoRa.setCodingRate4(5);

#ifdef DEBUG_SERIAL
    Serial.println("Init done");
#endif
    LoRa.onReceive(onReceive);
    LoRa.receive();
#ifdef DEBUG_SERIAL
    Serial.println("Receive mode enabled");
#endif
#endif

#ifdef DEVICE_MODE_RX
    /*
     * Initialize OLED display
     */
    // display.begin(SSD1306_SWITCHCAPVCC, 0x3C); // initialize with the I2C addr 0x3C (for the 128x32)
    // display.setTextSize(1);
    // display.setTextColor(WHITE);
    // display.clearDisplay();
    // display.display();

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

    pinMode(RX_ADC_PIN_1, INPUT);
    pinMode(RX_ADC_PIN_2, INPUT);
    pinMode(RX_ADC_PIN_3, INPUT);

#endif

    pinMode(LED_BUILTIN, OUTPUT);

/*
 * TX should start talking imediately after power up
 */
#ifdef DEVICE_MODE_TX
    qsp.canTransmit = true;
#endif

#ifdef DEBUG_SERIAL
    qsp.debugConfig |= DEBUG_FLAG_SERIAL;
#endif
#ifdef DEBUG_LED
    qsp.debugConfig |= DEBUG_FLAG_LED;
#endif
}

#ifdef DEVICE_MODE_RX

void writePpmOutput(uint8_t val) {
    if (qsp.deviceState == DEVICE_STATE_OK) {
        digitalWrite(PPM_OUTPUT_PIN, val);
    } else {
        //This is failsafe state, we pull output low so FC can decide about failsafe 
        digitalWrite(PPM_OUTPUT_PIN, LOW);
    }
}

ISR(TIMER1_COMPA_vect) { //leave this alone
    static boolean state = true;

    TCNT1 = 0;

    if (state)
    { //start pulse
        writePpmOutput(PPM_SIGNAL_POSITIVE_STATE);
        OCR1A = PPM_PULSE_LENGTH * 2;
        state = false;
    }
    else
    { //end pulse and calculate when to start the next pulse
        static byte cur_chan_numb;
        static unsigned int calc_rest;

        writePpmOutput(!PPM_SIGNAL_POSITIVE_STATE);
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

void updateRxDeviceState(RxDeviceState_t *rxDeviceState) {
    rxDeviceState->rxVoltage = map(analogRead(RX_ADC_PIN_1), 0, 1024, 0, 255);
    rxDeviceState->a1Voltage = map(analogRead(RX_ADC_PIN_2), 0, 1024, 0, 255);
    rxDeviceState->a2Voltage = map(analogRead(RX_ADC_PIN_3), 0, 1024, 0, 255);
    rxDeviceState->rssi = getRadioRssi();
    rxDeviceState->snr = getRadioSnr();
}    

#endif

void loop(void)
{
    uint32_t currentMillis = millis();
    bool transmitPayload = false;

    /*
     * Watchdog for frame decoding stuck somewhere in the middle of a process
     */
    if (
        qsp.protocolState != QSP_STATE_IDLE && 
        abs(millis() - qsp.frameDecodingStartedAt) > QSP_MAX_FRAME_DECODE_TIME 
    ) {
        qsp.protocolState = QSP_STATE_IDLE;
    }

    if (
        qsp.forcePongFrame && 
        !transmitPayload && 
        qsp.protocolState == QSP_STATE_IDLE
    )
    {
        qsp.forcePongFrame = false;
        qsp.lastFrameTransmitedAt[QSP_FRAME_PONG] = currentMillis;
        qsp.frameToSend = QSP_FRAME_PONG;
        transmitPayload = true;
    }


#ifdef DEVICE_MODE_TX

#ifdef DEBUG_PING_PONG
    //PING frame
    if (
        currentMillis - qsp.lastFrameTransmitedAt[QSP_FRAME_PING] > TX_PING_RATE && 
        !transmitPayload && 
        qsp.protocolState == QSP_STATE_IDLE
    )
    {
        qsp.lastFrameTransmitedAt[QSP_FRAME_PING] = currentMillis;

        qspClearPayload(&qsp);
        encodePingPayload(&qsp, micros());
        qsp.frameToSend = QSP_FRAME_PING;
        
        transmitPayload = true;
    }
#endif

    /*
     * RC_DATA QSP frame
     */
    if (
        currentMillis - qsp.lastFrameTransmitedAt[QSP_FRAME_RC_DATA] > TX_RC_FRAME_RATE && 
        !transmitPayload && 
        qsp.protocolState == QSP_STATE_IDLE
    )
    {
        qsp.lastFrameTransmitedAt[QSP_FRAME_RC_DATA] = currentMillis;

        qspClearPayload(&qsp);
        encodeRcDataPayload(&qsp, &ppmReader, PPM_CHANNEL_COUNT);
        qsp.frameToSend = QSP_FRAME_RC_DATA;

        transmitPayload = true;
    }

#endif

#ifdef DEVICE_MODE_RX
    /*
     * RX_HEALTH QSP frame
     */
    if (
        currentMillis - qsp.lastFrameTransmitedAt[QSP_FRAME_RX_HEALTH] > RX_RX_HEALTH_FRAME_RATE && 
        !transmitPayload && 
        qsp.protocolState == QSP_STATE_IDLE
    )
    {
        qsp.lastFrameTransmitedAt[QSP_FRAME_RX_HEALTH] = currentMillis;

        updateRxDeviceState(&rxDeviceState);

        qspClearPayload(&qsp);
        encodeRxHealthPayload(&qsp, &rxDeviceState);
        qsp.frameToSend = QSP_FRAME_RX_HEALTH;

        transmitPayload = true;
    }

#endif

#ifdef LORA_HARDWARE_SERIAL
    if (Serial.available())
    {
        qspDecodeIncomingFrame(&qsp, Serial.read(), ppm);
    }
#endif

    if (qsp.canTransmit && transmitPayload)
    {
        radioPacketStart();
        qspEncodeFrame(&qsp);
        radioPacketEnd();

    #ifdef DEBUG_SERIAL
        Serial.print("Frame ");
        Serial.print(qsp.frameToSend);
        Serial.println(" sent");
    #endif

    #ifdef DEBUG_LED
        digitalWrite(LED_BUILTIN, HIGH);
        delay(10);
        digitalWrite(LED_BUILTIN, LOW);
        delay(70);
        digitalWrite(LED_BUILTIN, HIGH);
        delay(10);
        digitalWrite(LED_BUILTIN, LOW);
    #endif
        transmitPayload = false;
    }

    /*
     * Here we do state handling and similar operations 
     */
#ifdef DEVICE_MODE_RX
    if (abs(currentMillis - qsp.lastFrameReceivedAt[QSP_FRAME_RC_DATA]) > RX_FAILSAFE_DELAY) {
        qsp.deviceState = DEVICE_STATE_FAILSAFE;
    } else {
        qsp.deviceState = DEVICE_STATE_OK;
    }
#endif
}

#ifdef LORA_HARDWARE_SPI
void onReceive(int packetSize)
{
    if (packetSize == 0)
        return;

    while (LoRa.available())
    {
        qspDecodeIncomingFrame(&qsp, LoRa.read(), ppm, &rxDeviceState);
    }
}
#endif