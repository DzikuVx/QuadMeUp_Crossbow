// #define DEVICE_MODE_TX
#define DEVICE_MODE_RX

#define FEATURE_TX_OLED

// #define DEBUG_SERIAL
// #define DEBUG_PING_PONG
// #define DEBUG_LED
// #define WAIT_FOR_SERIAL

#include <LoRa.h>
#include "variables.h"
#include "qsp.h"

volatile int ppm[16] = {0};

// LoRa32u4 ports
#define LORA32U4_SS_PIN     8
#define LORA32U4_RST_PIN    4
#define LORA32U4_DI0_PIN    7

/*
 * Main defines for device working in TX mode
 */
#ifdef DEVICE_MODE_TX
#include <PPMReader.h>
PPMReader ppmReader(PPM_INPUT_PIN, PPM_INPUT_INTERRUPT, true);

#include "txbuzzer.h"

BuzzerState_t buzzer;

#ifdef FEATURE_TX_OLED

#define OLED_RESET -1
#include <Adafruit_SSD1306.h>
Adafruit_SSD1306 display(OLED_RESET);
uint32_t lastOledTaskTime = 0;

#endif

#endif

/*
 * Main defines for device working in RX mode
 */
#ifdef DEVICE_MODE_RX

    #include "sbus.h"

    uint32_t sbusTime = 0;
    uint8_t sbusPacket[SBUS_PACKET_LENGTH] = {0};
    uint32_t lastRxStateTaskTime = 0;
#endif

/*
 * Start of QSP protocol implementation
 */
volatile QspConfiguration_t qsp = {};
volatile RxDeviceState_t rxDeviceState = {};
volatile TxDeviceState_t txDeviceState = {};


uint8_t getRadioRssi(void)
{
    //Map from -164:0 to 0:255
    return map(constrain(LoRa.packetRssi() * -1, 0, 164), 0, 164, 255, 0);
}

uint8_t getRadioSnr(void)
{
    return (uint8_t) constrain(LoRa.packetSnr(), 0, 255);
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

void setup(void)
{
#ifdef DEBUG_SERIAL
    Serial.begin(115200);
#endif

    qsp.hardwareWriteFunction = writeToRadio;

#ifdef DEVICE_MODE_RX
    qsp.deviceState = DEVICE_STATE_FAILSAFE;
#else 
    qsp.deviceState = DEVICE_STATE_OK;
#endif

#ifdef WAIT_FOR_SERIAL
    while (!Serial) {
        ; // wait for serial port to connect. Needed for native USB
    }
#endif

    /*
     * Setup hardware
     */
    LoRa.setPins(
        LORA32U4_SS_PIN,
        LORA32U4_RST_PIN,
        LORA32U4_DI0_PIN
    );
    
    if (!LoRa.begin(868E6))
    {
    #ifdef DEBUG_SERIAL
        Serial.println("LoRa init failed. Check your connections.");
    #endif
        while (true);
    }

    LoRa.setSignalBandwidth(500E3);
    LoRa.setSpreadingFactor(7);
    LoRa.setCodingRate4(5);

    /*
     * Use interrupt driven approach only on RX side
     * TX interrupts breaks PPM readout
     */ 
// #ifdef DEVICE_MODE_RX
    LoRa.onReceive(onReceive);
// #endif
    LoRa.receive();

#ifdef DEVICE_MODE_RX
    //initiallize default ppm values
    for (int i = 0; i < 16; i++)
    {
        ppm[i] = PPM_CHANNEL_DEFAULT_VALUE;
    }

    pinMode(RX_ADC_PIN_1, INPUT);
    pinMode(RX_ADC_PIN_2, INPUT);
    pinMode(RX_ADC_PIN_3, INPUT);

    Serial1.begin(100000, SERIAL_8E2);
#endif

#ifdef DEVICE_MODE_TX
    TCCR1A = 0;  //reset timer1
    TCCR1B = 0;
    TCCR1B |= (1 << CS11);  //set timer1 to increment every 0,5 us or 1us on 8MHz

    pinMode(TX_BUZZER_PIN, OUTPUT);

#ifdef FEATURE_TX_OLED
    display.begin(SSD1306_SWITCHCAPVCC, 0x3C); // initialize with the I2C addr 0x3C (for the 128x32)
    display.setTextSize(1);
    display.setTextColor(WHITE);
    display.clearDisplay();
    display.display();
#endif

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

int8_t txSendSequence[16] = {
    QSP_FRAME_PING,
    QSP_FRAME_RC_DATA,
    QSP_FRAME_RC_DATA,
    QSP_FRAME_RC_DATA,
    QSP_FRAME_RC_DATA,
    QSP_FRAME_RC_DATA,
    QSP_FRAME_RC_DATA,
    QSP_FRAME_RC_DATA,
    QSP_FRAME_RC_DATA,
    QSP_FRAME_RC_DATA,
    QSP_FRAME_RC_DATA,
    QSP_FRAME_RC_DATA,
    QSP_FRAME_RC_DATA,
    QSP_FRAME_RC_DATA,
    QSP_FRAME_RC_DATA,
    QSP_FRAME_RC_DATA
};

int8_t rxSendSequence[16] = {
    QSP_FRAME_RX_HEALTH,
    -1,
    -1,
    -1,
    QSP_FRAME_RX_HEALTH,
    -1,
    -1,
    -1,
    QSP_FRAME_RX_HEALTH,
    -1,
    -1,
    -1,
    QSP_FRAME_RX_HEALTH,
    -1,
    -1,
    -1
};

uint8_t currentSequenceIndex = 0;
#define TRANSMIT_SEQUENCE_COUNT 16

#ifdef DEVICE_MODE_RX

void updateRxDeviceState(RxDeviceState_t *rxDeviceState) {
    rxDeviceState->rxVoltage = map(analogRead(RX_ADC_PIN_1), 0, 1024, 0, 255);
    rxDeviceState->a1Voltage = map(analogRead(RX_ADC_PIN_2), 0, 1024, 0, 255);
    rxDeviceState->a2Voltage = map(analogRead(RX_ADC_PIN_3), 0, 1024, 0, 255);
}    

int8_t getFrameToTransmit(QspConfiguration_t *qsp) {
    
    if (qsp->forcePongFrame) {
        qsp->forcePongFrame = false;
        return QSP_FRAME_PONG;
    }

    int8_t retVal = rxSendSequence[currentSequenceIndex];
    
    currentSequenceIndex++;
    if (currentSequenceIndex >= TRANSMIT_SEQUENCE_COUNT) {
        currentSequenceIndex = 0;
    }

    return retVal;
}

#endif

#ifdef DEVICE_MODE_TX
int8_t getFrameToTransmit(QspConfiguration_t *qsp) {
    int8_t retVal = txSendSequence[currentSequenceIndex];

    currentSequenceIndex++;
    if (currentSequenceIndex >= TRANSMIT_SEQUENCE_COUNT) {
        currentSequenceIndex = 0;
    }

    return retVal;
}
#endif

void loop(void)
{
#ifdef DEVICE_MODE_TX
    if (txDeviceState.readPacket) {
        if (LoRa.available()) {
            qspDecodeIncomingFrame(&qsp, LoRa.read(), ppm, &rxDeviceState);
        } else {
            txDeviceState.rssi = getRadioRssi();
            txDeviceState.snr = getRadioSnr();
            txDeviceState.readPacket = false;
        }
    }
#endif

    uint32_t currentMillis = millis();
    bool transmitPayload = false;

    /*
     * Watchdog for frame decoding stuck somewhere in the middle of a process
     */
    if (
        qsp.protocolState != QSP_STATE_IDLE && 
        abs(currentMillis - qsp.frameDecodingStartedAt) > QSP_MAX_FRAME_DECODE_TIME 
    ) {
        qsp.protocolState = QSP_STATE_IDLE;
    }

#ifdef DEVICE_MODE_TX

    if (
        abs(currentMillis - qsp.lastTxSlotTimestamp) > TX_TRANSMIT_SLOT_RATE &&
        qsp.protocolState == QSP_STATE_IDLE
    ) {
        
        int8_t frameToSend = getFrameToTransmit(&qsp);

        if (frameToSend == QSP_FRAME_RC_DATA && !ppmReader.isReceiving()) {
            frameToSend = -1;
        }

        if (frameToSend > -1) {

            qsp.frameToSend = frameToSend;
            qspClearPayload(&qsp);

            switch (qsp.frameToSend) {
                case QSP_FRAME_PING:
                    encodePingPayload(&qsp, micros());
                    break;

                case QSP_FRAME_RC_DATA:
                    encodeRcDataPayload(&qsp, &ppmReader, PPM_INPUT_CHANNEL_COUNT);
                    break;
            }

            transmitPayload = true;
        }

        qsp.lastTxSlotTimestamp = currentMillis;
    }

#endif

#ifdef DEVICE_MODE_RX

    /*
     * This routine updates RX device state and updates one of radio channels with RSSI value
     */
    if (abs(currentMillis - lastRxStateTaskTime) > RX_TASK_HEALTH) {
        lastRxStateTaskTime = currentMillis;
        updateRxDeviceState(&rxDeviceState);
        ppm[RSSI_CHANNEL - 1] = map(rxDeviceState.rssi, 0, 255, 1000, 2000);
        if (qsp.deviceState == DEVICE_STATE_FAILSAFE) {
            digitalWrite(LED_BUILTIN, HIGH);
        } else {
            digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
        }
    }
    
    /*
     * Main routine to answer to TX module
     */
    if (qsp.transmitWindowOpen && qsp.protocolState == QSP_STATE_IDLE) {
        qsp.transmitWindowOpen = false;

        int8_t frameToSend = getFrameToTransmit(&qsp);
        if (frameToSend > -1) {
            qsp.frameToSend = frameToSend;

            if (frameToSend != QSP_FRAME_PONG) {
                qspClearPayload(&qsp);
            }
            switch (qsp.frameToSend) {
                case QSP_FRAME_PONG:
                    /*
                     * Pong frame just responses with received payload
                     */
                    break;

                case QSP_FRAME_RX_HEALTH:
                    encodeRxHealthPayload(&qsp, &rxDeviceState);
                    break;
            }

            transmitPayload = true;
        }

    }

    if (currentMillis > sbusTime) {
        sbusPreparePacket(sbusPacket, ppm, false, (qsp.deviceState == DEVICE_STATE_FAILSAFE));
        Serial1.write(sbusPacket, SBUS_PACKET_LENGTH);
        sbusTime = currentMillis + SBUS_UPDATE_RATE;
    }

    if (abs(currentMillis - qsp.lastFrameReceivedAt[QSP_FRAME_RC_DATA]) > RX_FAILSAFE_DELAY) {
        qsp.deviceState = DEVICE_STATE_FAILSAFE;
    } else {
        qsp.deviceState = DEVICE_STATE_OK;
    }

#endif

#ifdef DEVICE_MODE_TX

    buzzerProcess(TX_BUZZER_PIN, currentMillis, &buzzer);

#ifdef FEATURE_TX_OLED
    if (
        currentMillis - lastOledTaskTime > OLED_UPDATE_RATE
    ) {
        lastOledTaskTime = currentMillis;

        display.clearDisplay();

        display.setTextColor(WHITE, BLACK);
        display.setCursor(0, 0);
        display.setTextSize(3);
        display.print(txDeviceState.rssi); 

        display.setCursor(18, 28);
        display.setTextSize(2);
        display.print(txDeviceState.snr);
        
        display.setCursor(74, 0);
        display.setTextSize(3);
        display.print(rxDeviceState.rssi); 

        display.setCursor(92, 28);
        display.setTextSize(2);
        display.print(rxDeviceState.snr); 

        display.setCursor(54, 48);
        display.setTextSize(2);
        display.print(rxDeviceState.roundtrip); 

        display.display(); 
    }
#endif

#endif

    if (qsp.canTransmit && transmitPayload)
    {
        radioPacketStart();
        qspEncodeFrame(&qsp);
        radioPacketEnd();
        transmitPayload = false;
    }

}

void onReceive(int packetSize)
{
    if (packetSize == 0)
        return;

#ifdef DEVICE_MODE_TX
    txDeviceState.readPacket = true;
#endif

#ifdef DEVICE_MODE_RX
    while (LoRa.available())
    {
        qspDecodeIncomingFrame(&qsp, LoRa.read(), ppm, &rxDeviceState);
    }

    rxDeviceState.rssi = getRadioRssi();
    rxDeviceState.snr = getRadioSnr();
#endif
}