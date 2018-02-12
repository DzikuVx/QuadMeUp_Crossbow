/*
This Source Code Form is subject to the terms of the Mozilla Public
License, v. 2.0. If a copy of the MPL was not distributed with this file,
You can obtain one at http://mozilla.org/MPL/2.0/.

Copyright (c) 20xx, MPL Contributor1 contrib1@example.net
*/

#include "config.h"


#include "lora.h"
#include "variables.h"
#include "main_variables.h"
#include "qsp.h"
#include "sbus.h"

#ifdef ARDUINO_AVR_FEATHER32U4
 #define LORA_SS_PIN     8
 #define LORA_RST_PIN    4
 #define LORA_DI0_PIN    7
#elif defined(ARDUINO_SAMD_FEATHER_M0)
 #define LORA_SS_PIN     8
 #define LORA_RST_PIN    4
 #define LORA_DI0_PIN    3
#else
 #error please select hardware
#endif

/*
 * Main defines for device working in TX mode
 */
#ifdef DEVICE_MODE_TX

#ifdef FEATURE_TX_INPUT_PPM
  #include "ppm_reader.h"
  PPM_Reader txInput(PPM_INPUT_PIN, true);

#elif defined(FEATURE_TX_INPUT_SBUS)
  #include "sbus.h"
  SbusInput txInput(Serial1);

#else
  #error please select tx input source
#endif

volatile int16_t TxInput::channels[TX_INPUT_CHANNEL_COUNT];


#include "txbuzzer.h"

BuzzerState_t buzzer;

#ifdef FEATURE_TX_OLED
#include "Wire.h"

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
    uint32_t sbusTime = 0;
    uint8_t sbusPacket[SBUS_PACKET_LENGTH] = {0};
    uint32_t lastRxStateTaskTime = 0;
#endif

/*
 * Start of QSP protocol implementation
 */
QspConfiguration_t qsp = {};
RxDeviceState_t rxDeviceState = {};
TxDeviceState_t txDeviceState = {};
volatile RadioState_t radioState = {};

uint8_t tmpBuffer[MAX_PACKET_SIZE];

uint8_t getRadioRssi(void)
{
    return 164 - constrain(LoRa.packetRssi() * -1, 0, 164);
}

uint8_t getRadioSnr(void)
{
    return (uint8_t) constrain(LoRa.packetSnr(), 0, 255);
}

void onQspSuccess(QspConfiguration_t *qsp, TxDeviceState_t *txDeviceState, RxDeviceState_t *rxDeviceState, volatile RadioState_t *radioState) {
    //If devide received a valid frame, that means it can start to talk
    qsp->canTransmit = true;

    //Store the last timestamp when frame was received
    if (qsp->frameId < QSP_FRAME_COUNT) {
        qsp->lastFrameReceivedAt[qsp->frameId] = millis();
    }
    qsp->anyFrameRecivedAt = millis();
    switch (qsp->frameId) {
        case QSP_FRAME_RC_DATA:
            qspDecodeRcDataFrame(qsp, rxDeviceState);
            break;

        case QSP_FRAME_RX_HEALTH:
            decodeRxHealthPayload(qsp, rxDeviceState);
            break;

        case QSP_FRAME_PING:
            qsp->forcePongFrame = true;
            break;

        case QSP_FRAME_PONG:
            txDeviceState->roundtrip = qsp->payload[0];
            txDeviceState->roundtrip += (uint32_t) qsp->payload[1] << 8;
            txDeviceState->roundtrip += (uint32_t) qsp->payload[2] << 16;
            txDeviceState->roundtrip += (uint32_t) qsp->payload[3] << 24;

            txDeviceState->roundtrip = (micros() - txDeviceState->roundtrip) / 1000;
            break;

        default:
            //Unknown frame
            //TODO do something in this case
            break;
    }

    qsp->transmitWindowOpen = true;
}

void onQspFailure(QspConfiguration_t *qsp, TxDeviceState_t *txDeviceState, RxDeviceState_t *rxDeviceState, volatile RadioState_t *radioState) {

}

void setup(void)
{
#ifdef DEBUG_SERIAL
    Serial.begin(115200);
#endif

    qsp.onSuccessCallback = onQspSuccess;
    qsp.onFailureCallback = onQspFailure;

#ifdef DEVICE_MODE_RX
    qsp.deviceState = DEVICE_STATE_FAILSAFE;
#else
    qsp.deviceState = DEVICE_STATE_OK;
#endif

    /*
     * Setup hardware
     */
    LoRa.setPins(
        LORA_SS_PIN,
        LORA_RST_PIN,
        LORA_DI0_PIN
    );

    if (!LoRa.begin(radioState.frequency))
    {
    #ifdef DEBUG_SERIAL
        Serial.println("LoRa init failed. Check your connections.");
    #endif
        while (true);
    }

    //Configure LoRa module
    LoRa.setSignalBandwidth(radioState.loraBandwidth);
    LoRa.setSpreadingFactor(radioState.loraSpreadingFactor);
    LoRa.setCodingRate4(radioState.loraCodingRate);
    LoRa.setTxPower(radioState.loraTxPower);
    LoRa.enableCrc();

    //Setup ISR callback and start receiving
    LoRa.onReceive(onReceive);
    LoRa.receive();
    radioState.deviceState = RADIO_STATE_RX;

#ifdef DEVICE_MODE_RX
    //initiallize default ppm values
    for (int i = 0; i < 16; i++)
    {
        rxDeviceState.channels[i] = PPM_CHANNEL_DEFAULT_VALUE;
    }

    pinMode(RX_ADC_PIN_1, INPUT);
    pinMode(RX_ADC_PIN_2, INPUT);
    pinMode(RX_ADC_PIN_3, INPUT);

    /*
     * Prepare Serial1 for S.Bus processing
     */
    Serial1.begin(100000, SERIAL_8E2);
#endif

#ifdef DEVICE_MODE_TX

#ifdef FEATURE_TX_OLED
    Wire.setClock(400000);

    display.begin(SSD1306_SWITCHCAPVCC, 0x3C); // initialize with the I2C addr 0x3C (for the 128x32)
    display.setTextSize(1);
    display.setTextColor(WHITE);
    display.clearDisplay();
    display.display();
#endif

    /*
     * TX should start talking imediately after power up
     */
    qsp.canTransmit = true;

    pinMode(TX_BUZZER_PIN, OUTPUT);

    //Play single tune to indicate power up
    buzzerSingleMode(BUZZER_MODE_CHIRP, &buzzer);

    /*
     * Prepare Serial1 for S.Bus processing
     */
    txInput.start();
#endif

    pinMode(LED_BUILTIN, OUTPUT);

#ifdef DEBUG_SERIAL
    qsp.debugConfig |= DEBUG_FLAG_SERIAL;
#endif
#ifdef DEBUG_LED
    qsp.debugConfig |= DEBUG_FLAG_LED;
#endif

}

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

/*
 *
 * Main loop starts here!
 *
 */
void loop(void)
{

    uint32_t currentMillis = millis();

    /*
     * Detect the moment when radio module stopped transmittig and put it
     * back in to receive state
     */
    if (
        currentMillis > radioState.nextTxCheckMillis &&
        radioState.deviceState == RADIO_STATE_TX &&
        !LoRa.isTransmitting()
    ) {
        LoRa.receive();
        radioState.deviceState = RADIO_STATE_RX;
        radioState.nextTxCheckMillis = currentMillis + 1; //We check of TX done every 1ms
    }

    if (radioState.bytesToRead != NO_DATA_TO_READ) {
        LoRa.read(tmpBuffer, radioState.bytesToRead);

        for (int i = 0; i < radioState.bytesToRead; i++) {
            qspDecodeIncomingFrame(&qsp, tmpBuffer[i], &rxDeviceState, &txDeviceState, &radioState);
        }

        radioState.rssi = getRadioRssi();
        radioState.snr = getRadioSnr();

        //After reading, flush radio buffer, we have no need for whatever might be over there
        LoRa.sleep();
        LoRa.receive();
        radioState.deviceState = RADIO_STATE_RX;

        radioState.bytesToRead = NO_DATA_TO_READ;
    }

    bool transmitPayload = false;

    /*
     * Watchdog for frame decoding stuck somewhere in the middle of a process
     */
    if (
        qsp.protocolState != QSP_STATE_IDLE &&
        qsp.frameDecodingStartedAt + QSP_MAX_FRAME_DECODE_TIME < currentMillis
    ) {
        qsp.protocolState = QSP_STATE_IDLE;
    }

#ifdef DEVICE_MODE_TX

    txInput.loop();

    if (
        radioState.deviceState == RADIO_STATE_RX &&
        qsp.protocolState == QSP_STATE_IDLE &&
        qsp.lastTxSlotTimestamp + TX_TRANSMIT_SLOT_RATE < currentMillis
    ) {

        int8_t frameToSend = getFrameToTransmit(&qsp);

    #ifndef FORCE_TX_WITHOUT_INPUT
        /*
         * If module is not receiving data from radio, do not send RC DATA
         * This is the only way to trigger failsafe in that case
         */
        if (frameToSend == QSP_FRAME_RC_DATA && !txInput.isReceiving()) {
            frameToSend = -1;
        }
    #endif

        if (frameToSend > -1) {

            qsp.frameToSend = frameToSend;
            qspClearPayload(&qsp);

            switch (qsp.frameToSend) {
                case QSP_FRAME_PING:
                    encodePingPayload(&qsp, micros());
                    break;

                case QSP_FRAME_RC_DATA:
                    encodeRcDataPayload(&qsp, txInput.channels, PPM_INPUT_CHANNEL_COUNT);
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
    if (lastRxStateTaskTime + RX_TASK_HEALTH < currentMillis) {
        lastRxStateTaskTime = currentMillis;
        updateRxDeviceState(&rxDeviceState);

        uint8_t output = constrain(radioState.rssi - 40, 0, 100);

        rxDeviceState.channels[RSSI_CHANNEL - 1] = (output * 10) + 1000;
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
                    encodeRxHealthPayload(&qsp, &rxDeviceState, &radioState);
                    break;
            }

            transmitPayload = true;
        }

    }

    if (currentMillis > sbusTime) {
        sbusPreparePacket(sbusPacket, rxDeviceState.channels, false, (qsp.deviceState == DEVICE_STATE_FAILSAFE));
        Serial1.write(sbusPacket, SBUS_PACKET_LENGTH);
        sbusTime = currentMillis + SBUS_UPDATE_RATE;
    }

    if (qsp.lastFrameReceivedAt[QSP_FRAME_RC_DATA] + RX_FAILSAFE_DELAY < currentMillis) {
        qsp.deviceState = DEVICE_STATE_FAILSAFE;
    } else {
        qsp.deviceState = DEVICE_STATE_OK;
    }

#endif

    if (qsp.canTransmit && transmitPayload)
    {
        uint8_t size;
        LoRa.beginPacket();
        //Prepare packet
        qspEncodeFrame(&qsp, tmpBuffer, &size);
        //Sent it to radio in one SPI transaction
        LoRa.write(tmpBuffer, size);
        LoRa.endPacketAsync();

        //Set state to be able to detect the moment when TX is done
        radioState.deviceState = RADIO_STATE_TX;

        transmitPayload = false;
    }

#ifdef DEVICE_MODE_TX

    buzzerProcess(TX_BUZZER_PIN, currentMillis, &buzzer);

    // This routing enables when TX starts to receive signal from RX for a first time or after
    // failsafe
    if (txDeviceState.isReceiving == false && qsp.anyFrameRecivedAt != 0) {
        //TX module started to receive data
        buzzerSingleMode(BUZZER_MODE_DOUBLE_CHIRP, &buzzer);
        txDeviceState.isReceiving = true;
        qsp.deviceState = DEVICE_STATE_OK;
    }

    //Here we detect failsafe state on TX module
    if (
        txDeviceState.isReceiving &&
        qsp.anyFrameRecivedAt + TX_FAILSAFE_DELAY < currentMillis
    ) {
        txDeviceState.isReceiving = false;
        rxDeviceState.a1Voltage = 0;
        rxDeviceState.a2Voltage = 0;
        rxDeviceState.rxVoltage = 0;
        rxDeviceState.rssi = 0;
        rxDeviceState.snr = 0;
        rxDeviceState.flags = 0;
        txDeviceState.roundtrip = 0;
        qsp.deviceState = DEVICE_STATE_FAILSAFE;
        qsp.anyFrameRecivedAt = 0;
    }

    //FIXME rxDeviceState should be resetted also in RC_HEALT frame is not received in a long period

    //Handle audible alarms
    if (qsp.deviceState == DEVICE_STATE_FAILSAFE) {
        //Failsafe detected by TX
        buzzerContinousMode(BUZZER_MODE_SLOW_BEEP, &buzzer);
    } else if (txDeviceState.isReceiving && (rxDeviceState.flags & 0x1) == 1) {
        //Failsafe reported by RX module
        buzzerContinousMode(BUZZER_MODE_SLOW_BEEP, &buzzer);
    } else if (txDeviceState.isReceiving && radioState.rssi < 60) {
        buzzerContinousMode(BUZZER_MODE_DOUBLE_CHIRP, &buzzer); // RSSI below 60dB
    } else if (txDeviceState.isReceiving && radioState.rssi < 80) {
        buzzerContinousMode(BUZZER_MODE_CHIRP, &buzzer); // RSSI below 80dB
    } else {
        buzzerContinousMode(BUZZER_MODE_OFF, &buzzer);
    }

#ifdef FEATURE_TX_OLED
    if (
        currentMillis - lastOledTaskTime > OLED_UPDATE_RATE
    ) {
        lastOledTaskTime = currentMillis;
        display.clearDisplay();

        display.setTextColor(WHITE, BLACK);
        display.setCursor(0, 0);
        display.setTextSize(3);
        display.print(radioState.rssi);

        display.setCursor(18, 28);
        display.setTextSize(2);
        display.print(radioState.snr);

        display.setCursor(74, 0);
        display.setTextSize(3);
        display.print(rxDeviceState.rssi);

        display.setCursor(92, 28);
        display.setTextSize(2);
        display.print(rxDeviceState.snr);

        #ifdef DEBUG_TX_INPUT_ON_OLED
        display.setCursor(0, 48);
        display.setTextSize(2);
        display.print(txInput.channels[0]);
        #endif

        display.setCursor(54, 48);
        display.setTextSize(2);
        display.print(txDeviceState.roundtrip);

        display.display();
    }
#endif

#endif


}

void onReceive(int packetSize)
{
    /*
     * We can start reading only when radio is not reading.
     * If not reading, then we might start
     */
    if (radioState.bytesToRead == NO_DATA_TO_READ) {

        if (packetSize >= MIN_PACKET_SIZE && packetSize <= MAX_PACKET_SIZE) {
            //We have a packet candidate that might contain a valid QSP packet
            radioState.bytesToRead = packetSize;
        } else {
            /*
            That packet was not very interesting, just flush it, we have no use
            */
            LoRa.sleep();
            LoRa.receive();
            radioState.deviceState = RADIO_STATE_RX;
        }
    }
}