#include "tx_oled.h"

TxOled::TxOled(void) {
    Adafruit_SSD1306 _display(-1);
}

void TxOled::init() {
    _display.begin(SSD1306_SWITCHCAPVCC, 0x3C); // initialize with the I2C addr 0x3C (for the 128x32)
    _display.setTextSize(1);
    _display.setTextColor(WHITE);
    _display.clearDisplay();
    _display.display();
}

void TxOled::loop(
    volatile RadioState_t *radioState,
    RxDeviceState_t *rxDeviceState,
    TxDeviceState_t *txDeviceState,
    Tactile *button0,
    Tactile *button1
) {
    bool update = false;

    //Depending on page, things might be different
    switch (_page) {

        case TX_PAGE_INIT:
            //Second button has notthing to do over here
            break;

        case TX_PAGE_STATS:
            //Second button refreshes this page
            if (button1->getState() == TACTILE_STATE_SHORT_PRESS) {
                update = true;
            }
            break;

    }

    //Short press of button0 always toggles no next page
    if (button0->getState() == TACTILE_STATE_SHORT_PRESS) {
        _mainPageSequenceIndex++;
        if (_mainPageSequenceIndex == TX_OLED_PAGE_COUNT) {
            _mainPageSequenceIndex = 0;
        }
        update = true;
    }

    if (update) {
        page(
            radioState,
            rxDeviceState,
            txDeviceState,
            pageSequence[_mainPageSequenceIndex]
        );
    }

}

void TxOled::page(
    volatile RadioState_t *radioState,
    RxDeviceState_t *rxDeviceState,
    TxDeviceState_t *txDeviceState,
    int page
) {
    switch (page) {
        case TX_PAGE_INIT:
            renderPageInit(radioState, rxDeviceState, txDeviceState);
            break;
        case TX_PAGE_STATS:
            renderPageStats(radioState, rxDeviceState, txDeviceState);
            break;
        case TX_PAGE_PWR:
            renderPagePwr(radioState, rxDeviceState, txDeviceState);
            break;
        case TX_PAGE_BIND:
            renderPageBind(radioState, rxDeviceState, txDeviceState);
            break;
        case TX_PAGE_MODE:
            renderPageMode(radioState, rxDeviceState, txDeviceState);
            break;

    }
    _page = page;
}

void TxOled::renderPagePwr(
    volatile RadioState_t *radioState,
    RxDeviceState_t *rxDeviceState,
    TxDeviceState_t *txDeviceState
) {
    _display.clearDisplay();
    _display.setTextColor(WHITE, BLACK);

    _display.setCursor(0, 0);
    _display.setTextSize(2);
    _display.print("PWR");

    //TODO content
    _display.setCursor(0, 25);
    _display.setTextSize(3);
    _display.print(radioState->loraTxPower);
    _display.print("dBm");

    _display.display();
}

void TxOled::renderPageBind(
    volatile RadioState_t *radioState,
    RxDeviceState_t *rxDeviceState,
    TxDeviceState_t *txDeviceState
) {
    _display.clearDisplay();
    _display.setTextColor(WHITE, BLACK);

    _display.setCursor(0, 0);
    _display.setTextSize(2);
    _display.print("Bind");

    //TODO content

    _display.display();
}

void TxOled::renderPageMode(
    volatile RadioState_t *radioState,
    RxDeviceState_t *rxDeviceState,
    TxDeviceState_t *txDeviceState
) {
    _display.clearDisplay();
    _display.setTextColor(WHITE, BLACK);

    _display.setCursor(0, 0);
    _display.setTextSize(2);
    _display.print("Mode");

    _display.setCursor(0, 25);
    _display.setTextSize(3);
    _display.print("Full");

    _display.display();
}

void TxOled::renderPageStats(
    volatile RadioState_t *radioState,
    RxDeviceState_t *rxDeviceState,
    TxDeviceState_t *txDeviceState
) {
    _display.clearDisplay();
    _display.setTextColor(WHITE, BLACK);
    
    _display.setCursor(0, 0);
    _display.setTextSize(3);
    _display.print(radioNode.rssi);

    _display.setCursor(18, 28);
    _display.setTextSize(2);
    _display.print(radioNode.snr);

    _display.setCursor(74, 0);
    _display.setTextSize(3);
    _display.print(rxDeviceState->rssi);

    _display.setCursor(92, 28);
    _display.setTextSize(2);
    _display.print(rxDeviceState->snr);

    _display.setCursor(54, 48);
    _display.setTextSize(2);    
    _display.print(txDeviceState->roundtrip);

    _display.display();
}

void TxOled::renderPageInit(
    volatile RadioState_t *radioState,
    RxDeviceState_t *rxDeviceState,
    TxDeviceState_t *txDeviceState
) {
    _display.clearDisplay();
    _display.setTextColor(WHITE, BLACK);
    _display.setTextSize(2);

    _display.setCursor(0, 0);
    _display.print("Rdy ");
    _display.print(radioState->loraTxPower);
    _display.print("dBm");

    _display.setTextSize(1);
    _display.setCursor(0, 32);
    _display.print("Bandwitdh: ");
    _display.print(radioState->loraBandwidth / 1000);
    _display.print("kHz");

    _display.setCursor(0, 42);
    _display.print("SF: ");
    _display.print(radioState->loraSpreadingFactor);

    _display.setCursor(64, 42);
    _display.print("CR: ");
    _display.print(radioState->loraCodingRate);

    _display.setCursor(0, 52);
    _display.print("Rate: ");
    _display.print(1000 / TX_TRANSMIT_SLOT_RATE);
    _display.print("Hz");

    _display.display();
}



