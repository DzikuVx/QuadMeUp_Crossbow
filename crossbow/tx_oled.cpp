#include "tx_oled.h"

TxOled::TxOled(void) {
    Adafruit_SSD1306 _display(-1);
}

void TxOled::init(
    volatile RadioState_t *radioState,
    RxDeviceState_t *_rxDeviceState,
    TxDeviceState_t *_txDeviceState,
    Tactile *button0,
    Tactile *button1
) {
    _display.begin(SSD1306_SWITCHCAPVCC, 0x3C); // initialize with the I2C addr 0x3C (for the 128x32)
    _display.setTextSize(1);
    _display.setTextColor(WHITE);
    _display.clearDisplay();
    _display.display();

    _radioState = radioState;
    _button0 = button0;
    _button1 = button1;
}

void TxOled::loop(void) {
    bool update = false;

    //Depending on page, things might be different
    switch (_page) {

        case TX_PAGE_INIT:
            //Second button has notthing to do over here
            break;

        case TX_PAGE_STATS:
            //Second button refreshes this page
            if (_button1->getState() == TACTILE_STATE_SHORT_PRESS) {
                update = true;
            }
            break;

    }

    //Short press of button0 always toggles no next page
    if (_button0->getState() == TACTILE_STATE_SHORT_PRESS) {
        _mainPageSequenceIndex++;
        if (_mainPageSequenceIndex == TX_OLED_PAGE_COUNT) {
            _mainPageSequenceIndex = 0;
        }
        update = true;
    }

    if (update) {
        page(pageSequence[_mainPageSequenceIndex]);
    }

}

void TxOled::page(int page) {
    switch (page) {
        case TX_PAGE_INIT:
            renderPageInit();
            break;
        case TX_PAGE_STATS:
            renderPageStats();
            break;
        case TX_PAGE_PWR:
            renderPagePwr();
            break;
        case TX_PAGE_BIND:
            renderPageBind();
            break;
        case TX_PAGE_MODE:
            renderPageMode();
            break;

    }
    _page = page;
}

void TxOled::renderPagePwr(void) {
    _display.clearDisplay();
    _display.setTextColor(WHITE, BLACK);

    _display.setCursor(0, 0);
    _display.setTextSize(2);
    _display.print("PWR");

    //TODO content

    _display.display();
}

void TxOled::renderPageBind(void) {
    _display.clearDisplay();
    _display.setTextColor(WHITE, BLACK);

    _display.setCursor(0, 0);
    _display.setTextSize(2);
    _display.print("Bind");

    //TODO content

    _display.display();
}

void TxOled::renderPageMode(void) {
    _display.clearDisplay();
    _display.setTextColor(WHITE, BLACK);

    _display.setCursor(0, 0);
    _display.setTextSize(2);
    _display.print("Mode");

    //TODO content

    _display.display();
}

void TxOled::renderPageStats(void) {
    _display.clearDisplay();
    _display.setTextColor(WHITE, BLACK);
    
    _display.setCursor(0, 0);
    _display.setTextSize(3);
    _display.print(_radioState->rssi);

    _display.setCursor(18, 28);
    _display.setTextSize(2);
    _display.print(_radioState->snr);

    _display.setCursor(74, 0);
    _display.setTextSize(3);
    _display.print(_rxDeviceState->rssi);

    _display.setCursor(92, 28);
    _display.setTextSize(2);
    _display.print(_rxDeviceState->snr);

    _display.setCursor(54, 48);
    _display.setTextSize(2);
    
    if (_txDeviceState->roundtrip < 100) {
        _display.print(_txDeviceState->roundtrip);
    } else {
        _display.print(0);
    }
    
    _display.display();
}

void TxOled::renderPageInit(void) {
    _display.clearDisplay();
    _display.setTextColor(WHITE, BLACK);
    _display.setTextSize(2);

    _display.setCursor(0, 0);
    _display.print("Rdy ");
    _display.print(_radioState->loraTxPower);
    _display.print("dBm");

    _display.setTextSize(1);
    _display.setCursor(0, 32);
    _display.print("Bandwitdh: ");
    _display.print(_radioState->loraBandwidth / 1000);
    _display.print("kHz");

    _display.setCursor(0, 42);
    _display.print("SF: ");
    _display.print(_radioState->loraSpreadingFactor);

    _display.setCursor(64, 42);
    _display.print("CR: ");
    _display.print(_radioState->loraCodingRate);

    _display.setCursor(0, 52);
    _display.print("Rate: ");
    _display.print(1000 / TX_TRANSMIT_SLOT_RATE);
    _display.print("Hz");

    _display.display();
}



