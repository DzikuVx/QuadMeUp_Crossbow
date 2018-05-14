#include "tx_oled.h"

TxOled::TxOled(void) {
    Adafruit_SSD1306 _display(-1);
}

void TxOled::init(volatile RadioState_t *radioState) {
    _display.begin(SSD1306_SWITCHCAPVCC, 0x3C); // initialize with the I2C addr 0x3C (for the 128x32)
    _display.setTextSize(1);
    _display.setTextColor(WHITE);
    _display.clearDisplay();
    _display.display();

    _radioState = radioState;
}

void TxOled::page(int page) {
    switch (page) {

        case TX_PAGE_INIT:
            pageInit();
            break;

    }
}

void TxOled::pageInit(void) {
    _display.clearDisplay();

    _display.setTextColor(WHITE, BLACK);
    _display.setCursor(0, 0);
    _display.setTextSize(2);
    _display.print("Rdy ");
    _display.print(_radioState->loraTxPower);
    _display.print("dBm");
}



