#include "tx_oled.h"

#define OLED_COL_COUNT 16

TxOled::TxOled(void) {
    U8X8_SSD1306_128X64_NONAME_HW_I2C _display(U8X8_PIN_NONE);
}

void TxOled::init() {
    _display.begin();
    _display.clear();
}

void TxOled::loop() {
    bool update = false;

    //Depending on page, things might be different
    switch (_page) {

        case TX_PAGE_INIT:
            //Second button has notthing to do over here
            break;

        case TX_PAGE_STATS:
            //Second button refreshes this page
            if (button1.getState() == TACTILE_STATE_SHORT_PRESS) {
                update = true;
            }
            break;

    }

    //Short press of button0 always toggles no next page
    if (button0.getState() == TACTILE_STATE_SHORT_PRESS) {
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

void TxOled::page(uint8_t page) {

    static uint32_t lastUpdate = 0;

    //Do not allow for OLED to be updated too often
    if (lastUpdate > 0 && millis() - lastUpdate < 200) {
        return;
    }

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

    lastUpdate = millis();
}

void TxOled::renderPagePwr() {
    // _display.clearDisplay();
    // _display.setTextColor(WHITE, BLACK);

    // _display.setCursor(0, 0);
    // _display.setTextSize(2);
    // _display.print("PWR");

    // //TODO content
    // _display.setCursor(0, 25);
    // _display.setTextSize(3);
    // _display.print(radioNode.loraTxPower);
    // _display.print("dBm");

    // _display.display();
}

void TxOled::renderPageBind() {
    // _display.clearDisplay();
    // _display.setTextColor(WHITE, BLACK);

    // _display.setCursor(0, 0);
    // _display.setTextSize(2);
    // _display.print("Bind");

    // //TODO content

    // _display.display();
}

void TxOled::renderPageMode() {
    // _display.clearDisplay();
    // _display.setTextColor(WHITE, BLACK);

    // _display.setCursor(0, 0);
    // _display.setTextSize(2);
    // _display.print("Mode");

    // _display.setCursor(0, 25);
    // _display.setTextSize(3);
    // _display.print("Full");

    // _display.display();
}

void TxOled::renderPageStats() {
    // _display.clearDisplay();
    // _display.setTextColor(WHITE, BLACK);
    
    // _display.setCursor(0, 0);
    // _display.setTextSize(3);
    // _display.print(radioNode.rssi);

    // _display.setCursor(18, 28);
    // _display.setTextSize(2);
    // _display.print(radioNode.snr);

    // _display.setCursor(74, 0);
    // _display.setTextSize(3);
    // _display.print(rxDeviceState.rssi);

    // _display.setCursor(92, 28);
    // _display.setTextSize(2);
    // _display.print(rxDeviceState.snr);

    // _display.setCursor(54, 48);
    // _display.setTextSize(2);    
    // _display.print(txDeviceState.roundtrip);

    // _display.display();
}

void TxOled::renderPageInit() {

    char buf[OLED_COL_COUNT];

    _display.clear();

    _display.setFont(u8x8_font_pxplustandynewtv_f);

    snprintf(buf, OLED_COL_COUNT, "Rdy %d %s", radioNode.loraTxPower, "dBm");
    _display.drawString(0, 0, buf);

    _display.setFont(u8x8_font_chroma48medium8_r);

    snprintf(buf, OLED_COL_COUNT, "BW %dkHz", radioNode.loraBandwidth / 1000);
    _display.drawString(0, 3, buf);

    snprintf(buf, OLED_COL_COUNT, "SF %d", radioNode.loraSpreadingFactor);
    _display.drawString(0, 4, buf);

    snprintf(buf, OLED_COL_COUNT, "CR %d", radioNode.loraCodingRate);
    _display.drawString(8, 4, buf);


    snprintf(buf, OLED_COL_COUNT, "Rate: %dHz", 1000 / TX_TRANSMIT_SLOT_RATE);
    _display.drawString(0, 6, buf);
}



