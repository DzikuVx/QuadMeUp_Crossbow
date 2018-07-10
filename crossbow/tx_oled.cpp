#include "tx_oled.h"

#define OLED_COL_COUNT 16

TxOled::TxOled(void) {
    U8X8_SSD1306_128X64_NONAME_HW_I2C _display(U8X8_PIN_NONE);
}

void TxOled::init() {
    _display.begin();
    _display.setFont(u8x8_font_pxplustandynewtv_f);
    _display.clear();
}

void TxOled::loop() {
    bool update = false;

    //Depending on page, things might be different
    switch (_page) {

        case TX_PAGE_INIT:
            //Second button has notthing to do over here
            break;

        case TX_PAGE_BIND:
            if (button1.getState() == TACTILE_STATE_LONG_PRESS) {

                if (!platformNode.isBindMode) {
                    platformNode.enterBindMode();
                } else {
                    platformNode.leaveBindMode();
                }
                update = true;
            }
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
    char buf[OLED_COL_COUNT];

    _display.clear();
    _display.draw1x2String(0, 0, "Power");

    snprintf(buf, OLED_COL_COUNT, "%d%s", radioNode.loraTxPower, "dBm");
    _display.draw1x2String(0, 4, buf);
}

void TxOled::renderPageBind() {
    char buf[OLED_COL_COUNT];

    _display.clear();
    _display.draw1x2String(0, 0, "Bind");

    if (platformNode.isBindMode) {
        snprintf(buf, OLED_COL_COUNT, "Binding!!");
    } else {
        snprintf(buf, OLED_COL_COUNT, "Bind?");
    }

    _display.draw1x2String(0, 4, buf);
}

void TxOled::renderPageMode() {
    char buf[OLED_COL_COUNT];

    _display.clear();
    _display.draw1x2String(0, 0, "Mode");

    snprintf(buf, OLED_COL_COUNT, "Full");
    _display.draw1x2String(0, 4, buf);
}

void TxOled::renderPageStats() {

    char buf[OLED_COL_COUNT];

    _display.clear();
    _display.draw1x2String(0, 0, "Stats");

    _display.drawString(0, 3, "RSSI");
    _display.drawString(0, 5, "SNR");
    
    snprintf(buf, OLED_COL_COUNT, "%d", radioNode.rssi);
    _display.drawString(6, 3, buf);
    snprintf(buf, OLED_COL_COUNT, "%d", rxDeviceState.rssi);
    _display.drawString(11, 3, buf);

    snprintf(buf, OLED_COL_COUNT, "%d", radioNode.snr);
    _display.drawString(6, 5, buf);
    snprintf(buf, OLED_COL_COUNT, "%d", rxDeviceState.snr);
    _display.drawString(11, 5, buf);

    snprintf(buf, OLED_COL_COUNT, "Trip: %d", txDeviceState.roundtrip);
    _display.drawString(0, 7, buf);
}

void TxOled::renderPageInit() {

    char buf[OLED_COL_COUNT];

    _display.clear();

    snprintf(buf, OLED_COL_COUNT, "Rdy %d %s", radioNode.loraTxPower, "dBm");
    _display.draw1x2String(0, 0, buf);

    snprintf(buf, OLED_COL_COUNT, "BW %dkHz", radioNode.loraBandwidth / 1000);
    _display.drawString(0, 4, buf);

    snprintf(buf, OLED_COL_COUNT, "SF %d", radioNode.loraSpreadingFactor);
    _display.drawString(0, 5, buf);

    snprintf(buf, OLED_COL_COUNT, "CR %d", radioNode.loraCodingRate);
    _display.drawString(8, 5, buf);

    snprintf(buf, OLED_COL_COUNT, "Rate: %dHz", 1000 / TX_TRANSMIT_SLOT_RATE);
    _display.drawString(0, 7, buf);
}



