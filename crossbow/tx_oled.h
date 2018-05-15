#pragma once

#ifndef TX_OLED_H
#define TX_OLED_H

#include <Adafruit_SSD1306.h>
#include "Wire.h"
#include "variables.h",
#include "tactile.h"

enum txOledPages {
    TX_PAGE_NONE,
    TX_PAGE_INIT,
    TX_PAGE_STATS,
    TX_PAGE_PWR,
    TX_PAGE_BIND,
    TX_PAGE_MODE
};

#define TX_OLED_PAGE_COUNT 5

const uint8_t pageSequence[TX_OLED_PAGE_COUNT] = {
    TX_PAGE_INIT,
    TX_PAGE_STATS,
    TX_PAGE_PWR,
    TX_PAGE_BIND,
    TX_PAGE_MODE
};

class TxOled {
    public:
        TxOled(void);
        void init(
            volatile RadioState_t *radioState,
            RxDeviceState_t *rxDeviceState,
            TxDeviceState_t *txDeviceState,
            Tactile *button0,
            Tactile *button1
            );
        void loop(void);
        void page(int page);
    private:
        volatile RadioState_t *_radioState;
        RxDeviceState_t *_rxDeviceState;
        TxDeviceState_t *_txDeviceState;
        Tactile *_button0;
        Tactile *_button1;
        Adafruit_SSD1306 _display;
        void renderPageInit(void);
        void renderPageStats(void);
        void renderPagePwr(void);
        void renderPageBind(void);
        void renderPageMode(void);
        uint8_t _page = TX_PAGE_NONE;
        uint8_t _mainPageSequenceIndex = 0;
};

#endif