#pragma once

#ifndef TX_OLED_H
#define TX_OLED_H

#include "Wire.h"
#include <U8x8lib.h>
#include "variables.h"
#include "tactile.h"
#include "radio_node.h"

extern RadioNode radioNode;
extern RxDeviceState_t rxDeviceState;
extern TxDeviceState_t txDeviceState;
extern Tactile button0;
extern Tactile button1;

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
        void init();
        void loop();
        void page(uint8_t page);
    private:
        U8X8_SSD1306_128X64_NONAME_HW_I2C _display;
        void renderPageInit();
        void renderPageStats();
        void renderPagePwr();
        void renderPageBind();
        void renderPageMode();
        uint8_t _page = TX_PAGE_NONE;
        uint8_t _mainPageSequenceIndex = 0;
};

#endif