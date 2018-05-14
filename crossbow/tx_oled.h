#pragma once

#ifndef TX_OLED_H
#define TX_OLED_H

#include <Adafruit_SSD1306.h>
#include "Wire.h"
#include "variables.h"

#define OLED_RESET -1

enum txOledPages {
    TX_PAGE_NONE,
    TX_PAGE_INIT,
};

class TxOled {
    public:
        TxOled(void);
        void init(volatile RadioState_t *radioState);
        void page(int page);
    private:
        volatile RadioState_t *_radioState;
        Adafruit_SSD1306 _display;
        void pageInit(void);
};

#endif