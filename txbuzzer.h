#pragma once

#include "Arduino.h"

#define PATTERN_CYCLE_OFF 127
#define PATTERN_CYCLE_ON -1
#define PATTERN_CYCLE_IGNORE -2

#define PATTERN_MODES_NUMBER 6
#define PATTERN_ELEMENT_NUMBER 4

enum {
    BUZZER_MODE_OFF = 0,
    BUZZER_MODE_CONTINUOUS = 1,
    BUZZER_MODE_SLOW_BEEP = 2,
    BUZZER_MODE_FAST_BEEP = 3,
    BUZZER_MODE_CHIRP = 4,
    BUZZER_MODE_DOUBLE_CHIRP = 5
};

struct BuzzerState_t {
    bool enabled = false; //Continous mode buzzer
    bool singleModeEnabled = false;
    uint8_t mode = BUZZER_MODE_OFF;
    
    uint32_t updateTime = 0;

    uint8_t tick = 0;
    uint8_t element = 0;
    const uint8_t patternMaxTick = 20;
    const uint8_t patternTickPerdiod = 75;

    const int8_t pattern[PATTERN_MODES_NUMBER][PATTERN_ELEMENT_NUMBER] = {
        {PATTERN_CYCLE_OFF},
        {PATTERN_CYCLE_ON},
        {0, 7, 10, 17},
        {0, 4, 10, 14},
        {0, 1, PATTERN_CYCLE_IGNORE, PATTERN_CYCLE_IGNORE},
        {0, 1, 2, 3}
    };

};

void buzzerSingleMode(uint8_t mode, BuzzerState_t *buzzer);
void buzzerContinousMode(uint8_t mode, BuzzerState_t *buzzer);
void buzzerProcess(uint8_t pin, uint32_t timestamp, BuzzerState_t *buzzer);