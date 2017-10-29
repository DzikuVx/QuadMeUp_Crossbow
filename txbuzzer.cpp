#include "Arduino.h"
#include "txbuzzer.h"

/**
 * This method plays selected pattern only once
 * It disables continious mode 
 */
void buzzerSingleMode(uint8_t mode, BuzzerState_t *buzzer) {
    buzzer->singleModeEnabled = true;
    buzzer->enabled = false;
    buzzer->mode = mode;
    buzzer->tick = 0;
}

void buzzerContinousMode(uint8_t mode, BuzzerState_t *buzzer) {
    buzzer->singleModeEnabled = false;
    buzzer->enabled = true;
    buzzer->mode = mode;
}

void buzzerProcess(uint8_t pin, uint32_t timestamp, BuzzerState_t *buzzer)
{

    if (!buzzer->enabled && !buzzer->singleModeEnabled)
    {
        digitalWrite(pin, LOW);
        return;
    }

    if (timestamp > buzzer->updateTime)
    {

        int8_t currentPattern = buzzer->pattern[buzzer->mode][buzzer->element];

        if (currentPattern == PATTERN_CYCLE_OFF)
        {
            digitalWrite(pin, LOW);
        }
        else if (currentPattern == PATTERN_CYCLE_ON)
        {
            digitalWrite(pin, HIGH);
        }
        else if (currentPattern == PATTERN_CYCLE_IGNORE || currentPattern == buzzer->tick)
        {

            if (currentPattern != PATTERN_CYCLE_IGNORE)
            {
                digitalWrite(pin, !digitalRead(pin));
            }

            buzzer->element++;
            if (buzzer->element == PATTERN_ELEMENT_NUMBER)
            {
                buzzer->element = 0;
            }
        }

        buzzer->tick++;
        if (buzzer->tick >= buzzer->patternMaxTick)
        {
            buzzer->tick = 0;
            buzzer->singleModeEnabled = false;
        }
        buzzer->updateTime = timestamp + buzzer->patternTickPerdiod;
    }
};