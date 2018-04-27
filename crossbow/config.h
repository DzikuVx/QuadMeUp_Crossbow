#ifndef CONFIG_H
#define CONFIG_H

/*
 *  Hardware type. Available types:
 *  ARDUINO_AVR_FEATHER32U4
 *  ARDUINO_SAMD_FEATHER_M0
 * 
 *  Leave commented for autodetect
 */
// #define ARDUINO_AVR_FEATHER32U4

/*
 * TX or RX mode for hardware. Available types:
 * DEVICE_MODE_TX
 * DEVICE_MODE_RX
 */
#define DEVICE_MODE_TX
// #define DEVICE_MODE_RX

// #define FEATURE_TX_OLED
// #define FORCE_TX_WITHOUT_INPUT

/*
 * Default mode of TX data input: SBUS
 * Possible values:
 * FEATURE_TX_INPUT_PPM
 * FEATURE_TX_INPUT_SBUS
 */
#define FEATURE_TX_INPUT_SBUS

#define DEBUG_SERIAL
// #define DEBUG_PING_PONG
// #define DEBUG_LED
// #define DEBUG_TX_INPUT_ON_OLED

#endif
