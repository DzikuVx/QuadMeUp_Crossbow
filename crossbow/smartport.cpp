#include "smartport.h"

SmartPort::SmartPort(SoftwareSerial &serial) : _serial(serial) {
}

void SmartPort::start(void) {
    _serial.begin(SMARTPORT_BPS);
}

void SmartPort::stop(void) {
    _serial.end();
}

void SmartPort::set(uint8_t sensor, int value) {
    SmartPort::_data[sensor] = value;
}

void SmartPort::loop(bool &windowOpen) {
    if (windowOpen && millis() > SmartPort::_nextTransmitMillis) {
        windowOpen = false;

        //ATM we send only RSSI
        sendSensor(INTERNAL_SENSOR_RSSI);

        SmartPort::_nextTransmitMillis = millis() + SMARTPORT_UPDATE_RATE;
    }
}

void SmartPort::sendSensor(uint8_t sensor) {
    uint8_t buffer[9];
    uint16_t crc = 0;
    
    buffer[0] = 0x98;
    buffer[1] = 0x10;

    switch (sensor) {

        case INTERNAL_SENSOR_RSSI: // SMARTPORT_SENSOR_RSSI = 0xf101,
            buffer[2] = (uint8_t) (SMARTPORT_SENSOR_RSSI & 0xff);
            buffer[3] = (uint8_t) (SMARTPORT_SENSOR_RSSI >> 8);
            buffer[4] = SmartPort::_data[sensor];
            break;

        default:
            return;
    }

    SmartPort::_serial.write(0x7e); // No CRC for start byte
    
    //Iterate over buffer
    for (uint8_t i = 0; i < 9; i++) {

        if (i == 8) {
            buffer[i] = 0xff - crc; //Write CRC as last byte in buffer
        }

        if (buffer[i] == 0x7e || buffer[i] == 0x7d) {
            SmartPort::_serial.write(0x7d);
            SmartPort::_serial.write(0x20 ^ buffer[i]);
        } else {
            SmartPort::_serial.write(buffer[i]);
        }

        //SmartPort CRC routine
        crc += buffer[i];
        crc += crc >> 8;
        crc &= 0x00ff;
    }

}