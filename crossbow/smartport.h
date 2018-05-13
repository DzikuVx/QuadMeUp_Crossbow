#ifndef _SMARTPORT_H_
#define _SMARTPORT_H_

#include "Arduino.h"
#include <SoftwareSerial.h>

#define SMARTPORT_BPS 57600
#define SMARTPORT_UPDATE_RATE 250

enum {
  SMARTPORT_SENSOR_RSSI = 0xf101,
  SMARTPORT_SENSOR_VFAS = 0x0210
};

enum {
  INTERNAL_SENSOR_RSSI,
  INTERNAL_SENSOR_VFAS
};

class SmartPort
{
  public:
  	SmartPort(SoftwareSerial &serial);
    void start(void);
    void stop(void);
    void loop(bool &windowOpen);
    void set(uint8_t sensor, int value);
  private:
  	SoftwareSerial _serial;
    int32_t _data[2];
    uint32_t _nextTransmitMillis = 0;
    void sendSensor(uint8_t sensor);
};

#endif