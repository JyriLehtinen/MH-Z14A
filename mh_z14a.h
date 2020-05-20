/* Simple library for MH-Z14A
 * By Jyri Lehtinen (JyriLehtinen @ github)
 *  May 2020
 */
#ifndef MH_Z14A_H
#define MH_Z14A_H 

#include <stdint.h>

#if ARDUINO >= 100
  #include "Arduino.h"
#else
  #include "WProgram.h"
#endif

#ifndef ESP32 
  #include <SoftwareSerial.h>
#endif

class MH_Z14A {
  public:
    MH_Z14A(void);
    void begin(HardwareSerial* serial);
    void begin(HardwareSerial* serial, int8_t pin_rx, int8_t pin_tx);
#ifndef ESP32 
    void begin(SoftwareSerial* serial);
    void begin(uint8_t pin_rx, uint8_t pin_tx);
#endif
    uint8_t read(uint32_t *ppm);
    uint8_t calibrate_zero_point(void);
	uint8_t calibrate_span_point(void);
  private:
    uint8_t _pin_rx, _pin_tx;
    Stream *mh_data;
};

#endif
