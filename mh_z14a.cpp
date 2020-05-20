/**** CO2 Sensor Definitions ****/
// Component Datasheet: https://www.openhacks.com/uploadsproductos/mh-z14_co2.pdf

#include "mh_z14a.h"


#define MH_Z14A_INVALID_MSG 2
#define MH_Z14A_CHECKSUM_ERR 3


// Calibrate zero point command  
static const uint8_t calib_zero_cmd[9] = {0xFF,0x01,0x87,0x00,0x00,0x00,0x00,0x00,0x78};

//Calibrate span point command
static const uint8_t calib_span_cmd[9] = {0xFF,0x01,0x88,0x07,0xD0,0x00,0x00,0x00,0xA0};

//Reqest Gas concentration command
static const uint8_t read_ppm_cmd[9] = {0xFF,0x01,0x86,0x00,0x00,0x00,0x00,0x00,0x79}; 

uint8_t getCheckSum(uint8_t *packet);

MH_Z14A::MH_Z14A(void) {
//
}

uint8_t MH_Z14A::read(uint32_t *ppm) {

    while (mh_data->available() > 0) {
      mh_data->read(); // Clear out the RX buffer
    }
    
    // Send command to serial and make sure the command is fully transmitted
    mh_data->write(read_ppm_cmd, 9);
    mh_data->flush();

    uint8_t rx_buffer[9];
    mh_data->readBytes(rx_buffer, 9);
    
    int ppm_serial;

    if (rx_buffer[0] != 0xFF)
       return MH_Z14A_INVALID_MSG;
    if (rx_buffer[1] != 0x86)
      return MH_Z14A_INVALID_MSG;


    // CheckSum
    if( rx_buffer[8] != getCheckSum(rx_buffer) )
      return MH_Z14A_CHECKSUM_ERR;
    else {
	  ppm_serial = (rx_buffer[2]*256) + rx_buffer[3];
	  *ppm = ppm_serial;
      return 0;
	}
}

uint8_t MH_Z14A::calibrate_zero_point(void) {
    mh_data->write(calib_zero_cmd, 9);
    mh_data->flush();
    return 0;
}

uint8_t MH_Z14A::calibrate_span_point(void) {
    mh_data->write(calib_span_cmd, 9);
    mh_data->flush();
    return 0;
}



#ifndef ESP32
void MH_Z14A::begin(uint8_t pin_rx, uint8_t pin_tx) {
    _pin_rx = pin_rx;
    _pin_tx = pin_tx;

    SoftwareSerial *softSerial = new SoftwareSerial(_pin_rx, _pin_tx);

    softSerial->begin(9600);

    mh_data = softSerial;
}
#endif


void MH_Z14A::begin(HardwareSerial* serial) {
    serial->begin(9600);
    mh_data = serial;
}

void MH_Z14A::begin(HardwareSerial* serial, int8_t rxPin, int8_t txPin) {
    serial->begin(9600, SERIAL_8N1, rxPin, txPin);
    mh_data = serial;
}

#ifndef ESP32
void MH_Z14A::begin(SoftwareSerial* serial) {
    serial->begin(9600);
    mh_data = serial;
}
#endif

uint8_t getCheckSum(uint8_t *packet)
{
  uint8_t checksum = 0;
  for( int i = 1; i < 8; i++)
  {
    checksum += packet[i];
  }
  checksum = 0xFF - checksum;
  checksum += 1;
  return checksum;
}
