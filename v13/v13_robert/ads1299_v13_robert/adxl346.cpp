#include <stdint.h>
#include "adxl346.h"


const SPISettings ADXL346::SerialSettings = SPISettings(SerialClockSpeed, MSBFIRST, SPI_MODE3);


ADXL346::ADXL346(uint8_t _ADXL_CS) {
  ADXL_CS = _ADXL_CS;
}


void ADXL346::init() {
  SPI.beginTransaction(SerialSettings);

  // Data format
  digitalWriteFast(ADXL_CS, LOW);
  delayNanoseconds(5); // CS low to first serial clock
  SPI.transfer(DATA_FORMAT | ADXL_WRITE);
  SPI.transfer(0x0B); // 4-pin SPI, full scale (+/-16g), active high interrupts
  delayNanoseconds(10); // Final serial clock falling edge to CS high
  digitalWriteFast(ADXL_CS, HIGH);

  delayNanoseconds(150); // Minimum CS deassertion between commands
  
  // Sampling rate
  digitalWriteFast(ADXL_CS, LOW);
  delayNanoseconds(5); // CS low to first serial clock
  SPI.transfer(BW_RATE | ADXL_WRITE);
  SPI.transfer(0x0B); // 200Hz sampling rate, normal power mode (low noise)
  delayNanoseconds(10); // Final serial clock falling edge to CS high
  digitalWriteFast(ADXL_CS, HIGH);

  delayNanoseconds(150); // Minimum CS deassertion between commands
  
  // Power state
  digitalWriteFast(ADXL_CS, LOW);
  delayNanoseconds(5); // CS low to first serial clock
  SPI.transfer(POWER_CTL | ADXL_WRITE);
  SPI.transfer(0b00101000); // Measurement active 
  delayNanoseconds(10); // Final serial clock falling edge to CS high
  digitalWriteFast(ADXL_CS, HIGH);

  SPI.endTransaction();
}


void ADXL346::read(int32_t *frame) {
  int16_t x, y, z;
  RREG(&x, &y, &z);
  frame[0] = (int32_t) x;
  frame[1] = (int32_t) y;
  frame[2] = (int32_t) z;
}


void ADXL346::RREG(int16_t *x, int16_t *y, int16_t *z) {
  SPI.beginTransaction(SerialSettings);
  digitalWriteFast(ADXL_CS, LOW);
  delayNanoseconds(5); // CS low to first serial clock
  SPI.transfer(DATAX0 | ADXL_READ_MULT);
  *x = (int16_t) SPI.transfer(0) | ((int16_t) SPI.transfer(0) << 8);
  *y = (int16_t) SPI.transfer(0) | ((int16_t) SPI.transfer(0) << 8);
  *z = (int16_t) SPI.transfer(0) | ((int16_t) SPI.transfer(0) << 8);
  delayNanoseconds(10); // Final serial clock falling edge to CS high
  digitalWriteFast(ADXL_CS, HIGH);
  SPI.endTransaction();
}
