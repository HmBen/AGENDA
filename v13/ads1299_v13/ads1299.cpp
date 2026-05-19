#include "ads1299.h"


const SPISettings ADS1299::SerialSettings = SPISettings(SerialClockSpeed, MSBFIRST, SPI_MODE1);


ADS1299::ADS1299(uint8_t _ADS_CS) {
  ADS_CS = _ADS_CS;
}


void ADS1299::read(int32_t *frame) {
  SPI.beginTransaction(SerialSettings);
  delayNanoseconds(10);
  digitalWriteFast(ADS_CS, LOW);
  delayNanoseconds(6); // CS low to first serial clock, at DVDD > 2.7V
  SPI.transfer(0x00, 3); // Skip status registers
  for(int i = 0; i < N_CH; i++) {
      int32_t sample = 0;
      sample = SPI.transfer(0x00);
      sample = (sample<<8) | SPI.transfer(0x00);
      sample = (sample<<8) | SPI.transfer(0x00);
      sample = (sample & 0x800000) ? (sample | 0xFF000000) : sample; // Sign extension
      frame[i] = sample;
  }
  delayMicroseconds(2); // Final serial clock falling edge to CS high: t(CLK) * 4 = 2us
  digitalWriteFast(ADS_CS, HIGH);
  SPI.endTransaction();
}


void ADS1299::RESET() {
  SPI.beginTransaction(SerialSettings);
  digitalWrite(ADS_CS, LOW);
  delayNanoseconds(6); // CS low to first serial clock, at DVDD > 2.7V
  SPI.transfer(0x06); // RESET
  delayMicroseconds(2); // Final serial clock falling edge to CS high: t(CLK) * 4 = 2us
  digitalWrite(ADS_CS, HIGH);
  SPI.endTransaction();
}


void ADS1299::SDATAC() {
  SPI.beginTransaction(SerialSettings);
  digitalWrite(ADS_CS, LOW);
  delayNanoseconds(6); // CS low to first serial clock, at DVDD > 2.7V
  SPI.transfer(0x11); // SDATAC
  delayMicroseconds(2); // Final serial clock falling edge to CS high: t(CLK) * 4 = 2us
  digitalWrite(ADS_CS, HIGH);
  SPI.endTransaction();
}


void ADS1299::RDATAC() {
  SPI.beginTransaction(SerialSettings);
  digitalWrite(ADS_CS, LOW);
  delayNanoseconds(6); // CS low to first serial clock, at DVDD > 2.7V
  SPI.transfer(0x10); // RDATAC
  delayMicroseconds(2); // Final serial clock falling edge to CS high: t(CLK) * 4 = 2us
  digitalWrite(ADS_CS, HIGH);
  SPI.endTransaction();
}


void ADS1299::WREG(uint8_t ADDRESS, uint8_t BYTE) {
  SPI.beginTransaction(SerialSettings);
  digitalWrite(ADS_CS, LOW);
  delayNanoseconds(6); // CS low to first serial clock, at DVDD > 2.7V
  SPI.transfer(ADDRESS + 0x40);
  delayMicroseconds(2); // Wait at least t(Serial decode) = t(CLK) * 4 = 2us between bytes
  SPI.transfer(0x00); // Notifying to write 1 byte
  delayMicroseconds(2); // Wait at least t(Serial decode) = t(CLK) * 4 = 2us between bytes
  SPI.transfer(BYTE);
  delayMicroseconds(2); // Final serial clock falling edge to CS high: t(CLK) * 4 = 2us
  digitalWrite(ADS_CS, HIGH);
  SPI.endTransaction();
}


uint8_t ADS1299::RREG(uint8_t ADDRESS) {
  uint8_t val = 0;
  SPI.beginTransaction(SerialSettings);
  digitalWrite(ADS_CS, LOW);
  delayNanoseconds(6); // CS low to first serial clock, at DVDD > 2.7V
  SPI.transfer(ADDRESS + 0x20);
  delayMicroseconds(2); // Wait at least t(Serial decode) = t(CLK) * 4 = 2us between bytes
  SPI.transfer(0x00); // Requesting 1 byte
  delayMicroseconds(2); // Wait at least t(Serial decode) = t(CLK) * 4 = 2us between bytes
  val = SPI.transfer(0x00); // Produce clock for receiving data by transmitting empty byte
  delayMicroseconds(2); // Final serial clock falling edge to CS high: t(CLK) * 4 = 2us
  digitalWrite(ADS_CS, HIGH);
  SPI.endTransaction();
  return val;
}
