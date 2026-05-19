#ifndef ADXL346_H
#define ADXL346_H

#include <stdint.h>
#include <SPI.h>


class ADXL346 {
  private:
    // Serial communication
    uint8_t ADXL_CS;
    static constexpr int SerialClockSpeed = 5E6;
    static const SPISettings SerialSettings;

    // Regigster map
    static constexpr uint8_t BW_RATE     = 0x2C;  // Data rate and power mode control
    static constexpr uint8_t POWER_CTL   = 0x2D;  // Power-saving features control
    static constexpr uint8_t DATA_FORMAT = 0x31;  // Data format control
    static constexpr uint8_t DATAX0      = 0x32;  // X-Axis Data 0
    static constexpr uint8_t DATAX1      = 0x33;  // X-Axis Data 1
    static constexpr uint8_t DATAY0      = 0x34;  // Y-Axis Data 0
    static constexpr uint8_t DATAY1      = 0x35;  // Y-Axis Data 1
    static constexpr uint8_t DATAZ0      = 0x36;  // Z-Axis Data 0
    static constexpr uint8_t DATAZ1      = 0x37;  // Z-Axis Data 1

    // Register access modes
    static constexpr uint8_t ADXL_WRITE     = 0x00;
    static constexpr uint8_t ADXL_READ      = 0x80;
    static constexpr uint8_t ADXL_READ_MULT = 0xC0;

    void RREG(int16_t *x, int16_t *y, int16_t *z);

  public:
    ADXL346(uint8_t _ADXL_CS);
    void init();
    void read(int32_t *frame);
};

#endif
