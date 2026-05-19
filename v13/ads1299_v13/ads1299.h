#ifndef ADS1299_H
#define ADS1299_H

#include <stdint.h>
#include <SPI.h>


class ADS1299 {
  private:
    // Serial communication
    uint8_t ADS_CS;
    static constexpr int SerialClockSpeed = 20E6;
    static const SPISettings SerialSettings;

    static constexpr int N_CH = 8;

  public:
    ADS1299(uint8_t _ADS_CS);
    //void init();

    void read(int32_t *frame);

    // The following should eventually be made privates
    void RESET();
    void SDATAC();
    void RDATAC();
    void WREG(uint8_t ADDRESS, uint8_t BYTE);
    uint8_t RREG(uint8_t ADDRESS);


};

#endif
