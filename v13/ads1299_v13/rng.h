#ifndef RNG_H
#define RNG_H

#include <stdint.h>

class RNG {
  private:
    // Settings
    uint8_t pin; // Pin to display output on
    uint32_t seed;
    uint32_t ON_LENGTH_MS;
    uint32_t INTERVAL_MIN_MS;
    uint32_t INTERVAL_MAX_MS;

    // State variables
    uint32_t xorshift_state;  // Must be initialized to a non-zero value
    uint32_t random_interval;
    uint32_t next_interval;    
    unsigned long previous_millis;
    uint8_t ready;
    uint8_t state;
    
    // Core algorithm
    void xorshift32();
    
  public:
    RNG(uint32_t _seed, uint32_t _ON_LENGTH_MS, uint32_t _INTERVAL_MIN_MS, uint32_t _INTERVAL_MAX_MS, uint8_t _pin);

    void init();
    void read(int32_t *x);
    void tick(unsigned long current_millis);
};



#endif
