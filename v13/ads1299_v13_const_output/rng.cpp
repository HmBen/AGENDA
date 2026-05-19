#include "core_pins.h"
#include "rng.h"


RNG::RNG(uint32_t _seed, uint32_t _ON_LENGTH_MS, uint32_t _INTERVAL_MIN_MS, uint32_t _INTERVAL_MAX_MS, uint8_t _pin) {
  // Set parameters
  seed = _seed;
  ON_LENGTH_MS = _ON_LENGTH_MS;
  INTERVAL_MIN_MS = _INTERVAL_MIN_MS;
  INTERVAL_MAX_MS = _INTERVAL_MAX_MS;
  pin = _pin;

  // Initialise states
  xorshift_state = seed;
  previous_millis = 0;
  next_interval = _INTERVAL_MAX_MS;
  random_interval = 0;
  state = LOW;
  ready = 0;
}


void RNG::init() {
  xorshift_state = seed;
  state = LOW;
  ready = 0;
  pinMode(pin, OUTPUT);
  digitalWriteFast(pin, state);
}


void RNG::read(int32_t *x) {
  *x = state;
}


void RNG::tick(unsigned long current_millis) {
  if (current_millis - previous_millis >= next_interval) {
    previous_millis = current_millis;
    if (state == LOW) {
      state = HIGH;
      next_interval = ON_LENGTH_MS;
    } else {
      state = LOW;
      next_interval = random_interval - ON_LENGTH_MS;
      ready = 0; // we used up the random sample
    }
    digitalWriteFast(pin, state);
    // make sure this is run last, so does not delay timing
    if (!ready) {
      xorshift32();
      random_interval = INTERVAL_MIN_MS + ((((uint64_t) xorshift_state) * (INTERVAL_MAX_MS - INTERVAL_MIN_MS)) >> 32);
      ready = 1;
    }
  }
}


// Algorithm "xor" from p. 4 of Marsaglia, "Xorshift RNGs"
void RNG::xorshift32() {
  uint32_t x = xorshift_state;
  x ^= x << 13;
  x ^= x >> 17;
  x ^= x << 5;
  xorshift_state = x;
}
