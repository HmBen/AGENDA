#include <SPI.h>
#include "SdFat.h"
#include "RingBuf.h"
#include "xorshift.h"
#include "math.h" // implementation is platform dependent
// #include "arm_math.h" // from ARM, has FFT
// #include "filters.h"

extern "C" uint32_t set_arm_clock(uint32_t frequency);

// @SETTING RECORDING LENGTH SECONDS, 3600 seconds per hour, set number of hours
#define REC_SEC 120
// #define REC_SEC 1200 // 20 minutes
// #define REC_SEC 28800 // 8 hours
// #define REC_SEC 57600 // 16 hours

// SD Card data logger settings
// Size to log 8 ch LFP, 3 ch accel, 1 ch sync at 1kHz for 24 hours: (12 * 4) * (1000 * 3600 * 24) bytes
#define LOG_FILE_SIZE 4147200000


// Space to hold 530 ms of data, in case writing to SD card stalls
#define RING_BUF_CAPACITY 50 * 512
#define LOG_FILENAME "SdioLogger.bin"

SdFs sd;
FsFile file;

// RingBuf for File type FsFile.
RingBuf<FsFile, RING_BUF_CAPACITY> rb;

// Synchronissation signal
const int SYNC = 0;

// Defines the SYNC pulse seed, timing of pulse, min and max in msec
#define RNG_SEED 0x12345678
#define ON_LENGTH_MS 100
#define INTERVAL_MIN_MS 500
#define INTERVAL_MAX_MS 2000

uint32_t xorshift_state = RNG_SEED;
uint32_t previous_millis = 0;
uint8_t  out_state = LOW;
uint32_t next_interval = 1000; // ms
uint32_t random_interval = 0;
uint8_t  rng_ready = 0;

// ADXL settings
SPISettings ADXL_settings(5E6, MSBFIRST, SPI_MODE3);

const int ADXL_CS = 10;

// Register addresses: 0x0B
const uint8_t BW_RATE     = 0x2C;  // Data rate and power mode control
const uint8_t POWER_CTL   = 0x2D;  // Power-saving features control
const uint8_t DATA_FORMAT = 0x31;  // Data format control
const uint8_t DATAX0      = 0x32;  // X-Axis Data 0
const uint8_t DATAX1      = 0x33;  // X-Axis Data 1
const uint8_t DATAY0      = 0x34;  // Y-Axis Data 0
const uint8_t DATAY1      = 0x35;  // Y-Axis Data 1
const uint8_t DATAZ0      = 0x36;  // Z-Axis Data 0
const uint8_t DATAZ1      = 0x37;  // Z-Axis Data 1

// Register access modes
const uint8_t ADXL_WRITE     = 0x00;
const uint8_t ADXL_READ      = 0x80;
const uint8_t ADXL_READ_MULT = 0xC0;

void ADXL_init(void);
void ADXL_read(int16_t *x, int16_t *y, int16_t *z);

// ADS1299 settings

SPISettings ADS1299_SPI_settings(20E6, MSBFIRST, SPI_MODE1);

const int FS = 1000;

const int ADS1299_CS1 = 7;
const int ADS1299_CS2 = 6;
const int ADS1299_CS3 = 5;
const int START = 15;
//const int N_DRDY = 17; // v1
const int N_DRDY = 22; // v2
const int N_PWDN = 14;

const int N_CH = 8;

const int GAIN_1x =  0x00; // step: 536nV, range: +/- 4.500V
const int GAIN_2x =  0x10; // step: 268nV, range: +/- 2.250V
const int GAIN_4x =  0x20; // step: 134nV, range: +/- 1.125V
const int GAIN_6x =  0x30; // step:  89nV, range: +/- 0.750V
const int GAIN_8x =  0x40; // step:  67nV, range: +/- 0.563V
const int GAIN_12x = 0x50; // step:  45nV, range: +/- 0.375V
const int GAIN_24x = 0x60; // step:  22nV, range: +/- 0.188V

const int FS_250 = 0x06;
const int FS_500 = 0x05;
const int FS_1k =  0x04;
const int FS_2k =  0x03;
const int FS_4k =  0x02;
const int FS_8k =  0x01;
const int FS_16k = 0x00;

const int CHSET_INPUT = 0x00;
const int SHORT = 0x01;
const int TEST =  0x05;

const int SRB1_CLOSED = 0x20;

int32_t buffer[N_CH+3+1];
volatile bool ADS1299_data_ready = false;

volatile int accel_sampling_divider = 0;

// Impedance measurement, assumes Fs=1kHz
#define SAMPLE_SIZE 20  // samples per Fourier integral
#define IMP_RATE 20     // impedance measurement rate in seconds

volatile bool imp_ready = false;
volatile int imp_sampling_divider = IMP_RATE*FS - SAMPLE_SIZE;
volatile int fourier_count = 0;

int32_t cos_accum[N_CH];
int32_t sin_accum[N_CH];

// values corresponding to SAMPLE_SIZE 20
#define IMP_CALC_OFFSET 198.7231f
#define IMP_CALC_SLOPE 0.009418f

float fast_sqrt_f32(float x) {
  float returnValue;
  __asm__("VSQRT.F32 %0, %1" : "=t" (returnValue) : "t" (x));
  return returnValue;
}

void ADS1299_dataReady_ISR() {

    SPI.beginTransaction(ADS1299_SPI_settings);
    delayNanoseconds(10);
    digitalWriteFast(ADS1299_CS1, LOW);
    delayNanoseconds(6); // CS low to first serial clock, at DVDD > 2.7V
    SPI.transfer(0x00, 3); // Skip status registers
    for(int i = 0; i < N_CH; i++) {
      int32_t sample = 0;
      sample = SPI.transfer(0x00);
      sample = (sample<<8) | SPI.transfer(0x00);
      sample = (sample<<8) | SPI.transfer(0x00);
      sample = (sample & 0x800000) ? (sample | 0xFF000000) : sample; // Sign
      buffer[i] = sample;
    }
    delayMicroseconds(2); // Final serial clock falling edge to CS high: t(CLK) * 4 = 2us
    digitalWriteFast(ADS1299_CS1, HIGH);
    SPI.endTransaction();
    ADS1299_data_ready = true;

    // Impedance calculation
    if (imp_sampling_divider == 0) {
      for(int j = 0; j < N_CH; j++) {
        switch (fourier_count % 4) {
          case 0:
            sin_accum[j] += buffer[j];
            break;
          case 1:
            cos_accum[j] -= buffer[j];
            break;
          case 2:
            sin_accum[j] -= buffer[j];
            break;
          case 3:
            cos_accum[j] += buffer[j];
            break;
        }
      }
      fourier_count++;
      if ((fourier_count % SAMPLE_SIZE) == 0) {
        fourier_count = 0;
        imp_sampling_divider = IMP_RATE*FS - SAMPLE_SIZE;
        imp_ready = true;
      }
    }
    else {
      imp_sampling_divider--;
    }

    // Accelerometer samples
    if (accel_sampling_divider == 0) {
      int16_t x, y, z;
      ADXL_read(&x, &y, &z);
      buffer[N_CH+0] = (int32_t) x;
      buffer[N_CH+1] = (int32_t) y;
      buffer[N_CH+2] = (int32_t) z;
      accel_sampling_divider = ((FS/200) - 1);
    } else {
      accel_sampling_divider--;
    }

    // Sync signal
    buffer[N_CH+3] = out_state;
}

void RESET(const int CS) {
  SPI.beginTransaction(ADS1299_SPI_settings);
  digitalWrite(CS, LOW);
  delayNanoseconds(6); // CS low to first serial clock, at DVDD > 2.7V
  SPI.transfer(0x06); // RESET
  delayMicroseconds(2); // Final serial clock falling edge to CS high: t(CLK) * 4 = 2us
  digitalWrite(CS, HIGH);
  SPI.endTransaction();
}

void SDATAC(const int CS) {
  SPI.beginTransaction(ADS1299_SPI_settings);
  digitalWrite(CS, LOW);
  delayNanoseconds(6); // CS low to first serial clock, at DVDD > 2.7V
  SPI.transfer(0x11); // SDATAC
  delayMicroseconds(2); // Final serial clock falling edge to CS high: t(CLK) * 4 = 2us
  digitalWrite(CS, HIGH);
  SPI.endTransaction();
}

void RDATAC(const int CS) {
  SPI.beginTransaction(ADS1299_SPI_settings);
  digitalWrite(CS, LOW);
  delayNanoseconds(6); // CS low to first serial clock, at DVDD > 2.7V
  SPI.transfer(0x10); // RDATAC
  delayMicroseconds(2); // Final serial clock falling edge to CS high: t(CLK) * 4 = 2us
  digitalWrite(CS, HIGH);
  SPI.endTransaction();
}

void WREG(const int CS, uint8_t ADDRESS, uint8_t BYTE) {
  SPI.beginTransaction(ADS1299_SPI_settings);
  digitalWrite(CS, LOW);
  delayNanoseconds(6); // CS low to first serial clock, at DVDD > 2.7V
  SPI.transfer(ADDRESS + 0x40);
  delayMicroseconds(2); // Wait at least t(Serial decode) = t(CLK) * 4 = 2us between bytes
  SPI.transfer(0x00); // Notifying to write 1 byte
  delayMicroseconds(2); // Wait at least t(Serial decode) = t(CLK) * 4 = 2us between bytes
  SPI.transfer(BYTE);
  delayMicroseconds(2); // Final serial clock falling edge to CS high: t(CLK) * 4 = 2us
  digitalWrite(CS, HIGH);
  SPI.endTransaction();
}

uint8_t RREG(const int CS, uint8_t ADDRESS) {
  uint8_t val = 0;
  SPI.beginTransaction(ADS1299_SPI_settings);
  digitalWrite(CS, LOW);
  delayNanoseconds(6); // CS low to first serial clock, at DVDD > 2.7V
  SPI.transfer(ADDRESS + 0x20);
  delayMicroseconds(2); // Wait at least t(Serial decode) = t(CLK) * 4 = 2us between bytes
  SPI.transfer(0x00); // Requesting 1 byte
  delayMicroseconds(2); // Wait at least t(Serial decode) = t(CLK) * 4 = 2us between bytes
  val = SPI.transfer(0x00); // Produce clock for receiving data by transmitting empty byte
  delayMicroseconds(2); // Final serial clock falling edge to CS high: t(CLK) * 4 = 2us
  digitalWrite(CS, HIGH);
  SPI.endTransaction();
  return val;
}

int logData(int32_t* x, int frame_size) {
  // Amount of data in ringBuf.
  uint64_t n = rb.bytesUsed();

  // Less than one full frame's worth of space left in file
  if ((n + file.curPosition()) > ((uint64_t) LOG_FILE_SIZE - frame_size*4)) {
    return 1;
  }

  if (n >= 512 && !file.isBusy()) {
    // Not busy only allows one sector before possible busy wait
    // Write one sector from RingBuf to file
    if (512 != rb.writeOut(512)) {
      return 1;  // Writeout failed
    }
  }

  rb.write((uint8_t*) x, frame_size*4);

  // Check for write error from too few free bytes in RingBuf
  if (rb.getWriteError()) {
    Serial.println("rb write error");
    return 1;
  }

  // Logging completed, passed all checks
  return 0;
}

// BMK1
void impedance_start() {
  WREG(ADS1299_CS1, 0x0F, 0xFF);  // LOFFP ON FOR ALL CH
  WREG(ADS1299_CS1, 0x10, 0xFF);  // LOFFN ON FOR ALL CH
  WREG(ADS1299_CS1, 0x04, 0x07);  // LOFF 24nA fs/4
}

void impedance_stop() {
  WREG(ADS1299_CS1, 0x0F, 0x00);  // LOFFP OFF FOR ALL CH
  WREG(ADS1299_CS1, 0x10, 0x00);  // LOFFN OFF FOR ALL CH
  WREG(ADS1299_CS1, 0x04, 0x03);  // LOFF 6nA fs/4
  delay(10);
}

void setup() {
  // Set CPU clock at ~150MHz
  set_arm_clock(151.2E6);

  // Serial
  Serial.begin(9600);
  SPI.begin();

  // Sync setup
  pinMode(SYNC, OUTPUT);
  digitalWriteFast(SYNC, LOW);

  // ADXL Setup
  pinMode(ADXL_CS, OUTPUT);
  digitalWriteFast(ADXL_CS, HIGH);
  ADXL_init();

  // ADS1299 Setup

  // 
  pinMode(START, OUTPUT);
  digitalWrite(START, LOW);
  pinMode(N_PWDN, OUTPUT);
  digitalWrite(N_PWDN, LOW);

  //
  pinMode(N_DRDY, INPUT);

  // SPI
  pinMode(ADS1299_CS1, OUTPUT);
  pinMode(ADS1299_CS2, OUTPUT);
  pinMode(ADS1299_CS3, OUTPUT);
  digitalWriteFast(ADS1299_CS1, HIGH);
  digitalWriteFast(ADS1299_CS2, HIGH);
  digitalWriteFast(ADS1299_CS3, HIGH);

  // ------------------------------
  // ADS1299 Startup sequence BEGIN

  // Set N_PWDN = 1 and N_RESET = 1
  digitalWrite(N_PWDN, HIGH);

  // Wait at least t(Power on Reset) = t(CLK) * 2^18 = 128ms
  delay(128); 

  // Issue reset command
  RESET(ADS1299_CS1);

  // Wait at least t(CLK) * 18 = 9us for reset to take effect
  delayMicroseconds(9);

  // Send SDATAC command (Device wakes up in RDATAC mode after reset)
  SDATAC(ADS1299_CS1);

  // @SETTING
  // Send command for internal reference
  // WREG(ADS1299_CS1, 0x01, 0x90 | 0x20 | FS_4k); // CONFIG1, enable clock output, 4000Hz sampling rate
  WREG(ADS1299_CS1, 0x01, 0x90 | 0x20 | FS_1k); // CONFIG1, enable clock output, 1000Hz sampling rate

  WREG(ADS1299_CS1, 0x03, 0xE0); // CONFIG3, enable internal reference buffer, diable BIAS measurement

  // Wait for internal reference to settle
  delay(10); // @TODO: measure minimum acceptable delay later

  // Read deivce ID
  Serial.println(RREG(ADS1299_CS1, 0x00));
  /*
  // @SETTING FOR RECORDING TEST
  // Send register settings
  WREG(ADS1299_CS1, 0x02, 0xD0); // CONFIG2, enable test source at 1Hz, small amplitude

  WREG(ADS1299_CS1, 0x05, GAIN_24x | TEST); // CH1SET
  WREG(ADS1299_CS1, 0x06, GAIN_24x | TEST); // CH2SET
  WREG(ADS1299_CS1, 0x07, GAIN_24x | TEST); // CH3SET
  WREG(ADS1299_CS1, 0x08, GAIN_24x | TEST); // CH4SET
  WREG(ADS1299_CS1, 0x09, GAIN_24x | TEST); // CH5SET
  WREG(ADS1299_CS1, 0x0A, GAIN_24x | TEST); // CH6SET
  WREG(ADS1299_CS1, 0x0B, GAIN_24x | TEST); // CH7SET
  WREG(ADS1299_CS1, 0x0C, GAIN_24x | TEST); // CH8SET
  */

  // @SETTING FOR RECORDING REAL INPUT
  WREG(ADS1299_CS1, 0x05, GAIN_24x | CHSET_INPUT); // CH1SET
  WREG(ADS1299_CS1, 0x06, GAIN_24x | CHSET_INPUT); // CH2SET
  WREG(ADS1299_CS1, 0x07, GAIN_24x | CHSET_INPUT); // CH3SET
  WREG(ADS1299_CS1, 0x08, GAIN_24x | CHSET_INPUT); // CH4SET
  WREG(ADS1299_CS1, 0x09, GAIN_24x | CHSET_INPUT); // CH5SET
  WREG(ADS1299_CS1, 0x0A, GAIN_24x | CHSET_INPUT); // CH6SET
  WREG(ADS1299_CS1, 0x0B, GAIN_24x | CHSET_INPUT); // CH7SET
  WREG(ADS1299_CS1, 0x0C, GAIN_24x | CHSET_INPUT); // CH8SET

  // SD Card
  while (!sd.begin(SdioConfig(FIFO_SDIO))) {
    Serial.println("Card not present");
    delay(1000); // 1s
  }

  // Open or create file - truncate existing file
  if (!file.open(LOG_FILENAME, O_RDWR | O_CREAT | O_TRUNC)) {
    Serial.println("File open failed");
    return; // @TODO
  }
  // File must be pre-allocated to avoid huge
  // delays searching for free clusters
  if (!file.preAllocate(LOG_FILE_SIZE)) {
    Serial.println("File preallocation failed");
    file.close();
    return; // @TODO
  }
  // Initialize the ring buffer
  rb.begin(&file);
  
  // Activate conversion
  digitalWrite(START, HIGH);
  
  // Return device to RDATAC mode
  RDATAC(ADS1299_CS1);

  // ADS1299 Startup sequence DONE
  // -----------------------------

  Serial.println("Impedance test fs/4 all Ps and Ns 24nA, fs=1kHz values in ohms 1 min");
  // Serial.println("Impedance test DC single P and N 6uA, fs=1kHz");
  Serial.println("Setup done");
  
  // @SETTING WAIT THIS LONG BEFORE RECORDING (milliseconds)
  delay(3000);
  
  // test impedance
  impedance_start();
  imp_ready = false;
  imp_sampling_divider = IMP_RATE*FS - SAMPLE_SIZE;
  for(int j = 0; j < N_CH; j++) {
    sin_accum[j] = 0;
    cos_accum[j] = 0;
  }

  Serial.println("Starting");
  attachInterrupt(digitalPinToInterrupt(N_DRDY), ADS1299_dataReady_ISR, FALLING);
}


void ADXL_init() {
  SPI.beginTransaction(ADXL_settings);

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

void ADXL_read(int16_t *x, int16_t *y, int16_t *z) {
  SPI.beginTransaction(ADXL_settings);
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

void loop() {

  float fourier_mag = 0.0f;
  int j = 0;
  static int counter = 0;

  if (ADS1299_data_ready) {
    ADS1299_data_ready = false;
    if (counter == REC_SEC*FS || logData(buffer, N_CH+4)) {
  
      // turn off impedance test
      impedance_stop();

      // Should end
      rb.sync();
      file.truncate();
      file.close();

      digitalWrite(SYNC, LOW); // Turn OFF SYNC LED
      digitalWrite(START, LOW); // Amplifier stops recording
      digitalWrite(N_PWDN, LOW); // Power down the amplifiers
      digitalWrite(13, LOW); // Turns OFF the Teensy LED
      Serial.println("Stopping");
      Serial.print("Counter = ");
      Serial.println(counter);

      while(1);
    }
    else {
      if (imp_ready) {
        // fourier_mag = powf((float) cos_accum, 2.0f) + powf((float) sin_accum, 2.0f);
        for(j = 0; j < N_CH; j++) {
          fourier_mag = ((float) cos_accum[j]) * ((float) cos_accum[j]);
          fourier_mag += ((float) sin_accum[j]) * ((float) sin_accum[j]);
          fourier_mag = fast_sqrt_f32(fourier_mag) / FS;
          Serial.print((fourier_mag - IMP_CALC_OFFSET)/IMP_CALC_SLOPE);
          Serial.print(", ");
          sin_accum[j] = 0;
          cos_accum[j] = 0;
        }
        Serial.println("");
        imp_ready = false;
      }
    }
    counter++;
  }

  unsigned long current_millis = millis();

  if (current_millis - previous_millis >= next_interval) {
    previous_millis = current_millis;
    if (out_state == LOW) {
      out_state = HIGH;
      next_interval = ON_LENGTH_MS;
    } else {
      out_state = LOW;
      next_interval = random_interval - ON_LENGTH_MS;
      rng_ready = 0; // we used up the random sample
    }
    digitalWrite(SYNC, out_state);
    // make sure this is run last, so does not delay timing
    if (!rng_ready) {
      xorshift32(&xorshift_state);
      random_interval = INTERVAL_MIN_MS + ((((uint64_t) xorshift_state) * (INTERVAL_MAX_MS - INTERVAL_MIN_MS)) >> 32);
      rng_ready = 1;
    }
  }
}
