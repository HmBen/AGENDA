#include <SPI.h>
#include "SdFat.h"
#include "RingBuf.h"

#include "rng.h"
#include "adxl346.h"
#include "ads1299.h"

extern "C" uint32_t set_arm_clock(uint32_t frequency);

// @SETTING RECORDING LENGTH SECONDS, 3600 seconds per hour, set number of hours
#define REC_SEC 30

// SD Card data logger settings
// Size to log 8 ch LFP, 3 ch accel, 1 ch sync at 1kHz for 24 hours: (12 *4) * (1000 * 3600 * 24) bytes
#define LOG_FILE_SIZE 2E9


// Space to hold 530 ms of data, in case writing to SD card stalls
#define RING_BUF_CAPACITY 50 * 512
#define LOG_FILENAME "SdioLogger.bin"

SdFs sd;
FsFile file;

// RingBuf for File type FsFile.
RingBuf<FsFile, RING_BUF_CAPACITY> rb;

// Synchronissation signal
const int SYNC = 0;

// SYNC pulse seed, ON length (ms), OFF interval minimum (ms), maximum (ms), IO pin
RNG rng(0x12345678, 100, 500, 2000, SYNC);

// ADXL settings
const int ADXL_CS = 10;
ADXL346 ADXL(ADXL_CS);

// ADS1299 settings
const int ADS1299_CS1 = 7;
const int ADS1299_CS2 = 6;
const int ADS1299_CS3 = 5;
const int START = 15;
//const int N_DRDY = 17; // v1
const int N_DRDY = 22; // v2
const int N_PWDN = 14;

ADS1299 ADS(ADS1299_CS1);
ADS1299 ADS2(ADS1299_CS2);
ADS1299 ADS3(ADS1299_CS3);

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

int32_t buffer[2*N_CH+3+1];
volatile bool ADS1299_data_ready = false;

volatile int accel_sampling_divider = 0;

void ADS1299_dataReady_ISR() {
  // ADS1299 samples
  ADS.read(&buffer[0]);
  ADS2.read(&buffer[N_CH]);
  //delay(50);
  ADS1299_data_ready = true;
  
  // Accelerometer samples
  if (accel_sampling_divider >= 0) {
    ADXL.read(&buffer[2*N_CH]);
    accel_sampling_divider = ((4000/200) - 1);
  } else {
    accel_sampling_divider--;
  }
  
  // Sync signal
  rng.read(&buffer[2*N_CH+3]);
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

void setup() {
  // Set CPU clock at ~150MHz
  set_arm_clock(151.2E6);

  // Serial
  Serial.begin(9600);
  SPI.begin();

  // Sync setup
  pinMode(SYNC, OUTPUT);
  digitalWriteFast(SYNC, LOW);
  rng.init();

  // ADXL Setup
  pinMode(ADXL_CS, OUTPUT);
  digitalWriteFast(ADXL_CS, HIGH);
  ADXL.init();

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
  ADS.RESET();
  ADS2.RESET();

  // Wait at least t(CLK) * 18 = 9us for reset to take effect
  delayMicroseconds(9);

  // Send SDATAC command (Device wakes up in RDATAC mode after reset)
  ADS.SDATAC();
  ADS2.SDATAC();

  // Read deivce ID
  Serial.print("Card 1: ");
  Serial.println(ADS.RREG(0x00));
  Serial.print("Card 2: ");
  Serial.println(ADS2.RREG(0x00));
  Serial.print("Card 3: ");
  Serial.println(ADS3.RREG(0x00));

  // @SETTING
  // Send command for internal reference
  //ADS.WREG(0x01, 0x90 | 0x20 | FS_1k); // CONFIG1, enable clock output, 1000Hz sampling rate
  ADS.WREG(0x01, 0x90 | FS_4k); // CONFIG1, 4000Hz sampling rate
  ADS2.WREG(0x01, 0x90 | FS_4k); // CONFIG1, 4000Hz sampling rate

  //*****************************************************************************************//
  //*****************************************************************************************//
  //*****************************************************************************************//

  ADS.WREG(0x03, 0xE0); // CONFIG3, enable internal reference buffer, diable BIAS measurement
  ADS2.WREG(0x03, 0xE0); // CONFIG3, enable internal reference buffer, diable BIAS measurement

  // Wait for internal reference to settle
  delay(10); // @TODO: measure minimum acceptable delay later

/*
  // @SETTING FOR RECORDING TEST
  
  // Send register settings
  ADS.WREG(0x02, 0xD0); // CONFIG2, enable test source at 1Hz, small amplitude

  ADS.WREG(0x05, GAIN_24x | TEST); // CH01SET
  ADS.WREG(0x06, GAIN_24x | TEST); // CH02SET
  ADS.WREG(0x07, GAIN_24x | TEST); // CH03SET
  ADS.WREG(0x08, GAIN_24x | TEST); // CH04SET
  ADS.WREG(0x09, GAIN_24x | TEST); // CH05SET
  ADS.WREG(0x0A, GAIN_24x | TEST); // CH06SET
  ADS.WREG(0x0B, GAIN_24x | TEST); // CH07SET
  ADS.WREG(0x0C, GAIN_24x | TEST); // CH08SET

  ADS2.WREG(0x02, 0xD0); // CONFIG2, enable test source at 1Hz, small amplitude

  ADS2.WREG(0x05, GAIN_24x | TEST); // CH09SET
  ADS2.WREG(0x06, GAIN_24x | TEST); // CH10SET
  ADS2.WREG(0x07, GAIN_24x | TEST); // CH11SET
  ADS2.WREG(0x08, GAIN_24x | TEST); // CH12SET
  ADS2.WREG(0x09, GAIN_24x | TEST); // CH13SET
  ADS2.WREG(0x0A, GAIN_24x | TEST); // CH14SET
  ADS2.WREG(0x0B, GAIN_24x | TEST); // CH15SET
  ADS2.WREG(0x0C, GAIN_24x | TEST); // CH16SET
*/
  
  // @SETTING FOR RECORDING REAL INPUT
  ADS.WREG(0x05, GAIN_24x | CHSET_INPUT); // CH01SET
  ADS.WREG(0x06, GAIN_24x | CHSET_INPUT); // CH02SET
  ADS.WREG(0x07, GAIN_24x | CHSET_INPUT); // CH03SET
  ADS.WREG(0x08, GAIN_24x | CHSET_INPUT); // CH04SET
  ADS.WREG(0x09, GAIN_24x | CHSET_INPUT); // CH05SET
  ADS.WREG(0x0A, GAIN_24x | CHSET_INPUT); // CH06SET
  ADS.WREG(0x0B, GAIN_24x | CHSET_INPUT); // CH07SET
  ADS.WREG(0x0C, GAIN_24x | CHSET_INPUT); // CH08SET

  ADS2.WREG(0x05, GAIN_24x | CHSET_INPUT); // CH09SET
  ADS2.WREG(0x06, GAIN_24x | CHSET_INPUT); // CH10SET
  ADS2.WREG(0x07, GAIN_24x | CHSET_INPUT); // CH11SET
  ADS2.WREG(0x08, GAIN_24x | CHSET_INPUT); // CH12SET
  ADS2.WREG(0x09, GAIN_24x | CHSET_INPUT); // CH13SET
  ADS2.WREG(0x0A, GAIN_24x | CHSET_INPUT); // CH14SET
  ADS2.WREG(0x0B, GAIN_24x | CHSET_INPUT); // CH15SET
  ADS2.WREG(0x0C, GAIN_24x | CHSET_INPUT); // CH16SET
  
/*
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

*/
  // Initialize the ring buffer
  rb.begin(&file);
  
  // Activate conversion
  digitalWrite(START, HIGH);

  
  // Read deivce ID
  Serial.print("Card 1: ");
  Serial.println(ADS.RREG(0x00));
  Serial.print("Card 2: ");
  Serial.println(ADS2.RREG(0x00));
  Serial.print("Card 3: ");
  Serial.println(ADS3.RREG(0x00));
  
  // Return device to RDATAC mode
  ADS.RDATAC();
  ADS2.RDATAC();

  
  // Read deivce ID
  Serial.print("Card 1: ");
  Serial.println(ADS.RREG(0x00));
  Serial.print("Card 2: ");
  Serial.println(ADS2.RREG(0x00));
  Serial.print("Card 3: ");
  Serial.println(ADS3.RREG(0x00));

  // ADS1299 Startup sequence DONE
  // -----------------------------

  Serial.println("Setup done");
  
  // @SETTING WAIT THIS LONG BEFORE RECORDING (milliseconds)
  delay(3000);
  
  Serial.println("Starting");
  attachInterrupt(digitalPinToInterrupt(N_DRDY), ADS1299_dataReady_ISR, FALLING);
}


void loop() {
  static int counter = 0;
  if (ADS1299_data_ready) {
    
    ADS1299_data_ready = false;
    for (int ch = 0; ch < 2*N_CH; ch++){
      if (ch == 0) {
          Serial.print("READINGS");
        }
        Serial.print(",");
        Serial.print(buffer[ch], 6);
        if (ch==15){
          Serial.println();
        }
  }
    /*
    if (counter == REC_SEC*4000 || logData(buffer, 2*N_CH+4)) {
      // Should end
      rb.sync();
      file.truncate();
      file.close();

      digitalWrite(SYNC, LOW); // Turn OFF SYNC LED
      digitalWrite(START, LOW); // Amplifier stops recording
      digitalWrite(N_PWDN, LOW); // Power down the amplifiers
      digitalWrite(13, LOW); // Turns OFF the Teensy LED
      Serial.println("Stopping");
      
      while(1);
    }
    */
    counter++;  
  }

  unsigned long current_millis = millis();

  rng.tick(current_millis);
}
