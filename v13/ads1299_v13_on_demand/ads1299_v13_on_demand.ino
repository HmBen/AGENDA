#include <SPI.h>
#include "SdFat.h"
#include "RingBuf.h"

#include "rng.h"
#include "adxl346.h"
#include "ads1299.h"

#include <RV3028C7.h>
#include <string>
RV3028C7 rtc;

extern "C" uint32_t set_arm_clock(uint32_t frequency);

// @SETTING RECORDING LENGTH SECONDS, 3600 seconds per hour, set number of hours
#define REC_SEC 3600
#define REC_SEC2 30


// SD Card data logger settings
// Size to log 8 ch LFP, 3 ch accel, 1 ch sync at 1kHz for 24 hours: (12 *4) * (1000 * 3600 * 24) bytes
#define LOG_FILE_SIZE 2E9

// --- Global State and Communication Variables ---
bool isCheckingImpedance = false;
bool startRec =false;
bool isRecording=false;
static int no_data_counter=0;
volatile bool newCommandReceived = false;
String commandFromISR = "";
uint8_t dataBuffer[512];


// Space to hold 530 ms of data, in case writing to SD card stalls
#define RING_BUF_CAPACITY 50 * 512
char LOG_FILENAME[32]= "SdioLogger1.bin";

SdFs sd;
FsFile file;
FsFile file2;

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

//Define Lead-off detection registers
#define LOFF (0x04)
#define LOFF_SENSP (0x0F)
#define LOFF_SENSN (0x10)
#define LOFF_FLIP (0x11)
#define LOFF_STATP (0x12)
#define LOFF_STATN (0x13)
#define CONFIG4 (0x17)

#define FLEAD_OFF (0x3)//AC lead of detection at Fdr/4
#define ILEAD_OFF_6n (0x0)//6nA
#define ILEAD_OFF_24n (0x4)//24nA
#define ILEAD_OFF_6u (0x8)//6uA
#define ILEAD_OFF_24u (0xC)//24uA

//for BPF
#define MWSPT_NSEC 3
static float filter_buffer_x[2*N_CH][3] = {0};  // Input delay line - previous 3 inputs
static float filter_buffer_y[2*N_CH][3] = {0};  // Output delay line - previous 3 outputs
int curr_min[2*N_CH]={1000,1000,1000,1000,1000,1000,1000,1000};
int curr_max[2*N_CH]={-1000,-1000,-1000,-1000,-1000,-1000,-1000,-1000};
int curr_imp[2*N_CH]={0};
int curr_v;
int filtered_signal;

void ADS1299_dataReady_ISR() {
  // ADS1299 samples
  ADS.read(&buffer[0]);
  ADS2.read(&buffer[N_CH]);
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

int logData(int32_t* x, int frame_size, FsFile file) {
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

//for LOD
void LOD(byte amplitudeCode, byte freqCode){

	amplitudeCode &= 0b00001100;  //only these two bits should be used - 24uA
	freqCode &= 0b00000011;  //only these two bits should be used- AC LOD at Fdr/4
	
	//get the current configuration of he byte
	byte reg, config, config2;
	reg = LOFF;
	config = ADS.RREG(reg); //get the current bias settings for each device
  config2 = ADS2.RREG(reg);
	
	//reconfigure the byte to get what we want
	config &= 0b11110000;  //clear out the last four bits
	config |= amplitudeCode;  //set the amplitude
	config |= freqCode;    //set the frequency

  config2 &= 0b11110000;  //clear out the last four bits
	config2 |= amplitudeCode;  //set the amplitude
	config2 |= freqCode;    //set the frequency
	
	//send the config byte back to the hardware
	ADS.WREG(reg,config);
  delay(1);
  ADS2.WREG(reg,config);
  delay(1);  //send the modified byte back to the ADS
	
}

void configure_channel(int code_OFF_ON){
  //write to change only that bit N in LOFF_SENSN and LOFF_SENSP register (N-1 corresponds to the channel)
  //to configure_channel for LOFF, code_OFF_ON == 1.
  //to turn off channel for LOFF, code_OFF_ON ==0.
  byte reg, config, config2;
	
  //check the inputs
  /*
  if ((N < 1) || (N > 2*N_CH)) return;
  N = constrain(N-1,0,2*N_CH-1);  //shift down by one
  */
  
  //proceed...first, disable any data collection
  ADS.SDATAC(); 
  delay(1);
  ADS2.SDATAC();
  delay(1);      // exit Read Data Continuous mode to communicate with ADS

  //Set LOFF_SENSP for channel
  reg = LOFF_SENSP;  
  config = ADS.RREG(reg); //get the current lead-off settings
  config2 = ADS2.RREG(reg);
  for(int ch=0;ch<2*N_CH;ch++){
    if (code_OFF_ON == 0) {
        bitClear(config,ch);        //clear this channel's bit to turn off lead-off signal on this channels postive side
        bitClear(config2,ch);
    } else {
        bitSet(config,ch); 			  //clear this channel's bit
        bitSet(config2,ch); 
    }
  }
  ADS.WREG(reg, config);

  delay(1);
  ADS2.WREG(reg, config2);

  delay(1);  //send the modified byte back to the ADS

  //Set LOFF_SENSN for channel
  reg = LOFF_SENSN;  //are we using the P inptus or the N inputs?
  config = ADS.RREG(reg); //get the current lead-off settings
  config2 = ADS2.RREG(reg);
  for(int ch=0;ch<2*N_CH;ch++){
    if (code_OFF_ON == 0) {
        bitClear(config,ch);        //clear this channel's bit
        bitClear(config2,ch);
    } else {
        bitSet(config,ch); 			  //clear this channel's bit
        bitSet(config2,ch);
    }     
  }      //set this channel's bit
  ADS.WREG(reg,config);
  delay(1);
  ADS2.WREG(reg,config2);  
  delay(1);  //send the modified byte back to the ADS

  //RDATAC(CS); //continue data collection
  
}

void setup() {
  // Set CPU clock at ~150MHz
  set_arm_clock(151.2E6);

  // Serial
  Serial.begin(9600);
  delay(2000);
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
  

  //Set up Lead off detection
  LOD(ILEAD_OFF_24n, FLEAD_OFF);
  ADS.WREG(LOFF_FLIP, 0x0);//Use default current direction.
  ADS2.WREG(LOFF_FLIP, 0x0);//Use default current direction.

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
  ADS.RDATAC();
  ADS2.RDATAC();

  // ADS1299 Startup sequence DONE
  // -----------------------------

  Serial.println("Setup done");
  
  // @SETTING WAIT THIS LONG BEFORE RECORDING (milliseconds)
  delay(3000);
  
  Serial.println("Starting");
  attachInterrupt(digitalPinToInterrupt(N_DRDY), ADS1299_dataReady_ISR, FALLING);
}

void deleteFile(SdFs *sd, const char* fileName) {
  if (!sd->exists(fileName)) {
    Serial.print("[ERROR] File does not exist\n");
    return;
  }
  if (!sd->remove(fileName)) {
    Serial.print("[ERROR] Failed to delete file\n");
    sdError(sd);
  }
  else { // Success
    Serial.print("Delete done\n");
  }
}

void sendFile(SdFs *sd, const char* fileName) {
  if (!sd->exists(fileName)) {
    Serial.print("[ERROR] File does not exist\n");
    return;
  }
  FsFile file = sd->open(fileName, O_RDONLY);
  if (!file) {
    Serial.println("[ERROR] Could not open file\n");
    return;
  }
  
  // Notify receiver of file size
  Serial.print("Size: ");
  Serial.print(file.fileSize());
  Serial.print("\n");
  Serial.flush();
  
  while(true) {
    // Handle serial input
    if (Serial.available()) {
      String command = Serial.readStringUntil('\n');
      command.trim();
      serialClear();
      if (command == "ABORT") {
        break;
      }
    }
    // Attempt to read one packet of data from file
    uint32_t bytesRead = file.read(dataBuffer, sizeof(dataBuffer));
    // If read was successful, write out data
    if (bytesRead > 0) {
      Serial.write(dataBuffer, bytesRead);
    }
    else {
      break;
    }
  }
  Serial.flush();
  file.close();
}

void printFileList() {
  Serial.println("Sending list of files");
  FsFile root;
  FsFile file;
  // Open root directory
  if (!root.open("/")) {
    sdError(&sd);
    return;
  }
  // Report number of files available
  int rootFileCount = 0;
  while (file.openNext(&root, O_RDONLY)) {
    if (file.isFile() && !file.isHidden()) {
      rootFileCount++;
    }
    file.close();
  }
  Serial.print("Files found: ");
  Serial.print(rootFileCount);
  Serial.print("\n");
  // If files available, print file info
  if (rootFileCount > 0) {
    root.rewind();
    while (file.openNext(&root, O_RDONLY)) {
      if (file.isFile() && !file.isHidden()) {
        file.printName(&Serial);
        Serial.print(",");
        Serial.print(file.fileSize());
        Serial.print(",");
        file.printCreateDateTime(&Serial);
        Serial.print("\n");
      }
      file.close();
    }
    Serial.println("LIST_DONE");
  }
  root.close();
}

void serialClear() {
  while (Serial.available()) {
    Serial.read();
  }
}

void sdError(SdFs *fs) {
  // Delay to trigger error mode in receiver
  delay(1250);
  // Send information on error
  Serial.print("[SdFat Error] Code: ");
  Serial.print(sd.sdErrorCode());
  Serial.print("\n");
}
int32_t filter_sig(int32_t curr_v, int ch) {

  //2nd order chebII IIR filter coeffs (single stage)
  const float NUM[MWSPT_NSEC] = { 1, 0, -1 };
  const float DEN[MWSPT_NSEC] = { 1, -1.187919512118e-16, 0.9390625058175 };

  //add current input to prev inputs array
  filter_buffer_x[ch][2] = filter_buffer_x[ch][1];
  filter_buffer_x[ch][1] = filter_buffer_x[ch][0];
  filter_buffer_x[ch][0] = curr_v * 0.03046874709125;  //input gain

  //differential eqn
  // y(n) = a0*x0+a1*x1+a2*x2-d1*y0-d2*y1
  float output = NUM[0] * filter_buffer_x[ch][0] + NUM[1] * filter_buffer_x[ch][1] + NUM[2] * filter_buffer_x[ch][2] - DEN[1] * filter_buffer_y[ch][0] - DEN[2] * filter_buffer_y[ch][1];

  //add current output to prev output array
  filter_buffer_y[ch][2] = filter_buffer_y[ch][1];
  filter_buffer_y[ch][1] = filter_buffer_y[ch][0];
  filter_buffer_y[ch][0] = output;

  return output;
}

void update_and_send_impedance(int32_t* buffer, int counter) {
    for (int ch = 0; ch < 2*N_CH; ch++) {
    //curr_v= buffer[ch] *1000000* (4.5 / (24 * 8388608.0));
    curr_v = buffer[ch] * 1e6 * 2 * 4.5 / (24 * 16777216);
    //int filtered_signal = curr_v;  //add filter later
    int32_t filtered_signal = filter_sig(curr_v, ch);


    // impedance calc
    if (filtered_signal < curr_min[ch]) {  //current value less than local min
      curr_min[ch] = filtered_signal;
    }
    if (filtered_signal > curr_max[ch]) {  //current value greater than local max
      curr_max[ch] = filtered_signal;
    }
    
    if (counter % 4000 == 0) {
      if (ch == 0) {
        Serial.print("IMPEDANCE");
      }
      curr_imp[ch] = abs(curr_min[ch] - curr_max[ch]) / 240;  // /(current*10)
      Serial.print(",");
      Serial.print(curr_imp[ch], 6);
      if (ch==15){
        Serial.println();
      }
    }
    curr_min[ch] = 1000;
    curr_max[ch] = -1000;
  }
  
}

void setTimeFromSerial(unsigned long time) {
  // Set external RTC with UNIX time, synchronise calendar registers
  rtc.setUnixTimestamp(time, true);
  
  // Set both the Teensy's hardware (SRTC) and software clocks (RTC)
  rtc_set(time);

  // Done
  Serial.print("Time set\n");
}

void impedance_start(){
  configure_channel(1); 
  ADS.RDATAC();
  ADS2.RDATAC();
}

void impedance_stop(){

  configure_channel(0);
  ADS.RDATAC();
  ADS2.RDATAC();
}

bool check_for_imp_signal(){
  String command = "";
  //Serial.println("Checking for signal");
  if (Serial.available()) {
    Serial.println("Command received in ISR");
    command = Serial.readStringUntil('\n');
    command.trim();
    if (command == "IMPEDANCE_START") {
      Serial.println("IMP_START_ACK");
      isCheckingImpedance = true;
      impedance_start();
      }
    else if (command == "IMPEDANCE_STOP") {
      Serial.println("IMP_STOP_ACK");
      isCheckingImpedance = false;
      impedance_stop();
    }  
  } 
  return isCheckingImpedance;
}

bool check_for_rec_signal(){
    String command = "";
  //Serial.println("Checking for signal");
  if (Serial.available()) {
    Serial.println("Command received");
    command = Serial.readStringUntil('\n');
    command.trim();
    if (command == "START_REC" && startRec!=true) { //only start recording if hasn't already started
      Serial.println("REC_START_ACK");
      startRec = true;
    } 
  } 
  return startRec;
}


void loop() {
  static int counter = 0;
  String command = "";
  if (Serial.available()) {
      Serial.println("Command received");
      command = Serial.readStringUntil('\n');
      command.trim();
      if (command == "IMPEDANCE_START") {
        Serial.println("IMP_START_ACK");
        isCheckingImpedance = true;
        impedance_start();
      }
      else if (command == "IMPEDANCE_STOP") {
        Serial.println("IMP_STOP_ACK");
        isCheckingImpedance = false;
        impedance_stop();
      }
      if (command == "START_REC" && startRec!=true) { //only start recording if hasn't already started
        Serial.println("REC_START_ACK");
        startRec = true;
      }   
      else if (command == "LIST") {
      Serial.println("LIST_ACK");
      printFileList();
      }
      else if (command.startsWith("REQUEST")) {
        Serial.println("REQUEST_ACK");
        if (command.length() <= 8) {
          Serial.print("[ERROR] File not specified\n");
         }
        else {
          command.remove(0, 8); // Remove command to get name
          sendFile(&sd, command.c_str());
        }
      }
      else if (command.startsWith("DELETE")) {
        Serial.println("DELETE_ACK");
        if (command.length() <= 7) {
         Serial.print("[ERROR] File not specified\n");
        }
        else {
        command.remove(0, 7); // Remove command to get name
        deleteFile(&sd, command.c_str());
        }
      }
      else if (command.startsWith("TIME")) {
        if (command.length() <= 5) {
         Serial.print("[ERROR] Time not specified\n");
        }
        else {
          command.remove(0, 5); // Remove command to get UNIX time
          setTimeFromSerial(std::stoul(command.c_str()));
        }
      }
    }   
  if (startRec == true) {

    //close impedance checking file
    // impedance_stop();
    Serial.println("Closing impedance file");
    rb.sync();
    file.truncate();
    file.close();
    delay(500);

    // start new file for recording

    while (!sd.begin(SdioConfig(FIFO_SDIO))) {
      Serial.println("Card not present");
      delay(1000);  // 1s
    }
    //create new file
    int file_num = 3;

    snprintf(LOG_FILENAME, sizeof LOG_FILENAME, "SdioLogger%d.bin", file_num);
    if (!file2.open(LOG_FILENAME, O_RDWR | O_CREAT | O_TRUNC)) {
      Serial.println("File open failed");
      return;  // @TODO
    }

    // File must be pre-allocated to avoid huge
    // delays searching for free clusters
    if (!file2.preAllocate(LOG_FILE_SIZE)) {
      Serial.println("File preallocation failed");
      file2.close();
      return;  // @TODO
    }

    Serial.println("Opened recording file");

    //reset counters
    counter = 0;
    //reset ring buffer to initial state
    rb.begin(&file2);
    isRecording = 1;
    startRec = false;
  }
  
  if (ADS1299_data_ready) {
    ADS1299_data_ready = false;

    if (counter==1){
       Serial.println("DATA READY");
    }
    
    if (isCheckingImpedance==1 && isRecording==0){
      update_and_send_impedance(buffer, counter);
    }
    if (startRec==true){

      //close impedance checking file
        Serial.println("Closing impedance file");
        rb.sync();
        file.truncate();
        file.close();
        delay(500);

      // start new file for recording

        while (!sd.begin(SdioConfig(FIFO_SDIO))) {
          Serial.println("Card not present");
          delay(1000); // 1s
        }
        //create new file
        int file_num=3;
        
        snprintf(LOG_FILENAME, sizeof LOG_FILENAME, "SdioLogger%d.bin", file_num);
        if (!file2.open(LOG_FILENAME, O_RDWR | O_CREAT | O_TRUNC)) {
          Serial.println("File open failed");
          return; // @TODO
        }

        // File must be pre-allocated to avoid huge
        // delays searching for free clusters
        if (!file2.preAllocate(LOG_FILE_SIZE)) {
          Serial.println("File preallocation failed");
          file2.close();
          return; // @TODO
        }        
        
        Serial.println("Opened recording file");

        //reset counters
        counter=0;
        //reset ring buffer to initial state
        rb.begin(&file2);
        isRecording=1;
        startRec=false;
    }
    if (isRecording==1){
      if (counter == REC_SEC2*4000 || logData(buffer, 2*N_CH+4, file2)) {
        // Should end
        rb.sync();
        file2.truncate();
        file2.close();

        digitalWrite(SYNC, LOW); // Turn OFF SYNC LED
        digitalWrite(START, LOW); // Amplifier stops recording
        digitalWrite(N_PWDN, LOW); // Power down the amplifiers
        digitalWrite(13, LOW); // Turns OFF the Teensy LED
        Serial.println("Stopping");
        
        while(1);
      }
    }
    counter++;
  }

  unsigned long current_millis = millis();

  rng.tick(current_millis);
}
