/*
   This is a high speed data logger. It acquires data and stores it on an SD
   card. In my tests using a Teensy 3.6 and a SanDisk Ultra 16GB SDHC micro SD
   card, I was able to sample an analog pin at 25 kHz.

   It relies on the beta version of the SdFat library written by Bill Greiman,
   which is available at https://github.com/greiman/SdFat-beta. I have tested
   this sketch with revision ffbccb0 of SdFat-beta. This code was inpired by
   SdFat's LowLatencyLogger and TeensySdioDemo examples. It uses the same
   binary file format as LowLatencyLogger, but with a bigger block size.

   Here is how the code works. We have four buffers that are used to store the
   data. The main loop is very simple: it checks to see if there are any full
   buffers, and if there are, it writes them to the SD card. When is the data
   acquired? Data is acquired in the function yield(), which is called
   whenever the Teensy is not busy with something else. yield() is called by
   the main loop whenever there is nothing to write to the SD card. yield() is
   also called by the SdFat library whenever it is waiting for the SD card.
*/

#include "SdFat.h"
#include "Registers.h"
#include <i2c_t3.h>

#define SYNC_PIN 17
#define SYNC_RATE  50 // integer between 0 (no sync pulses) and 10000 (sync pulse on every cycle)
#define SYNC_MICROSECONDS 30
#define SYNC_REFRACTORY_MICROS 66666 // 1/15 of a second

#define GYRO_SCALE 3 // 0=250dps, 1=500dps, 2=1000dps, 3=2000dps
#define ACCEL_SCALE 1 // 0=2g, 1=4g, 2=8g, 3=16g

const bool USE_MAGNETOMETER = true;

// Pin to record
const int SENSOR_PIN = A0;

// Pin with LED, which flashes whenever data is written to card, and does a
// slow blink when recording has stopped.
const int LED_PIN = 13;

// Attach a button to this pin to stop recordings. Button should ground the pin when pressed.
// When not grounded, the pin is pulled up by an internal pull-up resistor.
const int BUTTON_PIN = 14;

// 16 KiB buffer.
const size_t BUF_DIM = 16384;

// Sampling rate
const uint32_t sampleIntervalMicros = 5000;
// 5000 us interval = 200 Hz

// Use total of four buffers.
const uint8_t BUFFER_BLOCK_COUNT = 4;

// Number of data points per record
const uint8_t IMU_DIM = 20;
const uint8_t ADC_DIM = IMU_DIM * 2 + 1;

// Format for one data record
struct data_t {
  uint32_t time;
  uint8_t adc[ADC_DIM];
};
// Warning! The Teensy allocates memory in chunks of 4 bytes!
// sizeof(data_t) will always be a multiple of 4. For example, the following
// data record will have a size of 12 bytes, not 9:
// struct data_t {
//   uint32_t time; // 4 bytes
//   uint8_t  adc[5]; // 5 bytes
// }

// Use SdFatSdio (not SdFatSdioEX) because SdFatSdio spends more time in
// yield(). For more information, see the TeensySdioDemo example from the
// SdFat library.
SdFatSdio sd;

File file;

// Number of data records in a block.
const uint16_t DATA_DIM = (BUF_DIM - 4) / sizeof(data_t);

//Compute fill so block size is BUF_DIM bytes.  FILL_DIM may be zero.
const uint16_t FILL_DIM = BUF_DIM - 4 - DATA_DIM * sizeof(data_t);

// Format for one block of data
struct block_t {
  uint16_t count;
  uint16_t overrun;
  data_t data[DATA_DIM];
  uint8_t fill[FILL_DIM];
};

// Intialize all buffers
block_t block[BUFFER_BLOCK_COUNT];

// Initialize full queue
const uint8_t QUEUE_DIM = BUFFER_BLOCK_COUNT + 1;

// Index of last queue location.
const uint8_t QUEUE_LAST = QUEUE_DIM - 1;

block_t* curBlock = 0;

block_t* emptyStack[BUFFER_BLOCK_COUNT];
uint8_t emptyTop;
uint8_t minTop;

block_t* fullQueue[QUEUE_DIM];
uint8_t fullHead = 0;
uint8_t fullTail = 0;

uint32_t nextSampleMicros = 0;
uint8_t syncNow;
uint32_t previousSyncMicros = 0;
unsigned long myrand;
bool fileIsClosing = false;
bool collectingData = false;
bool fileIsComplete = true;
bool isSampling = false;
bool justSampled = false;

//-----------------------------------------------------------------------------
void setup() {
  Serial.begin(9600);
  delay(1000);

  Wire1.begin();
  Wire1.setDefaultTimeout(10000);  // ten milliseconds

  pinMode(LED_PIN, OUTPUT);
  pinMode(BUTTON_PIN, INPUT_PULLUP);

  sd.begin();
}

void init() {
  fileIsClosing = false;
  fileIsComplete = false;
  collectingData = false;
  justSampled = false;
  isSampling = false;
  
  // Put all the buffers on the empty stack.
  for (int i = 0; i < BUFFER_BLOCK_COUNT; i++) {
    emptyStack[i] = &block[i - 1];
  }
  emptyTop = BUFFER_BLOCK_COUNT;

  char filename[13] = "tnsy0000.bin";
  int fileNum = 0;
  while (sd.exists(filename)) {
    fileNum++;
    if (fileNum >= 10000) {
      Serial.print("All filenames taken. Cannot create new file.");
      blinkForever();
    }
    filename[4] = fileNum / 1000 + 48;
    filename[5] = fileNum % 1000 / 100 + 48;
    filename[6] = fileNum % 100 / 10 + 48;
    filename[7] = fileNum % 10 + 48;
  }
  Serial.print("Writing to file ");
  Serial.println(filename);

  
  if (!file.open(filename, O_RDWR | O_CREAT)) {
    error("open failed");
  }

  // Initialize IMUs
  while (!(initMPU9250(MPU9250_ADDRESS_1) && initAK8963(MPU9250_ADDRESS_1) && initMPU9250(MPU9250_ADDRESS_0) && initAK8963(MPU9250_ADDRESS_0))) {
    Serial.print("Error code is ");
    Serial.println(Wire1.getError());
    Serial.print("Status is ");
    Serial.println(Wire1.status());
    Wire1.resetBus();
    digitalWrite(LED_PIN, HIGH);
    delay(200);
    digitalWrite(LED_PIN, LOW);
    delay(200);
    digitalWrite(LED_PIN, HIGH);
    delay(200);
    digitalWrite(LED_PIN, LOW);
    delay(1000);
  }
  pinMode(SYNC_PIN, OUTPUT);

  Serial.print("Block size: ");
  Serial.println(BUF_DIM);
  Serial.print("Record size: ");
  Serial.println(sizeof(data_t));
  Serial.print("Records per block: ");
  Serial.println(DATA_DIM);
  Serial.print("Record bytes per block: ");
  Serial.println(DATA_DIM * sizeof(data_t));
  Serial.print("Fill bytes per block: ");
  Serial.println(FILL_DIM);
  Serial.println("Recording. Enter any key to stop.");
  delay(100);
  fullHead = 0;
  fullTail = 0;
  curBlock = 0;
  nextSampleMicros = micros() + sampleIntervalMicros;
  collectingData = true;
}
//-----------------------------------------------------------------------------
void loop() {
  init();
  while (!fileIsComplete) {
    writeBuffers();
    fileIsClosing = fileIsClosing || Serial.available() || (digitalRead(BUTTON_PIN) == LOW);
  }
}
//-----------------------------------------------------------------------------
void writeBuffers() {
  // Write the block at the tail of the full queue to the SD card
  if (fullHead == fullTail) { // full queue is empty
    if (fileIsClosing) {
      file.close();
      Serial.println("File complete.");
      fileIsComplete = true;
    } else {
      yield(); // acquire data etc.
    }
  } else { // full queue not empty
    // write buffer at the tail of the full queue and return it to the top of
    // the empty stack.
    digitalWrite(LED_PIN, HIGH);
    block_t* pBlock = fullQueue[fullTail];
    fullTail = fullTail < QUEUE_LAST ? fullTail + 1 : 0;
    if ((int)BUF_DIM != file.write(pBlock, BUF_DIM)) {
      error("write failed");
    }
    emptyStack[emptyTop++] = pBlock;
    digitalWrite(LED_PIN, LOW);
  }
}
//-----------------------------------------------------------------------------
void yield() {
  // This does the data collection. It is called whenever the teensy is not
  // doing something else. The SdFat library will call this when it is waiting
  // for the SD card to do its thing, and the loop() function will call this
  // when there is nothing to be written to the SD card.

  if (!collectingData || isSampling)
    return;

  isSampling = true;

  // If file is closing, add the current buffer to the head of the full queue
  // and skip data collection.
  if (fileIsClosing) {
    if (curBlock != 0) {
      putCurrentBlock();
    }
    collectingData = false;
    isSampling = false;
    return;
  }

  // If we don't have a buffer for data, get one from the top of the empty
  // stack.
  if (curBlock == 0) {
    curBlock = getEmptyBlock();
  }

  // If it's time, record one data sample.
  if (micros() >= nextSampleMicros) {
    if (justSampled) {
      Serial.print("rate too fast");
      fileIsClosing = true;
    } else {
      acquireData(&curBlock->data[curBlock->count++]);
      nextSampleMicros += sampleIntervalMicros;
      justSampled = true;
    }
  } else {
    justSampled = false;
  }

  // If the current buffer is full, move it to the head of the full queue. We
  // will get a new buffer at the beginning of the next yield() call.
  if (curBlock->count == DATA_DIM) {
    putCurrentBlock();
  }

  isSampling = false;
}
//-----------------------------------------------------------------------------
block_t* getEmptyBlock() {
  /*
     Takes a block form the top of the empty stack and returns it
  */
  block_t* blk = 0;
  if (emptyTop > 0) { // if there is a buffer in the empty stack
    blk = emptyStack[--emptyTop];
    blk->count = 0;
  } else { // no buffers in empty stack
    error("All buffers in use");
  }
  return blk;
}
//-----------------------------------------------------------------------------
void putCurrentBlock() {
  /*
     Put the current block at the head of the queue to be written to card
  */
  fullQueue[fullHead] = curBlock;
  fullHead = fullHead < QUEUE_LAST ? fullHead + 1 : 0;
  curBlock = 0;
}
//-----------------------------------------------------------------------------
void error(String msg) {
  file.close();
  Serial.print("ERROR: ");
  Serial.println(msg);
  blinkForever();
}
//-----------------------------------------------------------------------------
void acquireData(data_t* data) {
  data->time = millis();
  readBytes(MPU9250_ADDRESS_0, ACCEL_XOUT_H, IMU_DIM, &data->adc[0]);
  readBytes(MPU9250_ADDRESS_1, ACCEL_XOUT_H, IMU_DIM, &data->adc[IMU_DIM]);
  // Emit sync pulses at random
  myrand = random(10000);
  syncNow = (myrand < SYNC_RATE) && ((micros() - previousSyncMicros) > SYNC_REFRACTORY_MICROS);
  if (syncNow) {
    digitalWrite(SYNC_PIN, HIGH);
    delayMicroseconds(SYNC_MICROSECONDS);
    digitalWrite(SYNC_PIN, LOW);
    previousSyncMicros = micros();
  }
  data->adc[ADC_DIM - 1] = syncNow;
}
//-----------------------------------------------------------------------------
/*
 * Checks that device responds correctly to WHO_AM_I. If it fails,
 * an error message is printed to Serial and execution stops.
 */
bool checkWho(uint8_t deviceAddr, uint8_t regAddr, uint8_t correct) {
  byte whoIAm = readByte(deviceAddr, regAddr);
  if (whoIAm != correct) {
    Serial.print("Device at address 0x");
    Serial.print(deviceAddr, HEX);
    Serial.println(" did not respond correctly to WHO_AM_I.");
    Serial.print("Expected 0x");
    Serial.print(correct, HEX);
    Serial.print(" but recieved 0x");
    Serial.println(whoIAm, HEX);
  }
  return whoIAm == correct;
}
//-----------------------------------------------------------------------------
/*
 * Initialize AK8963 magnetometer that is inside the MPU9250 and
 * set it up to be an I2C slave
 */
bool initAK8963(uint8_t MPU9250_ADDRESS) {
  if (!USE_MAGNETOMETER)
    return true;

  Serial.print("Magnetometer for 0x");
  Serial.println(MPU9250_ADDRESS, HEX);
  
  // Tell MPU9250 to let us talk to AK8963 directly
  writeByte(MPU9250_ADDRESS, INT_PIN_CFG, 0x02); // enable pass thru when i2c master disabled
  writeByte(MPU9250_ADDRESS, USER_CTRL, B00000000); // turn off i2c master on mpu9250
  delay(500);


  if (!checkWho(AK8963_ADDRESS, WHO_AM_I_AK8963, 0x48))
    return false;

  writeByte(AK8963_ADDRESS, AK8963_CNTL, B00000010);

  // Enable I2C master functionality on MPU9250
  writeByte(MPU9250_ADDRESS, I2C_MST_CTRL, B00001000);
  //  Bit   Name           Description
  //   7    MULT_MST_EN    I2C multimaster (1 to enable)
  //   6    WAIT_FOR_ES    DRDY interrupt waits for external data (1 to enable)
  //   5    SLV_3_FIFO_EN  Write SLV_3 data to FIFO (1 to enable)
  //   4    I2C_MST_P_NSR  Behavior between reads (0 to stop, 1 to reset)
  //  3:0   I2C_MST_CLK    I2C master clock speed (B1000 for 258kHz, slowest)

  // Configure to read from WHO_AM_I register of AK8963
  writeByte(MPU9250_ADDRESS, I2C_SLV0_ADDR, 128 + AK8963_ADDRESS);
  // First bit is "1" for read

  writeByte(MPU9250_ADDRESS, I2C_SLV0_REG, AK8963_XOUT_L);
  // Value read should be 0x48

  writeByte(MPU9250_ADDRESS, I2C_SLV0_CTRL, B10000111);
  //  Bit   Name              Description
  //   7    I2C_SLV0_EN       Use this slave device (1 to enable)
  //   6    I2C_SLV0_BYTE_SW  Swap bytes in a word? (1 to enable)
  //   5    I2C_SLV0_REG_DIS  "When set, the transaction does not write a register value, it will only read data, or write data"
  //   4    I2C_SLV0_GRP      Grouping of bytes into words (0 for [0 1] [2 3]...; 1 for 0 [1 2] [3 4]...)
  //  3:0   I2C_MST_CLK       Number of bytes to read (seven bytes = B0111)

  // Enable I2C master
  writeByte(MPU9250_ADDRESS, USER_CTRL, B00100000);
  return true;
}
//-----------------------------------------------------------------------------
bool initMPU9250(uint8_t MPU9250_ADDRESS) {
  if (!checkWho(MPU9250_ADDRESS, WHO_AM_I_MPU9250, 0x71))
    return false;

  // wake up device
  writeByte(MPU9250_ADDRESS, PWR_MGMT_1, 0x00); // Clear sleep mode bit (6), enable all sensors
  delay(100); // Wait for all registers to reset

  // get stable time source
  writeByte(MPU9250_ADDRESS, PWR_MGMT_1, 0x01);  // Auto select clock source to be PLL gyroscope reference if ready else
  delay(200);

 // Configure Gyro and Thermometer
 // Disable FSYNC and set thermometer and gyro bandwidth to 41 and 42 Hz, respectively;
 // minimum delay time for this setting is 5.9 ms, which means sensor fusion update rates cannot
 // be higher than 1 / 0.0059 = 170 Hz
 // DLPF_CFG = bits 2:0 = 011; this limits the sample rate to 1000 Hz for both
 // With the MPU9250, it is possible to get gyro sample rates of 32 kHz (!), 8 kHz, or 1 kHz
  writeByte(MPU9250_ADDRESS, CONFIG, 0x03);

 // Set sample rate = gyroscope output rate/(1 + SMPLRT_DIV)
  writeByte(MPU9250_ADDRESS, SMPLRT_DIV, 0x04);  // Use a 200 Hz rate; a rate consistent with the filter update rate
                                    // determined inset in CONFIG above

 // Set gyroscope full scale range
 // Range selects FS_SEL and AFS_SEL are 0 - 3, so 2-bit values are left-shifted into positions 4:3
  uint8_t c = readByte(MPU9250_ADDRESS, GYRO_CONFIG); // get current GYRO_CONFIG register value
 // c = c & ~0xE0; // Clear self-test bits [7:5]
  c = c & ~0x02; // Clear Fchoice bits [1:0]
  c = c & ~0x18; // Clear AFS bits [4:3]
  c = c | GYRO_SCALE << 3; // Set full scale range for the gyro (3 for 2000 dps)
 // c =| 0x00; // Set Fchoice for the gyro to 11 by writing its inverse to bits 1:0 of GYRO_CONFIG
  writeByte(MPU9250_ADDRESS, GYRO_CONFIG, c ); // Write new GYRO_CONFIG value to register

 // Set accelerometer full-scale range configuration
  c = readByte(MPU9250_ADDRESS, ACCEL_CONFIG); // get current ACCEL_CONFIG register value
 // c = c & ~0xE0; // Clear self-test bits [7:5]
  c = c & ~0x18;  // Clear AFS bits [4:3]
  c = c | ACCEL_SCALE << 3; // Set full scale range for the accelerometer (1 for 4g)
  writeByte(MPU9250_ADDRESS, ACCEL_CONFIG, c); // Write new ACCEL_CONFIG register value

 // Set accelerometer sample rate configuration
 // It is possible to get a 4 kHz sample rate from the accelerometer by choosing 1 for
 // accel_fchoice_b bit [3]; in this case the bandwidth is 1.13 kHz
  c = readByte(MPU9250_ADDRESS, ACCEL_CONFIG2); // get current ACCEL_CONFIG2 register value
  c = c & ~0x0F; // Clear accel_fchoice_b (bit 3) and A_DLPFG (bits [2:0])
  c = c | 0x03;  // Set accelerometer rate to 1 kHz and bandwidth to 41 Hz
  writeByte(MPU9250_ADDRESS, ACCEL_CONFIG2, c); // Write new ACCEL_CONFIG2 register value
 // The accelerometer, gyro, and thermometer are set to 1 kHz sample rates,
 // but all these rates are further reduced by a factor of 5 to 200 Hz because of the SMPLRT_DIV setting

  // Configure Interrupts and Bypass Enable
  // Set interrupt pin active high, push-pull, hold interrupt pin level HIGH until interrupt cleared,
  // clear on read of INT_STATUS, and enable I2C_BYPASS_EN so additional chips
  // can join the I2C bus and all can be controlled by the Arduino as master
   writeByte(MPU9250_ADDRESS, INT_PIN_CFG, 0x22);
   writeByte(MPU9250_ADDRESS, INT_ENABLE, 0x01);  // Enable data ready (bit 0) interrupt
   delay(100);
   return true;
}

uint8_t writeByte(uint8_t address, uint8_t subAddress, uint8_t data) {
  /*
   * Write one byte of data to I2C slave register
   * Returns 0 on success
   *         1 if data is too long
   *         2 if received NACK on address
   *         3 if received NACK on data
   *         4 if other error
   */
  Wire1.beginTransmission(address);
  Wire1.write(subAddress);
  Wire1.write(data);
  Wire1.endTransmission();
  uint8_t i2cErrorCode = Wire1.getError();
  if(i2cErrorCode != 0) {  // if write was not successful
    Wire1.resetBus();
  }
  return i2cErrorCode;
}

uint8_t readByte(uint8_t address, uint8_t subAddress) {
  /*
   * Read one byte of data from a register in an I2C slave device.
   * Returns the data. If no data is received, returns 0.
   */
  Wire1.beginTransmission(address);
  Wire1.write(subAddress);
  Wire1.endTransmission();
  Wire1.requestFrom(address, (uint8_t) 1);
  if (Wire1.available()) {
    return Wire1.readByte();
  } else {
    return 0;
  }
}
//-----------------------------------------------------------------------------
void blinkForever() {
  while (1) {
    digitalWrite(LED_PIN, HIGH);
    delay(1000);
    digitalWrite(LED_PIN, LOW);
    delay(1000);
  }
}

uint8_t readBytes(uint8_t address, uint8_t subAddress, uint8_t count, uint8_t * dest) {
  /*
   * Read multiple bytes from registers on an I2C slave device. Store the data in *dest.
   * Returns 0
   */
  Wire1.beginTransmission(address);
  Wire1.write(subAddress);
  Wire1.endTransmission();
  Wire1.requestFrom(address, count);
  for (int i=0; i<count; i++) {
    dest[i] = Wire1.read();
  }
  return 0;
}

