/*
 * This is a high speed data logger. It acquires data and stores it on an SD 
 * card. In my tests using a Teensy 3.6 and a SanDisk Ultra 16GB SDHC micro SD 
 * card, I was able to sample an analog pin at 25 kHz.
 * 
 * It relies on the beta version of the SdFat library written by Bill Greiman,
 * which is available at https://github.com/greiman/SdFat-beta. I have tested
 * this sketch with revision ffbccb0 of SdFat-beta. This code was inpired by 
 * SdFat's LowLatencyLogger and TeensySdioDemo examples. It uses the same
 * binary file format as LowLatencyLogger, but with a bigger block size.
 * 
 * Here is how the code works. We have four buffers that are used to store the
 * data. The main loop is very simple: it checks to see if there are any full 
 * buffers, and if there are, it writes them to the SD card. When is the data 
 * acquired? Data is acquired in the function yield(), which is called 
 * whenever the Teensy is not busy with something else. yield() is called by 
 * the main loop whenever there is nothing to write to the SD card. yield() is 
 * also called by the SdFat library whenever it is waiting for the SD card.
 */

#include "SdFat.h"

// Pin to record
const int SENSOR_PIN = A0;

// 16 KiB buffer.
const size_t BUF_DIM = 16384;

// Sampling rate
const uint32_t sampleIntervalMicros = 40;
// 40 us interval = 25 kHz

// Use total of four buffers.
const uint8_t BUFFER_BLOCK_COUNT = 4;

// Number of data points per record
const uint8_t ADC_DIM = 1;

// Format for one data record
struct data_t {
  uint32_t time;
  uint32_t adc[ADC_DIM];
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
const uint16_t DATA_DIM = (BUF_DIM - 4)/sizeof(data_t);

//Compute fill so block size is BUF_DIM bytes.  FILL_DIM may be zero.
const uint16_t FILL_DIM = BUF_DIM - 4 - DATA_DIM*sizeof(data_t);

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
bool fileIsClosing = false;
bool collectingData = false;
bool isSampling = false;
bool justSampled = false;

//-----------------------------------------------------------------------------
void setup() {
  Serial.begin(9600);
  while (!Serial) {
  }

  // Put all the buffers on the empty stack.
  for (int i = 0; i < BUFFER_BLOCK_COUNT; i++) {
    emptyStack[i] = &block[i - 1];
  }
  emptyTop = BUFFER_BLOCK_COUNT;

  sd.begin();
  if (!file.open("TeensyDemo.bin", O_RDWR | O_CREAT)) {
    error("open failed");
  }
  
  Serial.print("Block size: ");
  Serial.println(BUF_DIM);
  Serial.print("Record size: ");
  Serial.println(sizeof(data_t));
  Serial.print("Records per block: ");
  Serial.println(DATA_DIM);
  Serial.print("Record bytes per block: ");
  Serial.println(DATA_DIM*sizeof(data_t));
  Serial.print("Fill bytes per block: ");
  Serial.println(FILL_DIM);
  Serial.println("Recording. Enter any key to stop.");
  delay(100);
  collectingData=true;
  nextSampleMicros = micros() + sampleIntervalMicros;
}
//-----------------------------------------------------------------------------
void loop() {
  // Write the block at the tail of the full queue to the SD card
  if (fullHead == fullTail) { // full queue is empty
    if (fileIsClosing){
      file.close();
      Serial.println("File complete.");
      while(1);
    } else {
      yield(); // acquire data etc.
    }
  } else { // full queue not empty
    // write buffer at the tail of the full queue and return it to the top of
    // the empty stack.
    block_t* pBlock = fullQueue[fullTail];
    fullTail = fullTail < QUEUE_LAST ? fullTail + 1 : 0;
    if ((int)BUF_DIM != file.write(pBlock, BUF_DIM)) {
      error("write failed");
    }
    emptyStack[emptyTop++] = pBlock;
//    Serial.print(micros());
//    Serial.println(" write complete");
  }

  // fileIsClosing = (micros() > 500000000);
  fileIsClosing = Serial.available();
}
//-----------------------------------------------------------------------------
void yield(){
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
      error("rate too fast");
    }
    acquireData(&curBlock->data[curBlock->count++]);
    nextSampleMicros += sampleIntervalMicros;
    justSampled = true;
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
   * Takes a block form the top of the empty stack and returns it
   */
  block_t* blk = 0;
  if (emptyTop > 0) { // if there is a buffer in the empty stack
    blk = emptyStack[--emptyTop];
    blk->count = 0;
//    Serial.print(micros());
//    Serial.print(" new block, remaining=");
//    Serial.println(emptyTop);
  } else { // no buffers in empty stack
    error("All buffers in use");
  }
  return blk;
}
//-----------------------------------------------------------------------------
void putCurrentBlock() {
  /*
   * Put the current block at the head of the queue to be written to card
   */
  fullQueue[fullHead] = curBlock;
  fullHead = fullHead < QUEUE_LAST ? fullHead + 1 : 0;
  curBlock = 0;
}
//-----------------------------------------------------------------------------
void error(String msg) {
  Serial.print("ERROR: ");
  Serial.println(msg);
  while(1);
}
//-----------------------------------------------------------------------------
void acquireData(data_t* data){
  data->time = micros();
  data->adc[0] = analogRead(SENSOR_PIN);
}

