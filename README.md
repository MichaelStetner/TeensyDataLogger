# TeensyDataLogger
Record data to SD card at a high sampling rate

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
