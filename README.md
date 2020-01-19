# ArduinoDIP16RamTest
Arduino Ram tester for 256 kilobit or less dram chip DIP16

I wrote this to locate any bad ram chip in my Tandy 1000A ram expansion board.  The Tandy had MT1257-20 chips, 16 of them. They are 256kbit x 1 DIP16 dram.  I also tested Goldstar GM71C256-10 a modern CMOS version of DIP16 chips.


## Description
* Designed to test 32,64,128,256kb x 1 bit DIP16 Dram chips.
* Incorporates a RAS only refresh cycle on a timer based interrupt. Default 2ms refresh
* Written around an Arduino Mega2560
* Writes byte at a time to every row
* Writes a full byte using the "fast page mode early write cycle" most DRAMs should support this

## Instructions
* Configure number of passes, address lines etc using #define statements
* If using different PINs, you will need to identify the PORTA,B,C registers etc that corresponds to the PINSs If not using an Arduino Mega, the address lines will have to be spread across more PORT registers. Requiring changes to the code.
* There is a 4s startup delay, helps keep the Serial Monitor cleaner
