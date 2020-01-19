/*
  Most early 256Kilobit DIP-16 ram chips had max 4ms refresh cycles to account for worst possible capacitor cell.
  This program will set an interrupt timer for x miliseconds refresh.

  Little-Endian right most bit of byte will be written to first memory location

DIP16 pinout, Mega2560 Pin, Mega2560 Port Register
A0 22 PA0
A1 23 PA1
A2 24 PA2
A3 25 PA3
A4 26 PA4
A5 27 PA5
A6 28 PA6
A7 29 PA7
A8 37 PC0

WE  49 PL0
CAS 48 PL1
RAS 47 PL2
DI  46 PL3
DO  45 PL4
*/

// PORTL bit positions
#define WE  0
#define CAS 1
#define RAS 2
#define DI  3
#define DO  4

#define REFRESH 2 // Refresh interval in miliseconds(ms)
#define PASSES 3  // Number of times to run through all rows
#define DEBUG 0  // 0 - Minimal, 1 - Show row data

#define ADDRBITS 9  // A0-A8 = 9 bits
#define RANDOMSEED 5487  // Change me between runs

// Fast page mode early write cycle (8bit data, 9bit Row Addr, 9bit Col Addr)
void writeByte(byte data, uint16_t row, uint16_t col) {
  noInterrupts(); // Don't interrupt for refresh while writting

  // Preset CAS/RAS
  PORTL |= (1 << RAS);  // RAS inactive
  PORTL |= (1 << CAS);  // CAS inactive

  // Set row Address
  PORTA = (row & 0x00FF); // Grab the 8 LSB
  PORTC = (row & 0x0100) >> 8; // 9th addr bit

  // Set write conditions
  PORTL &= ~(1 << RAS); // RAS active
  PORTL &= ~(1 << WE);  // WE active

  for(uint8_t colAddr = 0; colAddr < 8; colAddr++) {
    digitalWrite(46, bitRead(data, colAddr)); // Set DI value
    PORTA = (col+colAddr);       // Set COL address
    PORTC = (col & 0x0100) >> 8; // 9th addr bit
    
    // Latch data
    PORTL &= ~(1 << CAS);   // CAS active
    PORTL |= (1 << CAS);    // CAS inactive
  }

  PORTL |= (1 << WE);   // WE inactive
  PORTL |= (1 << RAS);  // RAS inactive
  interrupts();
}

// Fast page mode read cycle (9bit Row addr, 9bit Col addr)
byte readByte(uint16_t row, uint16_t col) {
  byte dO = 0;

  noInterrupts(); // Dont interrupt while reading

  // Preset CAS/RAS
  PORTL |= (1 << RAS);  // RAS inactive
  PORTL |= (1 << CAS);  // CAS inactive

  // Set row Address
  PORTA = (row & 0x00FF); // Grab the 8 LSB
  PORTC = (row & 0x0100) >> 8; // 9th addr bit

  // Set read conditions
  PORTL &= ~(1 << RAS); // RAS active
  PORTL |= (1 << WE);   // WE inactive

  for(uint8_t colAddr = 0; colAddr < 8; colAddr++) {
    PORTA = (col+colAddr);  // Set col Address
    PORTC = (col & 0x0100) >> 8; // 9th addr bit

    // Trigger CAS
    PORTL &= ~(1 << CAS);   // CAS active
    PORTL |= (1 << CAS);    // CAS inactive

    bitWrite(dO, colAddr, digitalRead(45));
  }

  PORTL |= (1 << RAS);  // RAS inactive
  interrupts();
  
  return dO;
}

byte ram[65];
byte fake[65];
uint16_t maxrc = 0;
uint8_t pass = 1;

void setup() {
  // Address lines 0-8 output
  DDRA |= 0xFF;
  DDRC |= 0xFF;

  // Data and CAS/RAS lines
  DDRL |= B00001111;

  // CAS, RAS and WE are active low
  PORTL |= B00000111;

  noInterrupts();
  TCCR1A = 0;
  TCCR1B = 0;

  // 2ms Timer trigger. Most ram is 4ms minimum refresh
  OCR1A = 32000;
  TCCR1B |= (1 << WGM12);
  // No Prescaller
  TCCR1B |= (1 << CS10);
  TIMSK1 |= (1 << OCIE1A);
 
  interrupts();

  // Calculate number of rows based on address bits
  maxrc = (1 << ADDRBITS);

  // Analog pins settles quickly and I would keep getting same seed no matter what after a few seconds of power on.
  randomSeed(RANDOMSEED);

  Serial.begin(115200);
  Serial.setTimeout(2000);
  Serial.flush();
  delay(4000); // Give 4s time to open serial monitor
  while (!Serial) continue;
}

void loop() {
  uint16_t col = 0;
  uint16_t row = 0;
  uint8_t colByte = 0;
  char passStr[150];
  char rowStr[200];
  char failStr[150];
  bool failed = false;

  for (pass; pass < (PASSES+1); pass++) {
    sprintf(passStr, "Starting Pass (%d/%d) AddrBits(%d) ROWS(%d) COLS(%d)----", pass, PASSES, ADDRBITS, maxrc, (maxrc / 8));
    Serial.print(passStr);
    if (DEBUG) Serial.println();
    for (row = 0; row < maxrc; row++) {
      if (DEBUG) sprintf(rowStr, "Wrt %03d: 0x", row);
      for (colByte = 0; colByte < (maxrc / 8); colByte++) {
        col = colByte*8;
        fake[colByte] = (uint8_t)random(256);
        writeByte(fake[colByte], row, col);
        if (DEBUG) sprintf(rowStr + strlen(rowStr), "%02X", fake[colByte]);       
      }

      if (DEBUG) Serial.println(rowStr);

      if (DEBUG) sprintf(rowStr, "Rrd %03d: 0x", row);
      for (colByte = 0; colByte < (maxrc / 8); colByte++) {
        col = colByte*8;
        ram[colByte] = readByte(row, col);
        
        if (fake[colByte] != ram[colByte]) {
          sprintf(failStr, "  *** Address(R:C)> %03X:%03X GOT[%02X] Expected[%02X]", row, col, ram[colByte], fake[colByte]);
          Serial.println(failStr);
          failed = true;
          break;
        } else {
          if (DEBUG) sprintf(rowStr+strlen(rowStr), "%02X", ram[colByte]);
        }
      }

      if (DEBUG) Serial.println(rowStr);
      if (failed) break;
    }

    if (failed) {
      sprintf(passStr, "FAILED at Row[%03X]", row);
      Serial.println(passStr);
    } else {
      Serial.println(" SUCCESS");
    }
    failed = false;
  }
}


// RAS only refresh cycle
ISR(TIMER1_COMPA_vect) {
  noInterrupts();
  // Save Register state
  uint8_t lsb = PORTA;
  uint8_t msb = PORTC;
  uint8_t ptl = PORTL;
  uint16_t row = 0;

  for (row = 0; row < 512; row++) {
    // Set row address
    PORTA = (row & 0x00FF); // Grab the 8 LSB
    PORTC = (row & 0x0100) >> 8; // 9th addr bit

    // Strobe row  
    PORTL &= ~(1 << RAS); // RAS active
    PORTL |= (1 << RAS);  // RAS inactive
  }

  // Restore register state
  PORTA = lsb;
  PORTC = msb;
  PORTL = ptl;
  interrupts();
}
