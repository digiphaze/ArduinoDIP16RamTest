#define WORDS 262144        // <--- Change me
#define BITS  4             // <--- Change to number of data bits/wordsize(Max 8)
#define SINGLE_IO 1         // 1 If data in and out are same pins
#define CHIP_OE 1           // 1 if DRAM has an output enable pin
#define REFRESHCYCLE 6      // In Miliseconds(ms) takes aprox 1ms to refresh 512 rows,
                            //  if max 8ms refresh then set the cycle to kick off 1ms less than max to ensure integrity
#define ARDUINO MEGA2560    // Set to board type, currently only MEGA2560

// TEST Parameters
#define STARTUP_DELAY 2000  // (ms) 2s default
#define RANDOMSEED 5487     // Change for different starting values
#define BAUDRATE 115200
#define PASSES 2            // How many times to run test
#define WRITE_TO_READ 2000  // How long to wait after writing a row (ms). Tests cell retention
#define DEBUG 1             // 0 for less console outut

#define MAX_ROWS (uint16_t)(sqrt(WORDS))
#define MAX_COLS (uint16_t)(sqrt(WORDS))
#define BYTES_PER_ROW (uint16_t)((MAX_COLS * BITS) / 8)
#define COLS_PER_BYTE (uint8_t)(8 / BITS)
#define BITSZMASK (uint8_t)((1 << BITS) - 1)

/* Wire RAM to MEGA2560 as follows
 ** LSB of Address/Data pins must follow order of Mega pins listed below
 ** ex.  A0 = D22, A1 = D23,  A8 = D37, A9 = D36 ... etc
 --------------------
  RAM         MEGA2560
  Addr(0-7)   A0-7
  Addr(8-16)  A8-16
  IO(0-8)     D22-29  >> If separate Data in/out pins, wire Data in here
  DO(0-8)     D37-30  >> Only if Data out pins are separate from in
  RAS         D42
  CAS         D43
  WE          D44
  OE          D45
*/
#if ARDUINO == MEGA2560
  #define ARDUINO_SPEED 16        // Speed in MHZ
  #define ADDRESS_LSB PORTF
  #define ADDRESS_LSB_DDR DDRF
  #define ADDRESS_MSB PORTK
  #define ADDRESS_MSB_DDR DDRK
  #define IO_DATA PORTA
  #define IO_DIR DDRA
  #define IO_VAL PINC
  #define DO_DIR DDRC
  #define DO_VAL PINC
  #define CTRL_PORT PORTL
  #define CTRL_DDR DDRL
  #define RAS_MASK 0x01
  #define CAS_MASK 0x02
  #define WE_MASK 0x03
  #define OE_MASK 0x04
#endif

#define RAM_RAS_INACTIVE CTRL_PORT |= RAS_MASK
#define RAM_RAS_ACTIVE CTRL_PORT &= ~(RAS_MASK)
#define RAM_CAS_INACTIVE CTRL_PORT |= CAS_MASK
#define RAM_CAS_ACTIVE CTRL_PORT &= ~(CAS_MASK)
#define RAM_WE_DISABLE CTRL_PORT |= WE_MASK
#define RAM_WE_ENABLE CTRL_PORT &= ~(WE_MASK)
#define RAM_OE_DISABLE CTRL_PORT |= OE_MASK
#define RAM_OE_ENABLE CTRL_PORT &= ~(OE_MASK)
#define ARDUINO_IO_READ IO_DIR &= 0x00
#define ARDUINO_IO_WRITE IO_DIR |= 0xFF

#define RAM_CYCLE_SETUP CTRL_PORT &= (0xF0 | (RAS_MASK | CAS_MASK | WE_MASK | OE_MASK))

#if SINGLE_IO == 1
  #define ARDUINO_READ IO_VAL
#else
  #define ARDUINO_READ DO_VAL
#endif

#define LATCH(addr) ADDRESS_LSB = (addr & 0x00FF); ADDRESS_MSB = (addr & 0xFF00)

// Read cycle
static inline void read_c(uint16_t row_addr, uint16_t col_addr, uint8_t *data) {
  noInterrupts();
  RAM_CYCLE_SETUP;
  ARDUINO_IO_READ;
  
  // Latch row (16mhz Arduino executes about 1 inst per tick (62.5ns)
  //  for an aprox 313 ns tASR (Row address setup time)
  LATCH(row_addr);
  RAM_RAS_ACTIVE;

  // Latch col (Most dram is 20ns tCAC (Access time from CAS))
  //  At 62.5ns per tick for 16mhz, we should be fine foregoing a delay
  LATCH(col_addr);
  RAM_CAS_ACTIVE;

  // Most dram is 20ns tOAC (Access time from OE)
  //  At 62.5ns per tick for 16mhz, we should be fine foregoing a delay
  #if CHIP_OE
    RAM_OE_ENABLE;
  #endif
  *data = ARDUINO_READ;
  #if CHIP_OE
    RAM_OE_DISABLE;
  #endif
  
  RAM_CAS_INACTIVE;
  RAM_RAS_INACTIVE;
  RAM_RAS_ACTIVE;
  interrupts();
}

// Write cycle (We aren't fast enough to qualify for Early write cycle, this more emulates Delayed Write Cycle)
static inline void write_c(uint16_t row_addr, uint16_t col_addr, uint8_t *data) {
  noInterrupts();
  RAM_CYCLE_SETUP;
  ARDUINO_IO_WRITE;
  
  // Latch row
  LATCH(row_addr);
  RAM_RAS_ACTIVE;

  // Set outbound data
  IO_DATA = data;
  RAM_WE_ENABLE;

  // Latch col
  LATCH(col_addr);
  RAM_CAS_ACTIVE;

  // Latch DATA
  RAM_CAS_INACTIVE;
  RAM_RAS_INACTIVE;
  RAM_RAS_ACTIVE;
  interrupts();
}

// Arduino isn't fast enough to justify implementing this cycle
// Placeholder for now
// static inline void read_modify_c() {}
// static inline void fast_p_mode_read_modify_c() {}

// Fast page mode read cycle
// Arduino Mega2560 has 8K variable memory.  A 262,144x4 bit DRAM will be 2KB of RAM usage per row
// There may not be enough ram on smaller Arduinos for a full row in one read cycle
// DON'T EXCEED BYTES_PER_ROW(MAX_COLS * BITS) / 8)
static inline void fast_p_mode_read_c(uint16_t row_addr, uint16_t bytes, uint8_t *data) {
  noInterrupts();
  RAM_CYCLE_SETUP;
  ARDUINO_IO_READ;

  // Latch row
  LATCH(row_addr);
  RAM_RAS_ACTIVE;

  #if CHIP_OE
    RAM_OE_ENABLE;
  #endif

  uint16_t col_addr = 0;
  uint8_t c;
  
  for (uint16_t b = 0; b < bytes; b++) {
    data[b] = 0; // Zero out destination memory for |= operation
    for (c = 0; c < COLS_PER_BYTE; c++) {
      LATCH(col_addr);
      RAM_CAS_ACTIVE;
      // Unsure how many ticks will be between RAM_CAS_ENABLE and the PIN registers latching the bus value
      // Adding a single NOP to give a clock tick to latch PIN register
      asm("nop");
      data[b] = (data[b] & 0xFF) | (ARDUINO_READ << (c*BITS));
      RAM_CAS_INACTIVE;
      col_addr++;
    }
  }

  #if CHIP_OE
    RAM_OE_DISABLE;
  #endif

  RAM_RAS_INACTIVE;
  RAM_RAS_ACTIVE;
}

static inline void fast_p_mode_write_c(uint16_t row_addr, uint16_t bytes, uint8_t *data) {
  noInterrupts();
  RAM_CYCLE_SETUP;
  ARDUINO_IO_WRITE;

  // Latch row
  LATCH(row_addr);
  RAM_RAS_ACTIVE;

  uint16_t col_addr = 0;
  uint8_t c;
  
  for (uint16_t b = 0; b < bytes; b++) {
    IO_DATA = 0x00;
    for (c = 0; c < COLS_PER_BYTE; c++) {
      LATCH(col_addr);
      RAM_CAS_ACTIVE;
      // Unsure how many ticks will be between RAM_CAS_ENABLE and the PIN registers latching the bus value
      // Adding a single NOP to give a clock tick to latch PIN register
      asm("nop");
      IO_DATA = (data[b] & (BITSZMASK << (BITS * c))) >> (BITS * c);
      RAM_WE_ENABLE;
      RAM_CAS_INACTIVE;
      RAM_WE_DISABLE;
      col_addr++;
    }
  }

  RAM_RAS_INACTIVE;
  RAM_RAS_ACTIVE;
}

static inline void cas_before_ras_refresh_c() {}
static inline void hidden_refresh_read_c() {}
static inline void hidden_refresh_write_c() {}

void setup() {
  // All address lines set as OUTPUT pins
  ADDRESS_LSB_DDR = 0xFF;
  ADDRESS_MSB_DDR = 0xFF;

  DO_DIR = 0x00;  // For split Din and Dout on ram, force this to always be input
  
  // Initialize at 0x00;
  ADDRESS_LSB = 0x00;
  ADDRESS_MSB = 0x00;

  // Interrupt Setup (16,000 ticks per mhz)
  // 1ms per 16,000 ticks.
  noInterrupts();
  TCCR1A = 0;
  TCCR1B = 0;
  OCR1A = (16,000 * REFRESHCYCLE);
  TCCR1B |= (1 << WGM12);
  TCCR1B |= (1 << CS10);    // No Prescaller
  TIMSK1 |= (1 << OCIE1A);  // Interrupt on compare match
  interrupts();

  randomSeed(RANDOMSEED);

  Serial.begin(BAUDRATE);
  Serial.setTimeout(1000);
  Serial.flush();
  delay(STARTUP_DELAY);
  while (!Serial) continue;
}

void loop() {
  // put your main code here, to run repeatedly:
  uint16_t row = 0;
  uint8_t fromRamStr[BYTES_PER_ROW];
  uint8_t randomStr[BYTES_PER_ROW];
  char diagStr[150];

  for (uint8_t pass = 1; pass < PASSES; pass++) {
    sprintf(diagStr, "Starting Pass (%d/%d) DataBits(%d), Rows(%d), Cols(%d)----",pass, PASSES, BITS, MAX_ROWS, MAX_COLS);
    Serial.println(diagStr);
    for (uint16_t row = 0; row < MAX_ROWS; row++) {
      // Generate row random data
      for (uint16_t byteW = 0; byteW < BYTES_PER_ROW;  byteW++) {
        randomStr[byteW] = random(256);
      }
      #if DEBUG == 1
        sprintf(diagStr," W -> Row[%d] -- 0x%X", (char *)randomStr);
        Serial.println(diagStr);
      #endif
      fast_p_mode_write_c(row, BYTES_PER_ROW, randomStr);
      #if WRITE_TO_READ > 0
        delay(WRITE_TO_READ);
      #endif
      fast_p_mode_read_c(row, BYTES_PER_ROW, fromRamStr);
      #if DEBUG == 1
        sprintf(diagStr, " R <- Row[%d] -- 0x%X", (char *)fromRamStr);
        Serial.println(diagStr);
      #endif
    }
  }
}

ISR(TIMER1_COMPA_vect) {
  noInterrupts();
  // Save Register state
  uint8_t lsb = ADDRESS_LSB;
  uint8_t msb = ADDRESS_MSB;
  uint8_t ctl = CTRL_PORT;
  uint16_t row = 0;

  for (row; row < MAX_COLS; row++) {
    // Set row address
    ADDRESS_LSB = (row & 0x00FF);
    ADDRESS_MSB = (row & 0xFF00) >> 8;

    // Strobe row
    RAM_RAS_ACTIVE;
    RAM_RAS_INACTIVE;
  }

  ADDRESS_LSB = lsb;
  ADDRESS_MSB = msb;
  CTRL_PORT   = ctl;
  interrupts();
}
