/*
 * 
 *     Arduino IDE setup:
 *
 *  add in boards.txt file something describing the ATMEGA88P  (P is important)
 *  Any megaxx8 should be ok.
 *


## Arduino Pro or Pro Mini (5V, 8 MHz) w/ ATmega88
## -------------------------------------------------
pro.menu.cpu.8MHzatmega88=ATmega88P (5V, 8 MHz no crystal)

pro.menu.cpu.8MHzatmega88.upload.maximum_size=8000
pro.menu.cpu.8MHzatmega88.upload.maximum_data_size=1024
pro.menu.cpu.8MHzatmega88.upload.speed=19200

pro.menu.cpu.8MHzatmega88.build.mcu=atmega88p
pro.menu.cpu.8MHzatmega88.build.f_cpu=8000000L
pro.menu.cpu.8MHzatmega88.bootloader.low_fuses=0xc2
pro.menu.cpu.8MHzatmega88.bootloader.high_fuses=0xdf
pro.menu.cpu.8MHzatmega88.bootloader.extended_fuses=0x01


 * IDE Settings
 *
 * Board     : Arduino Promini
 * Processor : ATmega88P
 * Progammer : USBASP
 *       
 *        
 * Hardware description:
 * 
 * The 16 bits shift registers (two classic 74595) control the hardware
 * --1 of 8 bits to be fused via mini relay
 * --5 bits for prom address
 * --LED1 (red)
 * --LED2 (red)
 * --LED3 (green)
 * 
 * Ports usage on Mega88P
 * 
 * PromR => prom read socket
 * PromW => prom write socket
 * 
 * PORTB 8 bits data bus from PromR and PromW
 * 
 * PC0 CSPROMW
 * PC1 HC595 latch rclk
 * PC2 HC595 serial clock
 * PC3 HC595 serial data
 * PC4 CSPROMR 
 * 
 * PC5 analogic buttons chain input
 * PC6 reset
 * PC7 nc
 * 
 * PD0 CH340 RX to USB
 * PD1 CH340 TX to USB
 * PD2 buckOn enable buck MT3608
 * PD3 vppBoost enable VPP or 5v
 * PD4 vppPulse pulseEnable
 * 
 * PD5 Buzzer
 * PD6_DG_CLK
 * PD7 DG_DIO (TM1637 4 digit display)
 *
 *
 * The burning generator hardware is based on Elektor september 1980 (french edition) prom programmer
 * 
 */

#define VERSION "v1.0"


#include <avr/wdt.h> // watchdog

/* Some code remaining from another app, not used yet. 
 *
 * Todo (on 328) : store / select prom files in eeprom.
 * It is human interface that is requiring lot of code.
 * I don't think the Mega88 have enougth code space.
 * Todo : use serial port to tranfert prom files
 *
 */
#include <EEPROM.h>  // for nvram


/*
 *  The 4 digits (aliexpress TM1637) is used to display
 *  the prom address (0..31) and its content.
 *  The buttons allow editing values. When in edit mode
 *  the value is blinking. To make this visually good
 *  canBlink, canBlink2 force visibility during this
 *  blinkState mode.
 *  As I chose interrupt every millisecond to update display
 *  the 'effective' call to the chip TM1637 must be made only 
 *  when a change has occured.This makes the display update 
 *  a little tricky, and more tricky when blinking 2 digits !
 */
#include "TM1637Display.h"


#define tm1637_c      PD6
#define tm1637_d      PD7
#define buzzer        PD5
TM1637Display         Info(tm1637_c,tm1637_d);
boolean               blinkState;
boolean               canBlink, canBlink2;
byte                  digits [4];
/*
 *  The crc is displayed just after reading until a button is pressed
 */
byte                  displayCrc = 0;  //mini state machine, displays high or low part or nothing
unsigned long         crc;

#define HC595         PORTC
#define HC595_SER     PC3
#define HC595_RCLK    PC1
#define HC595_SRCLK   PC2

#define CLR(x,y)  (x&=(~(1<<y)))
#define SET(x,y)  (x|=(1<<y))
#define _BV(bitN) (1 << (bitN))

//#define SetHC595_RCLK(bitval)  ((bitval>0) ? (SET(HC595,2)) : (CLR(HC595,2)))
//#define SetHC595_SRCLK(bitval) ((bitval>0) ? (SET(HC595,3)) : (CLR(HC595,3)))
//#define SetHC595_SER(bitval)   ((bitval>0) ? (SET(HC595,4)) : (CLR(HC595,4)))





/*
 * Data shifted out
 */
#define        HC595_REG_lamp   0
#define        HC595_REG_bitsel 1
volatile byte  HC595_REGS[2]; 

/* 
 *  
 * ls595 U5 controls Prom Address and LEDs
 * 
 */
#define setPromAddr(v) ( HC595_REGS[HC595_REG_lamp] = ( (HC595_REGS[HC595_REG_lamp] & 0B11100000) | (v & 0B00011111) ) )
#define setLedR(bitval) ((bitval>0) ? (SET(HC595_REGS[HC595_REG_lamp],5)) : (CLR(HC595_REGS[HC595_REG_lamp],5)))
#define setLedW(bitval) ((bitval>0) ? (SET(HC595_REGS[HC595_REG_lamp],6)) : (CLR(HC595_REGS[HC595_REG_lamp],6)))
#define setLed(bitval) ((bitval>0) ? (SET(HC595_REGS[HC595_REG_lamp],7)) : (CLR(HC595_REGS[HC595_REG_lamp],7)))

/*  
 * U4 selects the bit to be programmed. The delay let IRQ propagate to the ls595
 */
#define setPromBitSelector(v) ( HC595_REGS[HC595_REG_bitsel] = _BV(v & 0B00000111) ); \
        delay (3)
#define clearPromBitSelector ( HC595_REGS[HC595_REG_bitsel] = 0 ); \
        delay (3)

// buckOn, vppBoost, vppPulse
// do not use digitalWrite
#define buckOn 4        /*(PD2)*/
#define vppNoBoost 8    /*(PD3)*/
#define vppNoPulse 16   /*(PD4)*/


/*
 * BUTTONS
 * The programm ask the button function the state and is replied some kinf of message.
 * For eliminating rebounds (I didn't check if really necessary), it was quick
 * fixes to what seemed strange behavior of press/depress vs time response.
 * 
 */

#define AnalogButtons (PC5)
enum buttonsMessage   { btNone, btRead, btMoins, btPlus, btVal, btWrite, btReadRel, btMoinsRel, btPlusRel, btValRel, btWriteRel };
volatile unsigned int countDown;
volatile byte         countDownRebound;
unsigned int          repeatSpeed = 300;
boolean               btInRepeat = false;
buttonsMessage        lastBt = btNone;
int                   lastButtonVoltage = -1;

/* 
 *  Prom Chip Selects pins (CS)
 *
 * For some reason, using DigitalWrite with PC1 PC3 is not working
 * Also, they are code bytes consuming !
 * So, directly set bits
 * Disable interrupt because is would kill ls595 IRQ controling !
 * Todo: inline assembler sbi/cbi to remove noInterrupts()
 * 
 */
#define promReadCS0         \
    noInterrupts();         \
    PORTC = 0b11101111 ;    \
    interrupts()

#define promReadCS1         \
    noInterrupts();         \
    PORTC = 255;            \
    interrupts()

#define promWriteCS0        \
    noInterrupts();         \
    PORTC = 0b11111110;     \
    interrupts()

#define promWriteCS1        \
    noInterrupts();         \
    PORTC = 255;            \
    interrupts()


//Working Data

byte    promReadData [32];
byte    promAddress;
byte    promWriteData [32];
boolean promReadEditData = false;
boolean promReadEmpty = true;

/*
 *
 * This is the default PROM file used when Read is pressed without chip inserted. 
 * A double bip is emitted when this file is used.
 * The first goal of this project was Jeutel pinball display drivers on A1 board.
 * 
 */
byte jeutel [32] = { 0x3f, 0x80, 0x5b, 0x4f,  0x66, 0x6d, 0x7d, 0x07,  0x7f, 0x6f, 0x81, 0x79,  0x38, 0x3e, 0x1e, 0x00,
                     0x3f, 0x06, 0x5b, 0x4f,  0x66, 0x6d, 0x7d, 0x07,  0x7f, 0x6f, 0x81, 0x79,  0x38, 0x3e, 0x1e, 0x00 };


/*
 * NOT USED YET
 * Data in this structure are written to the eeprom
 * 
 * for now, this is unused. the eeprom could store lot of 32 bits bipolars but
 * it requires lot of programming to manage / choose and mega88 is full.
 * 
 */
 struct t_nvram {

  byte cm_okflg = 0x55;

} ;


class NvRam {
  public:
  t_nvram  d; //data
  void Init()
  {
      //Read or Init the eeprom
      char *b = &d.cm_okflg; 

      for ( byte i = 0; i < sizeof(d); i++, b++)
      {
        *b = EEPROM.read(i);
      }

      if (d.cm_okflg == 0x55)
      {
        Info.showNumberDec(45);
        return; // nvram is ok
      }

      //Set defaults
      t_nvram defaults;
      d = defaults; 
      Write();
      // Enter diagnostic mode
      Info.showNumberDec(42);
      DiagMode();
  };
  void Write()
  {
      //Write the updated bytes
      char *b = &d.cm_okflg; 

      for ( byte i = 0; i < sizeof(d); i++, b++)
      {
        EEPROM.update(i,*b);
      }
  }
  void DiagMode();

};

// NvRam nv;



void NvRam::DiagMode()
{
/*
  static int p = 0;
  if (! Sw.Ready()) return;
  switch (Sw.Read())
  {
  case 48:
      p++;
      if (p > sizeof(t_nvram))  p = 0;
      break;
  case 49:
      p--;
      if ( p < 0) p = sizeof(t_nvram);
      break;
  }

  Info.showNumberDec( EEPROM.read(p) );
  */
}



/* 
 * The Interrupt routine, every millisecond:
 * 
 * Update shift registers.
 * 
 * Decrease timers
 * 
 * Make the blinking byte blink
 * 
 * Display the CRC low or high part
 * 
 * Limit calls to TM1637 which is time costly
 * 
 */
ISR (TIMER1_COMPA_vect) {
   static unsigned int blinkCount;
   static bool         ttu;                 //time to update
   static byte         actualDigits [4];    // what is really on the display
   static byte         localDigits [4];     // what we want to display
  
  /*
   * Compiler produces a good code but I wanted to test weird assembler interface !
   */
  register unsigned char ByteToShift asm("r2");
  asm volatile (

    // Instruction        Clock   Description   Phase     Bit Transmitted

    "ldi  r30, lo8(HC595_REGS)       $"   // Z, points to byte array to shift
    "ldi  r31, hi8(HC595_REGS)       $"


    // BYTE 0 load byte from array
    "ld %[bts],  Z+                  $"

    "cbi %[hc595], %[hc595_srclk]    $"   // clk low
    "sbi %[hc595], %[hc595_ser]      $"   // set high
    "sbrs %[bts], 7                  $"   // Skip next instruction if Bit 7 is Set
    "cbi %[hc595], %[hc595_ser]      $"   // set low
    "sbi %[hc595], %[hc595_srclk]    $"   // clk high
    // repeat for remaining bits
    "cbi %[hc595], %[hc595_srclk]    $"   // clk low
    "sbi %[hc595], %[hc595_ser]      $"
    "sbrs %[bts], 6                  $"
    "cbi %[hc595], %[hc595_ser]      $"
    "sbi %[hc595], %[hc595_srclk]    $"   // clk high

    "cbi %[hc595], %[hc595_srclk]    $"   // clk low
    "sbi %[hc595], %[hc595_ser]      $"
    "sbrs %[bts], 5                  $"
    "cbi %[hc595], %[hc595_ser]      $"
    "sbi %[hc595], %[hc595_srclk]    $"   // clk high

    "cbi %[hc595], %[hc595_srclk]    $"   // clk low
    "sbi %[hc595], %[hc595_ser]      $"
    "sbrs %[bts], 4                  $"
    "cbi %[hc595], %[hc595_ser]      $"
    "sbi %[hc595], %[hc595_srclk]    $"   // clk high

    "cbi %[hc595], %[hc595_srclk]    $"   // clk low
    "sbi %[hc595], %[hc595_ser]      $"
    "sbrs %[bts], 3                  $"
    "cbi %[hc595], %[hc595_ser]      $"
    "sbi %[hc595], %[hc595_srclk]    $"   // clk high

    "cbi %[hc595], %[hc595_srclk]    $"   // clk low
    "sbi %[hc595], %[hc595_ser]      $"
    "sbrs %[bts], 2                  $"
    "cbi %[hc595], %[hc595_ser]      $"
    "sbi %[hc595], %[hc595_srclk]    $"   // clk high

    "cbi %[hc595], %[hc595_srclk]    $"   // clk low
    "sbi %[hc595], %[hc595_ser]      $"
    "sbrs %[bts], 1                  $"
    "cbi %[hc595], %[hc595_ser]      $"
    "sbi %[hc595], %[hc595_srclk]    $"   // clk high

    "cbi %[hc595], %[hc595_srclk]    $"   // clk low
    "sbi %[hc595], %[hc595_ser]      $"
    "sbrs %[bts], 0                  $"
    "cbi %[hc595], %[hc595_ser]      $"
    "sbi %[hc595], %[hc595_srclk]    $"   // clk high

    "ld %[bts],  Z+                  $"   // Next byte from the array

    "cbi %[hc595], %[hc595_srclk]    $"   // clk low
    "sbi %[hc595], %[hc595_ser]      $"   // set high
    "sbrs %[bts], 7                  $"   // Skip next instruction if Bit 7 is Set
    "cbi %[hc595], %[hc595_ser]      $"   // set low
    "sbi %[hc595], %[hc595_srclk]    $"   // clk high

    "cbi %[hc595], %[hc595_srclk]    $"   // clk low
    "sbi %[hc595], %[hc595_ser]      $"
    "sbrs %[bts], 6                  $"
    "cbi %[hc595], %[hc595_ser]      $"
    "sbi %[hc595], %[hc595_srclk]    $"   // clk high

    "cbi %[hc595], %[hc595_srclk]    $"   // clk low
    "sbi %[hc595], %[hc595_ser]      $"
    "sbrs %[bts], 5                  $"
    "cbi %[hc595], %[hc595_ser]      $"
    "sbi %[hc595], %[hc595_srclk]    $"   // clk high

    "cbi %[hc595], %[hc595_srclk]    $"   // clk low
    "sbi %[hc595], %[hc595_ser]      $"
    "sbrs %[bts], 4                  $"
    "cbi %[hc595], %[hc595_ser]      $"
    "sbi %[hc595], %[hc595_srclk]    $"   // clk high

    "cbi %[hc595], %[hc595_srclk]    $"   // clk low
    "sbi %[hc595], %[hc595_ser]      $"
    "sbrs %[bts], 3                  $"
    "cbi %[hc595], %[hc595_ser]      $"
    "sbi %[hc595], %[hc595_srclk]    $"   // clk high

    "cbi %[hc595], %[hc595_srclk]    $"   // clk low
    "sbi %[hc595], %[hc595_ser]      $"
    "sbrs %[bts], 2                  $"
    "cbi %[hc595], %[hc595_ser]      $"
    "sbi %[hc595], %[hc595_srclk]    $"   // clk high

    "cbi %[hc595], %[hc595_srclk]    $"   // clk low
    "sbi %[hc595], %[hc595_ser]      $"
    "sbrs %[bts], 1                  $"
    "cbi %[hc595], %[hc595_ser]      $"
    "sbi %[hc595], %[hc595_srclk]    $"   // clk high

    "cbi %[hc595], %[hc595_srclk]    $"   // clk low
    "sbi %[hc595], %[hc595_ser]      $"
    "sbrs %[bts], 0                  $"
    "cbi %[hc595], %[hc595_ser]      $"
    "sbi %[hc595], %[hc595_srclk]    $"   // clk high


    //and move registers to output lactches
    "sbi %[hc595], %[hc595_rclk]    $"   // high
    "cbi %[hc595], %[hc595_rclk]    $"   // low

    :// output operands
    :// input operands
    [bts]          "r" (ByteToShift),
    [hc595_srclk]  "M" (HC595_SRCLK),
    [hc595_ser]    "M" (HC595_SER),
    [hc595_rclk]   "M" (HC595_RCLK),
    [hc595]        "I" _SFR_IO_ADDR(HC595) 
    :// dirty regs
    "r26", "r30", "r31"

  );

  // for reading buttons, maybe simplified ?
  if (countDownRebound) countDownRebound--;
  if (countDown) countDown--;

  // Blinking ~1 second
  // by masking or not the two rightmost digits
  if (!blinkCount--) {
      blinkCount = 400;  // new cycle
      blinkState = !blinkState;
      ttu=true;          // time to update

      if (! (canBlink && canBlink2) ) {
        blinkState = 0; // force visible anyway
      }

      // CRC 8 digits repeatedly low or high or nothing
      switch (displayCrc) {

        case 10:  // high part
          digits[0] = digitToSegment[0xf & crc>>28];
          digits[1] = digitToSegment[0xf & crc>>24];
          digits[2] = digitToSegment[0xf & crc>>20];
          digits[3] = digitToSegment[0xf & crc>>16];
          displayCrc--;
          break;
        case 6:   // low part
          digits[0] = digitToSegment[0xf & crc>>12];
          digits[1] = digitToSegment[0xf & crc>>8];
          digits[2] = digitToSegment[0xf & crc>>4];
          digits[3] = digitToSegment[0xf & crc];
          displayCrc--;
          break;
        case 1:
          displayCrc = 10;  // restart sequence
        case 0:;            // off, do nothing
          break;
        case 3:   // 'blank' part by clearing all
          actualDigits [0] = actualDigits [1] = actualDigits [2] = actualDigits [3] = 0;
          localDigits [0] = localDigits [1] = localDigits [2] = localDigits [3] = 0;
          digits [0] = digits [1] = digits [2] = digits [3] = 0;
          Info.setSegments (actualDigits, 4, 0);

        default:
          displayCrc--;  // introduce delays to have time to read the CRC between states.
      }
  }

  // check if 'to' display is different from actual display
  if ( 
        (localDigits [0] != digits [0]) ||
        (localDigits [1] != digits [1]) ||
        (localDigits [2] != digits [2]) ||
        (localDigits [3] != digits [3])
     ){
      ttu = true;
      localDigits [0] = digits [0];
      localDigits [1] = digits [1];
      actualDigits [0] = digits [0];
      actualDigits [1] = digits [1];
      localDigits [2] = digits [2];
      localDigits [3] = digits [3];
  }


  // finally decides to update or not the TM1637
  if (ttu) {
  
    if (blinkState) {  // blink the value, not the address
      actualDigits [2] = 0;
      actualDigits [3] = 0;
    } else {  
      actualDigits [2] = localDigits [2];
      actualDigits [3] = localDigits [3];
    }
    Info.setSegments (actualDigits, 4, 0);
    ttu=false;
  }
};

/*
 * Some good info here to set up Timers 
 *   http://gammon.com.au/interrupts
 *   https://www.arduinoslovakia.eu/application/timer-calculator
 * 
 */
void setupTimer1() {
  //TCCR1A = 0;                        // normal operation
  //TCCR1B = bit(WGM12) | bit(CS10);   // CTC, no pre-scaling
  //OCR1A =  999;                      // compare A register value (1000 * clock speed)
  //TIMSK1 = bit (OCIE1A);             // interrupt on Compare A Match
  // set up Timer 2
  //TCCR2A = bit (COM1A0);            // toggle OC1A on Compare Match
  //TCCR2B = bit(WGM12) | bit(CS10) | bit (CS12);   // CTC, scale to clock / 1024

  noInterrupts();
  // Clear registers
  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1 = 0;

  // 1000 Hz (8000000/((124+1)*64))
  OCR1A = 124;
  // CTC
  TCCR1B |= (1 << WGM12);
  // Prescaler 64
  TCCR1B |= (1 << CS11) | (1 << CS10);
  // Output Compare Match A Interrupt Enable
  TIMSK1 |= (1 << OCIE1A);
  interrupts();
}

/*
 * 
 * Some debug helper and probably serial interface dialog
 * 
 */
void printHex (byte v) {
  //arduino provided funct is not displaying leading zero, not good.
  char x = (v >> 4) + '0';
  if (x>'9') x += 7;
  Serial.print (x);
  x = (v & 0x0f) + '0';
  if (x>'9') x += 7;
  Serial.print (x);
}

void printBin (byte v) {
  for (byte bit=8; bit; bit--) {
    if ( v & _BV (bit-1) ) Serial.print("1"); else Serial.print("0");
    if (bit==5) Serial.print(".");
  } 
}

void debug() {
  for (promAddress=0; promAddress<32; promAddress++) {
    if (promAddress<10) Serial.print("0");
    Serial.print(promAddress);
    Serial.print(": ");
    printBin (promReadData[promAddress]);
    Serial.print("(");
    printHex (promReadData[promAddress]);
    Serial.print(") : (");
    printHex (promWriteData[promAddress]);
    Serial.print(")");
    printBin (promWriteData[promAddress]);
    if (promReadData[promAddress] != promWriteData[promAddress] ) Serial.print(" <==");
    Serial.println();
  }
}

void beepBad(){
  byte irq = TIMSK1; // noInterrupts();
  TIMSK1 = 0;
  tone (buzzer, 80);
  delay (400);
  noTone(buzzer);
  TIMSK1 = irq;      // restore interrupts();
}

void beepOk(){
  byte irq = TIMSK1; // noInterrupts();
  TIMSK1 = 0;
  tone (buzzer, 600);
  delay (100);
  noTone(buzzer);
  tone (buzzer, 500);
  delay (50);
  noTone(buzzer);
  TIMSK1 = irq;      // restore interrupts();
}

void beep2(){
  byte irq = TIMSK1; // noInterrupts();
  TIMSK1 = 0;
  tone (buzzer, 120);
  delay (50);
  noTone(buzzer);
  delay (50);
  tone (buzzer, 120);
  delay (50);
  noTone(buzzer);
  TIMSK1 = irq;      // restore interrupts();
}


void setDisplay (byte address, byte data) {
  digits [0] = digitToSegment[address / 10];  // address is decimal
  digits [1] = digitToSegment[address % 10];
  digits [2] = digitToSegment[data >> 4];     // value is hexa displayed
  digits [3] = digitToSegment[data & 0xf];
  //Info.setSegments (digits, 4, 0);
}


#if (0)
/*
 * 
 *  This crc32 is a little bit longer than the iterative version.
 *  It can also be used 'on the fly', byte per byte
 *  https://wiki-content.arduino.cc/en/Tutorial/LibraryExamples/EEPROMCrc
 *  
 */
const uint32_t crc_table[16] = {
    0x00000000, 0x1db71064, 0x3b6e20c8, 0x26d930ac,
    0x76dc4190, 0x6b6b51f4, 0x4db26158, 0x5005713c,
    0xedb88320, 0xf00f9344, 0xd6d6a3e8, 0xcb61b38c,
    0x9b64c2b0, 0x86d3d2d4, 0xa00ae278, 0xbdbdf21c
};

uint32_t crc_update(uint32_t crc, byte data)
{
    crc = crc_table[(crc ^ data) & 0x0f] ^ (crc >> 4);
    crc = crc_table[(crc ^ (data >> 4)) & 0x0f] ^ (crc >> 4);
    return crc;
}

uint32_t crc32(const char *s)
{
  uint32_t crc = 0xFFFFFFFF;
  for(size_t i=0;i<32;i++) {
    byte data=s[i]; 
    crc = crc_table[(crc ^ data) & 0x0f] ^ (crc >> 4);
    crc = crc_table[(crc ^ (data >> 4)) & 0x0f] ^ (crc >> 4);
  }
  return ~crc;
}
#else
/*
 * 
 * CRC-32 on the file
 * https://lxp32.github.io/docs/a-simple-example-crc32-calculation/
 * 
 */
uint32_t crc32(const char *s) {
  uint32_t crc=0xFFFFFFFF;
  
  for(size_t i=0; i<32; i++) {
    byte ch=s[i];

    for(size_t j=0; j<8; j++) {
      uint32_t b=(ch^crc)&1;
      crc>>=1;
      if(b) crc=crc^0xEDB88320;
      ch>>=1;
    }
  }
  return ~crc;
}
#endif

/*
 * 
 * Read the 'read' zif socket
 * If read all FF or all 00, socket is empty, replace with 'jeutel' data
 * Also starts the 'crc' display mode
 * 
 */
void promReadRead() {
  setLed(1); 
  setLedR(0);
  setLedW(1);
  PORTB = 255; //turn on pull up resistors
  delay(250);
  Serial.print ("R:");

  for (promAddress=0; promAddress<32; promAddress++) {
    setPromAddr(promAddress);
    delay(30);  // let at least 2 irq to update shift registers
    promReadCS0;
    delayMicroseconds(50);
    promReadData[promAddress] = PINB;
    promReadCS1;
    setDisplay (promAddress, promReadData[promAddress]);
    printHex(promReadData[promAddress]); // serial display
    Serial.print(" ");
    delay(3);
    wdt_reset();
  }
  PORTB = 0; //turn off pull up resistors
  Serial.println();

  //Check the readings: all zero, new empty proms
  //                    all equal (FE, FF,.. ), empty socket, goes with Jeutel

  promReadEmpty = true; 
  bool eq;
  for (int n = 0; n<256; n++) {
    eq = true;   
    for (promAddress=0; promAddress<32; promAddress++) {
      if (promReadData[promAddress] !=  n) {
        eq=false;
        break;
      }
    }
    if (eq && !n) {
      beepBad();
      return;
    }
    if (eq && n) {
      // copy jeutel
      for (promAddress=0; promAddress<32; promAddress++) {
        promReadData[promAddress] = jeutel[promAddress];
        setDisplay (promAddress, promReadData[promAddress]);
      }
      beepOk();  // beep twice to indicate read from memory
    }
  }

  beepOk();
  promAddress--; // adjust Display == Index
  promReadEmpty = false;
  
  crc = crc32(promReadData);
  Serial.println (crc, HEX);
  displayCrc = 10;  // start the display 'case' showing the crc 
}

/*
 * Read the Write socket
 *   Turn on the 5v 
 *   read byte and compares to what is expected.
 *   If not matching, sets partialBurn to true
 */
bool partialBurn;
void promWriteRead() {
  setLed(1); 
  setLedW(0);
  setLedR(1);
  partialBurn = false;  // assume a win
  Serial.print("W:");
  clearPromBitSelector;
  PORTD = vppNoPulse;   // 5v while buck is 'off' goes through the bd139
  PORTB = 255;          // turn on pull up resistors

  for (promAddress=0; promAddress<32; promAddress++) {
    setPromAddr(promAddress);
    delay(30);  // let at least 2 irq to update shift registers
    promWriteCS0;
    delayMicroseconds(50);
    promWriteData[promAddress] = PINB;
    promWriteCS1;
    if (promWriteData[promAddress] != 0) partialBurn = true; //programming error
    setDisplay (promAddress, promWriteData[promAddress]);
    printHex(promWriteData[promAddress]);
    Serial.print(" ");
    wdt_reset();
  }
  PORTD = vppNoPulse+vppNoBoost;
  PORTB = 0; //turn off pull up resistors
  Serial.println();
  promAddress--; // adjust Display == Index
}

/*
 * 
 *  Animate buck and transistors to produce the pulse
 *  
 *  There is no automatic retry. Relauching a burn sequence will
 *  not touch already fused bit and do only what is required.
 *  
 *  A major diff with Elektor is duration of the pulse
 *  - datasheet for 82s23 says 10µs
 *  - Elektor uses 400µs
 * 
 *   Because it works with 10µs and I don't have a lot of 82S23 to throw away, 400µs is not tested
 *   
 */
void doPulse() {

  noInterrupts();
  PORTD = buckOn+vppNoPulse+vppNoBoost;//buck on + pulse closed + vcc 5v
  delay(20);                           // 3 to 4 ms to stabilize buck
  PORTD = buckOn+vppNoPulse;           //buck on + pulse closed + vcc 10v
  delayMicroseconds(100);
  PORTD = buckOn;                   // VOUT on : Q3 off, current toward the prom
  delayMicroseconds(100);           // let go up
  PORTC = 0b11111110;               // promWrite CS=0
  delayMicroseconds(10);            // fusing (400µ Elektor), 9 to 11µs on datasheet
  PORTC = 0b11111111;               // promWrite CS=1
  delayMicroseconds(5);             // data sheet insert delay
  PORTD = buckOn+vppNoPulse;        // power off phase 1
  delayMicroseconds(5);             // data sheet insert delay
  PORTD = vppNoPulse+vppNoBoost;    // power off phase 2
  interrupts();
  delay(500);                       // let cool the prom, we are not going after speed.
}


/*
 * Copy: main goal of the prog.
 * 
 * - must have read something with promR
 * - read promWrite content
 * - for each bit : compare and burn when possible (no unburn here,its OTP)
 * 
 */
void promWriteCopy(){

  if (promReadEmpty) {
    beepBad();
    return;
  }

  //We have the source, read content
  promWriteRead();

  //Minimize relays clicks by  doing by bit columns
  for (byte bit=0; bit<8; bit++) {
    setPromBitSelector(bit);
    delay(200); //relay response
    for (byte address=0; address<32; address++) {
      wdt_reset();
      setDisplay (address, bit);
      byte w = promWriteData[address];
      byte r = promReadData[address];
      setPromAddr(address);
      delay(30);  // let at least 2 irq to update shift registers
      // r=255;   // usefull for testing
      // w=0;
      if ( r & _BV (bit) ) {         // bit needs to be burnt
        if ((w & _BV (bit)) == 0) {  // already burnt, skip it
          if (partialBurn) {
            // not really usefull , just info
            Serial.print("Burning bit ");
            Serial.print(address);
            Serial.print("/");
            Serial.println(bit);
          }
          doPulse();
        }
      }
    }
    clearPromBitSelector;
    delay(500);
  }
  delay(300);
  PORTD=vppNoPulse+vppNoBoost;  

  // Read back the programmed prom and compare
  promWriteRead();
  byte err = false;
  for (byte address=0; address<32; address++) {
    byte w = promWriteData[address];
    byte r = promReadData[address];
    if ( r != w ) {
      err = true;
    }   
  }
  if (err) {
    beepBad();
    setLed(1); 
  } else {
    beepOk();
    setLed(0);   // the green Led , success ;)
  }
}

/*
 * 
 * Handle press/release/repeat
 * of each button
 * 
 */

buttonsMessage getButton(){
  unsigned int buttonVoltage;
  
  buttonVoltage = analogRead( AnalogButtons );

  //remove rebounds for 6ms
  if (countDownRebound) return btNone;
  
  //accept the press if nearly equal for 6ms
  if (abs (lastButtonVoltage - buttonVoltage) > 20 ) {
    lastButtonVoltage = buttonVoltage;
    noInterrupts();
    countDownRebound = 6;
    interrupts();
    return btNone;
  }
  // now check what button to translate the level to

  if (buttonVoltage > 882) {  // all butons released
    btInRepeat = false;
    noInterrupts();
    countDown = 0;
    interrupts();

    //insert one Release button event
    if (lastBt != btNone)
    switch (lastBt) {
      case btRead:  lastBt = btNone; return btReadRel;
      case btWrite: lastBt = btNone; return btWriteRel;
      case btMoins: lastBt = btNone; return btMoinsRel;
      case btPlus:  lastBt = btNone; return btPlusRel;
      case btVal:   lastBt = btNone; return btValRel;
    }
    return btNone;
  }

  // we have a button
  // repeat mode ?
  if (btInRepeat) {
    //check delay before repeating
    noInterrupts();
    if (countDown) {  //not yet
      interrupts();
      return btNone;
    }
    //Restart countDown and send the button
    countDown = repeatSpeed;
    interrupts();
    tone (PD7, 1000,2);
    return lastBt;
  }

  // just pressed button
  noInterrupts();
  countDown = 600;  //delay a little longer than repeatSpeed;
  interrupts();

  // find which button is pressed
  btInRepeat = true;
  if (buttonVoltage > 623) {
    lastBt = btWrite;
  } else
  if (buttonVoltage > 420) {
    lastBt =  btVal;
  } else
  if (buttonVoltage > 237) {
    lastBt =  btPlus;
  } else
  if (buttonVoltage > 70) {
    lastBt =  btMoins;
  }
  else lastBt =  btRead;
 
  return lastBt;
}



void setup()
{
  DDRB = B00000000;  // Port B input  (set pull up when reading)
  DDRC = B11011111;  // ADC on PC5
  DDRD = B11111110;  // input for buttons
  PORTC = 0;
  PORTD = vppNoPulse+vppNoBoost;      // no buck, no vcc, close Q3

  promReadCS1;
  promWriteCS1;

  HC595_REGS[0] = 0;
  HC595_REGS[1] = 0;
  setLed(1);
  setLedR(1);
  setLedW(1);
  clearPromBitSelector;

  //Timer at 1ms interrupt
  setupTimer1();  // irq will latch ASAP

  wdt_enable(WDTO_2S);

  //Info.TM1637Display(tm1637_c,tm1637_d);
  Info.setBrightness(7);

  Serial.begin(115200);
  beep2();
  Serial.println ("\nStarted Bipolar Prom Programmer " VERSION);
};

/*
 * Detect lonpress on 'V' 
 * 
 * Todo : do something, for example file selection in eeprom
 * 
*/
long int vpress;
bool vpressed=false;
bool t1, t2;

void loop(){

  wdt_reset();

  //main switches  
  switch (getButton()) {

    case btRead:
      if (!promReadEditData) { 
        promReadRead();
      }
      break;
    case btWrite:
      displayCrc = 0;
      if (!promReadEditData) {
        promWriteCopy();
      }
      break;
    case btMoinsRel: //during autorepeat, always disable blinking for readability
    case btPlusRel:
      canBlink2 = true; // allow blinking
      break;
    case btPlus:
      if (promReadEditData) {
        promReadData[promAddress]++;
        promReadEmpty = false;
      } else {
        promAddress++;
        promAddress &= 0B11111;
        setPromAddr(promAddress); //debug
      }
      canBlink2 = false;
      displayCrc = 0;
      setDisplay (promAddress, promReadData[promAddress]);
      break;
    case btMoins:
      if (promReadEditData) {
        promReadData[promAddress]--;
        promReadEmpty = false;
      } else {
        promAddress--;
        promAddress &= 0B11111;
        setPromAddr(promAddress); //debug
      }
      canBlink2 = false;
      displayCrc = 0;
      setDisplay (promAddress, promReadData[promAddress]);
      break;
    case btVal:
      if (!vpressed) {
        vpress = millis();
        vpressed = true;
      }
      displayCrc = 0;
      break;
    case btValRel:/******************
      t1 = (millis() - vpress) > 1000;
      t2 = (millis() - vpress) > 3000;

      if (t2) {
        t1 = false;
        //Serial.println ("T2");
        vpressed=t1=t2=false;
        break;
      }
      if (t1) {
        Serial.println ("T1");
        vpressed=t1=t2=false;
        break;
      }
      **********************/
      promReadEditData = !promReadEditData; //wait the button release
      if (promReadEmpty) { // at startup, display is null, so turn it on                
        setDisplay (promAddress, promReadData[promAddress]);
        canBlink2 = true;
        promReadEmpty = false; 
      }
      canBlink = promReadEditData;
      if (promReadEditData)
        repeatSpeed=100;
      else
        repeatSpeed=200;
 
    default:;
  }
}
