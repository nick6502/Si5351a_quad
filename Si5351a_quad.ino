
// V1.20, January 23, 2025
// After this revision is complete, the next revision will be V2.0 and will include the
// keyer.
// V1.21 Added keyer plus HW-8 mode
// V2.1c, Mateusz reported sideband selection didn't work. Fix in doLSB_USB()
// V2.1d, Don VE3IDS revealed problems with erratic frequency output 3/29/2025
// V2.1e, will add quick method for Phone/CW switch, request from Mateusz,. start 4/2/2025
// V2.1e also added bried display of keyer speed after changing
// V2.1f Mateusz reported CLK2 keyed ON in "TU" opening message
// V2.2 Issue major revision
// V2.2a, fixed a couple RIT clear and RIT swap problems reported by VE3IDS 4/15/2025

// I started this program in May, 2018

// interim version 2.0e.  After erratic program operation and noticing that I'm almost
// out of RAM, I started using the 'F' macro to store strings in Serial.print() statements
// that may have fixed it.
// V2.0g - abandoning shared pins for LCD.  Now DB7 will use D13 and DB6 will use A2
// SB select line will be removed (was A2)

// Note IC2_LCD is defined (or not) in the Si5351a_quad_config.h file

 #include "Si5351a_quad_config.h"

#include <EEPROM.h>
#include <Wire.h> 

#ifdef RELAY_CONTROL
#include <Adafruit_PCF8575.h>
Adafruit_PCF8575 pcf;
#endif




// NRK - Note that A0, A1, A2 are PORTC pins 0, 1 & 2.

#define ENC_A 3       // Encoder pin A to D3 (V1.8, was A1)
#define ENC_B 2       // Encoder pin B to D2 (V1.8, was A0)
//#define LEDPIN 13	   // Internal LED, can also use for I/O NRK, V2.0g got rid of it
#define spkrpin 9     // Pin for audio to speaker for beep
#define SW1 12		  // Pin for pushbutton PB1
#define SW2 11        // Pin for pushbutton PB2
#define Key_In 8 // Pin to monitor for key_down status. Reads LOW when key is down
#define TX_Out 10 // Echoes Key_In but changes *after* registers sent to chip
//#define SB_Relay A2 // Sideband select. HIGH in LSB, LOW in USB

#define ENC_AB_MASK 0b00001100 // mask off encoder bits V1.8 was 0,1 now 2,3
#define LCDBottomLine 1
#define LCDTopLine 0


// Define bits for output level control. The lowest two bits will
// be forced to the values shown.  Added 9/13/2020, this is for an
// experiment. The program will always maintain the highest output
// level.

#define mA_2 0b11111100
#define mA_4 0b11111101
#define mA_6 0b11111110
#define mA_8 0b11111111


// define EEPROM register locations for variable storage

  #define EE_FoutTX 0
  #define EE_hamBand_index EE_FoutTX + 4
  #define EE_step_index EE_hamBand_index + 1
  #define EE_flag EE_step_index + 1 // this tells the program EEPROM has been written
  #define EE_LSBMode EE_flag + 1
  #define EE_modeCW EE_LSBMode + 1
  #define EE_CW_pitch EE_modeCW + 1 // this one will use 2 bytes v1.4b
  #define EE_sidetone EE_CW_pitch + 2 // save boolean
  #define EE_QUAD EE_sidetone + 1 // V1.13, boolean
  #define EE_TX_CLK2 EE_QUAD + 1 // V1.13, boolean
  #define EE_CLK0_1_OFF_KYDN EE_TX_CLK2 + 1 // V1.13, boolean
  #define EE_TRIM EE_CLK0_1_OFF_KYDN + 1 // V1.13, boolean
  #define EE_DIV_FACTOR EE_TRIM + 1 // V1.13, uint8_t
  #define EE_X4 EE_DIV_FACTOR + 1 // V1.13 boolean
  #define EE_TX_ONLY EE_X4 + 1 // V1.19 boolean
  #define EE_keyDNstate EE_TX_ONLY + 1 // V2.0
  #define EE_SpeedPot EE_keyDNstate + 1
  #define EE_ModeA EE_SpeedPot + 1


// V2.1b 2/19/2025 below defs not used, so comment out for now:

/*
#define I2C_START 0x08
#define I2C_START_RPT 0x10
#define I2C_SLA_W_ACK 0x18
#define I2C_SLA_R_ACK 0x40
#define I2C_DATA_ACK 0x28
#define I2C_WRITE 0b11000000 // slave address is 0x60, 7 bit but here shifted
#define I2C_READ 0b11000001 // left 1 place so b0 of 8 bit field 0 is R/W bit
                            // NRK
                            */

#ifndef SI5351A_H
#define SI5351A_H

#define SI_CLK0_CONTROL 16 // Register definitions
#define SI_CLK1_CONTROL 17
#define SI_CLK2_CONTROL 18
#define SI_SYNTH_PLL_A 26
#define SI_SYNTH_PLL_B 34
#define SI_SYNTH_MS_0 42
#define SI_SYNTH_MS_1 50
#define SI_SYNTH_MS_2 58
#define SI_PLL_RESET 177
#define CLK0_PHOFF 165
#define CLK1_PHOFF 166
#define CLK2_PHOFF 167 // 3/17/2020 - WAS CLK1_PHOFF, but don't think this is used anyway

#define SI_R_DIV_1 0b00000000 // R-division ratio definitions
#define SI_R_DIV_2 0b00010000
#define SI_R_DIV_4 0b00100000
#define SI_R_DIV_8 0b00110000
#define SI_R_DIV_16 0b01000000
#define SI_R_DIV_32 0b01010000
#define SI_R_DIV_64 0b01100000
#define SI_R_DIV_128 0b01110000

#define SI_CLK_SRC_PLL_A 0b00000000
#define SI_CLK_SRC_PLL_B 0b00100000
#define denom 1048575UL // V1.1, denom is constant 2^20 - 1

	
	
//  ** W0EB definitions ***

// V2.1b 2/20/2025 moving address definition below to header file 
// for easire editing by users

//#define SI5351BX_ADDR 0x60   // I2C address of Si5351   (typical)
   
void si5351aOutputOff(uint8_t clk);
// void Set_Freq(uint32_t frequency); V2.0i, never called so I'm removing


#endif //SI5351A_H - corresponding #if is #ifndef SI5351A_H, up 30 lines

// NRK add some definitions for LCD pins


#define RS	  7 // LCD RS 
#define E		  6 // LCD Enable 
#define DB4		A0 // LCD DB4 V1.8, was 2
#define DB5		A1 // LCD DB5 V1.8, was 3
#define DB6		A2 // LCD DB6  - shared with paddle dot contact V2.0g, no more sharing
#define DB7		13 // LCD DB7  - shared with paddle dash contact,  V2.0g, no more sharing



// Note A4 is for SDA
// Note A5 is for SCL

// Below V2.0 defines from mini keyer

	#define DotContact 4 // is is shared with LCD
	#define DashContact 5 // is is shared with LCD
  #define POT_ADC A3	
	#define WPM 22
	#define SPD_LIMIT_LO 10.0 
	#define SPD_LIMIT_HI 35.0 // lowered from 45 to 35 for W A 5 B D U, who can't send that fast
	#define CMND_SPEED 22
  #define KYR_hold 3000 // USERS: milliseconds of KYR mode after paddle sending
    
// ************     GLOBAL VARIABLES    ****************************

	unsigned long timing_start, timing_start_2, timing_keyer;
  uint16_t loopCounter; // for testing/timing main loop V2.0 moved definition out ofloop
   volatile uint8_t encoder_pins; // last state of rotary encoder pins V1.8
   uint8_t encoder_now;
   uint8_t r_dir; // 1 is increment (CW), 2 decrement, 0 nothing V1.8 not used
	volatile int queue = 0; // step backlog, + for UP, - for DN V1.8
	//volatile uint8_t Divider = 0; // use to scale # of transitions per step
	volatile bool do_count = false; // togggle for when scaling transitions per step is desired
	//volatile bool do_countL = true; // to combine div4 with do_count
	volatile bool div4 = true;
	//volatile int qMAX = 0; // for TESTING to see how big it gets 
	//char qmaxASC[8]; // ******** TESTING *********
	//char qmax4LCD[3];
	
   uint8_t up_pile = 0; // count of CW encoder pulses, for low_rez
   uint8_t dn_pile = 0; //          CCW
   
   volatile uint8_t encoder_div = 0; // V1.9 counter to determine # of pulses per step

 
  bool FastStart = false; // add to V1.13 V1.14 replaced by Verbose V2.0 bring it back
  bool QUAD = true; // V1.13, do provide quadrature outputs on CLK0 & CLK1
  bool TX_CLK2 = false; // V1.13 don't provide TX output on CLK2
  bool CLK0_1_OFF_KYDN = false; // V1.13 don't turn off CLKs 0 & 1 on key down
  bool TRIM = true; // V1.13 do zero less significant digits on larger step
  uint8_t DIV_FACTOR = 4; // V1.13 encoder pulses per count. NOTE: EEPROM GOVERNS
  bool TX_ONLY = false; // V1.19, output only on key down, from CLK0, plus 1 if quadrature
	bool TX_ONLY_OLD = false; // V2.1e flag for exiting Send CW mode
  bool SPOT = false; // V1.19 toggled by PB1. When true, output is ON in key up state
  bool KeyerActive = false; // V2.0, paddle closed within the last 3 seconds then true
  bool printedKYR = false; // true when KYR was last printed to LCD field
	bool LCDtrash = false; // true if WPM update or other trashes display
	unsigned long LCDFixTime; // Need to time how long before fixing screen
  uint8_t EE_flagValue = 255; // V1.13, will be 170 after EEPROM written
  bool DidReadEE = false; // V2.1 I need to know if I did read EEPROM
  bool SkipEEread = false; // True when we want to skip EEPROM read. PB2Long during S/U

 //    FUNCTION DECLARATIONS 
 
	void RotaryISR(); // V1.8 - declaration
	bool GetButtons(); // "" 
  void instructions();
  void doPitch();
  void doCWPhone();
  void showSidetone();
	//void SPRINT_inhibits(); // TEMPORARY FOR TROUBLESHOOTING

  uint32_t FoutRX; // Desired output frequency V1.1, have RX and TX
	uint32_t FoutTX; 
	uint32_t RITSave; // Store FoutRX here when turning RIT off, so can restore
  #ifdef HW8 // V2.oi
  //#define HW8startDial 50000 // 50 kHz on dial
  uint32_t HW8DialRX = 50000;
  uint32_t HW8DialTX = 50000;
  #define HW8FoutMax 8895000 // Fout is this when dial is at 000
  uint32_t HW8FQtoDial(uint32_t actualFQ);
  uint32_t HW8DialtoFQ(uint32_t dialrdg);
 // void DoPrintHW8(); // ***********  FOR TESTING 

  #endif

	void menu();
	void showChoice();
	void doSteps();
	void showStepSize();
	void doBands();
	void showBand();
	void doSave();
	void any_action(uint16_t action_time);
	void setupPLL(uint8_t pll, uint8_t mult, uint32_t num);
	void calc_RX(uint32_t frequency);
	void calc_TX(uint32_t frequency);
	void send_regs_RX();
	void send_regs_TX();
	// void Set_Freq(uint32_t frequency); V2.0i declared above, so comment out
	void clock_0_ON();
	void clock_1_ON();
	void clock_2_ON();
  void clock_2_OFF(); // V1.14
  void clock_0_OFF(); // V1.19
  void clock_1_OFF(); // V1.19 
	void CLK0_1_on_quad(); // V2.1f
  void spot_it(); // V1.19
	void step_up();
	void step_down();
	void stepSize_up();
	void stepSize_down();
	uint8_t low_rez();
	uint8_t get_direction();
	void clearLine1();
	void clearLine0();
	uint8_t read_switches();
	uint8_t getSwitchWait();
	void doLSB_USB();
	void RIT_ON();
	void RIT_OFF();
	void TXRXSwap();
	void Go_RIT();
	void ShowStepFast();
	void UpdateScreen();
	void SingleBeep();
	void DoubleBeep();
  void bad_sound();
	void Go_(uint32_t thefreq);
	static void Print_freq(uint32_t f32, uint8_t line);
	uint8_t Rotary();
	void doAdvanced(); // V1.14
	void showChoiceA(); // V1.14, including next one
  void doDivFac();
  #ifdef RELAY_CONTROL
  void Select_Relay(uint8_t HBindex);
  //void CycleBands(); // V1.15 for testing relay control
  #endif
	uint8_t read_register(uint8_t regstr);
	void SetUp_TX_ONLY();
  void morseSetSpeed(); // V2.0 from keyer-mini
	bool morseSendString(char mstring[]);
  void SendTestText(); // V2.0 added
  void KeyDownActions(); // V2.0 created
  void KeyUpActions(); // same
  void readSpeedPot(); // V2.0
  void freeSpeedPot(); 
  void checkPaddles();
  void PspaceDit();
  void PspaceDah();
  void ProcessChar();
  void LockOut();
 // void xsetup();
  void xloop();
  //void pins4LCD();
 // void pins4paddle();
void  shiftMorseLeft();

  // V1.20 from keyer mini:
	uint16_t ditlen, dahlen, halfspace; 
	uint8_t speed, speed_old, speed_old_1;
//	char SpeedString[] = "00 WPM"; // text equivalent of speed V2.1e - found not being used
	uint16_t wordspace;
  uint16_t CW_pitch = 600; // local peak for CEM-1201(50) V0.1 keep temorarily
  uint16_t CW_pitch_ST = 480;  // ST means for sidetone
	uint8_t keyDNstate = LOW; // state of key out pin when key is closed
	uint8_t keyUPstate = HIGH;
bool DitLatch = false; 
	bool DahLatch = false;
	bool SpLatchDit = false;
	bool SpLatchDah = false;
	bool DoTransmit = true; // "true" keys TX on keydown, "false" tone only
	bool DoTransmitOld = true; // save old while primary temporarily altered
  // DoTransmit is for internal logic. OnAir is for the user to prohibit TX
  bool OnAir = true; // V2.0 1/30/25 true if KEY_OUT will be keyed
  bool testText = false; // V2.0h if sendCWtestText is going on, inhibit TX keying
	uint8_t buildchar = 1; // where dots and dashes assemble into a char
	uint8_t newchar = 0; // buildchar moved here when done. 0 means not done
	uint8_t i = 0; // counter
	bool char_done = true; // True when char sent by user is finished
	uint8_t DeadMan = 255; // Count for stuck key shut down feature
	bool DoLockout = false; // flag that lockout condition exists
	bool CMNDMode = false; 
	bool SpeedPot = true; // Do use speed pot to set speed
	int staticPot; // save current reading when going to disable mode
  // Users note that #define PotExists in header file tells if pot is installed
	uint8_t CMNDChar = 'Z'; // CMNDChar is 1st letter of cmnd seq. Z = none
	uint8_t CMND_SEQ[4]; // Here cmnd seq chars after CMNDChar are accumulated
	uint8_t CMND_SEQ_ptr = 0;
	uint8_t XChar; // char translated from newchar to ascii
  bool LCDmode = true; // V2.0 tells if Dot/Dash contact pins config for LCD or paddle
  char msg1[] = "TEST MESSAGE";
  char msg2[] ="MESSAGE NUMBER TWO";
  char msg3[] = "MESSAGE THREE";
	
	// **USER** two variables below
	
	bool ModeA = false; // true for Mode A, false for Mode B
	bool SideTone = true; // true means DO generate sidetone
	bool SideToneOld = true; // place to store current while changing
  bool Did_Once = false; // for something to execute once only ******  TESTING *********

	uint8_t misc_count = 0;
	uint8_t loopCount = 0; // count passes through main loop, for speed pot check every 255
	// unsigned long timing_start; // already defined
  //uint8_t MorseLCDptr = 15; // V2.0e next place in MorseField[] for char
	// String SerialMsg = "";

// From keyer-mini  Explanations related to array are in mini file:

	byte backmorse[128] = {32, 32, 'E', 'T', 'I', 'A', 'N', 'M', 'S', 'U', // 9
	                     'R', 'W', 'D', 'K', 'G', 'O', 'H', 'V', 'F', ' ', // 19
					'L', 10, 'P', 'J', 'B', 'X', 'C', 'Y', 'Z', 'Q', ' ', // 30
					8, '5', '4', 32, '3', 32, 32, 32, '2', '*', 32, 32, 32, // 43
					32, 32, 32, '1', '6', '=', '/', 32, 32, '!', 32, 32, // 55
					'7', 32, 32, 0,'8', 32, '9', '0', 32, 32, 32, 32, 32, // 68
					32, 32, 32, 32, 32, 32, 32, '?', 32, 32, 32, 32, 32, 34, // 82
					32, 32, '.', 32, 32, 32, 32, '@', 32, 32, 32, 39, 27, // 95
					32, 32, 32, 32, 32, 32, 32, 32, 32, 32, ';', 32, 32, 32,
					32, 32, 32, 32, 32, ',', 32, 32, 32, ':', 32, 32, 32, 32,
					32, 32, 32, 32,};
	// for RIT/XIT operations:
 
	uint8_t VFO_Rx_Tx = 0; // 0 = synch, 1 = RX, 2 = TX NOTE: 2 is never used,
	                       // 0 means no RIT and 1 means RIT is on
	
	
	
	uint8_t CLK1_hi = 0x4F; 
	
	
		 
	uint32_t TuningStepSize[7] = {1, 10, 100, 1000, 10000, 100000, 1000000};
	signed char step_index = 2; // start with 100 Hz steps	V1.6 changed type
    uint8_t PBstate;
	
	// USERS:
	// Users may wish to change the starting frequency for various bands
	// by editing the values below
	
    uint32_t Bands[11] = {3550000, 5330500, 7025000, 10110000, 14025000,
                        18078000, 21025000, 24910000, 28025000, 50125000,
                        146520000};
						
    signed char hamBand_index = 2; // start on 40 meters V1.18 change to signed so can test for < 0
	uint8_t	Menu_item; // pointer to currently selected menu entry
  uint8_t Menu_itemA; // V1.14




	boolean bandInclude[11] = {true, false, true, true, true, true, true, true, 
	                           true, true, false};
// V1.15



	// Variables for formatting frequency for ASCII from DDS60_M328P.c

  // Note that when I reserve char xx[12], the compiler reserves me
  // twelve locations numbered 0 through 11 and note that 11 must be
  // null or zero if this is a string. So I have left 0 through 10
  // or 11 characters

    char a_freq[10]; // 8-char storage for frequency in ASCII. It will hold the 9-digit
                    // frequency in ASCII chars, but no terminator, so don't print it.
                    // V2.0i, I'm adding 1 to the length, going from 9 to 10

    char a_freq_fmt[12]; // storage for 11 chars formatted freq 114,098.113
   // NOTE: Above changed from 11 to 12 for Si570 version > 100 MHz	
   
   char MorseField[17]; // V2.0e, for printing paddle sent text to bottom line
   bool MorseToLCD = false; // V2.0f, do/don't send paddle text to LCD

	// bool bandChanged;


// Convert register values from locals to globals and make
// separate sets for RX and TX

	uint8_t multRX;
	uint32_t numRX;
	uint32_t dividerRX;

	uint8_t multTX;
	uint32_t numTX;
	uint32_t dividerTX;	
	
	uint8_t level = mA_8; // 9/13/2020
	
	// V1.7, dividerLast will hold last dividerRX or dividerTX sent
	// to the chip, so if it changes I'll do a reset.  
	
	uint32_t dividerLast = 0; 
	
	// initialize the lcd library with the numbers of the interface pins
	
#ifndef I2C_LCD // V1.12
	LiquidCrystal lcd(RS, E, DB4, DB5, DB6, DB7);	
#endif

// V2.1b 2/19/2025 in library statement below, changed hard-coded lcd address of 0x3F
// to constant label LCD_I2C_ADR which is defined in the header file as 0x3F

#ifdef I2C_LCD
 LiquidCrystal_I2C lcd(LCD_I2C_ADR, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE); // Addr, En, Rw, Rs, d4, d5, d6, d7, backlighpin, polarity
#endif

	void dispfreq1();



  boolean modeCW = true; // true means do frequency display correction for offset
  boolean sidetone = true; // generate sidetone when key down
  boolean key_down = false; // flag key down state (key down = true)
  boolean LSBMode = true; // true for LSB mode, false for USB mode
  boolean Verbose = false; // V1.14 messages on startup to be brief
  
 
  boolean X4 = false; // Output freq is 4 times indicated. Normally FALSE

 	bool PB1Short = false; // These booleans indicate the result of a check of
	bool PB2Short = false; // the three pushbuttons
	bool PB1Long = false;
	bool PB2Long = false;
	bool PB3Short = false;
	bool PB3Long = false;

	uint8_t PB1CNT = 0; // V2.1e count successive taps
	uint8_t PB2CNT = 0;
	uint8_t PB3CNT = 0;

	unsigned long PB1TIME = 0; // V2.1e, time between successive presses
	unsigned long PB2TIME = 0;
	unsigned long PB3TIME = 0;
	
// Note: SREG is the MCU's status register ...

void i2cWrite(uint8_t reg, uint8_t val) {   // write reg via i2c
    uint8_t oldSREG = SREG;
   sei();
  Wire.beginTransmission(SI5351BX_ADDR);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
  SREG = oldSREG;
}

void i2cWriten(uint8_t reg, uint8_t *vals, uint8_t vcnt) {  // write array
   uint8_t oldSREG = SREG;
   sei();
  Wire.beginTransmission(SI5351BX_ADDR);
  Wire.write(reg);
  while (vcnt--) Wire.write(*vals++);
  Wire.endTransmission();
  SREG = oldSREG;
}

// V2.1f NRK attempt to read from Si5351a register, specifically CLK0_PHOFF
// I adapted this from SFRRanger_reader.ino in the examples of Wire.h
// I had to remove the check on Wire.available() to get any output, but 
// then I got the expected number.


uint8_t read_register(uint8_t regstr)
{
	uint8_t reading;
	uint8_t oldSREG = SREG;
  sei();
	Wire.beginTransmission(SI5351BX_ADDR); // transmit to device 0x60
  Wire.write(byte(regstr));      // sets register pointer to PHOFF register
  Wire.endTransmission();      // stop transmitting
	SREG = oldSREG;

  Wire.requestFrom(SI5351BX_ADDR, 1);    // request 1 bytes from slave device

	  // step 5: receive value from device
  if (1 ) { // if two bytes were received (removed) 2 <= Wire.available()
    reading = Wire.read();  // receive high byte (overwrites previous reading)
    //reading = reading << 8;    // shift high byte to be high 8 bits
    //reading |= Wire.read(); // receive low byte as lower 8 bits
		//Serial.print("Read from PHOFF: ");
    //Serial.println(reading);   // print the reading


		return reading;
  }

}


#ifdef RELAY_CONTROL

void Select_Relay(uint8_t HBindex)
{
 
 

// First turn all lines OFF

 for (uint8_t p=0; p<16; p++)
 {
  pcf.digitalWrite(p, HIGH);
 }

 // Next, turn ON specific line associated with the band index

 pcf.digitalWrite(Relay_lines[HBindex], LOW);

}

#endif



// ************************  S E T U P ************************************




void setup()
{
  Serial.begin(9600);

	uint8_t flagValue = 0;



 // Remind the user of compiler choices made, using the serial monitor



  Serial.println(F("\n   *** NEW START ***")); // I put this in when the program was auto rebooting

#ifdef SERIALINFO

  Serial.print(F("LCD interface type: "));


  #ifdef I2C_LCD
  Serial.println(F("I2C"));
  #else
  Serial.println(F("Parallel"));
  #endif
  
  Serial.print(F("Si5351a Crystal Frequency: "));
  Serial.println(XTAL_FREQ);

  Serial.print(F("IF OFFSET: "));
  Serial.println(IF_OFFSET);

  Serial.print(F("PCF8575 relay control enabled: "));
  #ifdef RELAY_CONTROL  
  Serial.println("YES");
  #endif
  #ifndef RELAY_CONTROL
  Serial.println("NO");
  #endif

  #ifdef HW8
  Serial.println(F("Compile for HW-8 mode: TRUE"));
  #endif

#endif

  MorseField[16] = '\n'; // V2.0f, null terminate string
  for(uint8_t ii = 0; ii < 16; ii++)
  {
    MorseField[ii] = ' ';
  }
	
 // pinMode(LEDPIN, OUTPUT); // NRK - on-board LED on Nano V2.0g
	pinMode(spkrpin, OUTPUT); // for speaker - beep to user
  pinMode(ENC_B, INPUT_PULLUP); // For rotary encoder
  pinMode(ENC_A, INPUT_PULLUP); // For rotary encoder
	pinMode(SW2, INPUT_PULLUP);
	pinMode(SW1, INPUT_PULLUP);	
	pinMode(Key_In, INPUT_PULLUP); // KEY DOWN (TX) status when LOW V2.0 renamed, was KeyOut
	pinMode(TX_Out, OUTPUT); // echoes Key_In after registers updated V1.1
	//pinMode(SB_Relay, OUTPUT); // HIGH in LSB mode, LOW in USB mode V2.0g, no more SB_Relay control
	digitalWrite(TX_Out, keyUPstate); // V1.13 - Ron Taylor reported his came up V2.0 changed to variable state
	                            // in Key Down mode. Make sure in Key UP
	pinMode(DotContact, INPUT_PULLUP); // moved these from xsetup, 2/4/2025 V2.0a
	pinMode(DashContact, INPUT_PULLUP);				


	attachInterrupt (digitalPinToInterrupt (ENC_A), RotaryISR, CHANGE);  // V1.8
	attachInterrupt (digitalPinToInterrupt (ENC_B), RotaryISR, CHANGE);  // V1.8
	


  	speed = WPM;
		// morseSetup calcs ditlen and dahlen
	   	morseSetSpeed(); 

// V2.0a, moved PinMode for dot/dash contacts to main program setup

    delay(50);
    Serial.print(F("Dot Contact state: "));
    Serial.println(digitalRead(DotContact));
    Serial.print(F("Dash Contact state: "));
    Serial.println(digitalRead(DashContact));

		analogReference(DEFAULT); // Use +5 volt Vcc for reference

     #ifdef PotExists
		
    if(SpeedPot)
    { 
    Serial.print(F("ADC counts = ")); // V0.1 ********* TESTING *************
    Serial.println(analogRead(POT_ADC)); // V0.1 ********* TESTING *************
		readSpeedPot(); 
		Serial.print("SPEED: ");
		Serial.println(speed, DEC);
    }
		speed_old = speed;
		
    #endif





//#ifndef I2C_LCD // V1.12- for parallel interface
 // pins4LCD(); // v2.0b 1ST TIME DON'T CHECK FLAG, JUST SET UP FOR LCD PARALLEL
	lcd.display(); // turn on display

//#endif

	// Set up the LCD's number of columns and rows 
 // V2.0y moved below to follow pins4LCD
	lcd.begin(16,2); 

#ifdef I2C_LCD//V1.12 - for I2C interface
	lcd.backlight();
#endif

  //#ifndef I2C_LCD // if parallel interface
  //if(!LCDmode) pins4LCD(); // make pins outputs for LCD
  
  //#endif

	lcd.setCursor(0,LCDBottomLine);
	lcd.print(" Si5351a Quad"); // display version number on the LCD
	lcd.setCursor(0,LCDTopLine);
	lcd.print(" WA5BDU V2.2a"); // V1.2 9/4/2020 V1.5 1/11/2021 v1.6 3/18/2022
	                           // V1.7 4/28/2022 V1.8 1/12/2023 V1.9 1/24/2023
                             // V1.10 4/16/2023
							 // V1.11 8/4/2023
							 // V1.12 8/8/2023
							 // V1.13 9/19/2023
               // V1.14 12/9/2023
               // V1.15 11/8/2024
               // V1.16 12/4/2024
               // V1.17 12/10/2024
               // V1.18 12/12/2024
               // V1.19 1/8/2025
               // V1.20x 1/23/2025, x means interim revision
               // V1.20 1/24/2025 finished V1.20
               // V2.0 1/20/2025 adding keyer
               // V2.1 2/14/2025 adding HW8 version
							 // V2.2 4/11/2025 fixed some bugs and did fine-tuning 
                

	delay(2500);
	lcd.clear();
	

  if(digitalRead(SW1) && !digitalRead(SW2)) // PB2 alone
  {
    SkipEEread = true; // true means don't read EEPROM data
    Serial.println(F("\nDon't read EEPROM data"));
    lcd.print(F("Skip EEPROM"));
    //PB2Long = false;
    while(!digitalRead(SW1) || !digitalRead(SW2)); // Stay until both released
  }


	
// Below, read in EEPROM values from last session, if any have been saved
// Otherwise, use default values:	

	EE_flagValue = EEPROM.read(EE_flag);
 
    if (EE_flagValue == 170 && !SkipEEread)
    {
    if(!FastStart) lcd.print(F("Reading EEPROM")); // V1.19
    EEPROM.get(EE_FoutTX, FoutTX);
    hamBand_index = EEPROM.read(EE_hamBand_index);
    step_index = EEPROM.read(EE_step_index);
		modeCW = EEPROM.read(EE_modeCW); // V1.3 default TRUE
		LSBMode = EEPROM.read(EE_LSBMode); // V1.3 default TRUE
		EEPROM.get(EE_CW_pitch, CW_pitch); // v1.4b
		EEPROM.get(EE_sidetone, sidetone);		
    EEPROM.get(EE_QUAD, QUAD); // V1.14 here down
    EEPROM.get(EE_TX_CLK2, TX_CLK2);
    EEPROM.get(EE_CLK0_1_OFF_KYDN, CLK0_1_OFF_KYDN);
    EEPROM.get(EE_TRIM, TRIM); 
    EEPROM.get(EE_DIV_FACTOR, DIV_FACTOR);
    EEPROM.get(EE_X4, X4);
    EEPROM.get(EE_TX_ONLY, TX_ONLY); // V1.19
    EEPROM.get(EE_keyDNstate, keyDNstate); // V2.0 additions (3)
    //EEPROM.get(EE_SpeedPot, SpeedPot); // V2.0z I don't want this to carry over
    EEPROM.get(EE_ModeA, ModeA);
    DidReadEE = true; // V2.0 - later on I need to know this
    }

    //Serial.print("FoutTX from EEPROM: ");
    //Serial.println(FoutTX);

    // Below - bandaid fix for finding boolean values of 255 and 254 for TX_ONLY
    // trying to print them to the Serial monitor.

    // V2.1d, change test from > 1 to > 0 for true/false distinction

    if((uint8_t) TX_ONLY > 0) TX_ONLY = true; // V1.20x here and below - V2.1d, no > 0
    else {TX_ONLY = false;}

    keyUPstate = !keyDNstate;

// After reading EEPROM data, report more status info via Serial Monitor:

  //Serial.println("After loading EEPROM values ...");

  #if 1

  Serial.print(F("Sidetone: "));
  if(sidetone) 
  {Serial.println(F("ENABLED"));}
  else Serial.println(F("DISABLED"));

  Serial.print(F("CW sidetone pitch: "));
  Serial.println(CW_pitch);
  Serial.print(F("TX out on CLK2? "));
  if(TX_CLK2){Serial.println("YES");}
  else {Serial.println("NO");}

	Serial.print(F("Rotary encoder division: "));
  Serial.println(DIV_FACTOR);

  Serial.print(F("\"TX ONLY\" MODE? "));
  if(TX_ONLY){Serial.println("YES");}
  else {Serial.println("NO");}

  Serial.print("Quadrature mode? ");
  if(QUAD){Serial.println("YES");}
  else {Serial.println("NO");} 

  Serial.print(F("TX OUT line Key-Down state? "));
  if(keyDNstate) Serial.println("HIGH"); // For non-zero it's HIGH, for zero: LOW
  else Serial.println("LOW");

   Serial.print(F("Output freq X4? "));
  if(X4){Serial.println("YES");}
  else {Serial.println("NO");} 

  Serial.print(F("Keyer Mode: "));
  if(ModeA) {Serial.println("A\n\n");}
  else Serial.println("B\n\n");

	Serial.print(F("Trim non-sig digits: "));
	if(TRIM) Serial.print(F("YES"));
	else Serial.println(F("NO"));

	Serial.print(F("LSB/USB: "));
	if(LSBMode) Serial.println(F("LSB"));
	else Serial.println(F("USB"));

	Serial.print(F("Phone/CW: "));
	if(modeCW) Serial.println(F("CW"));
	else Serial.println(F("Phone"));

	Serial.print(F("Frequency:  "));
	Serial.println(FoutTX);

#endif

    delay(1000); // V1.19
    lcd.clear();    
    
   // V2.1 - below I'm fixing a problem where the stored frequency was not being used
   // but instead the band table was used.  Assuming EE had been read.

		if(!DidReadEE) FoutTX = Bands[hamBand_index]; // Only use if EE was not read V2.1
                                                  // otherwise FoutTX came from EEPROM

  // Next, if in HW8 mode, I want to use the stored FoutTX if it was ready from EEPROM
  // so I'll use it to back-calculate HW8DialTX. Otherwise go with hard-coded value
  // But first I make sure it fits the HW8 0 to 500 kHz expanded range

    #ifdef HW8
 
    if(DidReadEE)
    {
      if(FoutTX >= 8395000 && FoutTX <= 8895000)
      {
      HW8DialTX = HW8FQtoDial(FoutTX);
      HW8DialRX = HW8DialTX;
      }
      else FoutTX = HW8DialtoFQ(HW8DialTX);   // if hard coded Dial was used 
    }

    #endif

   
  /*
    V2.1b - I need one more correction. Suppose I was in the HW-8 mode and saved
    FoutTX in the ~8.4 MHz range. Later, I'm NOT in the HW-8 mode and I read that
    frequency from EEPROM. In that case I need to look up a starting frequency
    based on the hamBand_index.
  */ 


  #ifndef HW8
   if(FoutTX >= 8395000 && FoutTX <= 8895000)
   {
    FoutTX = Bands[hamBand_index];
   }	
   #endif

	FoutRX = FoutTX; // initialize
	
//if(GetButtons)

 if(!digitalRead(SW1) && !digitalRead(SW2)) // IF a SW1 & 2 held closed during S/U, do Verbose
 //if(PB3Long)
	{
		Verbose = true; // V1.14
    FastStart = false; // no fast start when Verbose is desired
    lcd.print( "VERBOSE");
    delay(200);
    while(!digitalRead(SW1) || !digitalRead(SW2)); // Stay until both released
   // PB3Long = false;
	} 
  /* 2.0c move to before EEPROM read

  else if(digitalRead(SW1) && !digitalRead(SW2)) // PB2 alone
  {
    SkipEEread = true; // true means don't read EEPROM data
    Serial.println("Don't read EEPROM data");
    //PB2Long = false;
    while(!digitalRead(SW1) || !digitalRead(SW2)); // Stay until both released
  }
*/
	instructions(); // give user brief options V1.4 moved to after EEPROM read
                 // continues to verbose info if Verbose is true

// V1.14 change - now button closed on S/U sets VERBOSE mode true


	
	//lcd.clear(); // added 9/25/2020 to fix display. Not tested yet.
	

		delay(1000);
		lcd.clear();
	
	// End V1.2 additions for X1/X4 selection on startup
	
	// i2cInit();
	Wire.begin(); // NRKw
	
	Wire.setClock(400000); // Initial value of 100,000 didn't help but 
	                       // going to 400,000 cut my time from 8.19 to
						   // to 2.8 ms per frequency upload.
	
	calc_TX(FoutTX); // V1.1, need both sets ready
	calc_RX(FoutRX);
	send_regs_RX();
	// reset_PLL_A(); // V1.7, send_regs_RX() now does the reset if needed

// V1.19 some actions for when TX_ONLY is true

  if(TX_ONLY)
  {
    SetUp_TX_ONLY(); // V1.21
      key_down = false; // start with assumption
      if(!digitalRead(Key_In)) // then read the input
    {
      // V2.0 - let new routine handle this stuff:
      /*
      key_down = true;
      clock_0_ON();
      if(QUAD) clock_1_ON();
      */
      KeyDownActions(); // This will set the key_down flag
    }
    if(!key_down)
    {
        // V2.0 - let new routine handle this stuff:
      /*    
      clock_0_OFF();
      if(QUAD) clock_1_OFF();
      */
      KeyUpActions();
    }
  }
  // V1.19 assume key is UP so don't turn on CLK0/1 if in TX_ONLY mode:
  if(!TX_ONLY)
  {
	clock_0_ON(); // same	
	if (QUAD) clock_1_ON(); // same V1.12, turn on only if QUAD is true
  }

/*
	Serial.println("Reached line 914");
	while(1)
	{
	while(!GetButtons());
	LSBMode = !LSBMode; // Calling code does the swap
	doLSB_USB();
	Serial.print("SB Mode: ");
	if(LSBMode) Serial.println("LOWER");
	else Serial.println("UPPER");
	delay(250);
	}
*/

/*
  #ifdef HW8
	Print_freq(HW8DialTX, LCDTopLine);	
  #else 
  Print_freq(FoutTX, LCDTopLine);
  Serial.print("FoutTX line 843: ");
  Serial.println(FoutTX);
	#endif
*/

	encoder_pins = (PINC & ENC_AB_MASK)>>2;  // In V1.8, pins are sifted to <1-0>
	// 6/8/2018 avoid spurious step on start-up

  // V1.10, normal startup is in LSB mode. If USB mode was loaded from EEPROM,
  // need to call the routine that does the LSB/USB actions:

  if(!LSBMode)
  {
    doLSB_USB();
  }


  if(!TX_CLK2 || TX_ONLY) // V1.14 make sure CLK2 off V1.19 same for TX_ONLY mode
  {
    clock_2_OFF();
  }
  



#ifdef RELAY_CONTROL

 // V2.1b 2/19/2025 changing address below from 0x20 to constant PCF8575_I2C_ADR
 // which will be defined as 0x20 in the header file, for easier user access

  pcf.begin(PCF8575_I2C_ADR, &Wire);
  Select_Relay(hamBand_index);
  #endif

//Serial.print(" End of SETUP, TX_ONLY ="); // XXXOUT
//Serial.println(TX_ONLY);
if(CW_pitch > 200) CW_pitch_ST = CW_pitch; // V2.0, would like pitches to be the same

// SendTestText(); // V2.0 *********  TESTING ****************

// V2.0 moving CW "TU" from keyer section to here, so S/U will be complete when heard

    DoTransmitOld = DoTransmit;
		DoTransmit = false;
		morseSendString("TU");
		DoTransmit = DoTransmitOld;


loopCounter = 1; // ****** TESTING ********
timing_start = millis();

// Going back to paddle mode at end of setup()

//#ifndef I2C_LCD // if parallel mode
 // pins4paddle();
//#endif
#ifdef HW8
FoutTX = HW8DialtoFQ(HW8DialTX); // ***************  THESE TWO LINES FOR TESTING *******
FoutRX = FoutTX;
#endif

//////if(FoutTX == 0 || FoutRX == 0) Serial.println("Fout zero at end of setup.");
//Serial.println(FoutTX);
//Serial.println(FoutRX);

UpdateScreen(); // V2.1 I found screen not complete after startup.

// TESTING ********** DOES CLOCK 2 OFF WORK?

/*
while(!GetButtons())
{
	clock_2_ON();
	clock_0_ON();
	delay(4000);
	clock_2_OFF();
	clock_0_OFF();
	delay(4000);

}
*/

} //  ******************* END OF S E T U P **********************************



// ************************  M A I N   L O O P ****************************


void loop()
{

if(LCDtrash) // If display needs fixing ... V2.1e
{
	if(millis()-LCDFixTime > 2000)
	{
		UpdateScreen();
		LCDtrash = false;
	}

}



xloop(); // V2.0 loop from keyer



// Below, I see if the encoder ISR has accumulated any steps that
// need to be taken. First steps up, then steps down ...

	if(queue != 0)
	{		
	if(queue > 0) 
	  {
		  if((queue < 10) || (step_index == 6)) // || is logical OR. Increase
												// step if not already at max
		  {
			step_up(); 
			queue--;
		  }
		  else
		  {			  			  
		  step_index++;
		  step_up();
		  step_index--;
		  queue = queue - 10;	  
		  }			 

	 }
	 
		if(queue < 0) 
		{
			if((queue > -10) || (step_index == 6))
			{
			  step_down();
			  queue++;
			}
			  else
			  {			  			  
			  step_index++;
			  step_down();
			  step_index--;
			  queue = queue + 10;	  
			  }	
		}
		
		// V1.1 here we are sure in key up mode, so send out the RX registers
		// Note, we are still the section where queue was != 0
		// This is after stepping in response to encoder movement:
			
		   send_regs_RX();
       #ifndef HW8					   
		   Print_freq(FoutRX, LCDTopLine);
       #else
		   Print_freq(HW8DialRX, LCDTopLine);
       #endif
	}

	
// Now check for key down condition
// In V1.16, I make sure that the "key up" actions only occur the one time
// that the keyed line has returned to open. Note that this only occurs
// for the manual key input. Paddle generated key down periods must 
// control their own timing.


  if(!digitalRead(Key_In))
  {
    KeyDownActions(); // V2.0 moving these actions to a function
    
			while(!digitalRead(Key_In)); // stay here until key returns to up
		
// Below actions take place after key goes back UP
			KeyUpActions(); 

  } 


        if (GetButtons())
        {
 
        if(!KeyerActive) // V2.0 when keyer is active, buttons have different functions
        {
        if(PB3Long) // main menu
			{
				menu(); 
				//lcd.clear();
				UpdateScreen();// restore normal display
				PB3Long = false;
				//Print_freq(Fbase, 0); 
               // lcd.setCursor(14, 0);
               // LCDPrintMult();
			}
            else if (PB2Short)
            {
				//SingleBeep();  // **** TESTING ****
				// v1.4a - what if we're in some odd step not 10 or 100 or 1000?
				if(step_index < 1) step_index = 1;
				if(step_index > 3) step_index = 3; // V1.19 upper limit was 2
                //step_index = (step_index==2)?1:2; // alternate 1 & 2
                step_index++;
                if(step_index == 4) step_index = 1; // V1.19 roll from 3 to 0
                ShowStepFast(); 
				PB2Short = false;
			if(TRIM && (step_index == 2))
			{
			FoutRX = FoutRX - FoutRX % TuningStepSize[step_index];
			FoutTX = FoutTX - FoutTX % TuningStepSize[step_index];
			}
            }

			else if (PB1Long && !TX_ONLY) // V1.19 skip if TX_ONLY mode
			{
				if(VFO_Rx_Tx == 1)
				{
					TXRXSwap();
				}
				PB1Long = false;
			}

      // ************** PB1 Short ***************************

						else if(PB1Short)
			{
        if(TX_ONLY) // V1.19 alternate function for PB1Short when in TX_ONLY
        {
          spot_it();
          goto _DidClear;
        }
				if(VFO_Rx_Tx == 0) RIT_ON();	
				
				// Below I'm adding a check for a "double tap" on PB1 when RIT
				// is ON, which will result in clearing the offset, not 
				// turning RIT off
				
				else 
				{
				timing_start = millis();
				while((millis() - timing_start) < 333)//allow 1/3 s. for tap 2
					{
					if(digitalRead(SW1) == 0)
						{
						SingleBeep();
						FoutRX = FoutTX; // eliminate offset
						#ifdef HW8
						HW8DialRX = HW8FQtoDial(FoutRX); // V2.2a
						#endif
						Go_RIT();
       #ifndef HW8					   
		   Print_freq(FoutTX, LCDTopLine);
       #else
		   Print_freq(HW8DialTX, LCDTopLine);
       #endif
						while(digitalRead(SW1) == 0);// stay until PB released
						goto _DidClear;
						}
					}
				RIT_OFF();

				_DidClear:;
				}
           // PBstate = 0;
            	PB1Short = false;
			}
			
			
			else if(PB2Long) // v.14a
			{
				doBands();
				PB2Long = false;
			}
			else if(PB3Short)
			{
				LSBMode = !LSBMode;
				doLSB_USB();
				PB3Short = false;
			}
        
    }	// Below, button usage when keyer is active V2.0 Here, end of !KeyerActive
    else {
   
    
    if(PB1Short)
      {
      morseSendString(msg1);
      timing_keyer = millis(); // V2.0a - give extra time after message sends
															// refers to the 3 seconds to stay in KYR mode
			//SPRINT_inhibits(); // ************ TESTING ***********************
      PB1Short = false;
      }     

      else if(PB2Short)
      {
      morseSendString(msg2);
      timing_keyer = millis(); // V2.0a - give extra time after message sends
      PB2Short = false;
      }
      
      else if(PB3Short)
       {
      morseSendString(msg3);
      timing_keyer = millis(); // V2.0a - give extra time after message sends
      PB3Short = false;
      }  
      
        }   // End of 'else' for KeyerActive is true
    } // End of GetButtons();
/*
  #ifdef HW8
  if(!Did_Once)
  {
    DoPrintHW8();
    Did_Once = true;
  }
  #endif
*/

} //  *********** end of loop() function  **************************** ENDOFLOOP

/*
  #ifdef HW8
  void DoPrintHW8()
  {
    Serial.print("HW8DialRX: ");
    Serial.println(HW8DialRX);
    Serial.print("FoutRX: ");
    Serial.println(FoutRX);
    Serial.print("FoutTX: ");
    Serial.println(FoutTX);   
    Serial.print("From HW8DialtoFQ: ");
    Serial.println(HW8DialtoFQ(HW8DialTX));
   // Serial.print("Step Size: ");
    //Serial.println(TuningStepSize[step_index]);
    //Serial.print("Step Index: ");
    //Serial.println(step_index); 
    Did_Once = true;   
  }
  #endif

*/

	void menu()
	{
//#ifndef I2C_LCD // if parallel mode
  ////if(!LCDmode) pins4LCD();
//#endif

        lcd.clear();
        lcd.print("MENU");
        lcd.setCursor(0,1);
        lcd.print("ROTATE");
        any_action(1000); 
        lcd.clear(); 
		boolean chosen = false; // user selection not yet made
		Menu_item = 0;
		showChoice();

		while(!chosen)

		{   
            r_dir = low_rez();
            while (r_dir)
            {
    			if (r_dir == 1)
    			{
    			Menu_item++; // currently, 0, 1, 2, 3, 4, 5, 6 are valid
    			if (Menu_item > 6) Menu_item = 0; // V1.14 added another item
    			}
    			else if(r_dir == 2)
    			{
    				if(Menu_item == 0) Menu_item = 6; // V1.14
    				Menu_item--;
    			}
    
    			
    			showChoice();
                r_dir = 0;
			
            }

		   if(GetButtons())
		   {
			   if (PB3Short) 
			   {
				   chosen = true;
				   PB3Short = false;
			   }
		   }
			
		} // Below here 'chosen' is true
		

		if (Menu_item == 0) doSteps();
		else if (Menu_item == 1) doPitch();
		else if (Menu_item == 2) doCWPhone();

    else if (Menu_item == 3) doSave();
		else if (Menu_item == 4)
		{
			if(sidetone)
			{
				sidetone = false;
			}
			else sidetone = true;
			
			showSidetone();

		}
		else if (Menu_item == 5) doAdvanced(); // V1.14 
	
	}

void showChoice()
	{
		lcd.clear();
		lcd.setCursor(0,0);
		if (Menu_item == 0)
		{
			lcd.print("STEP SIZE");
		}
		else if (Menu_item == 1)
		{
			lcd.print("CW PITCH");
      lcd.setCursor(1,LCDBottomLine); // V1.14 - show value
      lcd.print(CW_pitch); 
		}

        else if (Menu_item == 2)
        {
            lcd.print("CW/PHONE");  
        }
        else if (Menu_item == 3)
        {
            lcd.print("SAVE STATE");
        }
        else if (Menu_item == 4)
        {
            lcd.print("SIDETONE");
            lcd.setCursor(1,LCDBottomLine); // V1.14 - show true or false
            if(sidetone)
            {
            lcd.print("true");
            }
            else
            {
            lcd.print("false");
            }            
        }	
        else if (Menu_item == 5) // V1.14 added ADVANCED
        {
            lcd.print("ADVANCED");
        }				
		else if (Menu_item == 6)  // V1.14 was 5
        {
            lcd.print("EXIT");
        }				
	}

// V1.14 sub-menu to select among advanced menu choices
// It copies menu(). It monitors the encoder and moves to another choice
// with each pulse. It also monitors for a PB3 tap and selects the
// currently displayed choice when detected. Then the actions for the
// choice are taken.

  void doAdvanced()
  {
    lcd.clear();

		boolean chosenA = false; // user selection not yet made
		Menu_itemA = 0;
		showChoiceA(); // Put choice 0 to the LCD

		while(!chosenA)

// In this "while" loop, first see if the encoder has moved and if so, move
// pointer to the next sub-menu item. If encoder hasn't moved, continue
// to see if a PB3Short even has happened. If so, the current item is
// selected and the routine is called. If not, loop back to the top.

// V2.0 increase number of menu choices to 8 so I can add "send test text"
// to the list


		{   
        r_dir = low_rez(); // get UP, DOWN, or NONE from rotary encoder
        while (r_dir) // If zero, no encoder motion so skip this ...
        {
      if (r_dir == 1)
      {
      Menu_itemA++; //  0, 1, 2, 3, 4, 5, 6, 7, 8 are valid
      if (Menu_itemA > 7) Menu_itemA = 0; // V1.19 was > 6
      }
      else if(r_dir == 2)
      {
        if(Menu_itemA == 0) Menu_itemA = 8; // V2.0,  was 7
        else Menu_itemA--;
    	}
    
    		// We know encoder motion occurred, so update LCD with current choice
    			showChoiceA();
          r_dir = 0;
			
      }
// Now see if the user is selecting the currently displayed option

		   if(GetButtons())
		   {
			   if (PB3Short) 
			   {
				   chosenA = true;
				   PB3Short = false;
			   }
		   }
			
		} // Below here 'chosenA' is true
		

		if (Menu_itemA == 0)
    {
      QUAD = !QUAD;
      showChoiceA();
			// V2.1e - Found that Quadrature? choice not being enforced ...
			if(QUAD)
			{ 
				if(LSBMode) CLK1_hi = 0x4F; // V2.1e
				else CLK1_hi = 0x5F;

				//clock_1_ON();
				CLK0_1_on_quad(); // V2.1f this and next sort of bandaid to restore quadrature
				reset_PLL_A();   // after not-quadrature ends
				//doLSB_USB(); // V2.1e this is a bandaid. I found phase messed up after
				//delay(100); // turning Quadrature off and back on
				//doLSB_USB();
			}
			else clock_1_OFF();

      delay(1000);

    }
		else if (Menu_itemA == 1)
        {
      TX_CLK2 = !TX_CLK2;
      if(TX_CLK2) 
      {
        TX_ONLY = false; // V1.20 - can't have both on
        if(CW_pitch == 0) CW_pitch = CW_pitch_ST; // V1.20 restore if suspected off from TX_ONLY
      }
			else CLK0_1_OFF_KYDN = false; // V2.1f, make false when TX_CLK2 is turned off
      showChoiceA();
      delay(1000);
    }
		else if (Menu_itemA == 2)
        {
      CLK0_1_OFF_KYDN = !CLK0_1_OFF_KYDN;
      if(!TX_CLK2) CLK0_1_OFF_KYDN = false; // V2.0 can be true only of TX_CLK2 is true
      showChoiceA();
      delay(1000);
    }

    else if (Menu_itemA == 3)
        {
      TRIM = !TRIM;
      showChoiceA();
      delay(1000);
    }
		else if (Menu_itemA == 4) doDivFac();
		else if (Menu_itemA == 5)
      {
      X4 = !X4;
      showChoiceA();
      delay(1000);
      }
      else if (Menu_itemA == 6) // V1.19 added this block
      {
        TX_ONLY = !TX_ONLY;
        SetUp_TX_ONLY(); // V1.21
        showChoiceA();
       // Serial.print(" Choice 6, 1507, TX_ONLY = "); // XXXOUT
       // Serial.println(TX_ONLY);

        delay(1000);
      }
      else if (Menu_itemA == 7)
      {
        // First I want to make sure VFO is in one of the two modes that keys
        // the output, either TX_CLK2 or TX_ONLY. First see if TX_CLK2 is
        // current mode. If not, check TX_ONLY mode and if it's also not,
        // then make it the current mode

      if(!TX_CLK2)
      {
				TX_ONLY_OLD = TX_ONLY; // V2.1e save to restore mode if not TX_ONLY
        if(!TX_ONLY)
        {
          TX_ONLY = true;
          SetUp_TX_ONLY();
        }
      }   
			SideToneOld = sidetone;
          testText = true; // V2.0h, use to inhibit TX key closure
          sidetone = false; // V2.0h, no sidetone please
          SendTestText();
	// Now we're finished with the send text function
					TX_ONLY = TX_ONLY_OLD; // V2
          UpdateScreen(); // fix LCD screen
					if(!TX_ONLY) SetUp_TX_ONLY(); // Note: SetUp also has some stuff for exiting mode V2.1e
          sidetone = SideToneOld;
          testText = false;
					CLK0_1_on_quad(); // V2.1f this and next sort of bandaid to restore quadrature
				  reset_PLL_A();   // after sendTestText ends
      }

	} // end of DoAdvanced() function	

void showChoiceA()
	{
//#ifndef I2C_LCD // if parallel mode
  //if(!LCDmode) pins4LCD();
//#endif

   // Serial.print(" showChoiceA entry, TX_ONLY ="); // XXXOUT
   // Serial.println(TX_ONLY);  

		lcd.clear();
		lcd.setCursor(0,0);
		if (Menu_itemA == 0)
		{
			lcd.print("Quadrature?");
      lcd.setCursor(1,LCDBottomLine);
      if (QUAD)
      {
        lcd.print(" true ");
      }
      else
      {
        lcd.print(" false");
      }
		}
		else if (Menu_itemA == 1)
		{
			lcd.print("TX on CLK2?");
	    lcd.setCursor(1,LCDBottomLine);
     if (TX_CLK2)
      {
        lcd.print(" true ");
      }
      else
      {
        lcd.print(" false");
      } 


		}

        else if (Menu_itemA == 2)
        {
          lcd.print("CLK0/1 KEYED?");
        	  lcd.setCursor(1,LCDBottomLine);
      if (CLK0_1_OFF_KYDN)
        {
          lcd.print(" true ");
        }
        else
        {
          lcd.print(" false");
        }   
          }
        else if (Menu_itemA == 3)
        {
          lcd.print("TRIM SIG DIGITS?");
          lcd.setCursor(1,LCDBottomLine);
     if (TRIM)
      {
        lcd.print(" true ");
      }
      else
      {
        lcd.print(" false");
      } 
        }
        else if (Menu_itemA == 4)
        {
            lcd.print("ENC DIV FACTOR:");
            lcd.setCursor(1,LCDBottomLine);
            lcd.print(DIV_FACTOR);
        }	
        else if (Menu_itemA == 5) 
        {
        lcd.print("FREQ MULT x4?");
      	lcd.setCursor(1,LCDBottomLine);
        if (X4)
            {
              lcd.print(" true ");
            }
            else
            {
              lcd.print(" false");
            } 
          }

          // V1.19 add item 6
          else if(Menu_itemA == 6)
          {
            lcd.print("TX ONLY VFO?");
                	lcd.setCursor(1,LCDBottomLine);
        if (TX_ONLY)
            {
              lcd.print(" true ");
            }
            else
            {
              lcd.print(" false");
            } 
          }  
    else if (Menu_itemA == 7)
    {
      lcd.print("Send CW Test TXT");
    }				

		else if (Menu_itemA == 8)  // V1.14 was 5 V1.19 was 6
        {
            lcd.print("EXIT");
        }
			
	}

  
  
  void doDivFac()
  {
  

   while(!GetButtons())
		{   
        r_dir = low_rez(); // get UP, DOWN, or NONE from rotary encoder
        while (r_dir) // If zero, no encoder motion so skip this ...
        {
      if (r_dir == 1)
      {
      DIV_FACTOR++; 
      if (DIV_FACTOR == 0) DIV_FACTOR = 1; // if it rolls 255 to 0, bump to 1
      }
      else if(r_dir == 2)
      {
        DIV_FACTOR--;
        if(DIV_FACTOR == 0) DIV_FACTOR = 4; // let it roll under to 4 from 0
        // (I found that if I let it roll under to 255, encoder action gets
        //  slow that the thing appears to have stopped working.
    	}   
    		// We know encoder motion occurred, so update LCD with current choice
          lcd.setCursor(1,LCDBottomLine);
          lcd.print(DIV_FACTOR);
          lcd.print("  "); // in case # of digits decreased
          r_dir = 0; // reset rotary encoder count
			
      } 



  }

// Here, a PB has been depressed, so exit this function

  delay(1000);
 }


	void doSteps()
	{
		boolean chosen = false;
        showStepSize();

		while (!chosen)
		{

		r_dir = low_rez();	
		if (r_dir == 1)
			{
			    stepSize_up();
			}
			else if(r_dir == 2)
			{
				stepSize_down();
			}
			
		if (r_dir) showStepSize();
		r_dir = 0;
        PBstate = getSwitchWait();
		if (PBstate) 
		{
			chosen = true; // V1.1 - any non-zero value accepted
			// V1.5, below, zero out the less significant digits of frequency
			if(TRIM)
			{
			FoutRX = FoutRX - FoutRX % TuningStepSize[step_index];
			FoutTX = FoutTX - FoutTX % TuningStepSize[step_index];
			}
		}
	
	    }
	}


	void showStepSize()
	{
	//#ifndef I2C_LCD // if parallel mode
 // if(!LCDmode) pins4LCD();
//#endif

		clearLine1();
		lcd.print(TuningStepSize[step_index]);	
	}

	void doBands()
	{
	#ifdef HW8
  bad_sound();
  goto quitdoBands;
  #endif

		boolean chosen = false; 
		char index_now = hamBand_index; // so can see if we change bands
		showBand();
		
		while (!chosen)
		{

		r_dir = low_rez();	
		if (r_dir == 1)
			{
				// V1.18 - keep incrementing until we hit a band that's not locked out.
				do 
				{
				hamBand_index++; //V1.18
				if(hamBand_index > 10) hamBand_index = 0; // V1.18 roll over to zero
				} while(!bandInclude[hamBand_index]); // V1.18 if band is false, increment again
			}
			else if(r_dir == 2)
			{
				// V1.18 - keep decrementing until we hit a band that's not locked out.
				do 
				{
				hamBand_index--; // V1.18
				if(hamBand_index < 0) hamBand_index = 10; // V1.18 roll under to top
				} while(!bandInclude[hamBand_index]); // V1.18 if band is false, decrement again
			}
			
    		if (r_dir) showBand();
    		r_dir = 0;
            PBstate = getSwitchWait();
    		if (PBstate) chosen = true; // V1.1, accept any PB
		}

        // Now, if we changed bands, put the old band current freq into the
        // register for that band, so when I return I'll be on the same freq
        // as when I left

        if(hamBand_index != index_now)
        {
            Bands[index_now] = FoutTX; // current freq to band register
        }
        

		FoutTX = Bands[hamBand_index]; 

		FoutRX = FoutTX;
		calc_RX(FoutRX); // V1.1, 1/11/2020 change bands must do both RX & TX
		send_regs_RX();
		//reset_PLL_A(); // V1.7, send_regs_RX or _TX now does the resets
		calc_TX(FoutTX); // V1.11 - try to fix wrong transmit freq after band change
		RITSave = 0; // V1.7 prevent going to old offset after changing bands
		
		lcd.clear();

       #ifndef HW8					   
		   Print_freq(FoutTX, LCDTopLine);
       #else
		   Print_freq(HW8DialTX, LCDTopLine);
       #endif

// V1.15, if PCF8575 is in use, set the output lines to match the band
// *bk4
#ifdef RELAY_CONTROL
  Select_Relay(hamBand_index);
  #endif 
  quitdoBands:;
  } // end of doBands 

	

		void showBand()
		{
			clearLine1();
			lcd.print(Bands[hamBand_index]);
		}

// ****** doSave saves the current state of the VFO, including
// ****** frequency, step size, mode and band into EEPROM for next startup
// ****** It also sets a flag in EEPROM, so the program on startup can know
// ****** that saved values are available.
// ****** note .update writes only if new data is different from existing

    void doSave()
    {

        EEPROM.put(EE_FoutTX, FoutRX); // V2.1 I now save FoutRX instead of FoutTX
        EEPROM.put(EE_hamBand_index, hamBand_index);
        EEPROM.put(EE_step_index, step_index);
    //    EEPROM.put(EE_mode, quad);
        EEPROM.update(EE_flag, 170); // save specific # 0b10101010 to show EPROM written
		EEPROM.put(EE_LSBMode, LSBMode); 
		EEPROM.put(EE_modeCW, modeCW); 
    if(!TX_ONLY) // V1.20 - I don't want to save that temporary 0 Hz pitch
		EEPROM.put(EE_CW_pitch, CW_pitch); 
		EEPROM.put(EE_sidetone, sidetone); // boolean value
    EEPROM.put(EE_QUAD, QUAD); // These six added V1.14
    EEPROM.put(EE_TX_CLK2, TX_CLK2);
    EEPROM.put(EE_CLK0_1_OFF_KYDN, CLK0_1_OFF_KYDN);
    EEPROM.put(EE_TRIM, TRIM);
    EEPROM.put(EE_DIV_FACTOR, DIV_FACTOR);
		EEPROM.put(EE_X4, X4);
    EEPROM.put(EE_TX_ONLY, TX_ONLY); // V1.19
    EEPROM.put(EE_keyDNstate, keyDNstate); // V2.0 additions (3)
    EEPROM.put(EE_SpeedPot, SpeedPot);
    EEPROM.put(EE_ModeA, ModeA);
		

     // To tell the user that EEPROM has been written, I'll do two
		// beeps, at 400 Hz and then 500 Hz.
		tone(spkrpin,400, 250);
		delay(250);
		tone(spkrpin,500, 250);
	
    }

// **********  ANY ACTION -- RETURN IF SWITCH OR ENCODER CHANGES *************
// ********** OR SPECIFIED # OF MILLISECONDS PASSES **************************
// Timing is in 20 ms per check. 
// Status of encoder or switch not changed by this routine

// This routine provides a delay to be used, for example, when printing
// something temporarily to the display. The calling program section wants to 
// clear the temporary message either as soon as the user takes any action or
// as a minimum, after a certain time delay long enough to allow reading the
// message

    void any_action(uint16_t action_time)
    {
        boolean happening = false;
        uint16_t holdtime = 0;
        uint16_t A_time = action_time;

        while(!happening)
        {
            delay(20);
            holdtime += 20;
            
            encoder_now = (PINC & ENC_AB_MASK)>>2; // 2/1/2016 V1.8 for >>2
            if (encoder_pins ^ encoder_now) happening = true; 
            if (!digitalRead(SW1) || !digitalRead(SW2)) happening = true;//V1.1
            if (holdtime > A_time) happening = true;
        
        }
        
    }    
        
// Print brief instructions to user on start-up

	void instructions()
	{
    if(!FastStart) // V2.0 do fast start, if Verbose, don't do fast start
    {
  //#ifndef I2C_LCD // if parallel mode
  //if(!LCDmode) pins4LCD();
  //#endif
    uint16_t ExtraTime = 0; // V1.19
    if(Verbose) ExtraTime = 1000;

    #ifdef HW8
    lcd.clear();
    lcd.print("HW-8 Mode");
    delay(1000);
    #endif

		lcd.clear();
		lcd.print("PB3 HOLD");
		lcd.setCursor(0, LCDBottomLine);
		lcd.print("FOR MENU");
		delay(1800 + ExtraTime);
		lcd.clear();
		lcd.print("PB2 TAP CHANGES");
		lcd.setCursor(0, LCDBottomLine);
		lcd.print("STEP SIZE");
		delay(2200 + ExtraTime);
		
		// V1.8, add startup display of encoder speed
		lcd.clear();
    }
    if(Verbose)
    {
		lcd.print("Encoder: 1:");
		lcd.print(DIV_FACTOR);
		delay(2500);
		lcd.clear();

		// V1.12 add messages for new config choices
		lcd.print("QUAD OUT: ");
		if(QUAD) lcd.print("Y");
		else	lcd.print("N");
    delay(2500);
		lcd.clear();
		lcd.print("CLOCK 2 = TX: ");
		if(TX_CLK2) lcd.print("Y");
		else lcd.print("N");
		delay(2500);
		lcd.clear();
		lcd.print("I.F. Offset:");
		lcd.setCursor(2, LCDBottomLine);
		lcd.print(IF_OFFSET);
		delay(2500);
		lcd.clear();
		if(TX_CLK2) // Message only if CLK2 output is used
		{
			lcd.print("RX OUT keyed? "); // Yes if CLK0&1 off during key down
			if(CLK0_1_OFF_KYDN)
			{
			lcd.print("Y");
			}
			else lcd.print("N");
      delay(2500);
      lcd.clear();
    }


    #ifdef RELAY_CONTROL // V1.17 add this status to verbose mode  
    lcd.print("PCF8575: YES");
    delay(2500); // V1.19
    lcd.clear(); 
    #endif
 
    if(TX_ONLY) // V1.19
    {
      lcd.print("TX ONLY MODE: Y");

    }
      delay(2500);
      lcd.clear();    

	if(X4)
	{
		lcd.print("F_OUT is X4");
	}
	else // V1.12, removed extra ; 
	{
		lcd.print("F_OUT is X1");
	}
  delay(2500);			
	
  lcd.clear(); // Adding V2.0 keyer info
  lcd.print("Keyer Speed:");
  lcd.print(speed);
  delay(2500);

  lcd.clear();
  lcd.print("Keyer mode: ");
  if(ModeA) lcd.print('A');
	else lcd.print('B');
  delay(2500);

  lcd.clear();
  lcd.print("Speed Pot? ");
  #ifdef PotExists
  lcd.print("YES"); // V2.0z
  #else
  lcd.print("NO");
  #endif
  delay(2500);

  }
  } // end Verbose section
	/*
  Long instructions, called from menu and scrolled with encoder
  */
/*
    void long_instr()
    {
      
    }
*/

// V1.21 adds procedure below to set some flags for TX_ONLY mode

    void SetUp_TX_ONLY()
    {
      if(TX_ONLY)
      {
      CLK0_1_OFF_KYDN = false; // not compatible with TX_ONLY
      TX_CLK2 = false; // This and TX_ONLY are mutually exclusive
      VFO_Rx_Tx = 0;// NOT RIT mode
      CW_pitch = 0; // Make sure no offset
      clock_2_OFF(); // V2.0

    // V2.1d will turn off outputs if key is up
    if(digitalRead(Key_In)) // key input high means key is UP (open)
    {
      clock_0_OFF();
      clock_1_OFF();

    }

      }
      else
      {
        CW_pitch = CW_pitch_ST; // This is the only thing I'll restore when leaving TX_ONLY
        //clock_0_ON(); // No, I need to turn the outputs back on too ...
        //if(QUAD) clock_1_ON();
				CLK0_1_on_quad(); // V2.1f this and next sort of bandaid to restore quadrature
				 reset_PLL_A();   // after sendTestText ends
      }
    }
//
// Set up specified PLL with mult, num and denom
// mult is 15..90
// num is 0..1,048,575 (0xFFFFF)
// denom is 0..1,048,575 (0xFFFFF)
//

void setupPLL(uint8_t pll, uint8_t mult, uint32_t num) // , uint32_t denom
{
	uint32_t P1; // PLL config register P1
	uint32_t P2; // PLL config register P2
	uint32_t P3; // PLL config register P3

	P1 = (uint32_t)(128 * ((float)num / (float)denom));
	P1 = (uint32_t)(128 * (uint32_t)(mult) + P1 - 512);
	P2 = (uint32_t)(128 * ((float)num / (float)denom));
	P2 = (uint32_t)(128 * num - denom * P2);
	P3 = denom;

// NRKw below change i2cSendRegister to i2cWrite
	i2cWrite(pll + 0, (P3 & 0x0000FF00) >> 8);
	i2cWrite(pll + 1, (P3 & 0x000000FF));
	i2cWrite(pll + 2, (P1 & 0x00030000) >> 16);
	i2cWrite(pll + 3, (P1 & 0x0000FF00) >> 8);
	i2cWrite(pll + 4, (P1 & 0x000000FF));
	i2cWrite(pll + 5, ((P3 & 0x000F0000) >> 12) | ((P2 & 
	0x000F0000) >> 16));
	i2cWrite(pll + 6, (P2 & 0x0000FF00) >> 8);
	i2cWrite(pll + 7, (P2 & 0x000000FF));
}

//
// Set up MultiSynth with integer divider and R divider
// R divider is the bit value which is OR'ed onto the appropriate 
// register, it is a #define in si5351a.h
//
void setupMultisynth(uint8_t synth, uint32_t divider, uint8_t rDiv)
{
	uint32_t P1; // Synth config register P1
	uint32_t P2; // Synth config register P2
	uint32_t P3; // Synth config register P3

	P1 = 128 * divider - 512;
	P2 = 0; // P2 = 0, P3 = 1 forces an integer value for the divider
	P3 = 1;

	i2cWrite(synth + 0, (P3 & 0x0000FF00) >> 8);
	i2cWrite(synth + 1, (P3 & 0x000000FF));
	i2cWrite(synth + 2, ((P1 & 0x00030000) >> 16) | rDiv);
	i2cWrite(synth + 3, (P1 & 0x0000FF00) >> 8);
	i2cWrite(synth + 4, (P1 & 0x000000FF));
	i2cWrite(synth + 5, ((P3 & 0x000F0000) >> 12) | ((P2 & 
	0x000F0000) >> 16));
	i2cWrite(synth + 6, (P2 & 0x0000FF00) >> 8);
	i2cWrite(synth + 7, (P2 & 0x000000FF));
}

//
// Switches off Si5351a output
// Example: si5351aOutputOff(SI_CLK0_CONTROL);
// will switch off output CLK0
//
void si5351aOutputOff(uint8_t clk)
{
i2cWrite(clk, 0x80); // Refer to SiLabs AN619 to see 
//bit values - 0x80 turns off the output stage
}



	void dispfreq1()
	{
  //#ifndef I2C_LCD // if parallel mode
  //if(!LCDmode) pins4LCD();
  //#endif
	  lcd.setCursor(0,LCDBottomLine);
	  lcd.print("         "); // clear 9 spaces
	  lcd.setCursor(0,LCDBottomLine);
	  lcd.print(FoutRX);
	}


	
// V1.1 adds routines to calculate the RX and TX register values
// borrowing from the existing Set_Freq() function

// *********************** calc_RX() *****************************************

	void calc_RX(uint32_t frequency)
	{
		// Note below, in CW mode the frequency input is first altered to
		// allow for the required shift needed for CW_pitch. That shift is
		// not made to the FoutRX value but instead is done at the time
		// that registers are calculated - meaning right here. V1.1
		
		// V1.12 After the pitch adjustment but before the X4 adjustment,
		// the I.F. frequency will be added to the receive frequency.
		
		if(modeCW)
		{
			if(LSBMode)
			{
				frequency += CW_pitch; 
			}
			else frequency -= CW_pitch; 
		}

		frequency += IF_OFFSET; // V1.12
		
		if(X4) frequency = frequency * 4; // V1.2 x4 option, after pitch adjustment
		
		uint32_t pllFreq;
		uint32_t xtalFreq = XTAL_FREQ;
		uint32_t l;
		float f;

		uint32_t try_vco = 900000000; // NRK

	// NRK - Can't get below 7.14 MHz using I/Q if vco is ~900 MHz and can't 
	//       get below 4.76 MHz if the lower limit of 600 MHz is used. 
	// 		 I'll try ignoring the lower limit and see if it works

		if(frequency < 7200000)
		{
			try_vco = frequency * 126;
			if(try_vco > 900000000) try_vco = 900000000;
		}

		// NRK - yes, that did work. I get output at 3.5 MHz and 10 MHz with
		// what looks like  90 degree phase shift on the scope.

	dividerRX = try_vco / frequency;// Calculate the division ratio. 
	// 900,000,000 is the maximum internal
	// PLL frequency: 900MHz
	if (dividerRX % 2) dividerRX--; // Ensure an even integer 
	//division ratio

	pllFreq = dividerRX * frequency; // Calculate the pllFrequency: 
	//the divider * desired output frequency

	multRX = pllFreq / xtalFreq; // Determine the multiplier to 
	//get to the required pllFrequency. It has three parts.
	l = pllFreq % xtalFreq; // x % y returns remainder of x/y 
	f = l; // Converts integer l to float f
	//multRX is an integer that must be in the range 15..90
	f *= 1048575; // num and denom are the fractional parts, the numerator and denominator
	f /= xtalFreq; // each is 20 bits (range 0..1048575)
	numRX = f; // the actual multiplier is mult + num / denom
	// denom = 1048575; // For simplicity we set the denominator to the maximum 1048575
	// which is 2^20 - 1
	// V1.1 will change denom to a defined constant		
		
	}




// *********************** calc_TX() *****************************************

	void calc_TX(uint32_t frequency)
	{
	uint32_t pllFreq;
		uint32_t xtalFreq = XTAL_FREQ;
		uint32_t l;
		float f;
		
		if(X4) frequency = frequency * 4; // V1.2

		uint32_t try_vco = 900000000; // NRK 900,000,000

	// NRK - Can't get below 7.14 MHz using I/Q if vco is ~900 MHz and can't 
	//       get below 4.76 MHz if the lower limit of 600 MHz is used. 
	// 		 I'll try ignoring the lower limit and see if it works

		if(frequency < 7200000)
		{
			try_vco = frequency * 126;
			if(try_vco > 900000000) try_vco = 900000000;
		}

		// NRK - yes, that did work. I get output at 3.5 MHz and 10 MHz with
		// what looks like  90 degree phase shift on the scope.

		// Calculate the division ratio. 900,000,000 is the maximum internal
	dividerTX = try_vco / frequency;
	// PLL frequency: 900MHz
	if (dividerTX % 2) dividerTX--; // Ensure an even integer 
	//division ratio

	pllFreq = dividerTX * frequency; // Calculate the pllFrequency: 
	//the divider * desired output frequency

	multTX = pllFreq / xtalFreq; // Determine the multiplier to 
	//get to the required pllFrequency
	l = pllFreq % xtalFreq; // It has three parts:
	f = l; // mult is an integer that must be in the range 15..90
	f *= 1048575; // num and denom are the fractional parts, the numerator and denominator
	f /= xtalFreq; // each is 20 bits (range 0..1048575)
	numTX = f; // the actual multiplier is mult + num / denom
	// denom = 1048575; // For simplicity we set the denominator to the maximum 1048575
	// V1.1 will change denom to a defined constant		
		
	}


// V2.0i convert synth frequency to HW8 dial frequency to be displayed

#ifdef HW8
// Take VFO output frequency and convert to HW8 dial reading:

uint32_t HW8FQtoDial(uint32_t actualFQ)
{
  uint32_t Returndial;
 Returndial = 8895000 - actualFQ;
  return Returndial;
}

// Take HW8 dial reading and convert to required frequency out

uint32_t HW8DialtoFQ(uint32_t dialrdg)
{
 // uint32_t FQtoReturn;
  return HW8FoutMax - dialrdg;
  //return FQtoReturn;
}

#endif



// ************  SEND RX REGISTERS TO Si5351a ********************************

// NOTE: This routine exists twice, itentical except once for RX values and
// once for TX values.

	void send_regs_RX()
	{
		
	// Set up PLL A with the calculated  multiplication ratio
	setupPLL(SI_SYNTH_PLL_A, multRX, numRX); // , denom
	// Set up MultiSynth divider 0, with the calculated divider.
	// The final R division stage can divide by a power of two, from 1..128.
	// represented by constants SI_R_DIV1 to SI_R_DIV128 (see si5351a.h header file)
	// If you want to output frequencies below 1MHz, you have to use the
	// final R division stage
	setupMultisynth(SI_SYNTH_MS_0, dividerRX, SI_R_DIV_1);
	// Reset the PLL. This causes a glitch in the output. For small changes to
	// the parameters, you don't need to reset the PLL, and there is no glitch

	setupMultisynth(SI_SYNTH_MS_1, dividerRX, SI_R_DIV_1); // NRK clock 1 same freq

	// NRK - next I set the phase OS register (165) for CLK0 to 'divider'
	// Doing this action for CLK0 and leaving CLK1 in its default state
	// results in CLK0 being shifted 90 degrees from CLK1.

	if (QUAD) i2cWrite(CLK0_PHOFF, dividerRX); // NRK add; V1.12 added if stmt
	
	// V1.7 if() statement below added to see if dividerRX/TX has changed
	// and if so, do a reset of PLL_A
	// This is supposed to correct the loss of 90 degree phase shift.
	
	if(dividerRX != dividerLast)
	{
		dividerLast = dividerRX; // update to new value
		reset_PLL_A(); // do a reset when dividerRX or dividerTX has changed
	}




	// NRK - and then also switch on CLK1, using PLL_A

//	i2cSendRegister(SI_CLK1_CONTROL, 0x4F | SI_CLK_SRC_PLL_A);

}	


// ************  SEND TX REGISTERS TO Si5351a ******************************
//
// NOTE: This routine exists twice, identical except once for RX values and
// once for TX values

	void send_regs_TX()
	{
		
	if (!TX_CLK2) setupPLL(SI_SYNTH_PLL_A, multTX, numTX); 
	if (TX_CLK2) setupPLL(SI_SYNTH_PLL_B, multTX, numTX); // V1.5, V1.12 change to PLL_B
	if(!TX_CLK2)
	{
	setupMultisynth(SI_SYNTH_MS_0, dividerTX, SI_R_DIV_1);
	setupMultisynth(SI_SYNTH_MS_1, dividerTX, SI_R_DIV_1); // NRK clock 1 same freq
	if (QUAD) i2cWrite(CLK0_PHOFF, dividerTX); // NRK add V1.12 for if statement	
	}
	
	if (TX_CLK2) setupMultisynth(SI_SYNTH_MS_2, dividerTX, SI_R_DIV_1); 
	// NRK - next I set the phase OS register (165) for CLK 0 to 'divider'


	
	// V1.7 "if" statement below added to see if dividerRX/TX has changed
	// and if so, do a reset of PLL_A
	// This is supposed to correct the loss of 90 degree phase shift.
	
	if(dividerTX != dividerLast)
	{
		dividerLast = dividerTX; // update to new value
		reset_PLL_A();
		// Note for V1.12: I added a reset of PLL_B when using CLK2 but
		// was reminded that reset_PLL_A does both PLLs so I don't have
		// or need a separtate function for PLL B.
	}	

}	

// ***************************************************************************

// V1.1 this routine will now assume it is to change the TX frequency

// V2.0i, below is never called so I'm removing 2/12/2025

/*
void Set_Freq(uint32_t frequency)
{
	FoutTX = frequency;
	calc_TX(FoutTX);
	send_regs_TX();
}

*/

// NRK - make reset PLL a separate routine as it generates a pop when it is
// executed.

	void reset_PLL_A()
	{
		
		//i2cSendRegister(SI_PLL_RESET, 0xA0); // NRKw - change to below
		i2cWrite(SI_PLL_RESET, 0xA0);
		
		// NRKw - 0xA0 will reset both A and B per AN619. b7 resets B
		// and b5 resets A. So 0x80 clears B and 0x20 clears A and both
		// give 0xA0
	}

	void clock_0_ON()
	{
		// Finally switch on the CLK0 output (0x4F)
		// and set the MultiSynth0 input to be PLL A 
		// i2cSendRegister(SI_CLK0_CONTROL, 0x4F | SI_CLK_SRC_PLL_A); // NRK to below
		// Adding level control to the function 9/13/2020
		
		i2cWrite(SI_CLK0_CONTROL, (0x4F | SI_CLK_SRC_PLL_A) & level);
	}

	

		void clock_1_ON()
	{
		// i2cSendRegister(SI_CLK1_CONTROL, 0x4F | SI_CLK_SRC_PLL_A);
		// | = bitwise OR and & = bitwise AND
		// V1.7, the constant 0x4F is being changed to CLK1_hi so the
		// sideband select routine can alternate between 0x4F (output not
		// inverted) and 0x5F (output inverted).
		
		i2cWrite(SI_CLK1_CONTROL, (CLK1_hi | SI_CLK_SRC_PLL_A) & level); // NRK
	}

	void CLK0_1_on_quad() // V2.1f attempted sure fire on with good quad phase
	{
		send_regs_RX();
		//if (QUAD) i2cWrite(CLK0_PHOFF, dividerRX);
		clock_0_ON();
		if(QUAD) clock_1_ON();
		//if(!QUAD) Serial.println("QUAD is false");
	//	Serial.print("dividerRX = ");
	//	Serial.println(dividerRX);

	}

		void clock_2_ON() // V1.12 added
	{
		
		i2cWrite(SI_CLK2_CONTROL, 0x4F | SI_CLK_SRC_PLL_B & level); // V1.5: PLL_B
	}

  void clock_2_OFF() // V1.14
  {
		i2cWrite(SI_CLK2_CONTROL, (0x4F | 0x80) | SI_CLK_SRC_PLL_B & level); // V1.5: PLL_B, V2.1f
  //si5351aOutputOff(SI_CLK2_CONTROL); // turn clock 2 off
  }


  void clock_0_OFF() // V1.19
  {
  //si5351aOutputOff(SI_CLK0_CONTROL); // turn clock 0 off
	// V2.1f - turn off without changing the other bits

	i2cWrite(SI_CLK0_CONTROL, (0xCF | SI_CLK_SRC_PLL_A) & level);

	
  }


  void clock_1_OFF() // V1.19
  {
  //si5351aOutputOff(SI_CLK1_CONTROL); // turn clock 1 off
	i2cWrite(SI_CLK1_CONTROL, ((CLK1_hi | 0x80) | SI_CLK_SRC_PLL_A) & level);
  }  

// *********************** STEP UP AND STEP DOWN FREQUENCY ******************
 
 // Step the frequency up or down by current step value
 // V1.1 - in the Si5351 version, the stepping routines will also calculate
 // the new register values for RX and TX as appropriate

	  void step_up()
	 {
      //uint32_t adj_step_size;
      //adj_step_size = TuningStepSize[step_index];

    #ifndef HW8 // Below for NOT HW8

	  FoutRX += TuningStepSize[step_index]; // always step RX
	  calc_RX(FoutRX);

    #else  // Below for HW8 YES

    HW8DialRX += TuningStepSize[step_index];
    FoutRX = HW8DialtoFQ(HW8DialRX);
    calc_RX(FoutRX);
    #endif

	  
	  if(!VFO_Rx_Tx) // also step TX if in synch mode
	  {

			#ifndef HW8
			FoutTX += TuningStepSize[step_index]; // always step RX V2.1d uncommented this line
			calc_TX(FoutTX);

			#else // below for HW8 mode

			HW8DialTX += TuningStepSize[step_index]; // V2.1f uncommented this line
			FoutTX = HW8DialtoFQ(HW8DialTX);
			calc_TX(FoutTX);
			#endif

	  }
	 }

	  void step_down()
	 {
      //uint32_t adj_step_size;
      //adj_step_size = TuningStepSize[step_index];
    #ifndef HW8
      FoutRX -= TuningStepSize[step_index];
	  calc_RX(FoutRX);

	  #else // bdlow for HW8 mode

    if(TuningStepSize[step_index] <= HW8DialRX) // V2.0i make sure dial doesn't go below 0
    {HW8DialRX -= TuningStepSize[step_index];}
    FoutRX = HW8DialtoFQ(HW8DialRX);
    calc_RX(FoutRX);

    #endif

	  if(!VFO_Rx_Tx) // also step TX if in synch mode
	  {
				#ifndef HW8 // below NOT HW8
				FoutTX -= TuningStepSize[step_index];
				calc_TX(FoutTX);

				#else // below for HW8 mode

			HW8DialTX -= TuningStepSize[step_index]; // V2.1f uncommented this line
			FoutTX = HW8DialtoFQ(HW8DialTX);
			calc_TX(FoutTX);
			#endif

	  }  
	  
	 }

// ******************  INCREASE AND DECREASE STEP SIZE ***********************

// There are 7 step sizes, 0 through 6 step_index

	  void stepSize_up()
	  {
		step_index++;
		if (step_index == 7) step_index = 0; // roll over
	  }
	  
	  void stepSize_down()
	  {
		step_index--;
		if (step_index < 0) step_index = 6; // roll under
	  }
	 
// Borrowing a lot of code for rotary encoder and general logic from
// Si570_2015.ino

// ******************  LOW_REZ ***********************************

// The rotary encoder can be too active for fine control, and when
// pressing the knob to activate the switch, it's easy to accidentally
// move the knob, selecting the wrong option. 
// LOW_REZ will work like get_direction, but will only output an UP
// or DN count if a certain number of UP / DN counts have accumulated.

    uint8_t low_rez()
    {
        uint8_t now_dir, rvalue;
        rvalue = 0;

        now_dir = get_direction();

        if(now_dir)
        {
            if(now_dir == 1)
            {
                up_pile++;
                dn_pile = 0;
            }
            else if (now_dir == 2)
            {
                up_pile = 0;
                dn_pile++;
            }
            
           if(up_pile == 4) // V1.14 changed 3 to 4
           {
            rvalue = 1;
            up_pile = 0;
           }
           else if(dn_pile == 4) // V1.14
           {
            rvalue = 2;
            dn_pile = 0;
           }
           
        }
        
        return rvalue;
    }
	

// ********************** ROTARY ENCODER *********************************
// 
// Note that I have a Word document called Rotary Encoder.doc in my Word
// programming folder. It explains my method for reading the encoder.

// entered when ENC_A or ENC_B known to have changed. But not which one.

// figure out which way to move and then do it ...

// Returns 1 for CW, 2 for CCW, 0 if there was no valid step

   void RotaryISR()

    {
        uint8_t enc_now;

       // r_dir = 0;
        enc_now = (PIND & ENC_AB_MASK)>>2; // 1/31/2016, 6/6/2018, 1/10/2023 V1.1
		// Note on the right shift >>2 above. The logic below is hard coded for
		// the encoder bits to be PORTC, 0 and 1 and now they're PORTD 2 and 3.
		// Rather than do that logic over, I'm just shifting my byte holding
		// pin status two to the right to put it in 0 and 1.

// See if A (bit 1) changed, if so then if A<>B, increment,
// otherwise decrement

// In the IF below I XOR bit 1 of the current pin stats with bit 1
// of the stored status.  If different, the result will be 1 (TRUE)

// In the nested IF, I see of bit 0 equals bit 1.  On the left, shift
// bit 0 to bit 1 position and mask off bit 1, on the right just mask.
// Then XOR to see if they're equal. Note caret ^ is bitwise XOR operator

// V1.9: Below gives the ability to set transitions per step to anything
// 1 to 255


	do_count = false;
	
	encoder_div++; // v1.9
	if(encoder_div == DIV_FACTOR)
	{
		do_count = true;
		encoder_div = 0;
	}
	

        if ((enc_now & 0b00000010) ^ (encoder_pins & 0b00000010)) 
        {


            if (((enc_now<<1) & 0b00000010) ^ (enc_now & 0b00000010)) // 2/1/2016, 6/6/2018
                {


				if(do_count) queue--;

                }

                else
                
                {
            
               // r_dir = 2;
				if(do_count)queue++;
                }
        }

// Now if A wasn't the one that changed it must be B (bit 0), so we do
// the opposite for A==B or A<>B ...
// So below, I'm doing the same comparison I did above, but taking the opposite
// actions regarding assigning values to r_dir

// queue is V1.0 addition, represents backlog of step demands for the mainline
// code to do. Positive for UP steps and negative for DN.

            else

            {

            if (((enc_now<<1) & 0b00000010) ^ (enc_now & 0b00000010)) // 2/1/2016, 6/6/2018
            
            {

               // r_dir = 2;
				if(do_count)queue++;
            }

                else

                {
               // r_dir = 1;
				if(do_count)queue--;
                }
				
            }

		//if(abs(queue) > qMAX) qMAX = abs(queue); // ******************* TESTING **************
		
        encoder_pins = enc_now; // reset historic state of pins. 
		                        //Note: Here it's already shifted >>

	
    }

// **************************************************************************


// V1.8, below I'm recreating Rotary() for functions outside the main loop. It
// will look at queue and respond based on those counts.

   uint8_t Rotary()

    {

        // consider a ms or so of delay here to debounce ...


        uint8_t r_direction;

        r_direction = 0;
   
		// V1.8, return direction or no movement based on queue from ISP
		
		if(queue > 0) r_direction = 1;
		if(queue < 0) r_direction = 2;
		queue = 0; // reset to no encoder action
        return(r_direction);
    }


// ***** get_direction will check on the encoder to see if it has
// ***** changed, and if so will return the direction code. If 
// ***** not, it will return 0.

    uint8_t get_direction()
    {

      uint8_t zz = 0;

      zz = Rotary(); // Get direction               
        
      return zz;     
     }

	 


// Clear LCD line 1 (bottom line)
// and clear LCD line 0 (top line)
// Also puts cursor back to start of line
	void clearLine1()
	{  
   // #ifndef I2C_LCD // if parallel mode
  //if(!LCDmode) pins4LCD();
 // #endif
		lcd.setCursor(0,LCDBottomLine);
		lcd.print("               "); // 15 blanks
		lcd.setCursor(0,LCDBottomLine);
	}
	
	void clearLine0()
	{
		lcd.setCursor(0,LCDTopLine);
		lcd.print("               "); // 15 blanks
		lcd.setCursor(0,LCDTopLine);
	} 
	

// Format frequency as ASCII with delimiters and print to LCD
// Borrowed from DDS60_m328P.c, with some changes made

// Arguments are 32 bit frequency unsigned integer and line 0 for upper
// 1 for lower. It starts at column 0 either way.

// In V1.3, I'm adjusting displayed by CW_PITCH if modeCW is TRUE
// In V1.4, I'm taking that adjustment out since Fbase and FbaseR contain
// the operating frequencies I want to display.
// In V1.4 I also move printing one space to the right to leave room for R or
// T prefix. All the stuff for formatting doesn't have to change, just the LCD
// printing statement.
// The calling routine will decide whether to print Fbase or FbaseR and will
// make the LCD line number suit, 0 for RX and 1 for TX.

// In V1.5, I'm going to "trim" less significant digits when changing the
// step value. So in 1 kHz step mode, I won't show 7,025,756, it will just
// be 7,025,000

/* REWORKED WITH V2.0i,2/13/2025.  WITH HW8, I HAD A LOT OF TROUBLE WITH
   displaying the frequency. This routine expected that the lowest two
   3-digit fields would always contain data. Trying to print a number like
   50,000 dial counts could cause it to blow up. So now it will work all
   the way down to 000,000,001, I think. (Although higher zeroes are
   suppressed.)
   */

static void Print_freq(uint32_t f32, uint8_t line)

	{
/*
    #ifdef HW8
    Serial.print("f32 before and after: ");
    Serial.print(f32);
    Serial.print(", ");
    f32 = HW8FQtoDial(f32);
    Serial.println(f32);
    #endif
*/

		uint8_t length, i, n; // V2.0i added n
		uint8_t dest_point = 10; // end of dest field  *** WAS 9
    char T_R_CHAR = 'R';
		
// V1.4: If I'm printing to line 0 (upper), I'll put an 'R' prefix in pos. 0
//       If line 1 (lower), I'll put a 'T' there.		

// V1.19 skipt the R and/or T if in TX_ONLY mode NO - I'm going to put in an * to 
// show I'm in TX_ONLy mode
// V2.0i don't assume any field holds 3 digits of data.

 if(TX_ONLY) T_R_CHAR = '*';
  {
	if(line == 0)
	{
		lcd.setCursor(0, line);
		lcd.print(T_R_CHAR);
	}
	else
	{
		lcd.setCursor(0, line);
		lcd.print('T');
	}
  }
		ultoa (f32, a_freq, 10); // left justifies in 2nd arg field
		                             // 3rd argument is RADIX

/*
#ifdef HW8
Serial.print("a_freq: ");
Serial.println(a_freq);
#endif
*/


// I need to fill the destination field with space chars or I can have a
// null in the first position if my frequency doesn't have a 10 MHz digit

	for (i=0; i<11; i++) // *** 11 WAS 10
	{
		a_freq_fmt[i] = ' ';
	}

	a_freq_fmt[11] = 0; // just in case the compiler doesn't initialize this
	                    // NULL terminator

// Initially I'll copy the whole thing to a_freq_fmt with comma & decimal

		length = (uint8_t) strlen(a_freq); // length of string not including null at end.
		--length; // make zero based

    n = 3;
    if(length < 2) n = length + 1;
		for (i=n; i!=0; i--)
		{
			a_freq_fmt[dest_point--] = a_freq[length--]; // move fract kHz, ones, tens, hundreds
		}
  //if(length > 0) 
  if(f32 >= 1000) a_freq_fmt[dest_point--] = '.'; // add a decimal point
 // else goto completed;
		else goto completed;

    n = 3;
    if(length < 2) n = length + 1;

		for (i=n; i!=0; i--)
		{
			a_freq_fmt[dest_point--] = a_freq[length--];
		}
		if(f32 >= 1000000)  a_freq_fmt[dest_point--] = ','; // add the comma
    else goto completed;

		while (length != 0xFF) 
		{
			
			a_freq_fmt[dest_point--] = a_freq[length--];
		}
    completed:;
		lcd.setCursor(1, line);// V1.4 changed 0 to 1
		lcd.print(a_freq_fmt);

    /*
    #ifdef HW8
    Serial.print("a_freq_fmt: ");
    Serial.println(a_freq_fmt);
    #endif
    */

	// V1.3 I'm going to print LSB or USB along with the frequency
	
	printSBSelected();
	// LCDPrintMult(); 
	ShowStepFast();
	//SingleBeep(); // **** TESTING  YES, LOTS OF BEEPS *****
	}


// Read the pushbutton switches and return value
// Values returned are 1 & 2 for switches 1 & 2
// and 3 for both switches closed, and zero if
// no switches are closed. Note that SW3 is wired
// to close both 1 and 2 using diodes.

// Below, pushbuttons ground pins when closed, so XOR operation 
// ^ 0x01 makes switch state true (1) when depressed. Intent is
// PBstate = 1: SW1, 2: SW2, 3: SW3 (both) and 0: NONE

// r1.2, rewrite the part that avoids split read
// to eliminate wasting 100 us if read was 0


    uint8_t read_switches()
    {
        uint8_t x, y;

        x = digitalRead(SW1)^0x01; // V1.1 was ENC_SW
        y = digitalRead(SW2)^0x01;
        if(y)
        {
            delayMicroseconds(100); // avoid "split read" if both closed
            x = digitalRead(SW1)^01;
            x = x + (y<<1);
        }
        return x;
    }
	
	
// Get switch value and if it is not zero, wait until it returns
// to zero before returning with the original value. There is a 1 ms
// delay before each re-check for returning to zero. Also has a
// debounce feature - after the switches return to zero, the reading
// is checked again after 5 ms to make sure they stayed zero.


    uint8_t getSwitchWait()
    {
        uint8_t x;
        do
        {
        x = read_switches();
        if(x)
            {
                
                delay(1);
            }
          delay(5);
        } while (read_switches());

        return x;
    }
	


// ************ READ STATUS OF PUSHBUTTON SWITCHES ************************

// This routine will return false if neither switch is closed. If a switch
// is closed, It checks for six possible statuses. PB1 short or long close,
// PB2 short or long close, and PB3 short or long close.
// In Elecraft terminology, these actions are called "Tap" and "Hold".

// Note || operator is Boolean (logical) OR 


	bool GetButtons()
	{
		bool rvalue = false; // means no info, no PB closed
		bool beep1 = false; // latches to limit to one beep
		bool beep2 = false; // latches to limit to one doublebeep
		
		if(digitalRead(SW1) && digitalRead(SW2)) goto gohome; // no buttons
		
		rvalue = true; // at least something was closed, flag it
		//battTimer = millis(); // reset inactivity timer
		
		delay(10); // delay 10 ms and see if both are closed
		
		timing_start_2 = millis();	// measure how long button held closed
		
		while(!digitalRead(SW1) && !digitalRead(SW2)) // Here checking PB3
		{
			if(millis() > timing_start_2 + 500)
			{
				PB3Long = true;
				PB3Short = false;
				if(!beep2) DoubleBeep();
				beep2 = true;
			}
			else
			{
				PB3Short = true;
				if(!beep1) SingleBeep();
				beep1 = true;
			}
						
		}
		
		
		if(PB3Long || PB3Short) goto waitforopen; // only latch one
		
		// Wasn't PB3, so check for SW1		
		
		while(!digitalRead(SW1))
		{
			if(millis() > timing_start_2 + 500)
			{
				PB1Long = true;
				PB1Short = false;
				if(!beep2) DoubleBeep();
				beep2 = true;
			}
			else
			{
				PB1Short = true;
				if(!beep1) SingleBeep();
				beep1 = true;
			}
			//goto waitforopen;
		}
		
		if(PB1Long || PB1Short) goto waitforopen; // can only latch one boolean
		
	// Now we check for SW2 short or long
	
	while(!digitalRead(SW2))
		{
			if(millis() > timing_start_2 + 500)
			{
				PB2Long = true;
				PB2Short = false;
				if(!beep2) DoubleBeep();
				beep2 = true;
			}
			else
			{
				PB2Short = true;
				if(!beep1) SingleBeep();
				beep1 = true;
			}
			//goto waitforopen;
		}	
		
		waitforopen:;
		
		while(!digitalRead(SW1) || !digitalRead(SW2)); // Stay until both open
		
		// Now with V2.1e, I want to count the number of short presses
		// I only need to do PB3 now. So the same later for others if needed

		if(!PB3Short) PB3CNT = 0; // any action other than PB3 short resets count

		if(PB3Short)
		{
			if(millis() - PB3TIME < 2500)
			{
			PB3TIME = millis();
				if(++PB3CNT == 3)
				{
					doCWPhone(); // If I do this action, make PB3Short false
					PB3CNT = 0;
					PB3Short = false;
					rvalue = false; // Return with no button information
					//PB3TIME = 0;
				}
			}
		
		else // Here, time was > 2.5 seconds
			{
				PB3CNT = 1; // too late for 3rd, but now is new 1st tap
				PB3TIME = millis();
			}
		}

		gohome:; // jumping here if not buttons pressed
		return rvalue;
		
	}
//  v1.8 COMMENT OUT FOR NOW
/*
	void testButtons()
	{
		if(GetButtons())
		{
			if(PB1Short)
			{
				Serial.println("PB1 Tap");
				PB1Short = false;
			}
			
			if(PB1Long)
			{
				Serial.println("PB1 Hold");
				PB1Long = false;
			}
		if(PB2Short)
			{
				Serial.println("PB2 Tap");
				PB2Short = false;
			}
			
			if(PB2Long)
			{
				Serial.println("PB2 Hold");
				PB2Long = false;
			}
		if(PB3Short)
			{
				Serial.println("PB3 Tap");
				PB3Short = false;
			}
			
			if(PB3Long)
			{
				Serial.println("PB3 Hold");
				PB3Long = false;
			}
		}
	}
	
	*/	
	void doLSB_USB() // V1.3 adds this option.
	{
		//LSBMode = !LSBMode; // toggle state let calling routine do this
		
		// Note, if I was in CW-USB mode, my VFO was shifted by the pitch value
		// above the displayed frequency. So I need to subtract 2 * pitch from
		// Fbase to get my offset in the right direction and magnitude.
		// Similarly, If I was in the CW-LSB mode before and now go to USB, I need
		// to ADD 2 * pitch to Fbase.  NRK 9/2019
		
		/* V1.1, Si5351a does not have sideband relays ..
		
		if(LSBMode)
		{
			digitalWrite(LSB_RESET, HIGH); // energize reset coil
			delay(250); // latching relay so just pulse needed
			digitalWrite(LSB_RESET, LOW);
			// Fbase -= 2*CW_PITCH; // V1.4b - fix freq change
		}
		else
		{
			digitalWrite(USB_SET, HIGH);
			delay(250);
			digitalWrite(USB_SET, LOW);
			// Fbase += 2*CW_PITCH; // V1.4b - fix freq change
		}
		*/
		
		if(VFO_Rx_Tx == 0) // v1.4b, address RIT on or off
		{
			Go_(FoutTX); // swap freq to opposite side of zero beat
		}
		else
		{
			Go_RIT(); // v1.4b for case where RIT is on
		}
		
		// Print_freq(Fbase, 0); // This will fix display offset and USB/LSB 
		                      // indication v1.4b remove this
							  
		// Set SSB relay choice to agree with sideband. LSB gives HIGH and 
		// USB gives low on output pin.
		
		
		// V1.7, I'm adding to the LSB/USB selection function a part to 
		// invert the CLK1 output. I do this by setting CLK1_hi to 0x4F
		// for LSB and to 0x5F for USB, then calling clock_1_ON which will
		// toggle that bit to invert / not invert the clock 1 output
		
		CLK1_hi = 0x5F; // assume USB V1.7
		if(LSBMode) 
		{
			// digitalWrite(SB_Relay, HIGH); V2.0g no more SB_relay
			CLK1_hi = 0x4F; // for LSB, V1.7
			
		}
    // V2.1c: Mateusz report SB selection doesn't work. I see that below, I left in
    // an 'else' statement resulting in skipping the call to clock_1_ON()

		//else // digitalWrite(SB_Relay, LOW); V2.0g no more SB_relay V2.1c, see above comment

		if(QUAD && !TX_ONLY) clock_1_ON(); // V1.7, call to change output inverted / not inverted
		                       // V1.12, don't turn CLK1 ON if QUAD not true
                           // V1.19, don't turn CLK1 ON if TX_ONLY mode
		
		printSBSelected(); // v1.4b
	}

// *********** TURN RIT ON ***********************************************

// Just flag that it's on and put the TX frequency on the lower line
// Initially TX and RX frequencies will be identical


	void RIT_ON()
	{
		VFO_Rx_Tx = 1; // flag RIT mode
		if(RITSave != 0) 
		{
			FoutRX = RITSave; // Restore previous offset
			Go_RIT();
       #ifndef HW8					   
		   Print_freq(FoutRX, LCDTopLine);
       #else
		   Print_freq(HW8DialRX, LCDTopLine);
       #endif
		}
       #ifndef HW8					   
		   Print_freq(FoutTX, 1);
       #else
		   Print_freq(HW8DialTX, 1);
       #endif
	}

// When RIT is turned OFF, the transmit frequency becomes the operating
// frequency, so FbaseR is set equal to Fbase
	
	void RIT_OFF()
	{
		VFO_Rx_Tx = 0; // TX & RX in synch
		LCDClearFreq(1); // Clear frequency info from bottom line
		RITSave = FoutRX; // Keep and restore next time RIT turned on V1.4a
		FoutRX = FoutTX;
		Go_(FoutTX); // return to TX (TRX) frequency
       #ifndef HW8					   
		   Print_freq(FoutTX, LCDTopLine);
       #else
		   Print_freq(HW8DialTX, LCDTopLine);
       #endif
	}
		
// Swap TX and RX frequencies when in RIT mode

	void TXRXSwap()
	{
		uint32_t holder;
		holder = FoutTX;
		FoutTX = FoutRX;
		FoutRX = holder; // Now they're swapped ...
		#ifdef HW8 // V2.2a
		HW8DialTX = HW8FQtoDial(FoutTX);
		HW8DialRX = HW8FQtoDial(FoutRX);
		#endif
		Go_RIT(); // Send RX freq to the synthesizer
		UpdateScreen(); // and fix screen
	}

// I'm adding Go_RIT() for when in RIT mode. No argument because I think
// Fbase and FbaseR will always be right when the call is made. V1.4
	
	void Go_RIT()
	{
		//if(VFO_Rx_Tx == 1) // shouldn't need "if" if I use as intended
		{
		//buildSiWords(); // Will calc new RFREQ and build the register values
		//send_new_freq(); // Will send only RX update
		calc_RX(FoutRX);
		calc_TX(FoutTX);
		send_regs_RX();
		}
	}

  void spot_it()
  {
    SPOT = !SPOT; // toggle state
    if(SPOT)
    {
    lcd.setCursor(0, LCDBottomLine );
		lcd.print(" SPOT");
   // clock_0_ON();
   // if(QUAD) clock_1_ON();
	 		CLK0_1_on_quad(); // V2.1f this and next sort of bandaid to restore quadrature
			reset_PLL_A();   // during SPOT function
    }
    else {

    lcd.setCursor(0, LCDBottomLine );
		lcd.print("     "); // V1.19 - erase 
    if(!key_down) // outputs off unless key is closed
    {
    clock_0_OFF();
    if(QUAD) clock_1_OFF();
    }
    }  
  }
// USERS:
// If you changed the size of the step choices elsewhere, you will want to
// change the text fields below to suit. The field only has three spaces,
// so be aware of that.

// ShowStepFast() shows the step size in the top line, rightmost three chars


	void ShowStepFast()
	{
 // #ifndef I2C_LCD // if parallel interface
  //if(!LCDmode) pins4LCD(); // make pins outputs for LCD
  
  //#endif
		//SingleBeep(); // beeped like crazy
		lcd.setCursor(13, 0);
		lcd.print("   "); // clear 13, 14, 15
		lcd.setCursor(13, 0);
		//lcd.print("TST");// *************** TESTING
		//goto outtahere;
		if(step_index == 0) lcd.print("  1");
		else if(step_index == 1) lcd.print(" 10");
		else if(step_index == 2) lcd.print("100");
		else if(step_index == 3) lcd.print(" 1k");
		else if(step_index == 4) lcd.print("10k");
		else if(step_index == 5) lcd.print(".1M");
		else if(step_index == 6) lcd.print(" 1M");
		//outtahere:;
	}

// *** UpdateScreen() will update everything to the screen, taking
// RIT into consideration and printing one or two lines as required, 
// plus doing the x1/x4 and USB/LSB fields

	void UpdateScreen()
	{
		//SingleBeep(); // *****************TEST
		lcd.clear(); 
		if(VFO_Rx_Tx == 0)
		{
       #ifndef HW8					   
		   Print_freq(FoutTX, LCDTopLine);
       #else
		   Print_freq(HW8DialTX, LCDTopLine);
       #endif// Here in sync mode
		}
		if(VFO_Rx_Tx == 1)
		{
       #ifndef HW8					   
		   Print_freq(FoutRX, LCDTopLine);
       #else
		   Print_freq(HW8DialRX, LCDTopLine);
       #endif
       #ifndef HW8					   
		   Print_freq(FoutTX, 1);
       #else
		   Print_freq(HW8DialTX, 1);
       #endif
		}
		printSBSelected();
		ShowStepFast();
		// SingleBeep(); // TESTING *********************

	}

// speaker beeps to go with PB closures ...

// V2.0a 2/4/2025 if Keyer is Active, button press is for sending
// a message, so I will silence the beeps at that time

	void SingleBeep() // A beep of 40 WPM speed
	{
    if(!KeyerActive)
    {
		delay(20);
		tone(spkrpin, CW_pitch_ST, 30); // V1.19
		//delay(30);
		//noTone(spkrpin);
		delay(20);
    }
	}
	
	void DoubleBeep() // Two dits at 40 WPM speed (30 ms/dit)
	{
		delay(20);
		tone(spkrpin, CW_pitch_ST, 30); // V1.19
		//delay(30);
		//noTone(spkrpin);
		delay(30);
		tone(spkrpin, CW_pitch_ST, 30); // V1.19
		//delay(30);
		//noTone(spkrpin);
		delay(20);
	}

// ************** Go_ ******************************************************


// With V1.4, Go_ will stay the same, but it should only be called when in 
// sync mode. This will cause bounds and freeze checks and will update 
// both the TX and RX registers.



    void Go_(uint32_t thefreq)
    {
        //Fbase = thefreq;
        //if(quad) 
        //{Fout = thefreq << 2;}
        //else 
        {FoutTX = thefreq;}
        //UpdateFrequency();
		calc_RX(FoutRX);
		calc_TX(thefreq);
        send_regs_RX();
    }


 
	void LCDClearFreq(uint8_t line)
	{
		// Argument is 0 for top line or 1 for second line
		// Note this routine does not put cursor back home
		// Here we clear just the 1st 13 chars, leaving room
		// for USB or LSB or x4 info at end of field
		
		lcd.setCursor(0,line);
		lcd.print(F("             "));
		
	} 

	
// Print "LSB" or "USB" depending on which is selected v1.3
// As of v1.4b, the labels are CWL and CWB when in CW mode


// V2.0 adds 'KYR' in this field, if in keyer mode

	void printSBSelected()
	{
  //#ifndef I2C_LCD // V2.0g
  //if(!LCDmode) pins4LCD(); // make pins outputs for LCD
  
  //#endif

    if(!MorseToLCD)
    {
		lcd.setCursor(13,1); // bottom line, far right
    printedKYR = false; // assumption
    if(!KeyerActive)
    {
		if (LSBMode)
        {
			if(modeCW) lcd.print("CWL"); // v1.4b
            if(!modeCW) lcd.print("LSB");
        }
        else
        {
			if(modeCW) lcd.print("CWU"); // v1.4b
            if(!modeCW) lcd.print("USB");
        }
    }
    else // V2.0f - else, if keyerActive
    {lcd.print("KYR");
    printedKYR = true;
	}
    }
  }
// V2.0 below actions taken from loop() in-line code so keyer can also call it
/*
  Actions include
  sidetone ON/OFF
  TX_Out key jack Low or High (Low is generally key down)
  CLK0, 1, 2 turn ON or OFF. They may go different ways
  Shift frequency of specific CLKs for pitch and RIT and mode

	With V2.1f, if DoTransmit is false, no keying of output
	signals CLK0,1,2 ON or OFF will happen. Just sidetone.

*/



  void KeyDownActions()
  {
    key_down = true;

  //************* First, sidetone  **********


		 if(sidetone) tone(spkrpin, CW_pitch_ST); // V1.3 create sidetone when key down V1.19 add _ST


  // Actions below won't take place if in command mode
	// also, if DoTransmit is false V2.1f

    if(!CMNDMode && DoTransmit)
    {
			send_regs_TX(); // change output frequency if freqired

    // turn ON clocks 0 and 1 if required by TX_ONLY
    if(TX_ONLY)
    {
     //clock_0_ON(); // V1.19
     //if(QUAD) clock_1_ON(); // V1.19
		CLK0_1_on_quad(); // V2.1f this and next sort of bandaid to restore quadrature
		reset_PLL_A();   // after sendTestText ends
    }

    // turn OFF clocks 0 and 1 if required by TX_CLK2 and CLK0_1_OFF
    // Plus, turn ON CLK2 if required

    	if(TX_CLK2 && CLK0_1_OFF_KYDN)
			{
			//si5351aOutputOff(SI_CLK0_CONTROL); // CLK0 off
			//si5351aOutputOff(SI_CLK1_CONTROL); // CLK1 off
			clock_0_OFF();
			clock_1_OFF();
			}
			if(TX_CLK2) clock_2_ON();  // V1.12
    // V2.0h below - testText inhibits transmit key closure
			if(DoTransmit && OnAir && !testText) digitalWrite(TX_Out, keyDNstate); // echo keyed line AFTER registers updated   

      } // CMNDMode false block ends here
 
  } // KeyDownActions function ends here.
 

			// V1.12 If using CLK2, turn off RX output to CLK0 and 1
			// during key down if flag is true
			


  void KeyUpActions()
  {
    key_down = false; // V1.4

			digitalWrite(TX_Out, keyUPstate); // raise keyed line BEFORE regs sent
			noTone(spkrpin); // turn off sidetone

      if(!CMNDMode && DoTransmit)
      {
			send_regs_RX(); // change output frequency 
       //tone(spkrpin, 300);
      //delay(100);
       //noTone(spkrpin);			
      if(TX_ONLY)
      {
      clock_0_OFF(); // V1.19
      if(QUAD) clock_1_OFF(); // V1.19
      }
			if(TX_CLK2) clock_2_OFF(); //si5351aOutputOff(SI_CLK2_CONTROL); // V1.12 turn clock 2 off
			
			// V1.12 - NOW, if CLK2 is in use and CLK0 was turned off during key down
			// turn it back on. Then, if CLK1 is not turned off by QUAD = false, then
			// turn it back on too.
			
			if(TX_CLK2 && CLK0_1_OFF_KYDN)
			{
				/*
				if(LSBMode) CLK1_hi = 0X4F; // V2.1f
				else CLK1_hi = 0x5F; // V2.1f
				if (QUAD) i2cWrite(CLK0_PHOFF, dividerRX); // V2.1f, trying to fix loss of quad
				clock_0_ON();
				if(QUAD) 
				{clock_1_ON();
				*/

				CLK0_1_on_quad();
				reset_PLL_A();
				/* 
				Serial.print("dividerRX: ");
				Serial.println(dividerRX);
				Serial.print("CLK0_PHOFF = ");
				Serial.println(read_register(CLK0_PHOFF)); // V2.1f

				Serial.print("SI_CLK0_CONTROL = ");
				Serial.println(read_register(SI_CLK0_CONTROL), HEX);
				Serial.print("SI_CLK1_CONTROL = ");
				Serial.println(read_register(SI_CLK1_CONTROL), HEX);
				*/
				}
			}
    }
 // }

// The doPitch routine will display the pitch and vary it in 10 Hz steps
// using the encoder. A maximum of 1000 Hz and minimum of 150 Hz is enforced
//
// After each step, the Si570's registers are updated with the new offset value
// This should allow hearing the effect on pitch when using a receiver 
// correctly tuned to a station. v1.4b

// V2.0 will turn the speaker on during adjustment, if it wasn't already

	void doPitch()
	{


		 // adapted from doSteps
        //delay(2000);
		boolean chosen = false;
        showPitch();
        //encoder_pins = PIND & 0b00001100; // r1.2		
		while (!chosen)
		{

		r_dir = low_rez();	
		if (r_dir == 1)
			{
			    CW_pitch += 10;
			}
			else if(r_dir == 2)
			{
				CW_pitch -= 10;
			}
		if (CW_pitch == 1010) CW_pitch = 1000; // v1.4b ceiling is 1000
		if (CW_pitch == 140) CW_pitch = 150; // v1.4b floor is 150
    CW_pitch_ST = CW_pitch; // V2.0 keep equal unless one is out of bounds
		
		if (r_dir) showPitch();
    tone(spkrpin, CW_pitch); // V2.0
		r_dir = 0;
		
		// Below, my intent is to update the Si570 with the new
		// offset figured in, but I hope not affect the display:
		
		if(VFO_Rx_Tx == 0) // v1.4b, address RIT on or off
		{
			Go_(FoutTX); // swap freq to opposite side of zero beat
		}
		else
		{
			Go_RIT(); // v1.4b for case where RIT is on
		}
		
        // PBstate = getSwitchWait(); // v1.4a
		// if (PBstate == 1) chosen = true;	
		if(GetButtons())
		{
			if (PB3Short)
			{
				chosen = true; //v1.4a
				PB3Short = false;
			}
	
	    }
		}
    noTone(spkrpin);
	}                     // end of doPitch


	void showSidetone()
	{
		clearLine1();
		if(sidetone)
		{
		lcd.print(" ON");
		tone(spkrpin,500, 250); // 0.25 s beep confirms ON
		}
		else
		{
			lcd.print(" OFF");
		}
		delay(1000);
		printSBSelected(); // V1.3 restore sideband indication
	}

	void showPitch()
	{
		clearLine1();
		lcd.print(CW_pitch);	
		printSBSelected(); // V1.3 restore sideband indication
	}

	// v1.4b the doCWPhone routine allows choosing CW or "phone" mode
	// the only difference is that in CW mode, the output frequency is
	// offset by the value of CW_pitch and in "phone" mode, the output
	// frequency equals the frequency displayed on the LCD.
	
	void doCWPhone()
	{
		modeCW = !modeCW; // toggle modes
		printSBSelected(); // change indication to suit
		
	// next I turn the offset on or off based on new status
	
		if(VFO_Rx_Tx == 0) // v1.4b, address RIT on or off
		{
			Go_(FoutTX); 
		}
		else
		{
			Go_RIT(); // v1.4b for case where RIT is on
		}	
		
	}

// For debugging, etc - routine to give the number of beeps in the argument
	
	void do_beeps(uint8_t b_count)
	{
		while(b_count)
		{		
			tone(spkrpin,600, 60);
			delay(100);
			b_count--;
		}
	}

// V2.1e, for testing
/*
	void SPRINT_inhibits()
	{
		Serial.print("\nDoTransmit = ");
		Serial.println(DoTransmit);
		Serial.print("OnAir = ");
		Serial.println(OnAir);
		Serial.print("testText = ");
		Serial.println(testText); // Note: = 0 when OK to key TX
	}
*/
	


// V2.0, making pins D4 and D5 share duty as LCD (outputs) and paddles (inputs)

/*
  void pins4LCD()
  {
    pinMode(DotContact, OUTPUT);
    pinMode(DashContact, OUTPUT);
    LCDmode = true;
  }

  void pins4paddle()
  {
    pinMode(DotContact, INPUT_PULLUP);
    pinMode(DashContact, INPUT_PULLUP);
    LCDmode = false;
  }
/*
// 9/13/2020 change output levels. This is an experiment where I'll 
// see the effect of the 4 possible states on the scope and log power
// meter.  Not used in normal program execution.
// Press a button to advance to the next state. After the 4th, revert
// to normal operation. To repeat, it's necessary to restart the Arduino.


		void do_levels()
		{
			uint8_t levels[] = {mA_2, mA_4, mA_6, mA_8};

			for(uint8_t q = 0; q < 4; q++)
			{
				level = levels[q];
				clock_0_ON();
				clock_1_ON();
				clearLine1();
				lcd.print(2 + 2*q); // 2, 4, 6, 8 mA out
				lcd.print(" mA");
				while(1)
				{
					if(GetButtons()) break;
				}
			}
			clearLine1();
		}
				
/*

// Trying to get an ASCII field to print to the LCD

void	qmax2LCD()
{
	uint8_t fieldindex = 0;
	while (!isNUM(qmaxASC[fieldindex])) fieldindex++;
	if(fieldindex > 4) fieldindex = 1;
	char a = ' ';
	if(isNUM(qmaxASC[fieldindex])) a = qmaxASC[fieldindex];
	qmax4LCD[0] = a;
	a = ' ';
	if(isNUM(qmaxASC[fieldindex+1])) a = qmaxASC[fieldindex+1];
	qmax4LCD[1] = a;
	a = ' ';
	if(isNUM(qmaxASC[fieldindex+2])) a = qmaxASC[fieldindex+2];
	qmax4LCD[2] = a;
}

bool isNUM(char z)
{	
	bool retBOOL = false;
	if ((z >= '0') && (z <= '9')) retBOOL = true;
	return retBOOL;
}
*/