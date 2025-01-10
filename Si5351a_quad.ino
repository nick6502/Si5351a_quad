
// V1.18, December 12, 2024


/*
	August 8, 2023 - starting modifications for V1.12. The following user
	configurable options have been added:
	
	* An option to use an I2C LCD interface or use the existing parallel 
	  interface. The required display is still 2x16 LCD
	
	* Ability to turn off the quadrature output if it's not desired or needed.
	  When this is done, there's no output from Clock 1
	  
	* Add an I.F. offset for superhet receivers. The specified offset is added
	  to the displayed receive frequency
	  
	* Add an output from Clock 2 which provides the transmit frequency for a
	  "straight through" transmitter with no hetrodyning adjustment. This 
	  output would only be present when the key or PTT is activated
	  
	* Turn Clock 0 and 1 (if used) off on Key Down. The intent is that this 
	  would mute the receiver using that clock for its L.O. This option is 
	  available when Clock 2 is being used for the transmitter as described
	  above
	
	
	
*/

// September 18/2023 - V1.13 Ron Taylor reported that his unit came up in
//                     the Key Down mode. I added a statement in SETUP() to
//                     start up in Key Up mode. 

// August 8, 2023 - Saving V1.11 as functional before proceeding to try to
// add I2C LCD as an option plus other features.

// **************************************************************************

// si5351a_quad.ino - a full-featured I/Q VFO
// by Nick Kennedy, WA5BDU
// V1.7 issued 4/29/2022
// V1.8 interrupt driven rotary encoder, scroll down for details
//      1/11/2023.  Previous revision BACKUP marked with this date
// V1.9, add more possible ratios for steps per encoder pulse 1/24/2023

// 4/16/2023 - IDE wouldn't load file because the first letter was s
// lowercase but the folder name started with S (UC). So I changed 
// filename 1st letter to UC.



// Si5351a controller with I/Q outputs, rotary encoder tuning
// and LCD display. I started with some demo code from Hans Summers, which
// could take a frequency, calculate registers, and send them to the chip.

// "Hans Summers, 2015 Website: http://www.hanssummers.com"


// Expanded by Nick Kennedy WA5BDU based on Hans' presentation at
// FDIM 2018. First I want to set up clock 1 to have output on the same
// frequency as clock 0. Secondly, I want to set the phase shift register
// per the presentation and see if I get 90 degree shifted signals.
// My changes generally marked 'NRK'
// NRK 5/23/2018
// Quadrature output down to 3.5 MHz working, 5/24/2018
// Moved the commands to turn CLK0 and CLK1 ON out of setFrequency and
// made them separate routines, called only once. That seems to work OK.
// Also moved the Reset PLL_A command to a separate routine which I currently
// only call when changing bands. That works OK too.

// Next, I'm importing and adapting the LCD and rotary encoder routines from
// my si570_2015 program. Also adding ham bands and saving the controller's
// state in EEPROM, with auto loading on boot up.

// I did a loop of 100 frequency step changes with an LCD update for each.
// One step plus display update takes 13.2 ms.
// But wait, the update associated with normal T/R switching doesn't require
// a display update. So without LCD updating, it's 9.4 ms. Still not as fast
// as I'd like. But wait again - When I'm on a frequency toggling between
// TX and RX, I don't need to calculate registers every time. I need TX and
// RX pre-calculated and just send them in response to a transition from 
// RX to TX and back.

// 1/3/2019 - I changed the I2C routines from those Hans used to the Wire
// library in hopes of speeding up a frequency change. I only got minor speed
// increase.

// 1/4/2019 Added this function: Wire.setClock(400000);
// Initial value of 100,000 didn't help but 
// going to 400,000 cut my time from 8.19
// to 2.8 ms per frequency upload.

// V1.1 is the version change which adds TX offset, RIT and so on
// As of 1/12/2020, I have added about all the features I can think of and
// tested them and fixed any bugs.

// 1/16/2020 - adding an output PIN to control a sideband select relay. Also
// adding another output PIN for key down to echo the KEY IN line, but not
// change state until after the TX registers have been sent and change back
// before the RX registers are sent.

// 3/17/2020- I continue to have problems with the newTone() library not
// working or disappearing, so I've gone back to the old built-in tone()
// function

// 9/4/2020 - I'm going to add an option to have the output frequency be
// 4 times the indicated frequency for people using a Tayloe type receiver
// with a /4 circuit at the front end. This means Jim Dunlap, WB5ZOR
// Boolean X4 will be true if this is in effect. V1.2. NOTE: As of V1.14
// X4 will be in the ADVANCED sub-menu and will be saved in EEPROM

// 3/27/2021 - V1.5 trims the extra digits when step size is changed. 
// Also, HARDWARE CHANGE - I changed out the Nano because the USB housing
// was slightly bent on #17 and when I tried to plug in I pushed the whole
// whole thing out of the socket. Had to wait for some new ones.  I put in
// #20 which programs differently:
// Programmer: “Arduino as ISP”, and MCU: ATMega328P without the 
// “old bootloader” part.

// 3/18/2021 - V1.6 Ian, G3VAJ reported that the band selection rollover on
// both above 2 meters and below 80 meters first goes to an invalid frequency
// and if you continue, to the correct one. I had my hamBand_index
// variable maximum at 11 and it should be 10 (11 items, 0 through 10)
//
// Similarly, the variable step_index declared as uint8_t failed the 
// < 0 test so I changed it to a signed char type.
//
// V1.7 of 4/28/2022, reported by Tony Bertezzolo that the 90 degree phase
// shift is lost when tuning across certain boundaries. I found from reading
// Hans Summers' paper from FDIM 2018 that if the value of dividerRX (or TX)
// sent to the chip is different from the previous one, a reset of PLL A is
// necessary to maintain the phase relationship. This version tests for that
// change and does a reset of the PLL if it occurs. I also removed some
// other reset PLL calls which are no longer needed.  The last value sent
// is now stored in dividerLast 
//
// Also, while fixing the above, I discovered that enabling RIT after having
// used it on a previous band brings back that other bands last RX frequency
// which is not desired. I've set RITSave = 0 when changing bands to prevent
// this from happening.
//
// Another change is V1.7 is to add a sotware change to give 270 degrees shift
// instead of 90 degrees. This is incorporated into the existing doLSB_USB()
// routine. This means that switching between USB and LSB won't have to be
// done in hardware.

// V1.8 of 1/18/2023 changes to rotary encoder routines to be interrupt driven. This is 
// intended to help those with 'fast' encoders by preventing missed pulses.
// Al, K2BLA is one. I did this as an experiment in my Multifunk hardware
// in a sketch called Rotary_INT.ino. Now I have to adapt that code to this
// file. 
// I also found an apparent bug where the RX frequency was being written to
// both lines after going key up and seeing we are in RIT mode. Now puts 
// RX on top and TX on bottom line.

// V1.9 starting 1/24/2023 
// I barely got V1.9 out when I started getting requests for even more 
// division. It's not surprising based on the high ppr of some of 
// these optical encoders. I'm going to try what I hope is a more logical 
// approach this time. I'll just increment a byte for each transition and
// add to the step queue when it reaches my specified number. So should be
// able to divide by any number from 1 to 255.

// V1.10, when EEPROM has sideband = upper stored, program does not
// swap to USB or CWU immediate after reading EEPROM. Need to correct.
// 4/16/2023

// V1.11, report from Ron WA7GIL that after he changes bands and presses the
// key, the frequency (on a counter) goes back to the previous band. I added
// a line to the band change routine to have it calculate TX registers for 
// the new band. Apparently the older calculated TX registers were still
// being sent to the Si5351a.

/*
  V1.14, starting 12/6/2023. I'm going to move several options from source
  code '#define' statements to EEPROM, changed from a menu option. This is
  to make it easier on users. Creating a new "advanced" sub-menu under
  the main menu.

  V1.15, starting 11/8/2024, adding control of external filter switching
  on a per-band basis, for example LPFs in a transmitter. Switching is
  via a PCF8575 IC which is controlled by I2C and can do I/O on 16 lines.
  Requested by Matt (Mateusz), SQ3MB. It will be an option, to be 
  compiled if RELAY_CONTROL is defined.

  V1.16, I have freq info going to the SI5351a every time through the loop
  for T/R reasons. Should only update when the key line changes state. I fixed
  that. Problem noted by Mateusz SQ3MB 

  V1.17, more for the above problem. I needed to include a couple more statements
  in the area that just executes when the key has gone open (key up). I also 
  added a statement to the Verbose startup report saying PCF8575: YES, when 
  the compile flag for that has been defined. 12/10/2024
  
  V1.18, add a band map (boolean bandInclude[11]) for locking out certain bands 
  from the band selection menu so user can customize what bands are shown 
  when he turns the dial. 12/11/2024

 V1.19, I realized I may want to use this VFO to drive a transmitter only, for
 instance my EICO 720. In such a case, I don't want ANY outputs in the Key Up
 state as they might QRM my separate receiver. So here comes TX_ONLY as a
 true or false option. 

	
*/ 

//       ************ What is PIND? *****************
//
// With these pin registers you can read or write a whole byte of
// pins at once, if they are set as digital inputs.

// Below, I read the PORT D pins and mask off (keep) bits 2 & 3

// encoder_now = (PIND & 0b00001100);

// This is processor dependent, not a standard C thing. There's also 
// PINB and PINC. How many pins depends on the processor. Note that with
// the N5IB hardware, I'm using ADC pins from PORTC set up as digital
// inputs for the encoder inputs.  So I should read PINC

// #include <inttypes.h> // not sure what it's for. removed 1/16/2020

// Note IC2_LCD is defined (or not) in the Si5351a_quad_config.h file

#include "Si5351a_quad_config.h"



#include <EEPROM.h>
#include <Wire.h> // NRKw

#ifdef RELAY_CONTROL
#include <Adafruit_PCF8575.h>
Adafruit_PCF8575 pcf;
#endif

//#include <NewTone.h> 



// NRK - Note that A0, A1, A2 are PORTC pins 0, 1 & 2.

#define ENC_A 3       // Encoder pin A to D3 (V1.8, was A1)
#define ENC_B 2       // Encoder pin B to D2 (V1.8, was A0)
#define LEDPIN 13	   // Internal LED, can also use for I/O NRK
#define spkrpin 9     // Pin for audio to speaker for beep
#define SW1 12		  // Pin for pushbutton PB1
#define SW2 11        // Pin for pushbutton PB2
#define TX_MODE 8 // Pin to monitor for key_down status. Reads LOW when key is down
#define TX_Out 10 // Echoes TX_MODE but changes *after* registers sent to chip
#define SB_Relay A2 // Sideband select. HIGH in LSB, LOW in USB

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
  #define EE_flag EE_step_index + 1
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



#define I2C_START 0x08
#define I2C_START_RPT 0x10
#define I2C_SLA_W_ACK 0x18
#define I2C_SLA_R_ACK 0x40
#define I2C_DATA_ACK 0x28
#define I2C_WRITE 0b11000000 // slave address is 0x60, 7 bit but here shifted
#define I2C_READ 0b11000001 // left 1 place so b0 of 8 bit field 0 is R/W bit
                            // NRK
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

	
	/*
	USERS:
  Note: I moved the definition of the crystal frequency to the CONFIG
  file to make it more accessible.

	I determined my actual crystal frequency like this:
	Listening on my K3 with the Si5351a set at 7025000 Hz I read 7024816 Hz
	So my actual frequency is 25,000,000 * 7,024,816 / 7,025,000 which gives
	me 24,999,345. Of course I could have used a more accurate standard than
	the K3 if I'd chosen to.
	Please note that while in the CW mode, the output will be shifted by the
	value of CW_pitch, so you should take your reading in the “phone” mode
	or have the key closed.
	*/


//  ** W0EB definitions ***

#define SI5351BX_ADDR 0x60   // I2C address of Si5351   (typical)
   
void si5351aOutputOff(uint8_t clk);
void Set_Freq(uint32_t frequency);


#endif //SI5351A_H

// NRK add some definitions for LCD pins



#define RS		7 // LCD RS 
#define E		6 // LCD Enable 
#define DB4		A0 // LCD DB4 V1.8, was 2
#define DB5		A1 // LCD DB5 V1.8, was 3
#define DB6		4 // LCD DB6 
#define DB7		5 // LCD DB7 

// Below, connections for I2C Arduino to Si5351a, do not have to be
// defined because they are dedicated pins in the AVR chip:

// Note A4 is for SDA
// Note A5 is for SCL



// ************     GLOBAL VARIABLES    ****************************

	unsigned long timing_start, timing_start_2;

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
 
//  bool FastStart = false; // add to V1.13 V1.14 replaced by Verbose
  bool QUAD = true; // V1.13, do provide quadrature outputs on CLK0 & CLK1
  bool TX_CLK2 = false; // V1.13 don't provide TX output on CLK2
  bool CLK0_1_OFF_KYDN = false; // V1.13 don't turn off CLKs 0 & 1 on key down
  bool TRIM = true; // V1.13 do zero less significant digits on larger step
  uint8_t DIV_FACTOR = 1; // V1.13 encoder pulses per count. 
  bool TX_ONLY = false; // V1.19, output only on key down, from CLK0, plus 1 if quadrature
  bool SPOT = false; // V1.19 toggled by PB1. When true, output is ON in key up state

  uint8_t EE_flagValue = 255; // V1.13, will be 170 after EEPROM written

 //    FUNCTION DECLARATIONS 
 
	void RotaryISR(); // V1.8 - declaration
	bool GetButtons(); // "" 
  void instructions();
  void doPitch();
  void doCWPhone();
  void showSidetone();

  uint32_t FoutRX; // Desired output frequency V1.1, have RX and TX
	uint32_t FoutTX; 
	uint32_t RITSave; // Store FoutRX here when turning RIT off, so can restore
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
	void Set_Freq(uint32_t frequency);
	void clock_0_ON();
	void clock_1_ON();
	void clock_2_ON();
  void clock_2_OFF(); // V1.14
  void clock_0_OFF(); // V1.19
  void clock_1_OFF(); // V1.19 
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
	
	// for RIT/XIT operations:
 
	uint8_t VFO_Rx_Tx = 0; // 0 = synch, 1 = RX, 2 = TX NOTE: 2 is never used,
	                       // 0 means no RIT and 1 means RIT is on
	
	// V1.7, CLK1_hi is the starting point for setting up SI_CLK1_CONTROL
	// in the clock_1_ON() routine. It was previously static at 0x4F but
	// now to invert the output for the USB/LSB selection function, it 
	// will alternate between 0x4F (not inverted) and 0x5F (inverted)
	
	uint8_t CLK1_hi = 0x4F; 
	
	// Step sizes for use when incrementing & decrement Fout
	
	// USERS:
	// Users may wish to use some other step sizes by editing those seen
	// below. For example 1, 5, 10, 50 ...  Do keep seven total though.
	
	// V1.6 with step_index as a uint8_t, the test for roll-under to 
	// negative fails because it is unsigned. A char can be used as
	// a signed or unsigned 8-bit number, so change to signed char
		 
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

// V1.18 below, create a band map to show which bands will show in the menu
// selection.  Note that 'true' means DO include the band and 'false' means
// do not include it.

// USERS: To remove any band from the menu selection list, change the 'true' to 'false'
// for that entry. See the above "Bands[]" array for the sequence.


	boolean bandInclude[11] = {true, false, true, true, true, true, true, true, 
	                           true, true, false};
// V1.15



	// Variables for formatting frequency for ASCII from DDS60_M328P.c

    char a_freq[9]; // 8-char storage for frequency in ASCII
    char a_freq_fmt[12]; // storage for 11 chars formatted freq 114,098.113
   // NOTE: Above changed from 11 to 12 for Si570 version > 100 MHz	
   
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
	
#if !defined(I2C_LCD) // V1.12
	LiquidCrystal lcd(RS, E, DB4, DB5, DB6, DB7);	
#endif

#if defined(I2C_LCD)
 LiquidCrystal_I2C lcd(0x3F, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE); // Addr, En, Rw, Rs, d4, d5, d6, d7, backlighpin, polarity
#endif

	void dispfreq1();

// Bring in some variables from my Si570 program to allow T/R frequency
// switching, offset (pitch) and RIT.

  uint16_t CW_pitch = 480; 
  uint16_t CW_pitch_ST = 480;  
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
	
	
// *****************  BORROW SOME W0EB I2C ROUTINES **************************
// Note - here I was going from Hans' method of addressing the AVR's I2C 
// registers directly to use of the Wire library, to attempt to speed up I/O

// WRITE A REGISTER NUMBER, THEN DATA BYTE *************************
// SREG IS THE AVR STATUS REGISTER [NRK]

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

// V1.15 Select relay control lines in PCF8575 based on band. I look
// at each of the 16 bits and set outputs LOW (on) for '1' bits and
// HIGH (off) for '0' bits
// NOTE: V1.15 is a simplified version - it just uses hamBand_index
// to select one pin per band

#ifdef RELAY_CONTROL

void Select_Relay(uint8_t HBindex)
{
 
  /*
  uint16_t RelayWord = Relay_bits[HBindex];
  //Serial.print("HBindex: ");
  //Serial.println(HBindex);
  for (uint8_t p=0; p<16; p++)
  {
    Serial.print("RelayWord: ");
    Serial.println(RelayWord);
    if(RelayWord & 0x01) // look at bit 0
    {
     // Serial.print("Low for p = ");
      //Serial.println(p);
      pcf.digitalWrite(p, LOW); //output will sink current if bit was set
    }
    else
    {
      pcf.digitalWrite(p, HIGH); // output will NOT sink current
    }
    RelayWord = RelayWord >> 1; // shift right to look at next bit
   // Serial.print("RelayWord shifted: ");
   // Serial.println(RelayWord);
   // Serial.print("p = ");
   // Serial.println(p);
    //Serial.print("RelayWord and 1: ");
   // Serial.println(RelayWord & 1);
  }
// while(1);
*/

// First turn all lines OFF

 for (uint8_t p=0; p<16; p++)
 {
  pcf.digitalWrite(p, HIGH);
 }

 // Next, turn ON specific line associated with the band index

 pcf.digitalWrite(Relay_lines[HBindex], LOW);

}
/*
void testPCF()
{
  lcd.clear();	
lcd.print("ALL ON");
for (uint8_t p=0; p<16; p++)
{ pcf.digitalWrite(p, LOW); } // turn all relay control output lines OFF
delay(15000);
lcd.clear();	
lcd.print("ALL OFF");
for (uint8_t p=0; p<16; p++)
{ pcf.digitalWrite(p, HIGH); } // turn all relay control output lines OFF
delay(15000);
lcd.clear();	
lcd.print("line 0 ON");

pcf.digitalWrite(0, LOW);  // turn all relay control output lines OFF
delay(15000);
}
*/
/*

// The intent of CycleBands() was to do a continuous cycling of 80 through 2 meters
// holding each for 8 seconds. It incremented hamBand_index and called doBands().
// Ihad to modify doBands() by making chosen = true rather than false, so it
// wouldn't look for user input.
void CycleBands()
{
  while(1)
  {
  for(uint8_t i=0; i<11; i++)
  {
    hamBand_index = i;
    doBands();
    Serial.print("hamBand_index = ");
    Serial.println(hamBand_index);
    delay(8000);
  }
  }
}
*/
#endif

// ************************  S E T U P ************************************



void setup()
{
	uint8_t flagValue = 0;
	
  Serial.begin(9600); // V1.15 enabled for testing
	
  pinMode(LEDPIN, OUTPUT); // NRK - on-board LED on Nano
	pinMode(spkrpin, OUTPUT); // for speaker - beep to user
  pinMode(ENC_B, INPUT_PULLUP); // For rotary encoder
  pinMode(ENC_A, INPUT_PULLUP); // For rotary encoder
	pinMode(SW2, INPUT_PULLUP);
	pinMode(SW1, INPUT_PULLUP);	
	pinMode(TX_MODE, INPUT_PULLUP); // KEY DOWN (TX) status when LOW
	pinMode(TX_Out, OUTPUT); // echoes TX_MODE after registers updated V1.1
	pinMode(SB_Relay, OUTPUT); // HIGH in LSB mode, LOW in USB mode
	digitalWrite(TX_Out, HIGH); // V1.13 - Ron Taylor reported his came up
	                            // in Key Down mode. Make sure in Key UP
								
	// V1.8, I'm going to have interrupts on both encoder pin changes but both
	// will point to the same ISR routine and IT will figure out what changed

  // V1.13 - if PB1 is held in during bootup, FastStart will cause most 
  // initial messages to be skipped for a faster startup


  //if(digitalRead(SW1)==0) FastStart = true;

	attachInterrupt (digitalPinToInterrupt (ENC_A), RotaryISR, CHANGE);  // V1.8
	attachInterrupt (digitalPinToInterrupt (ENC_B), RotaryISR, CHANGE);  // V1.8
	
   //	encoder_pins = (PIND & ENC_AB_MASK)>>2; // In V1.8, pins are sifted to <1-0>
	// 6/8/2018 avoid spurious step on start-up

	// Flash the Arduino's LED a couple times to show S/U
	
	digitalWrite(LEDPIN, HIGH);
	delay(100);
	digitalWrite(LEDPIN, LOW);
	delay(100);
	digitalWrite(LEDPIN, HIGH);
	delay(100);
	digitalWrite(LEDPIN, LOW);
	
	// Set up the LCD's number of columns and rows 
	lcd.begin(16,2); 

#if !defined(I2C_LCD) // V1.12
	lcd.display(); // turn on display
#endif

#if defined(I2C_LCD)//V1.12
	lcd.backlight();
#endif
/*
if(FastStart)
{
  lcd.print("Fast Start");
  delay(700);
}
else
{
  */
	lcd.setCursor(0,LCDBottomLine);
	lcd.print(" Si5351a Quad"); // display version number on the LCD
	lcd.setCursor(0,LCDTopLine);
	lcd.print(" WA5BDU V1.19"); // V1.2 9/4/2020 V1.5 1/11/2021 v1.6 3/18/2022
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

	delay(2500);
	lcd.clear();
	

//}
// USERS: If you for some reason need to re-initialize the EEPROM data,
// uncomment the lines below, compile the program and send to the 
// Arduino, then come back to the source code, put the comment marks
// (/-star and star-/) back in and compile and send to the Arduino 
// again.
// An easier method would be to select the SAVE STATE option from the
// menu.


	//A one-time startup write to re-initialize EEPROM information:
	/*
	FoutTX = 7025000;
  QUAD = true;
  TX_CLK2 = false;
  CLK0_1_OFF_KYDN = false;
  TRIM = true;
  DIV_FACTOR = 7;
  X4 = false;
	doSave(); // One time to initialize
*/
	
// Below, read in EEPROM values from last session, if any have been saved
// Otherwise, use default values:	

	EE_flagValue = EEPROM.read(EE_flag);

    if (EE_flagValue == 170)
    {
    lcd.print("Reading EEPROM"); // V1.19
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

    delay(1000); // V1.19
    lcd.clear();    
    }
    else
    {	
	
		FoutTX = Bands[hamBand_index];

    }   
	
		FoutRX = FoutTX; // initialize
	
 if(!digitalRead(SW1) || !digitalRead(SW2)) // IF a button is held closed during S/U, do Verbose
	{
		Verbose = true; // V1.14
    lcd.print( "VERBOSE");
    delay(200);
    while(!digitalRead(SW1) || !digitalRead(SW2)); // Stay until both released
	} 

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
    CLK0_1_OFF_KYDN = false;
    TX_CLK2 = false;
    VFO_Rx_Tx = 0;// NOT RIT mode
    CW_pitch = 0;
      key_down = false;
      if(!digitalRead(TX_MODE))
    {
      key_down = true;
      clock_0_ON();
      if(QUAD) clock_1_ON();
    }
    if(!key_down)
    {
      clock_0_OFF();
      if(QUAD) clock_1_OFF();
    }
  }
  // V1.19 assume key is UP so don't turn on CLK0/1 if in TX_ONLY mode:
  if(!TX_ONLY)
  {
	clock_0_ON(); // same	
	if (QUAD) clock_1_ON(); // same V1.12, turn on only if QUAD is true
  }
	Print_freq(FoutTX, LCDTopLine);	
	
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
  
// USERS:
// I enable the serial command below when I want the program to report info
// to me via the serial monitor. You may wish to do the same.
	
//	Serial.begin(9600); // for troubleshooting ****** TESTING ***********

	// while(1) testButtons();
	
// Test my new routines which calculate RX & TX registers separately, and send
// them to the chip separately.

/*
	calc_RX(7020000);
	send_regs_RX();
	delay(2000);
	calc_TX(7019500);
	send_regs_TX();
	delay(1000);
	send_regs_RX();
	delay(1000);
*/


// Below is a test to see how much time it takes to change Fout and update
// the LCD. Also, without the display update.
// It took 1323 ms, so one step takes 13.23 ms. 
// Time for one step without LCD update is 9.4 ms.
// I changed the method of I2C communications from direct manipulation of
// the hardware registers to use of the "Wire" library to try to speed it
// up but it only reduced the time for an update to 8.3 ms.

// Next,I'll alter the routine so it doesn't calculate the registers, it
// just sends them 100 times ...

// That didn't help much, 8.19 ms per update. So all the time seems to be
// in the TWI operations

// OK, I used the Wire.setClock() function to raise my TWI clock to 400,000 Hz
// and that got me down to 2.8 ms per frequency upload.

/*	
	Serial.begin(9600);
	
	delay(2000); // user time to turn on serial monitor
	
	timing_start = millis();
	for(uint8_t ii = 100; ii > 0; ii--)
	{
		//Fout += 10;
		send_regs_TX();
		// Print_freq(Fout, LCDTopLine);
		
	}
	Serial.print("Time in ms for 100 steps: ");
	Serial.println(millis() - timing_start);	
	
	*/
	
	// USERS: un-comment the line below to enable a routine that cycles
	// through the four output level states, 2, 4, 6 and 8 mA. The routine
	// is entered on startup. Go to the next level by pressing a button.
	// After the 4th level, normal operation resumes.  9/20/2020
	// Note that the normal level used is the highest, 8 mA.
	
	// do_levels(); // 9/13/2020 - experiment with levels settings


// V1.15

/* USERS: You can define or undefine RELAY_CONTROL in the file "Si5351a_quad_config.h"
  depending on whether you want to compile the code to control a PCF8575 IC
  */


#ifdef RELAY_CONTROL

  // Below, post serial message if PCF not found, but I won't  trap all
  //other calls to it, so it needs to be fixed or RELAY_CONTROL undefined

 // if (!pcf.begin(0x20, &Wire)) {
 //   Serial.println("Couldn't find PCF8575");
 // }
 // else
 // {
  /*
     // Serial.println("PCF8575 comm OK");
     lcd.clear();
     lcd.setCursor(0,0);
     lcd.print("PCF8575 conn.");
     pcf.begin(0x20, &Wire);
     delay(2000);
      for (uint8_t p=0; p<16; p++) 
   { pcf.pinMode(p, OUTPUT);}

  // for (uint8_t p=0; p<16; p++)
  // { pcf.digitalWrite(p, HIGH); } // turn all relay control output lines OFF
  */
  pcf.begin(0x20, &Wire);
  Select_Relay(hamBand_index);
  #endif



}



// ************************  M A I N   L O O P ****************************


void loop()
{
uint16_t loopCounter; // for testing/timing main loop
// testPCF(); // v1.15 for testing

// CycleBands(); // *** TESTING ***
/*
 * Main loop needs to do these things:
 * 
 * 1) Check for rotary encoder change and respond if any
 * 2) check switches for user input and call routine indicated
 * 3) Check for key-down line and shift output freq if necessary
 *
 */
    //delay(2000);
   // measure_time();
    //delay(10000)


    // First check the encoder pins.  To debounce, if they have changed, wait
    // specified ms and then read pins again and re-check.  If still changed,
    // do the Rotary function
	
	// V1.8 - don't think I need this forever 'while' - loop() is forever.
	
	// If abs(queue) gets larger than 10, step is increased or decreased
	// depending on the sign. The larger step is taken, queue is reduced by
	// 10, and the step size is retured to previous value.

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
		// This is after stepping in response to encoder movement:
			
		   send_regs_RX();					   
		   Print_freq(FoutRX, LCDTopLine);
		   
	}

	
// Now check for key down condition
// In V1.16, I make sure that the "key up" actions only occur the one time
// that the keyed line has returned to open


  if(!digitalRead(TX_MODE))
  {
    key_down = true;

     clock_0_ON(); // V1.19
     if(QUAD) clock_1_ON(); // V1.19

			// key_down = true; // V1.16
			digitalWrite(LEDPIN, HIGH);
			send_regs_TX(); // change output frequency 
			digitalWrite(TX_Out, LOW); // echo keyed line AFTER registers updated
			
			if(sidetone) tone(spkrpin, CW_pitch_ST); // V1.3 create sidetone when key down V1.19 add _ST

			// V1.12 If using CLK2, turn off RX output to CLK0 and 1
			// during key down if flag is true
			
			if(TX_CLK2 && CLK0_1_OFF_KYDN)
			{
			si5351aOutputOff(SI_CLK0_CONTROL);
			si5351aOutputOff(SI_CLK1_CONTROL);
			}
			if(TX_CLK2) clock_2_ON(); // V1.12
			
			while(!digitalRead(TX_MODE)); // stay here until key returns to up
		
// Below actions take place after key goes back UP
			
			key_down = false; // V1.4
			digitalWrite(LEDPIN, LOW);
			digitalWrite(TX_Out, HIGH); // raise keyed line BEFORE regs sent
			send_regs_RX(); // change output frequency 
			noTone(spkrpin); // turn off sidetone
			
      if(TX_ONLY)
      {
      clock_0_OFF(); // V1.19
      if(QUAD) clock_1_OFF(); // V1.19
      }

  // } // V1.16 move brace to V1.17 marker below ...

			if(TX_CLK2) si5351aOutputOff(SI_CLK2_CONTROL); // V1.12 turn clock 2 off
			
			// V1.12 - NOW, if CLK2 is in use and CLK0 was turned off during key down
			// turn it back on. Then, if CLK1 is not turned off by QUAD = false, then
			// turn it back on too.
			
			if(TX_CLK2 && CLK0_1_OFF_KYDN)
			{
				clock_0_ON();
				if(QUAD) clock_1_ON();
			}

  } // V1.17 above two actions should only happen when key goes up

// V1.12 - I'm not sure I like the idea of contact bounce delay, 
// but I moved it to the end of the routine so it wouldn't delay 
// actions. Users who do not use mechanical keys may want to change 
// to 1 ms or eliminate it.
			
			delay(2); // 2 ms of contact bounce wait ...				
				


        // Toggle LED every 20000 times through loop to show Arduino is
        // alive

        if(loopCounter++ > 50000)
        {
            loopCounter = 0;
            digitalWrite(LEDPIN, !digitalRead(LEDPIN));      
			// 6/82018 - TURN OFF DISTRACTING LED FOR NOW
			// I THINK MY LOOP TIME IS ABOUT 6 uS
        }
        

        if (GetButtons())
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
				if(VFO_Rx_Tx == 0)RIT_ON();	
				
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
						Go_RIT();
						Print_freq(FoutTX, 0);
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
        
    }		
}



// ******************** END OF L O O P FUNCTION **************************



// Menu is entered when the user Holds PB3. LCD shows current choice
// & user can rotate the knob to cycle through choices. Pressing PB3
// takes the current choice, which goes to its sub-menu of choices.

// There are now 7 choices, 0 through 6

	void menu()
	{
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


		{   
        r_dir = low_rez(); // get UP, DOWN, or NONE from rotary encoder
        while (r_dir) // If zero, no encoder motion so skip this ...
        {
      if (r_dir == 1)
      {
      Menu_itemA++; //  0, 1, 2, 3, 4, 5, 6, 7 are valid
      if (Menu_itemA > 7) Menu_itemA = 0; // V1.19 was > 6
      }
      else if(r_dir == 2)
      {
        if(Menu_itemA == 0) Menu_itemA = 7; // V1.19 was 6
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
      delay(1000);
    }
		else if (Menu_itemA == 1)
        {
      TX_CLK2 = !TX_CLK2;
      showChoiceA();
      delay(1000);
    }
		else if (Menu_itemA == 2)
        {
      CLK0_1_OFF_KYDN = !CLK0_1_OFF_KYDN;
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
        showChoiceA();
        delay(1000);
      }

		//else if (Menu_itemA == 6) doAdvanced(); // V1.14 
    // item '7' falls through, resulting in EXIT
	
	}

// Show Advanced menu currently selected item:
// Show the name of the parameter and its current value

void showChoiceA()
	{
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
			
		else if (Menu_itemA == 7)  // V1.14 was 5 V1.19 was 6
        {
            lcd.print("EXIT");
        }				
	}

  
  
  void doDivFac()
  {
    /*
    Here I look for encoder counts and adjust DIV_FACTOR up or down in
    response to them. Then update the LCD with each change. Keep within
    1 and 255. It's a uint8_t, so I just have to avoid 0.
    Outsie this loop, look for any press of a PB, which will accept the
    current value and terminate this routine.
    */

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
		clearLine1();
		lcd.print(TuningStepSize[step_index]);	
	}

// 3/18/2021 - V1.6 Ian, G3VAJ reported that the band selection rollover on
// both above 2 meters and below 80 meters first goes to an invalid frequency
// and if you continue, to the correct one. I had my hamBand_index
// variable maximum at 11 and it should be 10 (11 items, 0 through 10)


	void doBands()
	{
	
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
		Print_freq(FoutTX, LCDTopLine);

// V1.15, if PCF8575 is in use, set the output lines to match the band
// *bk4
#ifdef RELAY_CONTROL
  Select_Relay(hamBand_index);
  #endif 
  }

	

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

        EEPROM.put(EE_FoutTX, FoutTX);
        EEPROM.put(EE_hamBand_index, hamBand_index);
        EEPROM.put(EE_step_index, step_index);
    //    EEPROM.put(EE_mode, quad);
        EEPROM.update(EE_flag, 170); // save specific # 0b10101010 to show EPROM written
		EEPROM.put(EE_LSBMode, LSBMode); 
		EEPROM.put(EE_modeCW, modeCW); 
		EEPROM.put(EE_CW_pitch, CW_pitch); 
		EEPROM.put(EE_sidetone, sidetone); // boolean value
    EEPROM.put(EE_QUAD, QUAD); // These six added V1.14
    EEPROM.put(EE_TX_CLK2, TX_CLK2);
    EEPROM.put(EE_CLK0_1_OFF_KYDN, CLK0_1_OFF_KYDN);
    EEPROM.put(EE_TRIM, TRIM);
    EEPROM.put(EE_DIV_FACTOR, DIV_FACTOR);
		EEPROM.put(EE_X4, X4);
    EEPROM.put(EE_TX_ONLY, TX_ONLY); // V1.19

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
    uint16_t ExtraTime = 0; // V1.19
    if(Verbose) ExtraTime = 1000;
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
	}

  }

	/*
  Long instructions, called from menu and scrolled with encoder
  */

    void long_instr()
    {
      
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

//
// Set CLK0 output ON and to the specified frequency
// Frequency is in the range 1MHz to 150MHz
// Example: Set_Freq(10000000);
// will set output CLK0 to 10MHz
//
// This example sets up PLL A
// and MultiSynth 0
// and produces the output on CLK0
//

// Display the Si5351A CLK0/1 frequency on the LCD
// bottom line to 1 Hz resolution. Not used as of 6/8/2018

	void dispfreq1()
	{

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


void Set_Freq(uint32_t frequency)
{
	FoutTX = frequency;
	calc_TX(FoutTX);
	send_regs_TX();
}



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
		// i2cSendRegister(SI_CLK0_CONTROL, 0x4F | SI_CLK_SRC_PLL_A); // NRKw to below
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
		
		i2cWrite(SI_CLK1_CONTROL, (CLK1_hi | SI_CLK_SRC_PLL_A) & level); // NRKw
	}

		void clock_2_ON() // V1.12 added
	{
		
		i2cWrite(SI_CLK2_CONTROL, 0x4F | SI_CLK_SRC_PLL_B); // V1.5: PLL_B
	}

  void clock_2_OFF() // V1.14
  {
  si5351aOutputOff(SI_CLK2_CONTROL); // turn clock 2 off
  }


  void clock_0_OFF() // V1.19
  {
  si5351aOutputOff(SI_CLK0_CONTROL); // turn clock 0 off
  }


  void clock_1_OFF() // V1.19
  {
  si5351aOutputOff(SI_CLK1_CONTROL); // turn clock 1 off
  }  

// *********************** STEP UP AND STEP DOWN FREQUENCY ******************
 
 // Step the frequency up or down by current step value
 // V1.1 - in the Si5351 version, the stepping routines will also calculate
 // the new register values for RX and TX as appropriate

	  void step_up()
	 {
      //uint32_t adj_step_size;
      //adj_step_size = TuningStepSize[step_index];
	  FoutRX += TuningStepSize[step_index]; // always step RX
	  calc_RX(FoutRX);
	  
	  if(!VFO_Rx_Tx) // also step TX if in synch mode
	  {
		  FoutTX += TuningStepSize[step_index];
		  calc_TX(FoutTX);
	  }
	 }

	  void step_down()
	 {
      //uint32_t adj_step_size;
      //adj_step_size = TuningStepSize[step_index];
      FoutRX -= TuningStepSize[step_index];
	  calc_RX(FoutRX);
	  
	  if(!VFO_Rx_Tx) // also step TX if in synch mode
	  {
		  FoutTX -= TuningStepSize[step_index];
		  calc_TX(FoutTX);
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

static void Print_freq(uint32_t f32, uint8_t line)

	{
		//DoubleBeep(); // ************ TEST
		uint8_t length, i;
		uint8_t dest_point = 10; // end of dest field  *** WAS 9
    char T_R_CHAR = 'R';
		
// V1.4: If I'm printing to line 0 (upper), I'll put an 'R' prefix in pos. 0
//       If line 1 (lower), I'll put a 'T' there.		

// V1.19 skipt the R and/or T if in TX_ONLY mode NO - I'm going to put in an * to 
// show I'm in TX_ONLy mode

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

// I need to fill the destination field with space chars or I can have a
// null in the first position if my frequency doesn't have a 10 MHz digit

	for (i=0; i<11; i++) // *** 11 WAS 10
	{
		a_freq_fmt[i] = ' ';
	}

	a_freq_fmt[11] = 0; // just in case the compiler doesn't initialize this
	                    // NULL terminator

// Initially I'll copy the whole thing to a_freq_fmt with comma & decimal

		length = (uint8_t) strlen(a_freq);
		--length; // make zero based

		for (i=3; i!=0; i--)
		{
			a_freq_fmt[dest_point--] = a_freq[length--]; // move fract kHz
		}

		a_freq_fmt[dest_point--] = '.'; // add a decimal point

		
		for (i=3; i!=0; i--)
		{
			a_freq_fmt[dest_point--] = a_freq[length--];
		}
		
		a_freq_fmt[dest_point--] = ','; // add the comma

		while (length !=0xFF)
		{
			
			a_freq_fmt[dest_point--] = a_freq[length--];
		}

		lcd.setCursor(1, line);// V1.4 changed 0 to 1
		lcd.print(a_freq_fmt);
	
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
		
		gohome:;
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
			digitalWrite(SB_Relay, HIGH);
			CLK1_hi = 0x4F; // for LSB, V1.7
			
		}
		else digitalWrite(SB_Relay, LOW);

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
			Print_freq(FoutRX, 0);
		}
		Print_freq(FoutTX, 1);
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
		Print_freq(FoutTX, 0);
	}
		
// Swap TX and RX frequencies when in RIT mode

	void TXRXSwap()
	{
		uint32_t holder;
		holder = FoutTX;
		FoutTX = FoutRX;
		FoutRX = holder; // Now they're swapped ...
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
    clock_0_ON();
    if(QUAD) clock_1_ON();
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
			Print_freq(FoutTX, 0); // Here in sync mode
		}
		if(VFO_Rx_Tx == 1)
		{
			Print_freq(FoutRX, 0); // In RIT, RX freq to top line
			Print_freq(FoutTX, 1); // In RIT, Fbase is TX freq - to bottom
		}
		printSBSelected();
		ShowStepFast();
		// SingleBeep(); // TESTING *********************

	}

// speaker beeps to go with PB closures ...

	void SingleBeep() // A beep of 40 WPM speed
	{
		delay(20);
		tone(spkrpin, CW_pitch_ST, 30); // V1.19
		//delay(30);
		//noTone(spkrpin);
		delay(20);
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

	void printSBSelected()
	{
		lcd.setCursor(13,1); // bottom line, far right
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

// The doPitch routine will display the pitch and vary it in 10 Hz steps
// using the encoder. A maximum of 1000 Hz and minimum of 150 Hz is enforced
//
// After each step, the Si570's registers are updated with the new offset value
// This should allow hearing the effect on pitch when using a receiver 
// correctly tuned to a station. v1.4b

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
		
		if (r_dir) showPitch();
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
	}

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