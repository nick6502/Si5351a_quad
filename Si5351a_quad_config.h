/*
As of V1.13, this configuration file still exists, but most options
will now be selected from a configuration menu at run time and stored
in EEPROM. This should be easier for the user. However, some options
still need to be chosen by editing source code in this file. Instructions
precede each line to potentially be edited.

*/

// USERS: Below is the I2C address of the Si5351a device

#define SI5351BX_ADDR 0x60   // I2C address of Si5351   (typical)

// USERS: Below is the I2C address for the PCF8575 realy control IC,, when used

#define PCF8575_I2C_ADR 0x20

// **********  BELOW - CHOSE I2C OR PARALLEL INTERFACE FOR LCD *************


// USERS: The line below (#define I2C_LCD) should exist if you are using an LCD
// with I2C interface. If you are using one with a parallel interface, delete
// or comment out that line.  
// The program will then select which LCD library to include
// Note that when using the I2C version, you can also change the I2C address if
// is something other than 0x3F shown below

#define I2C_LCD 1  

#if I2C_LCD
#include <LiquidCrystal_I2C.h>
#define LCD_I2C_ADR 0x3F
#else

#include <LiquidCrystal.h>
#endif


// **************************************************************************

// ********** BELOW: WHEN YOU INITIALLY START USING THE PROGRAM *************
// ********** YOU MAY WANT TO DETERMINE THE ACTUAL FREQUENCY OF THE *************
// ********** OF YOUR SI5351'S CRYSTAL AND ENTER BELOW TO CORRECT ***********
// ********** DISPLAY INACCURACY.                               *************

// USERS:
// Your Si5351a board may have a crystal frequency of 27 MHz or 25 MHz or
// possibly others. If your frequency output is significantly off, you may
// need to change the #define XTAL_FREQ line to suit. 
// Also you can fine tune or calibrate your device. See the user comment below.

	// #define XTAL_FREQ 27000000 // Crystal frequency for Hans' board
	// #define XTAL_FREQ 25000000 // NRK - Adafruit board is 25 MHz
	
	//#define XTAL_FREQ 24999345 // my specific Si5351a board
  #define XTAL_FREQ 25001301 // This one is for my development lash-up

	/*
	USERS:
	I determined my actual crystal frequency like this:
	Listening on my K3 with the Si5351a set at 7025000 Hz I read 7024816 Hz
	So my actual frequency is 25,000,000 * 7,024,816 / 7,025,000 which gives
	me 24,999,345. Of course I could have used a more accurate standard than
	the K3 if I'd chosen to.
	Please note that while in the CW mode, the output will be shifted by the
	value of CW_pitch, so you should take your reading in the “phone” mode
	or have the key closed.
	*/

// ****** ENTER INTERMEDIATE FREQUENCY FOR SUPERHET *************************
	
// ********** BELOW: YOU CAN USE THE CONTROLLER WITH A SUPERHET   ***********
// ********* RECEIVER. ENTER THE REQUIRED I.F. OFFSET BELOW AND *************
// ********* IT WILL BE ADDED TO THE DISPLAYED RECEIVE FREQUENCY ************	
	
	// *** INTERMEDIATE FREQUENCY OFFSET ***  V1.12
	// This value will be added to the receive frequency at the time the
	// registers are calculated. To NOT use an offest, simply make it equal
	// to 0. 
	
#define IF_OFFSET 0 // enter in Hertz, no commas or decimal, 0 for none

// ************ USE OF THE PCF8575 IC FOR RELAY SELECTION:  ******************

// USERS: COMMENT OUT OR DELETE LINE BELOW IF PCF8575 RELAY SELECTION IS NOT USED

// #define RELAY_CONTROL // When defined, do compile the PCF8575 code V1.15

#ifdef RELAY_CONTROL

// A table by band to control relay selection. 

 

 // USERS: The numbers in the array below designate the lines used for each
 // band, in the sequence 80 ,60, 40, 30, 20, 17, 15, 12, 10, 6, 2.  If you 
 // want one output (filter) to cover two bands, you should edit the array.
 // For examnple, if the third filter is to cover both 40 and 30 meters,
 // you could edit the array to:
 // {0, 1, 2, 2, 4, 5, 6, 7, 8, 9, 10 }

  uint8_t Relay_lines[11] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10 };
  
#endif

// USERS: If you install a pot for CW speed control, the statement below
// should not be commented out. If you do not use a speed pot, you should
// place comment marks ('//') in front ot the #define statement.

#define PotExists 1

// V2.0i add VFO coverage for HW8 as an option

// ***************************** HW-8  HW-8  HW-8 **************************

//#define HW8

//#define SERIALINFO // Says, do print out information to serial monitor on startup

#define SERIALINFO // Tell program to send some parameters to serial monitor on startup