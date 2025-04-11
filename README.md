# Si5351a_quad
A VFO with quadrature outputs based on the Si5351a frequency synthesizer <p>
This project contains source code for the synthesizer project. It is for the Arduinos based on the ATmega328 MCUs, such as the Nano or UNO. 
Written by Nick Kennedy, WA5BDU</p>
<p>V2.2 added 4/11/2025. My Keyer program has been merged with the program. It is a full-featured keyer well integrated into the program. <br>
The other major addition is an "HW-8 Mode" compilation option where the program is customized for use with the HW-8 transceiver. <br>
<br>NOTE: V1.19 adds another parameter to those saved in EEPROM. To insure that this location exists, one should execute SAVE STATE from the menu after loading V1.19. <p>
V1.19 of 1/10/2025 adds a mode called TX ONLY. This allows using the synthesizer as a simple VFO driving a transmitter. Output exists only
when the key is closed, but there is a SPOT function to turn output On and Off again for spotting. There is no output from any clock when the key is open.<br>
V1.19 also changes the function of the PB2 Tap action. Formerly it alternated betwewen 10 and 100 Hz step sizes. Now it steps through 10, 100 and 1000 Hz step sizes. The object is to allow faster large QSY without going through the menu.<p>
V1.18 of 12/18/2024 makes these changes:<br>
A bug caused frequency information to be sent to the Si5351a continuously via I2C instead of only when changed. V1.16 and V1.17 corrected this problem. In some systems, the bug might not be noticed while in others, the continuous I2C activity could be heard as noise in the receiver.<br>
V1.18 added the ability for the user to omit any of the 11 bands from the band selection function, skipping those not used in your design. The manual describes finding the line in source code and how to do the edit before sending the program to the Arduino.
This update also includes the updated user manual and schematic.<p>

