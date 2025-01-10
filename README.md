# Si5351a_quad
A VFO with quadrature outputs based on the Si5351a frequency synthesizer <br>
This project contains source code for the synthesizer project. It is for the Arduinos based on the ATmega328 MCUs, such as the Nano or UNO. 
Written by Nick Kennedy, WA5BDU<p>
V1.18 of 12/18/2024 makes these changes:
A bug caused frequency information to be sent to the Si5351a continuously via I2C instead of only when changed. V1.16 and V1.17 corrected this problem. In some systems, the bug might not be noticed while in others, the continuous I2C activity could be heard as noise in the receiver.
V1.18 added the ability for the user to omit any of the 11 bands from the band selection function, skipping those not used in your design. The manual describes finding the line in source code and how to do the edit before sending the program to the Arduino.
This update also includes the updated user manual and schematic.
