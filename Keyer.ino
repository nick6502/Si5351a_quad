
   
   
// **********************************************************************************
//                                                                                  *
//                              M A I N  x L O O P                                  *
//                                                                                  *
// **********************************************************************************

// xloop() is called at the start of every loop() execution

	void xloop()
	{
		
		
		#ifdef PotExists	// compile below only if pot exists

		loopCount++; // Count number of passes through main loop, 255 max
		
// V2.0f If not command mode and loopCount has turned over, then
// check on returning to pot control if it has moved. Otherwise
// just check the pot and set the speed.

		if(!CMNDMode && !loopCount)
		{
      if(!SpeedPot) freeSpeedPot(); // if pot moved >= 3 WPM re-enable
      else readSpeedPot(); // Every 256 times through main loop ~ 10 ms				
		}
		#endif
		
	/* V2.0f -- I was doing this twice for some reason ...
    #ifdef PotExists

		if(SpeedPot && !CMNDMode && !loopCount)
		{
			readSpeedPot(); // Every 256 times through main loop ~ 10 ms
		}

  #endif
    
    */

    // Check for either paddle closed ...
	

  if(digitalRead(DotContact)==LOW || digitalRead(DashContact)==LOW)
  {
    KeyerActive = true;      
    if(!printedKYR) printSBSelected();
    timing_keyer = millis();
    
  }
  else // here, no paddle closed so see if timing end of KeyerActiver mode
  {
    if(KeyerActive)
    {
    if(millis() - timing_keyer > KYR_hold) 
    {
      KeyerActive = false;
      printSBSelected(); // restore field to former use
    }
    }
  }

		// Below is the place in the main loop where we actually read the
		// paddles and create a dot or dash if demanded
	
		checkPaddles();
		if(SpLatchDit) PspaceDit();
		if(SpLatchDah) PspaceDah();		
			if(DitLatch) Pdit();
		if(SpLatchDah) PspaceDah();
		if(SpLatchDit) PspaceDit();		
			if(DahLatch) Pdah();

	
		if(newchar != 0)  // if not 0, a paddle-sent character is ready
		{
			Serial.write(backmorse[newchar]);
			ProcessChar();
      if(MorseToLCD)
      {
      //#ifndef I2C_LCD // if parallel interface
      //if(!LCDmode) pins4LCD(); // make pins outputs for LCD
  
      //#endif
      shiftMorseLeft();
      MorseField[15] = backmorse[newchar];
      lcd.setCursor(0,LCDBottomLine);
      lcd.print(MorseField); // V2.0e, 16 character field
      //#ifndef I2C_LCD // if parallel mode
    //if(LCDmode) pins4paddle();
    //#endif
      }
			//Serial.print(newchar, BIN);
			//Serial.println();
			newchar = 0;
			
			// Now, end of character has been detected. If six more spaces
			// occur with no paddle action, that's a word space. Note that
			// timing_start hasn't been reset yet ...
			
			while((millis() - timing_start) <= wordspace) 
			{
				newchar = 1;	// 1 will map to ASCII space   
				if(!digitalRead(DotContact) || !digitalRead(DashContact))
				{
					newchar = 0; // paddle action? Quit loop and set
					goto beyond; // newchar to 0
				}
			}
			beyond:;
		}
		
	if(DoLockout) LockOut();
		
  
}	 // END OF XLOOP()

	// I think Pdit() and Pdah() are for elements sent
	// from the paddle
	
	void Pdit()
	{
		char_done = false;
		// if(controlTR && !TRrelayON) closeTR(); // close T/R relay if rqd  // V0.1
		timing_start = millis();
		
		if(DoTransmit) KeyDownActions(); // V2.0
    else
    {
		if(SideTone) tone(spkrpin, CW_pitch); // produce tone V2.0 CW_pitch was pitch - let 'actions' do this
    //digitalWrite(LEDPIN, HIGH); // V0.1  
    }

		DitLatch = false; // clear if was latched
		//digitalWrite(LEDpin, HIGH); // V0.1
		buildchar = buildchar << 1; // shift in 0 for a dit
		DeadMan--;
		
		// Now while sending dit, if dash paddle closes, latch it
		
		while( (millis() -  timing_start) <= ditlen)
		{
			if(digitalRead(DashContact)==LOW) DahLatch = true;
		}
		
		// Now dit has timed out, dash may or may not be latched, time for
		// a space ...
		
		SpLatchDit = true;
		if(ModeA) DahLatch == false;
		
		//timingTXHold = millis(); // reset with each element generated // V0.1
		
	}


	void Pdah()
	{
		char_done = false;
		//if(controlTR && !TRrelayON) closeTR(); // close T/R relay if rqd // V0.1
		timing_start = millis();
		if(DoTransmit) KeyDownActions(); // V2.0
    else
     {
		if(SideTone) tone(spkrpin, CW_pitch);
 		 //digitalWrite(LEDPIN, HIGH); // V0.1   
     }


    //}
		DahLatch = false; // clear if was latched
		buildchar = (buildchar << 1) + 1; // shift in 1 for dash
		DeadMan--;
		
		// Now while sending dash, if dit paddle closes, latch it
		
		while( (millis() -  timing_start) <= dahlen)
		{
			if(digitalRead(DotContact)==LOW) DitLatch = true;
		}
		
		// Now dash has timed out, dit may or may not be latched, time for
		// a space ...
		
		SpLatchDah = true;
		if(ModeA) DitLatch = false;
		// timingTXHold = millis(); // reset with each element generated // V0.1
	}

// Creating separate routines for space following dot and space following
// dash. This is so I can delay latching same element but allow opposite
// element.

// V2.0 substitute KeyUP and DN actions for actions taken here because
// I'm missing my frequency shift as-is.  1/30/2025

// **************  SPACE FOLLOWING DOT ELEMENT ****************************


	void PspaceDit()
	{
		timing_start = millis();
		//digitalWrite(TX_Out, keyUPstate); // open keyed line
		//if(SideTone) noTone(spkrpin); // stop tone
    KeyUpActions();
		SpLatchDit = false; // clear if was latched
		//digitalWrite(LEDpin, LOW); // V0.1
		if(DeadMan == 0) DoLockout = true;
		
		// The purpose of the delay below is to prevent latching a new
		// element at the very start of the space period. The user needs
		// a little time to get his finger off the paddle, at least I do.
		// This is what I did with my Atari keyer way back around 1987 or
		// so.
		// As of 6/10/2019, I'm trying 'halfspace' equal to about 75% of
		// a space, mimicking what the K3, KX3 and FT-991A do.
		
	while( (millis() -  timing_start) <= halfspace)
		
		{
			// in this 1st half of space, I allow latching dash but not dot
			
			if(digitalRead(DashContact)==LOW) DahLatch = true;
		}
		
		// In 2nd half of space, either ele can latch if nothing latched yet
		// Since I haven't reset timing_start, I can continue to count to
		// full ditlen
		
	while( (millis() -  timing_start) <= ditlen)
		
		{		
			checkPaddles();
		}	

	// Now space has completed, new ele may or may not be latched
	// If nothing is latched, current char is finished
	// This is also where I'll start timing for T/R relay dropout
	// if we are doing T/R switching
	
	if(!DahLatch && !DitLatch)
		{
			newchar = buildchar; // save for whatever use
			buildchar = 1; // ready for new build and flag done
			DeadMan = 255; // reset count whenever both paddles open
			char_done = true;
			//if(controlTR) timingTXHold = millis() + ditlen;
		}
		
		// timingTXHold = millis(); // reset with each space generated // V0.1
	
	}

// ************  SPACE FOLLOWING DASH ELEMENT *****************************
	
	void PspaceDah()
	{
		timing_start = millis();
		//digitalWrite(TX_Out, keyUPstate); // open keyed line
		//if(SideTone) noTone(spkrpin); // stop tone
    KeyUpActions();
		SpLatchDah = false; // clear if was latched
		// digitalWrite(LEDpin, LOW); // V0.1
		if(DeadMan == 0) DoLockout = true;		
		// The purpose of the delay [same as above comment]
		
	while( (millis() -  timing_start) <= halfspace)
		
		{
			// in this 1st half of space, I allow latching dot but not dash
			
			if(digitalRead(DotContact)==LOW) DitLatch = true;
		}
		
		// While doing 2nd half of space, either ele can latch if no latch
		
	while( (millis() -  timing_start) <= ditlen)
		
		{
			
			checkPaddles();
		}	

	// Now space has completed, new ele may or may not be latched
	
	// If nothing is latched, current char is finished
	// This is also where I'll start timing for T/R relay dropout
	// if we are doing T/R switching	
	
	if(!DahLatch && !DitLatch)
		{
			newchar = buildchar; // save for whatever use
			buildchar = 1; // ready for new build and flag done
			DeadMan = 255; // reset count whenever both paddles open
			char_done = true;
			//if(controlTR) timingTXHold = millis() + ditlen;
		}
	
		// timingTXHold = millis(); // reset with each space generated // V0.1
	}



// *************  READ PADDLES ********************************************

	
	void checkPaddles()
	{
    //#ifndef I2C_LCD // if parallel mode
    //if(LCDmode) pins4paddle();
   // #endif

		
		if(!DitLatch && !DahLatch)
		{
			if(digitalRead(DotContact)==LOW) 
			{
				DitLatch = true; // 'P' distinguish from dit and

			}
			else 
			{
				if(digitalRead(DashContact)==LOW) DahLatch = true; // dah of character sender
			}
			
			// if(DitLatch || DahLatch) battTimer = millis(); // reset  timer // V0.1

		}
		//}
		
	}



	void ProcessChar()
	{
	bool changed = false; // Used in TX Line and ...
	
		// if(MsgActive) goto gotX; // have XChar from message send routine // V0.1
		
		XChar = backmorse[newchar];
		
		

		if(XChar == ' ') goto checkout; // ignore space character
		
			
		if(!CMNDMode && (XChar != '*')) goto checkout; // no CMND MODE
		if(!CMNDMode && (XChar == '*')) // enter CMND MODE
		{
			CMNDMode = true;
			CMNDChar = 'Z';
			// digitalWrite(CMDled, HIGH); // V0.1
			SideToneOld = SideTone;
			SideTone = true;
      
			//if(!MsgActive)
			//{
			delay(125); // DoubleBeep comes too fast ...
			DoubleBeep(); // acknowledge in command mode
			//}
      

			DoTransmitOld = DoTransmit; // save so can restore
			DoTransmit = false; // not sure why I needed this ...
		
			if(speed > CMND_SPEED)
			{
				speed_old = speed;
				speed = CMND_SPEED; // command speed, slow it down
				morseSetSpeed();
			}
			DoTransmit = DoTransmitOld; // V2.1e added
			goto checkout;
		}
	
	// Above code mainly concerned with finding '*' and getting into 
	// command mode. 
		
	// Below,keyer is in CMNDMode or should be, so check to see if first
	// char called CMNDChar has been received. If not, the current char
	// becomes CMNDChar. Commands that need more characters will exit and
	// the next time(s) through will get those chars. Commands that execute
	// after a single character (the CMNDChar) can do their action right here
	// (might call a function) and finish it. Those commands are H, A, B,
	// & P. Commands that need 1 or more additional chars are S, T, M, R.
	
		if(CMNDMode && (CMNDChar == 'Z')) // receiving 1st char of cmnd seq
		{
			CMNDChar = XChar;
			if(XChar == 'S') // Speed entry command
			{
				CMND_SEQ_ptr = 0;
				//MessageID = 0;
				goto checkout;

			}
    

      if(XChar == 'D') // add command V2.0f, V2.0i fixed == was =
      {
        MorseToLCD = !MorseToLCD;
        if(MorseToLCD)
        {
         // #ifndef I2C_LCD
         // if(!LCDmode) pins4LCD();
          //#endif
          lcd.setCursor(0, LCDBottomLine);
          clearLine1();
          lcd.write("Morse to LCD");
          delay(1000);
         // #ifndef I2C_LCD // if parallel mode
          //if(LCDmode) pins4paddle();
          //#endif
        }
        else
        {
          DoTransmitOld = DoTransmit;
          DoTransmit = false;
          delay(600);
          send_code('R');
          DoTransmit = DoTransmitOld;
          UpdateScreen(); // Restore LCD info V2.0f
        }
        goto goodexit;
      }
			if(XChar == 'H') HandKey();
			
      if(XChar == 'O') // On/Off air toggle, V2.0 1/30/2025
      {
        OnAir = !OnAir;
      delay(600); // give user time to think
       // DoTransmitOld = DoTransmit;
			//	DoTransmit = false; // don't send on the air
				if(!OnAir)
        {
				send_code('N');
        digitalWrite(TX_Out, keyUPstate);
				lcd.setCursor(0,LCDBottomLine); // V2.1f
				lcd.print(" OFF AIR");
				LCDFixTime = millis(); 
				LCDtrash = true;
        }
				else if(OnAir)
        {
				send_code('Y');
        }
				//DoTransmit = DoTransmitOld; // restore previous

      goto goodexit;
      } // end of 'O' command

			if(XChar == 'A')
			{
				ModeA = true;
				morseSendString(" OK A");
				Serial.println("\nMode A");
				goto goodexit;
			}
			if(XChar == 'B')
			{
				ModeA = false;
				morseSendString(" OK B");
				Serial.println("\nMode B");
				goto goodexit;
			}

			
			if(XChar == 'P') // Adjust pitch
			{
				PitchSet();
				Serial.println("\nPitch: ");
				Serial.print(CW_pitch, DEC); //  CW_pitch was pitch
				goto goodexit;
			}
			
      

			if(CMNDChar == 'T')
			{
				goto checkout;			
			}
			
			rasp(); //No valid entry
			
			goodexit:;
			ExitCMND(); // invalid entry cancels cmnd mode
		}
		
		// Above code was for capturing the first (or sometimes) only 
		// command code. For multi-char commands, we look below for
		// additional character(s).
		
		// Next, we process S for Speed command. After 'S', we accumulate
		// two numeric characters
		// Note at this point I know we are in CMND mode and valid CMNDChar
		// has been accepted
		
		
		if(CMNDChar == 'S')
		{
			
			// At this point I expect to receive two numeric digits
			// Note that if the user sends 'P' for either digit we cancel
			// the speed change and go back to using the speed pot. 
			// A 'Q' also cancels the function.
			
			/* No longer needed - now we automatically re-enable pot
			
			if(XChar == 'P')
			{
				SpeedPot = true;
				ExitCMND();
				// rasp();
				goto checkout;
			}
			*/ 
			
			if(XChar == 'Q') // 'Q' is quit cmnd to exit speed after starting
			{
				ExitCMND();
				
				rasp();
				goto checkout;
			}
			
			if(CheckNumeric(XChar))
			{
				CMND_SEQ[CMND_SEQ_ptr] = XChar;
				CMND_SEQ_ptr++;
				if(CMND_SEQ_ptr == 2) 
				{
					DoSpeedChange(); // I know I've captured two numerals
					morseSetSpeed();
					delay(200);
				//	if(!MsgActive) send_code('R'); // acknowledge good result // V0.1
					ExitCMND();
				}
			}
			else // here, numeric check failed so try again
			{
				// if(!MsgActive) bad_sound(); // V1.1a - don't play in message // V0.1
			}
		} // End of SPEED logic
		
		

 

// V2.0 attempt to bring 'T' command back ...

		 // V0.1

		if(CMNDChar == 'T')
		{
			changed = false;
			if(XChar == 'L')
			{
				keyDNstate = LOW;
				keyUPstate = HIGH;
				changed = true;
				morseSendString(" TX LO");
				Serial.println("\nTX logic LOW");
			}
			if(XChar == 'H')
			{
				keyDNstate = HIGH;
				keyUPstate = LOW;
				changed = true;
				morseSendString(" TX HI");
				Serial.println("\nTX logic HIGH");
			}
				
			if(!changed)
			{
				bad_sound();
			}
			
			ExitCMND();
		}		
	
  
	
		checkout:;
	}

// ***************  END OF PROCESS CHAR ROUTINE ***************************

	
	void ExitCMND()
	{
		CMND_SEQ_ptr = 0;
		CMNDChar = 'Z';
		CMNDMode = false;
		// digitalWrite(CMDled, LOW); // V0.1
		DoTransmit = DoTransmitOld;
		SideTone = SideToneOld;
    speed = speed_old; // restore speed V2.1b
		//MessageID = 0; // V0.1
		// Record = false; // V0.1
	}
		
	bool CheckNumeric(uint8_t x)
	{
		if(XChar == 'T') XChar = '0'; // Allow dash for zero
		if((XChar >= '0') && (XChar <= '9')) return true;
		else return false;
	}

// ********  SPEED CHANGE COMMAND FROM PADDLE *****************************
	
	void DoSpeedChange()
	{
		SpeedPot = false; // Don't use speed pot to set speed
		staticPot = analogRead(POT_ADC); // save current for break-out
		speed =  (CMND_SEQ[0] - '0') * 10; // tens digit
		speed = speed + (CMND_SEQ[1] - '0'); // ascii to numeric, then add
    
		if(speed > 50) speed = 30; // reasonableness check, 50 max
		if(speed < 10) speed = 10;
    Serial.print("\nWPM: ");
    Serial.print(speed); // V2.1b added

	}

// *************  HAND KEY MODE *******************************************

// In the hand key mode, if either paddle is closed, then do the key down
// stuff. Else do the key up stuff. If both paddles are closed, exit the
// hand key mode.
// 6/18/2019 - add T/R relay control to hand key

		void HandKey()
		{
     // #ifndef I2C_LCD // if parallel mode
     // if(LCDmode) pins4paddle();
     // #endif

      ExitCMND(); //V2.0 added

			bool keyclosed = false;
			
			while (!((digitalRead(DotContact)==LOW) && (digitalRead(DashContact)==LOW)))
			{
			if((digitalRead(DotContact)==LOW) || (digitalRead(DashContact)==LOW))
			{
        /* // V0.1
				if(controlTR && !TRrelayON)
				{
					digitalWrite(TX_RX, TX_RX_Tstate); // energize T/R relay
					delay(TXOnDelay);
					TRrelayON = true;
				}
        */
				if(!keyclosed) // Not, but about to become closed ...
				{
          // V2.0 substitute call to key closed actions
          /*
					digitalWrite(TX_Out, keyDNstate); 
					if(SideTone) tone(spkrpin, CW_pitch); // produce tone  CW_pitch was pitch
					// digitalWrite(LEDpin, HIGH); // V0.1
                 */   
					keyclosed = true; // main code uses a different label

          KeyDownActions();
				}
				
			}
			
			else
				
			{
				if(keyclosed) // time to open, but not open yet
				{
          // V2.0, comment out and substitute call for actions on key up
					// timingTXHold = millis(); // start timing key open period // V0.1
          /*
					digitalWrite(TX_Out, keyUPstate);  // open it
					if(SideTone) noTone(spkrpin);
					// digitalWrite(LEDpin, LOW);  // V0.1
                    */
					keyclosed = false; // flag that it is open

          KeyUpActions();
				}
        /* // V0.1
				if((millis() >= (timingTXHold + (TXHold * 60))) && TRrelayON)
				{
					digitalWrite(TX_RX, TX_RX_Rstate); // drop out the relay
					TRrelayON = false;
				}
        */
			}
			} // Both paddles are closed below here
      KeyUpActions(); // One more time V2.0
      // V2.0 - below, stay until both paddles are open
      while((digitalRead(DotContact)==LOW) || (digitalRead(DashContact)==LOW));

		}

// **************** SET SIDETONE PITCH ************************************

// Dash paddle makes pitch higher. Dot makes it lower.
// Both paddles closed exits routine with new pitch
// Added delay and recheck because both paddles was hard to detect
// I originally made my steps about a semitone (36 Hz) at 600 Hz but
// that seems to large in practice, so going lower.

	void PitchSet()
	{
    //#ifndef I2C_LCD // if parallel mode
   // if(LCDmode) pins4paddle();
    //#endif

		while (!((digitalRead(DotContact)==LOW) && (digitalRead(DashContact)==LOW)))
		{
		
		if(digitalRead(DashContact)==LOW)
		{
			delay(200);
			if(digitalRead(DotContact)==LOW) goto imdone;
			CW_pitch = CW_pitch + 20;
			if(CW_pitch > 999) CW_pitch = 999; // keep it to 3 digits, for one thing
			test_beep();
		}
		if(digitalRead(DotContact)==LOW)
		{
			delay(200);
			if(digitalRead(DashContact)==LOW) goto imdone;
			CW_pitch = CW_pitch - 20;
			if(CW_pitch < 200) CW_pitch = 200;
			test_beep();
		}
		}
    imdone:;
		CW_pitch_ST = CW_pitch; // V2.0
		Serial.print("\nCW_pitch: ");
		Serial.println(CW_pitch, DEC);
	}

 // V2.1b - I found that I was calculating speed each time through. No great harm, but
 // I'll just do 

	void readSpeedPot()
	{

		float speed_x;
		// speed = SPD_LIMIT_LO; 
		speed_x = SPD_LIMIT_LO + (SPD_LIMIT_HI - SPD_LIMIT_LO) * (float) analogRead(POT_ADC)/1023.0;
		speed = (uint8_t)  (speed_x + 0.5); // casting just truncates
    if(speed == speed_old) goto no_calc;
    //Serial.print("\nWPM: "); // V2.1e added WPM to LCD so removed from serial monitor
    //Serial.println(speed); // V2.1b added
		lcd.setCursor(0,LCDBottomLine); // V2.1e momentary speed to LCD after change
		lcd.print(speed);
		lcd.print(" WPM  "); 
		LCDtrash = true; // tell main code to fix after delay
		LCDFixTime = millis();
		morseSetSpeed(); // calculates dot/dash/space times in ms
		
    no_calc:;
    speed_old = speed;

		// below checking for any jitter. Didn't see any ...
		//Serial.println(speed, DEC);
		//delay(500);
		
	}


	
// ************* RE-ENABLE SPEED POT IF CHANGE > THRESHOLD ****************

	// If the speed pot has been disabled by manual speed entry, this 
	// function checks to see if the pot has been moved ~2 WPM and
	// if so, re-enables it. (Actually 1.7 WPM)

  #ifdef PotExists
	
	void freeSpeedPot()
	{
		if(abs(staticPot - analogRead(POT_ADC)) > 50) SpeedPot = true;
		
	}

  #endif
	

  

// Routine sets speed to 25 and pitch to 550 Hz and restores old
// values on exit. 

	void SayOK()
		{
				unsigned int old_pitch;
				old_pitch = CW_pitch;
				CW_pitch = 550;
				speed_old = speed;
				speed = 25;
				morseSetSpeed();
				SideToneOld = SideTone;
				SideTone = true; // do send sidetone
				DoTransmitOld = DoTransmit;
				DoTransmit = false; // don't send on the air
				send_code('O');
				send_code('K');
				CW_pitch = old_pitch;
				speed = speed_old;
				morseSetSpeed();
				DoTransmit = DoTransmitOld; // restore previous
				// battTimer = millis(); // V0.1
				SideTone = SideToneOld;
				
			}

// *************** ANY - DETECT ANY USER ACTION ***************************

		bool Any()
		{
      unsigned long TimeOutTMR = millis(); // V2.0d if user action is stuck
      //#ifndef I2C_LCD // if parallel mode
      //if(LCDmode) pins4paddle();
     // #endif
			bool rvalue = false;
			if(!digitalRead(DotContact)) rvalue = true;
			if(!digitalRead(DashContact)) rvalue = true;
			if(!digitalRead(SW1)) rvalue = true; // V0.1 V2.0 uncomment
			if(!digitalRead(SW2)) rvalue = true; // V0.1 V2.0 uncomment
			
			if(rvalue) // V1.1 - stay until action is released:
				while( !digitalRead(DotContact) || !digitalRead(DashContact)
						|| !digitalRead(SW1) || !digitalRead(SW2))
            {
              if(millis() - TimeOutTMR > 3000) Serial.println(F("Switch or paddle stuck")); // V2.0d
            } 
				
			return rvalue;
		}

 // *** DelayAndCheck will delay the number of ms in the argument unless
 // any user action occurs. Then it terminates and returns false. If it goes
 // to completion, it returns true. *** V1.1a
 

	bool DelayAndCheck(uint16_t mdelay)
	{
		bool hometruth = true;
		unsigned long targetTime;
		targetTime = millis() + mdelay;
		
		while(millis() < targetTime && hometruth == true)
		{
			if(Any()) hometruth = false;
		}
		
		return hometruth;
	}
		
		
	

// **********************  LOCK OUT ON STUCK PADDLE ***********************	
	
	void LockOut()
	{
    //#ifndef I2C_LCD // if parallel mode
		//if(LCDmode) pins4paddle();
   // #endif
		// This routine got a little complicated as I was chasing a bug, but
		// it's still good.
		
		digitalWrite(TX_Out, keyUPstate); // open keyed line
		noTone(spkrpin); // stop tone
		while(digitalRead(DotContact)==LOW || digitalRead(DashContact)==LOW) // stay while either or both closed
		{
			tone(spkrpin, 600); // warble tone to alert user to lockout
			delay(250);
			tone(spkrpin, 566);
			delay(250);
			noTone(spkrpin);
			delay(500);
			timing_start = millis(); // get back into range of unit8_t
		}
		
		DeadMan = 255;
		DoLockout = false;
	}
// ************************************************************************

// **********   ADDING THE CONTENTS OF Morse.ino here *************



/*


In Morse, the length of a dit or space in seconds = 1.2/WPM, in ms = 1200/WPM

Timings are (in dit-lengths):

dit:      1
dah:      3
inter-sp  1    (spaces between elements of a character)
char sp   3    (between letters)
word sp   7    (between words)


Arduino has a function NewTone(pin, freq, duration).  Duration is optional and is in ms.
If duration is not used, call noNewTone(pin) to shut it off.  Output is a 50% square 
wave. Note: NewTone is not a built in library, I had to download the zip, then use
sketch>Include Library>Add ZIP Library to bring it in.

*** I couldn't get it to work with the duration argument, so I switched to turning it
    on, timing the duration separately, then turning it off. ***

***  I'm using a 12 mm sounder I got from Digikey CEM-1201(50).  It's plenty loud.  I have 
     180 ohm resistor in series with it and connect it between Pin 7 and ground.  In its 
	  data sheet it has a resonance over 2,000 Hz and another one around 600 Hz, so I'm using
	  600 Hz.
***	V1.1 - I keep having trouble with NewTone disappearing, so I'm going to
    just tone()

*/


// Morse characters are stored like this:  The high bit represents the first
// element of a character.  0 is a dot and 1 is a dash.  The final 1 flags
// the end of this character and is not sent.  Therefore 'B' being _... is 
// stored as 0b10001000
// Sending happens by checking the high bit and sending dot or dash as
// appropriate, then left shifting the data by one bit.  Before sending though,
// if the data = 0b10000000, then stop.  Sending of this character is finished.

// There are some gaps in ASCII codes that have characters I don't need and
// so waste some RAM, but not enough to do special remapping.
// From 44 through 90 are 47 bytes and I figure 41 are moderately necessary.

// Notice below how it's possible to break lines and even have comments
// interspersed

byte morse[47] = {0b11001110, 0b10000110, 0b01010110,// ','  '-'  '.' 44-46
					0b10010100, 0b11111100, 0b01111100,// /, 0, 1  47-49
					0b00111100, 0b00011100, 0b00001100, // 2, 3, 4 50-52
					0b00000100, 0b10000100, 0b11000100, // 5, 6, 7 53-55
					0b11100100, 0b11110100, 0b10110100, // 8, 9, : (KN) 56-58
					0b10101010, 0b01010100, 0b10001100, // ; (KR), < (AR), = (BT) 59-61
					0b00010110, 0b00110010, 0b01101010, // > (SK), ?, @ (AC) 62-64
					0b01100000, 0b10001000, 0b10101000, // ABC starts at 65 DEC
					0b10010000, 0b01000000, 0b00101000, // DEF
					0b11010000, 0b00001000, 0b00100000, // GHI
					0b01111000, 0b10110000, 0b01001000, // JKL
					0b11100000, 0b10100000, 0b11110000, // MNO
					0b01101000, 0b11011000, 0b01010000, // PQR
					0b00010000, 0b11000000, 0b00110000, // STU
					0b00011000, 0b01110000, 0b10011000, // VWX
					0b10111000, 0b11001000};             // YZ ends at 90 DEC


// V2.0 I must alter dit() and dah() so that they use the routines for key up
// and down actions used by other parts of the program.

void dit()
	{
    /*
		if(DoTransmit)
		{
			digitalWrite(TX_Out, keyDNstate); // close keyed line
		}
		if(SideTone) tone(spkrpin, CW_pitch);
    */
    KeyDownActions();
		space(); // delay one space-time
		//if(SideTone) noTone(spkrpin);
    KeyUpActions();
		// digitalWrite(TX_Out, keyUPstate); // open keyed line 1/30 ALREADY HAPPENS
		space();
	}

void dah()
	{
    /*
		if(DoTransmit)
		{
			digitalWrite(TX_Out, keyDNstate); // close keyed line
		}		
		if(SideTone) tone(spkrpin, CW_pitch);
    */
    KeyDownActions();
		space();
		space();
		space();
		//if(SideTone) noTone(spkrpin);
		//digitalWrite(TX_Out, keyUPstate); // open keyed line
    KeyUpActions();
		space();
	}

void space()
	{
	   delay(ditlen); // delays ditlen milliseconds
	}

void char_space()
	{
		delay(2*ditlen);
	}
void word_space()
{
	  delay(4*ditlen);
}

// ******  SEND CHARACTER STRING IN MORSE *********************************


void send_code(int pointtocode)
{
	byte pattern;
	if (pointtocode == ' ')
	{
	   word_space();
	}
	else
	{
	pointtocode -= 44; // shift ASCII pointer to begin at 0 for comma
	pattern = morse[pointtocode];
	
	while (pattern != 128)
	{
		if (pattern & 0b10000000)
		{
			dah();
		}
		else
		{
		   dit();
		} 
		
		pattern <<= 1;
		
	}
	char_space();
	}
}		



void morseSetSpeed()
{
    ditlen = 1200/speed;
    dahlen = ditlen * 3;  
	//halfspace = ditlen/2;
	
	// halfspace = 2; // Change from half space to fixed 2 ms
	// As of 6/10/2019, I'm trying 'halfspace' equal to about 75% of
	// a space, mimicking what the K3, KX3 and FT-991A do.
	
	halfspace = ditlen/2 + ditlen/4;
	
	wordspace = 6 * (uint16_t) ditlen;
}

void sayRev()
{
	char morsechar;
	
	delay(100);
}
	
  
bool morseSendString(char mstring[])
{
// V2.0 will modify to check for user action and terminate sending
// if any is detected. Also returns bool false if stopped by user

      boolean rvalue = true;
      for(int x=0; mstring[x]; x++)
        {
            send_code(mstring[x]);
            if(Any())
            {
              rvalue = false;
              goto _stopsend;
            }
        }
      _stopsend:
      return rvalue;
    }

// trying for a slightly negative sound for error, or cancel:

	void rasp()
	{
		uint8_t icount = 70;

		while(icount)
		{
			digitalWrite(spkrpin, HIGH);
			delay(1);
			digitalWrite(spkrpin, LOW);
			delay(1);

			digitalWrite(spkrpin, HIGH);
			delay(2);
			digitalWrite(spkrpin, LOW);
			delay(2);			
			icount--;			
		}
			
		
	}
	
    void short_sound()
    {
			delay(50);
            tone(spkrpin, CW_pitch); // give user short beep feedback
            delay(25);
            noTone(spkrpin);
    }
	
	void test_beep()
	{
		tone(spkrpin,CW_pitch);
		delay(250);
		noTone(spkrpin);
		delay(200);
	}
	
	// Play a descending tone to indicate a bad outcome
	
	void bad_sound()
	{
		delay(150);
		tone(spkrpin, CW_pitch);
		delay(150);
		tone(spkrpin, (CW_pitch - 35));
		delay(150);
		noTone(spkrpin);
	}	
	
	// ascending tone indicates good outcome
	
	void good_sound()
	{
		delay(100);
		tone(spkrpin, CW_pitch -15);
		delay(100);
		tone(spkrpin, (CW_pitch + 15));
		delay(100);
		noTone(spkrpin);
	}	

// V2.0 add a function to transmit a CW message continuously until 
// some user action stops it. In this case "transmit" means
// one of the outputs is keyed. The transmitter output is not
// keyed

void SendTestText()
{
  lcd.clear(); // V2.0h
  Print_freq(FoutRX, LCDTopLine);
  lcd.setCursor(0, LCDBottomLine);
  lcd.print("Sending test TXT");
  boolean keepon = true;
  while(keepon)
  { keepon = morseSendString(" VVV TEST "); }
  KeyUpActions(); // make sure I leave with key up
	reset_PLL_A(); // V2.1a - ON/OFF keying upsets quadrature relationship, reset to fix
}

void  shiftMorseLeft()
    {
      for(uint8_t i = 1; i < 16; i++)
      MorseField[i - 1] = MorseField[i];

    }
