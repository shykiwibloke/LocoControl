/****************************************************
**                                                 **
**  Loco Control Stand version                     **/
#define VERSION "3.1.0"
/*                                                 **
**  Written by Chris Draper                        **
**  Copyright (c) 2016                             **
**                                                 **
*****************************************************
**
** This code is free software; you can redistribute it
** and/or modify it under the terms of the GNU Lesser
** General Public License as published by the Free Software
** Foundation; either version 2.1 of the License, or (at
** your option) any later version.
**
** This code is distributed in the hope that it will
** be useful, but WITHOUT ANY WARRANTY; without even the
** implied warranty of MERCHANTABILITY or FITNESS FOR A
** PARTICULAR PURPOSE.  See the GNU Lesser General Public
** License for more details.
**
** Should you need a copy of the
** GNU Lesser General Public License.
** Write to the Free Software Foundation, Inc.,
** 51 Franklin St, Fifth Floor, Boston, MA 02110-1301 USA
**
****************************************************/

/* Recent Changes Log
 *	3.1.0
06/10/2018: Add moving median code to throttle and amperge readings
 *  3.0.2
16/07/2018: If 5 comms errors detected while getting motor current then reset comms.
15/07/2018: Modified main loop to call amperage and Battery Voltage routine more efficiently when comms not needed for motor control
15/-7/2018: Modified DoTimedIntervalChecks() to be less of a distraction/interference with serial
23/06/2018: Start modifications to work on Mega - remove software serial in favour of hardware Serial1
 *  2.1.2
26/05/2018: Modified Battery voltage routine to ignore timeouts (and not set battery to zero)
26/05/2018: Commented out a few debug messages to limit print slowness
26/05/2018: Shortened Motor Comms timeout to 1 second to speed responsiveness
26/05/2018: Altered motor current routine to report down to one amp instead of two.
03/04/2018: Removed old debug code / shortened some text messages
03/04/2018: Fixed Volts and Amps from Motor Controllers.
 *  2.1.1
31/03/2018: Fixed Flashing lights - invalid setting was wrong - changed to 3
31/03/2018: moved CalcMotorNotchSize() to every ten seconds check instead of only at idle
31/03/2018: Fixed Bug in Arduino comms receive + put delay in reboot command execution
31/03/2018: SetGetTimeout increased from 1000 to 1500 to fix timeout for GetBatteryVoltage 
28/03/2018: SetMotorRamping configured to set to 2000 at startup (guess to be tested)
28/03/2018: Fixed bug in GetDynamicMode that was calling print endlessly in idle.
28/03/2018: Modify CalcMotorNotchSize to range of 128-255 for pot sweep
28/03/2018: Fixed FlashDitchLights --gDitchFlashCount < 2
28/03/2018: Fixed SetDynamicBrake Notch Calculation
13/03/2018: Added DYN_KEY_SW and its associated logic
12/03/2018: Dynamic Brake changes ready for testing (Forgotten when these were done)
12/03/2018: Changed Momentum() to CalcMotorNotchSize()
12/03/2018: Change MOTOR_NOTCH to MAX_NOTCH_SIZE and create gNotchSize variable for Speed calculation purposes
12/03/2018: Obsolete SetMotorRamping() - replaced by Max Speed calculations
12/03/2018: Change Version to 2.0.4
25/05/2017: Add notch detent and modify use of analog potentiometer to modify speed increment per notch

End Recent Changes Log  */


//#include <SoftwareSerial.h>   //used to drive comms to motor controllers
#include <USBSabertooth.h>    //comms library for sabertooth controllers
//#include <Servo.h>            //used for speedo - can be removed to make space if no speedo fitted 
#include <SimpleTimer.h>      //used for vigilance and heartbeat type functions

//***************************************************
// Defines
//***************************************************

//general constants

//#define LOG_LEVEL_4			//if defined then debug info will be output to the raspi


#define MOTOR_SERIAL_SPEED	9600      //9600 for slower testing 
#define RASPI_SERIAL_SPEED 19200			//faster speed over USB link
#define SERIAL_BUFSIZE	50		  			//Reserve 50 bytes for sending and receiving messages with the raspi
#define MOTOR_RAMPING 2400						//smooths transition between notches

#define MAX_NOTCHES		9								//allow for zero and 8 notches - Applies to Throttle and Dynamic

//Status Definitions

const int STATUS_ERROR 		= 99;
const int STATUS_IDLE 		= 10;
const int STATUS_POWER 		= 20;
const int STATUS_DYNAMIC 	= 30;

const int DIR_NONE 				= 0;
const int DIR_FWD 				= 1;
const int DIR_REV 				= 2;

//Sabertooth configuration defines
const int MOTOR_TIMEOUT = -32768;			//value returned by Sabertooth library when comms fails. Must be const int to work reliably.

#define MAX_NOTCH_SIZE      255       //the maximum value we can send for throttle or dynamic brake per notch position - 1

//Sabertooth ports for ditch lights and headlights
#define HEADLIGHT_ST	2
#define DITCHLIGHT_R 	1
#define DITCHLIGHT_L	0


#define LIGHT_OFF			-2000
#define LIGHT_HALF		-500
#define LIGHT_FULL		2000

//***************************************************
//Pin Use Definitions
//***************************************************

//Explanation

/* Devices attached to the Arduino are:
  Throttle        - 10k pot
  Dynamic Brake 	- 10k pot
  TrainWeight     - 10k pot
  Reverser        - 2 x microswitches
  Speedo          - IR interrupted beam or Hall effect for pickup
                - R/C Servo for speedo analogue indicator
  Horn            - Pushbutton with lever
  Key Switch	  - Key switch to toggle Dynamic Braking on or offDYN_KEY_SW
  Headlights      - Switch OFF/LOW/HIGH (Ditch lights steady/flashing handled by software)
  Vigilance Reset	- pushbutton input
  Buzzer          - used for warning buzzer (vigilance, serious error etc)
  USBSerial I/F	  - Serial (debug) interface used for debugging and comms to Raspi (for sound and stats)
  Sabertooth I/F5v- Serial interface

*/

//**************************************************
//Analogue Inputs

#define THROTTLE_PIN  A0		//A0 Throttle Handle - 10K Potentiometer
#define DYNAMIC_PIN   A1		//A1 Dynamic Brake Handle - 10K Potentiometer
#define MAX_SPEED_PIN A2		//A2 MaxSpeed (Train max speed bias) 10k Potentiometer

//**************************************************
//Digital I/O
//D1&2 - reserved for RS232 comms via USB Serial (a.k.a. Debug or programming port)
//D3&4 - reserved for soft serialRS232 comms port (Sabertooth Motor Controllers)
#define PIN_DIR_FWD 	5       //D5 - Microswitch - internal pullup - active low
#define PIN_DIR_REV 	6       //D6 - Microswitch - internal pullup - active low
#define TACHO_IN			7       //D7 - input for tachometer (interrupted IR)
#define BTN_HORN			8       //D8 - input switch on console
#define DYN_KEY_SW		9				//D9 - Key switch used to toggle Dynamic Braking Mode
#define BTN_VIGILANCE	10      //D10 - pushbutton on console - note input is active HIGH
#define SPEEDO_PIN		11      //D11 - RC Servo output to drive Speedometer
#define BUZZER_PIN		12      //D12 - Buzzer in console
#define VIGILANCE_EN	13      //D13 - Vigilance Button Enable + the on board LED
#define HEADLOW_PIN   A3      //A3 - Headlights to low power
#define HEADHIGH_PIN  A4      //A4 - Headlights to high power 


//***************************************************
// Global Variable Definitions
//***************************************************


//General Globals

bool gInitialized       	= false;
bool gRaspiOK           	= false;
bool gControlsChanged   	= false;
int  gControlStatus     	= 0;
int  gMotorStartDelayID 	= 0;
bool gGetMotorStatusFlag 	= false;

//Control Inputs
int  gCurrentThrValue			= 0;																	//Array to hold samples for moving Median Calculation
int  gCurrentThrNotch     = 0;																	//Notch currently calculated for throttle
int  gCurrentDynValue			= 0;																	//Array to hold samples for moving Median Calculation
int  gCurrentDynNotch			= 0;																	//Notch currently calculated for Dynamic
bool gDynBrakeActive			=	false;															//True if Dynamic Brake selection switch is set
int  gNotchSize						= 0;																	//Notch size - to scale output as directed by CalcMotorNotchSize()
int  gDirection         	= 0;																	//Forward, Reverse, Neutral ( See DIR_NONE, DIR_FWD, DIR_REV)
int  gBattery           	= 0;																	//Current Battery Voltage
int	 gHorn              	= 0;																	//Horn sounding 0 = no, 1 = yes
int  gHeadlights        	= 0;																	//Headlights off, dip, full (0,1,2 respectively)
int  gMotorCurrent[6]  		= {0};																//last read motor current for individual motorsint  gMotorCurrent[6]  		= {0};																//last read motor current for individual motors
int  gMotorOffset[6]  		= {0};																//last calculated speed offset for motors
int	 gVigilanceCount    	= 0;																	//Timer count for the Vigilance Alarm
int  gDitchFlashCount   	= 0;																	//Count for flashing the ditch lights when the horn is sounded

//***************************************************
//Define Timer
SimpleTimer gtimer;                         //Only need one timer as all timing callbacks can be handled by this one object

//***************************************************
//define soft serial port for motor comms here

//SoftwareSerial SWSerial(3, 4);              // RX, TX
//USBSabertoothSerial gSP(SWSerial);

USBSabertoothSerial gSP(Serial1);   //Assign to Mega Hardware serial port

//there are three motor controllers each with two motors. arrange these so they are front to rear in numerical order
USBSabertooth       gSabertooth[3] = { USBSabertooth(gSP, 128), USBSabertooth(gSP, 129), USBSabertooth(gSP, 130) };

//***************************************************
String  gRaspi_TxBuffer = "";               //A string to hold formatted messages for sending
String  gRaspi_RxBuffer = "";               //A string to hold incoming data
boolean gRaspi_RxComplete = false;          //True if the string is complete

//***************************************************

//Servo gSpeedo;                              // create servo object to control Speedo needle

//***************************************************
// Setup Code
//***************************************************

void setup()
{
  gInitialized = false;

  //set the general inputs and outputs
  ConfigDigitalPins();

  //Configure comms to motor controllers via software serial
  ConfigComms(false);

//Announce Our arrival
  Serial.print("L:1:Loco Control Stand Ver: ");
  Serial.println(VERSION);

  //configure the speedo output
//  gSpeedo.attach(SPEEDO_PIN);

  //call various subroutines to ensure internal state matches control panel
  initSystem();

  //Check slow controls and heartbeat comms every fifteen seconds
  gtimer.setInterval(15000, DoTimedIntervalChecks);

}

//***************************************************
// Main loop & Core Subroutines
//***************************************************

//Main loop must be executed frequently for system to run reliably
// (i.e. no long Delay() type statements in subroutines unless deliberately done)

void loop()
{

  ReadInputs();                   //Read input variables

  if (gControlsChanged)           //only process if controls have changed
  {

    ResetVigilanceWarning();    //Control movement resets vigilance
    CalcMotorNotchSize();       //check to see if user has changed the notch size
    EvaluateState();            //check to see if the control changes mean we are in a new state
    //and issue commands based on the changed controls - and also keep-alive heartbeats
  }
  else 
  {
    GetMotorAmpsVolts();              //fetch and send one motor's current per call, and battery voltage once per 6 motor checks.
    ServiceComms();                 // Comms Housekeeping route - handles reception of messages from Raspi etc
  }
  
  gtimer.run();                     //must be in main loop for timer to work

}


//***************************************************
void ReadInputs(void)
{

  int rtnval = 0;

  rtnval = GetDirection();
  rtnval = GetDynamic();
  rtnval = GetThrottle();

  CheckHorn();                      //if horn is pressed, then sound it. if released, then tell raspi
  CheckHeadlights();        		//checks the headlights/ditchlights

  if (digitalRead(BTN_VIGILANCE))   //Hitting vigilance button is considered a control change
  {
    ResetVigilanceWarning();        //Just reset vigilance - not a control 'change'
  }
  
  if (gControlStatus == STATUS_IDLE)   //only want to evaluate these items if the loco is at a standstill
  {
	  GetDynamicMode();
  }
  
  
  if (rtnval)
  {
    //todo - can evaluate any control errors here - that arise during running
    // if you so choose
  }
}

//***************************************************
void EvaluateState(void)
{

  if (gControlStatus == STATUS_ERROR)
  {
    //Stay there until the error clears
    Serial.println("L:1:EvalutateState Error Condition. Place Controls to Idle to clear");
    SetVigilanceWarning(true);
    delay(50);
    ResetVigilanceWarning();
    if ((gDirection == DIR_NONE) && (gCurrentThrNotch == 0) && (gCurrentDynNotch == 0))
    {
      //controls are safe and no errors detected with them
      gInitialized = true;
      ResetVigilanceWarning();
      CalcControlStatus(STATUS_IDLE, "Controls Now at Idle");
      SetMotorSpeed();
    }
  }
  else
  {
    if ((gCurrentThrNotch == 0) && (gCurrentDynNotch == 0))
    {
      CalcControlStatus(STATUS_IDLE, "Idle");
      SetMotorSpeed();			//ensure speed is indeed set to zero
    }
    else if (gDirection != DIR_NONE)
    {
      //Direction has been set to forward or reverse - dont actually care which here.

 /*     if (gCurrentDynNotch > 0 && gDynBrakeActive == true)
      {
        CalcControlStatus(STATUS_DYNAMIC, "Dyn Brake ON");

        Serial.print("S:");
        Serial.print(gCurrentDynNotch);
        Serial.println(":");
		    SetDynamicBrake();    //go adjust motors into proper dynamic notch
      }
      else */ if (gCurrentThrNotch > 0)
      {
        int old = gControlStatus;
        CalcControlStatus(STATUS_POWER, "Throttle ON");
      }
	/*	if (gDynBrakeActive == true)
		{
			//if we are in dynamic mode - need to 'coast' with throttle closed
			//do the calculations for coasting here.
			
			
			//**TODO** WORKING HERE **
			//(note no calcs means code just drops through and works as is now)
		}
		*/	
		//Send Sound Command
        Serial.print("S:");
        Serial.print(gCurrentThrNotch);
        Serial.println(":");
        SetMotorSpeed();   //go adjust motors
        
    }
  }

  gControlsChanged = false;     //reset the changed flag as we have just handled it.

}

//***************************************************
// Evaluation and Calculation subroutines
//***************************************************

void CalcControlStatus(const int NewStatus, const char* Msg)
{
  //Evaluates and logs a control state change.
  //NOTE - it will only log the state change ONCE so safe to call in a loop.
  //Classed as debug level message unless current or new state is ERROR

  if (NewStatus != gControlStatus)
  {
    if (NewStatus == STATUS_ERROR || gControlStatus == STATUS_ERROR)
    {
      Serial.print("L:1:Critical Error - ");    //print the error heading
      Serial.println(Msg);              //print the passed message

    }

    gControlStatus = NewStatus;       //set the new status

    //Send the proper Sound System status command
    if (NewStatus == STATUS_POWER)
      Serial.println("S:t:");

    if (NewStatus == STATUS_DYNAMIC)
      Serial.println("S:d:");

    if (NewStatus == STATUS_IDLE)
      Serial.println("S:0:");

  }
}

//***************************************************
int CalcNotch(const int newval, int * ptr_Notches)
{
  /*
	Takes the retrieved potentiometer value and calculates which notch the lever is currently in
	nb: this routine serves both throttle and dynamic brake
	
	Routine looks for the closest notch to the presented value.
	
	*/
	int nearestNotch = 0;
	int nearestGap =99;								//ensure this value is very high so at least one notch will bring it down
	
  for(int i = 0;i< MAX_NOTCHES;i++)
	{
	
		if (abs(ptr_Notches[i] - newval) < nearestGap)
		{
				//current Notch value is within 10. Make that our notch as it is the 'Nearest'
				nearestNotch = i;
				nearestGap = abs(ptr_Notches[i] - newval);
				
				//not sure how this works with the zero notch.
		}

	}
  return nearestNotch;

}

//***************************************************
void CalcMotorNotchSize(void)
{
  //Reads the MaxSpeed value and set the ramp variables accordingly if it has changed.

  //read it, note: Arduino analog range is roughly half that of the sabertooth & this routine takes that into account
  int newval = analogRead(MAX_SPEED_PIN) >> 3;
 
  newval += 128;			//shift up to 128-255 range
 
  //limit-check the value to fit range 128-255 before comparing it to last setting/using it
  if (newval > MAX_NOTCH_SIZE) newval = MAX_NOTCH_SIZE;
  if (newval < 128) newval = 128;

  //has it changed?
  if (gNotchSize != newval)
  {

   //set it
    gNotchSize = newval;
	
	//and Advise
	Serial.print("L:4:NotchSize=");
    Serial.println(gNotchSize);

  }
}

//***************************************************
void DoTimedIntervalChecks(void)
{
  //Stuff that only needs to be checked every fifteen seconds or so Called by timer routine
  static int callcount = 0;

  gGetMotorStatusFlag = true;   //set flag to get another round of measurements - if the serial buffers are empty
   
  if (++callcount == 2)
  {

    callcount = 0;
  }

  //Calculate Vigilance warnings

  if (gDirection == DIR_NONE)
  {
    ResetVigilanceWarning();        //suppress vigilance when in neutral.
  }
  else
  {
    if (++gVigilanceCount == 2)  //more than 30 seconds since a control was touched
    {
      //Serial.println("L:2:Vig Light");
      SetVigilanceWarning(false);   //turn light on now (no buzzer
    }

    if (gVigilanceCount == 3)  //light been on for at least 15 seconds
    {
      //Serial.println("L:2:Vig Buzzer");
      SetVigilanceWarning(true);  //Turn buzzer on
    }

  }
}

//***************************************************
//Output setting routines
//***************************************************

void SetMotorSpeed(void)
{

  int speed = gNotchSize * gCurrentThrNotch;        //used to scale the provided notch to a speed setting the controller understands (-2047-0-2047 range)

  if (gDirection == DIR_REV)  speed = -speed;                    //a negative number makes the motor go in reverse, positive = go forward
	
  //send the commands to the motor controllers including offsets to ensure all pull their load

  gSabertooth[0].motor(1, speed + gMotorOffset[0]);     //first pair
  gSabertooth[0].motor(2, speed + gMotorOffset[1]);
  gSabertooth[1].motor(1, speed + gMotorOffset[2]);     //2nd pair
  gSabertooth[1].motor(2, speed + gMotorOffset[3]);
  gSabertooth[2].motor(1, speed + gMotorOffset[4]);     //3rd pair
  gSabertooth[2].motor(2, speed + gMotorOffset[5]);

	Serial.println("Setting Actual Motor Speeds");
	Serial.println(speed + gMotorOffset[0]);
  Serial.println(speed + gMotorOffset[1]);
  Serial.println(speed + gMotorOffset[2]);
  Serial.println(speed + gMotorOffset[3]);
  Serial.println(speed + gMotorOffset[4]);
  Serial.println(speed + gMotorOffset[5]);
	Serial.println("**************");
}

//***************************************************

void SetDynamicBrake()
{

  int brake = (8 - gCurrentDynNotch) * gNotchSize;          //used to scale the provided notch to a speed setting the controller understands (-2047-0-2047 range)
  //Note the notch is inverted so most gentle braking is on Notch one

  if (gDirection == DIR_REV)  brake = -brake;     //a negative number makes the motor go in reverse, positive = go forward
  //Note this must stay in the same direction of travel for Sabertooth braking to work properly
 
   Serial.print("setmotor(D):");
   Serial.println(brake);
 
  //send the commands to the motor controllers
 
  gSabertooth[0].motor(1, brake);     //first pair
  gSabertooth[0].motor(2, brake);
  gSabertooth[1].motor(1, brake);     //2nd pair
  gSabertooth[1].motor(2, brake);
  gSabertooth[2].motor(1, brake);     //3rd pair
  gSabertooth[2].motor(2, brake);

}

//***************************************************

void SetMotorRamping(int newvalue)
{

  gSabertooth[0].setRamping(newvalue);
  gSabertooth[1].setRamping(newvalue);
  gSabertooth[2].setRamping(newvalue);

}

//***************************************************
void CheckHeadlights(void)
{
  int state = 0;

  if (digitalRead(HEADLOW_PIN) == 0) state = 1;
  if (digitalRead(HEADHIGH_PIN) == 0) state = 2;

  if (gHeadlights != state)
  {
    switch (state)
    {
      case 0:  //headlight off
        gSabertooth[HEADLIGHT_ST].power(LIGHT_OFF);
        gSabertooth[DITCHLIGHT_L].power(LIGHT_OFF);
        gSabertooth[DITCHLIGHT_R].power(LIGHT_OFF);
        //Serial.println("L:2:lights off");
        break;
      case 1:  //headlight on dip
        gSabertooth[HEADLIGHT_ST].power(LIGHT_HALF);
        gSabertooth[DITCHLIGHT_L].power(LIGHT_HALF);
        gSabertooth[DITCHLIGHT_R].power(LIGHT_HALF);
        //Serial.println("L:2:lights dip");
        break;
      case 2: //headlight on full
        gSabertooth[HEADLIGHT_ST].power(LIGHT_FULL);
        gSabertooth[DITCHLIGHT_L].power(LIGHT_FULL);
        gSabertooth[DITCHLIGHT_R].power(LIGHT_FULL);
        //Serial.println("L:2:lights full");
        break;

    }

    gHeadlights = state;


  }
}

//***************************************************
void FlashDitchlights(void)
{
  //Called by timer callback setup within CheckHorn()

  if (--gDitchFlashCount < 2)
  {
    //restore both ditch lights to whatever they were by invalidating current position
    gHeadlights = 3;
    CheckHeadlights();
    return;
  }

  if (gDitchFlashCount%2 == 0)     //if count is even
  {
      gSabertooth[DITCHLIGHT_L].power(LIGHT_FULL);  //turn left ditchlight
      gSabertooth[DITCHLIGHT_R].power(LIGHT_OFF);  //turn left ditchlight
  }
  else
  {
      gSabertooth[DITCHLIGHT_R].power(LIGHT_FULL);  //turn left ditchlight
      gSabertooth[DITCHLIGHT_L].power(LIGHT_OFF);  //turn left ditchlight
  }
}

//***************************************************
/*
void SetSpeedo(int speed)
{
  //Set the speedo needle to point to the supplied speed.

  Serial.println("L:4:Set Speedo ");
  speed = map(speed, 0, 1023, 0, 180);     // scale it to use it with the servo (value between 0 and 180)
  gSpeedo.write(speed);                  // sets the servo position according to the scaled value
}
*/
//***************************************************

void CheckHorn(void)
{

  static int newval[3] = {0, 0, 0};
  static int idx = 0;

  switch (idx)
  {
    case 0:
      newval[0] = digitalRead(BTN_HORN);
      idx++;                              //1st analog sample
      return;
    case 1:
      newval[1] = digitalRead(BTN_HORN);
      if (newval[0] != newval[1])
        idx = 0;                            //we are in the middle of a change so reset for three new samples
      else
        idx++;                              //1st two analog samples match, so advance ready for third
      return;
    case 2:
      newval[2] = digitalRead(BTN_HORN);
      idx = 0;                              //reset whatever outcome of next test
      if (newval[0] != newval[2])
      {
        return;                             //go take another three samples
      }
      break;                                //success - process received command
  }
  //Set the horn to match the button state.
  if (newval[0] == 0 && gHorn == 0)	//Horn button pressed, we have not seen this yet
  {
    Serial.println("S:h:");
    gHorn = 1;

    //set ditch-lights flashing for ten seconds
    gDitchFlashCount = 30;
    gtimer.setTimer(400, FlashDitchlights, gDitchFlashCount);
 
    ResetVigilanceWarning();		//horn press also resets the vigilance timer.

  }
  else if (newval[0] == 1 && gHorn == 1)		//Horn button released, we have not seen this yet
  {
    Serial.println("S:j:");
    gHorn = 0;
  }
}

//***************************************************
void SetVigilanceWarning(bool SoundBuzzer)
{

  //Set off Vigilance light the vigilance LED

  digitalWrite(VIGILANCE_EN, HIGH);

  if (SoundBuzzer) digitalWrite(BUZZER_PIN, HIGH);

}

//***************************************************
void ResetVigilanceWarning(void)
{
  //Reset Vigilance buzzer and light the vigilance LED

  digitalWrite(VIGILANCE_EN, LOW);
  digitalWrite(BUZZER_PIN, LOW);
  gVigilanceCount = 0;

}
//***************************************************
//Get State Routines
//***************************************************

void GetMotorAmpsVolts()
{
  
  static int motor = 0;
  static int ErrCount = 0;
  int      rawvalue = 0;

  //Gets amps on cycle - for motor one, then two on second run etc.
  if(gControlStatus != STATUS_ERROR 
     && Serial1.availableForWrite() > 60 
     && Serial.availableForWrite() > 60
     && gGetMotorStatusFlag == true
     && gControlsChanged == false)  //Only request from motor drivers if BOTH comms channels have the buffer space to cope AND there are no commands being processed AND only every 15seconds
  {
    switch(motor)
    {
        case 0:
            rawvalue = gSabertooth[0].getCurrent((1), false);
            break;
        case 1:
            rawvalue = gSabertooth[0].getCurrent((2), false);
            break;
        case 2:
            rawvalue = gSabertooth[1].getCurrent((1), false);
            break;
        case 3:
            rawvalue = gSabertooth[1].getCurrent((2), false);
            break;
        case 4:
            rawvalue = gSabertooth[2].getCurrent((1), false);
            break;
        case 5:
            rawvalue = gSabertooth[2].getCurrent((2), false);
            break;
        default:
						//Have all six - send update to display
						Serial.print("M:");
						for(motor = 0; motor < 6; motor++)
						{
							Serial.print(gMotorCurrent[motor]);         //Motor Current sent in 10ths of an amp
							if(motor<5)
								Serial.print(",");
						}
						Serial.println(" ");
						motor = 0;
            ErrCount = 0;
            GetBattery();                 //Get the battery voltage once per scan of all the motor currents
						SetMotorOffsets();						//Use latest values to ensure motors pulling equally
            gGetMotorStatusFlag = false;  //done a read through, wait for at least another 15 seconds.
            return;
    }
      
    if(rawvalue == MOTOR_TIMEOUT) //if timed out
    {
        if(++ErrCount > 4)          //if more than four errors have been collected this run through all the motors
        {
            ConfigComms(true);          //Go reset comms
            Serial.println(" ");
            Serial.println("L:1:Comms RESET");
            ErrCount = 0;
        }
    }
    else
    {

	if (rawvalue != gMotorCurrent[motor])
      {
  
        gMotorCurrent[motor] = rawvalue;
  
      }
    }
    motor++;
  }
}

//***************************************************
void GetBattery(void)
{

  int temp = 0;
  
  //Battery should be the same for all controllers - so only have to read one
  temp = gSabertooth[0].getBattery(1, false);
  if (temp != MOTOR_TIMEOUT)
  {

    //todo - maybe reset comms if battery timeout?
      
    if (temp != gBattery)
    {
      //only send battery if it changes
      gBattery = temp;
  
      Serial.print("V:1:");
      Serial.println(gBattery);
    }
  }
}

//***************************************************

void SetMotorOffsets(void)
{
//Routine to examine last lot of current readings and alter motor offsets so all pull their weight equally	
	
	//gMotorOffset[0-5]
	 int avg = gMotorCurrent[0]
						+gMotorCurrent[1]
						+gMotorCurrent[2]
						+gMotorCurrent[3]
						+gMotorCurrent[4]
						+gMotorCurrent[5];
	
	if(avg != 0)
	{
		avg = avg/6;
	}
	/*
	gMotorOffset[0] = gMotorCurrent[0]-avg;
	gMotorOffset[1] = gMotorCurrent[1]-avg;
	gMotorOffset[2] = gMotorCurrent[2]-avg;
	gMotorOffset[3] = gMotorCurrent[3]-avg;
	gMotorOffset[4] = gMotorCurrent[4]-avg;
	gMotorOffset[5] = gMotorCurrent[5]-avg;
	*/	
	
	Serial.print("Calculated Offsets (avg,m1,m2...) ");
	Serial.print(avg);
	Serial.print(",");
	Serial.print(gMotorCurrent[0]-avg);
	gMotorCurrent[1]-avg;
	Serial.print(",");
	gMotorCurrent[2]-avg;
	Serial.print(",");
	gMotorCurrent[3]-avg;
	Serial.print(",");
	gMotorCurrent[4]-avg;
	Serial.print(",");
	gMotorCurrent[5]-avg;
	Serial.println(" ");
	
			}


//***************************************************

/*
void GetTemperature(void)
{
  static int driver = 0;
  int temp = 0;

  //Gets temps on cycle - for driver one and two first time, then three and four on second run etc.

  //check and process timeout only on first call
  Serial.print("L:3: Temperature Data:");
  Serial.print(driver);
  Serial.print(":");
  temp = gSabertooth[driver].getTemperature(1, false);
  if (temp == MOTOR_TIMEOUT)
  {
    SendBeep(50, 1);
    Serial.println(" TIMEOUT");
  }
  else
  {
    Serial.print(temp);
    Serial.print(":");
    Serial.println(gSabertooth[driver].getTemperature(2, false));
  }
  driver ++;
  if (driver > 2) driver = 0;
}
*/
//***************************************************
bool GetDynamicMode(void)
{
	/* Get Dynamic Mode looks at the keyswitch and sets the global variable gDynBrakeActive
	*/
	
	bool temp = !digitalRead(DYN_KEY_SW);   //key pulls line low to activate DYn Mode.
	if (gDynBrakeActive != temp)
	{
		Serial.print("L:4: Dyn Mode ");
		if(temp)
		{
			Serial.println("ON");
		}
		else
		{
			Serial.println("OFF");
		}
	  gDynBrakeActive = temp;
	}
}
//***************************************************
int GetDirection(void)
{

  /* GetDirection looks more complicated than it should - but there is a safety reason for this.
     We need to be absolutely sure we are carrying out the users wishes. There could be a fault in
     the system resulting in two directions reading true at the same time - whichever we check first
     could carry the day and lead to disaster. So the only way is to look at all inputs together
     and determine a) system is functioning normally and b) there is only one valid direction bit set

  */
  int currentDir = gDirection;

  if ((digitalRead(PIN_DIR_FWD) == LOW) && (digitalRead(PIN_DIR_REV) == LOW))
  {
    //cant have both directions low at the same time - error
    gDirection = DIR_NONE;
    CalcControlStatus(STATUS_ERROR, "ERROR: Reverser FWD AND REV");
    return 1;
  }
  else if ((digitalRead(PIN_DIR_FWD) == LOW) && (gDirection == DIR_NONE) && (digitalRead(PIN_DIR_REV) == HIGH))
  {
    gDirection = DIR_FWD;    //We are going forward
    gControlsChanged = true;
    Serial.println("S:f:");
  }
  else if ((digitalRead(PIN_DIR_FWD) == HIGH)  && (gDirection == DIR_NONE) && (digitalRead(PIN_DIR_REV) == LOW))
  {
    gDirection = DIR_REV;    //We are going reverse
    gControlsChanged = true;
    Serial.println("S:r:");
  }
  else if ((digitalRead(PIN_DIR_FWD) == HIGH) && (gDirection != DIR_NONE) && (digitalRead(PIN_DIR_REV) == HIGH))
  {
    gControlsChanged = true;
    gDirection = DIR_NONE;   //We are in Neutral
    Serial.println("S:n:");
  }

  //otherwise business as usual - no change needed - move on through
  return 0;
}

//***************************************************
int GetDynamic(void)
{
  /*read the raw analog value and calculate the corresponding notch
    NOTE: debouncing is somewhat complicated for a deliberate reason:-
    when the lever is advanced several notches over a couple of seconds, we only
    want to know when it has stopped moving. this is to give the motor controllers and sound system a solid
    starting point and ending point for multi-notch accel/decel changes
		This is done in two stages:
		1) A three stage debounce - all values must be identical (i.e. the lever has stopped moving)
		2)Check-For-Nearest Notch in the calcNotch() routine. This removes drift in the analogue reading due to noise/voltage fluctuations

		original Dynamic notches = {740,770,790,825,855,880,915,945,975};	//These were the between settings.

  */
	//																	  0   1   2   3   4   5   6   7   8
	static int notches[MAX_NOTCHES]		= {740,760,795,830,855,885,912,935,965};	//Values set as centre of notches.
  static int debounce[3] 						= {0, 0, 0};
  static int debounce_idx 					= 0;
				 int notch									= 0;

  switch (debounce_idx)
  {
    case 0:
      debounce[0] = analogRead(DYNAMIC_PIN);           //take first analog sample
      debounce[1] = 0;
      debounce[2] = 0;
      debounce_idx++;
      return 0;

    case 1:
      debounce[1] = analogRead(DYNAMIC_PIN);           //take second analog sample
      if (debounce[0] != debounce[1])
        debounce_idx = 0;                            //we are in the middle of a change so reset for three new samples
      else
        debounce_idx++;                              //1st two analog samples match, so advance ready for third
      return 0;

    case 2:
      debounce[2] = analogRead(DYNAMIC_PIN);
      debounce_idx = 0;                                //completed 3 samples. start again either way
      if (debounce[0] != debounce[1] || debounce[0] != debounce[2])
      {
        return 0;                            //Still in the middle of a change so reset for three new samples
      }
      break;
  }

	//Have a debounced sample - now place it away in the array and calculate a new median
	//Then take the median and figure what notch it should be.
	if(abs(gCurrentDynValue - debounce[0]) > 8)
	{
	
		gCurrentDynValue = debounce[0];
	
		notch = CalcNotch(debounce[0],notches);              //success to get here - three analogs match, now make it a notch


		if (gCurrentDynNotch != notch)
		{
			//value has to have changed, and be measured as identical analog values three times to get here
			gControlsChanged = true;
			Serial.print("Current Dynamic Value is: ");
			Serial.println(gCurrentDynValue);

			if (gDirection == DIR_NONE && notch > 0)
			{
				CalcControlStatus(STATUS_ERROR, "ERROR: Dyn Set. Dir Neutral");
				return 1;
			}

			gCurrentDynNotch = notch;
			Serial.print("New Notch is: ");
			Serial.println(notch);

		}
	}
  return 0;
}

//***************************************************
int GetThrottle(void)
{

  //read the raw analog value and calculate the corresponding notch
  /*NOTE: debouncing is somewhat complicated for a deliberate reason:-
    when the lever is advanced several notches over a couple of seconds, we only
    want to know when it has stopped moving. this is to give the motor controllers and sound system a solid
    starting point and ending point for multi-notch accel/decel changes
		This is done in two stages:
		1) A three stage debounce - all values must be identical (i.e. the lever has stopped moving)
		2)Check-For-Nearest Notch in the calcNotch() routine. This removes drift in the analogue reading due to noise/voltage fluctuations

  */

	//																	  0   1   2   3   4   5   6   7   8
	static int notches[MAX_NOTCHES]		= {740,760,795,830,855,885,912,935,965};	//Values set as centre of notches.
	static int debounce[3] 						= {0, 0, 0};
 	static int debounce_idx 					= 0;
				 int notch									= 0;
				 int dyn_ref								= 0;
	
  switch (debounce_idx)
  {
    case 0:
      debounce[0] = analogRead(THROTTLE_PIN);        //take first analog sample
      debounce[1] = 0;															 //make sure the other two samples are zero'd out
      debounce[2] = 0;
      debounce_idx++;
      return 0;

    case 1:
       debounce[1] = analogRead(THROTTLE_PIN);        //take second analog sample
      if (debounce[0] != debounce[1])
        debounce_idx = 0;                            //we are in the middle of a change so reset for three new samples
      else
        debounce_idx++;                              //1st two analog samples match, so advance ready for third
      return 0;

    case 2:
      debounce[2] = analogRead(THROTTLE_PIN);
      debounce_idx = 0;                                //completed 3 samples. start again either way
      if (debounce[0] != debounce[1] || debounce[0] != debounce[2])
      {
        return 0;                            					//Still in the middle of a change so reset for three new samples
      }
      break;																					//Can only get here if all samples match
  }

	//Have a debounced sample - now place it away in the array and calculate a new median
	//Then take the median and figure what notch it should be.
	if(abs(gCurrentThrValue - debounce[0]) > 8)
	{
	
		gCurrentThrValue = debounce[0];     //All the debounce samples are the same so just grab the first one
		
		notch = CalcNotch(gCurrentThrValue,notches);      //success to get here - three analogs match, now make it a notch

		if (gCurrentThrNotch != notch)       //has notch changed?
		{
			//change has happened, and settled down sufficient to get three identical readings in order to get here

			gControlsChanged = true;
			Serial.print("Current Throttle Value is: ");
			Serial.println(gCurrentThrValue);

			if (gDirection == DIR_NONE && notch > 0)
			{
				CalcControlStatus(STATUS_ERROR, "ERROR: Throttle Set. Dir Neutral");
				return 1;
			}

			gCurrentThrNotch = notch;
			Serial.print("New Notch is: ");
			Serial.println(notch);

		}
	}
  return 0;
}

//***************************************************
//Serial Config & Handling routines
//***************************************************

void ConfigComms(bool IsReboot)
{
  //Configures two RS232 data Streams - one for Debug/Raspi Comms and the other to talk to the motor controllers

  gRaspi_RxBuffer.reserve(SERIAL_BUFSIZE);
  gRaspi_TxBuffer.reserve(SERIAL_BUFSIZE);


  //initialize Comms
  Serial.begin(RASPI_SERIAL_SPEED);  //Comms to Debug/Raspi
  //SWSerial.begin(MOTOR_SERIAL_SPEED);  //Motor Controller comms
  Serial1.begin(MOTOR_SERIAL_SPEED);  //Motor Controller comms
  //give the serial lines time to come up before using them - wait three seconds

  if(IsReboot)
  {
    SendBeep(150, 5);    //Will also give a one second delay to allow comms to come up properly
    gDitchFlashCount = 10;
    gtimer.setTimer(400, FlashDitchlights, gDitchFlashCount);

  }
  gSabertooth[0].setGetTimeout(1000);
  gSabertooth[1].setGetTimeout(1000);
  gSabertooth[2].setGetTimeout(1000);
  
  SetMotorRamping(MOTOR_RAMPING);

}

//***************************************************
void ServiceComms(void)
{
  //Check to see if anything has arrived from the Raspi
  if (gRaspi_RxComplete)
  {

    switch (gRaspi_RxBuffer[0])
    {
      case 'H':
      case 'h':
        gRaspiOK = true;                    //set true when we receive heartbeat from raspi
        break;
      case 'R':
      case 'r':
        //allows raspi to force reboot of arduino
        delay(10);
        softReset();
        break;
      default:
        Serial.print("L:4:Bad command rec'd:-");
        Serial.println(gRaspi_RxBuffer);
    }

    // clear the string:
    gRaspi_RxBuffer = "";
    gRaspi_RxComplete = false;
  }
}

//***************************************************
void serialEvent()
{

  /*
    SerialEvent occurs whenever a new data comes in the
    hardware serial RX.  This routine is run between each
    time loop() runs, so using delay inside loop can delay
    response.  Multiple bytes of data may be available.
  */

  //Raspi Serial Receive
  while (Serial.available())
  {
    // get the new byte:
    char inChar = (char)Serial.read();
    // add it to the inputString:
    gRaspi_RxBuffer += inChar;
    // if the incoming character is a newline, set a flag
    // so the main loop can do something about it:
    if (inChar == '\n')
    {
      gRaspi_RxBuffer += '\0';  //append a null to prevent overrun
      gRaspi_RxComplete = true;
    }
  }
}

//***************************************************
void SendCommsHeartbeat(void)
{
#ifdef LOG_LEVEL_4
  //Serial.println("L:4:Heartbeat");
#endif
  gSabertooth[0].keepAlive();
  gSabertooth[1].keepAlive();
  gSabertooth[2].keepAlive();
}

//***************************************************
//  Send Beep - beep once specified time
//***************************************************
void SendBeep(unsigned long len, int count)
{
  for (int f = 0; f < count; f++)
  {
    digitalWrite(BUZZER_PIN, HIGH);  //buzzer on
    delay(len); //wait
    digitalWrite(BUZZER_PIN, LOW);  //buzzer off
    delay(len); //wait
  }
}
//***************************************************
// Configuration, Test & Error handling routines
//***************************************************

//special routine to allow the arduino to reset itself in code.
void softReset()
{
  asm volatile ("  jmp 0");
}

//***************************************************
void ConfigDigitalPins(void)
{

  /*Config digital pins - for digital Devices attached to the Arduino:
    NB: All other devices pins are configured by their own device specific initialization code

    INPUTS
  	Reverser 		- 2 x microswitches  - internal pullups - active low
  	Speedo			- IR interrupted beam for pickup  - internal pullup - active low
  	Horn			- Pushbutton with lever  - internal pullup - active low
  	Key Switch	  - Key switch to toggle Dynamic Braking on or off
	Headlights		- Switch off/low/high - uses analog inputs
   	Vigilance Reset	- pushbutton active high - only works when vigilance is enabled


    OUTPUTS
  	Buzzer			- used for warning buzzer & light (vigilance, serious error etc)
  	Vigilance Enable - turns on the LED in the vigilance button and enables the Vigilance reset button

  */

  pinMode(PIN_DIR_FWD, INPUT);			//Set pin for input
  digitalWrite(PIN_DIR_FWD, HIGH);		//enable internal pullup

  pinMode(PIN_DIR_REV, INPUT);			//Set pin for input
  digitalWrite(PIN_DIR_REV, HIGH);		//enable internal pullup

  pinMode(TACHO_IN, INPUT);				//Set pin for input
  digitalWrite(TACHO_IN, HIGH);			//enable internal pullup

  pinMode(BTN_HORN, INPUT);				//Set pin for input
  digitalWrite(BTN_HORN, HIGH);			//enable internal pullup

  pinMode(DYN_KEY_SW, INPUT);			//Set pin for input
  digitalWrite(DYN_KEY_SW, HIGH);		//enable internal pullup

  pinMode(BTN_VIGILANCE, INPUT);		//Set pin for input  - remember it is active HIGH

  pinMode(HEADLOW_PIN, INPUT);			//Set pin for input
  digitalWrite(HEADLOW_PIN, HIGH);		//enable internal pullup

  pinMode(HEADHIGH_PIN, INPUT);			//Set pin for input
  digitalWrite(HEADHIGH_PIN, HIGH);		//enable internal pullup

  pinMode(BUZZER_PIN, OUTPUT);			//Set pin for output

  pinMode(VIGILANCE_EN, OUTPUT);		//Set pin for output

}

//***************************************************
void initSystem(void)
{
  //set all the values by calling all the relevant subroutines
  bool DoneOnce = false;

  gCurrentThrNotch = 0;
	gCurrentDynNotch = 0;

  SetMotorSpeed();                   //Force all motors to stop - issue commands before checking controllers are there for safety.

  //Ensure motor drivers are in a known minimal state
  //    GetTemperature();         //get and send to the raspi
  CalcMotorNotchSize();
  GetDynamicMode();
  GetBattery();
  CheckHeadlights();

  do {

    //Loop here until all controls are in the safe position
    // Reverser in Neutral. Throttle and Dynamic in idle
    //remember they are mechanically interlocked so really only have to check Dir NONE
    //  - but others are checked in case of faults which could cause unpredictable operation
    //note Buzzer is used to indicate the controls are not setup correctly

    ReadInputs();
    if ((gDirection == DIR_NONE) && (gCurrentThrNotch == 0) && (gCurrentDynNotch == 0))
    {
      //controls are safe and no errors detected with them
      gInitialized = true;
      ResetVigilanceWarning();
      Serial.println(" ");
			Serial.print("Zero throttle is: ");
			Serial.println(analogRead(THROTTLE_PIN));
			Serial.print("Zero Dynamic is: ");
			Serial.println(analogRead(DYNAMIC_PIN));
      CalcControlStatus(STATUS_IDLE, "Init OK");
    }
    else
    {
      if(!DoneOnce)
      {
        if (gDirection != DIR_NONE)
        {
          CalcControlStatus(STATUS_ERROR, "INIT:Reverser not Neutral");
        }
        if (gCurrentThrNotch != 0)
        {
          CalcControlStatus(STATUS_ERROR, "INIT:Throttle NOT ZERO");
        }
        if (gCurrentDynNotch != 0)
        {
          CalcControlStatus(STATUS_ERROR, "INIT:Dynamic NOT ZERO");
        }
        DoneOnce = true;	//only want to send the message once so debug screen is usable. Keep beeping tho!
      }
      SendBeep(1000, 1);

      gControlsChanged = 0;
    }
  } while (!gInitialized);

  //signal all OK
  SendBeep(200, 2);

}


//***************************************************


