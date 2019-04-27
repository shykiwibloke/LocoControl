/****************************************************
**                                                 **
**  Loco Control Stand version                     **/
#define VERSION "3.3.3"
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
 *  3.3.3
25/04/2019: Rewrite of Amperage measurement to Modified Moving Average method (Measurements were too volitile)
25/04/2019: Modified Serial Speed from 9600 to 38400 for motor controllers
 *  3.3.2
17/04/2019: Fixed bug in amperage fetch routine
17/04/2019: Increased serial speed to Raspberry Pi to 115200
 *  3.3.0
18/12/2018: Modified direction switch detection to avoid it ignoring fast changes between reverse and forward.
13/12/2018: Initial cut of Dynamic Mode coded in to GetThrottle, GetDynamic, SetDynamicBrake, and Evaluate state subroutines
 *  3.2.3
 3/12/2018:	trying debounce using 1/4 second - Great on throttle, not on Horn.
 3/12/2018: Debugged SetMotorSpeed() so it handles motor amperage variances properly
 *  3.2.2
 3/12/2018: Median calculation for current written. Calling of GetMotorAmpsVolts changed to 1/4 second.
						Check horn & lights changed to 1/4 second check - hoping to assist debounce
 *  3.2.1
 2/12/2018: Encoders and new switches integrated into existing code
 2/12/2018: Code refactoring including introducing the 1/4 second timer and some debouncing
						Debounce put on lightswitch to assist in preventing false zeros
						MotorsOn and DynamicEnabled switches tested OK & integrated into code
 *	3.2.0
13/10/2018: Begin major changes as follows:
	Add new switch - MOTOR OUTPUT ENABLE - which disables only motor control changes if false. Useful for testing
	Replace throttle/Dynamic analog pots with Absolute digital encoders
	Respecify pin assignments to match new mega shield with plug couplers for all devices
	Modify Serial Port configurations to replace radio link to motor controllers
	Remove Speedo servo driver (will use touchscreen to display speed)
	Remove Motor Controller Temperature check routine (never used in practice)
	Change Buzzer to PWM operation
	General relayout of the defines and code routines to better match current usage
13/10/2018: Earlier software changelog removed - see archived version 3.1.0 for details of earlier changes

End Recent Changes Log  */

#include <USBSabertooth.h>    //comms library for sabertooth controllers
#include <SimpleTimer.h>      //used for vigilance and heartbeat type functions

const byte gEncoderMap[256] = {
  0xFF,0x38,0x28,0x37,0x18,0xFF,0x27,0x34,0x08,0x39,0xFF,0xFF,0x17,0xFF,0x24,0x0D,
  0x78,0xFF,0x29,0x36,0xFF,0xFF,0xFF,0x35,0x07,0xFF,0xFF,0xFF,0x14,0x13,0x7D,0x12,
  0x68,0x69,0xFF,0xFF,0x19,0x6A,0x26,0xFF,0xFF,0x3A,0xFF,0xFF,0xFF,0xFF,0x25,0x0E,
  0x77,0x76,0xFF,0xFF,0xFF,0x6B,0xFF,0xFF,0x04,0xFF,0x03,0xFF,0x6D,0x6C,0x02,0x01,
  0x58,0xFF,0x59,0xFF,0xFF,0xFF,0xFF,0x33,0x09,0x0A,0x5A,0xFF,0x16,0x0B,0xFF,0x0C,
  0xFF,0xFF,0x2A,0x2B,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0x15,0xFF,0x7E,0x7F,
  0x67,0xFF,0x66,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0x5B,0xFF,0xFF,0xFF,0xFF,0xFF,
  0x74,0x75,0xFF,0xFF,0x73,0xFF,0xFF,0xFF,0x5D,0x5E,0x5C,0xFF,0x72,0x5F,0x71,0x00,
  0x48,0x47,0xFF,0x44,0x49,0xFF,0xFF,0x1D,0xFF,0x46,0xFF,0x45,0xFF,0xFF,0x23,0x22,
  0x79,0xFF,0x7A,0xFF,0x4A,0xFF,0xFF,0x1E,0x06,0xFF,0x7B,0xFF,0xFF,0xFF,0x7C,0x11,
  0xFF,0xFF,0xFF,0x43,0x1A,0xFF,0x1B,0x1C,0xFF,0x3B,0xFF,0xFF,0xFF,0xFF,0xFF,0x0F,
  0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0x05,0xFF,0xFF,0xFF,0x6E,0xFF,0x6F,0x10,
  0x57,0x54,0xFF,0x2D,0x56,0x55,0xFF,0x32,0xFF,0xFF,0xFF,0x2E,0xFF,0xFF,0xFF,0x21,
  0xFF,0x53,0xFF,0x2C,0x4B,0xFF,0xFF,0x1F,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0x20,
  0x64,0x3D,0x65,0x42,0xFF,0x3E,0xFF,0x31,0x63,0x3C,0xFF,0x2F,0xFF,0xFF,0xFF,0x30,
  0x4D,0x52,0x4E,0x41,0x4C,0x3F,0xFF,0x40,0x62,0x51,0x4F,0x50,0x61,0x60,0x70,0xFF};

//***************************************************
//Pin Use Definitions
//***************************************************

//Explanation

/* Devices attached to the Arduino are:
  Throttle        - Absolute Encoder on databus with select pin
  Dynamic Brake 	- Absolute Encoder on databus with select pin
  TrainWeight     - 10k pot
  Reverser        - 2 x microswitches
  Speedo          - IR interrupted beam or Hall effect for pickup
  Horn            - Pushbutton with lever
  Switch	  			- Switch to toggle Dynamic Braking on or off - when off it is in simple control mode
	Switch					- Switch to toggle Motor output on or off (Used for debugging)
  Headlights      - Switch OFF/LOW/HIGH (Ditch lights steady/flashing handled by software)
  Vigilance Reset	- pushbutton input
  Buzzer          - used for warning buzzer (vigilance, serious error etc)
  Serial (USB)		- Serial (debug) interface + connection to Raspi for sound
	Serial1					- Sabertooth I/F5v- Serial interface
  
*/

//**************************************************
//Analogue I/O Map

#define MAX_SPEED_PIN 	A0			//A0 MaxSpeed (Train max speed bias) 10k Potentiometer
//A1-A15 spare
//**************************************************
//Digital I/O Map

//D0&D1 	- reserved for RS232 Serial via USB Serial (Programming port) + Raspi
#define TACHO_IN_PIN		2       //D2  - Configured as Interrupt 4 - input for tachometer (for IR/hall effect sensor)
//D3&D4 spare
#define BUZZER_PIN			5      	//D5 - Buzzer in console
#define HEADLOW_PIN   	6       //D7 - Headlights to low power
#define HEADHIGH_PIN  	7       //D6 - Headlights to high power
#define BTN_HORN_PIN		8       //D8  - Input switch on console
#define VIGIL_EN_PIN    9       //D9 - Vigilance Button Enable + the lights on board LED
#define BTN_VIGIL_PIN		10      //D10 - pushbutton on console - note input is active HIGH
#define DYN_SW_PIN			11			//D11  - Key switch used to toggle Dynamic Braking Mode
#define MOTOR_SW_PIN		12			//D12 - Motor switch - if true, enable motor outputs, false for testing
//D13 -15 spare
//D16&D17 - reserved for RS232 Serial2 (Reserved for future comms)
//D18&D19 - reserved for RS232 Serial1 (Sabertooth Motor Controllers)
//D20-41 spare
#define DIR_FWD_PIN 		51      //Microswitch - internal pullup - active low Pin 51 on Mega is PB0
#define DIR_REV_PIN 		53      //Microswitch - internal pullup - active low Pin 53 on Mega is PB2
#define THR_SELECT_PIN	52			//Throttle Encoder Select Pin   - active low Pin 52 on Mega is PB1
#define DYN_SELECT_PIN  50			//Dynamic Brake Encoder Select  - active low Pin 50 on Mega is PB0
#define ENCODER_IN			PINL		//Port L is used for 8 bits of data (L0-L7)
#define ENCODER_DIR_REG	DDRL		//Port L Data Direction Register 0 - input, B11111111 = output
#define ENCODER_PULLUP	PORTL		//Port L reg that is used for output and pullups - set to B11111111 = all pulled up

/*
Arduino Shield Jackpoint Wiring Summary 
(Pin 1 on all jacks marked in RED)
J1 - 4 pin for Panel Switches 							(+5v, Gnd, MOTOR_SW_PIN, DYN_SW_PIN)
J2 - 4 pin for Vigil button, light + input 	(+5v, Gnd, BTN_VIGIL_PIN, VIGIL_EN_PIN)
J3 - 4 pin for Horn Button 									(+5v, Gnd, Horn input, MaxSpeed input)
J4 - 4 pin for Tacho 												(+5v, GND, BUZZER_PIN, TACHO_IN_PIN)
J5 - 4 pin for Headlight 										(spare, Gnd, HEADLOW_PIN, HEADHIGH_PIN)
J6 - 4 pin for comms to loco 								(TX2,RX2,TX1,RX1)
J7 - 16 pin header to cover direction and encoders
*/

//***************************************************
// General Defines & Constants
//***************************************************

//#define LOG_LEVEL_4								//if defined then debug info will be output to the console port
#define PORT_DIR_INPUT			0				//Constant used to set Direction Register for input use (all 8 bits at once)
#define PORT_DIR_OUTPUT 		255			//Constant used to set Direction Register for output use (all 8 buts at once)
#define PORT_ALL_PULLUP			255			//Constant used to set all 8 internal pullup resistors for port 

#define MOTOR_SERIAL_SPEED	38400      //for Motor Controllers
#define RASPI_SERIAL_SPEED 	115200		//faster speed over USB link
#define MAX_NOTCHES					9					//allow for zero and 8 notches - Applies to Throttle and Dynamic
#define MAX_NOTCH_SIZE      255       //the maximum value we can send for throttle or dynamic brake per notch position - 1

#define HEADLIGHT_ST				2					//Sabertooth port defines for ditch and head lights
#define DITCHLIGHT_R 				1
#define DITCHLIGHT_L				0
#define LIGHT_OFF						-2000			//Brightness defines for PWM controlled headlights
#define LIGHT_HALF					-500
#define LIGHT_FULL					2000

//Status Definitions

const int STATUS_ERROR 		= 99;
const int STATUS_IDLE 		= 10;
const int STATUS_POWER 		= 20;
const int STATUS_DYNAMIC 	= 30;
const int STATUS_COAST		=	40;				//Used when throttle is closed AND Dynamic is Enabled

const int DIR_NONE 				= 0;
const int DIR_FWD 				= 1;
const int DIR_REV 				= 2;

//Sabertooth configuration defines
const int MOTOR_RAMPING =	2400;				//smooths transition between notches
const int MOTOR_TIMEOUT = -32768;			//value returned by Sabertooth library when comms fails. Must be const int to work reliably.

//***************************************************
// Global Variable Definitions
//***************************************************

// Global State Variables 

bool gInitialized       	= false;
bool gRaspiOK           	= false;
bool gControlsChanged   	= false;
int  gControlStatus     	= 0;
bool gGetMotorStatusFlag 	= false;
bool gMotorsEnabled				= false;							//True if Motor output to the motor controllers switch is ON
bool gDynamicEnabled			= false;							//True if Dynamic Brake selection switch is ON
int  gThrZeroOffset				= 0;									//Throttle Encoder Offset. Evaluated when reverser in neutral
int  gDynZeroOffset				= 0;									//Dynamic Encoder Offset. Evaluated when reverser in neutral

// Control Inputs
int  gCurrentThrValue			= 0;																	//Array to hold samples for moving Median Calculation
int  gCurrentThrNotch     = 0;																	//Notch currently set for throttle
int  gDynMaxThrNotch			= 0;																	  //Throttle max under dynamic mode for coasting.
int  gCurrentDynValue			= 0;																	//Array to hold samples for moving Median Calculation
int  gCurrentDynNotch			= 0;																	//Notch currently set for Dynamic															
int  gNotchSize						= 0;																	//Notch size - to scale output as directed by CalcMotorNotchSize()
int  gDirection         	= 0;																	//Forward, Reverse, Neutral ( See DIR_NONE, DIR_FWD, DIR_REV)
int  gBattery           	= 0;																	//Current Battery Voltage
int	 gHorn              	= 0;																	//Horn sounding 0 = no, 1 = yes
int  gHeadlights        	= 0;																	//Headlights off, dip, full (0,1,2 respectively)
int  gMotorAmps[6]  		  = {0};																  //last read motor Amperage for individual motorsint  gMotorAmps[6]  		= {0};																//last read motor current for individual motors
//int  gMotorAmpVariance[6] = {0};																//last calculated speed offset for motors
int  gMotorValue[6]				= {0};																//Contains the last set of values output to motors
int	 gVigilanceCount    	= 0;																	//Timer count for the Vigilance Alarm
int  gDitchFlashCount   	= 0;																	//Count for flashing the ditch lights when the horn is sounded

//***************************************************
//Define Timer
SimpleTimer gtimer;                         //Only need one timer as all timing callbacks can be handled by this one object

//***************************************************
//define serial port for motor comms here

USBSabertoothSerial gSP(Serial1);   //Assign to Mega Hardware serial port one

//there are three motor controllers each with two motors. arrange these so they are front to rear in numerical order
USBSabertooth       gSabertooth[3] = { USBSabertooth(gSP, 128), USBSabertooth(gSP, 129), USBSabertooth(gSP, 130) };

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

	//Announce Our arrival to USB port AND Raspi
  Serial.print("L:1:Loco Control Stand Ver: ");
  Serial.println(VERSION);

  //call various subroutines to ensure internal state matches control panel
  initSystem();

  //Start timer - used to check slow controls and heartbeat comms every fifteen seconds
  gtimer.setInterval(15000, DoFifteenSecondChecks);
	gtimer.setInterval(250, DoQuarterSecondChecks);

}

//***************************************************
// Main loop & Core Subroutines
//***************************************************

//Main loop must be executed frequently for system to run reliably
// (i.e. no long Delay() type statements in subroutines unless deliberately done)

void loop()
{
	
  //Serial.println(gSabertooth[2].getCurrent(1));
	//Serial.println(gSabertooth[1].getCurrent(1));
	//Serial.println(gSabertooth[0].getCurrent(1));
	
  //ReadInputs();                   //Read input variables

  if (gControlsChanged)           //only process if controls have changed
  {

    ResetVigilanceWarning();    //Control movement resets vigilance
    CalcMotorNotchSize();       //check to see if user has changed the notch size before we use it
    EvaluateState();            //check to see if the control changes mean we are in a new state
																//and issue commands based on the changed controls - and also keep-alive heartbeats
  }
  else 
  {
		CheckHorn();                      	//if horn is pressed, then sound it. if released, then tell raspi
		GetMotorAmpsVolts();              //fetch and send one motor's current per call, and battery voltage once per 6 motor checks.

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

 
  if (digitalRead(BTN_VIGIL_PIN))   	//Hitting vigilance button is considered a control change
  {
    ResetVigilanceWarning();        	//Just reset vigilance - not a control 'change' that needs further analysis
  }
  
  if (gControlStatus == STATUS_IDLE)   //only want to evaluate these items if the loco is at a standstill
  {
	  GetDynamicMode();
		GetMotorMode();
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

  if (gControlStatus == STATUS_ERROR)   //Error state can be set by anywhere in the program
  {
    //Stay there until the error clears
    Serial.println("L:1:EvalutateState Error Condition. Place Controls to Idle to clear");
    SetVigilanceWarning(true);
    delay(50);
    ResetVigilanceWarning();
    if ((gDirection == DIR_NONE) || (gCurrentThrNotch == 0) && (gCurrentDynNotch == 0))
    {
      //controls are safe and no errors detected with them
      gInitialized = true;
      ResetVigilanceWarning();
      CalcControlStatus(STATUS_IDLE, "Controls Now at Idle");
      SetMotorSpeed();																			//Ensures motors are stopped when error is cleared
    }
  }
  else  
  {
		//Not an error condition - control readings are good to evaluate
		
		if(gDirection == DIR_NONE || ((gCurrentThrNotch == 0) && (gCurrentDynNotch == 0) && gControlStatus != STATUS_COAST))
		{
			//IDLE if the reverser is in neutral or (Throttle and Dynamic are both zero AND we are not coasting)
			//This also covers non dynamic mode where levers are zero 
			//as well as dynamic mode where throttle as become zero (Coast would be cancelled)
			
			CalcControlStatus(STATUS_IDLE, "Idle");
			SetMotorSpeed();			//ensure speed is indeed set to zero
		}
		else if(gDynamicEnabled && gControlStatus == STATUS_COAST)			//We are in dynamic mode - throttle lever reacts differently
		{
	
			//check for increased amperage and reduce throttle accordingly
			//(could be hill or air brake - either of which should reduce speed to zero over time
			
			
			//TODO - take a snapshot of current amperage readings, and continue to use them as 'centre' for changing throttle
			
			//nothing else do here right now - leave throttle as-is, 
			//one of two three things can happen
			// 1. loco drifts until dynamic adanced.
			// 2. loco drifts until throttle advanced past last max.
			// 3. loco drifts until air brake applied
			
			//sound will take care of itself.
	
		}
		else if(gDynamicEnabled && gCurrentDynNotch > 0)
		{
			//Dynamic Mode as dynamic lever is advanced. - use dynamic brake
			CalcControlStatus(STATUS_DYNAMIC,"Dynamic Active");
			
			//TODO - take a snapshot of current amperage readings, and continue to use them as 'centre' for changing throttle
			
			SetDynamicBrake();
		}
		else if (!gDynamicEnabled || (gCurrentThrNotch > gDynMaxThrNotch))
		{
			//No Dynamic - simple throttle OR the current notch is past what we thought was the highest until now

       CalcControlStatus(STATUS_POWER, "Throttle Active");
			 SetMotorSpeed();
 
		}
		else
			Serial.println("Unhandled Throttle State");
	}
	
	//Send Engine Sound Command - will work for coast, throttle, dynamic, or Error modes
        Serial.print("S:");
        Serial.print(gCurrentThrNotch);
        Serial.println(":");
	
	gControlsChanged = false;     //reset the changed flag as we have just handled it.
		 
}
		//<><><><><><><><><><><><><><><><><><><><><><???????????
		/*
		else if (gDirection != DIR_NONE)
    {
      //Direction has been set to forward or reverse - dont actually care which here.

      if (gCurrentDynNotch > 0 && gDynamicEnabled == true)
      {
        CalcControlStatus(STATUS_DYNAMIC, "Dyn Brake ON");

        Serial.print("S:");
        Serial.print(gCurrentDynNotch);
        Serial.println(":");
		    SetDynamicBrake();    //go adjust motors into proper dynamic notch
      }
      else if (gCurrentThrNotch > 0)
      {
        int old = gControlStatus;
        CalcControlStatus(STATUS_POWER, "Throttle ON");
      }
  	
		//TODO - replace this chunk with check for STATUS_COAST - which would have been set when the trottle was reduced
		//even to zero
		
		if (gDynamicEnabled == true)
		{
			//if we are in dynamic mode - need to 'coast' with throttle closed
			//do the calculations for coasting here.
			//Serial.print("Dynamic working area");
			
			//**TODO** WORKING HERE **
			//(note no calcs means code just drops through and works as is now)
		}
	
		
				
				
				//************************************************************
				//TODO - needs to account for coast mode and dynamic enabled...
				
				//do this by splitting SetMotorSpeed in two - SetThrottleSpeed and SetMotorSpeed
				//SetThrottleSpeed gets avoided and SetDynamicBrake gets called 
				//in coast or dynamic mode (which knows how to decrement existing speed gradually
				
				//SetThrottleSpeed() (which calls SetMotorSpeed)
        SetMotorSpeed();   //go adjust motors
        
    }
  }
	*/

 


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
		
		if (NewStatus == STATUS_COAST)
			Serial.println("L:4:Coasting");

  }
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
void DoQuarterSecondChecks(void)
{
	//Stuff that needs to be checked regularly to appear realtime, but also has a debounce component

  ReadInputs();                   //Read input variables
	CheckHeadlights();

}
//***************************************************
void DoFifteenSecondChecks(void)
{
  //Stuff that only needs to be checked every fifteen seconds or so Called by timer routine
  
  GetBattery();                 //Get the battery voltage

	gGetMotorStatusFlag = true;   //set flag to get another round of measurements - if the serial buffers are empty
   
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
	
  //Calculate output values including variations to compensate for motor differences
	if(speed == 0)
	{
		for(int f = 0; f < 6; f++)
		{
			gMotorValue[f] = 0;
	//		gMotorAmpVariance[f] = 0;					//Dampen out any values likely to interfere with zeroing motors.
		}
	}
	else
	{
		for(int f = 0; f < 6; f++)
		{
			gMotorValue[f] = speed; // + gMotorAmpVariance[f];
		}
	}
	//send the commands to the motor controllers if enabled
	if (!gMotorsEnabled)
	{	
		Serial.println("** Motors Disabled No Output Sent **");
		Serial.print("raw speed seed = ");
		Serial.println(speed);
		if(speed != 0)							//Dont print all of this if speed is currently zero
		{
			Serial.println("Theoretical Motor Speeds");
			Serial.println(gMotorValue[0]);
			Serial.println(gMotorValue[1]);
			Serial.println(gMotorValue[2]);
			Serial.println(gMotorValue[3]);
			Serial.println(gMotorValue[4]);
			Serial.println(gMotorValue[5]);
			Serial.println("**************");
		}
	}
	else    //Set the real speed
	{
		gSabertooth[0].motor(1, gMotorValue[0]);     //first pair
		gSabertooth[0].motor(2, gMotorValue[1]);
		gSabertooth[1].motor(1, gMotorValue[2]);     //2nd pair
		gSabertooth[1].motor(2, gMotorValue[3]);
		gSabertooth[2].motor(1, gMotorValue[4]);     //3rd pair
		gSabertooth[2].motor(2, gMotorValue[5]);
	}
}

//***************************************************

void SetDynamicBrake()
{

  int brake = (8 - gCurrentDynNotch) * gNotchSize;          //used to scale the provided notch to a speed setting the controller understands (-2047-0-2047 range)
  //Note the notch is inverted so most gentle braking is on Notch one

  if (gDirection == DIR_REV)  brake = -brake;     //a negative number makes the motor go in reverse, positive = go forward
  //Note this must stay in the same direction of travel for Sabertooth braking to work properly

  //Calculate output values including variations to compensate for motor differences
	for(int f = 0; f < 6; f++) 
		gMotorValue[f] -= brake;
	 
  if (!gMotorsEnabled)
	{	
		Serial.println("** Motors Disabled No Dynamic Sent **");
		Serial.print("raw brake seed = ");
		Serial.println(brake);
	}
	else    //calculate and set the real speed
	{
		gSabertooth[0].motor(1, gMotorValue[0]);     //first pair
		gSabertooth[0].motor(2, gMotorValue[1]);
		gSabertooth[1].motor(1, gMotorValue[2]);     //2nd pair
		gSabertooth[1].motor(2, gMotorValue[3]);
		gSabertooth[2].motor(1, gMotorValue[4]);     //3rd pair
		gSabertooth[2].motor(2, gMotorValue[5]);
	}

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
	
	static int debL[3] 						= {0, 0, 0};
 	static int debH[3] 						= {0, 0, 0};
	static int debounce_idx 			= 0;
				 int state 							= 4;

	if(gDitchFlashCount < 2)											//Dont check headlights if they are flashing- upsets stuff
	{
					 
		switch (debounce_idx)
		{
			case 0:

			debL[0] = digitalRead(HEADLOW_PIN);        //take first pin sample
				debH[0] = digitalRead(HEADHIGH_PIN);			 //take second pin sample
				debL[1] = 0;															 		//make sure the other two samples are zero'd out
				debL[2] = 0;
				debH[1] = 0;															 		//make sure the other two samples are zero'd out
				debH[2] = 0;
				debounce_idx++;
				return;
				
			case 1:
				debL[1] = digitalRead(HEADLOW_PIN);        //take first pin sample
				debH[1] = digitalRead(HEADHIGH_PIN);			 //take second pin sample
				if (debL[0] != debL[1] || debH[0] != debH[1])
					debounce_idx = 0;                            					//Still in the middle of a change so reset for three new samples
				else
					debounce_idx++;                              //1st two analog samples match, so advance ready for third
				return;

			case 2:

				debL[2] = digitalRead(HEADLOW_PIN);        //take first pin sample
				debH[2] = digitalRead(HEADHIGH_PIN);			 //take second pin sample
				debounce_idx = 0;                            					//Reset for three new samples whatever happens
				if (debL[0] != debL[2] || debH[0] != debH[2])
				{
					return;
				}
				break;	//All ok to go to next stage
		}

		
		if (debL[0] == 0) state = 1;
		if (debH[0] == 0) state = 2;
		if (debL[0] == 1 && debH[0] == 1) state = 0;

		if (gHeadlights != state)
		{
			switch (state)
			{
				case 0:  //headlight off
					gSabertooth[HEADLIGHT_ST].power(LIGHT_OFF);
					gSabertooth[DITCHLIGHT_L].power(LIGHT_OFF);
					gSabertooth[DITCHLIGHT_R].power(LIGHT_OFF);
					Serial.println("L:2:lights off");
					break;
				case 1:  //headlight on dip
					gSabertooth[HEADLIGHT_ST].power(LIGHT_HALF);
					gSabertooth[DITCHLIGHT_L].power(LIGHT_HALF);
					gSabertooth[DITCHLIGHT_R].power(LIGHT_HALF);
					Serial.println("L:2:lights dip");
					break;
				case 2: //headlight on full
					gSabertooth[HEADLIGHT_ST].power(LIGHT_FULL);
					gSabertooth[DITCHLIGHT_L].power(LIGHT_FULL);
					gSabertooth[DITCHLIGHT_R].power(LIGHT_FULL);
					Serial.println("L:2:lights full");
					break;
				case 4:  //not a valid state - ignore 
					return;

			}

    gHeadlights = state;
		}
	}
}

//***************************************************
void FlashDitchlights(void)
{
  //Called by timer callback setup within CheckHorn()

  if (--gDitchFlashCount < 2)
  {
    //restore both ditch lights to whatever they were by invalidating current position
    gHeadlights = 3;  //An invalid value. CheckHeadlights will reset
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

void CheckHorn(void)
{

  static int newval[3] = {0, 0, 0};
  static int idx = 0;

  switch (idx)
  {
    case 0:
      newval[0] = digitalRead(BTN_HORN_PIN);
      idx++;                              //1st analog sample
      return;
    case 1:
      newval[1] = digitalRead(BTN_HORN_PIN);
      if (newval[0] != newval[1])
        idx = 0;                            //we are in the middle of a change so reset for three new samples
      else
        idx++;                              //1st two analog samples match, so advance ready for third
      return;
    case 2:
      newval[2] = digitalRead(BTN_HORN_PIN);
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

  digitalWrite(VIGIL_EN_PIN, HIGH);

  if (SoundBuzzer) digitalWrite(BUZZER_PIN, HIGH);

}

//***************************************************
void ResetVigilanceWarning(void)
{
  //Reset Vigilance buzzer and light the vigilance LED

  digitalWrite(VIGIL_EN_PIN, LOW);
  digitalWrite(BUZZER_PIN, LOW);
  gVigilanceCount = 0;

}
//***************************************************
//Get State Routines
//***************************************************

void GetMotorAmpsVolts()
{
  
	float movingAverage[6] = {0};				//one for each motor (has to be preserved from one run to the next)
	float movingAverageSum[6] = {0};
	const int averageCount = 200;
				
	static int motor = 0;
  static int ErrCount = 0;
  int      rawvalue = 0;

  //Gets amps on cycle - for motor one, then two on second run etc.
  if(gControlStatus != STATUS_ERROR
		 && gControlStatus != STATUS_IDLE
     && Serial1.availableForWrite() > 50 
     && gGetMotorStatusFlag == true
     && gControlsChanged == false)  //Only request from motor drivers if comms channel has the buffer space to cope 
																		//AND there are no control commands being processed
																		//AND status is something that can use the current measurements (not Error or Idle)
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
						motor = 0;
            ErrCount = 0;
//						CalcMotorAmpVariances();						//Use latest values to ensure motors pulling equally
//            gGetMotorStatusFlag = false;  //done a read through, wait for at least another 15 seconds.
            return;
    }
      
    if(rawvalue == MOTOR_TIMEOUT && gMotorsEnabled) //if motor comms is supposed to be in use and timed out
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
			ErrCount = 0;				//Reset Error count as we got here OK (might have had a timeout and recovered)

			// Remove previous movingAverage from the sum
			movingAverageSum[motor] -= movingAverage[motor];

			// Replace it with the current sample
			movingAverageSum[motor] += rawvalue;

			// Recalculate movingAverage
			movingAverage[motor] = movingAverageSum[motor] / averageCount;

			if ((int) movingAverage[motor] != gMotorAmps[motor])    //Only update and send if value change > 1/10th amp
			{
					gMotorAmps[motor] = (int) movingAverage[motor]; //anything less than 10th of an amp will be zeroed
					Serial.print("M:");
					Serial.print(motor);
					Serial.print(":");
					Serial.println(gMotorAmps[motor]);         //Motor Current sent in 10ths of an amp
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
//MedianTest
//***************************************************
/*
void MedianTest() {

   int a[] = {150,200,0};

   int n = 3;

   int sum,i;

   SortIntArray(a,n);


   n = (n+1) / 2 - 1;  // -1 as array indexing in C starts from 0

   Serial.print("Median = ");

   Serial.println(a[n]);


   delay(5000);

  

}
*/
//***************************************************
/*
void CalcMotorAmpVariances(void)
{
	//Routine to examine last lot of current readings and alter motor offsets so all pull their weight equally	
		
		int median = 0;
		int SortArray[6];
		
		
		memcpy(SortArray,gMotorAmps,sizeof(gMotorAmps));		//take a copy to preserve currents from individual motors
		
		SortIntArray(SortArray,6);						//Sort the six elemtent array into ascending order
		
		median = SortArray[2];								//Use the middle element as the new target  (gMotorAmpVariance[0-5])
		
		if (median > 10 || median < 10)				//Only do this if variance is more than 1 amp - stops jitter
		{
	
			gMotorAmpVariance[0] = median - gMotorAmps[0];			//subtract actual from target to get correct signed integer variance
			gMotorAmpVariance[1] = median - gMotorAmps[1];
			gMotorAmpVariance[2] = median - gMotorAmps[2];
			gMotorAmpVariance[3] = median - gMotorAmps[3];
			gMotorAmpVariance[4] = median - gMotorAmps[4];
			gMotorAmpVariance[5] = median - gMotorAmps[5];

			
			Serial.print("Calculated Variances (median/m1,m2...) ");
			Serial.print(median);
			Serial.print("/");
			Serial.print(gMotorAmpVariance[0]);
			Serial.print(",");
			Serial.print(gMotorAmpVariance[1]);
			Serial.print(",");
			Serial.print(gMotorAmpVariance[2]);
			Serial.print(",");
			Serial.print(gMotorAmpVariance[3]);
			Serial.print(",");
			Serial.print(gMotorAmpVariance[4]);
			Serial.print(",");
			Serial.print(gMotorAmpVariance[5]);
			Serial.println(" ");
		}
	}
*/
			//***************************************************
bool GetMotorMode(void)
{
	/* Get Motor Mode looks at the switch and sets the global variable gMotorsEnabled
	*/
	
	bool temp = !digitalRead(MOTOR_SW_PIN);   //Switch pulls line low to activate DYn Mode.
	if (gMotorsEnabled != temp)
	{
		Serial.print("L:4: Motor Outputs ");
		if(temp)
		{
			Serial.println("Enabled");
		}
		else
		{
			Serial.println("Disabled");
		}
	  gMotorsEnabled = temp;
	}
}

//***************************************************
bool GetDynamicMode(void)
{
	/* Get Dynamic Mode looks at the switch and sets the global variable gDynamicEnabled
	*/
	
	bool temp = !digitalRead(DYN_SW_PIN);   //sw pulls line low to enable Dynamic Mode.
	if (gDynamicEnabled != temp)
	{
		Serial.print("L:4: Dyn Mode ");
		if(temp)
		{
			Serial.println("Enabled");
		}
		else
		{
			Serial.println("Disabled");
		}
	  gDynamicEnabled = temp;
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

  if ((digitalRead(DIR_FWD_PIN) == LOW) && (digitalRead(DIR_REV_PIN) == LOW))
  {
    //cant have both directions low at the same time - error
    gDirection = DIR_NONE;
    CalcControlStatus(STATUS_ERROR, "ERROR: Reverser FWD AND REV");
    return 1;
  }
//  else if ((digitalRead(DIR_FWD_PIN) == LOW) && (gDirection == DIR_NONE) && (digitalRead(DIR_REV_PIN) == HIGH))
  else if ((digitalRead(DIR_FWD_PIN) == LOW) && (digitalRead(DIR_REV_PIN) == HIGH) && (gDirection != DIR_FWD))
  {
		gDirection = DIR_FWD;    //We are going forward
    gControlsChanged = true;
    Serial.println("S:f:");
  }
//  else if ((digitalRead(DIR_FWD_PIN) == HIGH)  && (gDirection == DIR_NONE) && (digitalRead(DIR_REV_PIN) == LOW))
  else if ((digitalRead(DIR_FWD_PIN) == HIGH)  && (digitalRead(DIR_REV_PIN) == LOW) && (gDirection != DIR_REV))
  {
    gDirection = DIR_REV;    //We are going reverse
    gControlsChanged = true;
    Serial.println("S:r:");
  }
  else if ((digitalRead(DIR_FWD_PIN) == HIGH) && (digitalRead(DIR_REV_PIN) == HIGH) && (gDirection != DIR_NONE))
  {
    gControlsChanged = true;
    gDirection = DIR_NONE;   //We are in now in Neutral
    Serial.println("S:n:");
  }

  //otherwise business as usual - no change needed - move on through
  return 0;
}

//***************************************************
int GetDynamic(void)
{

  //read the raw Encoder value and calculate the corresponding notch
  /*NOTE: debouncing is somewhat complicated for a deliberate reason:-
    when the lever is advanced several notches over a couple of seconds, we only
    want to know when it has stopped moving. this is to give the motor controllers and sound system a solid
    starting point and ending point for multi-notch accel/decel changes
		This is done using a three stage debounce - all values must be identical (i.e. the lever has stopped moving)
  */

	static int debounce[3] 						= {0, 0, 0};
 	static int debounce_idx 					= 0;
				 int notch									= 0;

  switch (debounce_idx)
  {
    case 0:
      debounce[0] = GetEncoder(DYN_SELECT_PIN);        //take first encoder sample
			if(debounce[0] < 0 ) return -1; 									//check for errors returned - if so - ignore and start again.
      debounce[1] = 0;															 		//make sure the other two samples are zero'd out
      debounce[2] = 0;
			debounce_idx++;
      return 0;

    case 1:
      debounce[1] = GetEncoder(DYN_SELECT_PIN);        //take second analog sample
			if (debounce[0] != debounce[1])
        debounce_idx = 0;                            //we are in the middle of a change or error occurred so reset for three new samples
      else
        debounce_idx++;                              //1st two analog samples match, so advance ready for third
      return 0;

    case 2:
      debounce[2] = GetEncoder(DYN_SELECT_PIN);
      debounce_idx = 0;                                //completed 3 samples. start again either way
      if (debounce[0] != debounce[1] || debounce[0] != debounce[2])
      {
        return 0;                            					//Still in the middle of a change or error occurred so reset for three new samples
      }
      break;																					//Can only get here if all samples match
  }

	gCurrentDynValue = debounce[0] - (gDynZeroOffset);     //All the debounce samples are the same so use the first one
	

	if			(gCurrentDynValue < 3) notch = 0;			//Idle
	else if (gCurrentDynValue < 5) notch = 1;     //Notch 1
	else if (gCurrentDynValue < 7) notch = 2;     //Notch 2
	else if (gCurrentDynValue < 10) notch = 3;    //Notch 3
	else if (gCurrentDynValue < 12) notch = 4;    //Notch 4
	else if (gCurrentDynValue < 15) notch = 5;    //Notch 5
	else if (gCurrentDynValue < 18) notch = 6;    //Notch 6
	else if (gCurrentDynValue < 22) notch = 7;    //Notch 7
	else notch = 8; 															// Has to be Notch 8
		
	if (gCurrentDynNotch != notch)       //has notch changed?
	{
		//change has happened AND settled down sufficient to get three identical readings in order to get here

		if (!gMotorsEnabled)    //only show debug stuff if motors off
		{
			Serial.print("L:4:gCurrentDynValue now ");
			Serial.println(gCurrentDynValue);
			
			Serial.print("L:4:gCurrentDynNotch now ");
			Serial.println(gCurrentDynNotch);
			
		}

		gControlsChanged = true;
		gCurrentDynNotch = notch;
		

	}
	return 0;
}

//***************************************************
int GetThrottle(void)
{

  //read the raw Encoder value and calculate the corresponding notch
  /*NOTE: debouncing is somewhat complicated for a deliberate reason:-
    when the lever is advanced several notches over a couple of seconds, we only
    want to know when it has stopped moving. this is to give the motor controllers and sound system a solid
    starting point and ending point for multi-notch accel/decel changes
		This is done using a three stage debounce - all values must be identical (i.e. the lever has stopped moving)
  */

	static int debounce[3] 						= {0, 0, 0};
 	static int debounce_idx 					= 0;
				 int notch									= 0;

  switch (debounce_idx)
  {
    case 0:
      debounce[0] = GetEncoder(THR_SELECT_PIN);        //take first encoder sample
			if(debounce[0] < 0 ) return -1; 									//check for errors returned - if so - ignore and start again.
      debounce[1] = 0;															 		//make sure the other two samples are zero'd out
      debounce[2] = 0;
			debounce_idx++;
      return 0;

    case 1:
      debounce[1] = GetEncoder(THR_SELECT_PIN);        //take second analog sample
			if (debounce[0] != debounce[1])
        debounce_idx = 0;                            //we are in the middle of a change or error occurred so reset for three new samples
      else
        debounce_idx++;                              //1st two analog samples match, so advance ready for third
      
			return 0;

    case 2:
      debounce[2] = GetEncoder(THR_SELECT_PIN);
      debounce_idx = 0;                                //completed 3 samples. start again either way
      if (debounce[0] != debounce[1] || debounce[0] != debounce[2])
      {
        return 0;                            					//Still in the middle of a change or error occurred so reset for three new samples
      }
      break;																					//Can only get here if all samples match
  }

	gCurrentThrValue = debounce[0] - (gThrZeroOffset);     //All the debounce samples are the same so use the first one
	

	if			(gCurrentThrValue > -2) notch = 0;			//Idle
	else if (gCurrentThrValue > -5) notch = 1;     //Notch 1
	else if (gCurrentThrValue > -7) notch = 2;     //Notch 2
	else if (gCurrentThrValue > -10) notch = 3;    //Notch 3
	else if (gCurrentThrValue > -12) notch = 4;    //Notch 4
	else if (gCurrentThrValue > -15) notch = 5;    //Notch 5
	else if (gCurrentThrValue > -18) notch = 6;    //Notch 6
	else if (gCurrentThrValue > -22) notch = 7;    //Notch 7
	else notch = 8; 															// Has to be Notch 8
		
	if (gCurrentThrNotch != notch)       //has notch changed?
	{
		//change has happened AND settled down sufficient to get three identical readings in order to get here

		if (!gMotorsEnabled)    //only show debug stuff if motors off
		{
			Serial.print("L:4:gCurrentThrValue now ");
			Serial.println(gCurrentThrValue);
			
			Serial.print("L:4:gCurrentThrNotch now ");
			Serial.println(notch);
		}
		
		//If we are in dynamic mode AND the throttle has been reduced - we are coasting
		if(gDynamicEnabled)
		{
			if (notch < gDynMaxThrNotch)   //TODO - guard against making current dynamic mode into coast mode
			{
				CalcControlStatus(STATUS_COAST,"");		//Throttle is closed more than whatever speed we achieved
			}
			else if(notch > gDynMaxThrNotch)  //test for dynamic highwater mark
					gDynMaxThrNotch = notch;		//save the Max notch or highwater mark
					CalcControlStatus(STATUS_POWER,"");		//Throttle has been advanced past coasting point
		}
		else
		{
			//No dynamic mode operating - simple throttle
			
			CalcControlStatus(STATUS_POWER,"");		//Throttle has been advanced past coasting point
		}
			
		gControlsChanged = true;		
		gCurrentThrNotch = notch;						//Need to set notch properly - even idle for proper sound handling

	}
	return 0;
}


//***************************************************

int GetEncoder(int SelectPin)
{
	//GetEncoder will read the specified encoder and return the poistion value from the lookup table
	//If the value is negative - an error has occured
	

		byte temp = 0;
		int i = 0;
		
		digitalWrite(SelectPin,LOW);
		digitalWrite(SelectPin,LOW);
		//delay(250);
		temp = ENCODER_IN;
		digitalWrite(SelectPin,HIGH);

		if ( gEncoderMap[temp] == 0xFF)   //error!
		{
			return -1;    //return error
		}
		else
		{
			return (int) gEncoderMap[temp];			//return value from lookup table
		}
}

//***************************************************
int GetSpeed(void)
{
	//Get current speed from wheel sensor (which is probably an interrupt routine coupled with a fixed time sample
}


//***************************************************
//SortIntArray - sorts the array of integers a[] with total count of n elements
//***************************************************
 /*
void SortIntArray(int a[],int n) {

   int i,j,temp;

 

   for(i=0;i<n-1;i++) {

      for(j=0;j<n-i-1;j++) {

         if(a[j]>a[j+1])

            SwapInts(&a[j],&a[j+1]);

      }

   }

}

//***************************************************
//Swap - Swaps two array items for Sort
//***************************************************

void SwapInts(int *p,int *q) {

   int t;

  

   t=*p;

   *p=*q;

   *q=t;

}
*/

//***************************************************
//Serial Config & Handling routines
//***************************************************

void ConfigComms(bool IsReboot)
{
  //Configures three RS232 data Streams - 'Serial' for Debug & Raspi Comms and 'Serial1' to talk to the motor controllers

  //initialize Comms
  Serial.begin(RASPI_SERIAL_SPEED);  //Comms to Debug/Raspi
  Serial1.begin(MOTOR_SERIAL_SPEED);  //Motor Controller comms

  if(IsReboot)
  {
    SendBeep(150, 5);    //Will also give a one second delay to allow comms to come up properly
  }
	else
	{
		delay(2000);				//two second delay before using comms the first time.
	}
	
  gSabertooth[0].setGetTimeout(500);   //Set timeout for fetching parameters 
  gSabertooth[1].setGetTimeout(500);
  gSabertooth[2].setGetTimeout(500);
  
  SetMotorRamping(MOTOR_RAMPING);				//The amount of time to transition from one speed to another

}

//***************************************************
//  Send Beep - beep once specified time
//***************************************************
void SendBeep(unsigned long len, int count)
{
	//WARNING - this routine uses delays which will prevent other things from happening! Use wisely.
	//    Currently only used in the initialisation phases of the code for this reason.
	
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

void ConfigDigitalPins(void)
{

  /*Config digital pins - for digital Devices attached to the Arduino:
    NB: All other devices pins are configured by their own device specific initialization code
	*/

  pinMode(TACHO_IN_PIN, INPUT);				//Set pin for input
  digitalWrite(TACHO_IN_PIN, HIGH);		//enable internal pullup

  pinMode(BUZZER_PIN, OUTPUT);				//Set pin for output
	digitalWrite(BUZZER_PIN, LOW);			//Set initial value low = off

  pinMode(HEADLOW_PIN, INPUT);				//Set pin for input
  digitalWrite(HEADLOW_PIN, HIGH);		//enable internal pullup

  pinMode(HEADHIGH_PIN, INPUT);				//Set pin for input
  digitalWrite(HEADHIGH_PIN, HIGH);		//enable internal pullup

  pinMode(BTN_HORN_PIN, INPUT);				//Set pin for input
  digitalWrite(BTN_HORN_PIN, HIGH);		//enable internal pullup

  pinMode(VIGIL_EN_PIN, OUTPUT);			//Set pin for output
	digitalWrite(VIGIL_EN_PIN, LOW);		//set initial value low = off

	pinMode(BTN_VIGIL_PIN, INPUT);			//Set pin for input - remember it is active HIGH
	digitalWrite(BTN_VIGIL_PIN, HIGH);	//Enable internal pullup
	
  pinMode(DYN_SW_PIN, INPUT);					//Set pin for input
  digitalWrite(DYN_SW_PIN, HIGH);			//enable internal pullup

  pinMode(MOTOR_SW_PIN, INPUT);				//Set pin for input
  digitalWrite(MOTOR_SW_PIN, HIGH);		//enable internal pullup
	
	pinMode(DIR_FWD_PIN, INPUT);				//Set pin for input
  digitalWrite(DIR_FWD_PIN, HIGH);		//enable internal pullup

  pinMode(DIR_REV_PIN, INPUT);				//Set pin for input
  digitalWrite(DIR_REV_PIN, HIGH);		//enable internal pullup
	
	pinMode(THR_SELECT_PIN, OUTPUT);		//Set pin for output
  digitalWrite(THR_SELECT_PIN, HIGH);	//Set initial value high = off

  pinMode(DYN_SELECT_PIN, OUTPUT);		//Set pin for output
  digitalWrite(DYN_SELECT_PIN, HIGH);	//Set initial value high = off

	ENCODER_DIR_REG = PORT_DIR_INPUT;		//Set Encoder data port to input
	ENCODER_PULLUP = PORT_ALL_PULLUP;		//Ensure all pullups are enabled
	
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
  CalcMotorNotchSize();
  GetDynamicMode();
  GetBattery();
  CheckHeadlights();

  do 
	{

    //Loop here until all controls are in the safe position
    // Reverser in Neutral. Throttle and Dynamic in idle
    //remember they are mechanically interlocked so really only have to check Dir NONE
    //  - but others are checked in case of faults which could cause unpredictable operation
    //note Buzzer is used to indicate the controls are not setup correctly

    GetDirection();
    if (gDirection == DIR_NONE)
    {
			gThrZeroOffset = GetEncoder(THR_SELECT_PIN);
			Serial.print("Throttle Offset: ");
			Serial.println(gThrZeroOffset);

			gDynZeroOffset = GetEncoder(DYN_SELECT_PIN);
			
			if(gThrZeroOffset > -1 || gDynZeroOffset > -1)				//Ensure there were no errors obtaining the offsets
			{
				//controls are safe and no errors detected with them
				gInitialized = true;
				ResetVigilanceWarning();
				CalcControlStatus(STATUS_IDLE, "Init OK");
			}
    }
    else
    {
      if(!DoneOnce)
      {
        if (gDirection != DIR_NONE)
        {
          CalcControlStatus(STATUS_ERROR, "INIT:Reverser not Neutral");
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
