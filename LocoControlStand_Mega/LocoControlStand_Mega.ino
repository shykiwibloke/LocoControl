/****************************************************
**                                                 **
**  Loco Control Stand version                     **/
#define VERSION "4.1.0"
/*                                                 **
**  Written by Chris Draper                        **
**  Copyright (c) 2016 - 2019                      **
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

//#define LOG_LEVEL_4								//if defined then debug info will be output to the console port

/* Recent Changes Log
 *	4.1.0
13/5/2019:	Dynamic Brake changes tested and refined
13/5/2019:	Vigilance Warning now goes to emergency after 40 seconds unresponded.
 *  4.0.0
11/05/2019: Major rewrite of GetThrottle, GetDynamic and EvaluateState to implement dynamic braking without amperage measurement
 *	3.3.4 & 3.3.5 were unsuccessful Amperage Filter bug hunting versions
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

/***************************************************/
//Pin Use Definitions
/***************************************************/

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

/***************************************************/
// General Defines & Constants
/***************************************************/

#define PORT_DIR_INPUT			0				//Constant used to set Direction Register for input use (all 8 bits at once)
#define PORT_DIR_OUTPUT 		255			//Constant used to set Direction Register for output use (all 8 buts at once)
#define PORT_ALL_PULLUP			255			//Constant used to set all 8 internal pullup resistors for port 

#define MOTOR_SERIAL_SPEED	38400      //for Motor Controllers (The max that the sabertooth can talk at)
#define RASPI_SERIAL_SPEED 	115200		//faster speed over USB link to raspberry pi
#define MAX_NOTCHES					9					//allow for zero and 8 notches - Applies to Throttle and Dynamic
#define MAX_NOTCH_SIZE      255       //the maximum value we can send for throttle or dynamic brake per notch position - 1
#define MIN_NOTCH_SIZE			128				//Notch has to make a noticable difference at min setting

#define HEADLIGHT_ST				2					//Sabertooth port defines for ditch and head lights
#define DITCHLIGHT_R 				1
#define DITCHLIGHT_L				0
#define LIGHT_OFF						-2000			//Brightness defines for PWM controlled headlights
#define LIGHT_HALF					-500
#define LIGHT_FULL					2000

//State Definitions
typedef enum {
					CONTROL_STATE_UNDEFINED,
					CONTROL_STATE_IDLE,
					CONTROL_STATE_POWER,
					CONTROL_STATE_DYNAMIC,
					CONTROL_STATE_COAST,				//Used when throttle/dynamic are both closed AND Dynamic is Enabled
					CONTROL_STATE_EMERGENCY,		//USED when dynamic advanced to notch 8. Only reset by IDLE.
					CONTROL_STATE_ERROR
} controlState_t;

typedef enum {												//Used to formalise condition of reverser microswitchs in gDirection global
					DIR_NONE,
					DIR_FWD,
					DIR_REV
} direction_t;

//Sabertooth configuration defines
const int MOTOR_STD_RAMPING =	2400;				//Normal smoothing transition between notches
const int MOTOR_EMG_RAMPING = 2000;			  //Smoothing when in emergency
const int MOTOR_TIMEOUT 	  = SABERTOOTH_GET_TIMED_OUT;			//value returned by Sabertooth library when comms fails. Must be const int to work reliably.


/***************************************************/
// Global Variable Definitions
/***************************************************/

// Global State Variables 

controlState_t  gControlState     	= CONTROL_STATE_UNDEFINED;	//Holds overall controls status - Error/Idle/Coast/power/Dynamic
bool 						gInitialized       	= false;
bool 						gControlsChanged   	= false;
bool 						gMotorsEnabled			= false;				//True if Motor output to the motor controllers switch is ON
bool 						gDynamicEnabled			= false;				//True if Dynamic Brake selection switch is ON
int  						gThrZeroOffset			= 0;						//Throttle Encoder Offset. Evaluated when reverser in neutral
int  						gDynZeroOffset			= 0;						//Dynamic Encoder Offset. Evaluated when reverser in neutral

// Control Inputs
int  gCurrentThrNotch     = 0;											//Notch currently set for throttle & displayed by raspi
int  gCurrentDynNotch			= 0;											//Notch currently set for Dynamic & displayed by raspi													
int  gCurrentMotorSpeed		= 0;											//Current speed (-2048 - 2048) as last sent to Motors

int  gMotorNotchSize			= 0;											//Notch size - to scale output as directed by CalcMotorNotchSize()
int  gBrakeNotchSize			= 0;											//Notch size for braking calculated by CalcDynBrakeNotchSize()
direction_t gDirection   	= DIR_NONE;								//Forward, Reverse, Neutral ( See DIR_NONE, DIR_FWD, DIR_REV)

int  gBattery           	= 0;											//Current Battery Voltage
int	 gHorn              	= 0;											//Horn sounding 0 = no, 1 = yes
int  gHeadlights        	= 0;											//Headlights off, dip, full (0,1,2 respectively)
//int  gMotorAmps[6]  		  = {0};				HACK						//last read motor Amperage for individual motorsint  gMotorAmps[6]  		= {0};																//last read motor current for individual motors
int	 gVigilanceCount    	= 0;											//Timer count for the Vigilance Alarm
int  gDitchFlashCount   	= 0;											//Count for flashing the ditch lights when the horn is sounded

/***************************************************/
//Define Timer
SimpleTimer gtimer;                         //Only need one timer as all timing callbacks can be handled by this one object

/***************************************************/
//define serial port for motor comms here

USBSabertoothSerial gSP(Serial1);   //Assign to Mega Hardware serial port one

//there are three motor controllers each with two motors. arrange these so they are front to rear in numerical order
USBSabertooth       gSabertooth[3] = { USBSabertooth(gSP, 128), USBSabertooth(gSP, 129), USBSabertooth(gSP, 130) };

/***************************************************/
// Setup Code
/***************************************************/

void setup(void)
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
  gtimer.setInterval(10000, DoTenSecondChecks);
	gtimer.setInterval(250, DoQuarterSecondChecks);

}

/***************************************************/
// Main loop & Core Subroutines
/***************************************************/

//Main loop must be executed frequently for system to run reliably
// (i.e. no long Delay() type statements in subroutines unless deliberately done)

void loop(void)
{
	
  //Read input variables moved to DoQuarterSecondChecks()

  if (gControlsChanged)           //only process if controls have changed
  {

    ResetVigilanceWarning();    //Control movement resets vigilance
    EvaluateState();            //check to see if the control changes mean we are in a new state
																//and issue commands based on the changed controls
		gControlsChanged = false;     //reset the changed flag as we have just handled it.

  }
  else 
  {
		CheckHorn();                      	//if horn is pressed, then sound it. if released, then tell raspi
		//TESTING HACK  GetMotorAmps();              //fetch and send one motor's current per call, and battery voltage once per 6 motor checks.

		
		if (gControlState == CONTROL_STATE_IDLE)   //only want to evaluate these items if the loco is at a standstill
		{
			GetDynamicMode();
			GetMotorMode();
		}
	}
  
  gtimer.run();                     //must be in main loop for timer to work

}


/***************************************************/
void ReadInputs(void)
{

  GetDirection();
  GetDynamic();
  GetThrottle();
  
}

/***************************************************/
void EvaluateState(void)
{

	controlState_t newState = EvaluateLevers();
	static int lastDynamicNotch = 0;
	static int lastThrottleNotch = 0;
	
	//Current Error / Emergency states have to be handled first to prevent premature revocation
	if ((gControlState == CONTROL_STATE_ERROR || gControlState == CONTROL_STATE_EMERGENCY) && newState != CONTROL_STATE_IDLE)   
  {
				return;  //stay in current state and wait for new state to be idle
  }
  
	//Evaluate the new state (including the 'stuck' Error or Emergency states)
	switch(newState)
	{
	
		case CONTROL_STATE_IDLE:
		
			if(newState != gControlState)   //only want to perform this code once when state changes
			{
				//rather than figure how we got here to determine what sounds to send
				//	- just issue sound commands to cut dynamic sounds and engine to idle
				SendMotorSound(0);
				SendDynamicSound(0);
				SetMotorSpeed(0);				//also sets the variable gCurrentMotorSpeed
				
				ChangeControlState(newState,"IDLE OK");
			}
			break;
			
		case CONTROL_STATE_COAST:
		
			if(newState != gControlState)   //only want to perform this code once when state changes
			{
				//rather than figure how we got here to determin what sounds to send
				//	- just issue sound commands to cut dynamic sounds and engine to idle
				SendMotorSound(0);
				SendDynamicSound(0);
				ChangeControlState(newState,"COAST OK");
			}
			break;
			
		case CONTROL_STATE_DYNAMIC:
			
			if(lastDynamicNotch != gCurrentDynNotch)		//Only want to do this if the dyn notch has changed
			{
			SendDynamicSound(gCurrentDynNotch);

			SetMotorRamping(MOTOR_STD_RAMPING); 			//Ensure we have smooth dynamic brake
			SetDynamicBrake();
			ChangeControlState(newState,"DYNAMIC OK");
			}
			break;
			
		case CONTROL_STATE_POWER:    						//Advance the throttle
			
			if(lastThrottleNotch != gCurrentThrNotch)		//only perform this if the throttle notch has changed
			{
				SendMotorSound(gCurrentThrNotch);
				SendDynamicSound(0);

				SetMotorRamping(MOTOR_STD_RAMPING); 			//Ensure we have smooth acceleration
				SetMotorSpeed(CalcSpeedFromNotch(gCurrentThrNotch));
				ChangeControlState(newState,"POWER OK");
			}
			break;
			
		case CONTROL_STATE_EMERGENCY:
		
			SendDynamicSound(8);									//Place first ensures we get lever position shown, but idle sounds
			SendMotorSound(0);
			
			SetMotorRamping(MOTOR_EMG_RAMPING); 		//Ensure we have smooth motor changes if comms reset or motors turned on after arduino
			SetMotorSpeed(0);
			ChangeControlState(newState,"EMERGENCY OK");
			break;
		case CONTROL_STATE_UNDEFINED:						//undefined at this point is a bug - has to be an error!
		
			newState = CONTROL_STATE_ERROR;
			ChangeControlState(newState,"UNDEFINED MODE - BUG!");
			break;
		case CONTROL_STATE_ERROR:								//Stay there until the error clears
		
			Serial.println("L:1:EvalutateState Error Condition. Place Controls to Idle to clear");
			TriggerVigilanceWarning(true);
			delay(50);
			ResetVigilanceWarning();
			ChangeControlState(CONTROL_STATE_ERROR,"EvaluateState Error Mode Cauught");
			break;
	}
			 
}

/***************************************************/
// Evaluation and Calculation subroutines
/***************************************************/

void ChangeControlState(const controlState_t newState, const char* Msg)
{
  //Logs a control state change.
  //NOTE - it will only log the state change ONCE so safe to call in a loop.
  //Classed as debug level message unless current or new state is ERROR

  if (newState != gControlState)
  {
    if (newState == CONTROL_STATE_ERROR || gControlState == CONTROL_STATE_ERROR)
    {
      Serial.print("L:1:Critical Error - ");    //print the error heading
      Serial.println(Msg);              //print the passed message
    }
		#ifdef LOG_LEVEL_4
			Serial.print("L:4:State changed from ");
			Serial.print(gControlState);
			Serial.print(" to ");
			Serial.print(newState);
			Serial.print(" MSG: ");
      Serial.println(Msg);              //print the passed message
		#endif
		
    gControlState = newState;       //set the new status

  }
}

/***************************************************/
void CalcMotorNotchSize(void)
{
  //Reads the MaxSpeed value and set the ramp variables accordingly if it has changed.

  //read it, note: Arduino analog range is roughly half that of the sabertooth & this routine takes that into account
  int newval = analogRead(MAX_SPEED_PIN) >> 3;
 
  newval += MIN_NOTCH_SIZE;			//shift up to 128-255 range
 
  //limit-check the value to fit range 128-255 before comparing it to last setting/using it
  if (newval > MAX_NOTCH_SIZE) newval = MAX_NOTCH_SIZE;
  if (newval < MIN_NOTCH_SIZE) newval = MIN_NOTCH_SIZE;

  //has it changed?
  if (gMotorNotchSize != newval)
  {
   //set it
    gMotorNotchSize = newval;

	//and Advise
	Serial.print("L:4:NotchSize=");
    Serial.println(gMotorNotchSize);

  }
}

/***************************************************/
void CalcDynBrakeNotchSize(void)
{
	CalcMotorNotchSize();
		
	//NOTE: notch 8 is emergency for safety and handled by EvaluateState - dont have to do that here.
	//      notch 0 is handled by idle/coast logic 
	if(gCurrentDynNotch <1 || gCurrentDynNotch > 7) return;
	
	gBrakeNotchSize = ((gCurrentMotorSpeed>>4));       //reduce speed by 1/16th of the current speed per notch
  //Note notch 1 removes 1/16th while notch 7 removes 7/16ths 
	//Note how divide by 8 is done using binary shift left by 4
	
}
/***************************************************/
void DoQuarterSecondChecks(void)
{
	//Stuff that needs to be checked regularly to appear realtime, but also has a debounce component

  ReadInputs();                   //Read input variables
	CheckHeadlights();

	if (digitalRead(BTN_VIGIL_PIN))   	//Hitting vigilance button (active high) is not a control change, but should reset it
	{
		ResetVigilanceWarning();        	//Just reset vigilance - not a control 'change' that needs further analysis
	}

}
/***************************************************/
void DoTenSecondChecks(void)
{
  //Stuff that only needs to be checked every Ten seconds or so Called by timer routine
  
  GetBattery();                 //Get the battery voltage

  //Calculate Vigilance warnings

  if (gDirection == DIR_NONE)
  {
    ResetVigilanceWarning();        //continually suppress vigilance when in neutral.
  }
  else
  {
		gVigilanceCount++;
    
		if (gVigilanceCount == 2)  //more than 20 seconds since a control was touched
    {
      Serial.println("L:2:Vigilance Light");
      TriggerVigilanceWarning(false);   //turn light on now (no buzzer
    } 
		else if (gVigilanceCount == 3)  //light been on for at least 30 seconds - sound buzzer
    {
      Serial.println("L:2:Vigilance Buzzer");
      TriggerVigilanceWarning(true);  //Turn buzzer on
    }
		else if (gVigilanceCount == 4)  //Buzzer & light been on for at least 40 seconds
    {
			SetMotorRamping(MOTOR_EMG_RAMPING); 		//Ensure we have smooth motor changes if comms reset or motors turned on after arduino
			SetMotorSpeed(0);
			ChangeControlState(CONTROL_STATE_EMERGENCY,"Vigilance Triggered");
			Serial.println("L:2:Vigilance Triggered");
      SendDynamicSound(0);
			SendMotorSound(0);
			Serial.println("S:h:");
			delay(2000);				//two second delay before using comms the first time.
			Serial.println("S:j:");
    }

  }
}

/***************************************************/
//Output setting routines
/***************************************************/
int CalcSpeedFromNotch(const int newNotch)
{
	
//This routine takes a notch number as an argument and calculates the proper speed for the motors
		
	CalcMotorNotchSize();       											//check to see if user has changed the notch size before we use it

	int speed = gMotorNotchSize * newNotch;      //used to scale the provided notch to a speed setting the controller understands (-2047-0-2047 range)

  if (gDirection == DIR_REV)  speed = -speed;       //a negative number makes the motor go in reverse, positive = go forward
	
	return speed;

}

/***************************************************/

void SetMotorSpeed(const int newSpeed)
{

  	gCurrentMotorSpeed = newSpeed;										//We have been given a new speed to set - tell the world we have it.
	
  	//send the commands to the motor controllers if enabled
	if (!gMotorsEnabled)
	{	
		Serial.println("** Motors Disabled No Output Sent **");
		Serial.print("Speed = ");
		Serial.println(newSpeed);
		
	}
	else    //Set the real speed
	{
		gSabertooth[0].motor(1, newSpeed);     //first pair
		gSabertooth[0].motor(2, newSpeed);
		gSabertooth[1].motor(1, newSpeed);     //2nd pair
		gSabertooth[1].motor(2, newSpeed);
		gSabertooth[2].motor(1, newSpeed);     //3rd pair
		gSabertooth[2].motor(2, newSpeed);
	}

}

/***************************************************/
void SendMotorSound(const int Notch)
{
		//Send specified motor sound
	Serial.println("S:t:");					//throttle mode
	Serial.print("S:");							//Actual Sound command
  Serial.print(Notch);
  Serial.println(":");
}

/***************************************************/
void SendDynamicSound(const int Notch)
{
		//Send specified motor sound
	Serial.println("S:d:");					//dynamic mode
	Serial.print("S:");							//Actual Sound command
  Serial.print(Notch);
  Serial.println(":");
}

/***************************************************/
void SetDynamicBrake(void)
{
	int speedReduction = 0;
	
	//NOTE: notch 8 is emergency for safety and handled by EvaluateState - dont have to do that here.
	//      notch 0 is handled by idle/coast logic 
	if(gCurrentDynNotch <1 || gCurrentDynNotch > 7) return;
	
	CalcDynBrakeNotchSize();
	
	speedReduction = (gBrakeNotchSize*gCurrentDynNotch);
  
#ifdef LOG_LEVEL_4
		Serial.print("L:4:Planned Brake Reduction = ");
		Serial.println(speedReduction);
		
		Serial.print("L:4:Current Speed = ");
		Serial.println(gCurrentMotorSpeed);
#endif
	
	if(gMotorsEnabled)
	{
		if(speedReduction > gCurrentMotorSpeed || ((gCurrentMotorSpeed - speedReduction) < MIN_NOTCH_SIZE))
		{
			SetMotorSpeed(0);		//dont go negative, or leave motors stalled at too low a speed.
		
		} else 
		{	
			SetMotorSpeed(gCurrentMotorSpeed - speedReduction);
		}
	}
#ifdef LOG_LEVEL_4
	Serial.print("L:4:New Speed = ");
	Serial.println(gCurrentMotorSpeed);
#endif

	

}

/***************************************************/

void SetMotorRamping(const int newvalue)
{

  gSabertooth[0].setRamping(newvalue);
  gSabertooth[1].setRamping(newvalue);
  gSabertooth[2].setRamping(newvalue);

}

/***************************************************/
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

/***************************************************/
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

/***************************************************/

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

/***************************************************/
void TriggerVigilanceWarning(const bool SoundBuzzer)
{

  //Set off Vigilance light the vigilance LED
  digitalWrite(VIGIL_EN_PIN, HIGH);

  if (SoundBuzzer) digitalWrite(BUZZER_PIN, HIGH);

}

/***************************************************/
void ResetVigilanceWarning(void)
{
  //Reset Vigilance buzzer and light the vigilance LED

  digitalWrite(VIGIL_EN_PIN, LOW);
  digitalWrite(BUZZER_PIN, LOW);
  gVigilanceCount = 0;
	Serial.println("Vilgilance Reset");

}
/***************************************************/
//Get State Routines
/***************************************************/
/*
void GetMotorAmps()
{
	float movingAverage[6] = {1};				//one for each motor (has to be preserved from one run to the next)
	float movingAverageSum[6] = {1};
	const int AVERAGE_COUNT = 99;				//TODO - change to header const?
				
	static int motor = 0;
  static int ErrCount = 0;
  int      	 rawvalue = 0;

  //Gets amps on cycle - for motor one, then two on second run etc.
  if(gControlState != CONTROL_STATE_ERROR
		 && gControlState != CONTROL_STATE_IDLE
     && Serial1.availableForWrite() > 50 
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

			if (rawvalue -9 && rawvalue < 9) rawvalue = 0;
			//experimental rms stuff
			rawvalue = rawvalue * rawvalue;
			
			// Remove previous movingAverage from the sum
			movingAverageSum[motor] -= movingAverage[motor];

			// Replace it with the current sample
			movingAverageSum[motor] += (movingAverage[motor]*.875) + (rawvalue*0.125);

			// Recalculate movingAverage incl guard for divide by zero
			if(movingAverageSum[motor] == 0)
				movingAverageSum[motor] = 1;
			else
				movingAverage[motor] = sqrt(movingAverageSum[motor] / AVERAGE_COUNT);
			

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
*/
/***************************************************/
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

/***************************************************/
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
	return temp;
}

/***************************************************/
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
	return temp;
}

/***************************************************/
void GetDirection(void)
{

  /* GetDirection looks more complicated than it should - but there is a safety reason for this.
     We need to be absolutely sure we are carrying out the users wishes. There could be a fault in
     the system resulting in two directions reading true at the same time - whichever we check first
     could carry the day and lead to disaster. So the only way is to look at all inputs together
     and determine a) system is functioning normally and b) there is only one valid direction bit set

  */
 
  if ((digitalRead(DIR_FWD_PIN) == LOW) && (digitalRead(DIR_REV_PIN) == LOW))
  {
    //cant have both directions low at the same time - error
    gDirection = DIR_NONE;
    ChangeControlState(CONTROL_STATE_ERROR, "ERROR: Reverser FWD AND REV");
    return;
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
  return;
}

/***************************************************/

controlState_t EvaluateLevers(void)
{
	//Need to treat Dynamic and Throttle levers as a package - otherwise one overrules the other.
	//cleaner to do this outside the main EvaluateState routine
	
	controlState_t newState = CONTROL_STATE_UNDEFINED;
	
		if(gDynamicEnabled)  //if dynamic mode is permitted
		{
			if(gDirection == DIR_NONE)
			{
				newState = CONTROL_STATE_IDLE;				//reverser in neutral invalidates any checks over dynamic etc.
			}
			else if(gCurrentDynNotch == 0 && gCurrentThrNotch == 0)  //normal 'idle' wihtout moving reverser
			{			
				if(gCurrentMotorSpeed > 0)
				{
					newState = CONTROL_STATE_COAST;
				}
				else
				{
					newState = CONTROL_STATE_IDLE;
				}
			}
			else if(gCurrentDynNotch == 8) //Special Case - EMERGENCY BRAKE APPLICATION (can only be reset by going to IDLE)
			{
				newState = CONTROL_STATE_EMERGENCY;
			}
			else if(gCurrentDynNotch > 0) //Dynamic lever is advanced, but not notch 8
			{		
				if(gCurrentMotorSpeed > 0)
				{
					newState = CONTROL_STATE_DYNAMIC;		//gotta be moving for the dynamic to work!
				}
				else
				{
					newState = CONTROL_STATE_IDLE;			//Stay idle if not moving and dynamic advanced
				}
			}				
			else if(gCurrentThrNotch > 0)     //Throt advanced from prev (incl 0)
			{
				if (CalcSpeedFromNotch(gCurrentThrNotch) > gCurrentMotorSpeed)   //Throttle is above whatever speed we are currently
				{
						newState = CONTROL_STATE_POWER;	
				}
				else
				{
					newState = CONTROL_STATE_COAST;	
				}
			}
		}
		else  //dynamic not enabled simple throttle mode
		{
			if(gDirection == DIR_NONE || (gCurrentDynNotch == 0 && gCurrentThrNotch == 0))
			{
				newState = CONTROL_STATE_IDLE;				//reverser in neutral invalidates any checks over dynamic etc.
			}
			else
				newState = CONTROL_STATE_POWER;
		}
		
		return newState;
}

/***************************************************/
void GetDynamic(void)
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
			if(debounce[0] < 0 ) return; 									//check for errors returned - if so - ignore and start again.
      debounce[1] = 0;															 		//make sure the other two samples are zero'd out
      debounce[2] = 0;
			debounce_idx++;
      return;

    case 1:
      debounce[1] = GetEncoder(DYN_SELECT_PIN);        //take second analog sample
			if (debounce[0] != debounce[1])
        debounce_idx = 0;                            //we are in the middle of a change or error occurred so reset for three new samples
      else
        debounce_idx++;                              //1st two analog samples match, so advance ready for third
      return;

    case 2:
      debounce[2] = GetEncoder(DYN_SELECT_PIN);
      debounce_idx = 0;                                //completed 3 samples. start again either way
      if (debounce[0] != debounce[1] || debounce[0] != debounce[2])
      {
        return;                            					//Still in the middle of a change or error occurred so reset for three new samples
      }
      break;																					//Can only get here if all samples match
  }

	debounce[0] = debounce[0] - (gDynZeroOffset);     //All the debounce samples are the same so use the first one
	

	if			(debounce[0] < 3) notch = 0;		 //Idle					//NB: Values DIFFERENT for Throttle to Dynamic
	else if (debounce[0] < 5) notch = 1;     //Notch 1
	else if (debounce[0] < 7) notch = 2;     //Notch 2
	else if (debounce[0] < 10) notch = 3;    //Notch 3
	else if (debounce[0] < 12) notch = 4;    //Notch 4
	else if (debounce[0] < 15) notch = 5;    //Notch 5
	else if (debounce[0] < 18) notch = 6;    //Notch 6
	else if (debounce[0] < 22) notch = 7;    //Notch 7
	else notch = 8; 															// Has to be Notch 8
		
	if (gCurrentDynNotch != notch)       //has notch changed?
	{
		//change has happened AND settled down sufficient to get three identical readings in order to get here

			//Only bother to register change IF dynamic mode is enabled otherwise ignore
			gControlsChanged = true;
			gCurrentDynNotch = notch;		//save the value to be used by Evaluate routines

	}
	return;
}

/***************************************************/
void GetThrottle(void)
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
			if(debounce[0] < 0 ) return; 									   //check for errors returned - if so - ignore and start again.
      debounce[1] = 0;															 		//make sure the other two samples are zero'd out
      debounce[2] = 0;
			debounce_idx++;
      return;

    case 1:
      debounce[1] = GetEncoder(THR_SELECT_PIN);        //take second analog sample
			if (debounce[0] != debounce[1])
        debounce_idx = 0;                            //we are in the middle of a change or error occurred so reset for three new samples
      else
        debounce_idx++;                              //1st two analog samples match, so advance ready for third
      
			return;

    case 2:
      debounce[2] = GetEncoder(THR_SELECT_PIN);
      debounce_idx = 0;                                //completed 3 samples. start again either way
      if (debounce[0] != debounce[1] || debounce[0] != debounce[2])
      {
        return;                            					//Still in the middle of a change or error occurred so reset for three new samples
      }
      break;																					//Can only get here if all samples match
  }

	debounce[0] = debounce[0] - (gThrZeroOffset);     //All the debounce samples are the same so use the first one
	
	if			(debounce[0] > -2) notch = 0;			//Idle					//NB: Values DIFFERENT for Throttle to Dynamic
	else if (debounce[0] > -5) notch = 1;     //Notch 1
	else if (debounce[0] > -7) notch = 2;     //Notch 2
	else if (debounce[0] > -10) notch = 3;    //Notch 3
	else if (debounce[0] > -12) notch = 4;    //Notch 4
	else if (debounce[0] > -15) notch = 5;    //Notch 5
	else if (debounce[0] > -18) notch = 6;    //Notch 6
	else if (debounce[0] > -22) notch = 7;    //Notch 7
	else notch = 8; 															// Has to be Notch 8
		
	if (gCurrentThrNotch != notch)       //has notch changed?
	{
		//change has happened AND settled down sufficient to get three identical readings in order to get here

		gControlsChanged = true;		
		gCurrentThrNotch = notch;						//Need to set notch properly - even idle for proper sound handling

	}
	return;
}

/***************************************************/

int GetEncoder(const int SelectPin)
{
	//GetEncoder will read the specified encoder and return the poistion value from the lookup table
	//If the value is negative - an error has occured
	

		byte temp = 0;
		
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

/***************************************************/
int GetSpeed(void)
{
	//Get current speed from wheel sensor (which is probably an interrupt routine coupled with a fixed time sample
	return 0;
}

/***************************************************/
//Serial Config & Handling routines
/***************************************************/

void ConfigComms(const bool IsReboot)
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
  
  SetMotorRamping(MOTOR_STD_RAMPING);				//The amount of time to transition from one speed to another

}

/***************************************************/
//  Send Beep - beep once specified time
/***************************************************/
void SendBeep(const unsigned long len, const int count)
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

/***************************************************/
// Configuration, Test & Error handling routines
/***************************************************/

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
	digitalWrite(BTN_VIGIL_PIN, LOW);		//set initial value low = off
	//DO NOT ENABLE INTERNAL PULLUP - This is ACTIVE HIGH
	
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

/***************************************************/
void initSystem(void)
{
  //set all the values by calling all the relevant subroutines
  bool DoneOnce = false;

  gCurrentThrNotch = 0;
	gCurrentDynNotch = 0;

  SetMotorSpeed(0);                   //Force all motors to stop - issue commands before checking controllers are there for safety.

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
				ChangeControlState(CONTROL_STATE_IDLE, "Init to STATE_IDLE OK");
			}
    }
    else
    {
      if(!DoneOnce)
      {
        if (gDirection != DIR_NONE)
        {
          ChangeControlState(CONTROL_STATE_ERROR, "INIT:Reverser not Neutral");
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


/***************************************************/
