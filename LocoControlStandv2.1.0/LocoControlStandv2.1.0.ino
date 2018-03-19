/****************************************************
**                                                 **
**  Loco Control Stand version                     **/
#define VERSION "2.1.0"
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
13/03/2018: Added DYN_KEY_SW and its associated logic
12/03/2018: Dynamic Brake changes ready for testing (Forgotten when these were done)
12/03/2018: Changed Momentum() to CalcMotorNotchSize()
12/03/2018: Change MOTOR_NOTCH to MAX_NOTCH_SIZE and create gNotchSize variable for Speed calculation purposes
12/03/2018: Obsolete SetMotorRamping() - replaced by Max Speed calculations
12/03/2018: Change Version to 2.0.4
25/05/2017: Add notch detent and modify use of analog potentiometer to modify speed increment per notch

End Recent Changes Log  */


#include <SoftwareSerial.h>   //used to drive comms to motor controllers
#include <USBSabertooth.h>    //comms library for sabertooth controllers
//#include <Servo.h>            //used for speedo - can be removed to make space if no speedo fitted 
#include <SimpleTimer.h>      //used for vigilance and heartbeat type functions

//***************************************************
// Defines
//***************************************************

//general constants

//#define LOG_LEVEL_4			//if defined then debug info will be output to the raspi


#define MOTOR_SERIAL_SPEED	9600      //9600 for slower testing 
#define RASPI_SERIAL_SPEED 19200     //faster speed over USB link
#define SERIAL_BUFSIZE	50		  //Reserve 50 bytes for sending and receiving messages with the raspi
#define MOTOR_TIMEOUT -32768   //value returned by Sabertooth library when comms fails

#define NOTCH_DETENT 0     //used to adjust the notch boundaries
#define NOTCH1 760 + NOTCH_DETENT     //below NOTCH1 is IDLE
#define NOTCH2 790 + NOTCH_DETENT      //below NOTCH2 is NOTCH1
#define NOTCH3 830 + NOTCH_DETENT      //below NOTCH3 is NOTCH2   etc
#define NOTCH4 855 + NOTCH_DETENT
#define NOTCH5 880 + NOTCH_DETENT
#define NOTCH6 915 + NOTCH_DETENT
#define NOTCH7 945 + NOTCH_DETENT
#define NOTCH8 975     //Above NOTCH8 is all Notch 8 (i.e. measurement in the range of 912 - 1024)

//Status Definitions

const int STATUS_ERROR = 99;
const int STATUS_IDLE = 10;
const int STATUS_POWER = 20;
const int STATUS_DYNAMIC = 30;

const int DIR_NONE = 0;
const int DIR_FWD = 1;
const int DIR_REV = 2;

//Sabertooth configuration defines
#define MAX_NOTCH_SIZE      255            //the maximum value we can send for throttle or dynamic brake per notch position - 1

//Sabertooth ports for ditch lights and headlights
#define HEADLIGHT_ST	2
#define DITCHLIGHT_R 	1
#define DITCHLIGHT_L	0


#define LIGHT_OFF		-2000
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
#define TACHO_IN		7       //D7 - input for tachometer (interrupted IR)
#define BTN_HORN		8       //D8 - input switch on console
#define DYN_KEY_SW		9		//D9 - Key switch used to toggle Dynamic Braking Mode
#define BTN_VIGILANCE	10      //D10 - pushbutton on console - note input is active HIGH
#define SPEEDO_PIN		11      //D11 - RC Servo output to drive Speedometer
#define BUZZER_PIN		12      //D12 - Buzzer in console
#define VIGILANCE_EN	13      //D13 - Vigilance Button Enable + the on board LED
#define HEADLOW_PIN     A3      //A3 - Headlights to low power
#define HEADHIGH_PIN    A4      //A4 - Headlights to high power 


//***************************************************
// Global Variable Definitions
//***************************************************

//General Globals

bool gInitialized       = false;
bool gRaspiOK           = false;
bool gControlsChanged   = false;
int  gControlStatus     = 0;
int  gMotorStartDelayID = 0;

//Control Inputs
int  gThrottleNotch     = 0;	//Notch currently calculated for throttle
int  gDynamicNotch      = 0;	//Notch currently calculated for Dynamic
int  gNotchSize			= 0;	//Notch size - to scale output as directed by CalcMotorNotchSize()
int  gDirection         = 0;
int  gBattery           = 0;
int	 gHorn              = 0;
bool gDynBrakeActive	= false;	//Set by DYN_KEY_SW - set to off or simple throttle mode for start up.
int  gHeadlights        = 0;
int  gMotorCurrent[7]   = {0, 0, 0, 0, 0, 0, 0}; //last read motor current TOTAL + individual motors
int	 gVigilanceCount    = 0;                //Timer count for the Vigilance Alarm
int  gDitchFlashCount   = 0;

//***************************************************
//Define Timer
SimpleTimer gtimer;                         //Only need one timer as all timing callbacks can be handled by this one object

//***************************************************
//define soft serial port for motor comms here

SoftwareSerial SWSerial(3, 4);              // RX, TX
USBSabertoothSerial gSP(SWSerial);

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
  ConfigComms();


  //configure the speedo output
//  gSpeedo.attach(SPEEDO_PIN);

  //call various subroutines to ensure internal state matches control panel
  initSystem();

  //Check slow controls and heartbeat comms every five seconds
  gtimer.setInterval(5000, DoTimedIntervalChecks);

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
    EvaluateState();            //check to see if the control changes mean we are in a new state
    //and issue commands based on the changed controls - and also keep-alive heartbeats
  }

  ServiceComms();                   // Comms Housekeeping route - handles reception of messages from Raspi etc

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
    Serial.println(F("L:2:Vigilance Manual Reset"));
    ResetVigilanceWarning();        //Just reset vigilance - not a control 'change'
  }
  
  if (gControlStatus == STATUS_IDLE)   //only want to evaluate these items if the loco is at a standstill
  {
	  CalcMotorNotchSize();
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
    Serial.println(F("L:1:EvalutateState Error Condition Detected"));
  }
  else
  {
    if ((gThrottleNotch == 0) && (gDynamicNotch == 0))
    {
      CalcControlStatus(STATUS_IDLE, "Idle");
      SetMotorSpeed();			//ensure speed is indeed set to zero
    }
    else if (gDirection != DIR_NONE)
    {
      //Direction has been set to forward or reverse - dont actually care which here.

 /*     if (gDynamicNotch > 0 && gDynBrakeActive == true)
      {
        CalcControlStatus(STATUS_DYNAMIC, "Dyn Brake ON");

        Serial.print(F("S:"));
        Serial.print(gDynamicNotch);
        Serial.println(F(":"));
		    SetDynamicBrake();    //go adjust motors into proper dynamic notch
      }
      else */ if (gThrottleNotch > 0)
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
        Serial.print(F("S:"));
        Serial.print(gThrottleNotch);
        Serial.println(F(":"));
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
      Serial.print(F("L:1:Critical Error - "));    //print the error heading
      Serial.println(Msg);              //print the passed message

    }

    gControlStatus = NewStatus;       //set the new status

    //Send the proper Sound System status command
    if (NewStatus == STATUS_POWER)
      Serial.println(F("S:t:"));

    if (NewStatus == STATUS_DYNAMIC)
      Serial.println(F("S:d:"));

    if (NewStatus == STATUS_IDLE)
      Serial.println(F("S:0:"));

  }
}

//***************************************************
int CalcNotch(int newval)
{
  //nb: this routine serves both throttle and dynamic brake

  if (newval < NOTCH1)  return 0;  //less than Notch1 is IDLE
  if (newval < NOTCH2)  return 1;
  if (newval < NOTCH3)  return 2;
  if (newval < NOTCH4)  return 3;
  if (newval < NOTCH5)  return 4;
  if (newval < NOTCH6)  return 5;
  if (newval < NOTCH7)  return 6;
  if (newval < NOTCH8)  return 7;

  return 8;    //no other possibilities left except Notch 8

}

//***************************************************
void CalcMotorNotchSize(void)
{
  //Reads the MaxSpeed value and set the ramp variables accordingly if it has changed.

  //read it, note: Arduino analog range is roughly half that of the sabertooth & this routine takes that into account
  int newval = analogRead(MAX_SPEED_PIN) >> 4;

  //limit-check the value to fit ramping range before comparing it to last setting/using it
  if (newval > MAX_NOTCH_SIZE) newval = MAX_NOTCH_SIZE;
  if (newval < 32) newval = 32;			//Cant be zero as that would be a hard thing to figure for newbie drivers!
										//Make it of sufficient size that loco moves when wound right up to notch 8

  //has it changed?
  if (gNotchSize != newval)
  {

   //set it
    gNotchSize = newval;
	
	//and Advise
	Serial.print(F("L:4:Set NotchSize to:  "));
    Serial.println(gNotchSize);

  }
}

//***************************************************
void DoTimedIntervalChecks(void)
{
  //Stuff that only needs to be checked every five seconds or so Called by timer routine
  static int callcount = 0;
  static bool OddCall = false;

  //SendCommsHeartbeat();     //we will send it every five seconds even if there are commands going to the controllers.
  
  //IF we are busy flashing the ditch lights - dont query the slow-to-answer variables from the motor controlers
  // just exit and come back later.
  if(gDitchFlashCount > 0)
    return;

  if (OddCall)
  {
    GetMotorCurrent();   //get and send to the raspi every ten seconds
    OddCall = !OddCall;
  }
 // else
 // {
    // todo - calc something every ten seconds, but opp motor current call
 // }

  if (callcount == 3)    //stuff every 15 seconds, but opposite the next lot
  {
    GetBattery();
  }
  if (callcount++ > 6)   //stuff that can be done every 30 seconds
  {
    // GetTemperature();         //TODO - keep? get and send to the raspi
    callcount = 0;
  }

  //Calculate Vigilance warnings

  if (gDirection == DIR_NONE)
  {
    ResetVigilanceWarning();        //suppress vigilance when in neutral.
  }
  else
  {
    if (gVigilanceCount++ > 6)  //more than 30 seconds since a control was touched
    {
      Serial.println(F("L:2:Vigilance Light Triggered"));
      SetVigilanceWarning(false);
    }

    if (gVigilanceCount > 8)  //light been on for at least 10 seconds
    {
      Serial.println(F("L:2:Vigilance Buzzer Triggered"));
      SetVigilanceWarning(true);   //buzzer goes off 10 seconds later
    }

    if (gVigilanceCount > 10)
    {
      //todo - drop throttle to zero, amperage full, MaxSpeed none, go to error mode, send message to raspi
    }
  }
}

//***************************************************
//Output setting routines
//***************************************************

void SetMotorSpeed(void)
{

  int speed = gNotchSize * gThrottleNotch;        //used to scale the provided notch to a speed setting the controller understands (-2047-0-2047 range)

  if (gDirection == DIR_REV)  speed = -speed;                    //a negative number makes the motor go in reverse, positive = go forward

  Serial.print("setmotor(T):");
  Serial.println(speed);

	   //send the commands to the motor controllers

  gSabertooth[0].motor(1, speed);     //first pair
  gSabertooth[0].motor(2, speed);
  gSabertooth[1].motor(1, speed);     //2nd pair
  gSabertooth[1].motor(2, speed);
  gSabertooth[2].motor(1, speed);     //3rd pair
  gSabertooth[2].motor(2, speed);
}

//***************************************************

void SetDynamicBrake()
{

  int brake = (8 - gNotchSize) * gDynamicNotch;          //used to scale the provided notch to a speed setting the controller understands (-2047-0-2047 range)
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
/*
// Obsolete code after CalcMotorNotchSize changes?
void SetMotorRamping(int newvalue)
{

  Serial.print(F("L:4:Set Ramping to "));
  Serial.println(newvalue);

  gSabertooth[0].setRamping(newvalue);
  gSabertooth[1].setRamping(newvalue);
  gSabertooth[2].setRamping(newvalue);

}
*/
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
        Serial.println(F("L:2:Headlights off"));
        break;
      case 1:  //headlight on dip
        gSabertooth[HEADLIGHT_ST].power(LIGHT_HALF);
        gSabertooth[DITCHLIGHT_L].power(LIGHT_HALF);
        gSabertooth[DITCHLIGHT_R].power(LIGHT_HALF);
        Serial.println(F("L:2:Headlights dip"));
        break;
      case 2: //headlight on full
        gSabertooth[HEADLIGHT_ST].power(LIGHT_FULL);
        gSabertooth[DITCHLIGHT_L].power(LIGHT_FULL);
        gSabertooth[DITCHLIGHT_R].power(LIGHT_FULL);
        Serial.println(F("L:2:Headlights full"));
        break;

    }

    gHeadlights = state;


  }
}

//***************************************************
void FlashDitchlights(void)
{
  //Called by timer callback setup within CheckHorn()

  if (--gDitchFlashCount < 1)
  {
    //restore both ditch lights to whatever they were by invalidating current position
    gHeadlights -1;
    CheckHeadlights();
    return;
  }

  if (gDitchFlashCount%2 == 0)     //if count is even
  {
      //Serial.println(F("L:4:FlashL"));
      gSabertooth[DITCHLIGHT_L].power(LIGHT_FULL);  //turn left ditchlight
      gSabertooth[DITCHLIGHT_R].power(LIGHT_OFF);  //turn left ditchlight
  }
  else
  {
      //Serial.println(F("L:4:FlashR"));
      gSabertooth[DITCHLIGHT_R].power(LIGHT_FULL);  //turn left ditchlight
      gSabertooth[DITCHLIGHT_L].power(LIGHT_OFF);  //turn left ditchlight
  }
}

//***************************************************
/*
void SetSpeedo(int speed)
{
  //Set the speedo needle to point to the supplied speed.

  Serial.println(F("L:4:Set Speedo "));
  speed = map(speed, 0, 1023, 0, 180);     // scale it to use it with the servo (value between 0 and 180)
  gSpeedo.write(speed);                  // sets the servo position according to the scaled value
}
*/
//***************************************************

void CheckHorn(void)
{

  int newval[3];

  newval[0] = digitalRead(BTN_HORN);
  delay(10);
  newval[1] = digitalRead(BTN_HORN);
  delay(10);
  newval[2] = digitalRead(BTN_HORN);

  //Set the horn to match the button state.
  if (newval[0] == 0 && newval[1] == 0 && newval[2] == 0 && gHorn == 0)	//Horn button pressed, we have not seen this yet
  {
    Serial.println(F("S:h:"));
    gHorn = 1;

    //set ditch-lights flashing for ten seconds
    gDitchFlashCount = 30;
    gtimer.setTimer(400, FlashDitchlights, gDitchFlashCount);
 
    ResetVigilanceWarning();		//horn press also resets the vigilance timer.

  }
  else if (newval[0] == 1 && newval[1] == 1 && newval[2] == 1 && gHorn == 1)		//Horn button released, we have not seen this yet
  {
    Serial.println(F("S:j:"));
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

void GetMotorCurrent()
{
  //call with unscaled true to get real current (to use to determine scaling)
  //call with unscaled false in normal operation to obtain % of useable capacity
  //todo - gotta do something with the data

  static int Motor = 0;

  //Gets amps on cycle - for motor one, then two on second run etc.


  if (Motor == 0) //slot zero used to calc total amperage
  {
    gMotorCurrent[0] = gMotorCurrent[1] + gMotorCurrent[2] + gMotorCurrent[3] + gMotorCurrent[4] + gMotorCurrent[5] + gMotorCurrent[6];
  }
  else
  {
    //Get current via first driver using ascii representation of motor number
    gMotorCurrent[Motor] = gSabertooth[0].getCurrent((Motor + 49), false);

    if (abs(gMotorCurrent[Motor]) < 300 || gMotorCurrent[Motor] > -31000) //if less than 300ma or timed out
      gMotorCurrent[Motor] = 0;
    else
      gMotorCurrent[0] = gMotorCurrent[0] / 100;              //change milliamps to Amps
  }

  //log the retrieved value or calculated total
  Serial.print(F("M:"));
  Serial.print(Motor);
  Serial.print(":");
  Serial.println(gMotorCurrent[0]);


  Motor ++;
  if (Motor > 6) Motor = 0;

}

//***************************************************
void GetBattery(void)
{

  //Battery should be the same for all controllers - so only have to read one
  gBattery = gSabertooth[2].getBattery(1, false);
  if (gBattery == MOTOR_TIMEOUT)
  {
    SendBeep(50, 1);
    gBattery = 0;
  } else if (gBattery > 0)
  {
    gBattery = gBattery / 10;
  }

  Serial.print(F("V:1:"));
  Serial.println(gBattery);

}

//***************************************************
void GetTemperature(void)
{
  static int driver = 0;
  int temp = 0;

  //Gets temps on cycle - for driver one and two first time, then three and four on second run etc.

  //check and process timeout only on first call
  Serial.print(F("L:3: Temperature Data:"));
  Serial.print(driver);
  Serial.print(":");
  temp = gSabertooth[driver].getTemperature(1, false);
  if (temp == MOTOR_TIMEOUT)
  {
    SendBeep(50, 1);
    Serial.println(F(" TIMEOUT"));
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

//***************************************************
bool GetDynamicMode(void)
{
	/* Get Dynamic Mode looks at the keyswitch and sets the global variable gDynBrakeActive
	*/
	
	bool temp = digitalRead(DYN_KEY_SW);
	if (gDynBrakeActive != temp)
	{
		Serial.print(F("L:4: Dynamic Mode "));
		if(temp)
		{
			Serial.println("ON");
		}
		else
		{
			Serial.println("OFF");
		}
	gDynBrakeActive == temp;
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
    Serial.println(F("S:f:"));
  }
  else if ((digitalRead(PIN_DIR_FWD) == HIGH)  && (gDirection == DIR_NONE) && (digitalRead(PIN_DIR_REV) == LOW))
  {
    gDirection = DIR_REV;    //We are going reverse
    gControlsChanged = true;
    Serial.println(F("S:r:"));
  }
  else if ((digitalRead(PIN_DIR_FWD) == HIGH) && (gDirection != DIR_NONE) && (digitalRead(PIN_DIR_REV) == HIGH))
  {
    gControlsChanged = true;
    gDirection = DIR_NONE;   //We are in Neutral
    Serial.println(F("S:n:"));
  }

  //otherwise business as usual - no change needed - move on through
  return 0;
}

//***************************************************
int GetDynamic(void)
{
  //read the raw analog value and calculate the corresponding notch
  /*NOTE: debouncing is somewhat complicated for a deliberate reason:-
    when the lever is advanced several notches over a couple of seconds, we only
    want to know when it has stopped moving. this is to give the motor controllers and sound system a solid
    starting point and ending point for multi-notch Dyn braking changes
  */

  static int newval[3] = {0, 0, 0};
  static int idx = 0;

  switch (idx)
  {
    case 0:
      newval[0] = analogRead(DYNAMIC_PIN);           //take first analog sample
      newval[1] = 0;
      newval[2] = 0;
      idx++;
      return 0;

    case 1:
      newval[1] = analogRead(DYNAMIC_PIN);           //take second analog sample
      if (newval[0] != newval[1])
        idx = 0;                            //we are in the middle of a change so reset for three new samples
      else
        idx++;                              //1st two analog samples match, so advance ready for third
      return 0;

    case 2:
      newval[2] = analogRead(DYNAMIC_PIN);
      idx = 0;                                //completed 3 samples. start again either way
      if (newval[0] != newval[1] || newval[1] != newval[2])
      {
        return 0;                            //Still in the middle of a change so reset for three new samples
      }
      break;
  }

  newval[0] = CalcNotch(newval[0]);              //success to get here - three analogs match, not make it a notch


  if (gDynamicNotch != newval[0])
  {
    //value has to have changed, and be measured as identical analog values three times to get here
    gControlsChanged = true;

    if (gDirection == DIR_NONE && newval[1] > 0)
    {
      CalcControlStatus(STATUS_ERROR, "ERROR: Dynamic Brake Set. Reverser Neutral");
      return 1;
    }

    gDynamicNotch = newval[0];

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
  */

  static int newval[3] = {0, 0, 0};
  static int idx = 0;

  switch (idx)
  {
    case 0:
      newval[0] = analogRead(THROTTLE_PIN);           //take first analog sample
      newval[1] = 0;
      newval[2] = 0;
      idx++;
      return 0;

    case 1:
      newval[1] = analogRead(THROTTLE_PIN);           //take second analog sample
      if (newval[0] != newval[1])
        idx = 0;                            //we are in the middle of a change so reset for three new samples
      else
        idx++;                              //1st two analog samples match, so advance ready for third
      return 0;

    case 2:
      newval[2] = analogRead(THROTTLE_PIN);
      idx = 0;                                //completed 3 samples. start again either way
      if (newval[0] != newval[1] || newval[1] != newval[2])
      {
        return 0;                            //Still in the middle of a change so reset for three new samples

      }
      break;
  }

  newval[0] = CalcNotch(newval[0]);              //success to get here - three analogs match, not make it a notch

  if (gThrottleNotch != newval[0])                 //has notch changed?
  {
    //change has happened, and settled down sufficient to get three identical readings in order to get here

    gControlsChanged = true;

    if (gDirection == DIR_NONE && newval > 0)
    {
      CalcControlStatus(STATUS_ERROR, "ERROR: Throttle Set. Reverser Neutral");
      return 1;
    }

    gThrottleNotch = newval[0];

  }

  return 0;
}

//***************************************************
//Serial Config & Handling routines
//***************************************************

void ConfigComms(void)
{
  //Configures two RS232 data Streams - one for Debug/Raspi Comms and the other to talk to the motor controllers

  gRaspi_RxBuffer.reserve(SERIAL_BUFSIZE);
  gRaspi_TxBuffer.reserve(SERIAL_BUFSIZE);

  //initialize Comms
  Serial.begin(RASPI_SERIAL_SPEED);  //Comms to Debug/Raspi
  SWSerial.begin(MOTOR_SERIAL_SPEED);  //Motor Controller comms

  //give the serial lines time to come up before using them - wait three seconds
  delay(3000);

  gSabertooth[0].setGetTimeout(1000);
  gSabertooth[1].setGetTimeout(1000);
  gSabertooth[2].setGetTimeout(1000);
  

  //Announce Our arrival
  Serial.print(F("L:1:Loco Control Stand Version: "));
  Serial.println(F(VERSION));
}

//***************************************************
void ServiceComms(void)
{
  //Check to see if anything has arrived from the Raspi
  if (gRaspi_RxComplete)
  {
    Serial.print(F("Received:-"));
    Serial.println(gRaspi_RxBuffer);     //todo - replace with a call to a future Raspi command interpreter

    switch (gRaspi_RxBuffer[1])
    {
      case 'H':
      case 'h':
        gRaspiOK = true;                    //set true when we receive heartbeat from raspi
        break;
      case 'R':
      case 'r':
        //allows raspi to force reboot of arduino
        softReset();
        break;
      default:
        Serial.print(F("L:4:Unrecognised command from Raspi:-"));
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
  Serial.println(F("L:4:Heartbeat"));
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

  gThrottleNotch = 0;

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
    if ((gDirection == DIR_NONE) && (gThrottleNotch == 0) && (gDynamicNotch == 0))
    {
      //controls are safe and no errors detected with them
      gInitialized = true;
      ResetVigilanceWarning();
      CalcControlStatus(STATUS_IDLE, "Initialized OK Now at Idle");
    }
    else
    {

      if (gDirection != DIR_NONE)
      {
        CalcControlStatus(STATUS_ERROR, "INIT:Reverser not in Neutral");
      }
      if (gThrottleNotch != 0)
      {
        CalcControlStatus(STATUS_ERROR, "INIT:Throttle not at zero");
      }
      if (gDynamicNotch != 0)
      {
        CalcControlStatus(STATUS_ERROR, "INIT:Dynamic not zero");
      }

      SendBeep(1000, 1);

      gControlsChanged = 0;
    }
  } while (!gInitialized);

  //signal all OK
  SendBeep(200, 2);

}


//***************************************************


