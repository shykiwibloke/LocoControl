/****************************************************
**                                                 **
**  Loco Control Stand version                     **/
      #define VERSION "1.3.0"
/*                                                 **
**  Written by Chris Draper                        **
**  Copyright (c) 2016 - 2017                      **
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

#include <SoftwareSerial.h>   //used to drive comms to motor controllers
#include <USBSabertooth.h>    //comms library for sabertooth controllers
#include <Servo.h>            //used for speedo - can be removed to make space if no speedo fitted 
#include <SimpleTimer.h>      //used for vigilance and heartbeat type functions

//***************************************************
// Defines
//***************************************************

 //general constants
 
 const int LOG_LEVEL = 4;	//change at compile time to remove debug info from screen
                          //priority 1 = cant run until corrected
                          //priority 2 = serious but can run
                          //priority 3 = informational
                          //priority 4 = debug info, not needed in normal operation

 
 #define SERIAL_SPEED	9600      //9600 for slower testing 
 #define SERIAL_BUFSIZE	50		  //Reserve 50 bytes for sending and receiving messages with the raspi
 #define MOTOR_TIMEOUT -32768   //value returned by Sabertooth library when comms fails
 
 #define NOTCH1 760       //below NOTCH1 is IDLE
 #define NOTCH2 790       //below NOTCH2 is NOTCH1
 #define NOTCH3 830       //below NOTCH3 is NOTCH2   etc
 #define NOTCH4 855
 #define NOTCH5 880
 #define NOTCH6 915
 #define NOTCH7 945
 #define NOTCH8 975     //Above NOTCH8 is Notch 8 (i.e. measurement in the range of 912 - 1024)

 //Status Definitions

 const int STATUS_ERROR = 99;
 const int STATUS_IDLE = 10;
 const int STATUS_POWER = 20;
 const int STATUS_DYNAMIC = 30;

 const int DIR_NONE = 0;
 const int DIR_FWD = 1;
 const int DIR_REV = 2;
 
 //Sabertooth configuration defines
 #define MOTOR_NOTCH      255            //the maximum value we can send for throttle or dynamic brake per notch position - 1

 //Sabertooth ports for ditch lights and headlights
 //TODO - fix this for healights dim, high and ditchlights flashing for horn
 #define HEADLIGHT_ST	1
 #define DITCHLIGHT_ST	2
 

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
 #define MOMENTUM_PIN  A2		//A2 Momentum (Train Weight bias) 10k Potentiometer

 //************************************************** 
 //Digital I/O								
 //D1&2 - reserved for RS232 comms via USB Serial (a.k.a. Debug or programming port)
 //D3&4 - reserved for soft serialRS232 comms port (Sabertooth Motor Controllers)
 #define PIN_DIR_FWD 	5       //D5 - Microswitch - internal pullup - active low
 #define PIN_DIR_REV 	6       //D6 - Microswitch - internal pullup - active low
 #define TACHO_IN		7       //D7 - input for tachometer (interrupted IR)
 #define BTN_HORN		8       //D8 - input switch on console
 //D9 is spare 
 #define BTN_VIGILANCE	10      //D10 - pushbutton on console - note input is active HIGH
 #define SPEEDO_PIN		11      //D11 - RC Servo output to drive Speedometer
 #define BUZZER_PIN		12      //D12 - Buzzer in console
 #define VIGILANCE_EN	13      //D13 - Vigilance Button Enable + the on board LED
 #define HEADLOW_PIN    A3      //A3 - Headlights to low power
 #define HEADHIGH_PIN   A4      //A4 - Headlights to high power 
 
 
//***************************************************
// Global Variable Definitions
//***************************************************

//General Globals

bool gInitialized       = false;
bool gRaspiOK           = false; 
bool gControlsChanged   = false;
int  gControlStatus     = 0;

//Control Inputs
int  gThrottleNotch     = 0;
int  gDynamicNotch      = 0;
int  gDirection         = 0;
int  gMomentum          = 0;
int  gBattery           = 0;
int	 gHorn              = 0;
int  gHeadlights        = 0;
int  gMotorCurrent[7]   = {0,0,0,0,0,0,0};  //last read motor current TOTAL + individual motors
int	 gVigilanceCount    = 0;                //Timer count for the Vigilance Alarm

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

Servo gSpeedo;                              // create servo object to control Speedo needle

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
	gSpeedo.attach(SPEEDO_PIN);

    //Perform tests ensure system is safe to operate
	TestSystem();		

    //Check slow controls and heartbeat comms every five seconds
	gtimer.setInterval(5000,DoTimedIntervalChecks);                    
 
}

//***************************************************
// Main loop & Core Subroutines
//***************************************************

//Main loop must be executed frequently for system to run reliably 
// (i.e. no long Delay() type statements in subroutines unless deliberately done)

void loop()
{

    ReadInputs();                   //Read input variables

	if(gControlsChanged)            //only process if controls have changed
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
  
  if(digitalRead(BTN_VIGILANCE))    //Hitting vigilance button is considered a control change
    ResetVigilanceWarning();        //But should not set the controls changed flag - just reset vigilance
 
  if(rtnval)	 
  {
    //todo - need to evaluate any control errors that arise during running
    // set gcontrols changed back to false if we are waiting for a correction. Failsafe called if real problem
  }
}

//***************************************************
void EvaluateState(void)
{

  if(gControlStatus == STATUS_ERROR)
  {
    //todo - ensure we are in a safe state and stay there until the error clears
    Serial.println(F("L:4:EvalutateState Error Condition Detected"));
  }
  else
  {
    if((gThrottleNotch == 0) && (gDynamicNotch == 0))
    {
      CalcControlStatus(STATUS_IDLE,"Idle");
      SetMotorSpeed(0);
    }
    else if(gDirection != DIR_NONE)
    {
      //Direction has been set to forward or reverse - dont actually care which here.		
      
      if(gDynamicNotch > 0)
      {
        CalcControlStatus(STATUS_DYNAMIC, "Dyn Brake ON");  
        //todo - flesh out dynamic once throttle debugged
        //when dynamic advanced - gradually increase permitted regen amperage. momentum = set value from pot.
        //when dynamic reduced - gradually decrease permitted regen amperage. momentum = max
      }
      else if (gThrottleNotch > 0)
      {
        CalcControlStatus(STATUS_POWER, "Throttle ON");
        //todo - In Dynamic mode - when throttle advanced - momentum = set value from pot
        //when throttle reduced - keep 'speed up to within -2 amps to 0 amps. momentum=max
        
		//scale notch to an actual speed setting, set them in gMotorSpeed and issue the commands
        SetMotorSpeed(gThrottleNotch);
      } 
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
    
    if(NewStatus != gControlStatus)
    {
        if(NewStatus == STATUS_ERROR || gControlStatus == STATUS_ERROR)
        {
            Serial.print(F("L:1:Critical Error. "));    //print the error heading
        }
        else
        {
        Serial.print(F("L:4:"));      //print the normal heading
        }      

        //print the rest of the message for either case
    
        Serial.print(F("Status was "));
        Serial.print(gControlStatus);
        Serial.print(F(" now "));
        Serial.print(NewStatus);
        Serial.print(F(" : "));
        Serial.println(Msg);              //print the passed message

        gControlStatus = NewStatus;       //set the new status
  }
}

//***************************************************
int CalcNotch(int newval)
{
    //nb: this routine serves both throttle and dynamic brake
    
    if(newval < NOTCH1)  return 0;   //less than Notch1 is IDLE
    if(newval < NOTCH2)  return 1;
    if(newval < NOTCH3)  return 2;
    if(newval < NOTCH4)  return 3;
    if(newval < NOTCH5)  return 4;
    if(newval < NOTCH6)  return 5;
    if(newval < NOTCH7)  return 6;
    if(newval < NOTCH8)  return 7;
  
    return 8;    //no other possibilities left except Notch 8

}

//***************************************************
void CalcMomentum(void)
{
    //Reads the momentum value and set the ramp variables accordingly if it has changed.
    
    //read it and double it. Arduino range is roughly half that of the sabertooth.
    int newval = analogRead(MOMENTUM_PIN)*2;
    
    //limitcheck the value to fit ramping range before comparing it to last setting/using it
    if(newval > 2046) newval = 2046;
    if(newval < 0) newval = 0;
    
    //has it changed?
    if(gMomentum != newval)
    {
        
        Serial.print(F("L:4:Set Momentum to:  "));
        Serial.println(newval);
        
        // and set it
        gMomentum = newval;
        SetMotorRamping(gMomentum);
    }
}

//***************************************************
void DoTimedIntervalChecks(void)
{
    //Stuff that only needs to be checked every five seconds or so Called by timer routine
    static int callcount = 0;
    static bool OddCall = false;
    
    CheckHeadlights();        //checks the headlights/ditchlights
    SendCommsHeartbeat();     //we will send it every five seconds even if there are commands going to the controllers.

    if(OddCall)
    {
        GetMotorCurrent();   //get and send to the raspi
        OddCall = !OddCall;
    }
  
    if(callcount == 3)     //stuff every 30 seconds, but opposite the next lot
    {
        GetBattery();
    }
    if(callcount++ > 6)    //stuff that can be done every 30 seconds
    {
        GetTemperature();         //get and send to the raspi
        CalcMomentum();
        callcount = 0;
    }
  
  //Calculate Vigilance warnings
 
    if(gDirection == DIR_NONE)
    {
        ResetVigilanceWarning();        //suppress vigilance when in neutral.  
    } 
    else
    {
        if(gVigilanceCount++ > 6)   //more than 30 seconds since a control was touched
        {
            Serial.println(F("L:4:Vigilance Light"));
            SetVigilanceWarning(false);
        }
    
        if(gVigilanceCount > 8)   //light been on for at least 10 seconds
        {
            Serial.println(F("L:4:Vigilance Buzzer"));
          	SetVigilanceWarning(true);   //buzzer goes off 10 seconds later
        }
        
        if (gVigilanceCount > 10)
        {
            //todo - drop throttle to zero, amperage full, momentum none, go to error mode, send message to raspi
        }
    }  
}

//***************************************************
//Output setting routines
//***************************************************

void SetMotorSpeed(int Notch)
{
	
    int speed = MOTOR_NOTCH * Notch;        //used to scale the provided notch to a speed setting the controller understands (-2047-0-2047 range)
    
    if(gDirection == DIR_REV)  speed = -speed;                     //a negative number makes the motor go in reverse, positive = go forward
    
    Serial.print(F("L:4:Motor set to "));
    Serial.println(speed);
	
	SendSoundCommand("T");
	SendSoundCommand('0' + Notch);
	
	//send the commands to the motor controllers
	
	gSabertooth[0].motor(1, speed);     //first pair
	gSabertooth[0].motor(2, speed);
	gSabertooth[1].motor(1, speed);     //2nd pair
	gSabertooth[1].motor(2, speed);
	gSabertooth[2].motor(1, speed);     //3rd pair
	gSabertooth[2].motor(2, speed);

}

//***************************************************

void SetDynamicBrake(int Notch)
{
	
    int brake = (8-MOTOR_NOTCH) * Notch;            //used to scale the provided notch to a speed setting the controller understands (-2047-0-2047 range)
                                                  //Note the notch is inverted so most gentle braking is on Notch one
    if(gDirection == DIR_REV)  brake = -brake;      //a negative number makes the motor go in reverse, positive = go forward
                                                  //Note this must stay in the same direction of travel for Sabertooth braking to work properly
    Serial.print(F("L:4:D Brake set to "));
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

  Serial.print(F("L:4:Set Ramping to "));
  Serial.println(newvalue);

	gSabertooth[0].setRamping(newvalue);
	gSabertooth[1].setRamping(newvalue);
	gSabertooth[2].setRamping(newvalue);
	
}

//***************************************************

void SetMotorAmps(int NewVal)
{
  //todo - sets all motors to the prescribed amperage amount within the max and min bounds
  //OR - monitors amperage and adjusts speed to keep amps within a certain range

  Serial.println(F("L:4:Set Motor Amps"));

  
  //  gSabertooth[0].   <<TODO HERE >>
	//	gSabertooth[1].
  //	gSabertooth[2].  

}

//***************************************************
void CheckHeadlights(void)
{
	int state = 0;
	
    if (digitalRead(HEADLOW_PIN) == 0) state = 1;
    if (digitalRead(HEADHIGH_PIN) == 0) state = 2;
    
    if(gHeadlights != state)
    {
        
        Serial.print(F("L:4:Headlights now "));
        Serial.println(state);
        gHeadlights = state;
        
         
        //1 to set loco lights on, 0 to turn off
        gSabertooth[HEADLIGHT_ST].power(1, state);
    }
}

//***************************************************
void SetDitchlightsFlashing(bool flash)
{

    //True sets them flashing, false places them back under the control of the headlights logic
    Serial.println(F("L:4:Set Ditchlights "));
    
    if(flash)
    {
        gSabertooth[DITCHLIGHT_ST].power(1, 0);  //turn off the ditchlight output to trigger flashing circuit
    }
    else
    {
    	gSabertooth[DITCHLIGHT_ST].power(1, 1);			//turn them on solid.
    }
}

//***************************************************

void SetSpeedo(int speed)
{
    //Set the speedo needle to point to the supplied speed.
    
    Serial.println(F("L:4:Set Speedo "));
    speed = map(speed, 0, 1023, 0, 180);     // scale it to use it with the servo (value between 0 and 180)
    gSpeedo.write(speed);                  // sets the servo position according to the scaled value
}

//***************************************************

void CheckHorn(void)
{	
    int HornState = digitalRead(BTN_HORN);
  
	//Set the horn to match the button state.
	if(HornState == 0 && gHorn == 0)			//Horn button pressed, we have not seen this yet
	{
    Serial.println(F("L:4:Horn"));

		SendSoundCommand("H");
		gHorn = 1;
    //todo - set ditchlights flashing for ten seconds
		ResetVigilanceWarning();		//horn press also resets the vigilance timer.

	}	
	else if(HornState == 1 && gHorn == 1)		//Horn button released, we have not seen this yet
	{
		SendSoundCommand("J");
		gHorn = 0;
	}
}

//***************************************************
void SetVigilanceWarning(bool SoundBuzzer)
{

    //Set off Vigilance light the vigilance LED

	digitalWrite(VIGILANCE_EN,HIGH);

    if(SoundBuzzer) digitalWrite(BUZZER_PIN,HIGH);
	
}

//***************************************************
void ResetVigilanceWarning(void)
{
    //Reset Vigilance buzzer and light the vigilance LED

	Serial.println(F("L:4:Vigilance Reset"));
	digitalWrite(VIGILANCE_EN,LOW);
	digitalWrite(BUZZER_PIN,LOW);
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
	

    if(Motor == 0) //slot zero used to calc total amperage
    {
        gMotorCurrent[0] = gMotorCurrent[1] + gMotorCurrent[2] + gMotorCurrent[3] + gMotorCurrent[4] + gMotorCurrent[5] + gMotorCurrent[6]; 
    }
    else
    {
        //Get current via first driver using ascii representation of motor number
    	gMotorCurrent[Motor] = gSabertooth[0].getCurrent((Motor+49), false);  
 
	    if(abs(gMotorCurrent[Motor]) < 300 || gMotorCurrent[Motor] > -31000)  //if less than 300ma or timed out
	        gMotorCurrent[Motor] = 0;
	    else
	        gMotorCurrent[0] = gMotorCurrent[0] / 100;              //change milliamps to Amps
    }

    //log the retrieved value or calculated total
    Serial.print(F("L:3:Motor Amps"));
    Serial.print(Motor);
    Serial.print(":");
    Serial.println(gMotorCurrent[0]);
 

    Motor ++;
    if(Motor > 6) Motor = 0;
   
}

//***************************************************
void GetBattery(void)
{

    //Battery should be the same for all controllers - so only have to read one
    gBattery = gSabertooth[2].getBattery(1,false);
    if(gBattery == MOTOR_TIMEOUT)
    {
        SendBeep(50,1);
        gBattery = 0;
    } else if(gBattery > 0)
    {
        gBattery = gBattery/10;
    }
    
    Serial.print(F("L:4:Battery="));
    Serial.print(gBattery);
    Serial.println("v");

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
    if(temp == MOTOR_TIMEOUT)
    {
        SendBeep(50,1);
        Serial.println(F(" TIMEOUT"));
    }
    else
    {
      Serial.print(temp);
      Serial.print(":");
      Serial.println(gSabertooth[driver].getTemperature(2, false));
    }
    driver ++;
    if(driver > 2) driver = 0;
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
    
    if((digitalRead(PIN_DIR_FWD) == LOW) && (digitalRead(PIN_DIR_REV) == LOW))
    {
        //cant have both directions low at the same time - error
        gDirection = DIR_NONE;
        CalcControlStatus(STATUS_ERROR,"ERROR: Reverser FWD AND REV");
        return 1;
    }
    else if((digitalRead(PIN_DIR_FWD) == LOW)&& (gDirection == DIR_NONE) && (digitalRead(PIN_DIR_REV) == HIGH))
    {
        gDirection = DIR_FWD;    //We are going forward
        gControlsChanged = true;
        Serial.println(F("L:4:Dir FWD"));
    }
    else if((digitalRead(PIN_DIR_FWD) == HIGH)  && (gDirection == DIR_NONE) && (digitalRead(PIN_DIR_REV) == LOW))
    {
        gDirection = DIR_REV;    //We are going reverse
        gControlsChanged = true;
        Serial.println(F("L:4:Dir REV"));
    }
    else if((digitalRead(PIN_DIR_FWD) == HIGH)&& (gDirection != DIR_NONE) && (digitalRead(PIN_DIR_REV) == HIGH))
    {
        gControlsChanged = true;
        gDirection = DIR_NONE;   //We are in Neutral
        Serial.println(F("L:4:Dir NONE"));
    }
    
    //otherwise business as usual - no change needed - move on through
    return 0;
}


//***************************************************
int GetDynamic(void)
{
    //read the raw analog value and calculate the corresponding notch
    int newval = CalcNotch(analogRead(DYNAMIC_PIN));
    
    if(gDynamicNotch != newval)
    {
        //by evaluating at notch level - we remove a lot of jitter automatically
        gControlsChanged = true;
        
        Serial.print(F("L:4:Dynamic Notch now "));
        Serial.println(newval);
        
        if(gDirection == DIR_NONE && newval > 0)
        {
            CalcControlStatus(STATUS_ERROR,"ERROR: Dynamic Brake Set. Reverser Neutral");
            return 1;
        }
        
        gDynamicNotch = newval;
    
    }
    
    return 0;
}

//***************************************************
int GetThrottle(void)
{

    //read the raw analog value and calculate the corresponding notch
    int newval = CalcNotch(analogRead(THROTTLE_PIN));
    
    if(gThrottleNotch != newval)
    {
        //by evaluating at notch level - we remove a lot of jitter automatically
        gControlsChanged = true;
    
        Serial.print(F("L:4:Throttle Notch now "));
        Serial.println(newval);
    
        if(gDirection == DIR_NONE && newval > 0)
        {
            CalcControlStatus(STATUS_ERROR,"ERROR: Throttle Set. Reverser Neutral");
            return 1;
        }
        
        gThrottleNotch = newval;
    
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
    Serial.begin(SERIAL_SPEED);  //Comms to Debug/Raspi
    SWSerial.begin(SERIAL_SPEED);  //Motor Controller comms
    
    //give the serial lines time to come up before using them - wait three seconds
    delay(3000);

    gSabertooth[0].setGetTimeout(1000);
    gSabertooth[1].setGetTimeout(1000);
    gSabertooth[2].setGetTimeout(1000);
    
    //Announce Our arrival

    Serial.println(" ");
    Serial.println("************************************");
    Serial.println(" ");
    
    Serial.print(F("L:2:Loco Control Stand Version: "));
    Serial.print(F(VERSION));
    Serial.println(" ");
    Serial.println("************************************");
    Serial.println(" ");
 
}

//***************************************************
void ServiceComms(void)
{
	//Check to see if anything has arrived from the Raspi
	if (gRaspi_RxComplete) 
	{
		Serial.print(F("Received:-"));
		Serial.println(gRaspi_RxBuffer);     //todo - replace with a call to a future Raspi command interpreter
		
    switch(gRaspi_RxBuffer[1])
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
      case 'S':
      case 's': 
        //S for 'Skip' - allows user to skip wait for motor controllers to come up and go directly into test mode.
//         gNoMotorComms = true;
         break;
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
		  gRaspi_RxComplete = true;
		}
	}
}

//***************************************************
void SendCommsHeartbeat(void)
{
  Serial.println(F("L:4:Heartbeat"));
  gSabertooth[0].keepAlive();
  gSabertooth[1].keepAlive();
  gSabertooth[2].keepAlive();
}

//***************************************************
void SendSoundCommand(const char* SoundCommand)
{

	Serial.print("S:");				// 'S:' = Sound Command - to differentiate from all the error and log messages which are prefixed with L:
	Serial.println(SoundCommand);
}

//***************************************************
//  Send Beep - beep once specified time
//***************************************************
void SendBeep(unsigned long len, int count)
{
   for(int f = 0; f<count;f++)
   {
      digitalWrite(BUZZER_PIN,HIGH);   //buzzer on
      delay(len); //wait
      digitalWrite(BUZZER_PIN,LOW);   //buzzer off
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
	Headlights		- Switch off/low/high - uses analog inputs
 	Vigilance Reset	- pushbutton active high - only works when vigilance is enabled 


 OUTPUTS
	Buzzer			- used for warning buzzer & light (vigilance, serious error etc)
	Vigilance Enable - turns on the LED in the vigilance button and enables the Vigilance reset button

 */

    pinMode(PIN_DIR_FWD, INPUT);            //Set pin for input
    digitalWrite(PIN_DIR_FWD, HIGH);		//enable internal pullup
    
    pinMode(PIN_DIR_REV, INPUT);            //Set pin for input
    digitalWrite(PIN_DIR_REV, HIGH);        //enable internal pullup
    
    pinMode(TACHO_IN, INPUT);               //Set pin for input
    digitalWrite(TACHO_IN, HIGH);			//enable internal pullup
    
    pinMode(BTN_HORN, INPUT);               //Set pin for input
    digitalWrite(BTN_HORN, HIGH);           //enable internal pullup
    
    pinMode(BTN_VIGILANCE, INPUT);          //Set pin for input  - remember it is active HIGH
    
    pinMode(HEADLOW_PIN, INPUT);            //Set pin for input
    digitalWrite(HEADLOW_PIN, HIGH);        //enable internal pullup
    
    pinMode(HEADHIGH_PIN, INPUT);           //Set pin for input
    digitalWrite(HEADHIGH_PIN, HIGH);       //enable internal pullup
    
    pinMode(BUZZER_PIN, OUTPUT);            //Set pin for output
    
    pinMode(VIGILANCE_EN, OUTPUT);          //Set pin for output
    
    Serial.println(F("L:4:I/O Setup"));
  
}

//***************************************************
void TestSystem(void)
{
	//test motor controllers are available here
	//Only one controller present is valid to continue

    gThrottleNotch = 0;
    SetMotorSpeed(0);                   //Force all motors to stop - issue commands before checking controllers are there for safety.

//	if (TestMotorControllers())
//	{
		SendBeep(200,2);
		
		//Ensure motor drivers are in a known minimal state
    GetTemperature();         //get and send to the raspi
    CalcMomentum();
		GetBattery();
		CheckHeadlights();
		SetDitchlightsFlashing(false);		//ditchlights not flashing (under headlight control)

/*	}
	else
	{
		Serial.println(F("L:1:No motor controllers present. Entering Panel DEBUG Mode"));
		gNoMotorComms = true;
        SendBeep(300,4);
 
	}
	*/
	do {

      //Loop here until all controls are in the safe position
      // Reverser in Neutral. Throttle and Dynamic in idle
      //remember they are mechanically interlocked so really only have to check Dir NONE 
      //  - but others are checked in case of faults which could cause unpredictable operation
      //note Buzzer is used to indicate the controls are not setup correctly

	    ReadInputs();
	    if((gDirection == DIR_NONE) && (gThrottleNotch == 0) && (gDynamicNotch ==0))
	    {
	    	//controls are safe and no errors detected with them
	    	gInitialized = true;
        ResetVigilanceWarning(); 
        CalcControlStatus(STATUS_IDLE,"Initialized OK Now at Idle");
	    }
        else
	    {
		
	    	if(gDirection != DIR_NONE)
	    	{
	    	  CalcControlStatus(STATUS_ERROR,"INIT:Reverser not in Neutral");
	    	}
	    	if(gThrottleNotch != 0)
	    	{
	    	  CalcControlStatus(STATUS_ERROR,"INIT:Throttle not at zero");
	    	}
	    	if(gDynamicNotch != 0)
	    	{
	    	  CalcControlStatus(STATUS_ERROR,"INIT:Dynamic not zero");
	    	}

        SendBeep(1000,1);
		
        gControlsChanged = 0;
	    }
	} while (!gInitialized);
}

//***************************************************
bool TestMotorControllers(void)
{
    //tests that the three motor controllers are present and ready to work
    //note also acts as a test for the traction isolator....
    //battery voltage must be above ten volts for this to succeed
    
    int timeout;
    int alive = 0;
    
    for(int f=0;f<3;f++)
    {
    	Serial.print(F("L:1:Contacting Motor Controller "));
      Serial.println(f+1);
 
      timeout = gSabertooth[f].getGetTimeout();
        if(timeout != 1000) {
        			Serial.print(F("L:1:Disconnected or faulty. Timeout returned was: "));
              Serial.println(timeout);
        } else {
    			Serial.println(F("L:1:Online."));
     			alive += 1;
        }
      delay(1000); 
    }
    
    return (alive > 0);

}

