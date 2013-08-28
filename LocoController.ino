//#define DEBUGON;

/*
This the the LocoController Sketch for use on my girlfriends 24vDC Electric 5" Gauge Loco. This work can be adapted for use on other engines, however this work is provide with ABSOLUTELY NO WARRANTY and I take NO LIABILITY if you use this for your own purposes. You have been warned.

This Sketch is designed for use on an ARDUINO NANO/ATMega328. It pokes the PWM registers DIRECTLY and DOES NOT USE the analogWrite functions __AT__ALL__. If you plan on using this on a non ATMega328 chip, you'll need to modify these appropriately - YOU HAVE BEEN WARNED
The Loco Control panel is fairly simple. It has the following:

7 backlit push buttons (Each backlight has variable brightness control, to indicate if the light is on, or which direction we're set for):

Brake 1 - Switches the "flyback diode" in reverse with the motors providing a .7v drop and some dynamic braking effect
Brake 2 - Switches the loco contactor to "Short Out" the motors, providing a very effective dynamic brake
Forward - This puts the loco contactor into the "Forward" position assuming that the contactor is currently in neutral AND the throttle is at zero
Neutal - This puts the loco into neutral assuming that the throttle is at zero
Reverse - This puts the loco contactor into the "Reverse" position assuming that the contactor is currently in neutral AND the throttle is at zero
Horn - This activates the Horn
Light - This activates the headlight on the loco

There is also a keyswitch, which when activated will disable the throttle, and short out the motors bringing the engine to an almost dead stop. For normal operation the key must be inserted and switched on. If you remove the key, you'll come to a VERY quick stop.

There are two "Special Bootup Modes" which comprise of:

Pressing "Brake1" on startup, toggles the throttle limit. You can determine if the throttle limit is set on bootup by how the display flashes at you. If the throttle limit is enabled, the backlights will flash at you once for one second, and then again for half a second to tell you you're in "Limited Mode"
Pressing "Brake2" on startup "Sets" the throttle limit to the current value that the throttle is set to. This is confirmed by a 1 second flash on the backlights, followed by a second one second flash. If the throttle limit is enabled, you'll then get a third "half second" flash on the display.

Summary:

For ANY operation, the keyswitch must be enabled

The throttle must be in the "zero" (Less than a reading of 10 on the ADC) position in order to:
 - Move from Neutral into Forward
 - Move from Neutral into Reverse
 - Move from Forward into Neutral
 - Move from Reverse into Neutral
 - Activate the dynamic brakes (Either Brake1 or Brake2)

Once in a "gear" (Forward or Reverse)
 - If in "Normal" mode, you have full control of the throttle up to it's maximum output
 - If in "Limited" mode, you have control of the throttle up to the set limit
 
To toggle Throttle Limit mode, hold Brake1 on power on
To set the Throttle Limit mode, set the throttle to your desired limit, and hold "Brake2" on startup

Startup backlight "Flashes"

Bright - 1s - Dim - Normal operation
Bright - 1s - Dim - .5s - Bright - .5s - Dim - Throttle Limited Operation
Bright - 1s - Dim - 1s - Bright - 1s - Dim - Throttle Limit Set
Bright - 1s - Dim - 1s - Bright - 1s - Dim - .5s - Bright - .5s - Dim - Throttle Limit Set and Throttle Limit Enabled


*/

//Define our inputs
#define THROTTLE_INPUT A0 //Throttle input - Obviously this one is an analog input
#define BUTTONS_INPUT A1 //The buttons are wired up on a single analog input, as a multi-tap voltage divider. The buttons are each biased with a slightly different voltage, the internal pull-up is enabled and when you press the button it applies this voltage (which we then read) to the analog input
#define CURRENT_INPUT A2 //Not presently used, possible future expansion for current sensing
#define KEY_INPUT 2 //Key Switch input

//Set up some parameters for the ShiftPWM powered Shift Register that handles the dimming of each of the backlights in the 7 push buttons
#define SHIFTPWM_NOSPI
const int ShiftPWM_dataPin = 11;
const int ShiftPWM_clockPin = 13;
//Set out LED indication output (Actually uses 11, 12 and 13)
const int ShiftPWM_latchPin=12;

//Let's define the output pins on the Shift Register for the backlights
#define BRAKE1_LED 7
#define BRAKE2_LED 6
#define FORWARD_LED 5
#define NEUTRAL_LED 4
#define REVERSE_LED 3
#define LIGHT_LED 2
#define HORN_LED 1

//Define our logic outputs to the Loco - These at 5V TTL on/off outputs, that then trigger FETs located on the loco.
#define FORWARD_PIN 9 //Output to the FET that controls the forward pin on the contactor
#define REVERSE_PIN 8 //Output to the FET that controls the reverse pin on the contactor
#define LIGHT_PIN 7 //Output to the FET that controls the headlight
#define HORN_PIN 6 //Output to the FET that controls the horn

//Define our register for our PWM output
#define PWM_PIN OCR2B // Digital Pin 3  - PORTD4 - This is the Output Compare Match, that will determine what duty cycle our PWM will be

//Set up some defintions for each direction
#define FORWARD 1
#define NEUTRAL 2
#define REVERSE 3

//Set the EEPROM locations for the throttleLimit and the throttleLimitEnabled flags in the EEPROM - This will be used later if we should be limiting the user, and what limit we should set
uint8_t throttleLimitAddr = 0x00;
uint8_t throttleLimitEnabledAddr = 0x01;

//Set up our default ShiftPWM parameters
const bool ShiftPWM_invertOutputs = false; //We want regular output on the shift register
const bool ShiftPWM_balanceLoad = false; //We don't care about balacing the load as we've only one shift register
int numRegisters = 1; //We're only using one Shift Register

// include ShiftPWM.h after setting the pins!
#include <ShiftPWM.h>
//We'll need the EEPROM functions
#include <avr/eeprom.h>

//Set up the brightness levels for the LEDs
unsigned char maxBrightness = 63; //Only use 7 bits for the dimming of the backlights in the switches - Saves on CPU cycles
unsigned char pwmFrequency = 75; //75Hz - We're only talking about pulsing LEDs here
unsigned char dimBrightness = 3;

int currentDir = NEUTRAL; //Set the starting direction to NEUTRAL (Probably a good idea!)

//This will hold our throttle value;
uint8_t throttleVal;

//This will tell us if the throttle limit is enabled
uint8_t throttleLimitEnabled;

//This will hold the throttle limit
uint8_t throttleLimit;

//This will hold the analogRead from the buttons
int buttonVal;

//Headlight off by default
boolean lightState = false;

void setup() {
  
  //Let's start ShiftPWM
  ShiftPWM.Start(pwmFrequency,maxBrightness);
  ShiftPWM.SetAmountOfRegisters(numRegisters);
  
  //Let's set the KEY pin to INPUT and enable the internal pull-up
  pinMode(KEY_INPUT, INPUT);
  digitalWrite(KEY_INPUT, HIGH);
  
  //Let's show the user we're doing *something* and set the brightness of all the backlights to high
  ShiftPWM.SetAll(maxBrightness);

  //If we're debugging, we probably want to, ya know, debug!
  #ifdef DEBUGON 
    Serial.begin(9600);
  #endif  
  
  //Now, lets set PORTD4(OCR2B) (Arduino Pin 3) for output in the DDR4 register - This is because we're NOT using the arduino PWM "features"
  DDRD |= (1<<DDD3);
    
  //Clear OC2B on Compare Match (For each PWM cycle, set PORTD4 high at the start, and then pull low when we hit our PWM value)
  TCCR2A |= (1<<COM2B1);

  //Set FastPWM (See ATMega328 Datasheet for the difference between FastPWM and Phase-Correct PWM - FastPWM in this case with out set prescaler gives us 16Khz - or near enough)
  TCCR2A |= (1<<WGM21);
  TCCR2A |= (1<<WGM20);
  
  //Let's change the prescaler on Timer2 so that we're FastPWM at 15625 Hz instead of a measly 450ish
  TCCR2B &= ~(1<<CS22);
  TCCR2B |= (1<<CS21);
  TCCR2B &= ~(1<<CS20);
  
  //Lets setup the main input pins
  pinMode(THROTTLE_INPUT, INPUT);
  pinMode(BUTTONS_INPUT, INPUT);
  pinMode(CURRENT_INPUT, INPUT);
  digitalWrite(BUTTONS_INPUT, HIGH);
  
  //Now let's setup the output pins
  pinMode(FORWARD_PIN, OUTPUT);
  pinMode(REVERSE_PIN, OUTPUT);
  pinMode(LIGHT_PIN, OUTPUT);
  pinMode(HORN_PIN, OUTPUT);

  
  //Let's set everything else to off 
  digitalWrite(FORWARD_PIN, LOW);
  digitalWrite(REVERSE_PIN, LOW);
  digitalWrite(LIGHT_PIN, LOW);
  digitalWrite(HORN_PIN, LOW);
  
  //Set the output compare to Zero
  PWM_PIN = 0;
  //Turn off the OCR
  TCCR2A &= ~(1<<COM2B1);
  //Let's write a low value to the port
  PORTD &= ~(1<<PORTD3);

  delay(1000);
  //Let's Dim the backlight now that the main setup is done
  ShiftPWM.SetAll(dimBrightness);
  
  //Let's read the buttons, to see if we're doing something with the throttle limit.
  buttonVal = analogRead(BUTTONS_INPUT);
  
  //Let's read in the throttle limit from the 0x00 EEPROM address
  throttleLimit = eeprom_read_byte((uint8_t *) throttleLimitAddr);
  //Let's read in if the throttle limit is enabled
  throttleLimitEnabled = eeprom_read_byte((uint8_t *) throttleLimitEnabledAddr);
  
  //Let's make sure the throttle limit is a valid value (less than 255);
  if (throttleLimit > 255) { 
    throttleLimit = 255; 
  }
  
  //Are we enabling the throttle limit (Holding Brake1 - OR enabled on last boot)
  if ( 910 > buttonVal && buttonVal > 890 )
  {
    //Are we holding Brake1? - Yep
    if (throttleLimitEnabled == 1) {
      //Was the throttleLimit Enabled when we booted up? - Yep, in that case, we'll turn it off (later) and write that change to the EEPROM
      throttleLimitEnabled = 0;
      eeprom_update_byte_stupidarduino(throttleLimitEnabledAddr, 0x00);
    } else {
      //Was the throttleLimit Enabled when we booted up? - Nope, in that case, we'll turn it on (later) and write that change to the EEPROM
      throttleLimitEnabled = 1;
      eeprom_update_byte_stupidarduino(throttleLimitEnabledAddr, 0x01);
    }
  }

  //Are we trying to set the throttle Limit value in the EEPROM?
  if (790 > buttonVal && buttonVal > 770) {
    //Yep, let's get the value, set it, and write it into the EEPROM then
    uint8_t newLimit = analogRead(THROTTLE_INPUT) / 4;
    eeprom_update_byte_stupidarduino(throttleLimitAddr, analogRead(THROTTLE_INPUT) / 4);
    throttleLimit = newLimit;
    //Flash the display slowly to indicate that we've set the limit
    delay(1000);
    ShiftPWM.SetAll(maxBrightness);
    delay(1000);
    ShiftPWM.SetAll(dimBrightness);
  }

  
  if (throttleLimitEnabled != 1) {
    //Thorttle limit ISNT enabled - Give the user Full Throttle
    throttleLimit = 255;
  } else {
    //Flash the display quickly to indicate that we've got the limit enabled
    delay(500);
    ShiftPWM.SetAll(maxBrightness);
    delay(500);
    ShiftPWM.SetAll(dimBrightness);
  }
  
  //Aaaand we probably want to be in neutral (This is more about setting a state than anything else, we're actually already in Neutral anyway)
  changeDirection(NEUTRAL);
  
}

void loop() {
  
  if ( digitalRead(KEY_INPUT) == HIGH ) {
    //As we've got an internal pull-up, if the keyswitch is "off" the pin will read high, and we really shouldn't be doing anything
    
    //Set the OCR to zero (PWM output)
    PWM_PIN = 0;
    //Turn off the OCR
    TCCR2A &= ~(1<<COM2B1);
    //Let's write a LOW value to the port
    PORTD &= ~(1<<PORTD3);    
    //Put the loco into neutral
    changeDirection(NEUTRAL);
    while ( digitalRead(KEY_INPUT) == HIGH ) {
      delay(1000);
      //Wait until the key is enabled again
    } 
  }
  
  
  throttleVal = analogRead(THROTTLE_INPUT) / 4; //analogRead provides 0-1024, we want this to be a 0-255 value
  if (throttleVal >= throttleLimit) {
    throttleVal = throttleLimit;  
  }
  
  //Read the buttons (Oh yeah - read those buttons!)
  buttonVal = analogRead(BUTTONS_INPUT);

  //if debugging, print some stuff to the serial port.
  #ifdef DEBUGON
    Serial.println(throttleVal);
    Serial.println(buttonVal);
  #endif
  
  //Where's out throttle? At our minimum value?
  if ( throttleVal < 10 ) {
    //Set the OCR to zero
    PWM_PIN = 0;
    //Turn off the OCR
    TCCR2A &= ~(1<<COM2B1);
    //Let's write a low value to the port
    PORTD &= ~(1<<PORTD3);
    
    //If one of our braking buttons is on, then we can to do something
    if ((910 > buttonVal && buttonVal > 890) || (780 > buttonVal && buttonVal > 760)) {
      // "Light" Braking
      while (910 > buttonVal && buttonVal > 890) {
        //We want light braking on this one, so flyback diode, let's throw her into reverse!
        if (currentDir == FORWARD) {
          digitalWrite(FORWARD_PIN, LOW);
          digitalWrite(REVERSE_PIN, HIGH);
        } else if (currentDir == REVERSE) { 
          digitalWrite(REVERSE_PIN, LOW);
          digitalWrite(FORWARD_PIN, HIGH);
        }
        //Add some delay to account for bouncing in the button
        delay(100);
        buttonVal = analogRead(BUTTONS_INPUT);
      }
      
      //"Heavy" braking
      while (790 > buttonVal && buttonVal > 770) {
        //Short out the motors, just make sure both outputs are off
        digitalWrite(REVERSE_PIN, LOW);
        digitalWrite(FORWARD_PIN, LOW);
        //Add some delay to account for bouncing in the button
        delay(100);      
        buttonVal = analogRead(BUTTONS_INPUT);
      }
    //Now that we've meddled with the outputs above, let's reset everything back to the way it was
    changeDirection(currentDir);
    }

    //Are we tring to shift into forward?
    if (434 > buttonVal && buttonVal > 414) {
      //We can only do this if we're already in Neutral
      if (currentDir == NEUTRAL) {
        changeDirection(FORWARD);
      }
    } 
    //Are we trying to shift into neutral?
    else if ( 555 > buttonVal && buttonVal > 535 ) {
      changeDirection(NEUTRAL); 
    } 
    //Are we trying to shift into revese?
    else if ( 673 > buttonVal && buttonVal > 630 ) {
      //We can only do that if we're currently in neutral
      if (currentDir == NEUTRAL) {
        changeDirection(REVERSE);
      }
    }
  }
  //Well, if the throttle isn't at zero 
  else if (currentDir != NEUTRAL) {
    //AND we're not in Neutral, we probably want to go somewhere
    //Enable the OCR (PWM Output)
    TCCR2A |= (1<<COM2B1);
    //Write out throttle Value
    PWM_PIN = throttleVal;
  } else {
    //We're in Neutral, no point doing any PWM for now
    //Write zero to the OCR
    PWM_PIN = 0;
    //Disable the OCR
    TCCR2A &= ~(1<<COM2B1);
    //Write low to the port
    PORTD &= ~(1<<PORTD3);
  }
  
  //Are we pressing the horn button?
  if (173 > buttonVal && buttonVal > 153) {
    digitalWrite(HORN_PIN, HIGH);
  } else {
    digitalWrite(HORN_PIN, LOW);
  }
  
  //Are we toggling the light?
  if (307 > buttonVal && buttonVal > 287) {
    //Yep
    if (lightState == true) {
      //Light is currently on, let's turn it off and turn off the backlight
      digitalWrite(LIGHT_PIN, LOW);
      ShiftPWM.SetOne(LIGHT_LED, dimBrightness);
      lightState = false;
      delay(200);
    } else {
      //Light is currently off, let's turn it on and turn on the backlight
      digitalWrite(LIGHT_PIN, HIGH);
      ShiftPWM.SetOne(LIGHT_LED, maxBrightness);
      lightState = true;
      delay(200);
    }
  } 
  //let's have a short rest before we do the loop again
  delay(10);
}

void changeDirection(int dir) {
  //We're changing a direction, so let's disable all our outputs
  ShiftPWM.SetOne(REVERSE_LED, dimBrightness);
  ShiftPWM.SetOne(NEUTRAL_LED, dimBrightness);
  ShiftPWM.SetOne(FORWARD_LED, dimBrightness);
  digitalWrite(REVERSE_PIN, LOW);
  digitalWrite(FORWARD_PIN, LOW);

  //Let's evaluate the input and see what we're trying to do
  switch(dir) {
    case FORWARD:
      #ifdef DEBUGON
        Serial.println("Changing direction to foward");
      #endif
      //Put her into forward gear, and enable the backlight
      digitalWrite(FORWARD_PIN, HIGH);
      ShiftPWM.SetOne(FORWARD_LED, maxBrightness);
      currentDir = FORWARD;
      break;
    case REVERSE:
      //Put her into reverse gear, and enable the backlight
      #ifdef DEBUGON
        Serial.println("Changing direction to reverse");
      #endif
      digitalWrite(REVERSE_PIN, HIGH);
      ShiftPWM.SetOne(REVERSE_LED, maxBrightness);
      currentDir = REVERSE;
      break;
    default:
      //Nothing to do for Neutral, so just enable the backlight
      #ifdef DEBUGON
        Serial.println("Changing direction to neutral");
      #endif
      ShiftPWM.SetOne(NEUTRAL_LED, maxBrightness);
      currentDir = NEUTRAL;
  }
  delay(250);
}


static void eeprom_update_byte_stupidarduino(uint8_t address, uint8_t value) {
  //This function checks to see if we're trying to write the same value into the EEPROM or not (EEPROM has a limited amount of writes) and gets around some arduino stupidness to do with EEPROM values
  if (eeprom_read_byte((uint8_t *) address) == value) {
    return;
  } else {
    eeprom_write_byte((uint8_t *) address, value);
    return;
  }
}

