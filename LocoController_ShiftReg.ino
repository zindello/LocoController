#define DEBUGON;

//Define our inputs
#define THROTTLE_INPUT A0
#define BUTTONS_INPUT A1
#define CURRENT_INPUT A2

#define SHIFTPWM_NOSPI
const int ShiftPWM_dataPin = 11;
const int ShiftPWM_clockPin = 13;
//Set out LED indication output (Actually uses 11, 12 and 13)
const int ShiftPWM_latchPin=12;

#define BRAKE1_LED 7
#define BRAKE2_LED 6
#define FORWARD_LED 5
#define NEUTRAL_LED 4
#define REVERSE_LED 3
#define LIGHT_LED 2
#define HORN_LED 1


//Define our logic outputs to the Loco
#define FORWARD_PIN 9
#define REVERSE_PIN 8
#define LIGHT_PIN 7
#define HORN_PIN 6

//Define our register for our PWM output
#define PWM_PIN OCR2B // Digital Pin 3 PORTD4

//Set up some defintions for each direction
#define FORWARD 1
#define NEUTRAL 2
#define REVERSE 3

//Set up our ShiftPWM parameters
const bool ShiftPWM_invertOutputs = false;
const bool ShiftPWM_balanceLoad = false;
int numRegisters = 1;

// include ShiftPWM.h after setting the pins!
#include <ShiftPWM.h>

//Set up the brightness levels for the LEDs
unsigned char maxBrightness = 63;
unsigned char pwmFrequency = 75;
unsigned char dimBrightness = 3;

int currentDir = NEUTRAL; //1 = Forward, 2 = Neutral, 3 = Reverse;
uint8_t throttleVal;
int buttonVal;
boolean lightState = false;

void setup() {
  
  ShiftPWM.Start(pwmFrequency,maxBrightness);
  ShiftPWM.SetAmountOfRegisters(numRegisters);
  
  //First, lets set PORTD4(OCR2B) (Arduino Pin 3) for output in the DDR4 register
  DDRD |= (1<<DDD3);
    
  //Clear OC2B on Compare Match (Start high, set low when we hit our PWM value)
  TCCR2A |= (1<<COM2B1);
  //Set FastPWM
  TCCR2A |= (1<<WGM21);
  TCCR2A |= (1<<WGM20);
  //Let's change the prescaler on Timer2 so that we're FastPWM at 7812.5 Hz instead of a measly 900ish
  TCCR2B &= ~(1<<CS22);
  TCCR2B |= (1<<CS21);
  TCCR2B &= ~(1<<CS20);
  
  //Lets setup the input pins
  pinMode(THROTTLE_INPUT, INPUT);
  pinMode(BUTTONS_INPUT, INPUT);
  pinMode(CURRENT_INPUT, INPUT);
  digitalWrite(BUTTONS_INPUT, HIGH);
  
  //Now let's setup the output pins
  pinMode(FORWARD_PIN, OUTPUT);
  pinMode(REVERSE_PIN, OUTPUT);
  pinMode(LIGHT_PIN, OUTPUT);
  pinMode(HORN_PIN, OUTPUT);
//  pinMode(PWM_PIN, OUTPUT);
  
  //Let's give the user some indication that we're booting up
  
  ShiftPWM.SetAll(maxBrightness);
  
  //Let's set everything else to off
  
  digitalWrite(FORWARD_PIN, LOW);
  digitalWrite(REVERSE_PIN, LOW);
  digitalWrite(LIGHT_PIN, LOW);
  digitalWrite(HORN_PIN, LOW);
  PWM_PIN = 0;
  delay(1000);
  ShiftPWM.SetAll(dimBrightness);

  
  changeDirection(NEUTRAL);
  #ifdef DEBUGON 
    Serial.begin(9600);
  #endif
}

void loop() {
  throttleVal = analogRead(THROTTLE_INPUT) / 4; //analogRead provides 0-1024, we want this to be a 0-255 value
  buttonVal = analogRead(BUTTONS_INPUT);

  #ifdef DEBUGON
    Serial.println(throttleVal);
    Serial.println(buttonVal);
  #endif
  if ( throttleVal < 10 ) {
    PWM_PIN = 0;
    DDRD &= ~(1<<DDD3);
    TCCR2B &= ~(1<<CS21);
    if (434 > buttonVal && buttonVal > 414) {
      if (currentDir == NEUTRAL) {
        changeDirection(FORWARD);
      }
    } else if ( 555 > buttonVal && buttonVal > 535 ) {
      changeDirection(NEUTRAL); 
    } else if ( 673 > buttonVal && buttonVal > 630 ) {
      if (currentDir == NEUTRAL) {
        changeDirection(REVERSE);
      }
    }
  } else if (currentDir != NEUTRAL) {
    TCCR2B |= (1<<CS21);
    DDRD |= (1<<DDD3);
    PWM_PIN = throttleVal;
  } else {
    TCCR2B &= ~(1<<CS21);
    DDRD &= ~(1<<DDD3);
    PWM_PIN = 0;
  }
  
  if (173 > buttonVal && buttonVal > 153) {
    digitalWrite(HORN_PIN, HIGH);
  } else {
    digitalWrite(HORN_PIN, LOW);
  }
  
  if (307 > buttonVal && buttonVal > 287) {
    if (lightState == true) {
      digitalWrite(LIGHT_PIN, LOW);
      ShiftPWM.SetOne(LIGHT_LED, dimBrightness);
      lightState = false;
      delay(200);
    } else {
      digitalWrite(LIGHT_PIN, HIGH);
      ShiftPWM.SetOne(LIGHT_LED, maxBrightness);
      lightState = true;
      delay(200);
    }
  } 
  delay(10);
}

void changeDirection(int dir) {
  
  digitalWrite(REVERSE_PIN, LOW);
  digitalWrite(FORWARD_PIN, LOW);
  ShiftPWM.SetOne(FORWARD_LED, dimBrightness);
  ShiftPWM.SetOne(NEUTRAL_LED, dimBrightness);
  ShiftPWM.SetOne(REVERSE_LED, dimBrightness);
  
  switch(dir) {
    case FORWARD:
      #ifdef DEBUGON
        Serial.println("Changing direction to foward");
      #endif
      digitalWrite(FORWARD_PIN, HIGH);
      ShiftPWM.SetOne(FORWARD_LED, maxBrightness);
      currentDir = FORWARD;
      break;
    case REVERSE:
      #ifdef DEBUGON
        Serial.println("Changing direction to reverse");
      #endif
      digitalWrite(REVERSE_PIN, HIGH);
      ShiftPWM.SetOne(REVERSE_LED, maxBrightness);
      currentDir = REVERSE;
      break;
    default:
      #ifdef DEBUGON
        Serial.println("Changing direction to neutral");
      #endif
      ShiftPWM.SetOne(NEUTRAL_LED, maxBrightness);
      currentDir = NEUTRAL;
  }
  delay(250);
}




