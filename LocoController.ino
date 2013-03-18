//#define DEBUGON;

#define DIRECTION_BUTTON 13
#define LIGHT_BUTTON 12
#define HORN_BUTTON 11

#define FORWARD_LED 10
#define NEUTRAL_LED 9
#define REVERSE_LED 8

#define FORWARD_PIN 7
#define REVERSE_PIN 6

#define LIGHT_PIN 3
#define HORN_PIN 4

#define PWM_PIN OCR3A // Digital Pin 5
#define THROTTLE_INPUT A0

#define FORWARD 1
#define NEUTRAL 2
#define REVERSE 3

int currentDir = NEUTRAL; //1 = Forward, 2 = Neutral, 3 = Reverse;
int lastDir = REVERSE; //We want this to think the last direction was reverse, to that we bump into forward when we hit the button
uint8_t throttleVal;
boolean lightState = false;


void setup() {

  //Let's change the prescaler on Timer1 so that we're FastPWM at 7812.5 Hz instead of a measly 900ish
  TCCR3A = 0x01;
  TCCR3B = 0x0A;
  
  //Now lets set PORTC6 for output in the DDR6 register
  DDRC = (1<<DDC6);
  
  //Lets setup the input pins
  pinMode(DIRECTION_BUTTON, INPUT);
  pinMode(LIGHT_BUTTON, INPUT);
  pinMode(HORN_BUTTON, INPUT);
  pinMode(THROTTLE_INPUT, INPUT);
  
  //Now let's tie the buttons to high
  digitalWrite(DIRECTION_BUTTON, HIGH);
  digitalWrite(LIGHT_BUTTON, HIGH);
  digitalWrite(HORN_BUTTON, HIGH);
  
  //Now let's setup the output pins
  pinMode(FORWARD_LED, OUTPUT);
  pinMode(NEUTRAL_LED, OUTPUT);
  pinMode(REVERSE_LED, OUTPUT);
  pinMode(FORWARD_PIN, OUTPUT);
  pinMode(REVERSE_PIN, OUTPUT);
  pinMode(LIGHT_PIN, OUTPUT);
  pinMode(HORN_PIN, OUTPUT);
//  pinMode(PWM_PIN, OUTPUT);
  
  //Let's give the user some indication that we're booting up
  
  digitalWrite(FORWARD_LED, HIGH);
  digitalWrite(NEUTRAL_LED, HIGH);
  digitalWrite(REVERSE_LED, HIGH);
  
  //Let's set everything else to off
  
  digitalWrite(FORWARD_PIN, LOW);
  digitalWrite(REVERSE_PIN, LOW);
  digitalWrite(LIGHT_PIN, LOW);
  digitalWrite(HORN_PIN, LOW);
  PWM_PIN = 0;
  delay(1000);
  changeDirection(2);
  #ifdef DEBUGON 
    Serial.begin(9600);
  #endif
}

void loop() {
  throttleVal = analogRead(THROTTLE_INPUT) / 4; //analogRead provides 0-1024, we want this to be a 0-255 value
  #ifdef DEBUGON
    Serial.println(throttleVal);
  #endif
  if ( throttleVal < 10 ) {
    PWM_PIN = 0;
    
    if (digitalRead(DIRECTION_BUTTON) == LOW) {
      if (lastDir == REVERSE && currentDir == NEUTRAL) {
        changeDirection(FORWARD); 
      } else if (lastDir == FORWARD && currentDir == NEUTRAL) {
        changeDirection(REVERSE);
      } else {
        changeDirection(NEUTRAL);
      }
    }
  } else {
    PWM_PIN = throttleVal;
  }
  
  if (digitalRead(HORN_BUTTON) == LOW) {
    digitalWrite(HORN_PIN, HIGH);
  } else {
    digitalWrite(HORN_PIN, LOW);
  }
  
  if (digitalRead(LIGHT_BUTTON) == LOW) {
    if (lightState == true) {
      digitalWrite(LIGHT_PIN, LOW);
      lightState = false;
      delay(200);
    } else {
      digitalWrite(LIGHT_PIN, HIGH);
      lightState = true;
      delay(200);
    }
  } 
  delay(10);
}

void changeDirection(int dir) {
  
  digitalWrite(REVERSE_PIN, LOW);
  digitalWrite(FORWARD_PIN, LOW);
  digitalWrite(FORWARD_LED, LOW);
  digitalWrite(NEUTRAL_LED, LOW);
  digitalWrite(REVERSE_LED, LOW);
  
  switch(dir) {
    case FORWARD:
      digitalWrite(FORWARD_PIN, HIGH);
      digitalWrite(FORWARD_LED, HIGH);
      currentDir = FORWARD;
      lastDir = FORWARD;
      break;
    case REVERSE:
      digitalWrite(REVERSE_PIN, HIGH);
      digitalWrite(REVERSE_LED, HIGH);
      currentDir = REVERSE;
      lastDir = REVERSE;
      break;
    default:
      digitalWrite(NEUTRAL_LED, HIGH);
      currentDir = NEUTRAL;
  }
  delay(250);
}




