//#define DEBUGON;

#define DIRECTION_BUTTON 13
#define LIGHT_BUTTON 12
#define HORN_BUTTON 11
#define FORWARD_LED 10
#define NEUTRAL_LED 9
#define REVERSE_LED 8
#define FORWARD_PIN 7
#define REVERSE_PIN 6
#define LIGHT_PIN 5
#define HORN_PIN 4
#define PWM_PIN 3
#define THROTTLE_INPUT A0

int currentDir = 2; //1 = Forward, 2 = Neutral, 3 = Reverse;
int lastDir = 3; //We want this to think the last direction was reverse, to that we bump into forward when we hit the button
int throttleVal;
boolean lightState = false;


void setup() {

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
  pinMode(PWM_PIN, OUTPUT);
  
  //Let's give the user some indication that we're booting up
  
  digitalWrite(FORWARD_LED, HIGH);
  digitalWrite(NEUTRAL_LED, HIGH);
  digitalWrite(REVERSE_LED, HIGH);
  
  //Let's set everything else to off
  
  digitalWrite(FORWARD_PIN, LOW);
  digitalWrite(REVERSE_PIN, LOW);
  digitalWrite(LIGHT_PIN, LOW);
  digitalWrite(HORN_PIN, LOW);
  analogWrite(PWM_PIN, 0);
  delay(1000);
  changeDirection(2);
  #ifdef DEBUGON 
    Serial.begin(9600);
  #endif
}

void loop() {
  throttleVal = analogRead(THROTTLE_INPUT) / 4;
  #ifdef DEBUGON
    Serial.println(throttleVal);
  #endif
  if ( throttleVal < 10 ) {
    analogWrite(PWM_PIN, 0);
    
    if (digitalRead(DIRECTION_BUTTON) == LOW) {
      if (lastDir == 3 && currentDir == 2) {
        changeDirection(1); 
        currentDir = 1;
        lastDir = 1;
      } else if (lastDir == 1 && currentDir == 2) {
        changeDirection(3);
        currentDir = 3;
        lastDir = 3;
      } else {
        changeDirection(2);
        currentDir = 2;
      }
    }
  } else {
    analogWrite(PWM_PIN, throttleVal);
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
      delay(100);
    } else {
      digitalWrite(LIGHT_PIN, HIGH);
      lightState = true;
      delay(100);
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
    case 1:
      digitalWrite(FORWARD_PIN, HIGH);
      digitalWrite(FORWARD_LED, HIGH);
      break;
    case 3:
      digitalWrite(REVERSE_PIN, HIGH);
      digitalWrite(REVERSE_LED, HIGH);
      break;
    default:
      digitalWrite(NEUTRAL_LED, HIGH);
  }
  delay(250);
}




