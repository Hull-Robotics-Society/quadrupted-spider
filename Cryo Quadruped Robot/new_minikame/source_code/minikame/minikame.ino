#include <Arduino.h>
#include "MiniKame.h"

//uncomment out below line if you want to use HC_SR04 sensor
#define __HC_SR04__

#ifdef __HC_SR04__
#define HC_SR04_TRIGGER_PIN 10
#define HC_SR04_ECHO_PIN 11
#define MIN_DISTANCE 10
#define MAX_DISTANCE MIN_DISTANCE + 10
#endif

#define CAL_TRIGGER_PIN 12
#define LED_PIN A0
//#define TIME_INTERVAL 5000
//#define TIME_INTERVAL 2000

#define FORWARD 'f'
#define ROTATE_LEFT 'j'
#define LEFT 'l'
#define STAND 's'
#define RIGHT 'r'
#define ROTATE_RIGHT 'k'
#define BACKWARD 'b'
#define PUSHUP 'g'
#define HELLO 'c'
#define RIGHT_BACK 'e'
#define DANCE 'd'
#define MOONWALK 'h'
#define JUMP 'p'
#define SWIM 'n'
#define PADDLE 'q'

MiniKame robot;

volatile boolean auto_mode = true;
bool state = true;
char cmd = STAND;
boolean random_walk = false;
static int calls = 0;

void setup() {
  Serial.begin(115200);

#ifdef __HC_SR04__
  pinMode(HC_SR04_TRIGGER_PIN, OUTPUT);
  pinMode(HC_SR04_ECHO_PIN, INPUT);
#endif
  randomSeed(analogRead(A7));

  //
  robot.init();
  delay(2000);

  //begin: triggering delay for servo calibrating
  pinMode(CAL_TRIGGER_PIN, OUTPUT);
  digitalWrite(CAL_TRIGGER_PIN, 0);
  pinMode(CAL_TRIGGER_PIN, INPUT);
  while (digitalRead(CAL_TRIGGER_PIN)) {
    analogWrite(LED_PIN, 128 * state); // on calibarting indication LED
    delay(1000);
    state = !state;
  }
  analogWrite(LED_PIN, 0); // off calibarting indication LED
  //end:
}

void loop() {
  if (auto_mode) {
    if (random_walk)
      gaits2(0);
    else
      gaits2(1);
  } else {
    gaits(cmd);
  }
}

void serialEvent() {
  char tmp = -1;
  boolean taken = false;

  while (Serial.available()) {
    state = !state;
    analogWrite(LED_PIN, 128 * state); //flashing LED indicating cmd received
    tmp = Serial.read (); //Serial.println(cmd);
    not_gaits(tmp);
    taken = gaits(tmp);
    if (taken) cmd = tmp;
  }
}

boolean gaits(char cmd) {
  bool taken = true;
  switch (cmd) {
    case PUSHUP:
      robot.walk();
      break;
    case FORWARD:
      robot.walk();
      break;
    case BACKWARD:
      robot.walk(0);
      break;
    case RIGHT:
      robot.turnR(1, 550);
	  robot.walk();
      break;
	case ROTATE_RIGHT:
      robot.turnR(1, 550);
      break;
    case LEFT:
      robot.turnL(1, 550);
	  robot.walk();
      break;
    case ROTATE_LEFT:
      robot.turnL(1, 550);
      break;
    case HELLO:
      robot.turnR(1, 750);
      robot.walk();
      break;
    case RIGHT_BACK:
      robot.turnR(1, 550);
      robot.walk(0);
      break;
    case DANCE:
      robot.turnL(1, 550);
      robot.walk();
      break;
    case MOONWALK:
      robot.turnL(1, 550);
      robot.walk(0);
      break;
    case STAND:
      robot.home();
      break;
	case JUMP:
      robot.jump();
      break;
	case SWIM:
      robot.swim();
      break;
	case PADDLE:
      robot.paddle();
      break;
    default:
      taken = false;
  }
  return taken;
}

void not_gaits(char cmd) {
  static short receivedNumber = 0;
  static bool negative = false;
  static byte channel = 1;

  switch (cmd) {
    case 'L':
      robot.loadTrim();
      break;
    case 'S':
      robot.storeTrim();
      break;
    case  '<':  // fall through to start a new number with prefix x,y char
      receivedNumber = 0;
      channel = 0;
      negative = false;
      break;
    case 'x':
      channel = 1;
      break;
    case 'y':
      channel = 2;
      break;
    case 'z':
      channel = 3;
      break;
    case 'Z':
      channel = 4;
      break;
    case 'a':
      channel = 5;
      break;
    case 'A':
      channel = 6;
      break;
    case '0' ... '9':
      receivedNumber *= 10;
      receivedNumber += cmd - '0';
      break;
    case '-':
      negative = true;
      break;
    case '>': // end of getting a number in format of <[x,y,z,a]number>
      if (negative) receivedNumber = -receivedNumber;
      switch (channel) {
        case 1:
          if (receivedNumber >= 127) {
            state = auto_mode = false;
            analogWrite(LED_PIN, 0);
            robot.home();
          }
          else {
            state = auto_mode = true;
            analogWrite(LED_PIN, 128);
          }
          break;
        case 2:
          if (receivedNumber >= 127)
            random_walk = false;
          else
            random_walk = true;
          break;
        case 3:
          if (auto_mode) break;
          robot.setTrim(FRONT_RIGHT_HIP, receivedNumber);
          robot.home();
          break;
        case 4:
          if (auto_mode) break;
          robot.setTrim(FRONT_LEFT_HIP, receivedNumber);
          robot.home();
          break;
        case 5:
          if (auto_mode) break;
          robot.setTrim(BACK_RIGHT_HIP, receivedNumber);
          robot.home();
          break;
        case 6:
          if (auto_mode) break;
          robot.setTrim(BACK_LEFT_HIP, receivedNumber);
          robot.home();
          break;
      }  // end of inner switch
      break;
  } // end of switch

}

void gaits2(int pattern) {
  char movements[] = {FORWARD, ROTATE_LEFT, LEFT, STAND, ROTATE_RIGHT, RIGHT, BACKWARD, HELLO, RIGHT_BACK, DANCE, MOONWALK, PUSHUP, JUMP, SWIM, PADDLE};
  char movements2[] = {FORWARD, ROTATE_LEFT, FORWARD, ROTATE_RIGHT, FORWARD};
  static unsigned long cur_time = 0, prev_time = 0;
  static char cmd = FORWARD, prev_cmd = -1;
  static int c = 0;
  static unsigned long time_interval = 3000;
  if (cmd != FORWARD) {
	time_interval = 1000;
  }
  cur_time = millis();
  if (cur_time - prev_time >= time_interval) {
    prev_time = cur_time;
    do {
      switch (pattern) {
        case 0: c = (int)random(0, sizeof(movements));
          cmd = movements[c];
          break;
        case 1:  c = c % sizeof(movements2);
          cmd = movements2[c++];
          break;
        default:
          pattern = 0;
      }
    } while (cmd == prev_cmd);
    prev_cmd = cmd; //Serial.println(cmd);
  }
#ifdef __HC_SR04__
  cmd = detect_obstacle(cmd);
#endif
  switch (cmd) {
    case PUSHUP:
      robot.pushUp(2, 3000);
      break;
    case FORWARD:
      robot.run();
      break;
    case BACKWARD:
      robot.run(0, 5);
      break;
    case RIGHT:
      robot.turnR(1, 550);
	  robot.home();
      break;
	case ROTATE_RIGHT:
      robot.turnR(3, 550);
      break;
    case LEFT:
      robot.turnL(1, 550);
	  robot.home();
      break;
    case ROTATE_LEFT:
      robot.turnL(3, 550);
      break;
    case HELLO:
      robot.hello();
      break;
    case DANCE:
      robot.dance(2, 1000);
      break;
    case RIGHT_BACK:
      robot.frontBack(2, 1000);
      break;
    case MOONWALK:
      robot.moonwalkL(2, 1000);
      break;
    case STAND:
      robot.upDown(4, 350);
      break;
	case JUMP:
      robot.jump();
      break;
	case SWIM:
      robot.swim();
      break;
	case PADDLE:
      robot.paddle();
      break;
  }
}


#ifdef __HC_SR04__
char detect_obstacle(char cmd) {
  char movements[] = {STAND, LEFT, RIGHT, HELLO, RIGHT_BACK, DANCE, MOONWALK, PUSHUP, SWIM, PADDLE};
  char i = cmd;
  long cm = distance_measure();
  if (cm < MIN_DISTANCE) {
	 if (calls == 11) {
		calls = 0;
	 } else {
		calls++;
	 }
	 switch (calls) {
	   case 3:
	     i = LEFT;
		 break;
	   case 4:
		 i = RIGHT;
		 break;
	   case 5:
		 i = PUSHUP;
		 break;
	   case 6:
		 i = RIGHT_BACK;
		 break; 
	   case 7:
		 i = DANCE;
		 break; 
	   case 8:
		 i = MOONWALK;
		 break; 
	   case 9:
		 i = SWIM;
		 break; 
	   case 10:
		 i = PADDLE;
		 break;
	   case 11:
		 i = HELLO;
		 break;	
       default:
		 i = STAND;
	 }
  } else if (cm >= MIN_DISTANCE && cm <= MAX_DISTANCE) {
    i = BACKWARD;
  }
  return i;
}

long distance_measure()
{
  // establish variables for duration of the ping,
  // and the distance result in inches and centimeters:
  long duration, cm;

  // The PING))) is triggered by a HIGH pulse of 2 or more microseconds.
  // Give a short LOW pulse beforehand to ensure a clean HIGH pulse:

  digitalWrite(HC_SR04_TRIGGER_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(HC_SR04_TRIGGER_PIN, HIGH);
  delayMicroseconds(5);
  digitalWrite(HC_SR04_TRIGGER_PIN, LOW);

  // The same pin is used to read the signal from the PING))): a HIGH
  // pulse whose duration is the time (in microseconds) from the sending
  // of the ping to the reception of its echo off of an object.
  duration = pulseIn(HC_SR04_ECHO_PIN, HIGH);

  // The speed of sound is 340 m/s or 29 microseconds per centimeter.
  // The ping travels out and back, so to find the distance of the
  // object we take half of the distance travelled.
  cm =  duration / 29 / 2;
  //Serial.print(cm);
  //Serial.println("cm");
  // delay(100);
  return cm;
}
#endif
