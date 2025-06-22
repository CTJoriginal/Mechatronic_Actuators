#include <Arduino.h>
#include <AccelStepper.h>
#include <math.h>

// Stepeprs
#define ENABLE 8
int maxSpeed = 70;  // S/s
int speedRampTime = 3000;
AccelStepper M_L(AccelStepper::DRIVER, 2, 5);  // STEP = 5, DIR = 2
AccelStepper M_D(AccelStepper::DRIVER, 3, 6);  // STEP = 6, DIR = 3

// Gumbi
#define G1 9     // X_lim
#define LS_S 10  // Y_lim
#define LS_Z 11  // Z_lim

// Program variables
String serialCommand = "";
unsigned long millisPrev = 0;
unsigned long t = 0;
int speed = 0;
bool SwitchState = false;
bool preveriousG1state = true;


typedef enum {
    STOP_UP,
    UP,
    STOP_DOWN,
    DOWN,
    NONE
} State;

typedef struct
{
    State now = State::STOP_UP;
    State prev = State::STOP_UP;
    State next = State::NONE;
} StateMachine;

StateMachine Window;



int speedRamp(double milis, double transitionTime, double finalSpeed) {
  return (int)round(finalSpeed * (1 - exp(-pow((milis / (transitionTime / 2)), 2.0))));
}

void setup() {
  Serial.begin(9600);

  pinMode(LS_Z, INPUT_PULLUP);  // LOW when swithch is activated
  pinMode(LS_S, INPUT_PULLUP);
  pinMode(G1, INPUT_PULLUP);
  pinMode(ENABLE, OUTPUT);

  M_L.setMaxSpeed(maxSpeed);  // Steps per second
  M_D.setMaxSpeed(maxSpeed);
}

void loop() {
  // Debug
  if (Window.prev != Window.now) {
    Serial.print(">State:"); //works great with Teleplot VS code extension
    Serial.println(Window.now);
    millisPrev = millis();
}
Window.prev = Window.now;

bool G1State = digitalRead(G1);
SwitchState = (preveriousG1state && !G1State); //detect falling edge
preveriousG1state = G1State;

  // GET SERIAL COMMANDS
  if (Serial.available() != 0) {
    serialCommand = Serial.readString();
    serialCommand.toUpperCase();
    serialCommand.trim();
  } else
    serialCommand = "";

  // STATE TRANSITION LOGIC
  Window.next = Window.now;

  switch (Window.now) {
    case State::STOP_UP:
      if (!digitalRead(LS_Z))
        Window.next = State::STOP_DOWN;
      else if (SwitchState || serialCommand == "UP")
        Window.next = State::UP;
      break;

    case State::STOP_DOWN:
      if (!digitalRead(LS_S))
        Window.next = State::STOP_UP;
      else if (SwitchState || serialCommand == "DOWN")
        Window.next = State::DOWN;
      break;


    case State::UP:
      if (SwitchState || !digitalRead(LS_Z) || serialCommand == "STOP")
        Window.next = State::STOP_DOWN;
      else if (serialCommand == "DOWN")
        Window.next = State::DOWN;
      break;

    case State::DOWN:
      if (SwitchState || !digitalRead(LS_S) || serialCommand == "STOP")
        Window.next = State::STOP_UP;
      else if (serialCommand == "UP")
        Window.next = State::UP;
      break;
}

Window.now = Window.next; // This has to be done this way so switch() variable doesnt change
     
  // IO
  switch (Window.now) {
    case State::STOP_UP:
    case State::STOP_DOWN:
      M_L.setSpeed(0);
      M_D.setSpeed(0);

      digitalWrite(ENABLE, HIGH);  // Release stepper control
      break;

    case State::UP:
    case State::DOWN:
      t = millis() - millisPrev;
      speed = speedRamp((double)t, speedRampTime, maxSpeed) + 100;  //Slowly increase speed using S-curve

      digitalWrite(ENABLE, LOW);

      M_L.setSpeed(Window.now == State::UP ? speed : -speed);
      M_D.setSpeed(Window.now == State::UP ? -speed : speed);

      M_L.runSpeed();
      M_D.runSpeed();
      break;
  }

  delay(1);
}
