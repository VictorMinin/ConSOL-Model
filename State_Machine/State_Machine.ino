#include <IRremote.h>
#include <TMCStepper.h>
#include <SoftwareSerial.h>

const int IR_RECEIVE_PIN = 12;

// --- Actuator Definitions ---
#define LA_MOTOR_SPEED 255  // Full speed

struct Actuator {
  int RPWM;
  int LPWM;
  int EN;
};

Actuator actuator[6] = {
// Assign pins in order of RPWM, LPWM, and EN
// Pins 2-12 and 44-46 are PWM Pins
  {22, 23, 24},    // Actuator set 1 lift
  {25, 26, 27},    // Actuator set 2 jack extension

  {28, 29, 30},    // Actuator 3 lift
  {31, 32, 33},    // Actuator 4 jack extension

  {2, 3, 4},       // Actuator 5 tilt
  {5, 6, 7}        // Actuator 6 tilt
};

// Define IR receiver pin
const int receiverPin = 12; 
#define IR_DELAY 500

// Declare IR objects
IRrecv irrecv(receiverPin); // Custom IR Receiver object from elegoo library
decode_results ir_results;  // Custom Decoder object from elegoo library

// Define motor control pins for the Stepper Motor 
#define EN_PIN_1           11   // Enable
#define DIR_PIN_1          10   // Direction
#define STEP_PIN_1         9   // Step

#define EN_PIN_2           44   // Enable
#define DIR_PIN_2          43   // Direction
#define STEP_PIN_2         42   // Step

#define MOTOR1_STEPS_PER_REV 400 // Experimentally determined - use to keep motors in sync. 
#define MOTOR2_STEPS_PER_REV 400 // Experimentally determined - use to keep motors in sync. 
// "Steps per 1 revolution," if one motor moves 2x the speed of the other and completes 2 revolutions in the time it takes  the  other to make one, then that motor has 200 steps/rev where the other has 400 steps/rev

#define MICROSTEPS 16

#define R_SENSE 0.11f  // Sense resistor used

TMC2209Stepper driver1(&Serial1, R_SENSE, 0); // Driver 1 communicates through TX18 and RX19
TMC2209Stepper driver2(&Serial2, R_SENSE, 0); // Driver 2 communicates through TX16 and RX17 (Arudino MEGA default)

void extendActuator(int id) {
  digitalWrite(actuator[id].EN, HIGH);
  digitalWrite(actuator[id].RPWM, LA_MOTOR_SPEED);
  digitalWrite(actuator[id].LPWM, 0);
}

void retractActuator(int id) {
  digitalWrite(actuator[id].EN, HIGH);
  digitalWrite(actuator[id].RPWM, 0);
  digitalWrite(actuator[id].LPWM, LA_MOTOR_SPEED);
}

void stopActuator(int id) {
  digitalWrite(actuator[id].RPWM, 0);
  digitalWrite(actuator[id].LPWM, 0);
  digitalWrite(actuator[id].EN, LOW);
}

// Select actuator 9 as the actuator in the back.
void tilt_front_panel_down() {
  while (analogRead(A0) < 900) {
  analogWrite(actuator[4].RPWM, 0);
  analogWrite(actuator[4].LPWM, 100);
  digitalWrite(actuator[4].EN, HIGH);
  }
  stopActuator(4);
}

void tilt_back_panel_down() {
  while (analogRead(A1) < 900) {
  analogWrite(actuator[5].RPWM, 0);
  analogWrite(actuator[5].LPWM, 100);
  digitalWrite(actuator[5].EN, HIGH);
  }
  stopActuator(5);
}

void tilt_front_panel_up() {
  while (analogRead(A0) > 400) {
  analogWrite(actuator[4].RPWM, 100);
  analogWrite(actuator[4].LPWM, 0);
  digitalWrite(actuator[4].EN, HIGH);
  }
  stopActuator(4);
}

void tilt_back_panel_up() {
  while (analogRead(A1) > 400) {
  analogWrite(actuator[5].RPWM, 100);
  analogWrite(actuator[5].LPWM, 0);
  digitalWrite(actuator[5].EN, HIGH);
  }
  stopActuator(5);
}

void moveBothMotors(float revs, bool dir1, bool dir2) {
  digitalWrite(DIR_PIN_1, dir1);
  digitalWrite(DIR_PIN_2, dir2);

  // Compute total steps (full steps × microsteps)
  long steps_motor1 = (long)(MOTOR1_STEPS_PER_REV * MICROSTEPS * revs);
  long steps_motor2 = (long)(MOTOR2_STEPS_PER_REV * MICROSTEPS * revs);

  long maxSteps = (steps_motor1 > steps_motor2) ? steps_motor1 : steps_motor2;
  long minSteps = (steps_motor1 < steps_motor2) ? steps_motor1 : steps_motor2;

  // Print for debugging
  Serial.print("Motor 1 steps: "); Serial.println(steps_motor1);
  Serial.print("Motor 2 steps: "); Serial.println(steps_motor2);

  // Initialize counters
  long counter1 = 0;
  long counter2 = 0;

  // Bresenham error terms
  long error1 = 0;
  long error2 = 0;

  for (long i = 0; i < maxSteps; i++) {
    bool step1 = false;
    bool step2 = false;

    // Decide if Motor 1 steps
    error1 += steps_motor1;
    if (error1 >= maxSteps) {
      error1 -= maxSteps;
      step1 = true;
      digitalWrite(STEP_PIN_1, HIGH);
    }

    // Decide if Motor 2 steps
    error2 += steps_motor2;
    if (error2 >= maxSteps) {
      error2 -= maxSteps;
      step2 = true;
      digitalWrite(STEP_PIN_2, HIGH);
    }

    delayMicroseconds(500);

    if (step1) digitalWrite(STEP_PIN_1, LOW);
    if (step2) digitalWrite(STEP_PIN_2, LOW);

    delayMicroseconds(500);
  }
}

// STATE MACHINE DEFINITIONS
enum SystemState {
  WAITING_FOR_COMMAND,
  LIFTING_CONTAINER,
  LOWERING_CONTAINER,
  EXTENDING_PANELS,
  TILTING_PANELS_DOWN,
  TILTING_PANELS_UP,
  RETRACTING_PANELS
};

SystemState currentState = WAITING_FOR_COMMAND;

// Track last actions to prevent double commands and invalid commands
enum ActuatorLastAction {
  ACT_NONE,
  ACT_EXTENDED,
  ACT_RETRACTED
};
ActuatorLastAction lastActuatorAction = ACT_NONE;

enum PanelsState {
  PANELS_RETRACTED,
  PANELS_EXTENDED,
  PANELS_FLAT,
  PANELS_TILTED_DOWN,
  PANELS_TILTED_UP
};
PanelsState panelsState = PANELS_RETRACTED;

void setup() {
  
for (int i = 0; i < 5; i++) {
  pinMode(actuator[i].RPWM, OUTPUT);
  pinMode(actuator[i].LPWM, OUTPUT);
  pinMode(actuator[i].EN, OUTPUT);
}

  pinMode(EN_PIN_1, OUTPUT);
  pinMode(STEP_PIN_1, OUTPUT);
  pinMode(DIR_PIN_1, OUTPUT);
  digitalWrite(EN_PIN_1, LOW); // Enable driver

  pinMode(EN_PIN_2, OUTPUT);
  pinMode(STEP_PIN_2, OUTPUT);
  pinMode(DIR_PIN_2, OUTPUT);
  digitalWrite(EN_PIN_2, LOW); // Enable driver


  Serial.begin(115200);
  Serial1.begin(115200);    // For driver 1
  Serial2.begin(115200);    // For driver 2
  delay(500);

  driver1.begin();               // SPI: Init
  driver1.toff(5);               // Enables driver in software
  driver1.rms_current(600);      // Set motor RMS current (600mA is safe-ish)
  driver1.microsteps(MICROSTEPS);// 1/8 microstepping NOTE: My motors had different steps/rev which is why I had to set different microsteps. Troubleshooting on your end may vary. Goal is to keep the motors running in sync.
  driver1.en_spreadCycle(false); // Use stealthChop for silent operation
  driver1.pwm_autoscale(true);   // Needed for stealthChop

  driver2.begin();               // SPI: Init
  driver2.toff(5);               // Enables driver in software
  driver2.rms_current(600);      // Set motor RMS current (600mA is safe-ish)
  driver2.microsteps(MICROSTEPS);// 1/16 microstepping
  driver2.en_spreadCycle(false); // Use stealthChop for silent operation
  driver2.pwm_autoscale(true);   // Needed for stealthChop

  // Start the IR receiver
  IrReceiver.begin(receiverPin);

  // Set actuator in desired position
  while (analogRead(A0) > 850) {
  analogWrite(actuator[4].RPWM, 100);
  analogWrite(actuator[4].LPWM, 0);
  digitalWrite(actuator[4].EN, HIGH);
  }
  stopActuator(4);

  while (analogRead(A0) < 850) {
  analogWrite(actuator[4].RPWM, 0);
  analogWrite(actuator[4].LPWM, 100);
  digitalWrite(actuator[4].EN, HIGH);
  }
  stopActuator(4);

  // Set actuator in desired position
  while (analogRead(A1) > 850) {
  analogWrite(actuator[5].RPWM, 100);
  analogWrite(actuator[5].LPWM, 0);
  digitalWrite(actuator[5].EN, HIGH);
  }
  stopActuator(5);

  while (analogRead(A1) < 850) {
  analogWrite(actuator[5].RPWM, 0);
  analogWrite(actuator[5].LPWM, 100);
  digitalWrite(actuator[5].EN, HIGH);
  }
  stopActuator(5);
}



void loop() {
  switch(currentState) {
    case WAITING_FOR_COMMAND:
      if (IrReceiver.decode()) {
        translateIR();
      }
      break;
    
    case LIFTING_CONTAINER:
      if (panelsState == PANELS_EXTENDED) {
        retractActuator(0);
        retractActuator(2);
        delay(8000);
        retractActuator(1);
        retractActuator(3);

      }

      extendActuator(1);
      extendActuator(3);
      delay(3000);
      extendActuator(0);
      extendActuator(2);
      delay(8000);

      stopActuator(0);
      stopActuator(1);      
      stopActuator(2);
      stopActuator(3);
      currentState = WAITING_FOR_COMMAND;
      break;
    
    case LOWERING_CONTAINER:
      if (panelsState == PANELS_EXTENDED) {
        retractActuator(0);
        retractActuator(2);
        delay(8000);
        retractActuator(1);
        retractActuator(3);

      }
      
      retractActuator(0);
      retractActuator(2);
      delay(8000);
      retractActuator(1);
      retractActuator(3);
      delay(3000);

      stopActuator(0);
      stopActuator(2);      
      stopActuator(1);
      stopActuator(3);

      currentState = WAITING_FOR_COMMAND;
      break;

    case EXTENDING_PANELS:

      while (analogRead(A0) > 500) {
      analogWrite(actuator[4].RPWM, 50);
      analogWrite(actuator[4].LPWM, 0);
      digitalWrite(actuator[4].EN, HIGH);
      }
      stopActuator(4);

      while (analogRead(A0) < 500) {
      analogWrite(actuator[4].RPWM, 0);
      analogWrite(actuator[4].LPWM, 50);
      digitalWrite(actuator[4].EN, HIGH);
      }
      stopActuator(4);

      extendPanels();

      currentState = WAITING_FOR_COMMAND;
      break;
    
    case TILTING_PANELS_DOWN:
      tilt_front_panel_down();
      //tilt_back_panel_down();
      currentState = WAITING_FOR_COMMAND;
      break;

    case TILTING_PANELS_UP:
      tilt_front_panel_up();
      //tilt_back_panel_up();
      currentState = WAITING_FOR_COMMAND;
      break;

    case RETRACTING_PANELS:
      while (analogRead(A0) > 500) {
      analogWrite(actuator[4].RPWM, 50);
      analogWrite(actuator[4].LPWM, 0);
      digitalWrite(actuator[4].EN, HIGH);
      }
      stopActuator(4);

      while (analogRead(A0) < 500) {
      analogWrite(actuator[4].RPWM, 0);
      analogWrite(actuator[4].LPWM, 50);
      digitalWrite(actuator[4].EN, HIGH);
      }
      stopActuator(4);

      retractPanels();

      currentState = WAITING_FOR_COMMAND;
      break;
  } 
}

void extendPanels() {
  Serial.println("Extending Panels 3 steps");

  moveBothMotors(3, HIGH, LOW);
  delay(2000);
}

void retractPanels() {
  Serial.println("Retracting Panels 3 steps");

  moveBothMotors(3, LOW, HIGH);
  delay(2000);
}

// Receives IR code and sets current command global var

void translateIR() {
  Serial.print("IR Code Received: 0x");
  Serial.println(IrReceiver.decodedIRData.decodedRawData, HEX);  

  switch (IrReceiver.decodedIRData.decodedRawData) {
    case 0xE916FF00:
      Serial.println("Button 0 pressed");
      currentState = LIFTING_CONTAINER;
      break;
    case 0xF30CFF00:
      Serial.println("Button 1 pressed");
      currentState = LOWERING_CONTAINER;
      break;
    case 0xE718FF00:
      Serial.println("Button 2 pressed");
      currentState = EXTENDING_PANELS;
      break;
    case 0xA15EFF00:
      Serial.println("Button 3 pressed");
      currentState = TILTING_PANELS_DOWN;
      break;
    case 0xF708FF00:
      Serial.println("Button 4 pressed");
      currentState = TILTING_PANELS_UP;
      break;
    case 0xE31CFF00:
      Serial.println("Button 5 pressed");
      currentState = RETRACTING_PANELS;
      break;
  }

// Print actual IR hex code
  delay(IR_DELAY);  // Debounce
  IrReceiver.resume();  // Continue receiving
}


/*
void loop() {

  extendActuator(1);
  extendActuator(3);
  delay(3000);
  extendActuator(0);
  extendActuator(2);
  delay(7000);

  delay(2000);
  stopActuator(0);
  stopActuator(1);      
  stopActuator(2);
  stopActuator(3);

  retractActuator(0);
  retractActuator(2);
  delay(7000);
  retractActuator(1);
  retractActuator(3);
  delay(3000);

  delay(2000);
  stopActuator(0);
  stopActuator(1);      
  stopActuator(2);
  stopActuator(3);

  extendPanels();
  delay(2000);
  retractPanels();
  delay(2000);
}
*/
