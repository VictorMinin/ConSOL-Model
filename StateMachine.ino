// TellCo Europe Mini-ConSol
// Victor Minin
// 5/09/2025

#include "IRremote.h"
// Define IR receiver pin
const int receiverPin = 12; 
#define IR_DELAY 500

// Declare IR objects
IRrecv irrecv(receiverPin); // Custom IR Receiver object from elegoo library
decode_results ir_results;  // Custom Decoder object from elegoo library

// Define motor control pins for the Linear Actuators
const int la1Motor1Pin1    = 2;  // Input 1 for Linear Actuator Set 1, Motor 1
const int la1Motor1Pin2    = 3;  // Input 2 for Linear Actuator Set 1, Motor 1
const int la1Motor2Pin1    = 4;  // Input 1 for Linear Actuator Set 1, Motor 2
const int la1Motor2Pin2    = 5;  // Input 2 for Linear Actuator Set 1, Motor 2
const int la1Enable1Pin    = 9;  // Enable pin for Linear Actuator Set 1, Motor 1
const int la1Enable2Pin    = 10; // Enable pin for Linear Actuator Set 1, Motor 2

// Define motor control pins for the Stepper Motor 
const int sm_IN1           = 6;  // Connect to L298N IN1 for Stepper Motor, Coil A+
const int sm_IN2           = 7;  // Connect to L298N IN2 for Stepper Motor, Coil A-
const int sm_IN3           = 8;  // Connect to L298N IN3 for Stepper Motor, Coil B+
const int sm_IN4           = 11; // Connect to L298N IN4 for Stepper Motor, Coil B-

// Other macros/ globals
#define ON                   1    // Macro infinite while loop 
#define MOTOR_RUN_TIME       5000 // Macro for time to run LA motor 5 seconds
#define MOTOR_SLEEP_TIME     2000 // Macro for time to sleep LA motor 2 seconds
#define LA_MOTOR_SPEED       255  // Macro for max LA motor speed (0-255)
#define BAUD_RATE            9600 // Macro for serial baud rate

// Stepper Motor Specific Macros
#define STEPS_PER_REVOLUTION 200  // Step size may vary due to weight of "solar panels" (1.8 degrees per step * 200 steps = 360 degrees)
int stepper_current_step   = 0;   // Current step number (0-3 for the sequence)
#define STEPPER_DELAY        50   // Delay between steps in milliseconds (controls speed, smaller = faster)

// Stepping sequence for two-phase full step 
const int step_sequence[4][4] = {
  {HIGH, LOW, HIGH, LOW},   // Step 1: Coil A=HIGH/LOW, Coil B=HIGH/LOW
  {LOW, HIGH, HIGH, LOW},   // Step 2: Coil A=LOW/HIGH, Coil B=HIGH/LOW
  {LOW, HIGH, LOW, HIGH},   // Step 3: Coil A=LOW/HIGH, Coil B=LOW/HIGH
  {HIGH, LOW, LOW, HIGH}    // Step 4: Coil A=HIGH/LOW, Coil B=LOW/HIGH
};

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
  // Set all the linear actuator motor control pins as outputs
  pinMode(la1Motor1Pin1, OUTPUT);
  pinMode(la1Motor1Pin2, OUTPUT);
  pinMode(la1Motor2Pin1, OUTPUT);
  pinMode(la1Motor2Pin2, OUTPUT);
  pinMode(la1Enable1Pin, OUTPUT);
  pinMode(la1Enable2Pin, OUTPUT);

  // Start with linear actuators stopped
  digitalWrite(la1Motor1Pin1, LOW);
  digitalWrite(la1Motor1Pin2, LOW);
  digitalWrite(la1Motor2Pin1, LOW);
  digitalWrite(la1Motor2Pin2, LOW);
  digitalWrite(la1Enable1Pin, LOW);
  digitalWrite(la1Enable2Pin, LOW);

  // Set all stepper motor control pins as outputs
  pinMode(sm_IN1, OUTPUT);
  pinMode(sm_IN2, OUTPUT);
  pinMode(sm_IN3, OUTPUT);
  pinMode(sm_IN4, OUTPUT);

  // Initialize serial communication for debugging
  Serial.begin(BAUD_RATE);

  // Start the IR receiver
  irrecv.enableIRIn(); 
}

// Function to output one step to the stepper motor
void applyStep(int step) { // step is 0, 1, 2, or 3
  digitalWrite(sm_IN1, step_sequence[step][0]);
  digitalWrite(sm_IN2, step_sequence[step][1]);
  digitalWrite(sm_IN3, step_sequence[step][2]);
  digitalWrite(sm_IN4, step_sequence[step][3]);
}

// Function to step Stepper Motor clockwise
void stepClockwise() {
  stepper_current_step++;
  if (stepper_current_step >= 4) { // 4 steps in the sequence
    stepper_current_step = 0;
  }
  applyStep(stepper_current_step);
  delay(STEPPER_DELAY); // Delay to control speed. This Macro may have to changed/ optimized for smoothness and speed.
}

// Function to step Stepper Motor counter-clockwise
void stepCounterClockwise() {
  stepper_current_step--;
  if (stepper_current_step < 0) {
    stepper_current_step = 3; // Loop back to the end of the sequence
  }
  applyStep(stepper_current_step);
  delay(STEPPER_DELAY); // Delay to control speed. This Macro may have to changed/ optimized for smoothness and speed.
}

void loop() {
  switch(currentState) {
    case WAITING_FOR_COMMAND:
      if (irrecv.decode(&ir_results)) {
        translateIR()
      }
      break;
    
    case LIFTING_CONTAINER:
      if (panelsState == PANELS_EXTENDED) {
        retractPanels()
      }
      extendActuators();
      stopActuators();
      currentState = WAITING_FOR_COMMAND;
      break;
    
    case LOWERING_CONTAINER:
      if (panelsState == PANELS_EXTENDED) {
        retractPanels();
      }
      retractActuators();
      stopActuators();
      currentState = WAITING_FOR_COMMAND;
      break;

    case EXTENDING_PANELS:
      extendPanels();
      currentState = WAITING_FOR_COMMAND;
      break;
    
    case TILTING_PANELS_DOWN:
      // tilt_panels_down();
      currentState = WAITING_FOR_COMMAND;
      break;

    case TILTING_PANELS_UP:
      // tilt_panels_up();
      currentState = WAITING_FOR_COMMAND;
      break;

    case RETRACTING_PANELS:
      retractPanels();
      currentState = WAITING_FOR_COMMAND;
      break;
  } 
}

void extendPanels() { // This function may have to be swapped to retractPanels()
  Serial.println("Stepping Stepper counter-clockwise for 1 revolution");
  for (int i = 0; i < STEPS_PER_REVOLUTION; i++) {
    stepCounterClockwise();
  }
  Serial.print("Stepper finished 1 revolution counter-clockwise. Holding position at step: ");
  Serial.println(stepper_current_step);
  delay(MOTOR_SLEEP_TIME); // Wait for 2 seconds

}

void retractPanels() { // This function may have to be swapped to extendPanels()
  Serial.println("Stepping Stepper clockwise for 1 revolution");
  for (int i = 0; i < STEPS_PER_REVOLUTION; i++) {
    stepClockwise();
  }
  Serial.print("Stepper finished 1 revolution clockwise. Holding position at step: ");
  Serial.println(stepper_current_step);
  delay(MOTOR_SLEEP_TIME); // Wait for 2 seconds
}

// Receives IR code and sets current command global var
void translateIR() { 
// I shall assume that only buttons 1-6 are to be used others are left here for your reference
  switch(ir_results.value) {
    case 0xFF6897: 
      Serial.println("0");    
      break;
    case 0xFF30CF: 
      Serial.println("1");
      currentState = LIFTING_CONTAINER;    
      break;
    case 0xFF18E7: 
      Serial.println("2");
      currentState = LOWERING_CONTAINER;    
      break;
    case 0xFF7A85: 
      Serial.println("3");
      currentState = EXTENDING_PANELS;
      break;
    case 0xFF10EF: 
      Serial.println("4");    
      currentState = TILTING_PANELS_DOWN;
      break;
    case 0xFF38C7: 
      Serial.println("5");
      currentState = TILTING_PANELS_UP;
      break;
    case 0xFF5AA5:
      Serial.println("6");
      currentState = RETRACTING_PANELS;
      break;
    case 0xFF42BD:
      Serial.println("7");
      break;
    case 0xFF4AB5:
      Serial.println("8");
      break;
    case 0xFF52AD:
      Serial.println("9");
      break;
    case 0xFFFFFFFF:
      Serial.println(" REPEAT");
      break; 

    default:
      Serial.println("Other button pushed"); // This is for debugging (All print statements are), not needed 
  }
  delay(IR_DELAY); // Delay to avoid immediate repeat
}

// Function to extend both actuators 
void extendActuators() {
  // Set speed (0-255) adjust as needed
  analogWrite(la1Enable1Pin, LA_MOTOR_SPEED);
  analogWrite(la1Enable2Pin, LA_MOTOR_SPEED);

  // Set direction for extension
  digitalWrite(la1Motor1Pin1, HIGH);
  digitalWrite(la1Motor1Pin2, LOW);
  digitalWrite(la1Motor2Pin1, HIGH);
  digitalWrite(la1Motor2Pin2, LOW);
}

// Function to retract both actuators 
void retractActuators() {
  // Set speed (0-255) adjust as needed
  analogWrite(la1Enable1Pin, LA_MOTOR_SPEED);
  analogWrite(la1Enable2Pin, LA_MOTOR_SPEED);

  // Set direction for retraction
  digitalWrite(la1Motor1Pin1, LOW);
  digitalWrite(la1Motor1Pin2, HIGH);
  digitalWrite(la1Motor2Pin1, LOW);
  digitalWrite(la1Motor2Pin2, HIGH);
}

// Function to stop both actuators
void stopActuators() {
  digitalWrite(la1Motor1Pin1, LOW);
  digitalWrite(la1Motor1Pin2, LOW);
  digitalWrite(la1Motor2Pin1, LOW);
  digitalWrite(la1Motor2Pin2, LOW);
  digitalWrite(la1Enable1Pin, LOW); 
  digitalWrite(la1Enable2Pin, LOW);
}
