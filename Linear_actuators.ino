// Define motor control pins for the L298N H-bridge and other macro/ globals
const int motor1Pin1       = 2;    // Input 1 for Motor 1
const int motor1Pin2       = 3;    // Input 2 for Motor 1
const int motor2Pin1       = 4;    // Input 1 for Motor 2
const int motor2Pin2       = 5;    // Input 2 for Motor 2
const int enable1Pin       = 9;    // Enable pin for Motor 1
const int enable2Pin       = 10;   // Enable pin for Motor 2
const int ON               = 1;    // Macro infinite while loop
const int MOTOR_RUN_TIME   = 5000; // Macro for time to run motor 5 seconds
const int MOTOR_SLEEP_TIME = 2000; // Macro for time to sleep motor 2 seconds
const int MOTOR_SPEED      = 255;  // Macro for max motor speed (0-255)
const int BAUD_RATE        = 9600; // Macro for serial baud rate

void setup() {
  // Set all the motor control pins as outputs
  pinMode(motor1Pin1, OUTPUT);
  pinMode(motor1Pin2, OUTPUT);
  pinMode(motor2Pin1, OUTPUT);
  pinMode(motor2Pin2, OUTPUT);
  pinMode(enable1Pin, OUTPUT);
  pinMode(enable2Pin, OUTPUT);

  // Start with motors stopped
  digitalWrite(motor1Pin1, LOW);
  digitalWrite(motor1Pin2, LOW);
  digitalWrite(motor2Pin1, LOW);
  digitalWrite(motor2Pin2, LOW);
  digitalWrite(enable1Pin, LOW);
  digitalWrite(enable2Pin, LOW);

  // Initialize serial communication for debugging
  Serial.begin(BAUD_RATE);
}

void loop() {
  while (ON) {
    // Extend both actuators
    Serial.println("Extending actuators...");
    extendActuators();
    delay(MOTOR_RUN_TIME); // Wait for 5 seconds (adjust as needed)

    // Stop actuators
    Serial.println("Stopping actuators...");
    stopActuators();
    delay(MOTOR_SLEEP_TIME); // Wait for 2 seconds

    // Retract both actuators
    Serial.println("Retracting actuators...");
    retractActuators();
    delay(MOTOR_RUN_TIME); // Wait for 5 seconds (adjust as needed)

    // Stop actuators
    Serial.println("Stopping actuators...");
    stopActuators();
    delay(MOTOR_SLEEP_TIME); // Wait for 2 seconds before repeating
  }
}

// Function to extend both actuators
void extendActuators() {
  // Set speed (0-255) adjust as needed
  analogWrite(enable1Pin, MOTOR_SPEED);
  analogWrite(enable2Pin, MOTOR_SPEED);

  // Set direction for extension 
  digitalWrite(motor1Pin1, HIGH);
  digitalWrite(motor1Pin2, LOW);
  digitalWrite(motor2Pin1, HIGH);
  digitalWrite(motor2Pin2, LOW);
}

// Function to retract both actuators
void retractActuators() {
  // Set speed (0-255) adjust as needed
  analogWrite(enable1Pin, MOTOR_SPEED);
  analogWrite(enable2Pin, MOTOR_SPEED);

  // Set direction for retraction
  digitalWrite(motor1Pin1, LOW);
  digitalWrite(motor1Pin2, HIGH);
  digitalWrite(motor2Pin1, LOW);
  digitalWrite(motor2Pin2, HIGH);
}

// Function to stop both actuators
void stopActuators() {
  digitalWrite(motor1Pin1, LOW);
  digitalWrite(motor1Pin2, LOW);
  digitalWrite(motor2Pin1, LOW);
  digitalWrite(motor2Pin2, LOW);
  digitalWrite(enable1Pin, LOW);
  digitalWrite(enable2Pin, LOW);
}