

const int MOTOR_RUN_TIME   = 5000;
const int MOTOR_SLEEP_TIME = 2000;
const int MOTOR_SPEED      = 255; // PWM 0–255
const int BAUD_RATE        = 9600;

void setup() {
  // Set all control pins as output
  pinMode(motor1RPWM, OUTPUT);
  pinMode(motor1LPWM, OUTPUT);
  pinMode(motor2RPWM, OUTPUT);
  pinMode(motor2LPWM, OUTPUT);
  pinMode(motor1RENA, OUTPUT);
  pinMode(motor1LENA, OUTPUT);
  pinMode(motor2RENA, OUTPUT);
  pinMode(motor2LENA, OUTPUT);

  // Enable both BTS7960 bridges
  digitalWrite(motor1RENA, HIGH);
  digitalWrite(motor1LENA, HIGH);
  digitalWrite(motor2RENA, HIGH);
  digitalWrite(motor2LENA, HIGH);

  // Start serial
  Serial.begin(BAUD_RATE);
}

void loop() {
  // Extend actuators
  Serial.println("Extending actuators...");
  extendActuators();
  delay(MOTOR_RUN_TIME);

  Serial.println("Stopping actuators...");
  stopActuators();
  delay(MOTOR_SLEEP_TIME);

  // Retract actuators
  Serial.println("Retracting actuators...");
  retractActuators();
  delay(MOTOR_RUN_TIME);

  Serial.println("Stopping actuators...");
  stopActuators();
  delay(MOTOR_SLEEP_TIME);
}

void extendActuators() {
  // Move both actuators forward
  analogWrite(motor1RPWM, MOTOR_SPEED);
  analogWrite(motor1LPWM, 0);
  analogWrite(motor2RPWM, MOTOR_SPEED);
  analogWrite(motor2LPWM, 0);
}

void retractActuators() {
  // Move both actuators in reverse
  analogWrite(motor1RPWM, 0);
  analogWrite(motor1LPWM, MOTOR_SPEED);
  analogWrite(motor2RPWM, 0);
  analogWrite(motor2LPWM, MOTOR_SPEED);
}

void stopActuators() {
  analogWrite(motor1RPWM, 0);
  analogWrite(motor1LPWM, 0);
  analogWrite(motor2RPWM, 0);
  analogWrite(motor2LPWM, 0);
}
