#include <TMCStepper.h>

#define EN_PIN_1           11   // Enable
#define DIR_PIN_1          10   // Direction
#define STEP_PIN_1         9   // Step

#define EN_PIN_2           44   // Enable
#define DIR_PIN_2          43   // Direction
#define STEP_PIN_2         42   // Step

#define MOTOR1_STEPS_PER_REV 200 // Experimentally determined - use to keep motors in sync. 
#define MOTOR2_STEPS_PER_REV 400 // Experimentally determined - use to keep motors in sync. 
// "Steps per 1 revolution," if one motor moves 2x the speed of the other and completes 2 revolutions in the time it takes  the  other to make one, then that motor has 200 steps/rev where the other has 400 steps/rev

#define MICROSTEPS 16

#define R_SENSE 0.11f  // Sense resistor used

TMC2209Stepper driver1(&Serial1, R_SENSE, 0); // Driver 1 communicates through TX18 and RX19
TMC2209Stepper driver2(&Serial2, R_SENSE, 0); // Driver 2 communicates through TX16 and RX17 (Arudino MEGA default)

void setup() {
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
}

void loop() {
  moveBothMotors(4, HIGH, LOW);
  delay(2000);
  moveBothMotors(4, LOW, HIGH);
  delay(2000);
}

// === Move both motors together for given revolutions ===
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