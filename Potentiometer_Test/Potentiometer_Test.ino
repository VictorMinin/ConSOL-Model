/*
  Run this script to experimentally determine the voltage values of the potentiometer once the desired extension of the actuator
  is achieved
  NOTE: my ADC equation transforming the analog value to digitial is only valid if using the boards 5V power
  if another voltage source/ value is used to power the potentiometer then that value must be accounted for in the EQ.
  
  To set up the potentiometer correctly simply attach the boards ground to the potentiometer's ground, the 5v to the positive V on 
  the potentiometer, and the A0 analog pin to the 3rd pin on the potentiometer.
*/
void setup() {
  Serial.begin(115200);
}

void loop() {
  int sensorValue = analogRead(A0);
  float voltage = sensorValue * (5.0 / 1023.0); // ADC EQ is V = Analog * (max Voltage/ bit resolution of ADC)
  Serial.println(sensorValue);
  delay(50);
}
