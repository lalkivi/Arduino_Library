#include <Wire.h>
#include <Adafruit_INA219.h>
#include <ams_as5048b.h>

// INA219 (current sensor) and AS5048B (encoder) setup
Adafruit_INA219 ina219(0x41); // Current sensor at address 0x41
AMS_AS5048B encoder(0x40);   // Encoder at address 0x40

// Motor control pins (L298N)
#define enA 9
#define in1 6
#define in2 7

// PI variables for current controller
float desiredCurrent = 100.0; // Desired current in mA
float Kp_current = 0.1;       // Proportional gain for current
float Ki_current = 0.01;      // Integral gain for current
float integral_current = 0.0; // Integral term for current control
float pwmOutput = 0;          // PWM output for motor (-255 to 255)

// PID variables for position controller
float desiredPosition = 180.0; // Desired position in degrees
float Kp_position = 12.5;       // Proportional gain for position
float Ki_position = 0.0;       // Integral gain for position
float Kd_position = 0.1;       // Derivative gain for position
float integral_position = 0.0; // Integral term for position
float previousPositionError = 0.0;

// Timing
unsigned long previousTime = 0;
unsigned long currentTime = 0;
float deltaTime = 0;

// Filtering variables
float filteredCurrent = 0.0; // Filtered current value
float alpha = 0.1;           // Smoothing factor (0 < alpha <= 1) 

// Encoder variables
float previousAngle = 0.0; // Previous angle from the encoder
float totalAngle = 0.0;    // Total angle for wrap-around logic

void setup() {
  // Motor control setup
  pinMode(enA, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);

  // Set initial motor direction
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);

  // Serial setup
  Serial.begin(115200);
  while (!Serial) {
    delay(1); // Wait for Serial console
  }

  // INA219 setup
  Serial.println("Initializing INA219...");
  if (!ina219.begin()) {
    Serial.println("Failed to find INA219 chip at address 0x41");
    while (1) {
      delay(10);
    }
  }

  // AS5048B encoder setup
  Serial.println("Initializing AS5048B Encoder...");
  encoder.begin();                // Initialize encoder
  encoder.setClockWise(true);     // Set clockwise counting

  Serial.println("Starting current and position control...");
  Serial.println("Commands:");
  Serial.println("Set Position PID gains: P <Kp_position>, I <Ki_position>, D <Kd_position>");
  Serial.println("Set Current PI gains: CP <Kp_current>, CI <Ki_current>");
}

float readEncoderAngle() {
  encoder.updateMovingAvgExp();   // Initialize moving average

  // Read the angle in degrees from the encoder
  float currentAngle = encoder.angleR(U_DEG, false);

  // Handle wrap-around logic
  if (currentAngle < previousAngle - 180) {
    totalAngle += (360 + currentAngle - previousAngle);
  } else if (currentAngle > previousAngle + 180) {
    totalAngle -= (360 - currentAngle + previousAngle);
  } else {
    totalAngle += (currentAngle - previousAngle);
  }

  previousAngle = currentAngle;
  return totalAngle; // Return the wrapped angle
}

void loop() {
  // Timing calculations
  currentTime = millis();
  deltaTime = (currentTime - previousTime) / 1000.0; // Convert to seconds
  previousTime = currentTime;

  // Read current from INA219
  float current_mA = ina219.getCurrent_mA();

  // Apply exponential moving average filter to current
  filteredCurrent = alpha * current_mA + (1 - alpha) * filteredCurrent;

  // Read position from encoder
  float currentPosition = readEncoderAngle();

  // Outer loop: Position control (PID)
  float positionError = desiredPosition - currentPosition;
  integral_position += positionError * deltaTime;
  integral_position = constrain(integral_position, -100, 100); // Anti-windup for position integral
  float derivative_position = (positionError - previousPositionError) / deltaTime;
  previousPositionError = positionError;

  // Desired current based on position PID output
  desiredCurrent = Kp_position * positionError + Ki_position * integral_position + Kd_position * derivative_position;

  // Inner loop: Current control (PI)
  float currentError = desiredCurrent - filteredCurrent;
  integral_current += currentError * deltaTime;
  integral_current = constrain(integral_current, -100, 100); // Anti-windup for current integral

  // Calculate PWM output for motor
  pwmOutput = Kp_current * currentError + Ki_current * integral_current;
  pwmOutput = constrain(pwmOutput, -255, 255); // Constrain PWM output to valid range

  // Small deadband to prevent small drift
  if (abs(pwmOutput) < 5) {
    pwmOutput = 0;
  }

  // Apply PWM and direction control
  if (pwmOutput > 0) {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    analogWrite(enA, pwmOutput); 
  } else if (pwmOutput < 0) {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    analogWrite(enA, -pwmOutput); 
  } else {
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
    analogWrite(enA, 0);
  }

  // Debugging output
  Serial.print("Desired Position: ");
  Serial.print(desiredPosition);
  Serial.print(" deg | Current Position: ");
  Serial.print(currentPosition);
  Serial.print(" deg | Desired Current: ");
  Serial.print(desiredCurrent);
  Serial.print(" mA | Measured Current: ");
  Serial.print(current_mA);
  Serial.print(" mA | PWM Output: ");
  Serial.println(pwmOutput);

  delay(20);
}

void handleSerialInput() {
  String input = Serial.readStringUntil('\n');
  input.trim();
  if (input.startsWith("P ")) {
    Kp_position = input.substring(2).toFloat();
  } else if (input.startsWith("I ")) {
    Ki_position = input.substring(2).toFloat();
  } else if (input.startsWith("D ")) {
    Kd_position = input.substring(2).toFloat();
  } else if (input.startsWith("CP ")) {
    Kp_current = input.substring(3).toFloat();
  } else if (input.startsWith("CI ")) {
    Ki_current = input.substring(3).toFloat();
  } else {
    Serial.println("Invalid command. Use:");
    Serial.println("P <value>");
    Serial.println("I <value>");
    Serial.println("D <value>");
    Serial.println("CP <value>");
    Serial.println("CI <value>");
  }
}
