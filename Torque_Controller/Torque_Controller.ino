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

// Control Mode
enum ControlMode { POSITION, TORQUE };
ControlMode controlMode = TORQUE; // Default mode: position control

// PI variables for current controller
float desiredCurrent = 100.0; // Desired current in mA
float Kp_current = 0.1;       // Proportional gain for current
float Ki_current = 0.01;      // Integral gain for current
float integral_current = 0.0; // Integral term for current control
float pwmOutput = 0;          // PWM output for motor (-255 to 255)

// PID variables for position controller
float desiredPosition = 180.0; // Desired position in degrees
float Kp_position = 0.5;       // Proportional gain for position
float Ki_position = 0.0;       // Integral gain for position
float Kd_position = 0.1;       // Derivative gain for position
float integral_position = 0.0; // Integral term for position
float previousPositionError = 0.0;

// Torque Control Variables
float desiredTorque = 0.0;     // Desired torque in Nm
float kt = 0.001;              // Torque Constant

// Timing
unsigned long previousTime = 0;
unsigned long currentTime = 0;
float deltaTime = 0;

// Filtering variables
float filteredCurrent = 0.0; // Filtered current value
float alpha = 0.2;           // Smoothing factor (0 < alpha <= 1)

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
  Serial.println("Switch Mode: M POSITION or M TORQUE");
  Serial.println("Set Desired Torque: T <value>");
}

float readEncoderAngle() {
  encoder.updateMovingAvgExp();   // Initialize moving average

  // Read the angle in degrees from the encoder
  float currentAngle = encoder.angleR(U_DEG, false);

  // Handle wrap-around logic
  if (currentAngle < previousAngle - 180) {
    // If the angle jumps from ~360 to ~0
    totalAngle += (360 + currentAngle - previousAngle);
  } else if (currentAngle > previousAngle + 180) {
    // If the angle jumps from ~0 to ~360
    totalAngle -= (360 - currentAngle + previousAngle);
  } else {
    // Normal increment
    totalAngle += (currentAngle - previousAngle);
  }

  // Update the previous angle
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

  // Control logic based on mode
  if (controlMode == POSITION) {
    // Read position from encoder
    float currentPosition = readEncoderAngle();

    // Outer loop: Position control (PID)
    float positionError = desiredPosition - currentPosition;
    integral_position += positionError * deltaTime;
    float derivative_position = (positionError - previousPositionError) / deltaTime;
    previousPositionError = positionError;

    // Desired current based on position PID output
    desiredCurrent = Kp_position * positionError + Ki_position * integral_position + Kd_position * derivative_position;

  } else if (controlMode == TORQUE) {
    // Torque control: Desired current is directly proportional to desired torque
    desiredCurrent = desiredTorque / kt;
  }

  // Inner loop: Current control (PI)
  float currentError = desiredCurrent - filteredCurrent;
  integral_current += currentError * deltaTime;

  // Calculate PWM output for motor
  pwmOutput = Kp_current * currentError + Ki_current * integral_current;

  // Constrain PWM output to valid range
  pwmOutput = constrain(pwmOutput, -255, 255);

  // Apply PWM and direction control
  if (pwmOutput > 0) {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    analogWrite(enA, pwmOutput); // Apply positive PWM
  } else if (pwmOutput < 0) {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    analogWrite(enA, -pwmOutput); // Apply the absolute value of negative PWM
  } else {
    // Stop the motor
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
    analogWrite(enA, 0);
  }

  // Debugging output
  Serial.print("Control Mode: ");
  Serial.print(controlMode == POSITION ? "POSITION" : "TORQUE");
  Serial.print(" | Desired Position: ");
  Serial.print(desiredPosition);
  Serial.print(" deg | Desired Torque: ");
  Serial.print(desiredTorque);
  Serial.print(" | Desired Current: ");
  Serial.print(desiredCurrent);
  Serial.print(" mA | Measured Current: ");
  Serial.print(current_mA);
  Serial.print(" mA | PWM Output: ");
  Serial.println(pwmOutput);

  // Check for Serial input to tune gains and switch modes
  if (Serial.available() > 0) {
    handleSerialInput();
  }

  // Small delay to stabilize readings
  delay(20);
}

void handleSerialInput() {
  String input = Serial.readStringUntil('\n'); // Read input until newline
  input.trim(); // Remove whitespace and newline

  if (input.startsWith("P ")) { // Set Kp for position
    Kp_position = input.substring(2).toFloat();
    Serial.print("Updated Kp_position: ");
    Serial.println(Kp_position);
  } else if (input.startsWith("I ")) { // Set Ki for position
    Ki_position = input.substring(2).toFloat();
    Serial.print("Updated Ki_position: ");
    Serial.println(Ki_position);
  } else if (input.startsWith("D ")) { // Set Kd for position
    Kd_position = input.substring(2).toFloat();
    Serial.print("Updated Kd_position: ");
    Serial.println(Kd_position);
  } else if (input.startsWith("CP ")) { // Set Kp for current
    Kp_current = input.substring(3).toFloat();
    Serial.print("Updated Kp_current: ");
    Serial.println(Kp_current);
  } else if (input.startsWith("CI ")) { // Set Ki for current
    Ki_current = input.substring(3).toFloat();
    Serial.print("Updated Ki_current: ");
    Serial.println(Ki_current);
  } else if (input.startsWith("M ")) { // Switch control mode
    String mode = input.substring(2);
    if (mode == "POSITION") {
      controlMode = POSITION;
      Serial.println("Switched to POSITION control mode.");
    } else if (mode == "TORQUE") {
      controlMode = TORQUE;
      Serial.println("Switched to TORQUE control mode.");
    } else {
      Serial.println("Invalid mode. Use M POSITION or M TORQUE.");
    }
  } else if (input.startsWith("T ")) { // Set desired torque
    desiredTorque = input.substring(2).toFloat();
    Serial.print("Updated Desired Torque: ");
    Serial.println(desiredTorque);

    // Reset integral term if torque is set to 0
    if (desiredTorque == 0) {
      integral_current = 0.0;
      Serial.println("Reset integral term for current control.");
    }
  } else {
    Serial.println("Invalid command. Use:");
    Serial.println("P <value> - Set Kp_position");
    Serial.println("I <value> - Set Ki_position");
    Serial.println("D <value> - Set Kd_position");
    Serial.println("CP <value> - Set Kp_current");
    Serial.println("CI <value> - Set Ki_current");
    Serial.println("M POSITION - Switch to position control");
    Serial.println("M TORQUE - Switch to torque control");
    Serial.println("T <value> - Set desired torque");
  }
}
