//Improved control logic with progressive steering radii

const int steeringInputPin = 2; 
const int throttlingInputPin = 3;

// Defining motor A control inputs
const int motorA1Pin = 7;
const int motorA2Pin = 6;
const int motorAPwmPin = 5;

// Defining motor B control inputs
const int motorB1Pin = 8; 
const int motorB2Pin = 10;
const int motorBPwmPin = 9;

// Steering configuration
const float STEER_DEADZONE = 5.0;  // Reduced deadzone for finer control
const float STEER_CURVE_EXPONENT = 1.5;  // Controls steering progressiveness (1.0 = linear, >1.0 = more progressive)
const float MIN_TURN_SPEED_FACTOR = 0.3;  // Minimum speed on inside wheel during turn (30%)
const float ZERO_RADIUS_THRESHOLD = 35.0;  // Steering must exceed this for zero-radius turn

void setup() {
  pinMode(steeringInputPin, INPUT);
  pinMode(throttlingInputPin, INPUT);

  pinMode(motorA1Pin, OUTPUT);
  pinMode(motorA2Pin, OUTPUT);
  pinMode(motorAPwmPin, OUTPUT);
  
  pinMode(motorB1Pin, OUTPUT);
  pinMode(motorB2Pin, OUTPUT);
  pinMode(motorBPwmPin, OUTPUT);

  Serial.begin(9600);
  Serial.println("Progressive Steering Control Active");
}

void loop() {
  float steer = GetPWM(steeringInputPin);
  float throttle = GetPWM(throttlingInputPin);
  
  // Check if transmitter is off
  if (steer == 0 && throttle == 0) {
    Serial.println("Make sure your transmitter is ON");
    stopMotors();
    return;
  }

  // Calculate base speed from throttle
  int speedscalemax = 255;
  float speed = speedscalemax * ((throttle - 50) / 50);
  
  // Center steering around 0 (-50 to +50)
  float steerCentered = steer - 50;
  
  // Calculate turn scales with progressive steering
  float leftTurnScale = 1.0;
  float rightTurnScale = 1.0;
  
  // Apply deadzone
  if (abs(steerCentered) > STEER_DEADZONE) {
    
    // Normalize steering to -1.0 to +1.0 range (outside deadzone)
    float steerNormalized;
    if (steerCentered > 0) {
      // Left turn: map from DEADZONE to 50 → 0 to 1.0
      steerNormalized = (steerCentered - STEER_DEADZONE) / (50 - STEER_DEADZONE);
    } else {
      // Right turn: map from -DEADZONE to -50 → 0 to -1.0
      steerNormalized = (steerCentered + STEER_DEADZONE) / (50 - STEER_DEADZONE);
    }
    
    // Clamp to [-1.0, 1.0]
    steerNormalized = constrain(steerNormalized, -1.0, 1.0);
    
    // Apply curve for progressive response
    // This creates smoother gradations in turning
    float steerCurved = sign(steerNormalized) * pow(abs(steerNormalized), STEER_CURVE_EXPONENT);
    
    // Calculate turn reduction factor (how much to slow down inside wheel)
    // Goes from 1.0 (straight) to MIN_TURN_SPEED_FACTOR (full turn)
    float turnReduction = 1.0 - (abs(steerCurved) * (1.0 - MIN_TURN_SPEED_FACTOR));
    
    if (steerCurved < 0) {  
      // Turn RIGHT: reduce right motor speed
      rightTurnScale = turnReduction;
      leftTurnScale = 1.0;
    } else {  
      // Turn LEFT: reduce left motor speed
      leftTurnScale = turnReduction;
      rightTurnScale = 1.0;
    }
    
    // Special case: Zero-radius turn when throttle is near neutral
    if (abs(speed) < 20 && abs(steerCentered) > ZERO_RADIUS_THRESHOLD) {
      speed = 255;  // Full power for pivot turn
      
      // Calculate how far past threshold (0 to 1.0)
      float pivotIntensity = (abs(steerCentered) - ZERO_RADIUS_THRESHOLD) / (50 - ZERO_RADIUS_THRESHOLD);
      pivotIntensity = constrain(pivotIntensity, 0.0, 1.0);
      
      if (steerCentered < 0) {
        // Pivot right: left forward, right backward
        leftTurnScale = 1.0;
        rightTurnScale = -pivotIntensity;  // Gradually increase reverse speed
      } else {
        // Pivot left: right forward, left backward
        rightTurnScale = 1.0;
        leftTurnScale = -pivotIntensity;  // Gradually increase reverse speed
      }
    }
    
    // Debug output
    Serial.print("Steer: ");
    Serial.print(steerCentered);
    Serial.print(" | Normalized: ");
    Serial.print(steerNormalized);
    Serial.print(" | Curved: ");
    Serial.print(steerCurved);
    Serial.print(" | L_Scale: ");
    Serial.print(leftTurnScale);
    Serial.print(" | R_Scale: ");
    Serial.println(rightTurnScale);
  }

  // Calculate final motor speeds
  float leftSpeed = speed * leftTurnScale;
  float rightSpeed = speed * rightTurnScale;

  // Set motor directions
  if (leftSpeed < 0) {
    digitalWrite(motorA1Pin, LOW);
    digitalWrite(motorA2Pin, HIGH);
  } else {
    digitalWrite(motorA1Pin, HIGH);
    digitalWrite(motorA2Pin, LOW);
  }
  
  if (rightSpeed < 0) {
    digitalWrite(motorB1Pin, LOW);
    digitalWrite(motorB2Pin, HIGH);
  } else {
    digitalWrite(motorB1Pin, HIGH);
    digitalWrite(motorB2Pin, LOW);
  }

  // Apply speed deadzone and set PWM
  if (abs(leftSpeed) < 20) leftSpeed = 0;
  if (abs(rightSpeed) < 20) rightSpeed = 0;
  
  analogWrite(motorAPwmPin, abs(leftSpeed));
  analogWrite(motorBPwmPin, abs(rightSpeed));
  
  // Debug output
  Serial.print("Speed: ");
  Serial.print(speed);
  Serial.print(" | L: ");
  Serial.print(leftSpeed);
  Serial.print(" | R: ");
  Serial.println(rightSpeed);
}

float sign(float value) {
  if (value > 0) return 1.0;
  if (value < 0) return -1.0;
  return 0.0;
}

void stopMotors() {
  analogWrite(motorAPwmPin, 0);
  analogWrite(motorBPwmPin, 0);
  digitalWrite(motorA1Pin, LOW);
  digitalWrite(motorA2Pin, LOW);
  digitalWrite(motorB1Pin, LOW);
  digitalWrite(motorB2Pin, LOW);
}

byte GetPWM(byte pin) {
  unsigned long highTime = pulseIn(pin, HIGH, 50000UL);
  unsigned long lowTime = pulseIn(pin, LOW, 50000UL);
  float rawPercent;
  float scaledPercent;

  if (highTime == 0 || lowTime == 0)
    return digitalRead(pin) ? 100 : 0;

  rawPercent = ((100.0 * highTime) / (highTime + lowTime));
  scaledPercent = (rawPercent - 6.65) * 13.09;
  scaledPercent = constrain(scaledPercent, 0, 100);

  return scaledPercent;
}