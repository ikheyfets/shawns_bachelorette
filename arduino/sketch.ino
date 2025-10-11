//The idea here is to create some arbsurdly wonky control logic that can be used to control the motors

const int steeringInputPin = 3; // Connect the PWM signal from the receiver channel 1 (steering)
const int throttlingInputPin = 2; // Connect the PWM signal from the receiver channel 2 (throttling)

// Defining motor A control inputs
const int motorB1Pin = 7; //was 10 
const int motorB2Pin = 6; //was 9
const int motorBPwmPin = 5; // moved pins around because one of them needs to be PWM 

// // Defining motor B control inputs
const int motorA1Pin = 8; 
const int motorA2Pin = 10; //was 5
const int motorAPwmPin = 9;

// PWM code source: https://lastminuteengineers.com/drv8833-arduino-tutorial/

void setup() {
  // Set pin modes
  pinMode(steeringInputPin, INPUT);
  pinMode(throttlingInputPin, INPUT);

  pinMode(motorA1Pin, OUTPUT);
  pinMode(motorA2Pin, OUTPUT);
  pinMode(motorAPwmPin, OUTPUT);
  
  pinMode(motorB1Pin, OUTPUT);
  pinMode(motorB2Pin, OUTPUT);
  pinMode(motorBPwmPin, OUTPUT);


  // TurnOffMotors(motorA1Pin, motorA2Pin, motorB1Pin, motorB2Pin);

  // Start serial communication at 9600 bps
  Serial.begin(9600);

  Serial.println("Reading digital state...");
}

void loop() {
  // Serial.print("Steer Percentage = "); //100 is left, 0 is right
  float steer = GetPWM(steeringInputPin);
  // Serial.print("Throttle Percentage = "); //100 is full forward, 0 is full reverse
  float throttle = GetPWM(throttlingInputPin);
  // Serial.println("");
  // Serial.println("");
  // Serial.println("throttle = ");
  // Serial.println(GetPWM(throttlingInputPin));
  // Serial.println("");


  
  //  if both are zero, the transmitter is OFF, do nothing 
  if (steer == 0 && throttle == 0) {
    Serial.println("Make sure your transmitter is ON");
    return; // Skip the remaining loop and move on to the next iteration
  }

  // Serial.println(throttle);
  // Serial.println(steer);

  // //  6-10 is throttle back, 10-14 is throttle forward
  int speedscalemax = 255;
  float speed;
  speed = speedscalemax * ((throttle - 50)/50);
  // Serial.println("Setting speed to:");
  // Serial.println(speed);


  // Do Steering Calculations

  // Set steering scaleing factor can be -1.0 to 1.0
  float leftTurnScale = 1.0;
  float rightTurnScale = 1.0;

  // bound steer -50 to 50;
  steer = steer - 50;

  //  Serial.print("steer");
  //  Serial.println(steer);

  //set a deadzone of 10%
  if (abs(steer) > 10) {

    if (steer < 0) {  // Turn right
      // Steer is -50 to 0 here
      leftTurnScale = (steer + 25) / 25;
    } else {  // Turn Left
      // Steer is 0 to 50 here
      rightTurnScale = (25 - steer) / 25;
    }

    Serial.println(leftTurnScale);
    Serial.println(rightTurnScale);


    // Special case for no throttle turning (zero radius)
    // Check that throttle is in the dead zone
    // Only zero radius turn if turning more than half way in that direction
    if (abs(speed) < 20 && abs(steer) > 25) {
      // Set speed to be scaled down on the motors
      speed = 255;
      // If turning right, set our left motor to be the opposite direction and power
      if (steer > 0)
        leftTurnScale = -rightTurnScale;
      // If turning left, set our right motor to be the opposite direction and power
      else
        rightTurnScale = -leftTurnScale;
    }

  }

  float leftSpeed = speed * leftTurnScale;
  float rightSpeed = speed * rightTurnScale;

  // set direction:
  if (leftSpeed < 0) {
    digitalWrite(motorA1Pin, LOW);
    digitalWrite(motorA2Pin, HIGH);
  } else {
    digitalWrite(motorA1Pin, HIGH);
    digitalWrite(motorA2Pin, LOW);
  }
  if (rightSpeed < 0) {
    digitalWrite(motorB1Pin, HIGH);
    digitalWrite(motorB2Pin, LOW);
  } else {
    digitalWrite(motorB1Pin, LOW);
    digitalWrite(motorB2Pin, HIGH);
  }

  //  set duty cycle:
  if ( abs(leftSpeed) < 20 )
    leftSpeed = 0;
  if ( abs(leftSpeed) < 20 )
    rightSpeed = 0;

  analogWrite(motorAPwmPin, abs(leftSpeed));
  analogWrite(motorBPwmPin, abs(rightSpeed));
}

// source: https://forum.arduino.cc/t/reading-a-pwm-signal/603967/4
byte GetPWM(byte pin) {
  unsigned long highTime = pulseIn(pin, HIGH,50000UL);  // 50 millisecond timeout
  unsigned long lowTime = pulseIn(pin, LOW, 50000UL);  // 50 millisecond timeout
  float rawPercent;
  float scaledPercent; //function returns a byte anyway

  // pulseIn() returns zero on timeout
  if (highTime == 0 || lowTime == 0)
    return digitalRead(pin) ? 100 : 0;  // HIGH == 100%,  LOW = 0%

  // Calculate the raw duty cycle percentage
  rawPercent = ((100.0 * highTime) / ( highTime + lowTime ));

  // Scale the 6 to 14 percentage to the full 100 percentage scale
  // These scaling factors were determined by the range on the throttle signal
  scaledPercent = ( rawPercent - 6.65 ) * 13.09;

  // Clamp the scaled percentage to be between 0 and 100 for later scaling
  scaledPercent = constrain ( scaledPercent, 0, 100 );

  // Return a value between 0 to 100, with based on the recieved radio input signal
  return scaledPercent;
  //return (100 * highTime) / (highTime + lowTime);  // highTime as percentage of total cycle time
}
