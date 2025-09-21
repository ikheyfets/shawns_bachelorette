const int steeringInputPin = 2; // Connect the PWM signal from the receiver channel 1 (steering)
const int throttlingInputPin = 3; // Connect the PWM signal from the receiver channel 2 (throttling)

// Defining motor A control inputs
const int motorA1Pin = 10;
const int motorA2Pin = 9;

// Defining motor B control inputs
const int motorB1Pin = 6;
const int motorB2Pin = 5;

// PWM code source: https://lastminuteengineers.com/drv8833-arduino-tutorial/

void setup() {
  // Set pin modes
  pinMode(steeringInputPin, INPUT);
  pinMode(throttlingInputPin, INPUT);

  pinMode(motorA1Pin, OUTPUT);
  pinMode(motorA2Pin, OUTPUT);
  pinMode(motorB1Pin, OUTPUT);
  pinMode(motorB2Pin, OUTPUT);

  TurnOffMotors(motorA1Pin, motorA2Pin, motorB1Pin, motorB2Pin);

  // Start serial communication at 9600 bps
  Serial.begin(9600);

  Serial.println("Reading digital state...");
}

void loop() {
  byte steer = GetPWM(steeringInputPin);
  byte throttle = GetPWM(throttlingInputPin);
  
  //  if both are zero, the transmitter is OFF, do nothing 
  if (steer == 0 && throttle == 0) {
    Serial.println("Make sure your transmitter is ON");
    return; // Skip the remaining loop and move on to the next iteration
  }


  
  // Ramp speed up.
  for (int i = 0; i < 11; i++) {
    spin_and_wait(25*i, 25*i, 500);
  }

  // Log the data
  Serial.print("steer PWM = ");
  Serial.print(steer);
  Serial.println();

  Serial.print("throttle PWM = ");
  Serial.print(throttle);
  Serial.println();

  // TODO: normalize the PWM data

  // TODO: custom logic to drive two motors

  // Wait for a short duration
  delay(10);
}

/// Set the current on a motor channel using PWM and directional logic.
///
/// param pwm    PWM duty cycle ranging from -255 full reverse to 255 full forward
/// param IN1_PIN  pin number xIN1 for the given channel
/// param IN2_PIN  pin number xIN2 for the given channel
void set_motor_pwm(int pwm, int IN1_PIN, int IN2_PIN)
{
  if (pwm < 0) {  // reverse speeds
    analogWrite(IN1_PIN, -pwm);
    digitalWrite(IN2_PIN, LOW);

  } else { // stop or forward
    digitalWrite(IN1_PIN, LOW);
    analogWrite(IN2_PIN, pwm);
  }
}

/// Set the current on both motors.
///
/// param pwm_A  motor A PWM, -255 to 255
/// param pwm_B  motor B PWM, -255 to 255
void set_motor_currents(int pwm_A, int pwm_B)
{
  set_motor_pwm(pwm_A, motorA1Pin, motorA2Pin);
  set_motor_pwm(pwm_B, motorB1Pin, motorB2Pin);

  // Print a status message to the console.
  Serial.print("Set motor A PWM = ");
  Serial.print(pwm_A);
  Serial.print(" motor B PWM = ");
  Serial.println(pwm_B);
}

/// Simple primitive for the motion sequence to set a speed and wait for an interval.
///
/// param pwm_A  motor A PWM, -255 to 255
/// param pwm_B  motor B PWM, -255 to 255
/// param duration delay in milliseconds
void spin_and_wait(int pwm_A, int pwm_B, int duration) {
  set_motor_currents(pwm_A, pwm_B);
  delay(duration);
}


// source: https://forum.arduino.cc/t/reading-a-pwm-signal/603967/4
byte GetPWM(byte pin) {
  unsigned long highTime = pulseIn(pin, HIGH,50000UL);  // 50 millisecond timeout
  unsigned long lowTime = pulseIn(pin, LOW, 50000UL);  // 50 millisecond timeout

  // pulseIn() returns zero on timeout
  if (highTime == 0 || lowTime == 0)
    return digitalRead(pin) ? 100 : 0;  // HIGH == 100%,  LOW = 0%

  return (100 * highTime) / (highTime + lowTime);  // highTime as percentage of total cycle time
}

void TurnOffMotors(int motorA1, int motorA2, int motorB1, int motorB2) {
  int allPins[] = {motorA1, motorA2, motorB1, motorB2};
  for (int i=0; i<sizeof(allPins); i++) {
    digitalWrite(allPins[i], LOW);
  }
}
