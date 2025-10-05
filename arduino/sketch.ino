//The idea here is to create some arbsurdly wonky control logic that can be used to control the motors

const int steeringInputPin = 2; // Connect the PWM signal from the receiver channel 1 (steering)
const int throttlingInputPin = 3; // Connect the PWM signal from the receiver channel 2 (throttling)

// Defining motor A control inputs
const int motorA1Pin = 7; //was 10 
const int motorA2Pin = 6; //was 9
const int motorAPwmPin = 5; // moved pins around because one of them needs to be PWM 

// // Defining motor B control inputs
const int motorB1Pin = 8; 
const int motorB2Pin = 10; //was 5
const int motorBPwmPin = 9;

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
  byte steer = GetPWM(steeringInputPin);
  // Serial.print("Throttle Percentage = "); //100 is full forward, 0 is full reverse
  float throttle = GetPWM(throttlingInputPin);
   Serial.println("");
  Serial.println("");
  Serial.println("Throttle = ");
  Serial.println(throttle);
  Serial.println("");
  
  //  if both are zero, the transmitter is OFF, do nothing 
  if (steer == 0 && throttle == 0) {
    Serial.println("Make sure your transmitter is ON");
    return; // Skip the remaining loop and move on to the next iteration
  }

  //  6-10 is throttle back, 10-14 is throttle forward
  int speedscalemax = 255;
  float speed;
  speed = speedscalemax * ((throttle - 10)/4);
  Serial.println("Setting speed to:");
  Serial.println(speed);

  // set direction:
  if (speed < 0) {
      digitalWrite(motorA1Pin, LOW);
      digitalWrite(motorA2Pin, HIGH);
      digitalWrite(motorB1Pin, LOW);
      digitalWrite(motorB2Pin, HIGH);
  } else {
      digitalWrite(motorA1Pin, HIGH);
      digitalWrite(motorA2Pin, LOW);
      digitalWrite(motorB1Pin, HIGH);
      digitalWrite(motorB2Pin, LOW);
  }
  //  set duty cycle:
  analogWrite(motorAPwmPin, abs(speed));
  analogWrite(motorBPwmPin, abs(speed));
}

// source: https://forum.arduino.cc/t/reading-a-pwm-signal/603967/4
byte GetPWM(byte pin) {
  unsigned long highTime = pulseIn(pin, HIGH,50000UL);  // 50 millisecond timeout
  unsigned long lowTime = pulseIn(pin, LOW, 50000UL);  // 50 millisecond timeout
  float rawpercent;
  float scaledpercent; //function returns a byte anyway

  // pulseIn() returns zero on timeout
  if (highTime == 0 || lowTime == 0)
    return digitalRead(pin) ? 100 : 0;  // HIGH == 100%,  LOW = 0%

  return (100 * highTime) / (highTime + lowTime);  // highTime as percentage of total cycle time
}

//Brainstorm junk

//ThrottleNorm is a normalized version of Throttle, which is the PWM Input Signal of Channel2 (trigger)
//void loop() {
//  byte ThrottleNorm = (Throttle - 10) * 0.25;
//SteeringNorm is a normalized version of Steering, which is the PWM Input Signal of Channel1 (wheel)
//  byte SteeringNorm = (Steer - 10) * 0.25;
//}

//ThrottleIntermediate will control both motors equally and proportionally to the duty cycle of the signal from Channel2 (trigger)

//ThrottleIntermediate = f(ThrottleNorm) //some function of ThrottleNorm, modified as needed

//SteeringIntermediate1 will control the left motor proportionally to the duty cycle of the signal from Channel1 (wheel)
//SteeringIntermediate1 = f(SteeringNorm) //some function of SteeringNorm that gives a value that the H bridge interprets

//SteeringIntermediate2 will control the right motor counter-proportionally to the duty cycle of the signal from Channel1 (wheel).
//SteeringIntermediate2 = f-1(SteeringNorm) //a function of SteeringNorm that gives the opposite value from that of SteeringIntermediate1

//OutputSignal1 will be a function of both ThrottleIntermediate and SteeringIntermediate1
//OutputSignal1 connected to H Bridge of Left Motor
//OutputSignal1 = ThrottleIntermediate * SteeringIntermediate1 //Tweak to make sense to drive treads

//OutputSignal2 will be a function of both ThrottleIntermediate and SteeringIntermediate2
//OutputSignal2 connected to H Bridge of Right Motor
//OutputSignal2 = ThrottleIntermediate * SteeringIntermediate2 //Tweak to make sense to drive treads
