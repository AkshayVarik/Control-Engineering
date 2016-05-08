//
// choose unique trainID (0 < ID < 5) for sending wireless data
// 
#define train_ID 6
//
// set the digital control frame rate
//

///////////// ZigBee Setup ////////////////

#include <SoftwareSerial.h> // <--- INSTALL THIS LIBRARY IF NEEDED
SoftwareSerial ZigBee(11, 12);
int ZigBee_5v = 10;
int ZigBee_GND1 = 9;
int ZigBee_GND2 = 8;
char buf[50];

///////////////////////////////////////////

double deltaT_ms = 50;

void setup(void)
{
  //
  // Initialize serial communication (Xbee)
  //
  Serial.begin(9600);
  Serial.println("Train Control Running");
  //
  // initialize the train motor
  //
  setTrainMotorPWM(0);
  
  ////////// Initialize Zigbee //////////////
  pinMode(ZigBee_5v, OUTPUT);
  digitalWrite(ZigBee_5v, HIGH);
  pinMode(ZigBee_GND1, OUTPUT);
  digitalWrite(ZigBee_GND1, LOW);
  pinMode(ZigBee_GND2, OUTPUT);
  digitalWrite(ZigBee_GND2, LOW);
  ZigBee.begin(9600);
  ///////////////////////////////////////////
  
}

void loop(void)
{
  //
  // wait for fixed frame rate
  //
  static unsigned long millisLast = 0;
  word deltaT_ms_actual;
  while (millis() - millisLast < deltaT_ms) {  }
  deltaT_ms_actual = millis() - millisLast;
  millisLast = millis();
  //
  // Get the distance to the train ahead
  //
  double d_cm = 0.0;
  d_cm = getPingDistance_cm();
  //
  // do the control law calculations
  //
  byte train_control_pwm;
  train_control_pwm = getTrainPWM(d_cm);
  //
  // send the PWM command to the motor
  //
  setTrainMotorPWM(train_control_pwm);
  //
  // compute distance as integer in millimeters to reduced transmission bandwidth
  //
  word d_mm;
  d_mm = 10 * d_cm;
  //
  // now send over conventional serial (to PC)
  //
//  Serial.println("ESE"),%2d,%3d,%4d,%3d,\r\n", train_ID, deltaT_ms_actual, d_mm, train_control_pwm);
  Serial.print(deltaT_ms_actual); Serial.print(" ");
  Serial.print(train_control_pwm); Serial.print(" ");
  Serial.println(d_mm);
  
  ///////////////////////// Create Zigbee Packet and Transmit! //////////////////////////
  memset(buf, 0, 50);
  sprintf(buf, "ESE,%2d,%4d,%3d,T\r\n", train_ID, d_mm, train_control_pwm);
  ZigBee.print(buf);
  //////////////////////////////////////////////////////////////////////////////////////
  
}

////////////////////////////////////////////////////////////
// Ping Sensor Input
////////////////////////////////////////////////////////////
double getPingDistance_cm()
{
  //
  // Pin # of PING sensor input
  //
  const int pingPin = 2;
  const long timeout_us = 5000;
  //
  // The PING))) is triggered by a HIGH pulse of 2 or more microseconds.
  // Give a short LOW pulse beforehand to ensure a clean HIGH pulse:
  //
  pinMode(pingPin, OUTPUT);
  digitalWrite(pingPin, LOW);
  delayMicroseconds(2);
  digitalWrite(pingPin, HIGH);
  delayMicroseconds(5);
  digitalWrite(pingPin, LOW);
  //
  // The same pin is used to read the signal from the PING))): a HIGH
  // pulse whose duration is the time (in microseconds) from the sending
  // of the ping to the reception of its echo off of an object.
  //
  unsigned long echo_time;
  pinMode(pingPin, INPUT);
  echo_time = pulseIn(pingPin, HIGH, timeout_us);
  if (echo_time == 0)
 {
    echo_time = timeout_us;
 }
  //
  // divide by 2 because we measure "round trip" time for echo
  // distance = (10^-6) * (echo_time_us) * (speed of sound m/s) * (100 cm/m) / 2
  // (0.000001 * echo_time_us * 340.0 * 100.0 / 2.0)
  // = 0.017*echo_time
  //
  return 0.017 * echo_time;
}

////////////////////////////////////////////////////////////
// Train Motor Output
////////////////////////////////////////////////////////////
void setTrainMotorPWM(byte pwm_command)
{
  byte pwm_command_limited;
  const int pwmPin1 = 6;
  //  const int dirPin1 = 12;
  static byte first_call = 1;
//
// first time just set the pwm to 0
//
  if (first_call)
  {
    analogWrite(pwmPin1, 0);
    first_call = 0;
  }
//
// limit the PWM to reduce train tipovers
// DON'T CHANGE THIS!
//
  else
  {
    pwm_command_limited = constrain(pwm_command, 0, 115);
    analogWrite(pwmPin1, pwm_command_limited);
    delay(2);
  }

  return;
}


////////////////////////////////////////////////////////////
// train control law goes here
// this is the only code that should be modified
// y = latest measured distance in cm
////////////////////////////////////////////////////////////
byte getTrainPWM(double y)
{
  double train_control_pct;
  byte train_control_pwm;

  if (y < 5.0)
  {
    train_control_pct = 0.0;  // emergency stop!
  }
  else
  {
    train_control_pct = 40.0;  // modify this to include feedback
  }
  train_control_pwm = 2.55 * train_control_pct; // modify this to scale percent into PWM byte  
  return train_control_pwm;
}
////////////////////////////////////////////////////////////
// end of train control law
////////////////////////////////////////////////////////////

