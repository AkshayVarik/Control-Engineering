#include <Bridge.h>
#include <Console.h>
#include <FileIO.h>
#include <HttpClient.h>
#include <Mailbox.h>
#include <Process.h>
#include <YunClient.h>
#include <YunServer.h>

//
// Headers contain device drivers
//
#include <SPI.h>  
#include <Pixy.h>
#include <Servo.h>

//
// Pixy is an object type defined in Pixy header file
// Servo is an object type defined in Servo header file
//
Pixy PingPongPixy;
Servo PingPongServo;

//
// here are the pins used
//
const int PingPongServoPin = 3;
const int JoystickPin = A0;
const int ServoDisabledPin = 6;
int sumerror = 0;
float setpoint [5] = {0,-0.30,0.34,-0.60,0.66};
float errorLast = 0;
float Slidecenter;

//
// servo constants -- YOU MAY CHANGE THE CENTER POSITION
//
const int ServoCenter_us = 1278;
const int ServoScale_us = 750;




void setup()
{
//
// Serial baud rate -- faster is usually better
//
  Serial.begin(57600);
//
// Servo attached to Arduino digital pin 3
//
  PingPongServo.attach(3);
  PingPongServo.writeMicroseconds(ServoCenter_us);
//
// initialize the Pixy camera
//
  PingPongPixy.init();
//
// Monitor Servo status as digital input
//
pinMode(ServoDisabledPin,INPUT);
}

void loop()
{ 
  for (int j = 0;j<5;j++)
  {
    Slidecenter = setpoint [j];
    for (int i=0;i< 350;i++)
    {          
      static unsigned long microsLast = 0; // elapsed time at end of last call to loop
      double deltaT = 0.02; // time between digital updates (20ms = PIXY & Servo update rates)
    
      word JoystickRaw = 512;
      double JoystickOne = 0.0;
      static int xBallRawLast = 2;  // start with max left position as indication of failed PIXY read
      int xBallRaw;
      double xBallOne = 0.0;
      double ServoOne = 0.0;
      double t_us;
      float error;
      
      
      boolean ServoDisabled;
    
    //
    // get the joystick position
    // Convert to +/- 1.0 relative to center
    //
      JoystickRaw = analogRead(JoystickPin);
      JoystickOne = -1.0 + (2.0*JoystickRaw)/1023.0;
    //
    // get the ball position
    // negative value indicates no ball found
    // convert to +/- 1.0 relative to camera field of view
    //
      xBallRaw = PingPongCheck();
      if (xBallRaw < 0)
      {
        xBallRaw = xBallRawLast;
      }
      else
      {
        xBallRawLast = xBallRaw;
      }
      xBallOne = -1.0 + (2.0*xBallRaw)/319.0;  // 319 = max pixel value in PIXY camera
    
    //
    // check if the servo is disabled
    // May use this logic in control laws
    // for example, suspend integral feedback if disabled
    //
      ServoDisabled = digitalRead(ServoDisabledPin);
      
    //////////////////////////////////////////////////////////////////
    // Control Law Starts Here
    //////////////////////////////////////////////////////////////////
    
    //  ServoOne = 0.5*JoystickOne;  // pass Joystick command only
    //  ServoOne = 0.1*JoystickOne - 0.1*xBallOne;
      float Ku = 0.5;
      float Kp = 0.7*Ku; //Stabilizes after 10 oscillations
      float tu = 1.5;
      float Kd = Kp*tu/8;
      float Ki = 2*Kp/tu;
      //Slidecenter = JoystickOne/1.5;
      
      
         
      error = Slidecenter - xBallOne ;
      ServoOne = Kp*error + Kd*(error - errorLast)/deltaT + Ki*sumerror;
      errorLast = error;
      sumerror = sumerror + error;
        
        
    
    //////////////////////////////////////////////////////////////////
    // Control Law Ends Here
    //////////////////////////////////////////////////////////////////
    
    //
    // convert ServoOne to MicroSeconds
    // then send the command to the servo
    // smaller values of t_us tilt right (positive sense of ServoOne)
    // DON'T CHANGE LIMITS IN CONSTRAIN FUNCTION -- CAN DAMAGE SERVO
    //  
      if (ServoDisabled)
      {
        t_us = constrain(ServoCenter_us, 1000, 1800);
      }
      else
      {
        t_us = constrain(ServoCenter_us - ServoScale_us * ServoOne, 1000, 1800);
      }
     
      PingPongServo.writeMicroseconds(t_us);
    
    //
    // now send serial information
    //
      Serial.print(JoystickOne,2);
      Serial.print(',');  
      Serial.print(xBallOne,2);
      Serial.print(',');
      Serial.print(Slidecenter,2);
      Serial.print(',');
      Serial.print(i);
      Serial.print(',');
      Serial.print(j);
      Serial.print(',');
      Serial.println(ServoOne,2);
      
    
    //
    // force constant frame rate
    //
      while (micros() < microsLast + deltaT*1000000);  // wait for deltaT since last time through
      microsLast = micros(); // save value for next time through
    }
  }
 }

  
//
// modified from online library
// assumes only 1 block is reported
// that block must be the Ping Pong Ball
//
word PingPongCheck()
{
  int xpos=-1;
  uint16_t nblocks;

  nblocks = PingPongPixy.getBlocks();  
  if (nblocks)
  {
    xpos = PingPongPixy.blocks[0].x;
  }
  
  return xpos;
}

void SimPlot(int data1, int data2, int data3, int data4)
{
  int pktSize;
  int buffer[20];  
  buffer[0] = 0xCDAB;             //SimPlot packet header. Indicates start of data packet
  buffer[1] = 4*sizeof(int);      //Size of data in bytes. Does not include the header and size fields
  buffer[2] = data1;
  buffer[3] = data2;
  buffer[4] = data3;
  buffer[5] = data4;
    
  pktSize = 2 + 2 + (4*sizeof(int)); //Header bytes + size field bytes + data
  
  Serial.write((uint8_t * )buffer, pktSize);
}
