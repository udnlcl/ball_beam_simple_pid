
/*
  Ball & Beam - regulate a ball's position using inbuilt PID controller
  
  The sketch is written for Quanser's Ball & Beam apparatus
  (https://www.quanser.com/products/ball-and-beam/)
  

  Connections:

  Input IN1: Motor's position - A0 
    The Quanser servo houses two kinds of encoder. This sketch uses the potentiometer
    as, an absolute, encoder, which is a potentiometer. The  jockey for this 
    "potentiometer setup" is connected to A0.
    
  Input IN2: Ball's position - A1 
    The beam creates a potential gradient, and the ball acts as a jockey for the 
    this "potentiometer setup". The output voltage is fed to A1.


  Output OP1: Motor 

    OP1.1 DIR pin --> #6
    The motor driver requiers a digital value to define the direction of its rotation. 

    OP1.2 PWM pin --> #7
    The motor driver requiers a pwm value to define the amount of current fed to it.


  Ref: Please see the circuit diagram for more details.

  -------------------------------------------------------------------------------
  
  This example code is in the public domain. 
  University of Dayton.

  modified 05 Feb 2018
  by Manoj Sharma.
  imanoj_sharma@yahoo.com
  
  https://twitter.com/irahulone

  -------------------------------------------------------------------------------
 
*/


// ------------ Physical connections -----------------------------
const int motorPos_pin = A0;     // Input IN1 (see above)                             
const int ballPos_pin = A1;      // Input IN2 (see above)
 
const int dirPin = 6;            // Output OP1.1 (see above)
const int pwmPin = 7;            // Output OP1.2 (see above)

// ------------ End ----------------------------------------------


// The servo rotation must be limited to a certain range;
// here the lower lim is set to 340, and upper lim to 460. 
const int motor_lim[] = { 340, 460 };


#include<PID_v1.h>    


int motorPos = 0;
int servoPos = 0;
int ballPos = 0;
int targetPos = 200;
 
 
float Kp = 0.76;                  // Initial Proportional Gain.
float Ki = 0;                     // Initial Integral Gain.
float Kd = 0.57;                  // Intitial Derivative Gain.

double Setpoint, Input, Output, ServoOutput;                                       

//Initialize PID object, which is in the class PID.
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);  

void setup() 
{
  Serial.begin(57600);
  pinMode(dirPin,OUTPUT);
  pinMode(pwmPin,OUTPUT);
  analogWrite(pwmPin,0);
  Serial.begin(9600);                                             
  
  Input = ballPos;                      // Calls function readPosition() and sets the balls
                                        // position as the input to the PID algorithm.
                                       
  myPID.SetMode(AUTOMATIC);             // Set PID object myPID to AUTOMATIC. 
  myPID.SetOutputLimits(0,120);         // Set Output limits to -80 and 80 degrees. (Experimentally)
}

void readPos()  // this function updates the positon of the motor, and ball.
{
  // analog read uses a 10 bit ADC
  motorPos = analogRead(motorPos_pin);   // 340 - 460, is the range of native ADC output.
  ballPos = analogRead(ballPos_pin)-312; // 314 - 700 is the range of native ADC output.
}

void servo(int angle) // funciton to handle the inner loop to regulate motor position.
{
  int a = angle+340;
  int error = (a - motorPos);
  // -----------------------------------------------------
  int duty = 10*error;
  // -----------------------------------------------------
  int dutyCycle = abs(duty);
  if(dutyCycle > 255)
  dutyCycle = 255;
  
  if(error > 0 && motorPos < motor_lim[1]) // motor upper lim
  {
    digitalWrite(dirPin,1);
    analogWrite(pwmPin,dutyCycle);
  }
  else if(error < 0 && motorPos > motor_lim[0]) // motor lower lim
  {
    digitalWrite(dirPin,0);
    analogWrite(pwmPin,dutyCycle);
  }
  else
  analogWrite(pwmPin,0);
}


void loop()
{
  readPos();
  
  Setpoint = 160;             // reference ball position.
  
  Input = ballPos;                                            
  myPID.Compute();            // computes Output in range of -80 to 80 degrees.
  
  ServoOutput = 0 + Output;   // offset if needed. 
  servo(ServoOutput);         // command the servo.

 
  Serial.print(ballPos); Serial.print("  ");
  Serial.println();
}
      
      




