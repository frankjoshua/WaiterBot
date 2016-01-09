#include <PocketBot.h> // https://github.com/frankjoshua/PocketBot
#include <Wire.h>
#include <Adafruit_MotorShield.h> // https://github.com/adafruit/Adafruit_Motor_Shield_V2_Library

/* This will be used to decode messages from the Android device */
PocketBot pocketBot;
/* Allocate space for the decoded message. */
PocketBotMessage message = PocketBotMessage_init_zero;

// Create the motor shield object with the default I2C address
Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_DCMotor *leftMotor = AFMS.getMotor(3);
Adafruit_DCMotor *rightMotor = AFMS.getMotor(4);

/* 1 to 255 Lower this number to slow down the robot */
const int maxSpeed = 255;
/* 1 to 255 Lower this number if the robot is turning too sharp */
const int maxTurnSpeed = maxSpeed / 2;

//Track time between messages
long mLastCommTime = 0;

int leftPower = 0;
int rightPower = 0;
int mLastLeft = 0;
int mLastRight = 0;

boolean mDirection = true;
long mLastHumanSpotted = 0;

void setup() {
  //Start motor shield
  AFMS.begin();  // create with the default frequency 1.6KHz
  //Talk to arduino through USB Serial at 115200 baud
  Serial.begin(115200);
}

void loop() {

  //If too much time has passed kill the motors
  if(millis() > mLastCommTime + 200){
    mLastCommTime += 50;
    //Drop power by 1%
    leftPower = leftPower * .99;
    rightPower = rightPower * .99;
    if(abs(leftPower) < 10){
     leftPower = 0;
    }
    if(abs(rightPower) < 10){
     rightPower = 0;
    }
  }
  
  //Check for PocketBot message
  if(pocketBot.read(Serial, message)){
    /* This code will only be called if a complete message is received*/
    mLastCommTime = millis();
    //Translate the Joystick X, Y (-1.0 <-> 1.0) to motor controler (0 <-> 255, FORWARD, BACKWARD) 
    //Speed
    int throttle = mapfloat(message.control.joy1.Y, -1, 1, -maxSpeed, maxSpeed);
    //Direction
    int dir = mapfloat(message.control.joy1.X, -1, 1, -maxTurnSpeed, maxTurnSpeed);
    //Left and right power
    int powerL = 0;
    int powerR = 0;
    powerL = constrain(throttle - dir, -maxSpeed, maxSpeed);
    powerR = constrain(throttle + dir, -maxSpeed, maxSpeed);
    
    //Check for no manual control
    if(powerL == 0 && powerR == 0){
        int turnSpeed = maxTurnSpeed * .6;
       //If not manual input check for a face to follow
       if(message.face.id != -1){
         if(message.face.id != -1 || millis() - mLastHumanSpotted < 2000){
           mLastHumanSpotted = millis();
         }
         if(message.face.X > 1.1){
            //Turn left
            powerR = -turnSpeed;
            powerL = turnSpeed; 
         } else if(message.face.X < .9){
            //Turn right 
            powerR = turnSpeed;
            powerL = -turnSpeed;
         }
       } else {
          //Automatic mode
          //Turn past destination
//          int heading = message.sensor.heading;
//          int destHeading = message.control.destHeading; 
//          if(mDirection){
//            destHeading += 90;
//            if(destHeading >= 360){
//              destHeading -= 360; 
//            }
//          } else {
//            destHeading -= 90;
//            if(destHeading < 0){
//              destHeading += 360; 
//            }
//          }
//          
//          int diff = destHeading - heading;
//          if(abs(diff) < 15){
//            if(millis() - mLastHumanSpotted > 5000){
//              mLastHumanSpotted = millis();
//              //Start turing the other direction
//              mDirection = !mDirection; 
//            }
//          } 
//          if(getdirection(heading, destHeading) == 0){
//            //Turn left
//            powerR = turnSpeed;
//            powerL = -turnSpeed; 
//          } else {
//            //Turn right 
//            powerR = -turnSpeed;
//            powerL = turnSpeed;
//          }
       }
    }
    
    //Send power values to the motors
    leftPower = powerL;
    rightPower = powerR; 
  } 
  
  driveMotors(leftPower, rightPower);
}

/*
* Translates values of left and right power to what the motor driver understands
* Changes the motor speed slowly to avoid jerking the robot
*/
void driveMotors(int leftPower, int rightPower){
  //If no change just return
  if(mLastLeft == leftPower && mLastRight == rightPower){
     return;
  }
  
  //Save the last values
//  if(leftPower > mLastLeft){
//    mLastLeft++;
//  } else if(leftPower < mLastLeft){
//    mLastLeft--;
//  }
//  if(rightPower > mLastRight){
//    mLastRight++;
//  } else if(rightPower < mLastRight){
//    mLastRight--;
//  }

  mLastLeft = leftPower;
  mLastRight = rightPower;
  
  //Set motors to FORWARD or BACKWARD
    if(mLastLeft < 0){
      leftMotor->run(BACKWARD);
    } else {
      leftMotor->run(FORWARD);
    }
    if(mLastRight < 0){
      rightMotor->run(BACKWARD);
    } else {
      rightMotor->run(FORWARD);
    }
    //Set motor speeds
    rightMotor->setSpeed(abs(mLastRight));
    leftMotor->setSpeed(abs(mLastLeft));
}

/*
* Helper function to map Floats, based on Arduino map()
*/
float mapfloat(float x, float in_min, float in_max, float out_min, float out_max){
 return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

float getdirection( int compassHeading, int heading ){

   int distance1 = 0;

   int readingval = compassHeading;  
   int goalval = heading; 

   int innerdistance =0;
   int outerdistance = 0;

   int innerturndirection = 1;
   int outerturndirection = 0;

   int godirection = 0;//outerturndirection;

distance1 =  goalval - readingval;  

if (distance1 < 0)
  {
  innerdistance = 360 + distance1; 
  innerturndirection = 1;  
  outerturndirection = 0;
  }
else // distance1 > 0
 {
    innerdistance  = distance1; 
    innerturndirection = 1;  // (Right)
    outerturndirection = 0; // (Left)
  }

outerdistance = 360 - innerdistance; 

  if (innerdistance < outerdistance){
     godirection = innerturndirection;  //turn right
  }
 
  // already by default "godirection" = 0 which means turn left
  if (outerdistance < innerdistance){
    godirection = outerturndirection;
  }


  // now to send back the godirection we have chosen
  //send motor drivers the "godirection" where if "godirection" = 0 then turn Left and if "godirection" = 1 turn Right
  return godirection;
}
