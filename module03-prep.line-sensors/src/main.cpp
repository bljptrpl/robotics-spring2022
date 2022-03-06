/*
 * Module 3 -- GET ON TRACK! (edit from module 2)
 */ 

//Fer,AJ,Daniel

#include <Arduino.h>
#include <wpi-32u4-lib.h>

#include <IRdecoder.h>
#include <ir_codes.h>

#include <Chassis.h>// TODO, Section, 4.2: Add line to include Chassis.h

#define LED_YELLOW 13
#define LED_RED 17
#define LED_GREEN 30
 
int baseSpeed = 10; //cm/second

// Sets up the IR receiver/decoder object
const uint8_t IR_DETECTOR_PIN = 1; 
IRDecoder decoder(IR_DETECTOR_PIN); 

//distance between tires first one, angle change third
Chassis chassis(7,1440,13.5);// TODO, Section 6.2: Adjust parameters to better match actual motion




void setLED(bool value) // A helper function for debugging
{
  Serial.println("setLED()");
  digitalWrite(LED_YELLOW, value);
}

// Defines the robot states
enum ROBOT_STATE {ROBOT_IDLE, ROBOT_DRIVE_FOR, ROBOT_LINING};
ROBOT_STATE robotState = ROBOT_IDLE;

void idle(void) // idle() stops the motors
{
  Serial.println("idle()");
  setLED(LOW);
  chassis.idle();  // TODO, Section 4.2: Uncomment call to chassis.idle() to stop the motors
  robotState = ROBOT_IDLE;   //set state to idle
}

void beginLineFollowing(void)
{
  Serial.println("beginLineFollowing()");
  //chassis.setWheelSpeeds(baseSpeed,baseSpeed);
  setLED(HIGH);
  robotState = ROBOT_LINING;
}
void handleLineFollow(int baseSpeed)
{
  Serial.println("handleLineFollow()");

}
/*
 * This is the standard setup function that is called when the board is rebooted
 * It is used to initialize anything that needs to be done once.
 */
void setup() 
{
  // This will initialize the Serial at a baud rate of 115200 for prints
  // Be sure to set your Serial Monitor appropriately
  Serial.begin(115200);

  chassis.init(); // TODO, Section 4.2: Initialize the chassis (which also initializes the motors)
  chassis.setMotorPIDcoeffs(3, 0.3);// TODO, Section 5.1: Adjust the PID coefficients

  idle();

  // Initializes the IR decoder
  decoder.init(); //Question 2

  Serial.println("/setup()");
}

// A helper command to drive a set distance
// At the start, it will take no arguments and we'll hardcode a motion
// TODO, Section 6.1 (but not before!): Edit the function definition to accept a distance and speed
void drive(float distance, float speed)
{
  setLED(HIGH);
  robotState = ROBOT_DRIVE_FOR;
  
  //chassis.setWheelSpeeds(30, 30);// TODO: In Section 4.2 and 5.1, add a call to chassis.setWheelSpeeds() to set the wheel speeds 2
  chassis.driveFor(distance,speed); // TODO: In Section 6.1, remove the call to setWheelSpeeds() and add a call to chassis.driveFor()

}

// A helper function to turn a set angle
void turn(float ang, float speed)
{
  setLED(HIGH);
  robotState = ROBOT_DRIVE_FOR;

  chassis.turnFor(ang,speed);// TODO, Section 6.1: Make a call to chassis.turnFor()

}

// TODO, Section 6.1: Declare function handleMotionComplete(), which calls idle()
void handleMotionComplete()
{
    idle();
}
// Handles a key press on the IR remote
void handleKeyPress(int16_t keyPress)
{
  Serial.println("Key: " + String(keyPress));
       if(keyPress == 9) // TODO, Section 3.2: add "emergency stop"
       {
          idle();
       }

  switch(robotState)
  {
    case ROBOT_IDLE:
      if(keyPress == 5) // TODO, Section 3.2: Handle up arrow button
      {
        Serial.println("Driving Foward");
        drive(80,baseSpeed);
      } 
      else if(keyPress == 13) //back arrow // TODO, Section 6.1: Handle remaining arrows
      {
        Serial.println("Driving Back");
        drive(-50,-(baseSpeed)); //distance, speed
      }
      else if(keyPress == 8) //left arrow
      {
        Serial.println("Turning Left");
        turn(90.0, 40.0);
      } 
     else if(keyPress == 10) //right arrow
      {
        Serial.println("Turning Right");
        turn(-90.0, 40.0);
      } 
      else if(keyPress == 4) //setup
      {
        Serial.println("Robot Lining");
        beginLineFollowing();
      } 
      break;
    case ROBOT_LINING:
      if(keyPress == 0) // VOL - 
      {
        baseSpeed -= 5; 
        Serial.println("baseSpeed - 5 = " + String(baseSpeed));

      } 
      else if(keyPress == 2) // VOL +
      {
         baseSpeed += 5; 
         Serial.println("baseSpeed + 5 = " + String(baseSpeed));
      }
    default:
      break;
  }
}

/*
 * The main loop for the program. The loop function is repeatedly called
 * after setup() is complete.
 */
void loop()
{
  // Checks for a key press on the remote
  int16_t keyPress = decoder.getKeyCode(); // TODO, Section 3.1: Temporarily edit to pass true to getKeyCode()
  if(keyPress >= 0) handleKeyPress(keyPress);

  // A basic state machine
  switch(robotState)
  {
    case ROBOT_DRIVE_FOR: 
        

      // TODO, Section 6.1: Uncomment to handle completed motion
      if(chassis.checkMotionComplete()) handleMotionComplete(); 
      break;
    case ROBOT_LINING:
      handleLineFollow(baseSpeed);
    break;
    default:
      break;
  }
}