/*
 * Module 2 -- Move it!
 */ 

//Fer,AJ,Daniel

#include <Arduino.h>
#include <wpi-32u4-lib.h>

#include <IRdecoder.h>
#include <ir_codes.h>

#include <Chassis.h>// TODO, Section, 4.2: Add line to include Chassis.h

 

// Sets up the IR receiver/decoder object
const uint8_t IR_DETECTOR_PIN = 1; // Question 2 (module 2)
IRDecoder decoder(IR_DETECTOR_PIN); //Question 1 (module 2)

//distance first one, angle change third
Chassis chassis(7,1440,13.5);// TODO, Section 4.2: Declare the chassis object (with default values)

// TODO, Section 6.2: Adjust parameters to better match actual motion

// A helper function for debugging
#define LED_PIN 13
void setLED(bool value)
{
  Serial.println("setLED()");
  digitalWrite(LED_PIN, value);
}

// Defines the robot states
enum ROBOT_STATE {ROBOT_IDLE, ROBOT_DRIVE_FOR};
ROBOT_STATE robotState = ROBOT_IDLE;

// idle() stops the motors
void idle(void)
{
  Serial.println("idle()");
  setLED(LOW);

  // TODO, Section 4.2: Uncomment call to chassis.idle() to stop the motors
  chassis.idle();

  //set state to idle
  robotState = ROBOT_IDLE;
  Serial.println("/idle()");
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
        drive(80,10); //5
      } 
      else if(keyPress == 13) //back arrow // TODO, Section 6.1: Handle remaining arrows
      {
        drive(-50,-10); //distance, speed
      }
      else if(keyPress == 8) //left arrow
      {
        printf("is this working left arrow");
        turn(90.0, 20.0);
      } 
     else if(keyPress == 10) //right arrow
      {
        printf("is this working right arrow");
        turn(-90.0, 20.0);
      } break;
      
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

    default:
      break;
  }
}