#include <Arduino.h>
#include <Servo.h> 
#include <ros.h>
#include <std_msgs/String.h>

Servo rightServo; // ↓ Max-Backward: 180, Stopped: 90, ↓ Max-Forward:0 
Servo leftServo;  // ↑ Max-Backward:   0, Stopped: 90, ↑ Max-Forward:180 

void servoControl(String direction){

  if(direction == "F"){       // Move forward
      rightServo.write(40);
      leftServo.write(180);
      delay(10);
  }
  else if(direction == "FR"){ // Lean right while moving forward
      rightServo.write(10);
      leftServo.write(180);
      delay(10);
  }
  else if(direction == "FL"){ // Lean left while moving forward
      rightServo.write(70);
      leftServo.write(180);
      delay(10);
  }  
  else if(direction == "R"){  // Rotate clockwise
      rightServo.write(90);
      leftServo.write(90);
      delay(10);
      rightServo.write(180);
      leftServo.write(172);
      delay(10);
  }
  else if(direction == "L"){  // Rotate counter-clockwise
      rightServo.write(90);
      leftServo.write(90);
      delay(10);
      rightServo.write(42);
      leftServo.write(0);
      delay(10);
  }
  else if(direction == "B"){  // Move backwards
      rightServo.write(170);
      leftServo.write(0);
      delay(10);
  } 
  else{                       // Uknown message, stop moving
      rightServo.write(90);
      leftServo.write(90);
      delay(10);
  }
}

void servoCb(const std_msgs::String& directionMsg){
  servoControl(directionMsg.data);
}

ros::NodeHandle nh;
ros::Subscriber<std_msgs::String> directionSub("direction", servoCb);

// Setup gets called upon boot
void setup(){
  Serial.begin(9600);               // Set Arduino's baud-rate
  nh.getHardware()->setBaud(9600);  // Set Arduino's ROS node baud-rate

  nh.initNode();
  nh.subscribe(directionSub);

  rightServo.attach(5);             // Initialize Servos on 2 PWM pins             
  leftServo.attach(11);
}

// Main arduino function
void loop(){
  nh.spinOnce();
  delay(10);
}
