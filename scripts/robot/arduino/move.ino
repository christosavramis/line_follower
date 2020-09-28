#include <Arduino.h>
#include <Servo.h> 
#include <ros.h>
#include <std_msgs/String.h>

Servo rightServo; // ↓ Max-Backward: 180, Stopped: 90, ↓ Max-Forward:0 
Servo leftServo;  // ↑ Max-Backward:   0, Stopped: 90, ↑ Max-Forward:180 

void servoControl(String direction){

  if(direction == "F"){       // Move forward
      rightServo.write(45);
      leftServo.write(135);
      delay(20);
  }
  else if(direction == "FR"){ // Lean right while moving forward
      rightServo.write(55);
      leftServo.write(145);
      delay(20);
  }
  else if(direction == "FL"){ // Lean left while moving forward
      rightServo.write(35);
      leftServo.write(125);
      delay(20);
  }  
  else if(direction == "R"){  // Rotate clockwise
      rightServo.write(90);
      leftServo.write(90);
      delay(20);
      rightServo.write(160);
      leftServo.write(160);
      delay(20);
  }
  else if(direction == "L"){  // Rotate counter-clockwise
      rightServo.write(90);
      leftServo.write(90);
      delay(20);
      rightServo.write(20);
      leftServo.write(20);
      delay(20);
  }
  else if(direction == "B"){  // Move backwards
      rightServo.write(135);
      leftServo.write(45);
      delay(20);
  } 
  else{                       // Uknown message, stop moving
      rightServo.write(90);
      leftServo.write(90);
      delay(20);
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
  delay(20);
}
