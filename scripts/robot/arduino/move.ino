#include <Arduino.h>
#include <Servo.h> 
#include <ros.h>
#include <std_msgs/String.h>

Servo rightServo; // Max-Backward:   0, Stopped: 90, Max-Forward:180 
Servo leftServo;  // Max-Backward: 180, Stopped: 90, Max-Forward:0 

void servoControl(String direction){

  if(direction == "F"){       // Move forward
      rightServo.write(100);
      leftServo.write(80);
      delay(100);
  }
  else if(direction == "FR"){ // Move Right while moving forward +F
      rightServo.write(100);
      leftServo.write(70);
      delay(100);
  }
  else if(direction == "FL"){ // Move Left while moving forward +F
      rightServo.write(110);
      leftServo.write(80);
      delay(100);
  }
  else if(direction == "RT"){ // Rotate Counter-Clockwise
      rightServo.write(180);
      leftServo.write(180);
      delay(100);
  }
  else if(direction == "B"){ // Move Backwards
      rightServo.write(0);
      leftServo.write(180);
      delay(100);
  }
  else if(direction == "R"){ // Turn Right while stopped
      rightServo.write(90);
      leftServo.write(45);
      delay(100);
  }
  else if(direction == "L"){ // Turn Left while stopped
      rightServo.write(135);
      leftServo.write(90);
      delay(100);
  } 
  else{                     // Stop moving
      rightServo.write(90);
      leftServo.write(90);
      delay(100);
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
                                    // Initialize Servos on 2 PWM pins
  rightServo.attach(5);             
  leftServo.attach(11);
}

// Main arduino function
void loop(){
  nh.spinOnce();
  delay(100);
}
