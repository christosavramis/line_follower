#include <Arduino.h>
#include <Servo.h> 
#include <ros.h>
#include <std_msgs/String.h>

Servo rightServo;
//Servo leftServo;

void servoControl(String direction){
  
  //String direction = directionMsg.data;

  if(direction == "F"){
      rightServo.write(135);
      //leftServo.write(135);
  }
  else if(direction == "FR"){
      rightServo.write(115);
      //leftServo.write(135);
  }
  else if(direction == "FL"){
      rightServo.write(135);
      //leftServo.write(115);
  }
  else if(direction == "RT"){
      rightServo.write(135);
      //leftServo.write(45);
  }
  else if(direction == "B"){
      rightServo.write(45);
      //leftServo.write(45);
  }
  else if(direction == "R"){
      rightServo.write(90);
      //leftServo.write(135);
  }
  else if(direction == "L"){
      rightServo.write(135);
      //leftServo.write(90);
  }
  else{
      rightServo.write(90);
      //leftServo.write(90);
  }
}


void servoCb(const std_msgs::String& directionMsg){
  //ROS_INFO("Arduino received direction: %s", directionMsg.data);
  Serial.print(directionMsg.data);
  servoControl(directionMsg.data);
}

ros::NodeHandle nh;
ros::Subscriber<std_msgs::String> directionSub("direction", servoCb);


void setup(){
  Serial.begin(57600);
  nh.getHardware()->setBaud(57600);
  
  nh.initNode();
  nh.subscribe(directionSub);

  rightServo.attach(9); 
  ////leftServo.attach(11);
}

void loop(){
  nh.spinOnce();
  delay(1);
}

