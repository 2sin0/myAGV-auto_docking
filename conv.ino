#include <ros.h>
#include <std_msgs/UInt8.h>

ros::NodeHandle nh;

int in3 = 7;   // motorB
int in4 = 6;
int enB = 5;

// Function prototype for motorControl
void motorControl(const std_msgs::UInt8 &msg);

ros::Subscriber<std_msgs::UInt8> sub("motor_control", &motorControl);

void setup() {
  nh.getHardware()->setBaud(115200);  // Set the baud rate

  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
  pinMode(enB, OUTPUT);

  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);

  nh.initNode();
  nh.subscribe(sub);

  Serial.begin(115200);  // Initialize serial communication
}

void loop() {
  nh.spinOnce();
  delay(1);
}

void motorControl(const std_msgs::UInt8 &msg) {
   Serial.println("Received");
   analogWrite(enB, 180);   // vel(0~255)
   digitalWrite(in3, HIGH); // direction
   digitalWrite(in4, LOW);
}
