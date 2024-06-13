#include "Adafruit_VL53L0X.h"
#include <ros.h>
#include <std_msgs/UInt8.h>

ros::NodeHandle nh;

Adafruit_VL53L0X lox = Adafruit_VL53L0X();

int in3 = 7;                                        // 모터 B연결
int in4 = 6;
int enB = 5;

// Function prototype for motorControl
void motorControl(const std_msgs::UInt8 &msg);

ros::Subscriber<std_msgs::UInt8> sub("motor_control", &motorControl);

void setup() {
  nh.getHardware()->setBaud(115200);  // Set the baud rate

	pinMode(enB, OUTPUT);
	pinMode(in3, OUTPUT);
	pinMode(in4, OUTPUT);
	
	digitalWrite(in3, LOW);
	digitalWrite(in4, LOW);

  nh.initNode();
  nh.subscribe(sub);

  Serial.begin(115200);

    // wait until serial port opens for native USB devices
  while (! Serial) {
    delay(1);
  }

  Serial.println("Adafruit VL53L0X test");
  if (!lox.begin()) {
    Serial.println(F("Failed to boot VL53L0X"));
    while(1);
  }
  // power 
  Serial.println(F("VL53L0X API Simple Ranging example\n\n")); 
}

void loop() {
  VL53L0X_RangingMeasurementData_t measure;

  Serial.print("Reading a measurement... ");
  lox.rangingTest(&measure, false); // pass in 'true' to get debug data printout!

  if (measure.RangeStatus != 4) {  // phase failures have incorrect data
    Serial.print("Distance (mm): "); Serial.println(measure.RangeMilliMeter);
  } else {
    Serial.println(" out of range ");
    delay(100);
  }

  if (75<measure.RangeMilliMeter && measure.RangeMilliMeter<285) {
	  directionControl();                           // 정회전
  }
  else if (67<measure.RangeMilliMeter && measure.RangeMilliMeter<=75){
    nh.spinOnce();
    delay(1);
  }
  else{
    stop();
  }
}

void directionControl() {     //정방향                  
	analogWrite(enB, 180);                       // B모터 속도 최대 설정(0~255)
	digitalWrite(in3, HIGH);                     // B모터 정방향 설정
	digitalWrite(in4, LOW);
}

void motorControl(const std_msgs::UInt8 &msg) { //역방향
  Serial.println("Received");
	analogWrite(enB, 180);                       // B모터 속도 최대 설정(0~255)
	digitalWrite(in3, LOW);                     // B모터 역방향 설정
	digitalWrite(in4, HIGH);
}

void stop() {
	digitalWrite(in3, LOW);                     // B모터 정지
	digitalWrite(in4, LOW);
}