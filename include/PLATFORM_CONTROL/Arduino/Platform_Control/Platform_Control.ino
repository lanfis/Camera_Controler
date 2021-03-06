#define PROG_NAME "PLATFORM_CONTROL"
#define vers 1.0

#if (ARDUINO >= 100)
 #include <Arduino.h>
#else
 #include <WProgram.h>
#endif

#define delayTime 20
#define timeOut delayTime*2
#define ROS_SERIAL_BAUD_RATE 57600
#define ROS_RX_PIN 
#define ROS_TX_PIN
/*
#define BRIDGE_SERIAL_BAUD_RATE 9600
#define BRIDGE_RX_PIN 10
#define BRIDGE_TX_PIN 11
*/
//#define PLATFORM_SERVO_BAUD_RATE_1 9600
//#define PLATFORM_SERVO_BAUD_RATE_2 9600
#define PLATFORM_SERVO_PIN_YAW 10
#define PLATFORM_SERVO_PIN_PITCH 11

#define ROS_STRING_SIZE 2
//#define BRIDGE_STRING_SIZE 2


#include <SoftwareSerial.h>
//SoftwareSerial BridgeSerial(BRIDGE_RX_PIN, BRIDGE_TX_PIN);//Rx, TX

#include <Servo.h>
Servo servo_yaw;  // create servo object to control a servo  // twelve servo objects can be created on most boards
Servo servo_pitch;
int servo_ms_yaw = 1500;
int servo_ms_pitch = 1500;
int servo_ms_yaw_new = 1500;
int servo_ms_pitch_new = 1500;
#define SERVO_MS_YAW_MIN  700+10
#define SERVO_MS_YAW_MAX  2300-10
#define SERVO_MS_PITCH_MIN  700+10
#define SERVO_MS_PITCH_MAX  2300-10
#define SERVO_MS_TOR  2//torlerance
#define SERVO_MS_SPEED 10
#define PLATFORM_CONTROL_HEADER "PC"
#define PLATFORM_CONTROL_YAW "Y"
#define PLATFORM_CONTROL_PITCH "P"


#include <ros.h>
#include <std_msgs/String.h>

#define ROS_TOPIC_NAME_SUB "ros_arduino_msgs"
#define ROS_TOPIC_NAME_PUB "arduino_ros_msgs"

ros::NodeHandle  nh;

//General
void stringSplit(String& string_cont, String& split);
void stringMerge(String& string_cont, String& merge_cont);
String  intToString(int num);
String str;
bool stat = false;
char terminator_char = '\n';


String rosToBridgeMsg(String& ros_msg);

//ROS
//String ros_str[ROS_STRING_SIZE];
void rosSubCallback( const std_msgs::String& str_msg);
void rosPub(String& msg);
void rosAck(String& msg);

String ros_topic_name_sub = ROS_TOPIC_NAME_SUB;
String ros_topic_name_pub = ROS_TOPIC_NAME_PUB;
std_msgs::String str_msg;
ros::Subscriber<std_msgs::String> ros_sub(ros_topic_name_sub.c_str(), rosSubCallback);
ros::Publisher ros_pub(ros_topic_name_pub.c_str(), &str_msg);


//BRIDGE
/*
String bridge_str[BRIDGE_STRING_SIZE];
void bridgeWrite(String& bridge_msg);   
bool bridgeRead(String& bridge_msg);
*/

//PLATFORM
bool platform_command_analysis(String& cmd);
void platform_run();


void setup() 
{
  Serial.begin(ROS_SERIAL_BAUD_RATE); 
  Serial.setTimeout(timeOut);
//  BridgeSerial.begin(BRIDGE_SERIAL_BAUD_RATE);
//  BridgeSerial.setTimeout(timeOut);
  servo_yaw.attach(PLATFORM_SERVO_PIN_YAW);
  servo_pitch.attach(PLATFORM_SERVO_PIN_PITCH);
  
  pinMode(13, OUTPUT);
  
  nh.initNode();
  nh.subscribe(ros_sub);
  nh.advertise(ros_pub);
  
  str = PROG_NAME;  str.concat(" On-Line !\n");
  Serial.write(str.c_str());
  rosPub(str);
}

void loop() 
{  
  digitalWrite(13, LOW);
//  if(bridgeRead(str))
//    rosPub(str);
  nh.spinOnce();
  delay(delayTime);
  digitalWrite(13, HIGH);
}


void rosSubCallback( const std_msgs::String& str_msg)
{
  str = str_msg.data;
  //rosAck(str);
  //Program
  platform_command_analysis(str);
  //str = rosToBridgeMsg(str);
//  bridgeWrite(str);
  //Program
}

void rosPub(String& msg)
{
  str_msg.data = msg.c_str();
  ros_pub.publish(&str_msg);
  msg = "";
}

void rosAck(String& msg)
{
  String ack_msg = "ACK ";
  stringMerge(ack_msg, msg);
  rosPub(ack_msg);
}

bool platform_command_analysis(String& cmd)
{
  String content = cmd;
  String split;
  String tmp;
  int ms_yaw = servo_ms_yaw;
  int ms_pitch = servo_ms_pitch;
  if(content.length() < 3) return false;
  while(content.length() > 2)
  {
    stringSplit(content, split);
    String Header = split.substring(0, 2);
    String Option = split.substring(2, 3);
    if(Header == PLATFORM_CONTROL_HEADER)
    {
      if(Option == PLATFORM_CONTROL_YAW)
	    {
  	    tmp = split.substring(3);
		    ms_yaw = (tmp.toInt() < SERVO_MS_YAW_MIN)? servo_ms_yaw : tmp.toInt();
		    ms_yaw = (tmp.toInt() > SERVO_MS_YAW_MAX)? servo_ms_yaw : tmp.toInt();
  	    if(abs(ms_yaw - servo_ms_yaw) > SERVO_MS_TOR)
    	    servo_ms_yaw_new = ms_yaw;
      }
        
      if(Option == PLATFORM_CONTROL_PITCH)
	    {
  	    tmp = split.substring(3);
		    ms_pitch = (tmp.toInt() < SERVO_MS_PITCH_MIN)? servo_ms_pitch : tmp.toInt();
		    ms_pitch = (tmp.toInt() > SERVO_MS_PITCH_MAX)? servo_ms_pitch : tmp.toInt();
	      if(abs(ms_pitch - servo_ms_pitch) > SERVO_MS_TOR)
      	  servo_ms_pitch_new = ms_pitch;
      }
    }
    //rosPub(tmp);
  }
  platform_run();
  return true;
}

void platform_run()
{
  bool flag_yaw_reach = (servo_ms_yaw == servo_ms_yaw_new)? true : false;
  bool flag_pitch_reach = (servo_ms_pitch == servo_ms_pitch_new)? true : false;
  while(!(flag_yaw_reach & flag_pitch_reach))
  {
    digitalWrite(13, LOW);
    if(servo_ms_yaw_new > servo_ms_yaw)
      servo_ms_yaw = (servo_ms_yaw + SERVO_MS_SPEED) > servo_ms_yaw_new ? servo_ms_yaw_new : (servo_ms_yaw + SERVO_MS_SPEED);
    else if(servo_ms_yaw_new < servo_ms_yaw)
      servo_ms_yaw = (servo_ms_yaw - SERVO_MS_SPEED) < servo_ms_yaw_new ? servo_ms_yaw_new : (servo_ms_yaw - SERVO_MS_SPEED);
    else
      flag_yaw_reach = true;
      
    if(servo_ms_pitch_new > servo_ms_pitch)
      servo_ms_pitch = (servo_ms_pitch + SERVO_MS_SPEED) > servo_ms_pitch_new ? servo_ms_pitch_new : (servo_ms_pitch + SERVO_MS_SPEED);
    else if(servo_ms_pitch_new < servo_ms_pitch)
      servo_ms_pitch = (servo_ms_pitch - SERVO_MS_SPEED) < servo_ms_pitch_new ? servo_ms_pitch_new : (servo_ms_pitch - SERVO_MS_SPEED);
    else
      flag_pitch_reach = true;
    
    servo_yaw.writeMicroseconds(servo_ms_yaw);
    servo_pitch.writeMicroseconds(servo_ms_pitch);
    delay(delayTime/2);
    digitalWrite(13, HIGH);
  }
  return;
}
/*
void bridgeWrite(String& bridge_msg)   
{
  digitalWrite(13, HIGH);
  bridge_msg = bridge_msg + "\n";
  BridgeSerial.write(bridge_msg.c_str());
  //delay(delayTime);
}

bool bridgeRead(String& bridge_msg)
{
  if(BridgeSerial.available() > 0)
  {
    digitalWrite(13, HIGH);
    bridge_msg = BridgeSerial.readStringUntil(terminator_char);
    return true;
  }
  return false;
}
*/
void stringSplit(String& string_cont, String& split)
{
  string_cont.trim();
  int i;
  for(i = 0; i < string_cont.length(); i++)
  {
    if(isSpace(string_cont[i]))
      break;
  }

  split = string_cont.substring(0, i);
  string_cont = string_cont.substring(i, string_cont.length());
  string_cont.trim();
  return split.trim();
}

void stringMerge(String& string_cont, String& merge_cont)
{
  string_cont.concat(merge_cont);
}

String  intToString(int num)
{
    String str = "";
    switch (num)
    {
        case 0:
          str = "0";
          return str;
        case 1:
          str = "1";
          return str;
        case 2:
          str = "2";
          return str;
        case 3:
          str = "3";
          return str;
        case 4:
          str = "4";
          return str;
        case 5:
          str = "5";
          return str;
        case 6:
          str = "6";
          return str;
        case 7:
          str = "7";
          return str;
        case 8:
          str = "8";
          return str;
        case 9:
          str = "9";
          return str;
    }
    return str;
}
    

/////////////////////////////////////////////

