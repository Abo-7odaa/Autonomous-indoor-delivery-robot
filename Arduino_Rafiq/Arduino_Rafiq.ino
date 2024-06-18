//Libraries
#include <util/atomic.h>  // For the ATOMIC_BLOCK macro
#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <PID_v1.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <ros/time.h>
#include <std_msgs/Int16.h>
//creat nodeHandle object to interface with ROS functionalities.
ros::NodeHandle nh;

//Left Motor Encoder
#define ENCA_LEFT 3
#define ENCB_LEFT 5
//Right Motor Encoder
#define ENCA_RIGHT 2
#define ENCB_RIGHT 4
//motor pins
const int D0 = 10;
const int D1 = 11;
const int D2 = 9;
const int D3 = 6;
//Ultrasonic pins
#define trigPin1 7
#define echoPin1 8
#define trigPin2 A1
#define echoPin2 A2
#define trigPin3 11
#define echoPin3 12
int IR1 = 4;
int IR2 = 13;
int IR3 = A3;
int state1 = 0;
int state2 = 0;
int state3 = 0;
int duration1, distance1;
int duration2, distance2;
int duration3, distance3;


bool CW = true;
bool CCW = false;
bool left_dir = 1;  //left wheel spining direction
volatile long left_prevPos = 0;
volatile long left_pos = 0;
bool right_dir = 1;  //right wheel spining direction
volatile long right_prevPos = 0;
volatile long right_pos = 0;


//volatile int right_wheel_tick_count = 0, left_wheel_tick_count = 0;
std_msgs::Int16 right_wheel_tick_count;
std_msgs::Int16 left_wheel_tick_count;
//initialize ROS publishers for the right and left wheel tick counts.
ros::Publisher rightPub("rwheel", &right_wheel_tick_count);
ros::Publisher leftPub("lwheel", &left_wheel_tick_count);


// PID variables for left wheel
float left_kp = 350;
float left_ki = 130;
float left_vt = 0.0;
float left_eintegral = 0;
float left_wheel_vel = 0.0;

// PID variables for right wheel
float right_kp = 450;
float right_ki = 130;
float right_vt = 0.0;
float right_eintegral = 0;
float right_wheel_vel = 0.0;

// One-second interval for measurements
int interval = 1000;
long previousMillis = 0;
long currentMillis = 0;

float demandx = 0;
float demandz = 0;

float demand_speed_left = 0.0;
float demand_speed_right = 0.0;
// Kinematics calculations
float wheel_radius = 0.065;          // robot's wheel radius (in meters)
float inter_wheel_distance = 0.212;  // inter-wheel distance (in meters)

void cmd_vel_cb(const geometry_msgs::Twist& twist) {
  demandx = twist.linear.x;
  demandz = twist.angular.z;

  // Calculate left and right wheel velocities based on the Twist message
  demand_speed_left = (demandx - (demandz * inter_wheel_distance / 2.0)) / wheel_radius;
  demand_speed_right = (demandx + (demandz * inter_wheel_distance / 2.0)) / wheel_radius;
  left_vt = demand_speed_left;
  right_vt = demand_speed_right;
}
//initializer ROS subsciber for cmd_vel
ros::Subscriber<geometry_msgs::Twist> sub("/cmd_vel", cmd_vel_cb);  //cmd_vel for navigation /diffbot_controller/cmd_vel for teleop



void setup() {
  Serial.begin(9600);
  pinMode(ENCA_LEFT, INPUT);
  pinMode(ENCB_LEFT, INPUT);
  pinMode(ENCA_RIGHT, INPUT);
  pinMode(ENCB_RIGHT, INPUT);
  attachInterrupt(digitalPinToInterrupt(ENCA_LEFT), readLeftEncoder, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCA_RIGHT), readRightEncoder, RISING);
  pinMode(D0, OUTPUT);
  pinMode(D1, OUTPUT);
  pinMode(D2, OUTPUT);
  pinMode(D3, OUTPUT);
  pinMode(trigPin1, OUTPUT);
  pinMode(echoPin1, INPUT);
  pinMode(trigPin2, OUTPUT);
  pinMode(echoPin2, INPUT);
  pinMode(trigPin3, OUTPUT);
  pinMode(echoPin3, INPUT);
  pinMode(IR1, INPUT);
  pinMode(IR2, INPUT);
  pinMode(IR3, INPUT);
  // ROS Setup
  nh.getHardware()->setBaud(115200);
  nh.initNode();
  nh.subscribe(sub);
  nh.advertise(rightPub);
  nh.advertise(leftPub);
}

void loop() {
  // Record the time
  currentMillis = millis();

  // If 1000ms have passed, update velocity
  if (currentMillis - previousMillis > interval) {

    previousMillis = currentMillis;

    rightPub.publish(&right_wheel_tick_count);
    leftPub.publish(&left_wheel_tick_count);
    left_wheel_vel = ((left_pos - left_prevPos) / 2000.0)/.01;  // m/s 2400 ticks per meter & 1000 ticks per revelution
    left_prevPos = left_pos;
    right_wheel_vel = ((right_pos - right_prevPos) / 2800.0)/.01;
    right_prevPos = right_pos;
    
  }
  //left motor PID calculations
  float left_error = left_vt - left_wheel_vel;  // calculate error velocity from target velocity and current velocity
    left_eintegral = left_eintegral + left_error * 1;
    float left_u = left_kp * left_error + left_ki * left_eintegral;
    if (left_vt < 0) {
      left_dir = 0;  //CCW
    } else {
      left_dir = 1;  //CW
    }
    if (left_vt == 0)
    {
        left_u = 0.0;      
    }
    int left_pwr = (int)fabs(left_u);
    if (left_pwr > 255) {
      left_pwr = 255;
    }
    //setMotorLeft(left_dir, left_pwr);

  /* Serial.print("data: ");
  Serial.print(left_wheel_tick_count.data);
  Serial.print("pos: ");
  Serial.print(left_pos);
  Serial.print("prev: ");
  Serial.println(left_prevPos);*/
  /*  
  Serial.print("target: ");
  Serial.print(left_vt);
  Serial.print("left_wheel_vel: ");
  Serial.print(left_wheel_vel);  
  Serial.print("left_error ");
  Serial.print(left_eintegral);   
  Serial.print("left eintegral: ");
  Serial.println(left_eintegral); */
   
  //right motor PID calculations
  if (right_vt < 0) {
    right_dir = 0;  //CCW
  } else {
    right_dir = 1;  //CW
  }
  float right_error = right_vt - right_wheel_vel;  // calculate error velocity from target velocity and current velocity
  right_eintegral = right_eintegral + right_error * 1;
  float right_u = right_kp * right_error + right_ki * right_eintegral;
  
  if(right_vt == 0)
  {
    right_u = 0;
  }
  int right_pwr = (int)fabs(right_u);
  if (right_pwr > 255) {
    right_pwr = 255;
  }
  setMotorRight(right_dir, right_pwr);
  setMotorLeft(left_dir, left_pwr);
  //*/
  //setMotorLeft(1, 160); //I was trying something (check if the odometry calculations is right)
  //setMotorRight(1, 160);
  nh.spinOnce();
}



/****function for test direction****/
void setMotorLeft(bool direction, int pwm) {
  //int pwm = map(speed, 0, 100, 0, 255);
  if (direction == CW) {
    analogWrite(D0, pwm);
    analogWrite(D1, LOW);
  } else if ((direction == CCW)) {
    analogWrite(D1, pwm);
    analogWrite(D0, LOW);
  } else {
    digitalWrite(D1, LOW);
    digitalWrite(D0, LOW);
  }
}

void setMotorRight(bool direction, int pwm) {
  //int pwm = map(speed, 0, 100, 0, 255);
  if (direction == CW) {
    analogWrite(D2, pwm);
    analogWrite(D3, LOW);
  } else if ((direction == CCW)) {
    analogWrite(D3, pwm);
    analogWrite(D2, LOW);
  } else {
    digitalWrite(D2, LOW);
    digitalWrite(D3, LOW);
  }
}



/****function Encoder****/
void readLeftEncoder() {
  int b = digitalRead(ENCB_LEFT);
  if (b > 0) {
    left_wheel_tick_count.data--;
  } else {
    left_wheel_tick_count.data++;
  }
  left_pos = left_wheel_tick_count.data;
}

void readRightEncoder() {
  int b = digitalRead(ENCB_RIGHT);
  if (b > 0) {
    right_wheel_tick_count.data--;
  } else {
    right_wheel_tick_count.data++;
  }
  right_pos = right_wheel_tick_count.data;
}

/****function UltraSonic****/
void UltraSonic1() {
  digitalWrite(trigPin1, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin1, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin1, LOW);
  duration1 = pulseIn(echoPin1, HIGH);
  distance1 = (duration1 / 2);  // 0.0343; // 343 * (100/1000000) = 343/10000
  Serial.println(distance1);
  delay(5);  // wait till next scan
}

void UltraSonic2() {
  digitalWrite(trigPin2, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin2, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin2, LOW);
  duration2 = pulseIn(echoPin2, HIGH);
  distance2 = (duration2 / 2);
  Serial.println(distance2);
  delay(5);
}

void UltraSonic3() {
  digitalWrite(trigPin3, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin3, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin3, LOW);
  duration3 = pulseIn(echoPin3, HIGH);
  distance3 = (duration3 / 2);
  Serial.println(distance3);
  delay(5);
}

/****functions IR****/
void IR_1() {
  state1 = digitalRead(IR1);
  Serial.println(state1);
  delay(10);
}

void IR_2() {
  state2 = digitalRead(IR2);
  Serial.println(state2);
  delay(10);
}

void IR_3() {
  state3 = digitalRead(IR3);
  Serial.println(state3);
  delay(10);
}