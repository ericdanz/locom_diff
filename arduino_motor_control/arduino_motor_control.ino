#include "arduino_motor_control.h"
#include "ros.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/Int32.h"
#include <std_msgs/Float64.h>

ros::NodeHandle nh;
int sentSpeed;
float rec;
int spd;
int turnAmount;
long timecheck = 0;

int spdMotor1;
int spdMotor2;

long encoder1;
long encoder2;

long last_encoder1;

long encoder1target;
long encoder2target;
int move_type;

int distanceForward = 0;
int degreesTurn = 0;

int motor_selector = 1;

float mpe = 0.8722;
float turnRadiusConstant = 1;

geometry_msgs::Twist reported_velocity;
ros::Publisher velocityReporter("velocity_reporter", &reported_velocity);

void modeMessageHandler(const std_msgs::Int32& locomotion_mode) {
  current_mode = locomotion_mode.data;
}

ros::Subscriber<std_msgs::Int32> modeSubscription("locomotion_mode", modeMessageHandler);


void velocityMessageHandler(const geometry_msgs::Twist& cmd_vel) {

  spd = cmd_vel.linear.x;
  if (spd != 128)
  {
    spd = 255 - spd;
  }
  turnAmount = cmd_vel.angular.z;
  timecheck = millis();

}

ros::Subscriber<geometry_msgs::Twist> velocitySubscription("cmd_vel_translated", velocityMessageHandler);



void distMessageHandler(const geometry_msgs::Twist& cmd_dist) {

  //distanceForward = cmd_dist.linear.x;
  //degreesTurn = cmd_dist.angular.z;
  enc1_PID_target = cmd_dist.linear.x;
  enc2_PID_target = cmd_dist.linear.y;
  timecheck = millis();

}

ros::Subscriber<geometry_msgs::Twist> distSubscription("cmd_dist", distMessageHandler);

geometry_msgs::Twist reported_dist;
ros::Publisher distReporter("dist_reporter", &reported_dist);

void setup() {
  // put your setup code here, to run once:
  pinMode(RX_PIN, INPUT);
  pinMode(TX_PIN, OUTPUT);

  // Begin serial and serial 1, which is how we are talking to the motor driver
  Serial1.begin(38400);
  //Serial1.begin(9600);

  // turn the PID on
  enc1PID.SetMode(AUTOMATIC);
  enc1PID.SetOutputLimits(PID_output_lower, PID_output_upper);
  enc1PID.SetSampleTime(20);

  enc2PID.SetMode(AUTOMATIC);
  enc2PID.SetOutputLimits(PID_output_lower, PID_output_upper);
  enc2PID.SetSampleTime(20);

  // ros stuff
  nh.initNode();
  nh.advertise(velocityReporter);
  nh.subscribe(velocitySubscription);

  nh.advertise(distReporter);
  nh.subscribe(distSubscription);

  nh.subscribe(modeSubscription);
  // Set baud rate to handle Twist messages
  nh.getHardware()->setBaud(115200);

  resetEncoders();
  delay(100);
  spd = 128;
  turnAmount = 128;
  
  setMode(INDIVIDUAL_MOTOR_CONTROL);
  delay(10);
  setSpeed1(128);
  delay(10);
  setSpeed2(128);
  delay(10);
  //setMode(INDIVIDUAL_MOTOR_CONTROL);
  
}


void loop() {
  if (current_mode == VEL_MODE)
  {
    
    int overcome = 0;
    encoder1 = getEncoder(1);
    delay(10);
    long diff;
    //setSpeedBoth(spd);
    //Turn(turnAmount);
    spdMotor1 = spd - (turnAmount - 128);
    spdMotor2 = spd + (turnAmount - 128);
    diff = abs(encoder1 - last_encoder1);
    //delay(50);
    if ((abs(encoder1 - last_encoder1) < 10) && (encoder1 != 0) && (spd > 130 || spd < 126))
    {
      //diff = abs(encoder1 - last_encoder1);
      overcome = 1;

      if (spdMotor1 > 128) {
        setSpeed1(254);
        setSpeed2(254);
        delay(20);
      }
      else
      {
        setSpeed1(1);
        setSpeed2(1);
        delay(20);
      }
    }

    last_encoder1 = encoder1;

    setSpeed1(spdMotor1);
    setSpeed2(spdMotor2);
    delay(10);

    reported_velocity.linear.x = getSpeed(1);
    reported_velocity.linear.y = overcome;
    reported_velocity.linear.z = turnAmount;
    reported_velocity.angular.x = encoder1;
    reported_velocity.angular.y = diff;
    reported_velocity.angular.z = spd;
    velocityReporter.publish(&reported_velocity);


  }
  else if (current_mode == DIST_MODE) {
      if (motor_selector == 1)
    {
      motor_selector = 2;
      enc1PID.Compute();
      enc1_PID_input = getEncoder(1) * mpe;
      delay(10);
      setSpeed1(enc1_PID_output);
      delay(10);
      }
    else
    {
      motor_selector = 1;
      enc2PID.Compute();
      enc2_PID_input = getEncoder(2) * mpe;
      delay(10);
      setSpeed2(enc2_PID_output);  
      delay(10);
    } 
   
    // Report the dist on the ROS publisher
    // Change to forward distance and total turns?
    reported_dist.linear.x = enc1_PID_input ;
    reported_dist.linear.y = enc2_PID_input ;
    reported_dist.linear.z = enc1_PID_target;
    reported_dist.angular.x = enc2_PID_target;
    reported_dist.angular.y = enc1_PID_output;
    reported_dist.angular.z = enc2_PID_output;
    distReporter.publish(&reported_dist);
  }


  if ((millis() - timecheck) > 1000)
  {
    spd = 128;
    turnAmount = 128;
    //setSpeedBoth(spd);
    setSpeed1(spd);
    delay(10);
    setSpeed2(spd);
    //Turn(turnAmount);
  }
  nh.spinOnce();
  delay(10);
}
