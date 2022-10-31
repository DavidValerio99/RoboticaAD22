//Includes required to use Roboclaw library
#include <ros.h> //
#include <geometry_msgs/Twist.h> //Type of message for the cmd_vel topic
#include <std_msgs/Float32.h> //Type of message for the /wl and /wr topics
#include <SoftwareSerial.h>
#include "RoboClaw.h"
#include <DHT11.h>


//CONSTANTS
int pin=2;
int led=3;
DHT11 dht11(pin);
float temp, hum;
#define WHEEL_PUB_PERIOD 100 //Period in [ms] in which we will publish wl and wr
#define WHEEL_DISTANCE 0.6 //Distance between wheels [m]
#define WHEEL_RAD 0.06 //Wheel radius [m]

int motorSpeed = 100;


int motorSpeedL, motorSpeedR = 0;
int counter=0;

unsigned long time_now = 0;
volatile float v, w; //Robot linear and angular speeds [m/s] and [rad/s]
volatile float wr, wl =0; //Wheel speeds wr (right wheel) , wl (left wheel)
ros::NodeHandle nh; //This structure is necessary to work with ROS (init the node, create publishers, etc)

#define addressLeft 0x80
#define addressRight 0x81

//SPEED CONTROLLER
//Velocity PID coefficients (take them from Basicmicro Motion Studio software).
//Driver Left
#define MA_Kp 1.48892
#define MA_Ki 0.23366 
#define MA_Kd 0.00000 
#define MA_qpps 13500
#define MB_Kp 1.34459 
#define MB_Ki 0.26019 
#define MB_Kd 0.00000 
#define MB_qpps 12750
//Driver Right 
#define MC_Kp 1.41957
#define MC_Ki 0.23791 
#define MC_Kd 0.00000 
#define MC_qpps 13312
#define MD_Kp 1.39365 
#define MD_Ki 0.24065 
#define MD_Kd 0.00000 
#define MD_qpps 13500

//Encoder CPR Counts per revolution this comes from the encoder's datasheet. 
#define CPR 4480.0 

//Arduino SoftwareSerial
SoftwareSerial serialLeft(10,11);  
SoftwareSerial serialRight(12,13);  
RoboClaw roboclawLeft(&serialLeft,10000);
RoboClaw roboclawRight(&serialRight,10000);


void cmd_vel_cb( const geometry_msgs::Twist& vel_msg){
 digitalWrite(13, HIGH-digitalRead(13)); // blink the led
 v=vel_msg.linear.x;
 w=vel_msg.angular.z;
}

ros::Subscriber<geometry_msgs::Twist> cmd_vel_sub("cmd_vel", &cmd_vel_cb );
std_msgs::Float32 wl_msg, wr_msg;
ros::Publisher wl_pub("wl", &wl_msg);
ros::Publisher wr_pub("wr", &wr_msg);

//SETUP
void setup() {
  //Open roboclaw serial ports
  roboclawLeft.begin(38400);
  roboclawRight.begin(38400);
  //Serial.begin(9600);

  //Output 
  pinMode(led,OUTPUT);
  
  //Set PID Coefficients
  //Driver Left
  roboclawLeft.SetM1VelocityPID(addressLeft,MA_Kd,MA_Kp,MA_Ki,MA_qpps); 
  roboclawLeft.SetM2VelocityPID(addressLeft,MB_Kd,MB_Kp,MB_Ki,MB_qpps);
  //Driver Right
  roboclawRight.SetM1VelocityPID(addressRight,MC_Kd,MC_Kp,MC_Ki,MC_qpps); 
  roboclawRight.SetM2VelocityPID(addressRight,MD_Kd,MD_Kp,MD_Ki,MD_qpps); 

  //ROS
   nh.initNode();
   nh.subscribe(cmd_vel_sub);
   nh.advertise(wl_pub); //Init the publisher to the /wl topic
   nh.advertise(wr_pub); //Init the publisher to the /wr topic
}


int rpm2qpps(float rpm) 
{ 
  /*** 
   * This funtion receives a desired speed value in RPM (rev/min] and transforms it to qpps 
   * RPM  rev    CPR counts      1 min 
   * --------- * ------------ * ------------ = speed [counts/s] or [qpps] 
   *   min         1  rev          60 s 
  ***/ 
  return (int) rpm*CPR/60.0;  
} 

void temperatura(){
    int err;
    float hum, temp;
    if(err = dht11.read(hum,temp)==0){
      Serial.print("Temperatura: ");
      Serial.print(temp);
      Serial.println();
      if(temp < 25){
          digitalWrite(led,HIGH);
        }
      else 
          digitalWrite(led,LOW);
      }
    }

void manual(int xAxis, int yAxis){
  if (yAxis > 550) {
  int yMapped = /*rpm2qpps(*/map(yAxis, 550, 1023, 0, 100)/*)*/;
  /*roboclawLeft.SpeedM1(addressLeft,rpm2qpps(motorSpeed)); //here the speed is in [rev/min]
  roboclawLeft.SpeedM2(addressLeft,rpm2qpps(motorSpeed)); //here the speed is in [rev/min]
  roboclawRight.SpeedM1(addressLeft,rpm2qpps(motorSpeed)); //here the speed is in [rev/min]
  roboclawRight.SpeedM2(addressLeft,rpm2qpps(motorSpeed)); //here the speed is in [rev/min]*/
  
  roboclawRight.ForwardM1(addressRight,yMapped); 
  roboclawRight.ForwardM2(addressRight,yMapped); 
  roboclawLeft.ForwardM1(addressLeft,yMapped);
  roboclawLeft.ForwardM2(addressLeft,yMapped);
  }
  else if (yAxis < 470) {
  int yMapped = /*rpm2qpps(*/map(yAxis, 470, 0, 0, 100)/*)*/;
  /*roboclawLeft.SpeedM1(addressLeft,-rpm2qpps(motorSpeed)); //here the speed is in [rev/min]
  roboclawLeft.SpeedM2(addressLeft,-rpm2qpps(motorSpeed)); //here the speed is in [rev/min]
  roboclawRight.SpeedM1(addressLeft,-rpm2qpps(motorSpeed)); //here the speed is in [rev/min]
  roboclawRight.SpeedM2(addressLeft,-rpm2qpps(motorSpeed)); //here the speed is in [rev/min]*/
  roboclawRight.BackwardM1(addressRight,yMapped); 
  roboclawRight.BackwardM2(addressRight,yMapped); 
  roboclawLeft.BackwardM1(addressLeft,yMapped);
  roboclawLeft.BackwardM2(addressLeft,yMapped);
  }
  
  else if (xAxis < 470) {
    int xMapped = /*rpm2qpps(*/map(xAxis, 470, 0, 0, 100)/*)*/;
    /*roboclawLeft.SpeedM1(addressLeft,-rpm2qpps(motorSpeed)); //here the speed is in [rev/min]
    roboclawLeft.SpeedM2(addressLeft,-rpm2qpps(motorSpeed)); //here the speed is in [rev/min]
    roboclawRight.SpeedM1(addressLeft,rpm2qpps(motorSpeed)); //here the speed is in [rev/min]
    roboclawRight.SpeedM2(addressLeft,rpm2qpps(motorSpeed)); //here the speed is in [rev/min]*/
    roboclawRight.ForwardM1(addressRight,xMapped);
    roboclawRight.ForwardM2(addressRight,xMapped);
    roboclawLeft.BackwardM1(addressLeft,xMapped);
    roboclawLeft.BackwardM2(addressLeft,xMapped);
  }
  
  else if (xAxis > 550) {
    int xMapped = /*rpm2qpps(*/map(xAxis, 550, 1023, 0, 100)/*)*/;
    /*roboclawLeft.SpeedM1(addressLeft,rpm2qpps(motorSpeed)); //here the speed is in [rev/min]
    roboclawLeft.SpeedM2(addressLeft,rpm2qpps(motorSpeed)); //here the speed is in [rev/min]
    roboclawRight.SpeedM1(addressLeft,-rpm2qpps(motorSpeed)); //here the speed is in [rev/min]
    roboclawRight.SpeedM2(addressLeft,-rpm2qpps(motorSpeed)); //here the speed is in [rev/min]*/
    roboclawRight.BackwardM1(addressRight,xMapped);
    roboclawRight.BackwardM2(addressRight,xMapped);
    roboclawLeft.ForwardM1(addressLeft,xMapped);
    roboclawLeft.ForwardM2(addressLeft,xMapped);
  }

  else{
    /*roboclawLeft.SpeedM1(addressLeft,0); //here the speed is in [rev/min]
    roboclawLeft.SpeedM2(addressLeft,0); //here the speed is in [rev/min]
    roboclawRight.SpeedM1(addressLeft,0); //here the speed is in [rev/min]
    roboclawRight.SpeedM2(addressLeft,0); //here the speed is in [rev/min]*/
    roboclawRight.ForwardM1(addressRight,0); 
    roboclawRight.ForwardM2(addressRight,0);
    roboclawLeft.ForwardM1(addressLeft,0);
    roboclawLeft.ForwardM2(addressLeft,0);
  }
}

void automatico(){
  wr = (2.0 * v + w * WHEEL_DISTANCE) / (2.0 * WHEEL_RAD);
  wl = (2.0 * v - w * WHEEL_DISTANCE) / (2.0 * WHEEL_RAD);
  wl_msg.data = wl; //Add some value to the wl message
  wr_msg.data = wr; //Add some value to the wr message
  motorSpeed = map(abs(wl), 0, 5, 0, 100);
 if((unsigned long)(millis() - time_now) > WHEEL_PUB_PERIOD){
  time_now = millis(); //reset the time
 
  wl_pub.publish( &wl_msg ); //Publish the message to the /wl topic
  wr_pub.publish( &wr_msg ); //Publish the message to the /wr topic
 }
 if (wl < 0 ){
      if (wr < 0){
        /*roboclawLeft.SpeedM1(addressLeft,-rpm2qpps(motorSpeed)); //here the speed is in [rev/min]
        roboclawLeft.SpeedM2(addressLeft,-rpm2qpps(motorSpeed)); //here the speed is in [rev/min]
        roboclawRight.SpeedM1(addressLeft,-rpm2qpps(motorSpeed)); //here the speed is in [rev/min]
        roboclawRight.SpeedM2(addressLeft,-rpm2qpps(motorSpeed)); //here the speed is in [rev/min]*/
        roboclawLeft.BackwardM1(addressLeft,motorSpeed);
        roboclawLeft.BackwardM2(addressLeft,motorSpeed);
        roboclawRight.BackwardM1(addressRight, motorSpeed);
        roboclawRight.BackwardM2(addressRight, motorSpeed);
      }
      else{
        /*roboclawLeft.SpeedM1(addressLeft,-rpm2qpps(motorSpeed)); //here the speed is in [rev/min]
        roboclawLeft.SpeedM2(addressLeft,-rpm2qpps(motorSpeed)); //here the speed is in [rev/min]
        roboclawRight.SpeedM1(addressLeft,rpm2qpps(motorSpeed)); //here the speed is in [rev/min]
        roboclawRight.SpeedM2(addressLeft,rpm2qpps(motorSpeed)); //here the speed is in [rev/min]*/
        roboclawLeft.BackwardM1(addressLeft,motorSpeed);
        roboclawLeft.BackwardM2(addressLeft,motorSpeed);
        roboclawRight.ForwardM1(addressRight, motorSpeed);
        roboclawRight.ForwardM2(addressRight, motorSpeed);
      }
    }
    else {
      if (wr < 0){
        /*roboclawLeft.SpeedM1(addressLeft,rpm2qpps(motorSpeed)); //here the speed is in [rev/min]
        roboclawLeft.SpeedM2(addressLeft,rpm2qpps(motorSpeed)); //here the speed is in [rev/min]
        roboclawRight.SpeedM1(addressLeft,-rpm2qpps(motorSpeed)); //here the speed is in [rev/min]
        roboclawRight.SpeedM2(addressLeft,-rpm2qpps(motorSpeed)); //here the speed is in [rev/min]*/
        roboclawLeft.ForwardM1(addressLeft,motorSpeed);
        roboclawLeft.ForwardM2(addressLeft,motorSpeed);
        roboclawRight.BackwardM1(addressRight, motorSpeed);
        roboclawRight.BackwardM2(addressRight, motorSpeed);
      }
      else{
        /*roboclawLeft.SpeedM1(addressLeft,rpm2qpps(motorSpeed)); //here the speed is in [rev/min]
        roboclawLeft.SpeedM2(addressLeft,rpm2qpps(motorSpeed)); //here the speed is in [rev/min]
        roboclawRight.SpeedM1(addressLeft,rpm2qpps(motorSpeed)); //here the speed is in [rev/min]
        roboclawRight.SpeedM2(addressLeft,rpm2qpps(motorSpeed)); //here the speed is in [rev/min]*/
        roboclawLeft.ForwardM1(addressLeft,motorSpeed);
        roboclawLeft.ForwardM2(addressLeft,motorSpeed);
        roboclawRight.ForwardM1(addressRight, motorSpeed);
        roboclawRight.ForwardM2(addressRight, motorSpeed);
      }
    }
 
 // Esperamos un tiempo para repetir el loop
 /*if (wl < 0){
    motorSpeed = map(abs(wl), 0, 1600, 0, 64);
    roboclawLeft.BackwardM1(addressLeft,motorSpeed);
    roboclawLeft.BackwardM2(addressLeft,motorSpeed);
  }
 else {
    motorSpeed = map(abs(wl), 0, 1600, 0, 64);
    roboclawLeft.ForwardM1(addressLeft,motorSpeed);
    roboclawLeft.ForwardM2(addressLeft,motorSpeed);
 }
 if (wr < 0){
  motorSpeed = map(abs(wr), 0, 1600, 0, 64);
  roboclawRight.BackwardM1(addressRight, motorSpeed);
  roboclawRight.BackwardM2(addressRight, motorSpeed);
 }
 else {
  motorSpeed = map(abs(wr), 0, 1600, 0, 64);
  roboclawRight.ForwardM1(addressRight, motorSpeed);
  roboclawRight.ForwardM2(addressRight, motorSpeed);
 }
 */


 
}

void loop() {
 int xAxis = analogRead(A0);  // Read Joysticks X-axis
 int yAxis = analogRead(A1);  // Read Joysticks Y-axis
 /*wr = (2.0 * v + w * WHEEL_DISTANCE) / (2.0 * WHEEL_RAD);
  wl = (2.0 * v - w * WHEEL_DISTANCE) / (2.0 * WHEEL_RAD);
  wl_msg.data = wl; //Add some value to the wl message
  wr_msg.data = wr; //Add some value to the wr message
 *///dht11.read(hum,temp);
 temperatura();  
 if(xAxis >= 550 || yAxis >= 550 ||  xAxis <=470 || yAxis <= 470){
  manual(xAxis,yAxis);
 }
 else{
  automatico();
  
  
 /*
 if((unsigned long)(millis() - time_now) > WHEEL_PUB_PERIOD){
  time_now = millis();       //reset the time
  wl_pub.publish( &wl_msg ); //Publish the message to the /wl topic
  wr_pub.publish( &wr_msg ); //Publish the message to the /wr topic
 }
    /*motorSpeedL = map((wl), -1600, 1600, 0, 127);
    roboclawLeft.ForwardBackwardM1(addressLeft,motorSpeedL);
    roboclawLeft.ForwardBackwardM2(addressLeft,motorSpeedL);
    motorSpeedR = map((wr), -1600, 1600, 0, 127);
    roboclawRight.ForwardBackwardM1(addressRight,motorSpeedR);
    roboclawRight.ForwardBackwardM2(addressRight,motorSpeedR);
    */
  /*   motorSpeed = map(abs(wl), 0, 1600, 0, 64);

    if (wl < 0 ){
      if (wr < 0){
        roboclawLeft.BackwardM1(addressLeft,motorSpeed);
        roboclawLeft.BackwardM2(addressLeft,motorSpeed);
        roboclawRight.BackwardM1(addressRight, motorSpeed);
        roboclawRight.BackwardM2(addressRight, motorSpeed);
      }
      else{
        roboclawLeft.BackwardM1(addressLeft,motorSpeed);
        roboclawLeft.BackwardM2(addressLeft,motorSpeed);
        roboclawRight.ForwardM1(addressRight, motorSpeed);
        roboclawRight.ForwardM2(addressRight, motorSpeed);
      }
    }
    else {
      if (wr < 0){
        roboclawLeft.ForwardM1(addressLeft,motorSpeed);
        roboclawLeft.ForwardM2(addressLeft,motorSpeed);
        roboclawRight.BackwardM1(addressRight, motorSpeed);
        roboclawRight.BackwardM2(addressRight, motorSpeed);
      }
      else{
        roboclawLeft.ForwardM1(addressLeft,motorSpeed);
        roboclawLeft.ForwardM2(addressLeft,motorSpeed);
        roboclawRight.ForwardM1(addressRight, motorSpeed);
        roboclawRight.ForwardM2(addressRight, motorSpeed);
      }
    }*/


    
 }
 
 nh.spinOnce(); //Very, very, very important to execute at least once every cycle.

}
