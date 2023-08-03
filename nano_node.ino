/* Header files Include for PWM, ros and Geometry message, Boolean,Integer */
#include <PWM.h>
#include <math.h>
#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int16.h>

#include <SoftwareSerial.h> // Debug serial
SoftwareSerial mySerial(6, 7); // RX, TX
/* Initialize node handle for ros communication */ 
ros::NodeHandle node;
std_msgs::Bool switch_data;// limit switch data publish to break node
ros::Subscriber<geometry_msgs::Twist> sub("/linear/angular", &onTwist);// command velocity subscribe topic creation
ros::Publisher limit_sw("/brake/limit_switch",&switch_data);// creating limit switch publish topic 

void setup() {
  setupPins(); // gpio pins initialize
   mySerial.begin(9600);// baudrate for debug serial communcation
  // Pwm timer initialize
  InitTimersSafe();
  bool success = SetPinFrequencySafe(L_PWM, frequency_left); // set pwm frequency for left side motor
  SetPinFrequencySafe(R_PWM, frequency_right);// set pwm frequency for left side motor

  node.initNode(); // ros node intialize
  node.subscribe(sub); //subcribe joystick value
}

void loop() {
  unsigned long currentMillis=millis(); // measure the time elapsed since the program started 
  node.spinOnce(); // where all of the ROS communication callbacks are handled
/*
  If description - linear value greater than zero will execute below statement
  purpose - While starting vehicle motor has to take the pwm gradually. Based on movement of joystick will 
  increase and decrease speed of vehicle as well */
  
  if(linear_velocity_ref) 
  {
      digitalWrite(LED_BUILTIN, HIGH);
      if(currentMillis-sig_started>=pwm_interval)
      {
        sig_started=currentMillis;
        //digitalWrite(LED_BUILTIN, HIGH);
         if(speed_left<lPwm)
         {
          pwmWrite(L_PWM, speed_left);
          speed_left++;
         }
         if(speed_left>lPwm)
         {
          pwmWrite(L_PWM, speed_left);
          speed_left--;
         }
         if(speed_right<rPwm)
         {
          pwmWrite(R_PWM, speed_right);
          speed_right++;
         }
         if(speed_right>rPwm)
         {
          pwmWrite(R_PWM, speed_right);
          speed_right--;
         }

          node.spinOnce();// where all of the ROS communication callbacks are handled
  
   }
  }
 /* When it is joystick value as zero, The motor speed is zero and motor doesnot run as well */    
   else
   {
    speed_left=25;
    speed_right=25;
    pwmWrite(L_PWM, 0);
    pwmWrite(R_PWM, 0);
    digitalWrite(LED_BUILTIN, LOW);
    }
 /* Read the limit switch status is High to low for changing the state of break condition */
    limit_sw_State = digitalRead(limit_switch);
  /* when limit switch is pressed limit switch state publish to break node
  if(limit_sw_State==HIGH)
    {
      if(limit_flag)
      {
        limit_flag=0;
       switch_data.data = true;
       limit_sw.publish(&switch_data);
      }
    }
    else
    {
      limit_flag=1;
      delay(50);
      }
  
  
    node.spinOnce();// where all of the ROS communication callbacks are handled


}
/* Callback function of Joystick value coming from industrial pc node */
void onTwist(const geometry_msgs::Twist &msg)
{
  /* Assigning global variable to access in loop*/
  linear_velocity_ref  = msg.linear.x;
  angular_velocity_ref = msg.angular.z;
  /* find the left and right motor how much distance should be travel */
  float l = (msg.linear.x - msg.angular.z) / 2;
  float r = (msg.linear.x + msg.angular.z) / 2;
  /* setting the direction of motor */
  digitalWrite(left_relay, l<0);
  digitalWrite(right_relay, r<0);
  
  // Then map those values to PWM intensities. PWMRANGE = full speed, while PWM_MIN = the minimal amount of power at which the motors begin moving.
   lPwm = mapPwm(fabs(l), PWM_MIN, PWMRANGE);
   rPwm = mapPwm(fabs(r), PWM_MIN, PWMRANGE);
  // Debug the linear value
//    mySerial.print("Linear:");
//  mySerial.println(msg.linear.x);

}

   
   
   
/* Set all gpio pins */
void setupPins()
{
   
  pinMode(right_relay, OUTPUT);
  pinMode(left_relay, OUTPUT);
  pinMode(limit_switch, INPUT_PULLUP);
  
}

// Map x value from [0 .. 1] to [out_min .. out_max]
float mapPwm(float x, float out_min, float out_max)
{
  return x * (out_max - out_min) + out_min;
}
