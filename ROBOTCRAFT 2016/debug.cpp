/* 
 * RobotCraft ROS Task 26_28 week - debug example
 *
 ** Changes:
 ** The topic /groupXX/odom will be changed from nav_msgs/Odometry to geometry_msgs/Pose2D
 ** Added the tf (http://wiki.ros.org/tf)
 */

#include <ros.h>
#include <ros/time.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <std_msgs/Bool.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/UInt8MultiArray.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose2D.h>
#include <std_msgs/String.h>                // Just for debug, once the serial port is busy with the ROS serial interface

unsigned long rosSync=0;
unsigned int pub_frequency=200;  // in ms   |  5hz = 200ms

/* Global struct */
struct {
  byte led_R = 0;
  byte led_G = 0;
  byte led_B = 0;
  float pose_x = 0.0;
  float pose_y = 0.0;
  float pose_theta = 0.0;
  float initial_pose_x = 0.0;
  float initial_pose_y = 0.0;
  float initial_pose_theta = 0.0;
  float vel_x = 0.0;
  float vel_w = 0.0;
  boolean rumble_flag = true;
} robot;
  
  
ros::NodeHandle  nh;

/* TF http://wiki.ros.org/tf */
geometry_msgs::TransformStamped t;
tf::TransformBroadcaster broadcaster;

/* Publishers */
std_msgs::String debug_msg;                                     // Just for debug, once the serial port is busy with the ROS serial interface
ros::Publisher pub_debug("/groupXX/arduino_debug", &debug_msg);


void setup()
{
  nh.initNode();

  /* init tf */
  broadcaster.init(nh);
  
  /* init pub */
  nh.advertise(pub_debug);  // Just for debug, once the serial port is busy with the ROS serial interface
}

void loop()
{
 
 if(robot.rumble_flag){     // The flag rumble published from pc/rpi side, will enable or disable the main routine


     /* Your robot routines here */

    // TF, this will be needed for Rviz animation during the final contest. more info http://wiki.ros.org/tf
    t.header.frame_id = "/groupXX/odom";
    t.child_frame_id = "/groupXX/base_link";
    t.transform.translation.x = robot.pose_x; // in meters | Pose x calculated in odometry expressions 
    t.transform.translation.y = robot.pose_y; // in meters | Pose x calculated in odometry expressions
    t.transform.rotation = tf::createQuaternionFromYaw(robot.pose_theta);// createQuaternionFromYaw() convert radians to quaternions | theta in radians | orientation calculated in odometry expressions
    t.header.stamp = nh.now();  // Just to get the time of the sample
 }

 

 if(millis()-rosSync > pub_frequency){  // Just publish only at 5hz, every 200ms
    rosSync=millis();
    
    /* Just for debug, once the serial port is busy with the ROS serial interface */
    char final_array[40];
    String str1 = String (robot.initial_pose_x,2);
    String str2 = String (robot.initial_pose_y,2);
    String str3 = String (robot.initial_pose_theta,2);
    String str4 = String (robot.led_R);
    String str5 = String (robot.led_G);
    String str6 = String (robot.led_B);
    String str7 = String (robot.vel_x,2);
    String str8 = String (robot.vel_w,2);
    String str9 = String (robot.rumble_flag);
    String final_str = str1+','+str2+','+str3+','+str4+','+str5+','+str6+','+str7+','+str8+','+str9;
    final_str.toCharArray(final_array, 40);
    debug_msg.data = final_array;
    pub_debug.publish( &debug_msg );
    /* --------------------------------------------------------------------------- */  
   
    nh.spinOnce();   // update available publish msg to ros pc/rpi side
  }
  
} 
  
  
