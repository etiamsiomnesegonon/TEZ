//ROS TOPICS
////////////////////////////
#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/UInt8MultiArray.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Twist.h>
#include <ros/time.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <Adafruit_NeoPixel.h>
#ifdef __AVR__
  #include <avr/power.h>
#endif

//ENCODER FOR MOTORS
///////////////////////////
#include <Encoder.h>
Encoder motorLeft(2, 3);
Encoder motorRight(18, 19);
// MOTORS CONTROL COMMANDS PINS
////////////////////////////
//Motor A
#define PWMA 13   //controls motor speed for A RIGHT
#define AIN2 12  
#define AIN1 11
//Motor B
#define PWMB 8   ///controls motor speed for B LEFT
#define BIN1 10  
#define BIN2 9   
//SENSORS PINS
//////////////////////////
#define RA A0 //RIGHT
#define LA A1 //LEFT
#define SL A2 //LIGHT SENSOR
#define LED 6 //LED

//RUMBLE VARIABLE
/////////////////////////////
volatile bool rumbleYorN(true);

//RGB LED
/////////////////////////////
volatile byte led_R=0;
volatile byte led_G=0;
volatile byte led_B=0;
Adafruit_NeoPixel ourLED = Adafruit_NeoPixel(1, LED, NEO_GRBW + NEO_KHZ800);  //TO POPULATE ARGUMENTS
  
//DIRECTIONS AND SPEED
////////////////////////////
volatile int stateR=HIGH;
volatile int stateL=HIGH;
volatile double desiredSpeedLeft(0.0);        // Speed of the Motor in pulses per second
volatile double desiredSpeedRight(0.0);       // Speed of the Motor in pulses per second

volatile double realVelocity(0.0);       //real linear velocity measured by Arduino
volatile double realOmega(0.0);          //real angular velocity measured by Arduino
volatile double desiredLinearVel(0.0);   //desired linear velocity sent by RPi3
volatile double desiredAngularVel(0.0);  //desired angular velocity sent by RPi3

//READINGS TIME AND # PULSES FROM ARDUINO
/////////////////////////////////////////
volatile int currentMotorReadingRight(0);  
volatile int currentMotorReadingLeft(0);
volatile int previousMotorReadingRight(0);
volatile int previousMotorReadingLeft(0);
volatile double currentTime(0.0);
volatile double previousTime(0.0);

//ROBOT PARAMETERS
//////////////////////////////
volatile double radius(0.016);              //Radius of wheel in m
volatile double C(3575.04);                 //resolution pulses per one revolution of the wheel
volatile double D=2*M_PI*radius/C;          //linear wheel displacement per pulse in m/pulse

volatile double b(0.082);                   //distance between wheels TO BE MEASURED
volatile double KpR(0.0531);                //PWMA gain for Right motor TO BE MEASURED BY STEPWISE FCT????
volatile double KpL(0.0489);                //PWMB gain for Left motor TO BE MEASURED

//volatile double KpR(0.08);                  //PWMA gain for Right motor FOR PID MEASUREMENTS TO BE DELETED EVENTUALLY
//volatile double KpL(0.08);                  //PWMB gain for Left motor FOR PID MEASUREMENTS TO BE DELETED EVENTUALLY

volatile double positionX(0.0);             //current X position in m ODOMETRY
volatile double positionY(0.0);             //current Y position in m ODOMETRY
volatile double angleTheta(0.0);            //current Theta position in Rad ODOMETRY
volatile double linearDisplacement(0.0);    //average displacemenent of the 2 wheels between deltaTime
volatile double deltaTheta(0.0);            //angular displacement between deltaTime

//ROS NODEHANDLE
/////////////////////////////////////
ros::NodeHandle nh;

//ROS PUBLISHER NODES DEFINITION
/////////////////////////////////////
std_msgs::UInt16 ir_left_msg,ir_right_msg, ldr_msg ; //Distance readings from the sensors in cm
ros::Publisher ir_left("groupF/IR_left", &ir_left_msg),ir_right("groupF/IR_right", &ir_right_msg),ldr("groupF/LDR", &ldr_msg) ;
geometry_msgs::Pose2D pose_data;                     //Pose measured in m and Rad
ros::Publisher pose("groupF/Pose",&pose_data);

//ROS SUBSCRIBER NODES DEFINITION
/////////////////////////////////////
void callback_rgbled( const std_msgs::UInt8MultiArray& rgb_led){
  led_R=rgb_led.data[0];
  led_G=rgb_led.data[1];
  led_B=rgb_led.data[2];
}
ros::Subscriber<std_msgs::UInt8MultiArray> rgb("groupF/rgb_led", &callback_rgbled );
//====================================================================================================
void callback_cmdvel( const geometry_msgs::Twist& velocity_cmd){
    desiredLinearVel=velocity_cmd.linear.x;
    desiredAngularVel=velocity_cmd.angular.z;
}
ros::Subscriber<geometry_msgs::Twist> cmdvel("groupF/velocity_cmd", &callback_cmdvel );
//====================================================================================================
void callback_intialpose( const geometry_msgs::Pose2D& initial_pose){
  positionX=initial_pose.x;      
  positionY=initial_pose.y;      
  angleTheta=initial_pose.theta;    
}
ros::Subscriber<geometry_msgs::Pose2D> initialpose("groupF/initial_pose", &callback_intialpose );
//====================================================================================================
void callback_letsrumble( const std_msgs::Bool& lets_rumble){rumbleYorN=lets_rumble.data;}
ros::Subscriber<std_msgs::Bool> letsrumble("lets_rumble", &callback_letsrumble );

//ROS tf BROADCASTER
////////////////////////////////////
geometry_msgs::TransformStamped t;
tf::TransformBroadcaster broadcaster;

//ROS COMMUNICATION SYNCHRONISATION
/////////////////////////////////////
volatile unsigned long rosSync=0;
volatile unsigned int pub_frequency=200;  // in ms   |  5hz = 200ms

//////////////////////////////////////////////////////////////////////////////////////////////////////
//                        SETUP
/////////////////////////////////////////////////////////////////////////////////////////////////////
void setup() {
  pinMode(PWMA, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(AIN1, OUTPUT);
  pinMode(PWMB, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(RA, INPUT);
  pinMode(LA, INPUT);
  pinMode(SL, INPUT);
  pinMode(LED, OUTPUT);

  Serial.begin(9600);           //if you want to retrieve data as printf SHIFT+CTRL+M
  
  nh.initNode();                //intialise ROS node
  nh.advertise(ir_left);        //to ROSCORE
  nh.advertise(ir_right);
  nh.advertise(ldr);
  nh.advertise(pose);
  
  nh.subscribe(rgb);            //From RPi3
  nh.subscribe(cmdvel);
  nh.subscribe(initialpose);
  nh.subscribe(letsrumble);
  
  broadcaster.init(nh);         //tf intialised
  
  ourLED.begin();
}
//////////LOOP///////////////////
void loop() {
  
if(rumbleYorN){
  //volatile double LinearVel=5;  //FOR TEST PURPOSE ONLY TO DELETE EVENTUALLY
  //volatile double AngularVel=0; //FOR TEST PURPOSE ONLY TO DELETE EVENTUALLY
  //speedMotors(LinearVel, AngularVel); //TO DELETE
  
  ourLED.setPixelColor(1, ourLED.Color(led_R,led_G,led_B)); // Moderately bright green color.
  ourLED.show(); // This sends t
  
  speedMotors(desiredLinearVel, desiredAngularVel); //CORRECT FCT TO USE

  currentTime=millis();
  currentMotorReadingRight=motorRight.read();
  currentMotorReadingLeft=motorLeft.read();

  //ir_left_msg.data=distanceMeasured(analogRead(LA));
  //ir_right_msg.data=distanceMeasured(analogRead(RA));
  ir_right_msg.data=distanceMeasured(analogRead(LA));
  ir_left_msg.data=distanceMeasured(analogRead(RA));
  ldr_msg.data=0.48875855*analogRead(SL)/5;          //should be percentage so: 100*0.0048875855*analogRead(SL)/5 5V 1023 analog readings (intervals) 1024 bits
  
    /*Serial.print("Motor Right Reading=; ");          //TO TUNE MOTORS PID CONTROLLERS TO DELETE EVENTUALLY
    Serial.print(motorRight.read());
    Serial.print(" ;");
    Serial.print(" Motor Left Reading=;");
    Serial.print(motorLeft.read()); 
    Serial.print(" ;");
    Serial.print(" time=;"); 
    Serial.print(currentTime);
    Serial.println();*/
    
  //============================
  //update Odometry in m and Rad
  linearDisplacement=averageRobotDisplacement(currentMotorReadingLeft-previousMotorReadingLeft, currentMotorReadingRight-previousMotorReadingRight);
  deltaTheta=changeInAngle(currentMotorReadingLeft-previousMotorReadingLeft, currentMotorReadingRight-previousMotorReadingRight);
  
  positionX=positionX+cos(angleTheta)*linearDisplacement; 
  positionY=positionY+sin(angleTheta)*linearDisplacement;      
  angleTheta=angleTheta+deltaTheta;
  angleTheta=atan2(sin(angleTheta),cos(angleTheta));
  
  //publishes current Odometry in m
  pose_data.x=positionX;
  pose_data.y=positionY;
  pose_data.theta=angleTheta;
  
  t.header.frame_id = "/groupF/odom";
  t.child_frame_id = "/groupF/base_link";
  t.transform.translation.x = positionX; // in meters | Pose x calculated in odometry expressions 
  t.transform.translation.y = positionY; // in meters | Pose y calculated in odometry expressions
  t.transform.rotation = tf::createQuaternionFromYaw(angleTheta);// createQuaternionFromYaw() convert radians to quaternions | theta in radians | orientation calculated in odometry expressions
  t.header.stamp = nh.now();  // Just to get the time of the sample
  broadcaster.sendTransform(t);
  
  //=======================================================
  //check real velocities optional in m and Rad per seconds
  realVelocity=abs(linearDisplacement)/(currentTime-previousTime);
  realOmega=deltaTheta/(currentTime-previousTime);
  
  //=======================================================
  //update intermediary variables
  previousMotorReadingRight=currentMotorReadingRight;
  previousMotorReadingLeft=currentMotorReadingLeft;
  previousTime=currentTime;
  
} //ALL INSIDE THE RUMBLE FLAG IF STATEMENT
else{speedMotors(0, 0); //CORRECT FCT TO USE
}

  //=======================================================
  if(millis()-rosSync > pub_frequency){  // To publish at 5Hz, ie every 200ms actually 205ms
    rosSync=millis();
    
    ir_left.publish( &ir_left_msg );                    //Publish in cm distanceMeasured
    ir_right.publish( &ir_right_msg );                  //Publish in cm 
    ldr.publish( &ldr_msg );
    pose.publish( &pose_data );
    
    nh.spinOnce();   //ros::spinOnce() handles all ROS communication callbacks no need for additional processing in the loop() passes messages to the subscriber callback. 
  }
  
}

//==========================================
//MOTOR COMMANDS WITH PROPORTIONAL GAIN Kp_x
//==========================================
void speedMotors(double desiredLinearVel, double desiredAngularVel){ //desiredSpeed in pulses per seconds moves each wheel based on
//PWM_x = Kp_x * pulses_per_second_x, x = {Left, Right}
    
    desiredSpeedLeft=(desiredLinearVel-b*desiredAngularVel/2)/D;        //CONVERSION INPUT FROM RASPERRY PI TO DESIRED VELOCITY IN PULSES PR SECONDS
    desiredSpeedRight=(desiredLinearVel+b*desiredAngularVel/2)/D;       //CONVERSION INPUT FROM RASPERRY PI TO DESIRED VELOCITY IN PULSES PR SECONDS
    
    /*if(desiredSpeedLeft==0 && desiredSpeedRight==0){
          analogWrite(PWMA, 0);
          digitalWrite(AIN2, false);
          digitalWrite(AIN1, false);
          analogWrite(PWMB, 0);
          digitalWrite(BIN2, false);
          digitalWrite(BIN1, false);
    }*/

    if (desiredSpeedLeft>=0){stateL=HIGH;}else{stateL=LOW;}
    if (desiredSpeedRight>=0){stateR=HIGH;}else{stateR=LOW;}

    analogWrite(PWMA, KpR*abs(desiredSpeedRight));
    digitalWrite(AIN2, stateR);
    digitalWrite(AIN1, !stateR);
    analogWrite(PWMB, KpL*abs(desiredSpeedLeft));
    digitalWrite(BIN2, !stateL);
    digitalWrite(BIN1, stateL);
}

//=================================
//wheel displacement function in m
//=================================
double wheelDisplacement(int numberOfpulses){
 return D*numberOfpulses;
}
//=================================
//average robot diplacement in m
//=================================
double averageRobotDisplacement(int numberOfpulsesLeft, int numberOfpulsesRight){
 return (wheelDisplacement(numberOfpulsesRight)+wheelDisplacement(numberOfpulsesLeft))/2.; 
}
//=================================
//angular displacement in Rad
//=================================
double changeInAngle(int numberOfpulsesLeft, int numberOfpulsesRight){
 return (wheelDisplacement(numberOfpulsesRight)-wheelDisplacement(numberOfpulsesLeft))/b;
}
//====================================================
// read sensor voltage in V and measure distance in cm
//====================================================
double distanceMeasured(int voltage){
  return 29.988 * pow (voltage * 0.0048875855 , (-1.173)) ;                              
}

//===========================================================================================================================
//                      ARCHIVE
//===========================================================================================================================
//=================================
// read sensor and measure distance
//=================================
/*void displaySensorReadings(int delayMilliSec=0){
    Serial.print("Sensor Right Reading=");
    Serial.print(distanceMeasured(analogRead(LA)));
    Serial.print(" Sensor Left Reading=");
    Serial.print(distanceMeasured(analogRead(RA)));
    Serial.println();
    delay(delayMilliSec);
}*/
//===========================================
//Read Motors
//===========================================
/*void displayMotorReadings(String periodDuration){
    Serial.print("Motor Right Reading=;");
    Serial.print(motorRight.read());
    Serial.print(" Motor Left Reading=;");
    Serial.print(motorLeft.read());
    Serial.println(";");
    if (periodDuration="millisec;"){
      Serial.print(millis());
    }
    if(periodDuration="microsec"){
      Serial.print(micros());
    }
    Serial.println();
}
void displayMotorReadings2(double left, double right){
    Serial.print("Motor Right Reading=; ");
    Serial.print(right);
    Serial.print(" ;");
    Serial.print(" Motor Left Reading=;");
    Serial.print(left);
    //Serial.print(" DeltaT ms =; ");
    //Serial.print(time);
    Serial.println();
}*/

/*volatile String forward("forward"); 
volatile String backward("backward"); 
volatile String left("left"); 
volatile String right("right");*/
//=================================
// Set motor direction and speed
//=================================
/*void MotorPWM (String Direction, int Speed) {
  
  if(Direction=="forward"){
    analogWrite(PWMA, Speed);
    digitalWrite(AIN2, HIGH);
    digitalWrite(AIN1, LOW);
    analogWrite(PWMB, Speed);
    digitalWrite(BIN2, LOW);
    digitalWrite(BIN1, HIGH);
  }
  if (Direction=="backward"){
    analogWrite(PWMA, Speed);
    digitalWrite(AIN2, LOW);
    digitalWrite(AIN1, HIGH);
    analogWrite(PWMB, Speed);
    digitalWrite(BIN2, HIGH);
    digitalWrite(BIN1, LOW);
  }
  if (Direction=="right"){   //right turn on the spot around axis of rotation
    analogWrite(PWMA, Speed);
    digitalWrite(AIN2, LOW);
    digitalWrite(AIN1, HIGH);
    analogWrite(PWMB, Speed);
    digitalWrite(BIN2, LOW);
    digitalWrite(BIN1, HIGH);
  }
  if (Direction=="left"){    //left turn on the spot around axis of rotation
    analogWrite(PWMA, Speed);
    digitalWrite(AIN2, HIGH);
    digitalWrite(AIN1, LOW);
    analogWrite(PWMB, Speed);
    digitalWrite(BIN2, HIGH);
    digitalWrite(BIN1, LOW);
  }

}*/
/*void displayReadings(String MotorOrSensor, int leftReading, int rightReading){
    String rightMeasurement=MotorOrSensor+" Right Reading=";
    String leftMeasurement="  "+MotorOrSensor+" Left Reading=";
    
    if (MotorOrSensor="Sensor"){
      Serial.print(rightMeasurement);
      Serial.print(distanceMeasured(rightReading));
      Serial.print(leftMeasurement)
      Serial.print(distanceMeasured(leftReading));
    }
    
    if(MotorOrSensor="Motor"){
      Serial.print(rightMeasurement);
      Serial.print(distanceMeasured(rightReading));
      Serial.print(leftMeasurement)
      Serial.print(distanceMeasured(leftReading));
    }
    
    motorLeft.read(), motorRight.read());
    analogRead(LA), analogRead(RA))
    
    Serial.println("   ");
    Serial.print(millis());
    Serial.println();
}*/







