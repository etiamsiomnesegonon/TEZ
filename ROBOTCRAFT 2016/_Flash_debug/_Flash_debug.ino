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
volatile double KpR(0.0531);                //PWMA 0.0531
volatile double KpL(0.055);                //PWMB 0.0489

//volatile double KpR(0.08);                  //PWMA gain for Right motor FOR PID MEASUREMENTS TO BE DELETED EVENTUALLY
//volatile double KpL(0.08);                  //PWMB gain for Left motor FOR PID MEASUREMENTS TO BE DELETED EVENTUALLY



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
  
}
//////////LOOP///////////////////
void loop() {
  
  speedMotors(1000, 1000); //CORRECT FCT TO USE LEFT THEN RIGHT

  currentTime=millis();
  currentMotorReadingRight=motorRight.read();
  currentMotorReadingLeft=motorLeft.read();

 
    Serial.print("Motor Right Reading=; ");          //TO TUNE MOTORS PID CONTROLLERS TO DELETE EVENTUALLY
    Serial.print(motorRight.read());
    Serial.print(" ;");
    Serial.print(" Motor Left Reading=;");
    Serial.print(motorLeft.read()); 
    Serial.print(" ;");
    Serial.print(" time=;"); 
    Serial.print(currentTime);
    Serial.println();

}

//==========================================
//MOTOR COMMANDS WITH PROPORTIONAL GAIN Kp_x
//==========================================
void speedMotors(double desiredLinearVel, double desiredAngularVel){ //desiredSpeed in pulses per seconds moves each wheel based on
//PWM_x = Kp_x * pulses_per_second_x, x = {Left, Right}
    
    //desiredSpeedLeft=(desiredLinearVel-b*desiredAngularVel/2)/D;        //CONVERSION INPUT FROM RASPERRY PI TO DESIRED VELOCITY IN PULSES PR SECONDS
    //desiredSpeedRight=(desiredLinearVel+b*desiredAngularVel/2)/D;     //CONVERSION INPUT FROM RASPERRY PI TO DESIRED VELOCITY IN PULSES PR SECONDS
    
    
    desiredSpeedLeft=desiredLinearVel;        //CONVERSION INPUT FROM RASPERRY PI TO DESIRED VELOCITY IN PULSES PR SECONDS
    desiredSpeedRight=desiredAngularVel;
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







