// Motors
////////////////////////////
#define PWM1 5   //pin for control motor direction 1
#define PWM2 6   //pin for control motor direction 2

#define FORW 1   //go forward
#define BACK 0   //go back

#define ENABLE 3 // Enable of the Motor Drive

// Encoder
////////////////////////////
#define SIGNAL 2  //encoder interrupt num Pin 2

volatile int impulses = 0;   //store position num

// Motor
////////////////////////////
volatile byte dir = 0;		 // Direction of the Motor
volatile int speed = 0;		 // Speed of the Motor



// the setup routine runs once when you press reset:
void setup() {
  
  // initialize the digital pin Enable as an output.
  pinMode(3, OUTPUT);
  //init the motor driver pins
  pinMode(PWM1, OUTPUT);
  pinMode(PWM2, OUTPUT);
  
  // Interrupt Pin
  pinMode(SIGNAL, INPUT);
  
  // Enable the Motor Drive
  digitalWrite(ENABLE, HIGH);
  
  attachInterrupt(SIGNAL, count, CHANGE); //attach isr function
  
  Serial.begin(9600);
}

// the loop routine runs over and over again forever:
void loop() {
  
  dir = FORW;
  speed = 64;
    
  // Function thar control the motors
  MotorPWM(dir, speed);
  
  // Print the postion of Motor
  Serial.print("impulses: ");
  Serial.println(impulses);
  //delay(10);

}


//=================================
// Set motor voltage
//=================================
void MotorPWM(boolean Mr_Dir, byte Mr_PWM) {
  
  //////////motor////////////////////////
  
  if(Mr_Dir){
  	analogWrite(PWM1, Mr_PWM);
  }else{
    analogWrite(PWM2, Mr_PWM);
  }

}

//=================================
// Interrpts for counting position of the motor
//=================================
void count() {
  //Serial.println("OK!");
  if (dir)
    impulses++;
  else
    impulses--;
}
