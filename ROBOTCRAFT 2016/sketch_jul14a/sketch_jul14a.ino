////////////////////////////
#define S1 8    //pin for controlling motor rotation
#define S2 9    //pin for controlling motor rotation
#define S3 10   //pin for controlling motor rotation
#define S4 11   //pin for controlling motor rotation

#define SW1 12  // read the direction of the motor HIGH = FORWARD LOW = BACKWARD
#define SW2 13  // read the speed of the motor HIGH = FAST LOW = SLOW

int state[4]={1,0,0,0}; //the first state is here to store a variable during the loop
// the setup routine runs once when you press reset:
void setup() {
  pinMode(S1, OUTPUT);
  pinMode(S2, OUTPUT);
  pinMode(S3, OUTPUT);
  pinMode(S4, OUTPUT); 
  pinMode(SW1, INPUT);
  pinMode(SW2, INPUT);
  
  Serial.begin(9600);
}

// the loop routine runs over and over again forever:
void loop() {
  motionMotor(digitalRead(SW1),digitalRead(SW2));
}

void motionMotor(int direct, int velocity){
  
  volatile int vel(velocity*100+(1-velocity)*1000);
 
  volatile int ghoststate=state[direct*3]; //stores value for the loop
  
  for (int i(0);i<=3;++i){                 //for the reverse motion it starts with a delay of vel due to the definition of the vector state
    digitalWrite(S1+i, state[direct*i+(1-direct)*(3-i)]);
    if(i!=0){
        if(i!=3){state[direct*i+(1-direct)*(3-i)]=state[(1-direct)*(3-(i+1))+direct*(i+1)];
      }else{
        state[direct*i+(1-direct)*(3-i)]=state[(1-direct)*i+direct*(3-i)];
      }
    }
    delay(vel);
  }
  
  state[(1-direct)*3]=ghoststate; 
}

