//motor A
#define AFOR1 12 // GO FORWORD
#define ABACK2 11 // GO BACKWORD
#define PWMA 13

//motor B
#define BFOR1 10 // GO FORWORD
#define BBACK2 9 // GO BACKWORD
#define PWMB 8

//SENSORS
#define RA A0
#define LA A1
//SENSORS signal
#define OUTAL 2 // OUTPUT A OF ENCODER L
#define OUTBL 3 // OUTPUT B OF ENCODER L

#define OUTAR 19 // OUTPUT A OF ENCODER R
#define OUTBR 18 // OUTPUT B OF ENCODER R

volatile int impulsesal = 0;   //store position num a left
volatile int impulsesbl = 0;   //store position num b left
volatile int impulsesar = 0;   //store position num a right
volatile int impulsesbr = 0;   //store position num b right

//define boolean direction
#define FORW 1   //go forward
#define BACK 0   //go back

// Motor A
////////////////////////////
volatile byte dira = 0;     // Direction of the Motor
volatile int speeda = 0;     // Speed of the Motor

// Motor B
////////////////////////////
volatile byte dirb = 0;    // Direction of the Motor
volatile int speedb = 0;     // Speed of the Motor

// the setup routine runs once when you press reset:
void setup() {
pinMode(PWMA, OUTPUT);
pinMode(PWMB, OUTPUT);
pinMode(RA, INPUT);
pinMode(LA, INPUT);
pinMode(OUTAL, INPUT);
pinMode(OUTBL, INPUT);
pinMode(OUTAR, INPUT);
pinMode(OUTBR, INPUT);
attachInterrupt(OUTAL, countal1, CHANGE); //attach isr function
attachInterrupt(OUTBL, countal2, CHANGE); //attach isr function
attachInterrupt(OUTAR, countar1, CHANGE); //attach isr function
attachInterrupt(OUTBR, countar2, CHANGE); //attach isr function
Serial.begin(9600);
}

//SET THE LOOP
void loop()
{
  int analr = analogRead(RA); //right sensor
  int anall = analogRead(LA);// left sensor
  
   int centr =calca (analr);
  int centl =calca (anall);
  Serial.print(centr);
  Serial.print(centl);
  delay(2000);
  if ( centl > 10 && centr > 10 )
  {
      //Motor A
  dira = FORW;
  speeda = 64;
  //Motor B
  dirb = FORW;
  speedb = 64;
  MtrA(dira, speeda);
  MtrB(dirb, speedb);  
  }
  else if (  centl < 10 && centr > 10 )
  {
  
  //Motor A
  dira = BACK;
  speeda = 0;
  //Motor B
  dirb = FORW;
  speedb = 64;
  MtrA(dira, speeda);
  MtrB(dirb, speedb); 
  }
    else if (  centl > 10 && centr < 10 )
  {
  
  //Motor A
  dira = FORW;
  speeda = 64;
  //Motor B
  dirb = BACK;
  speedb = 0;
  MtrA(dira, speeda);
  MtrB(dirb, speedb); 
  }
    else
  {
  
  //Motor A
  dira = BACK;
  speeda = 64;
  //Motor B
  dirb = BACK;
  speedb = 64;
  MtrA(dira, speeda);
  MtrB(dirb, speedb); 
  }
}
//function for motor a
void MtrA(boolean mtradir, byte pwma)
{
    if(mtradir){
       digitalWrite(AFOR1, HIGH);
      digitalWrite(ABACK2, LOW);
    analogWrite(PWMA, pwma);
  }else{
     digitalWrite(AFOR1, LOW);
      digitalWrite(ABACK2, HIGH);
    analogWrite(PWMA, pwma);
  }

}

//function for motor b
void MtrB(boolean mtrbdir, byte pwmb)
{
    if(mtrbdir){
      digitalWrite(BFOR1, HIGH);
      digitalWrite(BBACK2, LOW);
    analogWrite(PWMB, pwmb);
  }else{
     digitalWrite(BFOR1, LOW);
      digitalWrite(BBACK2, HIGH);
    analogWrite(PWMB, pwmb);
  }

}



// counterfor out a left encoder
void countal1() {
  //Serial.println("OK!");
  if (dira)
    impulsesal++;
  else
    impulsesal--;
}
// counterfor out b left encoder
void countal2() {
  //Serial.println("OK!");
  if (dira)
    impulsesbl++;
  else
    impulsesbl--;
}
// counterfor out a right encoder
void countar1() {
  //Serial.println("OK!");
  if (dirb)
    impulsesar++;
  else
    impulsesar--;
}
// counterfor out b right encoder
void countar2() {
  //Serial.println("OK!");
  if (dirb)
    impulsesbr++;
  else
    impulsesbr--;
}

float calca ( float a ){
  float a1 = a * 0.0048828125 ;
  float distancer = 29.988 * pow (a1 , (-1.173)) ;
  return distancer;
}


