//SW1 PB5 pin 12 direction
//SW2 PB6 pin 13 velocity

//PB1 pin8 S1
//PB2 pin9 S2
//PB3 pin10 S3
//PB4 pin11 S4
volatile int count(3);             //to count cylcle
volatile int changeInDirection(1); //to deal with requested changes in direction

void setup() {
DDRB  = B00001111; //define input pins
}

//x &= ~(1 << n);// forces nth bit of x to be 0.  all other bits left alone.
//x |= (1 << n);// forces nth bit of x to be 1.  all other bits left alone.
void loop(){
  
  if(PB5){
      if(changeInDirection==0){count=3;} //to deal with case !PB5 becomes PB5 reinitialise PORTB
      if (count==3){
      count=0;
      PORTB=B00000001;
    }else{
      PORTB=PORTB<<1;
      ++count;
    }
    changeInDirection=1;
  }else{
    if(changeInDirection==1){count=3;} //to deal with case PB5 becomes !PB5 reinitialise PORTB
    if (count==3){
      count=0;
      PORTB=B00001000; 
    }else{
      PORTB=PORTB>>1;
      ++count;
    }
    changeInDirection=0;
  }
  
  if(PB6){
    delay(100);
  }else{
    delay(1000);
  }
  
}
