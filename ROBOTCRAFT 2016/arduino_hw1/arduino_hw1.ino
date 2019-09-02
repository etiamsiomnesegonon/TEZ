//pin8=PB1=S1
//pin9=PB2=S2
//pin10=PB3=S3
//pin11=PB4=S4
//pin12=PB5=SW1
//pin13=PB6=SW2
//SW1=1 CW  SW1=0  CCW
//SW2=1 FAST  SW2=0  SLOW
void setup() {
DDRB  = B00001111;
}

void loop() {
  while(PB5 && PB6){
    if (PORTB=B00111000){
    PORTB=B00110001;
    }
    PORTB = PORTB<<1;
    delay(500);
  }
  while(!PB5 && PB6){
    if (PORTB=B00100001){
    PORTB=B00101000;
    }
    PORTB = PORTB>>1;
    delay(500);
  }
  while(PB5 && !PB6){
    if (PORTB=B00011000){
    PORTB=B00010001;
    }
    PORTB = PORTB<<1;
    delay(1000);
  }
  while(!PB5 && !PB6){
    if (PORTB=B00000001){
    PORTB=B00001000;
    }
    PORTB = PORTB>>1;
    delay(1000);
  }
}
