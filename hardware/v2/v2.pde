// this program does something, but noone knows what it does nor what it's supposed to do because it's not commented/documented yet. 



/*logobot pins (l33t ardino stuff)
data led         PC0    pin14
status led       PC1    pin15
power led        PC2    pin16
battery check    PC3    pin17
xbee_sleep       PC4    pin18
rx               PD0    pin0
tx               PD1    pin1
servo            PD2    pin2
M1_2             PD3    pin3  
M2_2             PD4    pin4
M1_1             PD5    pin5
M2_1             PD6    pin6
run button       PD7    pin7
connect button   PB0    pin8
ADNS_SDIO        PB1    pin9
ADNS_SCK         PB2    pin10
*/

// the following .h files contain libraries that might be useful or something
  #include "adns_2620.h"
  #include "comm.h"
  #include "dynamics.h"
  #include <avr/wdt.h>  //watchdog timer .h file

// defining variable names to be used in the ensuing control loops
  int rotation;
  int translation;
  int translation_error, old_translation_error, translation_error_derivative;
  int rotation_error, old_rotation_error, rotation_error_derivative;

// the values of translation_target and rotation_target are currently 
// (arbitrary) constants, for debugging & characterization purposes. 
  int translation_target=1000;
  int rotation_target=0;


  int leftMotorSpeed, rightMotorSpeed;

// maxSpeed is set to 100 for ____ reason. 
  int maxSpeed=100;
  
  float translation_p, translation_d;
  float rotation_p, rotation_d;

// l33t arduino stuff
  #define del 1
  #define SDIO 1
  #define SCK 2
  #define BLANK_B 249
  #define BLANK_D 0x87
  #define BLANK_SERVO 251
  #define SERVO 2

  #define UP 700
  #define DOWN 780

  #define ADNS_OUT (1<<SCK) | (1<<SDIO)
  #define ADNS_IN (1<<SCK)

  void ADNS_write(unsigned char address, unsigned char data);
  unsigned char ADNS_read(unsigned char address);
  void update();



  void setup()
  {
  
  // control constants - (there are two loops: translation and rotation) 
  translation_p  = 0.2;
  translation_d  = 0;
  rotation_p     = 0.3;
  rotation_d     = 0;
  
  // 
  old_translation_error=translation_target-translation;
  old_rotation_error=rotation_target-rotation;
  
  // l33t arduino stuff
  pinMode(2,OUTPUT);
  pinMode(3,OUTPUT);
  pinMode(4,OUTPUT);
  pinMode(5,OUTPUT);
  pinMode(6,OUTPUT);
  
  Serial.begin(19200);
}



void loop(){
  
  // update gets the newest x/y mouse sensor readings. 
  // readings accumulate 
    update();
  
  // Will set the derivative constant to 0 for now. 
  
  //translation PD loop
    translation_error=translation_target-translation;
    translation_error_derivative=translation_error-old_translation_error;
    leftMotorSpeed  =(int)(translation_p*(float)translation_error+translation_d*(float)translation_error_derivative);
    rightMotorSpeed =(int)(translation_p*(float)translation_error+translation_d*(float)translation_error_derivative);
    old_translation_error=translation_error;

  //rotation PD loop
    rotation_error=rotation_target-rotation;
    rotation_error_derivative=rotation_error-old_rotation_error;
    leftMotorSpeed+=(int)(rotation_p*(float)rotation_error+rotation_d*(float)rotation_error_derivative);
    rightMotorSpeed-=(int)(rotation_p*(float)rotation_error+rotation_d*(float)rotation_error_derivative);
    old_rotation_error=rotation_error;

  //  motorSpeed=map(abs(abs(error)-margin), 0,abs(target-translation),0,maxSpeed);  //proportional control;

    printStatus();
/*

  if(translation<target-margin)
    forwards(motorSpeed,motorSpeed);
  else if(translation>target+margin)
    backwards(motorSpeed,motorSpeed);
  else
    stop();
*/


  // 
    move(leftMotorSpeed,rightMotorSpeed);   //PD control;
    
    
    if(Serial.available()>0)
     {
       stop();
       wdt_enable(WDTO_15MS);  //reset
       
       while(1)
       {} 
     }
     
}

void printStatus()
{
    Serial.print("translation= ");
    Serial.print(translation,DEC);
    Serial.print(" rotation= ");
    Serial.print(rotation,DEC);
    Serial.print(" rotation error= ");
    Serial.print(rotation_error,DEC);
    Serial.print(" leftMotorSpeed= ");
    Serial.print(leftMotorSpeed,DEC);
    Serial.print(" rightMotorSpeed= ");
    Serial.print(rightMotorSpeed,DEC);
    Serial.println();
}

void forwards(int a, int b)
{
  analogWrite(5,a);
  analogWrite(6,b);
  digitalWrite(3,LOW);
  digitalWrite(4,LOW);
}
void backwards(int a, int b)
{
  analogWrite(5,255-a);
  analogWrite(6,255-b);
  digitalWrite(3,HIGH);
  digitalWrite(4,HIGH);
}



void move(int a, int b)
{
  
  // the following sets a "saturation" for the max and min PWM-values sent to the motor (full throttle).
  // if saturation is being reached here, the "PD control" will certainly not act like a PD controller. 
      if(abs(a)<20)
          a=0;
      if(abs(b)<20)
          b=0;
      if(a>maxSpeed)
          a=maxSpeed;
      if(a<(-1*maxSpeed))
          a=-1*maxSpeed;
      if(b>maxSpeed)
          b=maxSpeed;
      if(b<(-1*maxSpeed))
          b=-1*maxSpeed;


  // WHAT.
  // obviously some motor communicii, but...
      if(a>0)
      {
          digitalWrite(3,LOW);
          analogWrite(5,a);
      }  
      else if(a<0)   //moving backwards
      {
          digitalWrite(3,HIGH); 
          analogWrite(5,map(a,0,-1*maxSpeed,255,255-maxSpeed));  
      }
      else
      {
          digitalWrite(3,LOW);
          analogWrite(5,0);
      }
    
    
    if(b>0)
    {
        digitalWrite(4,LOW);
        analogWrite(6,b);
    }
    else if(b<0)   //moving backwards
    {
        digitalWrite(4,HIGH);
        analogWrite(6,map(b,0,-1*maxSpeed,255,255-maxSpeed));
    }  
    else
    {
        digitalWrite(4,LOW);
        analogWrite(6,0);
    }


}



void stop()
{
  analogWrite(5,0);
  analogWrite(6,0);
  digitalWrite(3,LOW);
  digitalWrite(4,LOW);
}

void update()
{
	char x=ADNS_read(DELTA_X);
	char y=ADNS_read(DELTA_Y);
	translation+=y;
	rotation+=x;
}


void ADNS_write(unsigned char address, unsigned char data)
{
	unsigned char out;
	char bit;
	DDRB=ADNS_OUT;
	out=((0<<SCK) | (1<<SDIO)) | (PORTB & BLANK_B);   //initialize a write
	PORTB=out;
//	pauseus(del);
	out=((1<<SCK) | (1<<SDIO)) | (PORTB & BLANK_B);   //initialize a write
	PORTB=out;
//	pauseus(del);
	for(bit=6;bit>=0;bit--)   //bit-banging out the clock and address byte
	{
		out=((0<<SCK) | (((address & (1<<bit))>>bit)<<SDIO) ) | (PORTB & BLANK_B);   //initialize a write
		PORTB=out;
//		pauseus(del);
		out=((1<<SCK) | (((address & (1<<bit))>>bit)<<SDIO) ) | (PORTB & BLANK_B);   //initialize a write
		PORTB=out;
//		pauseus(del);
	}
	for(bit=7;bit>=0;bit--)   //bit banging out the data byte
	{
		out=((0<<SCK) | (((data & (1<<bit))>>bit)<<SDIO) ) | (PORTB & BLANK_B);   //initialize a write
		PORTB=out;
//		pauseus(del);
		out=((1<<SCK) | (((data & (1<<bit))>>bit)<<SDIO) ) | (PORTB & BLANK_B);   //initialize a write
		PORTB=out;
//		pauseus(del);
	}

}

unsigned char ADNS_read(unsigned char address)
{
	unsigned char out, data;
	data=0;
	char bit;
	DDRB=ADNS_OUT;
	out=((0<<SCK) | (0<<SDIO)) | (PORTB & BLANK_B);   //initialize a write
	PORTB=out;
//	pauseus(del);
	out=((1<<SCK) | (0<<SDIO)) | (PORTB & BLANK_B);   //initialize a write
	PORTB=out;
//	pauseus(del);
	for(bit=6;bit>=0;bit--)   //bit-banging out the clock and address byte
	{
		out=((0<<SCK) | ((address & (1<<bit))>>bit)<<SDIO ) | (PORTB & BLANK_B);   //initialize a write
		PORTB=out;
//		pauseus(del);
		out=((1<<SCK) | ((address & (1<<bit))>>bit)<<SDIO ) | (PORTB & BLANK_B);   //initialize a write
		PORTB=out;
//		pauseus(del);
	}
	DDRB=ADNS_IN;
	PORTB=(1<<SCK) | (PORTB & BLANK_B);  //HI-Zing the SDIO line
	delayMicroseconds(50);
	for(bit=7;bit>=0;bit--)   //bit banging out the data byte
	{
		out=(0<<SCK) | (PORTB & BLANK_B);   
		PORTB=out;
//		pauseus(del);
		out=(1<<SCK) | (PORTB & BLANK_B); 
		data |= (((PINB & (1<<SDIO))>>SDIO)<<bit);
		PORTB=out;
//		pauseus(del);
	}
	PORTC=data;
	return data;
}





