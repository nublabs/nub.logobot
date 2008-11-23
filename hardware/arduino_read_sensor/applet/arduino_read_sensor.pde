/*logobot pins
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

#include "adns_2620.h"
#include "comm.h"
#include "dynamics.h"
#include <avr/wdt.h>  //watchdog timer .h file

byte state, command, parameter1,parameter2,checksum;
int parameter;
int rotation;
int translation;
int translation_error, old_translation_error, translation_error_derivative;
int rotation_error, old_rotation_error, rotation_error_derivative;
int translation_target=0;
int rotation_target=0;
int leftMotorSpeed, rightMotorSpeed;
int maxSpeed=100;
float translation_p, translation_d;
float rotation_p, rotation_d;
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
  translation_p=.2;
  translation_d=.3;
  rotation_p=.3;
  rotation_d=.3;
  old_translation_error=translation_target-translation;
  old_rotation_error=rotation_target-rotation;
  pinMode(2,OUTPUT);
  pinMode(3,OUTPUT);
  pinMode(4,OUTPUT);
  pinMode(5,OUTPUT);
  pinMode(6,OUTPUT);
  state=WAITING;
  Serial.begin(19200);
  changeChannel();
}

void changeChannel()
{
  Serial.print("+++");
  int i,j;
  for(i=0;i<1000;i++)
    for(j=0;j<1000;j++)
      {}
  Serial.println("ATCH0x0C");
  Serial.println("ATWR");
  Serial.println("ATCN");
}

void loop(){
  update();
  
  //translation PD loop
  translation_error=translation_target-translation;
  translation_error_derivative=translation_error-old_translation_error;
  leftMotorSpeed=(int)(translation_p*(float)translation_error+translation_d*(float)translation_error_derivative);
  rightMotorSpeed=(int)(translation_p*(float)translation_error+translation_d*(float)translation_error_derivative);
  old_translation_error=translation_error;

  //rotation PD loop
  rotation_error=rotation_target-rotation;
  rotation_error_derivative=rotation_error-old_rotation_error;
  leftMotorSpeed+=(int)(rotation_p*(float)rotation_error+rotation_d*(float)rotation_error_derivative);
  rightMotorSpeed-=(int)(rotation_p*(float)rotation_error+rotation_d*(float)rotation_error_derivative);
  old_rotation_error=rotation_error;

  //  motorSpeed=map(abs(abs(error)-margin), 0,abs(target-translation),0,maxSpeed);  //proportional control;
    
 // printStatus();  //uncomment this line to send debug statements
/*

  if(translation<target-margin)
    forwards(motorSpeed,motorSpeed);
  else if(translation>target+margin)
    backwards(motorSpeed,motorSpeed);
  else
    stop();
*/
  move(leftMotorSpeed,rightMotorSpeed);   //PD control;
  
  
  //here I'm checking to see if we're getting any packets, and interpreting them as we go
 if(Serial.available()>0)
 {
   byte data=Serial.read();
   switch(state)
   {
     case(WAITING):
       if(data==MSG_START)
         state=COMMAND;
        else
         error("not a message start byte");
        break;
     case(COMMAND):
        if(isValidCommand(data))
        {
          command=data;
          if(isShortPacket(command))
            state=LISTENING_FOR_A_CHECKSUM;
          else
            state=PARAMETER1;            
        }
        else
        {
          error("not a valid command");
          state=WAITING;
        } 
        break;
     case(PARAMETER1):
       parameter1=data;
       state=PARAMETER2;
       break;
     case(PARAMETER2):
       parameter2=data;
       parameter=256*(int)parameter1+(int)parameter2;
       state=LISTENING_FOR_A_CHECKSUM;
       break;
     case(LISTENING_FOR_A_CHECKSUM):
        checksum=data;
        if(checkChecksum())
        {
          state=LISTENING_FOR_THE_END;
        }
        else
        {
          error("invalid checksum");
          state=WAITING;          
        }
        break;
      case(LISTENING_FOR_THE_END):
        if(data==MSG_END) 
        {
          executeCommand();
          state=WAITING;
        }
        else
        {
          error("not a message end byte");
          state=WAITING;
        }
        break;
      default:
      break;
   }
 }
}

//right now, error() just prints out the message it's passed.  In the future, it could do some more sophisticated error handling.  
//just think of it as a useful abstraction for now.
void error(char* message)
{
  Serial.println(message);
}

//this checks against all the valid commands to make sure we've got a valid command.  returns true if the command is valid.
boolean isValidCommand(byte a)
{
  return((a==FORWARD)||(a==BACKWARD)||(a==LEFT)||(a==RIGHT)||(a==STOP)||(a==GET_X)||(a==GET_Y)||(a==GET_BATT)||(a==GET_STATUS));
}

//some packets (generally the GET_* and STOP) packets don't have a parameter.  This checks the command and returns true if it's a 
//command without a parameter
boolean isShortPacket(byte a)
{
  return((a==GET_STATUS)||(a==GET_X)||(a==GET_Y)||(a==GET_BATT)||(a==STOP));
}

//returns true if the received packet's checksum matches up with the one calculated locally
boolean checkChecksum()
{
  if(isShortPacket(command))
    return(checksum==command);
  else
    return(checksum==(command+parameter1+parameter2));
}

//once we get a valid message, we go to this function to figure out what we should do
void executeCommand()
{
  //this might give me guff later on, if I want to ask a status while it's moving.  just keep it in mind
  translation_target=0;
  rotation_target=0;
  translation=0;
  rotation=0;
  
  switch(command){
    case(FORWARD):
      translation_target=parameter;
      rotation_target=0;
      translation=translation_error;
      rotation=rotation_error;
    break;
    case(BACKWARD):
      translation_target=-1*parameter;
      rotation_target=0;
      translation=translation_error;
      rotation=rotation_error;
    break;
    case(LEFT):
      translation_target=0;
      rotation_target=parameter;
      translation=translation_error;
      rotation=rotation_error;
    break;
    case(RIGHT):
      translation_target=0;
      rotation_target=-1*parameter;
      translation=translation_error;
      rotation=rotation_error;
    break;
    case(STOP):
      translation_target=0;
      rotation_target=0;
      translation=0;
      rotation=0;
    break;
    default:
    break;
  }
}

//useful as a debugging function
void printStatus()
{
  Serial.print("translation= ");
  Serial.print(translation,DEC);
  Serial.print(" translation error= ");
  Serial.print(translation_error,DEC);
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





