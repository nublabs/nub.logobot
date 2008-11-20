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
#include "adns_2620.h"  //register names and values for the mouse sensor
#include "comm.h"  //commnunicaton codes between the robot and the computer
#include "dynamics.h"  //controls the robot's motion dynamics
#include "pins.h"     //pins specific to the logobot hardware


unsigned int servo=0;
unsigned char servo_running=0;
unsigned int pulse= 2000;
unsigned char state=WAITING;
int rotation;
int translation;
unsigned int parameter1,parameter2,parameter;
unsigned char command;
unsigned char checksum;
unsigned char start, end;
unsigned char message_valid=0;
unsigned char status=WAITING;
unsigned char pen;
unsigned char move=0;
int translation_target=0;
int rotation_target=0;
#define del 1
#define UP 200
#define DOWN 100
#define TRANSLATE 1
#define ROTATE 2
#define PEN 3
#define SDIO 1
#define SCK 2
#define BLANK_B 249
#define BLANK_D 0x87
#define BLANK_SERVO 251
#define SERVO 2

//ported, unverified
void ADNS_write(unsigned char address, unsigned char data);

//ported, unverified
unsigned char ADNS_read(unsigned char address);

//ported, unverified
void forwards(unsigned char a, unsigned char b);
//ported, unverified
void backwards(unsigned char a, unsigned char b);
//ported, unverified
void left(unsigned char a, unsigned char b);
//ported, unverified
void right(unsigned char a, unsigned char b);
//ported, unverified
void stop();

//newly written, untested
void readData();

//ported, unverified
void update();


//ported, unverified
unsigned char check_checksum();
//ported, unverified
unsigned char short_packet(unsigned char a);
//ported, unverified
unsigned char valid_command(unsigned char a);
//ported, unverified
void adns_init();
//ported, unverified
void logobot_init();

//ported, untested
void setup()
{
  adns_init();
  Serial.begin(9600);
  logobot_init();
}

//ported, untested
void loop()  
{
	start=0;
	end=0;

	unsigned char a,b;
	while(1)
	{
                if(Serial.available())   //check to see if we got any data from the computer
                   readData();
                  
		update();    //update posisiton info from mouse sensor

		if(move==TRANSLATE)    //
		{
			if(abs(translation-translation_target)<(2*translation_margin))  //set PWM speeds to slow if we're close to the target
			{
				a=225;
				b=225;
			}
			else
			{
				a=255;
				b=255;
			}
			if(translation>(translation_target+translation_margin))  //set the direction of motion if we're ahead or behind the target
				backwards(a,b);				
			else if(translation<(translation_target-translation_margin))
				forwards(a,b);
			else
				stop();
		}

		else if(move==ROTATE)
		{
			if(abs(rotation-rotation_target)<(2*rotation_margin))
			{
				a=225;
				b=225;
			}
			else
			{
				a=255;
				b=255;
			}
			if(rotation>(rotation_target+rotation_margin))
				left(a,b);				
			else if(rotation<(rotation_target-rotation_margin))
				right(a,b);
			else
				stop();			
		}
		else
			stop();

	}
}

void forwards(unsigned char a, unsigned char b)
{
	analogWrite(M1_1,a);
        analogWrite(M1_2,b);
        digitalWrite(M1_2,LOW);
        digitalWrite(M2_2,LOW);

}

void backwards(unsigned char a, unsigned char b)
{
	analogWrite(M1_1,50-a);
        analogWrite(M1_2,50-b);
        digitalWrite(M2_1,HIGH);
        digitalWrite(M2_2,HIGH);
}

void right(unsigned char a, unsigned char b)
{
	analogWrite(M1_1,50-a);
        analogWrite(M1_2,b);
        digitalWrite(M2_1,HIGH);
        digitalWrite(M2_2,LOW);


}

void left(unsigned char a, unsigned char b)
{
	analogWrite(M1_1,a);
        analogWrite(M1_2,50-b);
        digitalWrite(M2_1,LOW);
        digitalWrite(M2_2,HIGH);
}

void stop()
{
	digitalWrite(M1_1,LOW);
        digitalWrite(M1_2,LOW);
        digitalWrite(M2_1,LOW);
        digitalWrite(M2_2,LOW);
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
	pinMode(SDIO,OUTPUT);
      //initialize a write
        digitalWrite(SCK,LOW);
        digitalWrite(SDIO,HIGH);

//pause?
        digitalWrite(SCK,HIGH);
        digitalWrite(SDIO,HIGH);
                                    
      //end init
      
	for(bit=6;bit>=0;bit--)   //bit-banging out the clock and address byte
	{
                digitalWrite(SDIO,((address & (1<<bit))>>bit));
                digitalWrite(SCK,LOW);
  //pause
                digitalWrite(SDIO,((address & (1<<bit))>>bit));
                digitalWrite(SCK,HIGH);
	}
	for(bit=7;bit>=0;bit--)   //bit banging out the data byte
	{
                digitalWrite(SDIO,((data & (1<<bit))>>bit));
                digitalWrite(SCK,LOW);
//pause?
                digitalWrite(SDIO,((data & (1<<bit))>>bit));
                digitalWrite(SCK,LOW);
	}
}

unsigned char ADNS_read(unsigned char address)
{
	unsigned char out, data;
	data=0;
	char bit;
	pinMode(SDIO,OUTPUT);
      //initialize communication
        digitalWrite(SCK,LOW);
        digitalWrite(SDIO,HIGH);
//pause?
        digitalWrite(SCK,HIGH);
        digitalWrite(SDIO,HIGH);                            
      //end init
	for(bit=6;bit>=0;bit--)   //bit-banging out the clock and address byte
	{
                digitalWrite(SDIO,((address & (1<<bit))>>bit));
                digitalWrite(SCK,LOW);
  //pause
                digitalWrite(SDIO,((address & (1<<bit))>>bit));
                digitalWrite(SCK,HIGH);
	}
	pinMode(SDIO,INPUT);
        digitalWrite(SCK,HIGH);
	delayMicroseconds(50);
	for(bit=7;bit>=0;bit--)   //bit banging out the data byte
	{
                digitalWrite(SCK,LOW);
//pause?
                digitalWrite(SCK,HIGH);
		data |= (digitalRead(SDIO)<<bit);
//pause?
	}
	return data;
}

//ported
void adns_init()
{
	pinMode(SCK,OUTPUT);
        digitalWrite(SCK,HIGH);
        delay(1000);
}

//ported
void logobot_init()
{
	pinMode(14,OUTPUT);  //data
	pinMode(15,OUTPUT);  //status
	pinMode(16,OUTPUT);  //power
        pinMode(17,INPUT);   //battery input
	pinMode(18,OUTPUT);  //xbee sleep
	servo=200;
        pinMode(2,OUTPUT);   //servo
        pinMode(3,OUTPUT);   //M1_2
        pinMode(4,OUTPUT);   //M2_2
        pinMode(5,OUTPUT);   //M1_1
        pinMode(6,OUTPUT);   //M1_2
        pinMode(7,INPUT);    //run button
}

void readData(){
	unsigned char a=Serial.read();
	PORTC=a;
	switch(state){                    //check our state machine
		case(WAITING):                //we're idle
			if(a==MSG_START)          //if we got a "begin message" byte
			{
				state=LISTENING;      //listen for a command
			}
			else
				Serial.print(WTF,BYTE);      //throw a 'WTF' exception
		break;
		case(LISTENING):               //the message is already underway
			if(valid_command(a)==1)    //did they Serial.print us a valid command?
			{
				command=a;
				if(short_packet(a)==1)   //if the command is a data request
				{
					state=LISTENING_FOR_A_CHECKSUM;   //wait for a checksum
				}
				else
					state=PARAMETER1;
			}
			else
			{
				Serial.print(INVALID_COMMAND,BYTE);
				state=WAITING;
			}
			break;
		case(PARAMETER1):
			parameter1=a;
			state=PARAMETER2;
			break;
		case(PARAMETER2):
			parameter2=a;
			state=LISTENING_FOR_A_CHECKSUM;
			parameter=parameter1*256+parameter2;
			break;
		case(LISTENING_FOR_A_CHECKSUM):
			checksum=a;
			state=LISTENING_FOR_THE_END;
			break;
		case(LISTENING_FOR_THE_END):
			if(a==MSG_END)
			{
				if(check_checksum()==1)   //the message is valid!  ACT ACT ACT!
				{			
					switch(command){
						case(FORWARD):
							move=TRANSLATE;
							status=MOVING;
							translation=0;
							rotation=0;
							translation_target=parameter;
							rotation_target=0;
						break;
						case(BACKWARD):
							status=MOVING;
							move=TRANSLATE;
							translation=0;
							rotation=0;
							translation_target=-1*parameter;
							rotation_target=0;
						break;
						case(LEFT):
							status=MOVING;
							move=ROTATE;
							translation=0;
							rotation=0;
							translation_target=0;
							rotation_target=-1*parameter;
						break;
						case(RIGHT):
							status=MOVING;
							move=ROTATE;
							translation=0;
							rotation=0;
							translation_target=0;
							rotation_target=parameter;
						break;
						case(PEN_UP):
							pen=UP;
						break;
						case(PEN_DOWN):
							pen=DOWN;
						break;
						case(GET_STATUS):
							Serial.print(status,BYTE);
						break;
					}
				}
				else
				{
					Serial.print(BAD_CHECKSUM,BYTE);
					state=WAITING;
				}
			}
			else
			{
				Serial.print(BAD_MESSAGE_STRUCTURE,BYTE);
				state=WAITING;
			}
			break;
	}
}


//ported, untested, untouched
unsigned char check_checksum()  //calculate and validate checksum
{
	unsigned char check=0;
	if(short_packet(command))
		check=command;
	else
		check=command+parameter1+parameter2;
	if(check==checksum)
		return 1;
	else
	{
		return 0;
	}
}

//ported, untouched, untested 
unsigned char short_packet(unsigned char a)
{
	if((a==GET_X)||(a==GET_Y)||(a==GET_BATT)||(a==GET_STATUS)||(a==PEN_UP)||(a==PEN_DOWN))
		return 1;
	else
		return 0;

}

//ported, untouched, untested
unsigned char valid_command(unsigned char a)
{
	if((a==GET_X)||(a==GET_Y)||(a==GET_BATT)||(a==FORWARD)||(a==BACKWARD)||(a==LEFT)||(a==RIGHT)||(a==GET_STATUS))   //put the commands in a certain range if I want this comparison to be faster  i.e.  COMMAND1<a<COMMAND10
		return 1;
	else
		return 0;
}
