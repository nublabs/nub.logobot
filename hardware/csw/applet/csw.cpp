  // this program does something, but noone knows what it does nor what it's supposed to do because it's not commented/documented yet. 



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

// the following .h files contain libraries that might be useful or something
  #include "adns_2620.h"
  #include "comm.h"
  #include "dynamics.h"
  #include <avr/wdt.h>  //watchdog timer .h file



  #include "WProgram.h"
void setup();
void changeChannel();
void loop();
void error(char* message);
boolean isValidCommand(byte a);
boolean isShortPacket(byte a);
boolean checkChecksum();
void executeCommand();
void printStatus();
void forwards(int a, int b);
void backwards(int a, int b);
void move(int a, int b);
void stop();
void update();
void ADNS_write(unsigned char address, unsigned char data);
unsigned char ADNS_read(unsigned char address);
byte state, command, parameter1,parameter2,checksum;

// defining variable names to be used in the ensuing control loops
  int parameter;
  int rotation;
  int translation;
  int translation_error, old_translation_error, translation_error_derivative;
  int rotation_error, old_rotation_error, rotation_error_derivative;

  //state machine that lets us keep track of what we command the robot to do 
  int translate= 1;
  int rotate   = 2;
  int stopped     = 3;
  int mode = translate;

  int servo=1500;  //servo pulse width in microseconds

  int a=0;

  int translation_target=0;
  int rotation_target=0;
  int leftMotorSpeed, rightMotorSpeed;
  
  // hrmm. how max can it get??
  // these are the maximum speeds and average speeds for the drive signal. 
  int maxSpeed = 100;
  int avgSpeed = maxSpeed*0.45;
  int avgRotationSpeed = maxSpeed*0.45;
  
  
  float translation_p, translation_d;
  float rotation_p, rotation_d;
  
  
  // adding a variable to split up how often the debugger runs
  int dumb_count;
  int dumb_count_threshold;

  boolean moveServo=false; //keeps track of when we're commanding a servo move.  Usually you only need to send it a move signal
                            //for 20 pulses or so.  Then you can turn it off to save power

  int numPulses=25;

  #define del 1
  #define SDIO 1
  #define SCK 2
  #define BLANK_B 249
  #define BLANK_D 0x87
  #define BLANK_SERVO 251
  #define SERVO 2

  #define UP 700
  #define DOWN 700

  #define ADNS_OUT (1<<SCK) | (1<<SDIO)
  #define ADNS_IN (1<<SCK)


void ADNS_write(unsigned char address, unsigned char data);
unsigned char ADNS_read(unsigned char address);
void update();

void setup()
{

  // PD control loop gains (constants)
    translation_p= 0.3;
    translation_d= 0;
    rotation_p   = 0.3;
    rotation_d   = 0;
  
  // discretized control loop arithmetic 
    old_translation_error=translation_target-translation;
    old_rotation_error=rotation_target-rotation;
  
  // arduino pin I/O stuff
    pinMode(2,OUTPUT);
    pinMode(3,OUTPUT);
    pinMode(4,OUTPUT);
    pinMode(5,OUTPUT);
    pinMode(6,OUTPUT);
  
  // initial state machine setting
    state=WAITING;
  
  // serial port start! 
    Serial.begin(19200);
    
    dumb_count = 0;
    dumb_count_threshold = 100;


//  changeChannel();
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
  
  // update reads new values of the X and Y translation of the mouse sensor, leading to updates of the various "___error" values
  update();
  


  //  motorSpeed=map(abs(abs(error)-margin), 0,abs(target-translation),0,maxSpeed);  //proportional control;
  
 
 // this is a debug tool.
  // the if statement duty-cycles the calling of the debug tool 
  dumb_count = dumb_count + 1;
  if (dumb_count == dumb_count_threshold)
  {  
    printStatus();  //uncomment this line to send debug statements
    dumb_count = 0;
  }


  // signals sent to the motor
    for(a=0;a<4;a++)
    {
    move(100,100);   //PD control;
    delay(187);
    move(0,0);
    delay(2000);
    move(100,-100);   //PD control;
    delay(1122);
    move(0,0);
    delay(2000);
    }
    

    while(1)
    {
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
          return((a==FORWARD)||(a==BACKWARD)||(a==LEFT)||(a==RIGHT)||(a==STOP)||(a==GET_X)||(a==GET_Y)||(a==GET_BATT)||(a==GET_STATUS)||(a==PEN_UP)||(a==PEN_DOWN));
        }

    //some packets (generally the GET_* and STOP) packets don't have a parameter.  This checks the command and returns true if it's a 
    //command without a parameter
      boolean isShortPacket(byte a)
        {
          return((a==GET_STATUS)||(a==GET_X)||(a==GET_Y)||(a==GET_BATT)||(a==STOP)||(a==PEN_UP)||(a==PEN_DOWN));
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
                mode=translate;
                break;
    
              case(BACKWARD):
                translation_target=-1*parameter;
                rotation_target=0;
                translation=translation_error;
                rotation=rotation_error;
                mode=translate;
                break;
    
              case(LEFT):
                translation_target=0;
                rotation_target=parameter;
                translation=translation_error;
                rotation=rotation_error;
                mode=rotate;
                break;
              case(RIGHT):
                translation_target=0;
                rotation_target=-1*parameter;
                translation=translation_error;
                rotation=rotation_error;
                mode=rotate;
                break;
              case(STOP):
                translation_target=0;
                rotation_target=0;
                translation=0;
                rotation=0;
                mode=stopped;
                break;
              case(PEN_UP):
                servo=1800;
                moveServo=true;
              break;
              case(PEN_DOWN):
                servo=2600;
                moveServo=true;
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


// 
void move(int a, int b)
{
  
  
  // this switchboard sets maximum motor limits, essentially as a safety net for the motors.
  // whenver it is invoked, the PD controller is not being implemented anything approaching ideally
  // the minimum value that sets motor speeds to 0 is ... of what worth?
  if(abs(a) < 10)
    a = 0;
  if(abs(b) < 10)
    b = 0;
  if(a > maxSpeed)
    a = maxSpeed;
  if(a < (-1*maxSpeed))
    a = -1*maxSpeed;
  if(b > maxSpeed)
    b = maxSpeed;
  if(b < (-1*maxSpeed))
    b = -1*maxSpeed;

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






int main(void)
{
	init();

	setup();
    
	for (;;)
		loop();
        
	return 0;
}

