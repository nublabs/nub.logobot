
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

  #include "adns_2620.h"     //contains definitions that let the arduino talk to the mouse sensor
  #include "comm.h"          //contains the definitions for the logobot<-> computer communications protocol
  #include "dynamics.h"      //describes the dynamics of the 'bot's motions, specifically the allowable error margins for its motion
  #include "state_machine.h" //variables and defines that keep track of if the 'bot has completed its commanded motion
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
   

  int servo=1500;  //servo pulse width in microseconds

  int translation_target=0;
  int rotation_target=0;
  int leftMotorSpeed, rightMotorSpeed;
  
  // hrmm. how max can it get??
  // these are the maximum speeds and average speeds for the drive signal. 
  int maxSpeed = 220;
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
  
  int stable_counter=0;

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
  
  //translation PD loop
  translation_error=translation_target-translation;
  translation_error_derivative=translation_error-old_translation_error;
  rotation_error=rotation_target-rotation;
  rotation_error_derivative=rotation_error-old_rotation_error;
  
   if (mode == TRANSLATE)
   {
     if (abs(translation_error) < 200)
      {
        leftMotorSpeed  = (int)(translation_p*(float)translation_error+translation_d*(float)translation_error_derivative);
        rightMotorSpeed = (int)(translation_p*(float)translation_error+translation_d*(float)translation_error_derivative);
  
        //rotation PD loop
        rotation_p = 1;
        rotation_d = 0.0;
      
        leftMotorSpeed+=(int)(rotation_p*(float)rotation_error+rotation_d*(float)rotation_error_derivative);
        rightMotorSpeed-=(int)(rotation_p*(float)rotation_error+rotation_d*(float)rotation_error_derivative);  
      }
     else
      {
        // sets a cruising speed (forward or backwards) for the robot in translate mode.
        if ( translation_error > 0){ 
            leftMotorSpeed  = avgSpeed;
            rightMotorSpeed = avgSpeed;
        }
        if ( translation_error < 0){
            leftMotorSpeed  = -avgSpeed;
            rightMotorSpeed = -avgSpeed;
        }
         //rotation PD loop
         rotation_p = 0.4;
         rotation_d = 0;
       
         leftMotorSpeed+=(int)(rotation_p*(float)rotation_error+rotation_d*(float)rotation_error_derivative);
         rightMotorSpeed-=(int)(rotation_p*(float)rotation_error+rotation_d*(float)rotation_error_derivative);
     }
   }
   
   // if the mode is rotate, the above controller paradigm is inverted, giving rotation the dominant role.
   else if (mode == ROTATE)
   { 
     if ( abs(rotation_error) < 100) // what should the rotation threshold be!?? 30something clicks per degree...
      {
        
        //rotation PD loop
        rotation_p = 0.3;
        rotation_d = 0.0;
        
        leftMotorSpeed  = (int)(rotation_p*(float)rotation_error+rotation_d*(float)rotation_error_derivative);
        rightMotorSpeed = -((int)(rotation_p*(float)rotation_error+rotation_d*(float)rotation_error_derivative));
      
        rotation_error=rotation_target-rotation;
        rotation_error_derivative=rotation_error-old_rotation_error;
        leftMotorSpeed  += (int)(translation_p*(float)translation_error+translation_d*(float)translation_error_derivative);
        rightMotorSpeed += (int)(translation_p*(float)translation_error+translation_d*(float)translation_error_derivative); 
      }
     else
      {
        if (rotation_error > 0){ 
            leftMotorSpeed  =  avgRotationSpeed;
            rightMotorSpeed = -avgRotationSpeed;
        }
        if (rotation_error < 0){
            leftMotorSpeed  = -avgRotationSpeed;
            rightMotorSpeed =  avgRotationSpeed;
         
         //rotation PD loop
         translation_p = 1;
         translation_d = 0.5;
       
         leftMotorSpeed  += (int)(translation_p*(float)translation_error+translation_d*(float)translation_error_derivative);
         rightMotorSpeed += (int)(translation_p*(float)translation_error+translation_d*(float)translation_error_derivative);
       }
     }
   }
   else if (mode==STOPPED)
   {
     leftMotorSpeed=0;
     rightMotorSpeed=0;
   }
      
  old_translation_error=translation_error;
  old_rotation_error=rotation_error;

  //  motorSpeed=map(abs(abs(error)-margin), 0,abs(target-translation),0,maxSpeed);  //proportional control;
  
 
 // this is a debug tool.
  // the if statement duty-cycles the calling of the debug tool 
  dumb_count = dumb_count + 1;
  if (dumb_count == dumb_count_threshold)
  {  
    //printStatus();  //uncomment this line to send debug statements
    dumb_count = 0;
  }

  //check to see if we're within the error margins on rotation and translation
  if((mode==TRANSLATE)||(mode==ROTATE))  //don't run the counter if we're not moving--this avoids a potential overflow bug in stable_counter
  {
    if((abs(translation_error)<translation_margin)&&(abs(rotation_error)<rotation_margin))
      stable_counter++;
    else
      stable_counter=0;
  }
    
  if(stable_counter>=STABLE_COUNT)  //we've been within the error margins for long enough
  {
    Serial.print(ACTION_COMPLETE);  //tell the computer that we're done
    stable_counter=0;
    mode=STOPPED;                   //update the state machine (this will kill the motors next time around the loop)
                                    //if this doesn't kill the motors fast enough (unlikely), I can move this ahead of the motion 
                                    //control code, and I'll speed it up by one loop
    
  }

  
  // signals sent to the motor
 //   move(leftMotorSpeed,rightMotorSpeed);   //PD control;
    move(rightMotorSpeed, leftMotorSpeed);   //PD control;
  
  
  if(moveServo)
  {
    char count=0;
    for(count=0;count<numPulses;count++)
    {
      digitalWrite(2,HIGH);
      delayMicroseconds(servo);
      digitalWrite(2,LOW);
      delayMicroseconds(2000-servo);
      delay(50);
    }
    Serial.print(ACTION_COMPLETE);   //tell the computer that we're done
    moveServo=false;
  }
  
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
                mode=TRANSLATE;
                break;
    
              case(BACKWARD):
                translation_target=-1*parameter;
                rotation_target=0;
                translation=translation_error;
                rotation=rotation_error;
                mode=TRANSLATE;
                break;
    
              case(LEFT):
                translation_target=0;
                rotation_target=parameter;
                translation=translation_error;
                rotation=rotation_error;
                mode=ROTATE;
                break;
              case(RIGHT):
                translation_target=0;
                rotation_target=-1*parameter;
                translation=translation_error;
                rotation=rotation_error;
                mode=ROTATE;
                break;
              case(STOP):
                translation_target=0;
                rotation_target=0;
                translation=0;
                rotation=0;
                mode=STOPPED;
                break;
              case(PEN_UP):
                servo=1800;
                moveServo=true;
                mode=STOPPED;
              break;
              case(PEN_DOWN):
                servo=2600;
                moveServo=true;
                mode=STOPPED;
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

