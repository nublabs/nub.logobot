// deal with interrupts
#include <avr/io.h>
#include <inttypes.h>
#include <avr/interrupt.h>
#include <avr/signal.h>

#include "adns_2620.h"
#include "comm.h"
#include "dynamics.h"

#include "WProgram.h"
void setup();
void loop();
void forwards(unsigned char a, unsigned char b);
void backwards(unsigned char a, unsigned char b);
void left(unsigned char a, unsigned char b);
void right(unsigned char a, unsigned char b);
void stop();
void update();
void ADNS_write(unsigned char address, unsigned char data);
unsigned char ADNS_read(unsigned char address);
void uart_init();
void adns_init();
void logobot_init();
unsigned char check_checksum();
unsigned char short_packet(unsigned char a);
unsigned char valid_command(unsigned char a);
void pause(int a);
void pauseus(int a);
void send(unsigned char c);
void adc_init();
unsigned char getAval(char channel);
void blink();
unsigned int servo=0;
unsigned char servo_running=0;
unsigned int pulse= 2000;
unsigned char state=WAITING;
int rotation;
int translation;
unsigned int parameter1,parameter2,parameter;
unsigned char command;
unsigned char buffer[256];
unsigned char checksum;
unsigned char start, end;
unsigned char message_valid=0;
unsigned char status=WAITING;
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


void pause(int a);
void pauseus(int a);
void send(unsigned char c);
void adc_init();
unsigned char getAval(char channel);
void blink();

void ADNS_write(unsigned char address, unsigned char data);
unsigned char ADNS_read(unsigned char address);


void forwards(unsigned char a, unsigned char b);
void backwards(unsigned char a, unsigned char b);
void left(unsigned char a, unsigned char b);
void right(unsigned char a, unsigned char b);
void stop();


SIGNAL(SIG_USART_RECV);

unsigned char check_checksum();
unsigned char short_packet(unsigned char a);
unsigned char valid_command(unsigned char a);


void uart_init();
void adns_init();
void logobot_init();



void update();

void setup()
{

	uart_init();
	adc_init();
	adns_init();
	logobot_init();

	start=0;
	end=0;
	// enables interrupts; without this, serial communication impossible
	sei(); 

	PORTB=1<<SCK;
	// pause to let mouse initialize
	pause(100);
	DDRD=255;

	int count=0;
	while(1)
	{
		if(servo_running == 1)
		{
			// bit masking to appropriately set PORTD
			// TODO: explain and connect to schematic
			PORTD=(1<<SERVO) | (PORTD & BLANK_SERVO);

			pauseus(servo);
			PORTD=(PORTD & BLANK_SERVO);
			//pauses for max(pulse width)
			pauseus(pulse-servo);
			//pauses for time between pulses
			pause(20);
			count++;

			// makes sure servo reaches position
			// 1.5 ms dead center
			// 1ms left
			// 2ms right
			if(count>5)
			{
				servo_running=0;
				count=0;
			}
		}

		if(message_valid == 1)
		{
			update();  //this is going to accumulate errors.  I'm clearing the x and y registers
			// TODO: explain why x/y ==> translation/rotation

			switch(command){
				case(FORWARD):
				status = MOVING;
				translation = 0;
				rotation = 0;
					while(abs(translation) < (parameter - translation_margin))  //flips out (wraparound!) if the parameter is less than the translation margin
					{
						update();
						forwards(255,255);
					}
					stop();
				break;
				
				case(BACKWARD):
				status = MOVING;
				translation = 0;
				rotation = 0;
					while(abs(translation)<(parameter-translation_margin))
					{
						update();
						backwards(255,255);
					}
					stop();
				break;
				
				case(LEFT):
				status=MOVING;
				translation=0;
				rotation=0;
					while(abs(rotation)<(parameter-rotation_margin))
					{
						update();
						left(255,255);
					}
					stop();
				break;
				
				case(RIGHT):
				status=MOVING;
				translation=0;
				rotation=0;
					while(abs(rotation)<(parameter-rotation_margin))
					{
						update();
						right(255,255);
					}
					stop();
				break;

				case(GET_STATUS):
					send(status);
				break;

				case(PEN_UP):
					servo=UP;
					servo_running=1;
				break;

				case(PEN_DOWN):
					servo=DOWN;
					servo_running=1;
				break;


			}
			send(ACTION_COMPLETE);
			message_valid=0;
			status=WAITING;
		}
				
	}



}

void loop(){}
void forwards(unsigned char a, unsigned char b)
{
	PORTD=0x60 | (PORTD & BLANK_D);
}
void backwards(unsigned char a, unsigned char b)
{
	PORTD=0x18 | (PORTD & BLANK_D);
}
void left(unsigned char a, unsigned char b)
{
	PORTD=0x48 | (PORTD & BLANK_D);
}
void right(unsigned char a, unsigned char b)
{
	PORTD=0x30 | (PORTD & BLANK_D);
}
void stop()
{
	PORTD=(PORTD & BLANK_D);
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
	pauseus(50);
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

void uart_init()
{
	UCSR0A = (1<<RXC0) | (1<<U2X0);
	UCSR0B = (1<<RXEN0) | (1<<TXEN0) | (1<<RXCIE0);
	UCSR0C = (3<<UCSZ00);
	UBRR0H = 0;
	// sets baud rate
	UBRR0L = 51;  //19200 8n1
}

void adns_init()
{
	// sets clock high
	DDRB|=1<<SCK;
}

void logobot_init()
{
	//sets port directions to right numbers
	DDRD=127;
	DDRC=255;
	servo=200;
}

// = interrupt for Atmel
SIGNAL(SIG_USART_RECV){
	//disables interrupts
	cli();

	//stores received message in a
	unsigned char a=UDR0;
	//lights up LEDs upon message receipt
	PORTC=a;

	switch(state){                    //check our state machine
	//TODO: explain message protocol
		case(WAITING):                //we're idle
			if(a==MSG_START)          //if we got a "begin message" byte
			{
				state=LISTENING;      //listen for a command
			}
			else
				// general error
				send(WTF);      //throw a 'WTF' exception

		break;

		case(LISTENING):               //the message is already underway
			if(valid_command(a)==1)    //did they send us a valid command?
			{
				command=a;
				// a short packet is a paramaterless 'command'
				if(short_packet(a)==1)   //if the command is a data request
				{

					state=LISTENING_FOR_A_CHECKSUM;   //wait for a checksum
				}
				else
					state=PARAMETER1;
			}
			else
			{
				send(INVALID_COMMAND);
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
				// Add all bytes % 255
				if(check_checksum()==1)
				{			
					message_valid=1;
					state=WAITING;
				}
				else
				{
					send(BAD_CHECKSUM);
					state=WAITING;
				}
			}
			else
			{
				send(BAD_MESSAGE_STRUCTURE);
				state=WAITING;
			}
			break;
	}
	sei();
}

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
//		return 1;
}

unsigned char short_packet(unsigned char a)
{
	if((a==GET_X)||(a==GET_Y)||(a==GET_BATT)||(a==GET_STATUS)||(a==PEN_UP)||(a==PEN_DOWN))
		return 1;
	else
		return 0;
//		return 0;
}

unsigned char valid_command(unsigned char a)
{
	if((a==GET_X)||(a==GET_Y)||(a==GET_BATT)||(a==FORWARD)||(a==BACKWARD)||(a==LEFT)||(a==RIGHT)||(a==GET_STATUS)||(a==PEN_UP)||(a==PEN_DOWN))   //put the commands in a certain range if I want this comparison to be faster  i.e.  COMMAND1<a<COMMAND10
		return 1;
	else
		return 0;
}

void pause(int a)
{
	int b,c;
	for(b=0;b<a;b++)	
		for(c=0;c<1000;c++)
			;
}

void pauseus(int a)
{
	int b;
	for(b=0;b<a;b++)	
		;
}

void send(unsigned char c) {
   // wait until UDR ready
	//while(!(UCSR0A & (1 << UDRE0)))
		UDR0 = c;    // send character
		pause(1);
}


void adc_init()
{
	ADMUX=0x40;
	ADCSRA|=0x87;  //clock prescaler of 128
}



unsigned char getAval(char channel)
{
	// TODO: write battery state wrapper
	ADMUX=1<<channel;
	ADCSRA|=0x40;
	ADCSRB=0x00;
	while((ADCSRA & 0x40) !=0){};
		return ADCH;

}

void blink()
{
	PORTC=255;
	pause(10);
	PORTC=0;
}





int main(void)
{
	init();

	setup();
    
	for (;;)
		loop();
        
	return 0;
}

