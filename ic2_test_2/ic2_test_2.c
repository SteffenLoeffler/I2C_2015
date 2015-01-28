/*
 * ic2_test_2.c
 *
 * Created: 1/24/2015 5:19:34 PM
 *  Author: steffen
 */ 


#include <stdio.h>
#include <string.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdint.h>
#include <avr/wdt.h>

#define FOSC 8000000
#define BAUD 9600
#define MYUBRR (FOSC/16/BAUD-1)

#define timeout 10

#define sbi(var, mask)   ((var) |= (uint8_t)(1 << mask))
#define cbi(var, mask)   ((var) &= (uint8_t)~(1 << mask))

#define bool int
#define true 1
#define false 0
#define d_scl_p PORTC
#define d_scl_b 3
#define d_sda_p PORTD
#define d_sda_b 4
#define firstReg 0x80


void ioinit (void);
uint8_t uart_getchar(void);
void TWIInit(void);
void TWIStart(void);
void TWIStop(void);
void TWIWrite(uint8_t u8data);
int TWIRead(void);
static int uart_putchar(char c, FILE *stream);
static FILE mystdout = FDEV_SETUP_STREAM(uart_putchar, NULL, _FDEV_SETUP_WRITE);
void third_delay(void);
bool TWIReadAckNack(void);
void TWIWriteAckNack(bool result);
void message(char * msg);
int msgCntr;
char *messages[50];
char str[80];
char regStr[80];
int main(void)
{
	uint8_t key_press,i;
	int readValue,readValue1,counter;
	counter = 0;
	bool go_on;
	go_on = 1;
	readValue	= 255;
	readValue1	= 255;
	sprintf(regStr, "TWIWrite: writing 0x%x to request a first register read",firstReg);
		
		
	ioinit();
	// int IRsensors[] = { };

			
	TWIInit();	
	printf ("\n i2c test ready, press any key and watch the i2c log window.\n");
	while(1)
    {
		
		key_press = uart_getchar();
		if (key_press == 0) {
			// do nothing
		} 		
		else 
		{
			msgCntr = -1;
			go_on = 1;

			TWIInit();	

			message("TWIStart: Starting the sequence");
			TWIStart();
			message("TWIWrite: Writing 0x26");
			TWIWrite(0x26); // deviceID=0x13, appended with a 0 for write

			message("TWIReadAcknack: expecting a response");
			go_on &= TWIReadAckNack();
			
			if (go_on == true){

				message("\t-> ok");
				message(regStr);
				TWIWrite(firstReg); // register firstReg
				
				message("TWIRadAckNack: expecting a acknowledge");
				go_on &= TWIReadAckNack();
				if ( go_on == true) {message("\t-> ok");}
				else {message("\t-> failed");}
				
				message("TWIStop: going back idle");
				TWIStop();
				
				if (!go_on){
					message("2nd write failed, bailing out, attempting an TWIStop");
					TWIStop();
					}
			} 
	
			if (go_on == true){
				for (i=0; i<3; i++){ third_delay();}
			
				message("TWIStart: Starting the 2nd sequence that should read data");
				TWIStart();
				
				message("TWIWrite: writing 0x27: => read mode");
				TWIWrite(0x27);
				
				message("TWIReadAckNack: expecting an ACK");
				go_on &= TWIReadAckNack();
					
				if (go_on){
					message("\t-> ok");
				} else {
					message("third write failed, bailing out and attempting an \nTWIStop\n");
					TWIStop();
				}
			} 
			
			if ( go_on == true ){	
				message("TWIRead: now waiting for a byte coming back");
				readValue1 = TWIRead();  // should read reg 87
				
				if (readValue1>=0)
				{
					message("\tthe 1st register read may have worked");
					message("TWIWRiteAckNack: sending an ACK to the sensor");
					TWIWriteAckNack(1);
				}
				else
				{
					message("\tthe 1st register read seems to have failed");
					message("TWIWRiteAckNack: sending an NACK to the sensor");
					TWIWriteAckNack(0);
					go_on = false;
					message("first read failed, bailing out");
				
					message("TWIStop: attempting a stop");
					TWIStop();
				}
				
			}
			
			
			if (go_on == true){	
				message("TWIRead: reading the second register");
				readValue = TWIRead(); // should read reg 88
				if (readValue>=0)
				{
					message("\tthe 2nd register read may have worked");
					message("TWIWRiteAckNack: sending an ACK to the sensor");
					TWIWriteAckNack(1);
				}
				else
				{
					message("\tthe 2nd register read seems to have failed");
					message("TWIWRiteAckNack: sending an NACK to the sensor");
					TWIWriteAckNack(0);
					go_on = false;
					message("\tthe second read failed, bailing out with a");
					message("TWIStop");
					TWIStop();
				}
			}
			
			message("TWIStop: if we got here without an error then everything seems to have worked out ...");
			TWIStop();
			third_delay();
			for (i=0; i<=msgCntr; i++)
			{
				printf("%s\n",messages[i]);
			}
			printf(" I read: 0x%02x:0x%02x\n",readValue1, readValue);
			printf("done= %u, ready for next sequence ...\n", ++counter);
			printf ("\n i2c test ready, press any key and watch the i2c log window.\n");
		}
	}
}
void TWIWriteAckNack(bool result){
	int j;
	if (result)
	{   // send an ACK back => pull SDA low
		sbi(DDRC,4);
		cbi(d_sda_p,d_sda_b);
	}
	else 
	{	// send an NACK => release SDA
		cbi(DDRC,4);
		sbi(d_sda_p,d_sda_b);
	}
	third_delay();
	
	// release clock, it should go high now
	// for the remote device to register the ACK/NACK answer
	cbi(DDRC,5);  
	sbi(d_scl_p,d_scl_b);
	third_delay();
	
	// if someone else is keeping SCL low wait until it goes
	// high or you have a timeout
	j= 0;
	while (((PINC & 1<<5) == 0x0 ) && (j < timeout))
	{
		third_delay();
		j++;
	}
	if (j >= timeout){message("releasing the clock in WriteAckNack caused a timeout");}
	
	// pull the clock low again
	// to be ready for the next step 
	sbi(DDRC,5);
	cbi(d_scl_p,d_scl_b);
	third_delay();
	
	// wait for the remote device to release sda again?!
	
	
}
bool TWIReadAckNack(void)
{
		bool ackNack;
		ackNack = false;
		uint8_t j;
		
		cbi(DDRC,4);  // release SDA
		sbi(d_sda_p,d_sda_b);
		third_delay();		
		
		cbi(DDRC,5);  //release clock, it should go high now
		sbi(d_scl_p,d_scl_b);
		third_delay();
		j= 0;
		while (((PINC & 1<<5) == 0x0 ) && (j < timeout))
		{
			third_delay();  
			j++;
		}
		if (j >= timeout){message("releasing clock in ReadAckNack caused a timeout");}

		ackNack = ((PINC  & 1<<4) == 0x0);

		sbi(DDRC, 5); // bring the clock SCK  low again
		cbi(d_scl_p,d_scl_b);
		third_delay();
		
		
		sbi(DDRC,4); //force SDA low
		cbi(d_sda_p,d_sda_b);
		third_delay();
		//sbi(DDRC, 4); // bring SDA low
		//third_delay();   // ready for a stop bit or for a next  read/write
		
		return ackNack;

}
void third_delay(void)
{
  // a clock period at 100MHz is 10us,
  // I need 3 positions within a period:
  // 1: change data
  // 2: raise clock
  // 3: drop clock
  // this here is  a delay of about 3.3us
  asm volatile ("nop");
  asm volatile ("nop");
  asm volatile ("nop");
  asm volatile ("nop");
  asm volatile ("nop");
  asm volatile ("nop");
  asm volatile ("nop");
  asm volatile ("nop");
  asm volatile ("nop");
  asm volatile ("nop");
  asm volatile ("nop");
  asm volatile ("nop");
  asm volatile ("nop");
  asm volatile ("nop");
  asm volatile ("nop");
  asm volatile ("nop");
  asm volatile ("nop");
    
  asm volatile ("nop");
  asm volatile ("nop");
  asm volatile ("nop");
  asm volatile ("nop");
  asm volatile ("nop");
  asm volatile ("nop");
  asm volatile ("nop");
  asm volatile ("nop");
  asm volatile ("nop");
  asm volatile ("nop");
  asm volatile ("nop");
  asm volatile ("nop");
  asm volatile ("nop");
asm volatile ("nop");
asm volatile ("nop");
asm volatile ("nop");
asm volatile ("nop");
asm volatile ("nop");
asm volatile ("nop");
asm volatile ("nop");
asm volatile ("nop");
asm volatile ("nop");
asm volatile ("nop");
asm volatile ("nop");
asm volatile ("nop");
asm volatile ("nop");
asm volatile ("nop");
asm volatile ("nop");
asm volatile ("nop");
asm volatile ("nop");
asm volatile ("nop");
asm volatile ("nop");
asm volatile ("nop");
asm volatile ("nop");
asm volatile ("nop");
asm volatile ("nop");
asm volatile ("nop");
asm volatile ("nop");
asm volatile ("nop");}
void TWIInit(void)
{
	// SDA and SCK need to be high - this should be called only once ...
	// PORT should be low but input
	// then I can switch by toggling bettween input and output
	PORTC &= (uint8_t)~(1 << 5); // output low
    DDRC  &= (uint8_t)~(1 << 5); // make it an input
	sbi(d_scl_p,d_scl_b);
	
	PORTC &= (uint8_t)~(1 << 4); // outputs low
    DDRC  &= (uint8_t)~(1 << 4); // make it an input
	sbi(d_sda_p,d_sda_b);
	
	third_delay();
	third_delay();
	third_delay();
}
void TWIStart(void)
{
   // SDA drops while SCK stays high
   // SCK goes low
   DDRC |=  1<< 4; // 
   cbi(d_sda_p,d_sda_b);
   third_delay();
   
   DDRC |= 1 << 5; //SCK
   cbi(d_scl_p,d_scl_b);
   third_delay();	
}
int TWIRead(void){
	uint8_t i,j;
	int readValue;
	readValue = 0;
	bool timeoutFlag;
	
	timeoutFlag = false;
	
	for (i=7; i<=7; i--)
	{
		cbi(DDRC,4);
		sbi(d_sda_p,d_sda_b);
		third_delay();
		// now the other device should have set the SDA bit
		
		cbi(DDRC,5); //release the clock
		sbi(d_scl_p,d_scl_b);
		third_delay();
		j= 0;
		while (((PINC & 1<<5) == 0x0 ) && (j < timeout))
		{
			third_delay();
			j++;
		}
		
		if (PINC & 1<<4) 
		{
			readValue |= (1 << i);	
			message("\t-> I read a 1");
		}
		else
		{
			message("\t-> I read a 0");			
		}
		
		if (j>= timeout){
			timeoutFlag = true;	
			message("releasing the clock in WriteAckNack caused a timeout");
		}
		
		third_delay();
		sbi(DDRC,5); // pull SCL low 
		cbi(d_scl_p,d_scl_b);
		//printf("i=%u\n",i);
	}
	if (timeoutFlag){readValue = -1;}
	return(readValue);
}
void TWIWrite(uint8_t u8data)
{
	uint8_t i,j;
	for (i=0; i<8; i++)
	{
		// output data, either write 0 or switch to read for a 1
		if ((u8data & 0x80)  ==  0x00)
		{  // a one => make it an input
			sbi(DDRC,4); 
			cbi(d_sda_p,d_sda_b);
		}  else 
		{
			cbi(DDRC,4);
			sbi(d_sda_p,d_sda_b);
		}
		third_delay();
		
		cbi(DDRC,5);  //rising clock
		sbi(d_scl_p,d_scl_b);
		third_delay();
		
		j= 0;
		while (((PINC & 1<<5) == 0x0 ) && (j < timeout))
		{
			third_delay();
			j++;
		}
		if (j >= timeout){message("releasing the clock in TWIWrite caused a timeout");}
	
		third_delay();
		sbi(DDRC,5); //drop the clock again
		cbi(d_scl_p,d_scl_b);
		third_delay();
		u8data = u8data << 1;
	}
}
void ioinit (void)
{
	//1 = output, 0 = input
	DDRB = 0b11101111; //PB4 = MISO
	DDRC = 0b11111111; //
	DDRD = 0b11111010; //PORTD (RX on PD0/INT0 on PD2)
	
	PORTD |= (1<<2); // enable pullup on PD2

	//USART Baud rate: 9600
	UBRR0H = MYUBRR >> 8;
	UBRR0L = MYUBRR;
	UCSR0B = (1<<RXEN0)|(1<<TXEN0);
	
	stdout = &mystdout; //Required for printf init
	
}
void TWIStop(void)
{
	uint8_t i,j;
	
	sbi(DDRC,4); // pull SDA low, it may have been high
	cbi(d_sda_p,d_sda_b);
	third_delay();
	
	cbi(DDRC,5); // release scl
	sbi(d_scl_p,d_scl_b);
	third_delay();
	
	// wait for SCL go high if another device keeps it low
	j=0;
	while (((PINC & 1<<5) == 0x0 ) && (j < timeout))
	{
		third_delay();
		j++;
	}
	if (j >= timeout){message("releasing the clock in TWIStop caused a timeout");}
	
	
	cbi(DDRC,4); // release SDA, it should go high => stop condition
	sbi(d_sda_p,d_sda_b);
	third_delay();
	
	// SDA shoud be high but if it isn't ...
	j=0;
	while (((PINC & 1<<4) == 0x0) && (j < timeout))
	{
		j++;
		//provide extra clocks for the device to eventually release SDA
		sbi(DDRC,5);
		//third_delay();
		cbi(d_scl_p,d_scl_b);
		third_delay();

		cbi(DDRC,5);
		//third_delay();
		sbi(d_scl_p,d_scl_b);
		third_delay();
	}
	if (j >0){
		sprintf(str,"SDA did not get released right away but needed %u extra clock edges.",j);
		message(str);
	}
	if (j >= timeout){message("releasing the SDA in TWIStop caused a timeout");}


    for (i=0; i<3; i++)
	{
		third_delay();
	}
} // TWIStop
void message(char * msg)
{
	msgCntr++;
	messages[msgCntr] = msg;
}
uint8_t uart_getchar(void)
{
	// this is the original
	// it wait for a key to be pressed and
	// then returns it
	//    while( !(UCSR0A & (1<<RXC0)) );
	//    return(UDR0);

	// this here just checks if a key has been pressed
	// and returns it, otherwise 0
	if ( !(UCSR0A & (1<<RXC0)) ) {
	return(0);}
	else { return(UDR0);}
}
static int uart_putchar(char c, FILE *stream)
{
	if (c == '\n') uart_putchar('\r', stream);
	
	loop_until_bit_is_set(UCSR0A, UDRE0);
	UDR0 = c;
	
	return 0;
}
