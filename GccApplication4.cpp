/************************************************************************************
Written by: B. Eswar, Villy Gohil Vijay, Praneeth Chandra, Pratik Mapuskar
AVR Studio Version 6
 LCD Connections:
 			  LCD	  Microcontroller Pins
 			  RS  --> PC0
			  RW  --> PC1
			  EN  --> PC2
			  DB7 --> PC7
			  DB6 --> PC6
			  DB5 --> PC5
			  DB4 --> PC4

 ADC Connection:
 			  ACD CH.	PORT	Sensor
			  0			PF0		Battery Voltage
			  1			PF1		White line sensor 3
			  2			PF2		White line sensor 2
			  3			PF3		White line sensor 1
			  4			PF4		IR Proximity analog sensor 1*****
			  5			PF5		IR Proximity analog sensor 2*****
			  6			PF6		IR Proximity analog sensor 3*****
			  7			PF7		IR Proximity analog sensor 4*****
			  8			PK0		IR Proximity analog sensor 5
			  9			PK1		Sharp IR range sensor 1
			  10		PK2		Sharp IR range sensor 2
			  11		PK3		Sharp IR range sensor 3
			  12		PK4		Sharp IR range sensor 4
			  13		PK5		Sharp IR range sensor 5
			  14		PK6		Servo Pod 1
			  15		PK7		Servo Pod 2

 ***** For using Analog IR proximity (1, 2, 3 and 4) sensors short the jumper J2. 
 	   To use JTAG via expansion slot of the microcontroller socket remove these jumpers.  
 
 Motion control Connection:
 			L-1---->PA0;		L-2---->PA1;
   			R-1---->PA2;		R-2---->PA3;
   			PL3 (OC5A) ----> PWM left; 	PL4 (OC5B) ----> PWM right; 
 
 LCD Display interpretation:
 ****************************************************************************
 *LEFT WL SENSOR	CENTER WL SENSOR	RIGHT WL SENSOR		BLANK			*
 *BLANK				BLANK				BLANK				BLANK			*
 ****************************************************************************
 
 Note: 
 
 1. Make sure that in the configuration options following settings are 
 	done for proper operation of the code

 	Microcontroller: atmega2560
 	Frequency: 14745600
 	Optimization: -O0 (For more information read section: Selecting proper optimization 
 					options below figure 2.22 in the Software Manual)

 2. Make sure that you copy the lcd.c file in your folder

 3. Distance calculation is for Sharp GP2D12 (10cm-80cm) IR Range sensor

*********************************************************************************/
// define the frequency of the processor
#define F_CPU 14745600
// include the header files
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <math.h>
#include "lcd.h"

void lcd_port_config (void)
{
	DDRC = DDRC | 0xF7;  //all the LCD pin's direction set as output
	PORTC = PORTC & 0x80; // all the LCD pins are set to logic 0 except PORTC 7 
}

void adc_pin_config (void)
{
	DDRF = 0x00;  //set PORTF direction as input 
	PORTF = 0x00;  //set PORTF pins floating  
	DDRK = 0x00; //set PORTK direction as input 
	PORTK = 0x00;  //set PORTK pins floating 
}

void motion_pin_config (void)
{
	DDRA = DDRA | 0x0F;
	PORTA = PORTA & 0xF0;
	DDRL = DDRL | 0x18;   //Setting PL3 and PL4 pins as output for PWM generation
	PORTL = PORTL | 0x18; //PL3 and PL4 pins are for velocity control using PWM.
}

void left_encoder_pin_config (void) 
{ 
	DDRE  = DDRE & 0xEF;  //Set the direction of the PORTE 4 pin as input 
	PORTE = PORTE | 0x10; //Enable internal pull-up for PORTE 4 pin 
}

void right_encoder_pin_config (void) 
{ 
	DDRE  = DDRE & 0xDF;  //Set the direction of the PORTE 4 pin as input 
	PORTE = PORTE | 0x20; //Enable internal pull-up for PORTE 4 pin 
}

void left_position_encoder_interrupt_init (void) //Interrupt 4 enable 
{ 
	cli(); //Clears the global interrupt 
	EICRB = EICRB | 0x02; // INT4 is set to trigger with falling edge 
	EIMSK = EIMSK | 0x10; // Enable Interrupt INT4 for left position encoder 
	sei();   // Enables the global interrupt 
}

void right_position_encoder_interrupt_init (void) //Interrupt 5 enable 
{ 
	cli(); //Clears the global interrupt 
	EICRB = EICRB | 0x08; // INT5 is set to trigger with falling edge 
	EIMSK = EIMSK | 0x20; // Enable Interrupt INT5 for right position encoder 
	sei();   // Enables the global interrupt 
}

void sprayer_pin_config (void)
{
	DDRA = DDRA | 0xF0;
	PORTA = PORTA & 0x0F;
}

void sprayer_switch_on (void)
{
	PORTA = 0x60;
}

void sprayer_switch_off (void)
{
	PORTA = 0x00;
}

void buzzer_pin_config(void)
{
	DDRC = 0x08;
	PORTC = 0x00;
}

void buzzer_on(void)
{
	PORTC = 0x08;
}

void buzzer_off(void)
{
	PORTC = 0x00;
}

void out_of_path_alarm(void)
{
	buzzer_pin_config();
	while(1)
	{
		buzzer_on();// switches on the alarm
		_delay_ms(250);// keep the buzzer on
		buzzer_off();// switches off the buzzer
		_delay_ms(250);// delays to cause beeps
	}
}

void motion_set (unsigned char Direction)
{
	unsigned char PortARestore = 0;

	Direction &= 0x0F; 		// removing upper nibbel for the protection
	PortARestore = PORTA; 		// reading the PORTA original status
	PortARestore &= 0xF0; 		// making lower direction nibbel to 0
	PortARestore |= Direction; // adding lower nibbel for forward command and restoring the PORTA status
	PORTA = PortARestore; 		// executing the command
}

void forward (void)
{
	motion_set (0x06);
}

void stop (void)
{
	motion_set (0x00);
}

//Function for velocity control
void velocity (unsigned char left_motor, unsigned char right_motor)
{
	OCR5AL = (unsigned char)left_motor;
	OCR5BL = (unsigned char)right_motor;
}

void timer5_init(void)
{
	TCCR5B = 0x00;	//Stop
	TCNT5H = 0xFF;	//Counter higher 8-bit value to which OCR5xH value is compared with
	TCNT5L = 0x01;	//Counter lower 8-bit value to which OCR5xH value is compared with
	OCR5AH = 0x00;	//Output compare register high value for Left Motor
	OCR5AL = 0xFF;	//Output compare register low value for Left Motor
	OCR5BH = 0x00;	//Output compare register high value for Right Motor
	OCR5BL = 0xFF;	//Output compare register low value for Right Motor
	OCR5CH = 0x00;	//Output compare register high value for Motor C1
	OCR5CL = 0xFF;	//Output compare register low value for Motor C1
	TCCR5A = 0xA9;	/*{COM5A1=1, COM5A0=0; COM5B1=1, COM5B0=0; COM5C1=1 COM5C0=0}
 					  For Overriding normal port functionality to OCRnA outputs.
				  	  {WGM51=0, WGM50=1} Along With WGM52 in TCCR5B for Selecting FAST PWM 8-bit Mode*/
	
	TCCR5B = 0x0B;	//WGM12=1; CS12=0, CS11=1, CS10=1 (Prescaler=64)
}

void adc_init(void)
{
	ADCSRA = 0x00;
	ADCSRB = 0x00;		//MUX5 = 0
	ADMUX = 0x20;		//Vref=5V external --- ADLAR=1 --- MUX4:0 = 0000
	ACSR = 0x80;
	ADCSRA = 0x86;		//ADEN=1 --- ADIE=1 --- ADPS2:0 = 1 1 0
}

//Function For ADC Conversion
unsigned char ADC_Conversion(unsigned char Ch)
{
	unsigned char a;
	if(Ch>7)
	{
		ADCSRB = 0x08;
	}
	Ch = Ch & 0x07;
	ADMUX= 0x20| Ch;
	ADCSRA = ADCSRA | 0x40;		//Set start conversion bit
	while((ADCSRA&0x10)==0);	//Wait for conversion to complete
	a=ADCH;
	ADCSRA = ADCSRA|0x10; //clear ADIF (ADC Interrupt Flag) by writing 1 to it
	ADCSRB = 0x00;
	return a;
}

int ShaftCountRight;
int ShaftCountLeft;

//ISR for right position encoder
ISR(INT5_vect)
{
	ShaftCountRight++;  //increment right shaft position count
};
//ISR for left position encoder
ISR(INT4_vect)
{
	ShaftCountLeft++;  //increment left shaft position count
}


class motion_of_bot
{
	unsigned char Left_white_line;
	unsigned char Center_white_line;
	unsigned char Right_white_line;
	unsigned char battery_voltage;
	
	public:	
	
	motion_of_bot()
	{
		cli(); //Clears the global interrupts
		motion_pin_config();
		sprayer_pin_config();
		adc_pin_config();
		lcd_port_config();
		buzzer_pin_config();
		right_encoder_pin_config();
		left_encoder_pin_config();
		adc_init();
		timer5_init();
		left_position_encoder_interrupt_init(); 
		right_position_encoder_interrupt_init();
		sei();   //Enables the global interrupts
	}
	
	int give_percent_battery()
	{
		 battery_voltage=ADC_Conversion(0);
		 return battery_voltage;
	}
	void follow_line(void)
	{
	Left_white_line=ADC_Conversion(3);
	Center_white_line=ADC_Conversion(2);
	Right_white_line=ADC_Conversion(1);
	
	unsigned char flag = 0;
	
	if(Center_white_line<0x08)
	{
		flag=1;
		forward();
		velocity(120,120);
	}

	if((Left_white_line>0x10) && (flag==0))
	{
		flag=1;
		forward();
		velocity(120,50);
	}

	if((Right_white_line>0x10) && (flag==0))
	{
		flag=1;
		forward();
		velocity(50,120);
	}
	
	if(Right_white_line>0x08 && Right_white_line<0x10 && Left_white_line<0x08 && flag==0)
	{
		flag = 1;
		forward();
		velocity(40, 100);
	}

	if(Left_white_line>0x08 && Left_white_line<0x10 && Right_white_line<0x08 && flag==0)
	{
		flag = 1;
		forward();
		velocity(100, 40);
	}
	
	if(Center_white_line>0x10 && Left_white_line>0x10 && Right_white_line>0x10)
	{
		forward();
		velocity(0,0);
		out_of_path_alarm();
	}		
	}
	
};
int distance_in_row = 0; // variable for distance along present row
int sprayed_distance = 0; // previous distance at which bot has sprayed
int sidewise_distance = 0; // distance travelled prependicular to rows
int turns_taken = 0;// the no.of turns taken until present location
unsigned char plants_sprayed = 0;// the no.of plants that have been sprayed
int stops_in_row = 0; //no.of times the bot has stopped in the row

// function for printing the no.of plants sprayed in the row
void print_plants_sprayed(char row, char coloumn)
{
	lcd_print(row, coloumn, plants_sprayed, 3);
}

// function to count the turns taken
void turns_taken_change (void)
{
	if (ShaftCountRight-ShaftCountLeft>32)
	{
		turns_taken = turns_taken+1;
		ShaftCountLeft = 0;
		ShaftCountRight = 0;
		stops_in_row = 0;
		sprayed_distance = 0;
	}
	if (ShaftCountLeft-ShaftCountRight>32)
	{
		turns_taken = turns_taken+1;
		ShaftCountLeft = 0;
		ShaftCountRight = 0;
		stops_in_row = 0;	
		sprayed_distance = 0;
	}
}

// function to keep track of distance along the row
void distance_in_row_change (void)
{
	if (turns_taken%2==0)
	{
		distance_in_row = (ShaftCountRight+ShaftCountLeft)/2;
	}
	
}

// function to keep track of distance perpendicular to the row
void sidewise_distance_change (void)
{
	int previous_side_distance;
	if (turns_taken%2==1)
	{
		sidewise_distance = previous_side_distance+(ShaftCountRight+ShaftCountLeft)/2;
	}
	if (turns_taken%2==0)
	{
		previous_side_distance = sidewise_distance;
	}
}

//main function
int main(void)
{
	//initialize and configure all the devices
	lcd_set_4bit();
	lcd_init();
	motion_of_bot current_bot;
	while(plants_sprayed<16)// spray only as long as necessary 
	{
		current_bot.follow_line();
		turns_taken_change();
		distance_in_row_change();
		sidewise_distance_change();
		if (distance_in_row%20==0&&stops_in_row<4&&distance_in_row!=sprayed_distance)
		{
			sprayed_distance=distance_in_row;
			forward();
			velocity(0,0);
			sprayer_switch_on();
			_delay_ms(3000);
			sprayer_switch_off();
			forward();
			velocity(100,100);
			_delay_ms(500);
			if (turns_taken==0)
			{
				plants_sprayed = plants_sprayed +2;
			} 
			else
			{
				plants_sprayed = plants_sprayed +1;
			}
			stops_in_row++;
		}
		lcd_print(1,1,plants_sprayed,4);
	}		
	motion_set(0x00);// stop the bot once it's done spraying to all the plants
}