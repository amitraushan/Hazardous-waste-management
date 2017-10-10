/********************************************************************************
 Written by: Sachitanand Malewar, NEX Robotics Pvt. Ltd.
 AVR Studio Version 4.17, Build 666

 Date: 26th December 2010
 
 This is firmware for Fire Bird V ATMEGA2560 robot control over XBee wireless module.

 Communication protocol for this application example is covered in the chapter 7 of the Hardware Manual.
 You can use this firmware to control robot over Matlab, Scilab or Labview etc.

 There are 3 variants of this application example for RS232 serial, 
 USB (via FT232 USB to serial Converter on the robot) and using XBee wireless module.
 All are exactly identical except that they use different UART ports.
 
 Make sure that XBee wireless module on the robot and XBee USB wireless both are 
 configured at 115200bps.

 Concepts covered:
 Almost all the aspects are used in this application example.

 Note: Make sure that in the configuration options following settings are 
 done for proper operation of the code

 Microcontroller: atmega2560
 Frequency: 14745600
 Optimization: -O0 (For more information read section: Selecting proper optimization 
 					options below figure 2.22 in the Software Manual)
*********************************************************************************/

/********************************************************************************

   Copyright (c) 2010, NEX Robotics Pvt. Ltd.			-*- c -*-
   All rights reserved.

   Redistribution and use in source and binary forms, with or without
   modification, are permitted provided that the following conditions are met:

   * Redistributions of source code must retain the above copyright
     notice, this list of conditions and the following disclaimer.

   * Redistributions in binary form must reproduce the above copyright
     notice, this list of conditions and the following disclaimer in
     the documentation and/or other materials provided with the
     distribution.

   * Neither the name of the copyright holders nor the names of
     contributors may be used to endorse or promote products derived
     from this software without specific prior written permission.

   * Source code can be used for academic purpose. 
	 For commercial use permission form the author needs to be taken.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
  POSSIBILITY OF SUCH DAMAGE. 

  Software released under Creative Commence cc by-nc-sa licence.
  For legal information refer to: 
  http://creativecommons.org/licenses/by-nc-sa/3.0/legalcode

********************************************************************************/

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

#include "lcd.c"

unsigned char left_motor_velocity = 0x00;
unsigned char right_motor_velocity = 0x00;
unsigned char horz_servo_pos = 0x00;
unsigned char vert_servo_pos = 0x00;
unsigned char BATT_VALUE, WL_CENTER, WL_LEFT, WL_RIGHT, SHARP_3, SHARP_2, SHARP_4, SHARP_1, SHARP_5, IR1, IR2, IR3, IR4, IR5, IR6, IR7, IR8; 
unsigned char robot_id = 0x05;
unsigned char ADC_flag;

//Shaft encoder related variables
unsigned int left_motor_pulse_count = 0x00; 		//incremented at each isr
unsigned int left_motor_pulse_count_locked = 0x00;  // locked at serial comm isr

unsigned int right_motor_pulse_count = 0x00;        //incremented at each isr
unsigned int right_motor_pulse_count_locked = 0x00; // locked at serial comm isr
//-------------------------------------------------------------------------------

void motion_pin_config (void)
{
 DDRA = DDRA | 0x0F;   //set direction of the PORTA 3 to PORTA 0 pins as output
 PORTA = PORTA & 0xF0; // set initial value of the PORTA 3 to PORTA 0 pins to logic 0
 DDRL = DDRL | 0x18;   //Setting PL3 and PL4 pins as output for PWM generation
 PORTL = PORTL | 0x18; //PL3 and PL4 pins are for velocity control using PWM
}

//Configure PORTB 5 pin for servo motor 1 operation
void servo1_pin_config (void)
{
 DDRB  = DDRB | 0x20;  //making PORTB 5 pin output
 PORTB = PORTB | 0x20; //setting PORTB 5 pin to logic 1
}

//Configure PORTB 6 pin for servo motor 2 operation
void servo2_pin_config (void)
{
 DDRB  = DDRB | 0x40;  //making PORTB 6 pin output
 PORTB = PORTB | 0x40; //setting PORTB 6 pin to logic 1
}

//Configure PORTB 7 pin for servo motor 3 operation
void servo3_pin_config (void)
{
 DDRB  = DDRB | 0x80;  //making PORTB 7 pin output
 PORTB = PORTB | 0x80; //setting PORTB 7 pin to logic 1
}

void spi_pin_config (void)
{
 DDRB = DDRB | 0x07;
 PORTB = PORTB | 0x07;
}

void lcd_port_config (void)
{
 DDRC = DDRC | 0xF7;    //all the LCD pin's direction set as output
 PORTC = PORTC & 0x80;  // all the LCD pins are set to logic 0 except PORTC 7
}

//Function to initialize Buzzer 
void buzzer_pin_config (void)
{
 DDRC = DDRC | 0x08;		//Setting PORTC 3 as output
 PORTC = PORTC & 0xF7;		//Setting PORTC 3 logic low to turnoff buzzer
}

//Function to configure INT4 (PORTE 4) pin as input for the left position encoder
void left_encoder_pin_config (void)
{
 DDRE  = DDRE & 0xEF;  //Set the direction of the PORTE 4 pin as input
 PORTE = PORTE | 0x10; //Enable internal pull-up for PORTE 4 pin
}

//Function to configure INT5 (PORTE 5) pin as input for the right position encoder
void right_encoder_pin_config (void)
{
 DDRE  = DDRE & 0xDF;  //Set the direction of the PORTE 4 pin as input
 PORTE = PORTE | 0x20; //Enable internal pull-up for PORTE 4 pin
}

//ADC pin configuration
void adc_pin_config (void)
{
 DDRF = 0x00;  //set PORTF direction as input
 PORTF = 0x00; //set PORTF pins floating
 DDRK = 0x00;  //set PORTK direction as input
 PORTK = 0x00; //set PORTK pins floating
}

//Port init for different modules
void port_init(void)
{
 motion_pin_config();
 servo1_pin_config();
 servo2_pin_config();
 servo3_pin_config();
 spi_pin_config();
 lcd_port_config();
 buzzer_pin_config();           
 left_encoder_pin_config();
 right_encoder_pin_config();
 adc_pin_config();	 
}

// Timer 5 initialized in PWM mode for velocity control
// Prescale:256
// PWM 8bit fast, TOP=0x00FF
// Timer Frequency:225.000Hz
void timer5_init()
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

//TIMER1 initialize - prescale:256
// WGM: 7) PWM 10bit fast, TOP=0x03FF
// desired value: 40Hz
// actual value: 42.187Hz (5.2%)
void timer1_init(void)
{
 TCCR1B = 0x00; //stop
 TCNT1H = 0xFC; //setup
 TCNT1L = 0x01;
 OCR1AH = 0x03;
 OCR1AL = 0xFF;
 OCR1BH = 0x03;
 OCR1BL = 0xFF;
 OCR1CH = 0x00;
 OCR1CL = 0x00;
 //ICR1H  = 0x03;
 //ICR1L  = 0xFF;
 TCCR1A = 0xA3;
 TCCR1C = 0x00;
 TCCR1B = 0x0C; //start Timer
}

//UART1 initialize
// desired baud rate:115200
// char size: 8 bit
// parity: Disabled
void uart0_init(void)
{
 UCSR0B = 0x00; //disable while setting baud rate
 UCSR0A = 0x00;
 UCSR0C = 0x06;
 UBRR0H = 0x00; //set baud rate hi
 UBRR0L = 0x07; //set baud rate lo
 UCSR0B = 0x98;
}

//SPI initialize
// clock rate: 921600hz
void spi_init(void)
{
 SPCR = 0x53; //setup SPI
 SPSR = 0x00; //setup SPI
 SPDR = 0x00;}

//ADC initialize
// Conversion time: 56uS
void adc_init(void)
{
 ADCSRA = 0x00;
 ADCSRB = 0x00;		//MUX5 = 0
 ADMUX = 0x20;		//Vref=5V external --- ADLAR=1 --- MUX4:0 = 0000
 ACSR = 0x80;
 ADCSRA = 0x86;		//ADEN=1 --- ADIE=1 --- ADPS2:0 = 1 1 0
}

//-------------------------------------------------------------------------------
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

//-------------------------------------------------------------------------------
//ISR for right position encoder
//-------------------------------------------------------------------------------

ISR(INT5_vect)  
{
 right_motor_pulse_count++;
 //external interupt on INT5
}


//ISR for left position encoder

ISR(INT4_vect)   
{
 left_motor_pulse_count++;
 //external interupt on INT4
}

//-------------------------------------------------------------------------------
//-- ADC Conversion Function --------------
//-------------------------------------------------------------------------------
unsigned char ADC_Conversion(unsigned char ch)
{
unsigned char a;
 if(ch>7)
	{
		ADCSRB = 0x08;
	}
 ch = ch & 0x07;			  //Store only 3 LSB bits
 ADMUX= 0x20 | ch;			  //Select the ADC channel with left adjust select
 ADC_flag = 0x00; 			  //Clear the user defined flag
 ADCSRA = ADCSRA | 0x40;	  //Set start conversion bit
 while((ADCSRA&0x10)==0);	  //Wait for ADC conversion to complete
 a=ADCH;
 ADCSRA = ADCSRA|0x10;        //clear ADIF (ADC Interrupt Flag) by writing 1 to it
 ADCSRB = 0x00;
 return a;
}

//-------------------------------------------------------------------------------
//-- Function To Transmit/Receive through SPI interface ------
//-------------------------------------------------------------------------------
unsigned char spi_master_tx_and_rx (unsigned char data)
{
 unsigned char rx_data = 0;

 PORTB = PORTB & 0xFE;        // make SS pin low
 SPDR = data;
 while(!(SPSR & (1<<SPIF)));  //wait for data transmission to complete

 _delay_ms(1);                //time for ADC conversion in the slave microcontroller
 
 SPDR = 0x50;                 // send dummy byte to read back data from the slave microcontroller
 while(!(SPSR & (1<<SPIF)));  //wait for data reception to complete
 rx_data = SPDR;
 PORTB = PORTB | 0x01;        // make SS high
 return rx_data;
}

//-------------------------------------------------------------------------------

void sensor_data_interpretation(void) //ld, fd, rd, light int
{
 
 WL_LEFT = ADC_Conversion(3);
  
 WL_CENTER = ADC_Conversion(2);
  
 WL_RIGHT = ADC_Conversion(1);
 
 
 IR1 = ADC_Conversion(4); 
 
 IR2 = ADC_Conversion(5);
 
 IR3 = ADC_Conversion(6);
  
 IR4 = ADC_Conversion(7);

 IR5 = ADC_Conversion(8);
 
 IR6 = spi_master_tx_and_rx(5);  
 
 IR7 = spi_master_tx_and_rx (6); 
 
 IR8 = spi_master_tx_and_rx (7);
 
 
 SHARP_1 = ADC_Conversion(9);
 //lcd_print(2,1,SHARP_1,3);
 
 SHARP_2 = ADC_Conversion(10);
 //lcd_print(2,5,SHARP_2,3);
 
 SHARP_3 = ADC_Conversion(11);
 //lcd_print(1,13,SHARP_3,3);
 
 SHARP_4 = ADC_Conversion(12);
 //lcd_print(2,9,SHARP_4,3);
 
 SHARP_5 = ADC_Conversion(13);
 
 BATT_VALUE = ADC_Conversion(0);
  
}

//-------------------------------------------------------------------------------
//------Function used for setting motor's direction----------
//-------------------------------------------------------------------------------
void motion_set(unsigned char ucDirection)
{
 unsigned char ucPortARestore = 0;

 ucDirection &= 0x0F;            // removing upper nibbel for the protection
 ucPortARestore = PORTA;         // reading the PORTA original status
 ucPortARestore &= 0xF0;         // making lower direction nibbel to 0
 ucPortARestore |= ucDirection;  // adding lower nibbel for forward command and restoring the PORTA status
 PORTA = ucPortARestore;         // executing the command
}

void forward(void) 
{ 
 motion_set(0x06);
}

void back(void)  
{
  motion_set(0x09);  
}

void left(void)  
{
  motion_set(0x05);
}

void right(void)
{
  motion_set(0x0A);
}

void stop(void)
{
  motion_set(0x00);
}

//-------------------------------------------------------------------------------
//------- Function For Buzzer ON ----------
//-------------------------------------------------------------------------------
void buzzer_on(void)
{
 unsigned char portc_restore = 0;
 portc_restore = PORTC; // reading the PORTC original status
 portc_restore |= 0x08; // setting the bit to turn on the buzzer
 PORTC = portc_restore; // executing the command
}

//-------------------------------------------------------------------------------
//------ Function For Buzzer OFF -------------
//-------------------------------------------------------------------------------
void buzzer_off(void)
{
 unsigned char portc_restore = 0;
 portc_restore = PORTC; // reading the PORTC original status
 portc_restore &= 0xF7; // resetting the bit to turn off the buzzer
 PORTC = portc_restore; // executing the command
}


//-------------------------------------------------------------------------------
//call this routine to initialize all peripherals
void init_devices(void)
{
 //stop errant interrupts until set up
 cli();              //disable all interrupts
 port_init();
 uart0_init();
 adc_init();
 timer1_init();
 timer5_init();
 lcd_set_4bit();
 lcd_init();
 spi_init();
 
 // below for lines are important for Encoder init
 left_position_encoder_interrupt_init();
 right_position_encoder_interrupt_init();
 EICRB  = 0x0A;     //pin change int edge 4:7
 EIMSK  = 0x30;

 sei();             //re-enable interrupts
 //all peripherals are now initialized
}

//-------------------------------------------------------------------------------
SIGNAL(SIG_USART0_RECV) 		// ISR for receive complete interrupt
{
 unsigned char ser_data = 0x00;
 unsigned char ser_data_upper_nibbel = 0x00;
 unsigned char ser_data_lower_nibbel = 0x00;
 //uart has received a character in UDR0
 ser_data = UDR0;
 ser_data_upper_nibbel = ser_data & 0xF0;
 ser_data_lower_nibbel = ser_data & 0x0F;
//__________________________Locomotoion setting commands_____________
 if (ser_data_upper_nibbel == 0x10)
 {
 left_motor_velocity = ser_data_lower_nibbel;
 }
 
 if (ser_data_upper_nibbel == 0x20)
 {
 left_motor_velocity = ((ser_data_lower_nibbel * 16) + left_motor_velocity);
 if(left_motor_velocity > 0xFF)
 {
 left_motor_velocity = 0xFF;
 }
 OCR5AL = left_motor_velocity;
 //left_motor_pwm_feedback = left_motor_velocity;
 }
 
 if (ser_data_upper_nibbel == 0x30)
 {
 right_motor_velocity = ser_data_lower_nibbel;
 }
 
 if (ser_data_upper_nibbel == 0x40)
 {
 right_motor_velocity = ((ser_data_lower_nibbel * 16) + right_motor_velocity);
 if(right_motor_velocity > 0xFF)
 {
 right_motor_velocity = 0xFF;
 }
 OCR5BL = right_motor_velocity;
 //right_motor_pwm_feedback = right_motor_velocity;
 }
   
 if (ser_data == 0x51) //Forward
 {
  forward(); 
 }
 
 if (ser_data == 0x52) //Backward
 {
  back();
 }
 
 if (ser_data == 0x53) //Right
 {
  right();
 }
 
 if (ser_data == 0x54) //Left
 {
  left();
 }
 
 if (ser_data == 0x59) //Stop
 {
  stop();
 }

//_______________________Shaft encoder data__________________________

 if(ser_data == 0x72) //Lower_byte number of pulse counted
 {
  left_motor_pulse_count_locked = left_motor_pulse_count;
  UDR0 = left_motor_pulse_count_locked % 256;
 }
 
 if(ser_data == 0x73) //Upper_byte number of pulse counted
 {
  UDR0 = left_motor_pulse_count_locked / 256;
 }

 if(ser_data == 0x79) //Lower_byte number of pulse counted
 {
  right_motor_pulse_count_locked = right_motor_pulse_count;
  UDR0 = right_motor_pulse_count_locked % 256;
 }
 
 if(ser_data == 0x7A) //Upper_byte number of pulse counted
 {
  UDR0 = right_motor_pulse_count_locked / 256;
 }
   
//_______________________Servo motors position commands________________
 if (ser_data_upper_nibbel == 0x80)
 {
  horz_servo_pos = ser_data_lower_nibbel;
 }
 
 if (ser_data_upper_nibbel == 0x90)
 {
  horz_servo_pos = ((ser_data_lower_nibbel * 16) + horz_servo_pos);
  
  if(horz_servo_pos > 0x65)
  {
   horz_servo_pos = 0x65;
  }
 
  if(horz_servo_pos < 0x15)
  {
   horz_servo_pos = 0x15;
  }
 
  OCR1AH = 0x00;
  OCR1AL = horz_servo_pos;
 }
 
 if (ser_data_upper_nibbel == 0xA0)
  {
   vert_servo_pos = ser_data_lower_nibbel;
  }
 
  if (ser_data_upper_nibbel == 0xB0)
  {
   vert_servo_pos = ((ser_data_lower_nibbel * 16) + vert_servo_pos);
 
   if(vert_servo_pos > 0x65)
   {
    vert_servo_pos = 0x65;
   }
 
   if(vert_servo_pos < 0x15)
   {
    vert_servo_pos = 0x15;
   }
   OCR1BH = 0x00;
   OCR1BL = vert_servo_pos;
  }
 
//________________ IR PROXIMITY STATUS COMMANDS _____________________
 if (ser_data == 0xC1)       //0xC1
 {
  UDR0 = IR1;
 }
 
 if (ser_data == 0xC2)
 {
  UDR0 = IR2;
 }
 
 if (ser_data == 0xC3)
 {
  UDR0 = IR3;
 }
 
 if (ser_data == 0xC4) 
 {
  UDR0 = IR4;
 }
 
 if (ser_data == 0xC5)
 {
  UDR0 = IR5;
 }
 
 if (ser_data == 0xC6)
 {
  UDR0 = IR6;
 }
 
 if (ser_data == 0xC7)
 {
  UDR0 = IR7;
 }
 
 if (ser_data == 0xC8)      //0xC8
 {
  UDR0 = IR8;
 }

//________ BATT VALUE + WHITE LINE SENSORS + SHARP SENSORS _________
 
 if (ser_data == 0x60)
 {
   UDR0 = BATT_VALUE;
 } 
 
 if (ser_data == 0x62)
 {
  UDR0 = SHARP_3;
 } 
 
 if (ser_data == 0x63)
 {
  UDR0 = SHARP_2;
 } 
 
 if (ser_data == 0x64)
 {
  UDR0 = WL_LEFT;
 }  
 
 if (ser_data == 0x65)
 {
  UDR0 = WL_CENTER;
 }  
 
 if (ser_data == 0x66)
 {
  UDR0 = WL_RIGHT;
 }  
 
 if (ser_data == 0x67)
 {
  UDR0 = SHARP_4;
 }
 
 if (ser_data == 0xC9)         //0xC9
 {
  UDR0 = SHARP_1;
 }
 
 if (ser_data == 0xCA)		   //0xCA
 {
  UDR0 = SHARP_5;
 }
  
//______________________Identity tag ____________________________
 if (ser_data == 0x6B)
 {
  UDR0 = robot_id;
 }

//________________  Buzzer _______________________________________
 if (ser_data == 0x69) //Turn on the buzzer
 {
  buzzer_on();
 }
 
 if (ser_data == 0x6A) //Turn off the buzzer
 {
  buzzer_off();
 }

}

//-------------------------------------------------------------------------------
//Main Function
//-------------------------------------------------------------------------------
int main(void)
{
	init_devices();

	lcd_cursor(1,3);
	lcd_string("FB5 GUI MODE");
	lcd_cursor(2,1);
	lcd_string("NEX-ROBOTICS IND");

	while(1)
	{
		sensor_data_interpretation();
		//print_sensor(1,1,1);				//Prints IR Proximity Sensor 1
	}

}
//-------------------------------------------------------------------------------


