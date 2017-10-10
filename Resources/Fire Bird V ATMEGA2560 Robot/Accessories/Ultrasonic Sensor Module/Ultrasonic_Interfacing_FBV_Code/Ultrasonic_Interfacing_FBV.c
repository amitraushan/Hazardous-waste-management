/********************************************************************************
 Platform: Fire Bird V ATMEGA2560 
 Experiment: Fire Bird V interfacing with ultrasonic range sensor EZ4
 Written by: Vinod Desai, NEX Robotics Pvt. Ltd.
 Edited By: Sachitanand Malewar, NEX Robotics Pvt. Ltd.
 Last Modification: 8th June 2012
 AVR Studio Version 4.17, Build 666
 
 In this application ADC captures the analog sensor values form the Ultrasonic
 sensor mounted on the top acrylic plate of FBV displayes it on the LCD.

 Concepts covered: Ultrasonic Range Sensor EZ4, ADC and LCD interfacing

 ADC Connection:
 			  ACD CH.	PORT	Sensor

			  ADC15      PK7	Analog output from the EZ4 Sensor's AN pin		

 LCD Connections:
 			  LCD	  Microcontroller Pins
 			  RS  --> PC0
			  RW  --> PC1
			  EN  --> PC2
			  DB7 --> PC7
			  DB6 --> PC6
			  DB5 --> PC5
			  DB4 --> PC4

 Ultrasonic Triggering  --> PB4
              
  
 LCD Display interpretation:
 ******************
 *Nex Robotics    *
 *Dist: XXX.XXcm  *
 ******************

 Note: 
 
 1. Make sure that in the configuration options following settings are 
 	done for proper operation of the code

 	Microcontroller: atmega2560
 	Frequency: 14745600
 	Optimization: -O0 (For more information read section: Selecting proper optimization options 
					   below figure 2.22 in the FBV ATMEGA2560 software manual )

 2. Make sure that you copy the lcd.c file in your folder

*********************************************************************************/

/********************************************************************************

   Copyright (c) 2012, NEX Robotics Pvt. Ltd.                       -*- c -*-
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

#include <math.h> //included to support power function
#include "lcd.c"

                    
char first_row[17] = "Nex Robotics    ";
char second_row[17] = "Dist:    .  cm  ";

unsigned int ADC_Conversion(unsigned char);
unsigned int ADC_Value = 0;
float distance_in_cm = 0;

//Function to configure LCD port
void lcd_port_config (void)
{
 DDRC = DDRC | 0xF7;   //all the LCD pin's direction set as output
 PORTC = PORTC & 0x80; // all the LCD pins are set to logic 0 except PORTC 7
}

//ADC pin configuration
void adc_pin_config (void)
{
 DDRK = 0x7F;  //set PORTK7 direction as input
 PORTK = 0x00; //set PORTK7 pins floating
}

// ultarasonic trigger configuration
void ultrsonic_trigger_config(void)
{
 DDRB = DDRB | 0x10;   // PB4 direction set as output
 PORTB = PORTB & 0x00; // PB4 set to logic 0 
}

//Function to Initialize PORTS
void port_init()
{
 lcd_port_config();
 adc_pin_config();	
 ultrsonic_trigger_config();
}

//Function to Initialize ADC
void adc_init()
{
 ADCSRA = 0x00;
 ADCSRB = 0x00;		//MUX5 = 0
 ADMUX = 0x20;		//Vref=5V external --- ADLAR=1 --- MUX4:0 = 0000
 ACSR = 0x80;
 ADCSRA = 0x86;		//ADEN=1 --- ADIE=1 --- ADPS2:0 = 1 1 0
}

//This Function accepts the Channel Number and returns the corresponding Analog Value 
unsigned int ADC_Conversion(unsigned char Ch)
{
 //unsigned char a;
 unsigned int a = 0,b = 0;
 
 if(Ch>7)
  {
	ADCSRB = 0x08;
  }

 Ch = Ch & 0x07;  			
 ADMUX= 0x20| Ch;	   		
 ADCSRA = ADCSRA | 0x40;	    //Set start conversion bit
 while((ADCSRA&0x10)==0);	    //Wait for ADC conversion to complete
 b = (unsigned int)(ADCL>>6);   //read & adjust ADCL result to read as a right adjusted result
 a = (unsigned int)(ADCH<<2);   //read & adjust ADCH result to read as a right adjusted result
 a = a | b;                      
 ADCSRA = ADCSRA|0x10; //clear ADIF (ADC Interrupt Flag) by writing 1 to it
 ADCSRB = 0x00;
 return a;
}

// This Function prints the Analog Value Of Corresponding Channel No. at required Row
// and Coloumn Location. 
void print_sensor(char row, char coloumn,unsigned char channel)
{
 ADC_Value = ADC_Conversion(channel);
 lcd_print(row, coloumn, ADC_Value, 4);
}

//--------------------------------------------------------------------------------
// Ultrasonic sensor are connected in chaining mode. This function rise a 
// trigger pulse of >20usec to command ringing.     
//--------------------------------------------------------------------------------
void ultrasonic_trigger(void)                 
{
 PORTB = PORTB | 0x10;  // make high the Trigger input for Ultrasonic sensor
 _delay_us(50);         // Wait for >20usec
 PORTB = PORTB & 0xEF;  // make low the Trigger input for Ultrasonic sensor
}

void init_devices (void)
{
 cli(); //Clears the global interrupts
 port_init();
 adc_init();
 sei(); //Enables the global interrupts
}

//Main Function
int main(void)
{
 float distance_in_cm;
 unsigned int distance_in_cm_int;
 unsigned int distance_in_cm_fraction;

 init_devices();                  // initialise required modules
	
 lcd_set_4bit();                  // initialise LCD
 lcd_init();                      // initialise LCD

 lcd_string(first_row);           // display message on 1st row
 _delay_ms(150); 

 lcd_wr_command (0xC0);           // move cursor at start second row
 lcd_string(second_row);          // display message on 2nd row
 _delay_ms(150); 
	
 while(1)
  {
   ultrasonic_trigger();            // call ultrasonic triggering after enery 150msec  
    _delay_ms(150); 

   ADC_Value = ADC_Conversion(15);
 
   distance_in_cm = ADC_Value * 1.268;  // where, 5V/1024 = 0.00488/step & 9.85mV/2.54cm = 0.00385mV/cm
                                        // for distance in cm, we get 0.00488/0.00385 = 1.267 as const multiplier   

   distance_in_cm_int = floor(distance_in_cm);        // seperate integer part from float value
   distance_in_cm_fraction = (distance_in_cm * 100);  // seperate fractional part from float value

   lcd_print(2, 7, distance_in_cm_int, 3);            // print integer part on LCD
   lcd_print(2, 11, distance_in_cm_fraction, 2);       // print fractional part on LCD


  }
}
