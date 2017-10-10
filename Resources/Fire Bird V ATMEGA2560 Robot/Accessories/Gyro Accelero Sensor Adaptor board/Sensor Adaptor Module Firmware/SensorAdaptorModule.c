/********************************************************************************
 Platform: Fire Bird V ATMEGA2560 
 Experiment: Fire Bird V interfacing with the Sensor Adaptor Interfacing
 Written by: Vinod Desai, NEX Robotics Pvt. Ltd.
 Edited By: Sachitanand Malewar, NEX Robotics Pvt. Ltd.
 Last Modification: 29th Dec 2010
 AVR Studio Version 4.17, Build 666
 
 In this application ADC captures the analog sensor values form the Gyroscope and 
 Accelerometer sensor mounted on Sensor Adaptor Module and displayes it on the LCD

 Concepts covered:  Gyroscope, accelerometer, ADC, LCD interfacing

 ADC Connection:
 			  ACD CH.	PORT	Sensor

			  ADC6       PF6    Accelerometer analog output X axis (AX)	
              ADC7       PF7    Accelerometer analog output Y axis (AY)	
              ADC5       PF5    Accelerometer analog output Z axis (AZ)	
			  ADC14      PK6	Pitch Gyroscope analog output (GP)		
			  ADC15      PK7	Roll Gyroscope analog output (GR)	
			  ADC4       PF4	Yaw Gyroscope analog output (GY)		

 LCD Connections:
 			  LCD	  Microcontroller Pins
 			  RS  --> PC0
			  RW  --> PC1
			  EN  --> PC2
			  DB7 --> PC7
			  DB6 --> PC6
			  DB5 --> PC5
			  DB4 --> PC4

 Remove all J4 Jumpers from FBV ATMEGA2560 Adaptor Board

 Mount Sensor adaptor module: Put the jumpers on,

 PD, ST and Accelerometer range selectoion 
 AX,AY,AZ and GY on ADC/JATG connector 
 PD, ST and Accelerometer range selectoion 
 GY output at the Yaw Gyroscope output connector 
 connect 4P pitch Gyroscope output to ADC14 i.e pin no.1 of Sensor Pod Socket  
 connect 4R Roll Gyroscope output to ADC15 i.e pin no.2 of Sensor Pod Socket 
  
 LCD Display interpretation:
 *****************
 *AX	AY	  AZ *
 *GP    GR    GZ *
 *****************

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

   Copyright (c) 2010, NEX Robotics Pvt. Ltd.                       -*- c -*-
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

unsigned char ADC_Conversion(unsigned char);
unsigned char ADC_Value;

//Function to configure LCD port
void lcd_port_config (void)
{
 DDRC = DDRC | 0xF7;   //all the LCD pin's direction set as output
 PORTC = PORTC & 0x80; // all the LCD pins are set to logic 0 except PORTC 7
}

//ADC pin configuration
void adc_pin_config (void)
{
 DDRF = 0x00;  //set PORTF direction as input
 PORTF = 0x00; //set PORTF pins floating
}

//Function to Initialize PORTS
void port_init()
{
 lcd_port_config();
 adc_pin_config();	
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
unsigned char ADC_Conversion(unsigned char Ch)
{
 unsigned char a;
 if(Ch>7)
  {
	ADCSRB = 0x08;
  }
 Ch = Ch & 0x07;  			
 ADMUX= 0x20| Ch;	   		
 ADCSRA = ADCSRA | 0x40;	//Set start conversion bit
 while((ADCSRA&0x10)==0);	//Wait for ADC conversion to complete
 a=ADCH;
 ADCSRA = ADCSRA|0x10; //clear ADIF (ADC Interrupt Flag) by writing 1 to it
 ADCSRB = 0x00;
 return a;
}


// This Function prints the Analog Value Of Corresponding Channel No. at required Row
// and Coloumn Location. 
void print_sensor(char row, char coloumn,unsigned char channel)
{
	ADC_Value = ADC_Conversion(channel);
	lcd_print(row, coloumn, ADC_Value, 3);
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
	init_devices();
	
	lcd_set_4bit();
	lcd_init();
	
	while(1)
	{
		print_sensor(1,2,5);		//Print AX value 
		print_sensor(1,6,6);		//Print AY value
		print_sensor(1,10,7);		//Print AZ value

		print_sensor(2,2,14);		//Print GP value	
		print_sensor(2,6,15);		//Print GR value
		print_sensor(2,10,4);		//Print GY value
	}
}
