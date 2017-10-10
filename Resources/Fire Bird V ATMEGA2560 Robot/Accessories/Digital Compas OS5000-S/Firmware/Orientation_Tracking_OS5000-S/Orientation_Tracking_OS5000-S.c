/********************************************************************************
 Platform:FireBirdV ATmega2560 Robot 
 Experiment: ATmega 2560 interfacing with the OS5000-S digital compass module 
             NEMA message reciving and interpretation. 
 Written by: Vinod Desai, NEX Robotics Pvt. Ltd.
 Edited By: Sachitanand Malewar, NEX Robotics Pvt. Ltd.
 Last Modification: 19th Nov 2011
 AVR Studio Version 4.17, Build 666
 
 Concepts covered: Using  OS5000-S digital compass to know the position, 
 LCD Interfacing, Use of and UART1.

 Note: This code is applicable to any GPS receiver module working on 9600bps.
   
 LCD Connections:
 			  LCD	  Microcontroller Pins
 			  RS  --> PC0
			  RW  --> PC1
			  EN  --> PC2
			  DB7 --> PC7
			  DB6 --> PC6
			  DB5 --> PC5
			  DB4 --> PC4
 
 
 OS5000-S digital compass connections:
 			  compass TXD --> uC RXD1 
			  compass RXD --> uC TXD1 
			  compass Vin --> 5V From pin no 13/14 of Servo Pod connector on Robot  
			  compass GND --> uC GND  pin no 11/12 of Servo Pod connector on Robot  
			  
 Logic :
 In the fimware,OHPR msg data are receiver in UART1 receive ISR and stored in buffer,
 Then various data such as Azimuth angle, X,Y,Z accelerometer, ang magnetometer etc.
 Azimutuh angle is dispalyed on LCD.

 Also the received data is echoed on UART2 of uC where the USB port for robot is connected.
  
 LCD Display interpretation:
 ******************
 *Azimuth XXX.X   *
 *                *
 ******************

 Note: 
 
 1. Make sure that in the configuration options following settings are 
 	done for proper operation of the code

 	Microcontroller: atmega2560
 	Frequency: 14745600
 	Optimization: -O0

 2. Make sure that you copy the lcd.c file in your folder

*********************************************************************************/

/********************************************************************************

   Copyright (c) 2011, NEX Robotics Pvt. Ltd.                       -*- c -*-
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

#include <math.h>       //included to support power function
#include <string.h>
#include "lcd.c"

#define FCPU 14745600ul //defined here to make sure that program works properly

char data;                                     // to rx current character data
unsigned char rx_start,rx_start1,rx_start2,ii; // to avoid re-writing over string
unsigned char pos,OHPRbite;							
char rx_string[150];                           // to store current packet
char temprory[4];                           
char *msgid[] = {                                                           
                  "OHPR",
				};

char azimuth[8] = "Azimuth";

char OHPRmsg[150];                            // to final store of the OHPR received packet 

char azimuth_angle[6];							// azimuth_angle = ddmm.mmmm							

void readazimuth_angle(void);

//function to extract the azimuth_angle from the OPHR data stored array
void readazimuth_angle(void)
{
 for(pos=0,OHPRbite=6;pos<5;pos++,OHPRbite++)
  { azimuth_angle[pos] = OHPRmsg[OHPRbite];}
 azimuth_angle[5] = '\0';
 
 if(azimuth_angle[2] == ','){azimuth_angle[4] = ' ';azimuth_angle[3] = ' ';azimuth_angle[2] = ' ';}
 if(azimuth_angle[3] == ','){azimuth_angle[4] = ' ';azimuth_angle[3] = ' ';}
 if(azimuth_angle[4] == ','){azimuth_angle[4] = ' ';}

 lcd_cursor(1,9);
 lcd_string(azimuth_angle);
}

//Function to configure LCD port
void lcd_port_config (void)
{
 DDRC = DDRC | 0xF7; //all the LCD pin's direction set as output
 PORTC = PORTC & 0x80; // all the LCD pins are set to logic 0 
}

// Function To Initialize UART1
// desired baud rate:9600
// actual baud rate:9600 (error 0.0%)
// char size: 8 bit
// parity: Disabled
void uart1_init(void)
{
 UCSR1B = 0x00; //disable while setting baud rate
 UCSR1A = 0x00;
 UCSR1C = 0x06;
 UBRR1L = 0x5F; //set baud rate lo
 UBRR1H = 0x00; //set baud rate hi
 UCSR1B = 0x90;
}

// Function To Initialize UART2
// desired baud rate:9600
// actual baud rate:9600 (error 0.0%)
// char size: 8 bit
// parity: Disabled
void uart2_init(void)
{
 UCSR2B = 0x00; //disable while setting baud rate
 UCSR2A = 0x00;
 UCSR2C = 0x06;
 UBRR2L = 0x5F; //set baud rate lo
 UBRR2H = 0x00; //set baud rate hi
 UCSR2B = 0x08;
}

// Function to Initialize PORTs
void port_init()
{
 lcd_port_config();	
}

void init_devices (void)
{
 cli();        //Clears the global interrupts
 port_init(); 
 uart1_init(); //Initialize UART1 for serial communication for OS5000-S digital compass to MCU
 uart2_init(); //Initialize UART2 for serial communication for MCU to PC for debugging(Onboard USB interface used)
 sei();        //Enables the global interrupts
}

// call this function extract OPHR data stored in buffer and display on LCD  
void rx_data_store(void)                  
{    
 strlcpy(OHPRmsg,rx_string,150);       // copy the OPHR data from Received string to extract the data
 lcd_cursor(1,1);
 lcd_string(azimuth);                  // display msg at 1st row 1st location on LCD
 readazimuth_angle();                  // call the funcion for reading and dispalying azimuth_angle
 rx_start2 = 0;  
}

SIGNAL(SIG_USART1_RECV) 	// ISR for receive complete interrupt
{
 data = UDR1; 				// making copy of data from UDR1 in 'data' variable 

 UDR2 = data; 				// echo data back to PC

 if(!(rx_start2)) 
  {
   if(data == 0x24)         // ASCII value of $      // is it start of packet  
	{   
     rx_start = 1;          // enable rx_start, to receive the next data
	 ii = 0;                // initialise it to 0 for array element 
	}		 
  }
 if(rx_start)               // is it enabled start rx'ing
  {
   rx_string[ii] = data;    // store received data in buffer array
   if(ii>0 && ii<5)         // varible to get postion in array while storing the "OHPR"  
    {
     temprory[ii-1] = data; // store "OHPR" in temprory array          
     if(ii==4)              // at 5th position in array, while storing data
      { 
       if(!(strcmp(temprory,msgid[0])))     // is stored data in temprory array is equal to "OHPR"?
        {
	     ii--;                              // take array position variable to one step back position
	     rx_start = 0;                      
	     rx_start1 = 1;                     // enable this variable for receiving next data from OHPR 
	     rx_start2 = 1; 
	    }
       else
        { 
         rx_start = 0;
		 rx_start1 = 0;
		 rx_start2 = 0;                     // disable this varaible to receive the new OPHR sentence 
		} 
      }
    }	   
    ii++;
  }
  
  if(rx_start1)                             // if eabled, start rx'ing the OPHPR packet
   {
	rx_string[ii] = data;                   // store the received data from OHPR msg in to Rx_string buffer array
    if(data==0x0A && rx_string[ii-1]==0x0D) // is it end of the receiving packet
     { 
      ii = 0;
      rx_start1 = 0;
      rx_start2 = 1;                        // disable this varaible to receive new OHPR sentence
     }
     ii++;                                  // incrment the array position 
   }   
}

//Main Function
int main(void)
{
 init_devices();                             // call for initialisation 	 

 lcd_set_4bit();                             // call for LCD initialisation for 4 bit mode
 lcd_init();								 // call for LCD command initialisation

 rx_start = 0;                               // initialise this variable to zero
 rx_start1 = 0;                              // initialise this variable to zero
 rx_start2 = 0;                              // initialise this variable to zero
 ii = 0;                                     // initialise this variable to zero
 while(1)
  {
   rx_data_store();                          // call for extracting and displaying Azimuth angle from 'rx_string' buffer
  }
}





