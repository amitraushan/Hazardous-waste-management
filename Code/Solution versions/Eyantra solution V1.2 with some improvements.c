/*
 * Eyantra_solution.c
 *
 * Created: 1/4/2016 1:49:05 AM
 *  Author: ABHISHEK ACHARYA :)
 */ 
//V1.2
//improvement in rotation(no longer rotate in only one direction)
//improvement in node traversal (Bot will select shortest path)

#define F_CPU 14745600
#define max_node 18
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdlib.h>

#include <math.h> //included to support power function
#include "lcd.h"

volatile unsigned long int l_encode = 0; //to keep track of left position encoder
volatile unsigned long int r_encode = 0; //to keep track of right position encoder
volatile unsigned int digital[16]; // digital value of 16 ADC channel
volatile long double xpos=0,ypos=0,tempx=0,tempy=0;
char *navi_pointer;
int navi=180;
int node_num=0,pre_node,r_node,found;
char graph[max_node][max_node];
char p[max_node+1],path[max_node+1];
int path_index=0;
int que_length=0;
char Area='\0',bridge='\0';
int A=1,B=1,C=1,IP=3;
int low_ccl;
char color[3]={'\0','\0','\0','\0'};
int task1=0,task2=0,task3=0;


struct queue    //first in first out system
{
	struct queue *next;
	struct queue *prev; 
	int num;
}*root=NULL,*end=NULL;


void struct_config(void)
{
	cli();
	def();
	graph[0][1]='N';
	graph[0][2]='S';
	graph[0][6]='E';
	graph[0][10]='W';
	graph[1][0]='S';
	graph[2][0]='N';
	graph[2][3]='S';
	graph[2][4]='W';
	graph[3][2]='N';
	graph[3][5]='W';
	graph[4][2]='E';
	graph[5][3]='E';
	graph[6][0]='W';
	graph[6][7]='N';
	graph[6][9]='S';
	graph[7][6]='W';
	graph[7][8]='E';
	graph[8][7]='N';
	graph[9][6]='W';
	graph[10][0]='E';
	graph[10][11]='N';
	graph[10][13]='S';
	graph[10][16]='W';
	graph[11][14]='E';
	graph[11][10]='S';
	graph[11][12]='N';
	graph[12][15]='W';
	graph[12][11]='S';
	graph[13][10]='N';
	graph[13][17]='W';
	graph[14][11]='W';
	graph[15][12]='E';
	graph[16][10]='E';
	graph[17][13]='E';
	sei();
}
void def(void)
{
	int i=0,j=0;
	for(i=0;i<=max_node;i++)
	for(j=0;j<=max_node;j++)
	graph[i][j]='\0';
}
int path_logic(int curr_node, int req_node) //path find main logic
{
	if(curr_node==req_node && found==0)
	{
		found++;
		return(1);
	}
	int j,r=0;
	for(j=0;j<max_node;j++)
	{
		if(graph[curr_node][j]!='\0' && pre_node!=j)
		{
			p[path_index]=graph[curr_node][j];
			path_index=path_index+1;
			pre_node=curr_node;
			r=r+path_logic(j,req_node);
		}
	}
	if(r>0)
	{
		return(1);
	}
	else
	{
		path_index--;
		return(0);
	}
}
void path_find(int req_node) //path find pre and post operations
{
	int i;
	if(path[path_index]=='\0')
	{
		cli();
		r_node=req_node;
		pre_node=node_num;
		for(i=0;i<max_node+1;i++)
		path[i]='\0';
		path_index=1;
		found=0;
		path_logic(node_num,req_node);
		path_index--;
		for(i=0;i<=max_node+1;i++)
		{
			path[i]='\0';
		}
		for(i=1;i<=path_index;i++)
		{
			path[i]=p[i];
		}
		path_index=1;
		i=1;
		while(path[path_index]!='\0')
		{
			if(path[i]=='E' && path[i+1]=='W')
			path_index=path_index+2;
			path[i]=path[path_index];
			i++;
			path_index++;
		}
		for(path_index=i;path_index<=max_node+1;path_index++)
		path[path_index]='\0';
		path_index=1;
		i=1;
		while(path[path_index]!='\0')
		{
			if(path[i]=='S' && path[i+1]=='N')
			path_index=path_index+2;
			path[i]=path[path_index];
			i++;
			path_index++;
		}
		for(path_index=i;path_index<=max_node+1;path_index++)
		path[path_index]='\0';
		path_index=1;
		sei();
	}
	else
	{
		enqueue(req_node);
	}
}	
void enqueue(int a)
{
	struct queue *newnode;
	if(root==NULL)
	{
		root=(struct queue*)malloc(sizeof(struct queue));
		root->num=a;
		root->next=NULL;
		root->prev=NULL;
		end=root;
	}
	else
	{
		newnode=(struct queue*)malloc(sizeof(struct queue));
		newnode->num=a;
		root->next=newnode;
		newnode->prev=root;
		root=root->next;
		newnode->next=NULL;
	}
	que_length++;
}
void enq_end(int a)
{
	struct queue *newnode;
	if(end==NULL)
	{
		end=(struct queue*)malloc(sizeof(struct queue));
		end->num=a;
		end->next=NULL;
		end->prev=NULL;
		root=end;
	}
	else
	{
		newnode=(struct queue*)malloc(sizeof(struct queue));
		newnode->num=a;
		end->prev=newnode;
		newnode->next=end;
		end=end->prev;
		newnode->prev=NULL;
	}
	que_length++;
}
void dequeue(void)
{
	if(que_length!=0)
	{
		struct queue *newnode;
		newnode=end;
		end=end->next;
		free(newnode);
		que_length--;
	}
	else
	return;
}
void lcd_port_config (void)
{
	cli();
	DDRC = DDRC | 0xF7; //all the LCD pin's direction set as output
	PORTC = PORTC & 0x80; // all the LCD pins are set to logic 0 except PORTC 7
	sei();
}
void buzzer(int a)
{
	DDRC=0xFF;  //Buzzer is connected to pin no 3 of port c
	if(a==1)
	PORTC=PORTC|0b00001000;
	else if(a==0)
	PORTC=PORTC|0b00000000;
}
void motion(char a)
{
	DDRA=DDRA | 0X0F; // Port A pin no 0,1,2,3 are for motion controlling
	if(a=='f')
	PORTA=0b00000110;
	else if(a=='b')
	PORTA=0b00001001;
	else if(a=='l')
	PORTA=0b00000101;
	else if(a=='r')
	PORTA=0b00001010;
	else if('s')
	PORTA=0x00;
}
void raftaar_config(void)
{
	cli();
	DDRA=DDRA|0x0F;
	DDRL=DDRL | 0b00011000; // declaring pin mo Pl3 and Pl4 as output
	PORTL=PORTL|0b00011000; // if oc pins are disconnected than output Will be one at these pins
	TCNT5=0x00FF; // initial value of counter
	TCCR5A=TCCR5A | 0b10101001; // Timer counter control register
	TCCR5B=TCCR5B | 0b00001011;
	raftaar(255,249);
	sei();
}
void raftaar(unsigned int l,unsigned int r) // a=for left motor and b for right motor
{
	cli();
	OCR5A=l;      // output compare register
	OCR5B=r;
	sei();
}
ISR(INT4_vect)//Interrupt subroutine
{
	int i=0;
	l_encode++;
	if(abs(navi)==360)
	navi=0;
	for(i=0;i<16;i++)
	digital[i]=getdata(i);		
}
ISR(INT5_vect) // Interrupt subroutine 
{
	int i=0;
	r_encode++;
	if(abs(navi)==360)
	navi=0;
	for(i=0;i<16;i++)
	digital[i]=getdata(i);		
}
void distance(char a,float cm,char c)
{
	tempx=xpos;
	tempy=ypos;
	raftaar(255,249);
	float b=0;
	r_encode=0; //clearing encoder value
	l_encode=0;
	while((ceil(b))<(cm))   
	{
		if(a=='f')
		{
			PORTA=0b00000110;
		}
		else if(a=='b')
		{
			PORTA=0b00001001;
			
		}
		b=0.5413*r_encode; // feedback constant value
	}
	stop();
	r_encode=0;
	r_encode=0;
	xpos=tempx;
	ypos=tempy;
	if(c=='w')
	{
		if(a=='f')
		{
		if(*navi_pointer=='E')
		xpos=xpos+cm;
		else if(*navi_pointer=='W')
		xpos=xpos-cm;
		else if(*navi_pointer=='N')
		ypos=ypos+cm;
		else if(*navi_pointer=='S')
		ypos=ypos-cm;
		
		}
	else if(a=='b')
		{
		if(*navi_pointer=='E')
		xpos=xpos-cm;
		else if(*navi_pointer=='W')
		xpos=xpos+cm;
		else if(*navi_pointer=='N')
		ypos=ypos-cm;
		else if(*navi_pointer=='S')
		ypos=ypos+cm;
		}
	}		
}
//Function to configure ports to enable robot's motion
void motion_pin_config (void)
{
	DDRA = DDRA | 0x0F;
	PORTA = PORTA & 0xF0;
	DDRL = DDRL | 0x18;   //Setting PL3 and PL4 pins as output for PWM generation
	PORTL = PORTL | 0x18; //PL3 and PL4 pins are for velocity control using PWM.
	OCR5A=0x00FF;
	OCR5B=0x00FF;
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

void port_init()
{
	motion_pin_config(); //robot motion pins config
	left_encoder_pin_config(); //left encoder pin config
	right_encoder_pin_config(); //right encoder pin config
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

void left (void) //Left wheel backward, Right wheel forward
{
	motion_set(0x05);
}

void right (void) //Left wheel forward, Right wheel backward
{
	motion_set(0x0A);
}

void soft_left (void) //Left wheel stationary, Right wheel forward
{
	motion_set(0x04);
}

void soft_right (void) //Left wheel forward, Right wheel is stationary
{
	motion_set(0x02);
}

void stop (void)
{
	motion_set(0x00);
	//_delay_ms(1000);
}

void angle_rotate(unsigned int Degrees)
{
	raftaar(255,249);
	float ReqdShaftCount = 0;
	unsigned long int ReqdShaftCountInt = 0;

	ReqdShaftCount = (float) Degrees/ 4.090; // division by resolution to get shaft count
	ReqdShaftCountInt = (unsigned int) ReqdShaftCount;
	r_encode = 0;
	l_encode = 0;
	double r,l;
	tempx=xpos;
	tempy=ypos;
	while (1)
	{
		if((r_encode >= ReqdShaftCountInt) | (l_encode >= ReqdShaftCountInt))
		break;
	}
	cli();
	stop(); //Stop robot
	r_encode=0;
	l_encode=0;
	xpos=tempx;
	ypos=tempy;
	sei();
	navigate();
}
void left_degrees(unsigned int Degrees)
{
	// 88 pulses for 360 degrees rotation 4.090 degrees per count
	navi=navi+Degrees;
	left(); //Turn left
	angle_rotate(Degrees);
}
void right_degrees(unsigned int Degrees)
{
	// 88 pulses for 360 degrees rotation 4.090 degrees per count
	navi=navi-Degrees;
	right(); //Turn right
	angle_rotate(Degrees);
}
void soft_left_degrees(unsigned int Degrees)
{
	navi=navi+Degrees;
	// 176 pulses for 360 degrees rotation 2.045 degrees per count
	soft_left(); //Turn soft left
	Degrees=Degrees*2;
	angle_rotate(Degrees);
}
void soft_right_degrees(unsigned int Degrees)
{
	navi=navi-Degrees;
	// 176 pulses for 360 degrees rotation 2.045 degrees per count
	soft_right();  //Turn soft right
	Degrees=Degrees*2;
	angle_rotate(Degrees);
}
void INT_position(void)
{
	cli(); //Clears the global interrupt
	port_init();  //Initializes all the ports
	EICRB = EICRB | 0x08; // INT5 is set to trigger with falling edge
	EIMSK = EIMSK | 0x20; // Enable Interrupt INT5 for right position encoder
	EICRB = EICRB | 0x02; // INT4 is set to trigger with falling edge
	EIMSK = EIMSK | 0x10; // Enable Interrupt INT4 for left position encoder
	sei();   // Enables the global interrupt
}
void servo_config(void)
{
	cli();
	TCNT1=0x0000; // set starting value of counter
	ICR1=1150; // set top value of counter
	TCCR1A=0b10101010; // counter configuring
	TCCR1B=0b00011100;
	DDRB=DDRB|0b11100000; // set OC pins as output pins
	PORTB=PORTB|0b1110000;
	OCR1A=1150; // to free the servo
	OCR1B=1150;
	OCR1C=1150;
	sei();
}
void servo1_mov(int degree)
{
	OCR1A=ceil(((degree/1.89)+25));
	_delay_ms(1100);
}
void servo2_mov(int degree)
{
	OCR1B=ceil(((degree/1.89)+25));
	_delay_ms(1100);
}
void servo3_mov(int degree)
{
	OCR1C=ceil(((degree/1.89)+25));
	_delay_ms(1100);
}
void servo1_free(void)
{
	OCR1A=1150;
}
void servo2_free(void)
{
	OCR1B=1150;
}
void servo3_free(void)
{
	OCR1C=1150;
}
void ADC_config(void)
{
	cli();
	TCCR4A=0x00;
	TCCR4B=0x02; // with clock setting 8
	TCCR4C=0x00;
	TIMSK4=0x01; // to enable timer overflow interrupt
	DDRF = 0x00; //set PORTF direction as input
	PORTF = 0x00; //set PORTF pins floating
	DDRK = 0x00; //set PORTK direction as input
	PORTK = 0x00; //set PORTK pins floating
	ADCSRA = 0x00;
	ADCSRB = 0x00;		//MUX5 = 0
	ADMUX = 0x20;		//Vref=5V external --- ADLAR=1 --- MUX4:0 = 0000
	ACSR = 0x80;
	ADCSRA = 0x86;		//ADEN=1 --- ADIE=1 --- ADPS2:0 = 1 1 0
}
int getdata(int Ch)
{
	int a;
	if(Ch>7)
	{
		ADCSRB = 0x08;
	}
	Ch = Ch & 0x07;
	ADMUX= 0x20| Ch;
	ADCSRA = ADCSRA | 0x40;		//Set start conversion bit
	while((ADCSRA&0x10)==0);	//Wait for ADC conversion to complete
	a=ADCH;
	ADCSRA = ADCSRA|0x10; //clear ADIF (ADC Interrupt Flag) by writing 1 to it
	ADCSRB = 0x00;
	return a;
}
int sharp(int adc_reading)
{
	float distance;
	int distanceInt;
	distance = (int)(10.00*(2799.6*(1.00/(pow(adc_reading,1.1546)))));
	distanceInt = (int)distance;
	if(distanceInt>800)
	{
		distanceInt=800;
	}
	return distanceInt;
}
void navigate(void) //To navigate robot
{
	cli();
	if(navi==0)
	navi_pointer="East ";
	else if(abs(navi)==270)
	{
		if(navi>0)
		navi_pointer="South";
		else
		navi_pointer="North";
	}
	else if(abs(navi)==180)
	{
		navi_pointer="West ";
	}
	else if(abs(navi)==90)
	{
		if(navi>0)
		navi_pointer="North";
		else
		navi_pointer="South";
	}
	lcd_cursor(2,12);
	lcd_string(navi_pointer);
	lcd_cursor(2,1);
	lcd_string("Node:-");
	lcd_print(2,7,node_num,2);
	sei();
}
ISR(TIMER4_OVF_vect) // Timer 2 overflow interrupt to get digital value 
{
	int i=0;
	for(i=0;i<16;i++)
	digital[i]=getdata(i);
	if(node_num<10)
	Area='C';
	else
	Area='I';
}
void line_following(void) //right=digital[1];middel=digital[2];left=digital[3]
{
	if( digital[2]>50)
	{
		stop();
		node_behave(node_num,*navi_pointer);
	}
	else if(digital[1]>15 && digital[3]<15)
	{
		raftaar(180,174);
		right();
	}
	else if(digital[1]<15 && digital[3]>15)
	{
		raftaar(180,174);
		left();
	}
	else if(digital[1]>20 && digital[2]>20 && digital[3]<15)
	{
		while(digital[3]<15)
		soft_right();
	}
	else
	{
		raftaar(255,249);
		motion('f');
	}
}
void node_behave(void) //How the robot behave when it detects a node
{
	distance('f',10,'o');
	navigate();
	if(path[path_index]!='\0')
	{
		if((*navi_pointer-path[path_index])==-9 || (*navi_pointer-path[path_index])==4 || (*navi_pointer-path[path_index])==14) //-9,4,14 are ASCII values for 270 degree while rotating in rightwards
			while(*navi_pointer!=path[path_index])
				left_degrees(90);	
		else if(abs(*navi_pointer-path[path_index])==5 || abs(*navi_pointer-path[path_index])==18)
			while(*navi_pointer!=path[path_index])
				right_degrees(180);
		else
			while(*navi_pointer!=path[path_index])
			right_degrees(90);
		path_index++;
	}
	else
	{
		node_num=r_node;
		distance('b',10,'o');
		navigate();
		if(que_length!=0)
		{
			path_find(end->num);
			dequeue();
		}
		else
		{	buzzer(1);
			_delay_ms(3000);
			buzzer(0);
			exit(1);			
		}		
	}		
} 





int main(void)
{
	long int i=0;
	struct_config();
	lcd_port_config();
	lcd_init();
	INT_position();
	raftaar_config(); 
	servo_config();
	ADC_config();
	navigate();
	while(1)
	{
		line_following();
	}
}
