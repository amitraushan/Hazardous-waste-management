/*
*
* Team Id: 											eYRC-HD#1684
* Author List: 										Abhishek Acharya, Amit Raushan, Jai, Shubham Baranwal
* Filename: 										Eyantra solution.c
* Theme: 											Hazardous Waste Disposal
* Functions: 										struct_config(), def(), path_logic(int, int), path_find(int), enqueue(int), enq_end(int), dequeue(), buzzer(int), lcd_port_config(), 
													motion(char), sppeed_config(), sppeed(unsigned int, unsigned int), distance(char, float, char), motion_pin_config(), left_encoder_pin_config(),
													right_encoder_pin_config(), port_init(), motion_set(unsigned char), left(), right(), soft_left(), soft_right(), stop(), angle_rotate(unsigned int),
													left_degrees(unsigned int), right_degrees(unsigned int), soft_left_degrees(unsigned int), soft_right_degrees(unsigned int), INT_position(), servo_config(),
													servo1_mov(int), servo2_mov(int), servo1_free(), servo2_free(), pick_waste(), ADC_config(), getdata(int), sharp(int), navigate(), line_following(),
													node_behave(), color_sensor_pin_config(), clrport_init(), color_sensor_pin_interrupt_init(), init_devices(), filter_red(), filter_green(), filter_blue(),
													color_sensor_scaling(), red_read(), green_read(), blue_read(), color_sensor(), main()
* Global Variables:									left_position_encode, right_position_encode, servo_angle_1, servo_angle_2, navigation_pointer, navigation_in_degrees, node_number,
													previous_node, destinationn_node, found, color_found, graph[], p[], path[][], path_index, que_length, Area, bridge, color_identified,
													deposition_zone_A, deposition_zone_B, deposition_zone_C, IP, color[], color_deposited[], weight_block_position_1, weight_block_position_2, 
													weight_block_position_3, weight_block_position_4, weight_block_position_5, pulse, red, blue, green, queue
*
*/

#define __OPTIMIZE__ -O0
#define F_CPU 14745600
#define maximum_number_of_nodes 25
#define threshold_color_value 1300
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdlib.h>
#include <math.h>																		//included to support power function
#include "lcd.h"
volatile unsigned long int left_position_encode = 0;									//to keep track of left position encoder
volatile unsigned long int right_position_encode = 0;									//to keep track of right position encoder
volatile unsigned int digital[16];														// digital value of 16 ADC channel continuously updated by the TIMER4 Overflow Interrupt
volatile int servo_angle_1=180,servo_angle_2=180; //									//servo_angle_1 is the angle rotated by the servo 1 and servo_angle_2 is the angle rotated by the servo 2
char *navigation_pointer;																//navigation_pointer points the direction string of the bot
int navigation_in_degrees=180;															//navigation_in_degrees keep tracks of direction in degrees
int node_number=0 ,previous_node, destinationn_node=0, found, color_found=0;			//node_number is the current node, previous_node keep tracks of the previous node in path_find() function, 
																						//destinationn_node is the node where the bot is expected to traverse, found checks whether the node is found or not, color_found is used for checking the flag beforehand
char graph[maximum_number_of_nodes][maximum_number_of_nodes];							//graph[][] constructs the arena in the form of graph
char p[maximum_number_of_nodes+1],path[maximum_number_of_nodes+1];						//p[] is the original string of the path updated by the path_logic function (without filtration)
																						//path[] is the modified string of the path updated by path_find() function (with filtration)
int path_index=0;																		//path_index provides the index of the path string
int que_length=0;																		//que_length gives the length of the queue
char Area='\0', bridge='\0', color_identified='\0';										//Area locates the position of the bot (whether the bot is in CA or IA)
																						//bridge detects in which area the bridge is tilted
																						//color_identified is the color detected by the color sensor
int deposition_zone_A=1, deposition_zone_B=1, deposition_zone_C=1, IP=3;				//deposition_zone_A denotes the presence of waste block on deposition zone A
																						//deposition_zone_B denotes the presence of waste block on deposition zone B
																						//deposition_zone_C denotes the presence of waste block on deposition zone C
char color[3],color_deposited[3];														//color stores the color of the waste block in CA and color_deposited stores the color of the flag in IA
int color_index=0;																		//color_index is the index of the array color[3]

int weight_block_position_1=2, weight_block_position_2=2, weight_block_position_3=2, weight_block_position_4=2, weight_block_position_5=2;
																						//weight_block_position_1 denotes number of weight block present at w1
																						//weight_block_position_2 denotes number of weight block present at w2
																						//weight_block_position_3 denotes number of weight block present at w3
																						//weight_block_position_4 denotes number of weight block present at w4
																						//weight_block_position_5 denotes number of weight block present at w5
volatile unsigned long int pulse = 0;													//to keep the track of the number of pulses generated by the color sensor
volatile unsigned long int red;															// variable to store the pulse count when read_red function is called
volatile unsigned long int blue;														// variable to store the pulse count when read_blue function is called
volatile unsigned long int green;
int check=0;														// variable to store the pulse count when read_green function is called

struct queue																			//queue implemented by the linked list 
{
	struct queue *next;
	struct queue *prev;
	int num;
}*root=NULL,*end=NULL;

/*
*
* Function Name:	struct_config 
* Input:			NONE
* Output:			NONE
* Logic:			All the nodes of the arena is fitted in the form of graph.
					Their direction is also defined.
					
										   |w3|											   |SZ|
							
							 _														 __
							|A|----#(15)-------#(12)								|w1|		#(1)              #(21)
												|												|				  |
												|				___								|				  |									# :-	node
											(11)#------#(14)  |DZ1|________________				|		 (19)#___#(7)___#(20)						SZ :-	Sorting zone 
												|			   |					|			|			|			|							
							 _					|			   |		Bridge		|			|			|	____	|
							|B|----#(16)-------#(10)-----------|--------------------|-----------#(0)------#(6)	|IP	|	#(8)-----#(22)
												|			   |					|			|			|			|
												|			   |________________	|			|			|#___#(9)___|								N
												|								|DZ2|	#(4)----#(2)	 (18)	  |									    |
												|												|				  |								  W<----|---->E
							 _					|		 __						 __				|				  |									    |
							|C|----#(17)-------#(13)	|w4|					|SZ|	#(5)----#(3)			  #(23)									S
							
												 __												 __
												|w5|											|w2|
										
										fig: Graph of arena defined in the form of the node number
											
* Example Call:		struct_config();
*
*/

void struct_config(void)
{
	cli();                       // Clears the global interrupt
	def();						 // User defined function which gives null value to all the nodes of the arena
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
	graph[6][19]='N';
	graph[6][18]='S';
	graph[7][19]='W';
	graph[7][20]='E';
	graph[8][20]='N';
	graph[9][18]='W';
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
	graph[19][7]='E';
	graph[19][6]='S';
	graph[18][6]='N';
	graph[18][9]='E';
	graph[20][8]='S';
	graph[20][7]='W';
	graph[7][21]='N';
	graph[8][22]='E';
	graph[9][23]='S';
	graph[21][7]='S';
	graph[22][8]='W';
	graph[23][9]='N';
	sei();					//Enables all the global interrupt
}

/*
*
* Function Name:	def
* Input:			NONE
* Output:			NONE
* Logic:			All the nodes of the arena are accessed and given null value (more specifically null character)
* Example Call:		def();
*
*/

void def(void)
{
	int i=0,j=0;
	for(i=0;i<=maximum_number_of_nodes;i++)	//i: will iterate from 0 to the maximum nodes (maximum_number_of_nodes is the constant for the maximum number of nodes in the arena )
	for(j=0;j<=maximum_number_of_nodes;j++)	//j: will iterate from 0 to the maximum nodes (maximum_number_of_nodes is the constant for the maximum number of nodes in the arena )
	graph[i][j]='\0';
}

/*
*
* Function Name:	path_logic
* Input:			curr_node -> integer which gives the current node on which the bot is there
					req_node  -> integer which gives the destination node to which the bot has to traverse
* Output:			Updates the string (in terms of direction) to which the bot is instructed to follow
* Logic:			If the current node and the required node is same, then the function returns 1 and updates the path string with null character.
					If above case is not matched, then function starts checking which are nodes connected between current node and destination node. 
					According to that it updates the path string(global variable)
* Example Call:		path_logic(7,10);
*
*/

int path_logic(int curr_node, int req_node) 
{
	if(curr_node==req_node && found==0)  //found is the variable to check the bot's traversal
	{
		found++;
		return(1);
	}
	int j,r=0;							//r stores the value of return
	
	//j: will iterate from 0 to the maximum nodes (maximum_number_of_nodes is the constant for the maximum number of nodes in the arena )
	for(j=0;j<maximum_number_of_nodes;j++)
	{
		if(graph[curr_node][j]!='\0' && previous_node!=j)
		{
			p[path_index]=graph[curr_node][j];  //path index -> index of the path string
			path_index=path_index+1;
			previous_node=curr_node;					//previous_node  -> previous node
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

/*
*
* Function Name:		path_find
* Input:				req_node  -> integer which gives the destination node to which the bot has to traverse
* Output:				Modifies the string which is given by the above function (path_logic())
* Logic:				Filters the Counter direction. for example if the string gives North-South simultaneously then it removes both the direction.
						If the path string is fully occupied then it stores the node which the bot will traverse in the queue.
* Example Call:			path_find(10);
*
*/

void path_find(int req_node) 
{
	int i; 
	if(node_number==8 && req_node==23) // special case for the traversal of the bot from node 8 to node 23.
	{
		path[1]='S';
		path[2]='W';
		path[3]='S';
		path[4]='\0';
		path_index=1;
	}
	
	else if(path[path_index]=='\0')
	{
		cli();
		destinationn_node=req_node;
		previous_node=node_number;
		
		//i: will iterate from 0 to the maximum nodes (maximum_number_of_nodes is the constant for the maximum number of nodes in the arena )
		for(i=0;i<maximum_number_of_nodes+1;i++)
		path[i]='\0';
		path_index=1;
		found=0;
		path_logic(node_number,req_node);
		path_index--;
		
		//i: will iterate from 0 to the maximum nodes (maximum_number_of_nodes is the constant for the maximum number of nodes in the arena )
		for(i=0;i<=maximum_number_of_nodes+1;i++)
		{
			path[i]='\0';
		}
		
		//i: will iterate from 1 to the length of the path index
		for(i=1;i<=path_index;i++)  
		{
			path[i]=p[i];
		}
		path_index=1;
		i=1;
		
		
		while(path[path_index]!='\0')		//Filters the counter direction West and East   
		{
			if((path[i]=='E' && path[i+1]=='W') || (path[i]=='W' && path[i+1]=='E'))
			path_index=path_index+2;
			path[i]=path[path_index];
			i++;
			path_index++;
		}
		
		//path_index: will iterate from i to the maximum number of nodes
		for(path_index=i;path_index<=maximum_number_of_nodes+1;path_index++)		//only produces that string which is required for traversal and other elements are given null value
		path[path_index]='\0';
		path_index=1;
		i=1;
		
		while(path[path_index]!='\0')		//Filters the counter direction South and North
		{
			if((path[i]=='S' && path[i+1]=='N') || (path[i]=='N' && path[i+1]=='S'))
			path_index=path_index+2;
			path[i]=path[path_index];
			i++;
			path_index++;
		}
		
		for(path_index=i;path_index<=maximum_number_of_nodes+1;path_index++)
		path[path_index]='\0';
		
		path_index=1;
		sei();
	}
	else
	{
		enqueue(req_node);   //If the path string is fully occupied then it stores the node( which the bot will traverse) in the queue.
	}
}

/*
*
* Function Name:		enqueue
* Input:				node_added -> integer in the form of node which is to be queued
* Output:				Adds the node in the global queue implemented by linked list from the end
* Logic:				Follows the principle of doubly linked list and insert the nodes from the end
* Example Call:			enqueue(10);
*
*/

void enqueue(int node_added)
{
	struct queue *newnode;
	if(root==NULL)
	{
		root=(struct queue*)malloc(sizeof(struct queue));
		root->num=node_added;
		root->next=NULL;
		root->prev=NULL;
		end=root;
	}
	else
	{
		newnode=(struct queue*)malloc(sizeof(struct queue));
		newnode->num=node_added;
		root->next=newnode;
		newnode->prev=root;
		root=root->next;
		newnode->next=NULL;
	}
	que_length++;			//que_length -> length of the queue
}

/*
*
* Function Name:			enq_end
* Input:					node_added_start -> integer in the form of node which is to be queued	
* Output:					Adds the node in the global queue implemented by linked list from the beginning
* Logic:					Follows the principle of doubly linked list and insert the nodes from the beginning.
							Follows the abstract data type , Priority Queue
* Example Call:				enq_end(10);
*
*/

void enq_end(int node_added_start)
{
	struct queue *newnode;
	if(end==NULL)
	{
		end=(struct queue*)malloc(sizeof(struct queue));
		end->num=node_added_start;
		end->next=NULL;
		end->prev=NULL;
		root=end;
	}
	else
	{
		newnode=(struct queue*)malloc(sizeof(struct queue));
		newnode->num=node_added_start;
		end->prev=newnode;
		newnode->next=end;
		end=end->prev;
		newnode->prev=NULL;
	}
	que_length++;
}

/*
*
* Function Name:		dequeue
* Input:				NONE
* Output:				Deletes the queue element
* Logic:				Deletion of doubly linked list from the beginning
* Example Call:			dequeue();
*
*/

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

/*
*
* Function Name:		buzzer
* Input:				a ->  integer through which buzzer is turned ON or OFF
* Output:				Buzzer is turned on and continuous beep sound comes
						Buzzer is turned off
* Logic:				If a is equal to 1 then pin number 3 of port C is given high logic and continuous beep sound comes
						If a is equal to 0 then buzzer is turned off
* Example Call:			buzzer(1);
*
*/

void buzzer(int a)
{
	DDRC=0xFF;			  //Buzzer is connected to pin no 3 of port c
	if(a==1)
	PORTC=PORTC|0b00001000;
	else if(a==0)
	PORTC=PORTC & 0b11110111;
}

/*
*
* Function Name:		lcd_port_config
* Input:				NONE
* Output:				All the pins of port C (except pin 0) are set to the output pins 
* Logic:				1 denotes to the output pin and 0 denotes to the input pin
* Example Call:			lcd_port_config();
*
*/

void lcd_port_config (void)
{
	cli();
	DDRC = DDRC | 0xF7;		 //all the LCD pin's direction set as output
	PORTC = PORTC & 0x80;	 // all the LCD pins are set to logic 0 except PORTC 7
	sei();
}

/*
*
* Function Name:  				motion
* Input:						movement -> character which denotes the motion of the bot 
* Output:						depending upon the value of a, it assigns the value to the Register A 
* Logic:						If movement is equal to 'f', then the bot moves in forward direction
								If movement is equal to 'b', then the bot moves in backward direction
								If movement is equal to 'r', then the bot rotates in right direction
								If movement is equal to 'l', then the bot rotates in left direction
* Example Call:					motion('f');
*
*/

void motion(char movement)
{
	DDRA=DDRA | 0X0F; // Port A pin no 0,1,2,3 are for motion controlling
	if(movement=='f')
	PORTA=0b00000110;
	else if(movement=='b')
	PORTA=0b00001001;
	else if(movement=='l')
	PORTA=0b00000101;
	else if(movement=='r')
	PORTA=0b00001010;
	else if('s')
	PORTA=0x00;
}

/*
*
* Function Name:			sppeed_config
* Input:					NONE
* Output:					Sets the input/output pin configuration required for the speed of the bot
* Logic:					1 sets the pin as output and 0 for input
* Example Call:				sppeed_config();
*
*/

void sppeed_config(void)
{
	cli();
	DDRA=DDRA|0x0F;
	DDRL=DDRL | 0b00011000;			// declaring pin mo Pl3 and Pl4 as output
	PORTL=PORTL|0b00011000;			// if Output Compare pins are disconnected than output Will be one at these pins
	TCNT5=0x00FF;					// initial value of counter
	TCCR5A=TCCR5A | 0b10101001;		// Timer counter control register
	TCCR5B=TCCR5B | 0b00001011;
	sppeed(255,249);
	sei();
}

/*
*
* Function Name:			sppeed
* Input:					l -> intger that gives the digital value of PWM channel which is connected to the left channel
							r -> intger that gives the digital value of PWM channel which is connected to the right channel
* Output:					adjusts the speed of the d.c motors
* Logic:					Output compare registers A and B are given value to generate different duty cycle of PWM
* Example Call:				sppeed(180,180);
*
*/

void sppeed(unsigned int l,unsigned int r) 
{
	cli();
	OCR5A=l;		// OCR5A = for left motor
	OCR5B=r;		// OCR5B = for right motor
	sei();
}

/*
*
* Interrupt Service Routine Name:			ISR(INT4_vect)
* Input:									Negative edge of pulse receiving from left position encoder
* Output:									Continuously updates the value of left position encoder and digital array.
* Logic:									Whenever Interrupt service routine is called, the variable left_position_encoder gets incremented by 1
* Example Call:								NONE
*
*/

ISR(INT4_vect)
{
	int i=0;
	left_position_encode++;
	if(abs(navigation_in_degrees)==360)
	navigation_in_degrees=0;
	//i: will iterate from 0 to the channel length
	for(i=0;i<16;i++)
	digital[i]=getdata(i);
}

/*
*
* Interrupt Service Routine Name:			ISR(INT5_vect)
* Input:									Negative edge of pulse receiving from right position encoder
* Output:									Continuously updates the value of right position encoder and digital array.
* Logic:									Whenever Interrupt service routine is called, the variable right_position_encoder gets incremented by 1
* Example Call:								NONE
*
*/

ISR(INT5_vect) 
{
	int i=0;
	right_position_encode++;
	if(abs(navigation_in_degrees)==360)
	navigation_in_degrees=0;
	//i: will iterate from 0 to the channel length
	for(i=0;i<16;i++)
	digital[i]=getdata(i);
}

/*
*
* Function Name:			distance
* Input:					movement -> character which denotes the motion of the bot
							cm ->	float which tells the bot to move a particular distance	
* Output:					Moves the bot for a particular distance in a particular direction (either forward or backward)
* Logic:					By multiplying position encoder value with feedback constant(0.5413),the function calculates the distance which is to be
							moved by the bot. After that bot stops. 
* Example Call:				distance('f',11,'o');
*
*/

void distance(char a,float cm,char iop)
{
	sppeed(255,249);
	float b=0;
	right_position_encode=0; //clearing encoder value
	left_position_encode=0;
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
		b=0.5413*right_position_encode; // feedback constant value
	}
	stop();
}

//Function to configure ports to enable robot's motion

/*
*
* Function Name:		motion_pin_config		
* Input:				NONE
* Output:				Sets pin configuration for the motion of the bot
* Logic:				1 sets the pin as output and 0 for input
* Example Call:			motion_pin_config();
*
*/

void motion_pin_config (void)
{
	DDRA = DDRA | 0x0F;
	PORTA = PORTA & 0xF0;
	DDRL = DDRL | 0x18;			//Setting PL3 and PL4 pins as output for PWM generation
	PORTL = PORTL | 0x18;		//PL3 and PL4 pins are for velocity control using PWM.
	OCR5A=0x00FF;
	OCR5B=0x00FF;
}


/*
*
* Function Name:			left_encoder_pin_config
* Input:					NONE
* Output:					Sets pin configuration of the left encoder
* Logic:					1 sets the pin as output and 0 for input
* Example Call:				left_encoder_pin_config();
*
*/

void left_encoder_pin_config (void)
{
	DDRE  = DDRE & 0xEF;  //Set the direction of the PORTE 4 pin as input
	PORTE = PORTE | 0x10; //Enable internal pull-up for PORTE 4 pin
}

/*
*
* Function Name:			right_encoder_pin_config
* Input:					NONE
* Output:					Sets pin configuration of the right encoder
* Logic:					1 sets the pin as output and 0 for input
* Example Call:				right_encoder_pin_config();
*
*/

void right_encoder_pin_config (void)
{
	DDRE  = DDRE & 0xDF;		//Set the direction of the PORTE 4 pin as input
	PORTE = PORTE | 0x20;		//Enable internal pull-up for PORTE 4 pin
}

/*
*
* Function Name:			port_init
* Input:					NONE
* Output:					Calls motion_pin_config(), left_encoder_pin_config(), right_encoder_pin_config()
* Logic:					NONE
* Example Call:				port_init();
*
*/

void port_init()
{
	motion_pin_config(); //robot motion pins config
	left_encoder_pin_config(); //left encoder pin config
	right_encoder_pin_config(); //right encoder pin config
}

/*
*
* Function Name:		motion_set
* Input:				Direction -> character which sets the Register value of Port A
* Output:				It gives different sets of motion of the bot
* Logic:				1 sets the pin as output and 0 for input
* Example Call:			motion_set(0x05);
*
*/

void motion_set (unsigned char Direction)
{
	unsigned char PortARestore = 0;

	Direction &= 0x0F; 						// removing upper nibble for the protection
	PortARestore = PORTA; 					// reading the PORTA original status
	PortARestore &= 0xF0; 					// making lower direction nibble to 0
	PortARestore |= Direction;				// adding lower nibble for forward command and restoring the PORTA status
	PORTA = PortARestore; 					// executing the command
}

/*
*
* Function Name:			left
* Input:					NONE
* Output:					Left wheel moves backward, Right wheel moves forward			
* Logic:					NONE
* Example Call:				left();
*
*/

void left (void) 
{
	motion_set(0x05);
}

/*
*
* Function Name:			right
* Input:					NONE
* Output:					Left wheel moves forward, Right wheel moves backward
* Logic:					NONE
* Example Call:				right();
*
*/

void right (void) 
{
	motion_set(0x0A);
}

/*
*
* Function Name:			soft_left
* Input:					NONE
* Output:					Left wheel remains stationary, Right wheel moves forward
* Logic:					NONE
* Example Call:				soft_left();
*
*/

void soft_left (void) 
{
	motion_set(0x04);
}

/*
*
* Function Name:			soft_right
* Input:					NONE
* Output:					Left wheel moves forward, Right wheel remains stationary
* Logic:					NONE
* Example Call:				soft_right();
*
*/

void soft_right (void) 
{
	motion_set(0x02);
}

/*
*
* Function Name:			stop
* Input:					NONE
* Output:					Stops the bot
* Logic:					NONE
* Example Call:				stop();
*
*/

void stop (void)
{
	motion_set(0x00);
	//_delay_ms(1000);
}

/*
*
* Function Name:			angle_rotate
* Input:					Degrees -> the integer value of degree to which bot has to rotate 
* Output:					The bot will rotate to a particular angle
* Logic:					Function will divide its input with a constant and calculates required value of position encoder (Reqdshaftcount=degree/4.090), now the bot will rotate  
							till it achieves required value of position encoder
* Example Call:				angle_rotate(90);
*
*/

void angle_rotate(unsigned int Degrees)
{
	sppeed(255,249);  //sets the speed of motors
	float ReqdShaftCount = 0;
	unsigned long int ReqdShaftCountInt = 0;
	if(node_number==15 || node_number==16 || node_number==17)
	ReqdShaftCount = (float) Degrees/ 4.60;
	else
	ReqdShaftCount = (float) Degrees/ 4.090; // division by resolution to get shaft count
	ReqdShaftCountInt = (unsigned int) ReqdShaftCount;
	right_position_encode = 0;
	left_position_encode = 0;
	while (1)
	{
		if((right_position_encode >= ReqdShaftCountInt) | (left_position_encode >= ReqdShaftCountInt))
		break;
	}
	stop(); //Stop robot
	navigate();   //navigate the bot after rotating
}

/*
*
* Function Name:			left_degrees
* Input:					Degrees -> integer that instructs the bot to move to in a particular degree in left direction
* Output:					The bot rotates in given degree in left direction
* Logic:					Firstly the bot turns left and starts rotating as per the given degree and for error rejection purpose the bot is continued to get rotated till it detects the black line
* Example Call:				left_degrees(90);
*
*/

void left_degrees(unsigned int Degrees)
{
	// 88 pulses for 360 degrees rotation 4.090 degrees per count
	navigation_in_degrees = navigation_in_degrees + Degrees;
	left(); //Turn left
	angle_rotate(Degrees);
	while(digital[2]<15 && digital[1]<15 && digital[3]<15)
	{
		sppeed(255,249);
		if(node_number==10 || node_number==0)
		sppeed(180,174);
		left();
	}
	stop();
}

/*
*
* Function Name:			right_degrees
* Input:					Degrees -> integer that instructs the bot to move to in a particular degree in left direction
* Output:					The bot rotates in given degree in right direction
* Logic:					Firstly the bot turns right and starts rotating as per the given degree and for error rejection purpose the bot is continued to get rotated till it detects the black line
* Example Call:				right_degrees(90);
*
*/

void right_degrees(unsigned int Degrees)
{
	// 88 pulses for 360 degrees rotation 4.090 degrees per count
	navigation_in_degrees=navigation_in_degrees-Degrees;
	right(); //Turn right
	angle_rotate(Degrees);
	while(digital[2]<15 && digital[3]<15 && digital[1]<15)
	{
		sppeed(255,249);
		if(node_number==10 || node_number==0)
		sppeed(180,174);
		right();
	}
	stop();
}

/*
*
* Function Name:			soft_left_degrees
* Input:					Degrees -> integer that instructs the bot to move to in a particular degree in left direction
* Output:					The bot rotates in given degree in left direction slowly
* Logic:					Firstly the bot turns left and starts rotating as per the given degree 
* Example Call:				soft_left_degrees(90);
*
*/

void soft_left_degrees(unsigned int Degrees)
{
	navigation_in_degrees=navigation_in_degrees+Degrees;
	// 176 pulses for 360 degrees rotation 2.045 degrees per count
	soft_left(); //Turn soft left
	Degrees=Degrees*2;
	angle_rotate(Degrees);
}

/*
*
* Function Name:			soft_left_degrees
* Input:					Degrees -> integer that instructs the bot to move to in a particular degree in left direction
* Output:					The bot rotates in given degree in right direction slowly
* Logic:					Firstly the bot turns left and starts rotating as per the given degree
* Example Call:				soft_right_degrees(90);
*
*/

void soft_right_degrees(unsigned int Degrees)
{
	navigation_in_degrees=navigation_in_degrees-Degrees;
	// 176 pulses for 360 degrees rotation 2.045 degrees per count
	soft_right();  //Turn soft right
	Degrees=Degrees*2;
	angle_rotate(Degrees);
}

/*
*
* Function Name:			INT_position
* Input:					NONE
* Output:					NONE
* Logic:					Using the value of Special Function Register, the function configures External Interrupt 5 and 4 connected to the left and right position encoder
* Example Call:				INT_position();
*
*/

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

/*
*
* Function Name:					servo_config
* Input:							NONE
* Output:							NONE
* Logic:							Configures the Timer Counter Modes and Operation using Special Function Register
* Example Call:						servo_config();
*
*/

void servo_config(void)
{
	cli();
	TCNT1=0x0000;							// set starting value of counter
	ICR1=1150;								// set top value of counter
	TCCR1A=0b10101010;						// counter configuring
	TCCR1B=0b00011100;
	DDRB=DDRB|0b11100000;					// set Output Compare pins as output pins
	PORTB=PORTB|0b11100000;
	OCR1A=1150;								// to free the servo
	OCR1B=1150;
	OCR1C=1150;
	sei();
}

/*
*
* Function Name:				servo1_mov
* Input:						degree -> integer that instructs the servo 1 to rotate in a particular degree
* Output:						Servo 1 rotates in a particular degree
* Logic:						Changes the Output Compare Register 1A accordingly to generate Standard servo signal
* Example Call:					servo1_mov(10);
*
*/

void servo1_mov(int degree)
{

	if(node_number==13)
	OCR1A=((degree/1.89)+25);
	else
	OCR1A=ceil(((degree/1.89)+25));
	servo_angle_1=degree;
}

/*
*
* Function Name:				servo2_mov
* Input:						degree -> integer that instructs the servo 2 to rotate in a particular degree
* Output:						Servo 2 rotates in a particular degree
* Logic:						Changes the Output Compare Register 1B accordingly to generate Standard servo signal
* Example Call:					servo2_mov(10);
*
*/

void servo2_mov(int degree)
{
	OCR1B=ceil(((degree/1.89)+25));
	servo_angle_2=degree;
}

/*
*
* Function Name:				servo1_free
* Input:						NONE
* Output:						Frees the Servo 1
* Logic:						Top value of Timer Counter of Output Compare Register 1A so that output compare pin A is 5V
* Example Call:					servo1_free();
*
*/

void servo1_free(void)
{
	OCR1A=1150;
}

/*
*
* Function Name:				servo2_free
* Input:						NONE
* Output:						Frees the Servo 2
* Logic:						Top value of Timer Counter of Output Compare Register 1B so that output compare pin B is 5V
* Example Call:					servo2_free();
*
*/

void servo2_free(void)
{
	OCR1B=1150;
}


/*
*
* Function Name:					pick_waste
* Input:							NONE
* Output:							Rotates the servo2 to 5 degrees
* Logic:							Calling the function servo2_mov and delaying it to get hold of the waste block
* Example Call:						pick_waste();
*
*/


void pick_waste(void)
{
	servo2_mov(3);
	_delay_ms(1000);
}

/*
*
* Function Name:				ADC_config
* Input:						NONE
* Output:						Configures the ADC (Analog to Digital Converter) Registers
* Logic:						Using the value of Special Function Register, the function configures the ADC Registers
* Example Call:					ADC_config();
*
*/


void ADC_config(void)
{
	cli();
	TCCR3A=0x00;
	TCCR3B=0x02; // with clock setting 8
	TCCR3C=0x00;
	TIMSK3=0x01; // to enable timer overflow interrupt
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

/*
*
* Function Name:			getdata
* Input:					Ch -> integer that denotes Channel
* Output:					Channel connected to input gives analog value which gets converted to digital value using this function
* Logic:					Loads the Analog Value. Using ADCSRA register it starts converting analog to digital value which gets stored in ADCH register. ADCH register will be the output of the function
* Example Call:				getdata();
*
*/

int getdata(int Ch)
{
	int a;									//a -> digital value which stores the digital value of ADCH register
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

/*
*
* Function Name:			sharp
* Input:					adc_reading -> Gives ADC reading of the 9, 10, 11, 12 and 13 sharp sensors
* Output:					Calculates the distance as per the ADC reading
* Logic:					calibration of the distance is done by the formula = (int)(10.00*(2799.6*(1.00/(pow(adc_reading,1.1546)))))
* Example Call:				sharp(digital[11]);
*	
*/

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

/*
*
* Function Name:			navigate	
* Input:					NONE
* Output:					It updates the navigation pointer (navigation_pointer is a global variable),which is used to have a track on the direction of the bot. It also prints the status of the 
							program execution on lcd
* Logic:					Depends upon the value of int navigation_in_degrees (defined in global scope) navigation_pointer will be assigned direction string
							for navigation_in_degrees= 0 direction string will be "East"
							for navigation_in_degrees= -270 or navigation_in_degrees= 90 direction string will be "North"
							for navigation_in_degrees= -180 or navigation_in_degrees= 180 direction string will be "West"
							for navigation_in_degrees= 270 or navigation_in_degrees= -90 direction string will be "South"
* Example Call:				navigate();
*
*/

void navigate(void) 
{
	cli();
	if(navigation_in_degrees==0)
	navigation_pointer="East ";
	else if(abs(navigation_in_degrees)==270)
	{
		if(navigation_in_degrees>0)
		navigation_pointer="South";
		else
		navigation_pointer="North";
	}
	else if(abs(navigation_in_degrees)==180)
	{
		navigation_pointer="West ";
	}
	else if(abs(navigation_in_degrees)==90)
	{
		if(navigation_in_degrees>0)
		navigation_pointer="North";
		else
		navigation_pointer="South";
	}
	sei();
}

/*
*
* Interrupt Service Routine Name:			ISR(TIMER3_OVF_vect)
* Input:									NONE
* Output:									Updates the digital values of white line and sharp sensors and detects whether the bot is in City Area or Isolated Area
* Logic:									Timer 3 is configured as such that it gets overflow in every 0.3 seconds so that the sensor readings are updated continuously.
* Example Call:								NONE
*
*/

ISR(TIMER3_OVF_vect) // Timer 3overflow interrupt to get digital value
{
	int i=0;
	for(i=1;i<4;i++)
	digital[i]=getdata(i);
	digital[11]=getdata(11);
	if(node_number<10)
	Area='C';
	else
	Area='I';
}

/*
*
* Function Name:			line_following
* Input:					NONE
* Output:					Following of black line by the bot depends upon the readings of white line sensors. It detects the node and than executes "node_behave" function. 
* Logic:					Digital value of right sensor, middle sensor and left sensor will be updated by digital[1], digital[2] and digital[3] respectively, if digital 
							value is less than 15 it means sensors are on while surface else sensors are on black surface. 
							If left sensor is on black surface the bot will rotate in left direction.
							If right sensor is on black surface the bot will rotate in right direction.  
							If middle sensor has digital value more than 50 it means bot is on node and after that bot stops and executes "node_behave" function.
* Example Call:				line_following();
*
*/

void line_following(void)
{
	if (digital[2]>60 || (digital[1]>15 && digital[2]>15))
	{
		stop();
		node_behave();
	}
	else if(digital[2]>15 && (digital[1]>15 || digital[3]>15))
	{	stop();
		node_behave();
	}
	else if(digital[1]>15 && digital[3]<15)
	{
		sppeed(255,249);
		right();
	}
	else if(digital[1]<15 && digital[3]>15)
	{
		sppeed(255,249);
		left();
	}
	else
	{
		sppeed(255,249);
		motion('f');
	}
}

/*
*
* Function Name:			node_behave
* Input:					NONE
* Output:					Bot will rotate according to the path_string if path_string is loaded. If path string is empty the bot will move its arm or check its sensor value
							according to the node, on which currently it is.
* Logic:					If path string is loaded the bot will rotate till it will not achieve the direction instructed by the string and than it increases the path_index
							(index of path string). If path string is empty the bot will update its current node (updates integer node_number with current node value) then it performs
							task assigned at that particular node. These tasks are described as follows:-
												__                                              __
											   |w3|											   |SZ|
							
							 _														 __
							|A|----#(15)-------#(12)								|w1|		#(1)              #(21)
												|												|				  |
												|				___								|				  |									# :-	node
											(11)#------#(14)  |DZ1|________________				|		 (19)#___#(7)___#(20)						SZ :-	Sorting zone 
												|			   |					|			|			|			|							
							 _					|			   |		Bridge		|			|			|	____	|
							|B|----#(16)-------#(10)-----------|--------------------|-----------#(0)------#(6)	|IP	|	#(8)-----#(22)
												|			   |					|			|			|			|
												|			   |________________	|			|			|#___#(9)___|								N
												|								|DZ2|	#(4)----#(2)	 (18)	  |									    |
												|												|				  |								  W<----|---->E
							 _					|		 __						 __				|				  |									    |
							|C|----#(17)-------#(13)	|w4|					|SZ|	#(5)----#(3)			  #(23)									S
							
												 __												 __
												|w5|											|w2|
														
														fig: Arena in the form of graph which has been saved in the bot using "struct_config" function
														 
																								
							->For node 0 and 10, the bot will check whether the bridge is tilted towards the area, on which the bot is, using sharp sensor. 
							If bot is tilted toward counter area, it will traverse to the node where weight blocks are kept else it performs its main 
							tasks which are assigned to it at the starting of program.
							 
							->Node 1,3,12 and 13 are node which belongs to weight blocks, so at these node bot will move its arm to pick the weight block and then bot is instructed to 
							traverse to the node 4 (if the bot is on city area) or node 14 (if the bot is on isolated area).
							
							->Node 4 and 14 are the weight deposition zone for city and isolated area respectively, so at these nodes the bot will move its arm and deposit weight
							block in container (in DZ2 if bot is in city area else in DZ1).
							
							->Node 6 belongs to initial position of waste blocks, so at this node bot will move its arm accordingly and picks up waste after that 
							IP (flag gives the status that how many wastes are remaining at IP) gets decremented by one and the bot is instructed to waste deposition nodes(15,16,17).
							
							->Node 15,16 and 17 are waste deposition nodes of arena associated with waste deposition zone A,B and C respectively, so at these nodes the bot checks 
							whether the flag color is matching with the color of the waste which is in robotic arm or not, if yes then the bot will deposit the waste else it will
						    instructed towards other deposition nodes.
							             
* Example Call:				node_behave(); 
*
*/

void node_behave(void) 
{
	int i;
    if(path[path_index]!='\0')
	{
		if((node_number==6 && *navigation_pointer=='E' && destinationn_node!=21) ||(node_number==16 && *navigation_pointer=='W') ||(node_number==15 && *navigation_pointer=='W') ||(node_number==17 && *navigation_pointer=='W') )
		distance('f',3,'o');
		else if(node_number==14)
		distance('f',9.5,'o');
		else
		distance('f',8.5,'o');
		if((*navigation_pointer-path[path_index])==-9 || (*navigation_pointer-path[path_index])==4 || (*navigation_pointer-path[path_index])==14) //-9,4,14 are ASCII values for 270 degree while rotating in rightwards
			while(*navigation_pointer!=path[path_index])
				{left_degrees(90);}
		else if(node_number==3)
			while(*navigation_pointer!=path[path_index])
				right_degrees(180);
		else if(abs(*navigation_pointer-path[path_index])==5 || abs(*navigation_pointer-path[path_index])==18)
			while(*navigation_pointer!=path[path_index])
				left_degrees(180);
		else
			while(*navigation_pointer!=path[path_index])
			right_degrees(90);
		path_index++;
	}
	else
	{
		node_number=destinationn_node;
		navigate();
		if(node_number==15 || node_number==16 || node_number==17)
		{
			if((deposition_zone_A+deposition_zone_B+deposition_zone_C)!=1 && color_found==0 && check<2)
			{
				left(); //Turn left
				angle_rotate(45);
				distance('f',16,'o');
				right(); //Turn left
				angle_rotate(170);
				stop();
				color_identified='\0';
				color_sensor();
				if(color_identified=='\0')
				{
					distance('f',2,'o');
					color_sensor();
				}
				if(node_number==15)
				color_deposited[0]=color_identified;
				if(node_number==16)
				color_deposited[1]=color_identified;
				if(node_number==17)
				color_deposited[2]=color_identified;
				left(); //Turn left
				angle_rotate(170);
				distance('b',12,'o');
				while(digital[2]<15 && digital[1]<15 && digital[3]<15)
				{
					sppeed(255,249);
					right();
				}
				right();
				angle_rotate(25);
				stop();
			}
			if(color_identified==color[color_index-1] || (deposition_zone_A+deposition_zone_B+deposition_zone_C)==1 || check==2 || color_found>0)
			{
				check=0;
				servo1_mov(10);
				_delay_ms(650);
				servo2_mov(45);
				_delay_ms(100);
				servo1_mov(80);
				servo2_free();
				if(node_number==15)
				deposition_zone_A--;
				if(node_number==16)
				deposition_zone_B--;
				if(node_number==17)
				deposition_zone_C--;
				path_find(10);
				distance('b',6,'o');
			}
			else
			{
				check++;
				distance('b',6,'o');
				if(deposition_zone_B==1 && color_deposited[1]=='\0')
				path_find(16);
				else if(deposition_zone_A==1 && color_deposited[0]=='\0')
				path_find(15);
				else if(deposition_zone_C==1 && color_deposited[2]=='\0')
				path_find(17);
			}
		}
		if(node_number==1)
		{
			distance('f',10,'o');
			left();
			navigation_in_degrees=navigation_in_degrees+90;
			angle_rotate(95);
			stop();
			if(weight_block_position_1==2)
			{
				distance('f',3.5,'o');
				servo1_mov(16);
				_delay_ms(800);
				servo2_mov(0);
				_delay_ms(500);
				servo1_mov(130);
				_delay_ms(1100);
				servo1_free();
			}
			else if(weight_block_position_1==1)
			{
				distance('f',5,'o');
				servo1_mov(9);
				_delay_ms(800);
				servo2_mov(0);
				_delay_ms(500);
				servo1_mov(130);
				_delay_ms(1100);
				servo1_free();
			}
			weight_block_position_1--;
			distance('b',14,'o');
			enq_end(4);
		}

		if(node_number==13)
		{
			if(weight_block_position_5!=0)
			{
				distance('f',12,'o');
				if(weight_block_position_5==2)
				{
					servo1_mov(16);
					_delay_ms(800);
					servo2_mov(0);
					_delay_ms(500);
					servo1_mov(130);
					_delay_ms(1100);
					servo1_free();
				}
				else if(weight_block_position_5==1)
				{
					distance('f',2,'o');
					servo1_mov(9);
					_delay_ms(800);
					servo2_mov(0);
					_delay_ms(500);
					servo1_mov(130);
					_delay_ms(1100);
					servo1_free();
				}
				weight_block_position_5--;
				distance('b',14,'o');
				enq_end(14);
			}
			else
			{
				distance('f',11,'o');
				left();
				navigation_in_degrees=navigation_in_degrees+90;
				angle_rotate(97);
				stop();
				if(weight_block_position_4==2)
				{
					distance('f',4,'o');
					servo1_mov(16);
					_delay_ms(800);
					servo2_mov(0);
					_delay_ms(500);
					servo1_mov(130);
					_delay_ms(1100);
					servo1_free();
				}
				else if(weight_block_position_4==1)
				{
					distance('f',5,'o');
					servo1_mov(9);
					_delay_ms(800);
					servo2_mov(0);
					_delay_ms(500);
					servo1_mov(130);
					_delay_ms(1100);
					servo1_free();
				}
			weight_block_position_4--;
			distance('b',14,'o');
			enq_end(14);
			}
		}

		if(node_number==3)
		{
			distance('f',11.5,'o');
			if(weight_block_position_2==2)
			{
				servo1_mov(14);
				_delay_ms(800);
				servo2_mov(0);
				_delay_ms(500);
				servo1_mov(130);
				_delay_ms(1100);
				servo1_free();
			}
			else if(weight_block_position_2==1)
			{
				distance('f',4,'o');
				servo1_mov(9);
				_delay_ms(800);
				servo2_mov(0);
				_delay_ms(500);
				servo1_mov(130);
				_delay_ms(1100);
				servo1_free();
			}
			weight_block_position_2--;
			distance('b',14.5,'o');
			enq_end(4);              //adding node 4 in priority queue
		}
		if(node_number==12)
		{
			distance('f',12,'o');
			if(weight_block_position_3==2)
			{
				servo1_mov(16);
				_delay_ms(800);
				servo2_mov(0);
				_delay_ms(500);
				servo1_mov(130);
				_delay_ms(1100);
				servo1_free();
			}
			else if(weight_block_position_3==1)
			{
				distance('f',2.5,'o');
				servo1_mov(9);
				_delay_ms(800);
				servo2_mov(0);
				_delay_ms(500);
				servo1_mov(130);
				_delay_ms(1100);
				servo1_free();
			}
			weight_block_position_3--;
			distance('b',14,'o');
			enq_end(14);
		}
		if(node_number==6)
		{
			if(IP!=1)
			{
				distance('b',3,'0');
				if(IP==3)
				{
					distance('f',2,'0');
					servo1_mov(37);
					_delay_ms(800);
					pick_waste();
					servo1_mov(80);
					_delay_ms(800);
					left_degrees(180);
					servo1_mov(0);
					_delay_ms(1000);
					color_identified='\0';
					color_sensor();
					if(color_identified=='\0')
					{
						servo2_mov(45);
						distance('f',2,'o');
						color_sensor();
						distance('b',2,'o');
						pick_waste();
					}
					color[color_index]=color_identified;
					servo1_mov(80);
					if(deposition_zone_B==1)
					path_find(16);
					else if(deposition_zone_A==1)
					path_find(15);
					else if(deposition_zone_C==1)
					path_find(17);
					path_index++;
					color_index++;
				}
			if(IP==2)
				{
					distance('f',2,'o');
					servo1_mov(25);
					_delay_ms(800);
					pick_waste();
					servo1_mov(80);
					_delay_ms(800);
					left_degrees(180);
					servo1_mov(0);
					_delay_ms(1000);
					color_identified='\0';
					color_sensor();
					if(color_identified=='\0')
					{
						servo2_mov(45);
						distance('f',2,'o');
						color_sensor();
						distance('b',2,'o');
						pick_waste();
					}
					color[color_index]=color_identified;
					servo1_mov(80);
					int a=0;             //summing variable
					for(i=0;i<4;i++)
					{
						if(color[color_index]==color_deposited[i])
						{
							color_found++;
							path_find(15+i);
						}
						if(color_deposited[i]!='\0')
						a++;
					}
					if(a>2)
					color_found++;
					if(path[path_index]=='\0')
					{
						if(deposition_zone_B==1 && color_deposited[1]=='\0')
						path_find(16);
						else if(deposition_zone_A==1 && color_deposited[0]=='\0')
						path_find(15);
						else if(deposition_zone_C==1 && color_deposited[2]=='\0')
						path_find(17);
					}
					path_index++;
					color_index++;
				}
					IP--;
			}
			else
			{
				distance('f',2.5,'o');
				servo2_mov(45);
				_delay_ms(200);
				color_found++;
				servo1_mov(5);
				_delay_ms(800);
				pick_waste();
				servo1_mov(80);
				if(deposition_zone_B==1)
				path_find(16);
				else if(deposition_zone_A==1)
				path_find(15);
				else if(deposition_zone_C==1)
				path_find(17);
				color_index++;
				distance('f',3,'o');
			}

		}
		if(node_number==4)
		{
			distance('b',4,'o');
			servo1_mov(66);
			_delay_ms(700);
			servo2_mov(45);
			_delay_ms(100);
			servo1_mov(80);
			_delay_ms(1000);
			if(sharp(digital[11])<300)
			{
				bridge=Area;
				path_find(6);
			}
			else
			{
				bridge='I';
				if(weight_block_position_2==0)
				path_find(1);
				else
				path_find(3);
			}
			left();
			navigation_in_degrees=navigation_in_degrees+180;
			angle_rotate(180);
			path_index++;
			distance('b',5,'o');
		}
		if(node_number==14)
		{
			distance('b',4,'o');
			servo1_mov(66);
			_delay_ms(1000);
			servo2_mov(60);
			_delay_ms(100);
			servo1_mov(90);
			_delay_ms(1000);
			if(sharp(digital[11])<200)
			{
				bridge=Area;
				path_find(0);
			}
			else
			{
				bridge='C';
				if(weight_block_position_3!=0)
				path_find(12);
				else
				path_find(13);
			}
			left();
			navigation_in_degrees=navigation_in_degrees+180;
			angle_rotate(180);
			path_index++;
			distance('b',5,'o');
		}
		if(node_number==0)
		{
			if(*navigation_pointer!='W')
			{
				distance('f',11,'o');
				if((*navigation_pointer-'W'==-9) || (*navigation_pointer-'W')==4 || (*navigation_pointer-'W')==14) //-9,4,14 are ASCII values for 270 degree while rotating in rightwards
				while(*navigation_pointer!='W')
				left_degrees(90);
				else if(abs(*navigation_pointer-'W')==5 || abs(*navigation_pointer-'W')==18)
				while(*navigation_pointer!='W')
				right_degrees(180);
				else
				while(*navigation_pointer!='W')
				right_degrees(90);
				distance('b',11,'o');
			}
			if(sharp(digital[11])<400)
			{
				bridge=Area;
				path_find(6);
			}
			else
			{
				bridge='I';
				if(weight_block_position_2==0)
				enq_end(1);
				else
				enq_end(3);
			}
			navigate();
		}
		if(node_number==10)
		{
			if(*navigation_pointer!='E')
			{
				distance('f',11,'o');
				if((*navigation_pointer-'E'==-9) || (*navigation_pointer-'E')==4 || (*navigation_pointer-'E')==14) //-9,4,14 are ASCII values for 270 degree while rotating in rightwards
				while(*navigation_pointer!='E')
				left_degrees(90);
				else if(abs(*navigation_pointer-'E')==5 || abs(*navigation_pointer-'E')==18)
				while(*navigation_pointer!='E')
				right_degrees(180);
				else
				while(*navigation_pointer!='E')
				right_degrees(90);
				distance('b',12,'o');
			}
			if(sharp(digital[11])<450)
			{
				bridge=Area;
				path_find(0);
			}				
			else
			{
				bridge='C';
				if(weight_block_position_3!=0)
				enq_end(12);
				else
				enq_end(13);
			}
		}

		if(que_length!=0 && path[path_index]=='\0')
		{
			path_find(end->num);
			dequeue();
		}
		else if(path[path_index]=='\0')
		{
			buzzer(1);
			_delay_ms(2000);
			buzzer(0);
			exit(1);
		}
	}
}

/*
*
* Function Name:					color_sensor_pin_config
* Input:							NONE
* Output:							Enables color sensor pins
* Logic:							0 sets as input pin for D Register
* Example Call:						color_sensor_pin_config();
*
*/

void color_sensor_pin_config(void)
{
	DDRD  = DDRD | 0xFE; //set PD0 as input for color sensor output
	PORTD = PORTD | 0x01;//Enable internal pull-up for PORTD 0 pin
}


/*
*
* Function Name:			clrport_init
* Input:					NONE
* Output:					Initializes the color sensor pin
* Logic:					Calls the function color_sensor_pin_config()
* Example Call:				clrport_init();
*
*/

void clrport_init(void)
{
	color_sensor_pin_config();//color sensor pin configuration
}


/*
*
* Function Name:				color_sensor_pin_interrupt_init
* Input:						NONE
* Output:						NONE
* Logic:						Using the value of Special Function Register, the function configures Interrupt 0 for the color sensor
* Example Call:					color_sensor_pin_interrupt_init();
*
*/

void color_sensor_pin_interrupt_init(void) //Interrupt 0 enable
{
	cli(); //Clears the global interrupt
	EICRA = EICRA | 0x02; // INT0 is set to trigger with falling edge
	EIMSK = EIMSK | 0x01; // Enable Interrupt INT0 for color sensor
	sei(); // Enables the global interrupt
}

/*
*
* Interrupt Service Routine Name:					ISR(INT0_vect)
* Input:											Interrupt pulse at INT0 pin
* Output:											Counts the number of output pulses receiving from color sensor
* Logic:											Increments variable 'pulse' on receiving pulse from the color sensor
* Example Call:										NONE
*
*/

//ISR for color sensor
ISR(INT0_vect)
{
	pulse++; //increment on receiving pulse from the color sensor
}

/*
*
* Function Name:				init_devices
* Input:						NONE
* Output:						Initializes color sensor pins and color sensor interrupt pin
* Logic:						Calls the function color_sensor_pin_interrupt_init(); and clrport_init();
* Example Call:					init_devices();
*
*/

void init_devices(void)
{
	cli(); //Clears the global interrupt
	clrport_init();  //Initializes all the ports
	color_sensor_pin_interrupt_init();
	sei();   // Enables the global interrupt
}

/*
*
* Function Name:			filter_red
* Input:					NONE
* Output:					Selects the red filter
* Logic:					By setting the pins S2 and S3 as low
* Example Call:				filter_red();
*
*/

//Filter Selection
void filter_red(void)    //Used to select red filter
{
	//Filter Select - red filter
	PORTD = PORTD & 0xBF; //set S2 low
	PORTD = PORTD & 0x7F; //set S3 low
}

/*
*
* Function Name:			filter_green
* Input:					NONE
* Output:					Selects the green filter
* Logic:					By setting the pins S2 and S3 as high
* Example Call:				filter_green();
*
*/

void filter_green(void)	//Used to select green filter
{
	//Filter Select - green filter
	PORTD = PORTD | 0x40; //set S2 High
	PORTD = PORTD | 0x80; //set S3 High
}

/*
*
* Function Name:			filter_blue
* Input:					NONE
* Output:					Selects the blue filter
* Logic:					By setting the pins S2 as low and S3 as high
* Example Call:				filter_red();
*
*/

void filter_blue(void)	//Used to select blue filter
{
	//Filter Select - blue filter
	PORTD = PORTD & 0xBF; //set S2 low
	PORTD = PORTD | 0x80; //set S3 High
}



/*
*
* Function Name:				color_sensor_scaling
* Input:						NONE
* Output:						Selects the scaled down version of the original frequency of the output generated by the color sensor
* Logic:						Sets S0 as high, S1 as low and S1 as high
* Example Call:					color_sensor_scaling();
*
*/

//Color Sensing Scaling
void color_sensor_scaling()		//This function is used to select the scaled down version of the original frequency of the output generated by the color sensor, generally 20% scaling is preferable, though you can change the values as per your application by referring datasheet
{
	//Output Scaling 20% from data sheet
	//PORTD = PORTD & 0xEF;
	PORTD = PORTD | 0x10; //set S0 high
	//PORTD = PORTD & 0xDF; //set S1 low
	PORTD = PORTD | 0x20; //set S1 high
}

/*
*
* Function Name:				red_read
* Input:						NONE
* Output:						Selects red filter and display the count generated by the sensor on LCD
* Logic:						The function selects the red filter, captures the pulses for 0.1 second and stores the count in the variable red.
								The count will be more if the color is red. The count will be very less if its blue or green.
* Example Call:					red_read();
*
*/

void red_read(void) // function to select red filter and display the count generated by the sensor on LCD. The count will be more if the color is red. The count will be very less if its blue or green.
{
	//Red
	filter_red(); //select red filter
	pulse=0; //reset the count to 0
	_delay_ms(100); //capture the pulses for 100 ms or 0.1 second
	red = pulse;  //store the count in variable called red
}

/*
*
* Function Name:				green_read
* Input:						NONE
* Output:						Selects green filter and display the count generated by the sensor on LCD
* Logic:						The function selects the green filter, captures the pulses for 0.1 second and stores the count in the variable green.
								The count will be more if the color is green. The count will be very less if its blue or red.
* Example Call:					green_read();
*
*/

void green_read(void) // function to select green filter and display the count generated by the sensor on LCD. The count will be more if the color is green. The count will be very less if its blue or red.
{
	//Green
	filter_green(); //select green filter
	pulse=0; //reset the count to 0
	_delay_ms(100); //capture the pulses for 100 ms or 0.1 second
	green = pulse;  //store the count in variable called green
}

/*
*
* Function Name:				blue_read
* Input:						NONE
* Output:						Selects blue filter and display the count generated by the sensor on LCD
* Logic:						The function selects the blue filter, captures the pulses for 0.1 second and stores the count in the variable blue.
								The count will be more if the color is blue. The count will be very less if its red or green.
* Example Call:					blue_read();
*
*/

void blue_read(void) // function to select blue filter and display the count generated by the sensor on LCD. The count will be more if the color is blue. The count will be very less if its red or green.
{
	//Blue
	filter_blue(); //select blue filter
	pulse=0; //reset the count to 0
	_delay_ms(100); //capture the pulses for 100 ms or 0.1 second
	blue = pulse;  //store the count in variable called blue

}

/*
*
* Function Name:				color_sensor		
* Input:						NONE
* Output:						Identifies the color RED, GREEN or BLUE
* Logic:						Selects red, blue and green filters. If the all the filters give the value less than threshold value of color sensor then it is black color and it is denoted by null chracter.
								Otherwise, the filter which has the highest value of pulses will be identified as that color.
								For example if the color detected is blue, then the pulses for blue filter will be more than that of red and green.
* Example Call:					color_sensor();
*
*/

void color_sensor(void)
{
	red_read();
	blue_read();
	green_read();
	if(red<threshold_color_value && blue<threshold_color_value && green<threshold_color_value)
	color_identified='\0';
	else
	{
		if(red>blue && red>green)
		color_identified='R';
		else if(blue>red && blue>green)
		color_identified='B';
		else if(green>red && green>blue)
		color_identified='G';
	}
	lcd_cursor(2,10);
	lcd_wr_char(color_identified);
}

/*
*
* Function Name:					main
* Input:							NONE
* Output:							int to inform the caller that the program exited correctly or incorrectly
* Logic:							Area in the form of graph is made.
									LCD port, Speed, Servo, ADC, Interrupts are configured.
									All the ports and devices are initialized.
									Color sensor is scaled down and navigation process of the bot is initiated.
									Servo 1 is moved to 80 degree and Servo 2 to 45 degree and servo 2 is freed.
									While navigation, the bot continuously follows the black line and performs its required task.			
* Example Call:						Called automatically by the Operating System
*
*/

int main(void)
{
	struct_config();
	lcd_port_config();
	lcd_init();
	INT_position();
	sppeed_config();
	servo_config();
	ADC_config();
	init_devices();
	color_sensor_scaling();
	navigate();			//navigate the bot at starting
	servo1_mov(80);		//move arm to its initial position
	servo2_mov(45);
	_delay_ms(500);
	servo2_free();     // main 6 tasks of the bot
	while(1)				// Bot is instructed to follow line continuously 
	{
		line_following();
		if(deposition_zone_A+deposition_zone_B+deposition_zone_C==0)		// When all the three wastes are deposited at the deposition zone the bot																	  
		{																	// will beep the buzzer for 5 sec and then exits from the program.
			buzzer(1);
			_delay_ms(5000);
			buzzer(0);
			_delay_ms(500);
			break;
		}
	}	
}

