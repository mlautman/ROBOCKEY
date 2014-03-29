/*
 * main..c
 *
 * Created: 14/11/2012 08:40:57 PM
 *  Authors: Michael Lautman, Oliver Pacchiana Praveer Nidamaluri
 */ 


/* Name: main.c
 * Author: Oliver Pacchiana
 */
// function runs at .5Hz at 20% duty cycle

#include "m_general.h"
#include "m_bus.h"
#include "m_rf.h"
#include "m_usb.h"
#include "m_port.h"
#include <stdlib.h>
#include <math.h>


#define 	p_len		12
#define 	chan  	 	1
#define 	address 	0x46

#define 	commTest	0xA0
#define 	play		0xA1
#define 	goalA		0xA2
#define 	goalB		0xA3
#define 	pause		0xA4
#define 	halfTime   	0xA6
#define 	gameOver   	0xA7

#define 	STARS		4

#define 	X_BOT		512
#define 	Y_BOT		384



bool debug_com = true;

char buffer[ p_len ] = {0};
volatile bool comm_cmd = false;

volatile unsigned char state_requested = 0x00;   //    <---- TEST IF WE CAN REMOVE UNSIGNED 

void get_command( void );
void localize(unsigned int*, float*);
int get_bad_vals(char*);


int main(void) {
  
  //initialize mBUS
  	m_bus_init();
  	m_usb_init();
 /* 	m_port_init(port_num);
  	m_port_set(port_num,DDRG,PIN_Comm);
	m_port_set(port_num,DDRG,PIN_Play);
	m_port_set(port_num,DDRG,PIN_Stop);

 */ 
	set(DDRB,1);
	set(DDRB,2);
	set(DDRB,3);
	
  	m_clockdivide(0);

  	m_rf_open(chan,address,p_len);
	sei();
  	while (1) {
		if (comm_cmd  == true)	{
			get_command();
		}		
  	}
}

ISR(INT2_vect) { 
  	m_rf_read(buffer,p_len);
  	comm_cmd  = true;
}


void get_command(){

	comm_cmd = false;
	state_requested = buffer[0];
	if (debug_com){
		m_usb_tx_string("state requested:\t");
		m_usb_tx_int(state_requested);
		m_usb_tx_string("\n\r");
	}
	m_red(OFF);
	clear(PORTB,1);
	clear(PORTB,2);
	clear(PORTB,3);

	if(state_requested == commTest){ 
			set(PORTB,1);	
	}
	else if(state_requested == play){ 
			set(PORTB,2);	
	}
	else if(state_requested == goalA){ 
			set(PORTB,1);	
			set(PORTB,2);	
	}
	else if(state_requested == goalB){ 
			set(PORTB,3);	
	}
	else if(state_requested == pause){ 
			set(PORTB,3);	
			set(PORTB,1);	
	}
	else if(state_requested == halfTime){ 
			set(PORTB,3);	
			set(PORTB,2);	
	}
	else if(state_requested == gameOver){ 
			set(PORTB,3);	
			set(PORTB,2);	
			set(PORTB,1);	
	} else {
		m_red(ON);
	}
}


void localize(unsigned int* mWii_packet, float* localizationData){
	// use the mWii data to find the position and orientation of the bot relative to rink center
	
	// SUBROUTINE VARIABLES
	int X[STARS] = {0};
	int Y[STARS] = {0};
	unsigned int Dists[6]= {-1};  		//  dists[0] = s0s1 dists[1] = s0s2  dists[2] = s0s3 dists[3] = s1s2 dists[4] = s1s3 dists[5] = s2s3 
	int s_map[STARS] = {0};

	int cnt;
	for (cnt = 0 ; cnt< STARS;cnt++){
		if (mWii_packet[cnt] =1023)
			m_red(ON);
			break;
		} else{
			X[cnt]=mWii_packet[ 3*cnt ]    - X_BOT;
			Y[cnt]=mWii_packet[ 3*cnt + 1] - Y_BOT;
		}
	}	
	
	int other;
	int distIndex=0;
	
	int index;
	for (index = 0; index<STARS; index++){
		for(other = index+1; other <STARS; other ++){
			Dists[distIndex] = pow((X[index] - X[other]),2) + pow(( Y[index] - Y[other] ),2);
			distIndex++;
		}
	}


	int minIndex=-1;
	int maxIndex=-1;
	long  max = -9000;
	long  min = 90000; 
	for (index = 0 ; index < 6 ; index++){
		if (Dists[index] < min){
			min = Dists[index];
			minIndex = index;
		}
		if ( Dists[index] > max ){
			max  = Dists[index];
			maxIndex  = index;
		}
	}

	m_red(OFF);
		
	switch (maxIndex){
		case 0:
		case 1:
		case 2:
			switch (minIndex) {
				case 0:
				case 1:
				case 2:		
					s_map[1] = 0;
					s_map[3] = maxIndex + 1;
					break; 
				case 3: 
				case 4: 
				case 5: 
					s_map[1] = maxIndex + 1 ; 
					s_map[3] = 0 ;
					break;
				default: 
					m_red(ON);
			}
		case 3:
		case 4:
			switch (minIndex) {
				case 0:
				case 3:
				case 4:
					s_map[1] = 1;
					s_map[3] = maxIndex+1;
					break;
 				case 1:
				case 2:
				case 5:
					s_map[1] = maxIndex+1;
					s_map[3] = 1;
					break;
				default: 
					m_red(ON)
			}
		case 5:
			switch (minIndex){
				case 2:
				case 4:
					s_map[1]=3;
					s_map[3]=2;
					break;
				case 1:
				case 3:
					s_map[1] = 2;
					s_map[3] = 3;
					break;
				default:
					m_red(ON); 
			}
		default:
			m_red(ON);	
	}
	
		
	float s_Vert = sqrt(max);  		// find the pixel dist b/w stars 2 & 4
	float scale  = s_Vert/29.0;		// find the ration of pixel/cm

	// find the location of global center in local coordinates.
	int x_cent, y_cent;
	x_cent = ( X[ s_map[1] ] + X[s_map[3]] ) / 2.0;		// define the local position of the origin
	y_cent = ( Y[ s_map[1] ] + Y[s_map[3]] ) / 2.0;		// define the local position of the origin
	
	// find the dist from bot to global center
	float r =  sqrt( x_cent*x_cent + y_cent*y_cent);
		
	// find the angle between the local frame and the global frame
	float theta = atan2(  X[ s_map[1] ] - X[s_map[3]]  , Y[ s_map[1] ] - Y[s_map[3]] );

	// find the angle between the local x-axis and the vector to the global origin
	float alpha = atan2( x_cent, y_cent);
	
	// find the angle between the global x-axis and the vector to the origin
	float phi = theta + alpha;
	
	float XBOT = -r*cos(phi);		// determine the bot's global position
	float YBOT = -r*sin(phi);		// ^

	localizationData[0] = XBOT;
	localizationData[1] = YBOT;
	localizationData[2] = phi;
}	
