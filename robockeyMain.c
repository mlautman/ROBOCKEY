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




bool debug_com = true;

char buffer[ p_len ] = {0};
volatile bool comm_cmd = false;

volatile unsigned char state_requested = 0x00;   //    <---- TEST IF WE CAN REMOVE UNSIGNED 

void get_command( void );
void localize(char*);
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


void track(unsigned int *mWii_packet){
	// use the mWii data to find the position and orientation of the bot relative to rink center
	
	// SUBROUTINE VARIABLES
	int X[STARS] = {0};
	int Y[STARS] = {0};
	unsigned int Dists[6]= {-1};  		//  dists[0] = s0s1 dists[1] = s0s2  dists[2] = s0s3 dists[3] = s1s2 dists[4] = s1s3 dists[5] = s2s3 
	int s_map[STARS] = {-1};

	int x1; int x2; int x3; int x4; int y1; int y2; int y3; int y4;
	long v21; long v31; long v41; long v32; long v42; long v43;
	double v_vert;
	int x_vert1; int x_vert2; int y_vert1; int y_vert2;
	double scale;
	double xO; double yO;
	int x_other1; int x_other2;
	int x_actually2; int x_actually3; int y_actually2; int y_actually3;
	double alpha; double phi; double r;
	
	int index;
	for (index = 0 ; index< STARS;index++){
		X[i]=mWii_packet[ 3*i ];
		Y[i]=mWii_packet[ 3*i + 1];
	}	
	
	int other;
	int distIndex=0;

	for (index = 0; index<STARS; index++){
		for(other = index+1; other <STARS; other ++){
			Dists[distIndex] = pow((X[index] - X[other]),2) + pow(( Y[index] - Y[other] ),2);
			distIndex++;
		}
	}
	
	int max = -9000;
	unsigned int min = 90000; 
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
					s_map[2] = 0;
					s_map[4] = maxIndex + 1;
					break; 
				case 3: 
				case 4: 
				case 5: 
					s_map[2] = maxIndex + 1 ; 
					s_map[4] = 0;
					break:
				default: 
					m_red(ON);
			}
		case 3:
		case 4:
			switch (minIndex) {
				case 0:
				case 3:
				case 4:
					s_map[2] = 1;
					s_map[4] = maxIndex+1;
					break;
 				case 1:
				case 2:
				case 5:
					s_map[2] = maxIndex+1;
					s_map[4] = 1;
					break;
				default: 
					m_red(ON)
			}
		case 5:
			switch (minIndex){
				case 2:
				case 4:
					s_map[2]=3;
					s_map[4]=2;
					break;
				case 1:
				case 3:
					s_map[2] = 2;
					s_map[4] = 3;
					break;
				default:
					m_red(ON); 
			}
		default:
			m_red(ON);	
	}
	
		
	float s_Vert = sqrt(max);
	float scale    = starVert/29.0;


	x_cent = ( X[ s_map[2] ] + X[s_map[4]]) / 2.0;		// define the local position of the origin
	y_cent = (y_vert1 + y_vert2) / 2.0;		// ^
	

	// determine the mWii_packets 1, 2, and 3
	if( x_vert1 == x_other1 ){ x_actually2 = x_vert2; x_actually3 = x_vert1; y_actually2 = y_vert2; y_actually3 = y_vert1; }
	else if( x_vert2 == x_other1 ){ x_actually2 = x_vert1; x_actually3 = x_vert2; y_actually2 = y_vert1; y_actually3 = y_vert2; }
	else if( x_vert1 == x_other2 ){ x_actually2 = x_vert2; x_actually3 = x_vert1; y_actually2 = y_vert2; y_actually3 = y_vert1; }
	else if( x_vert2 == x_other2 ){ x_actually2 = x_vert1; x_actually3 = x_vert2; y_actually2 = y_vert1; y_actually3 = y_vert2; }
	else{ x_actually2 = 0; x_actually3 = 0; y_actually2 = 0; y_actually3 = 0; }

	// find the angle between the local frame and the global frame
	theta = atan2( (x_actually3 - x_actually2), (y_actually3 - y_actually2) );

	// find the magnitude of the vector to the origin
	r = xO*xO + yO*yO;
	r = sqrt(r);
	
	// find the angle between the local x-axis and the vector to the origin
	alpha = -atan2(yO,xO);
	
	// find the angle between the global x-axis and the vector to the origin
	phi = theta - alpha;
	
	XBOT = -r*cos(phi);		// determine the bot's global position
	YBOT = -r*sin(phi);		// ^
}	
