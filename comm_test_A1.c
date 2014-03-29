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

#define PL 12
#define channel 1
#define address 0x46

#define commTest	0xA0
#define play		0xA1
#define goalA		0xA2
#define goalB		0xA3
#define pause		0xA4
#define halfTime   	0xA6
#define gameOver   	0xA7

bool debug_com = true;

char buffer[PL] = {0};
volatile bool comm_cmd = false;

volatile unsigned char state_requested = 0x00;   //    <---- TEST IF WE CAN REMOVE UNSIGNED 

void get_command( void );

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

  	m_rf_open(channel,address,PL);
	sei();
  	while (1) {
		if (comm_cmd  == true)	{
			get_command();
		}		
  	}
}

ISR(INT2_vect) { 
  	m_rf_read(buffer,PL);
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
