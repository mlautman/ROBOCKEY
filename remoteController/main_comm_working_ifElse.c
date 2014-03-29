/*
 * CommTest.c
 *
 * Created: 14/11/2012 08:40:57 PM
 *  Author: Praveer Nidamaluri
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
#define halfTime    0xA6
#define gameOver    0xA7

int PIN = 6;
int PIN_LMotor_En = 6; //left motor enable pin
int PIN_RMotor_En = 6; //right motor enable pin
int PIN_LMotor_Dir = 3; //left motor direction pin
int PIN_RMotor_Dir = 4; //right motor direction pin
int PIN_Comm = 0;
int PIN_Play = 1;
int PIN_Stop = 2;
char buffer[PL] = {0};
char port_num = 0x20;
bool update = false;

volatile int state = 1;
volatile unsigned char state_requested = 0x00;

int main(void) {
  
  //initialize mBUS
  	m_bus_init();
  	m_usb_init();
  	m_port_init(port_num);
   
	set(DDRB,1);
	set(DDRB,2);
	set(DDRB,3);

	
  //enable 3 pins on the expansion port for the LEDs
  	m_port_set(port_num,DDRG,PIN_Comm);
  	m_port_set(port_num,DDRG,PIN_Play);
  	m_port_set(port_num,DDRG,PIN_Stop);

  	m_clockdivide(0);

  	m_rf_open(channel,address,PL);
  	sei();
  	while (1) {
    	m_rf_read(buffer,PL);
		if (update == true)	{
			update = false;
			m_usb_tx_int(state_requested);
			m_usb_tx_string("\n\r");
		
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
			}
		}		
  	}
}

ISR(INT2_vect) { 
  	state_requested = buffer[0];
  	m_green(TOGGLE);
  	update  = true;

}


