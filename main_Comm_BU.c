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



void setTimer(void);

int PIN = 6;
int PIN_LMotor_En = 6; //left motor enable pin
int PIN_RMotor_En = 6; //right motor enable pin
int PIN_LMotor_Dir = 3; //left motor direction pin
int PIN_RMotor_Dir = 4; //right motor direction pin
int PIN_Comm = 0;
int PIN_Play = 1;
int PIN_Stop = 2;
//int channel = 1;
char last = 'a';
 char buffer[PL] = {0};
int PWM_Left = 80; //percent on for left
int PWM_Right = 80; //percent on for right
int A_Off = 1000; //set OCR1A and OCR1B to these values to turn the motors off
int B_Off = 1;
char port_num = 0x20;
//char address = 0x33; //51 in hex (17*3)
//int address = 0x33;
//char address = 0x33;

volatile int state = 1;
volatile unsigned char state_requested = 0x00;

ISR(INT2_vect) { 
  
  state_requested = buffer[0];
  m_green(TOGGLE);

}

int main(void) {
  
  //initialize mBUS
  m_bus_init();
  m_usb_init();
  m_port_init(port_num);


  //enable 4 pins for the motor control
  set(DDRB,PIN_LMotor_En);
  set(DDRC,PIN_RMotor_En);
  set(DDRD,PIN_LMotor_Dir);
  set(DDRD,PIN_RMotor_Dir);
    
  //enable 3 pins on the expansion port for the LEDs
  m_port_set(port_num,DDRG,PIN_Comm);
  m_port_set(port_num,DDRG,PIN_Play);
  m_port_set(port_num,DDRG,PIN_Stop);

  //set pin output, clock
  //set(DDRB,PIN);
  m_clockdivide(3);
  
  //TIMER 1 - Left Motor
  //set clock source
  clear(TCCR1B,CS12);
  clear(TCCR1B,CS11);
  set(TCCR1B,CS10);
  
  //what numbers to count up to, PWM
  OCR1A = 900;
  OCR1B = OCR1A/100*PWM_Left;
  
  //set the mode, mode 15
  set(TCCR1B,WGM13);
  set(TCCR1B,WGM12);
  set(TCCR1A,WGM11);
  set(TCCR1A,WGM10);
  
  //set timer interrupt option (demasking)
  //set(TIMSK1,OCIE1A); //write handler for for TIMER1_COMPB
  //set(TIMSK1,OCIE1B);
  
  //channel output options
  set(TCCR1A,COM1B1);
  clear(TCCR1A,COM1B0);

  //TIMER 3 - Right Motor
  //set clock source
  clear(TCCR3B,CS32);
  clear(TCCR3B,CS31);
  set(TCCR3B,CS30);

  //what numbers count up to, PWM
  ICR3 = 900;
  OCR3A = ICR3/100*PWM_Right;

  //set the mode, mode 14
  set(TCCR3B,WGM33);
  set(TCCR3B,WGM32);
  set(TCCR3A,WGM31);
  clear(TCCR3A,WGM30);

  //channel A output
  set(TCCR3A,COM3A1);
  clear(TCCR3A,COM3A0);
  
  m_rf_open(channel,address,PL);
  
  sei();
  while (1) {
    m_rf_read(buffer,PL);
    switch(state_requested) {
      case commTest: // COMM
		//turn off other LEDs
		m_port_clear(port_num,PORTG,PIN_Play);
		m_port_clear(port_num,PORTG,PIN_Stop);
		
		//turn on Comm LED (Yellow)
		m_port_set(port_num,PORTG,PIN_Comm);
		
		OCR1A = A_Off;
		OCR1B = B_Off;
		ICR3 = A_Off;
		OCR3A = B_Off;
		break;
		
      case play: // Play 

		//turn off other LEDs
		m_port_clear(port_num,PORTG,PIN_Comm);
		m_port_clear(port_num,PORTG,PIN_Stop);
	      
		//turn on Play LED (Green)
		m_port_set(port_num,PORTG,PIN_Play); 
	      
		//clear(PORTD,PIN_LMotor_En);
		//clear(PORTD,PIN_RMotor_En);
		set(PORTD,PIN_LMotor_Dir);
		set(PORTD,PIN_RMotor_Dir);
	      
		//set enable pin
		//set OCR1A to proper PWM
	      
		break;
      
      case pause: //Stop
		//turn off other LEDs
		m_port_clear(port_num,PORTG,PIN_Comm);
		m_port_clear(port_num,PORTG,PIN_Play);
	      
		//turn on Stop LED (Red)
		m_port_set(port_num,PORTG,PIN_Stop); 

		//set OCR1A to 0
		OCR1A = A_Off;
		OCR1B = B_Off;
		ICR3 = A_Off;
		OCR3A = B_Off;
		break;
      }	
//	if(m_usb_rx_available()) {
		m_usb_tx_int(state_requested);
		m_usb_tx_string("\n\r");
  }
  
  
  //return 0;
}
