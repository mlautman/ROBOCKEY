#define 	p_length 	8
#define 	chan		1
#define	 	RX_add 		0x1D
#define 	TX_add 		0x17
#define 	ADC_DELAY 	2000

#include <avr/io.h>
#include "m_imu.h"
#include "m_bus.h"
#include "m_usb.h"
#include "m_general.h"
#include "m_rf.h"
#include <stdlib.h>

#include "m_wireless.h"

void setupUSB(void);
void deal_with_new(void );
void setupADC(void);
void update_ADC(void);
void TX_ADC(void);

bool isTX = !true;

char 	pot[p_length]	;
char 	new		 	=	 0;
char 	buffer[p_length]	=	{0};

int main(void)
{
	m_bus_init();
	m_clockdivide(0);
	if (isTX){
		m_rf_open(chan,TX_add,p_length);
		setupADC();
	}else {
		m_rf_open(chan,RX_add,p_length);
		setupUSB();
		sei();
	}

	while (1){
		if ( new == 1 ){
			deal_with_new();
		}
		if (isTX){
			update_ADC();
			m_green(ON);
			m_wait(10);
			TX_ADC();
			m_green(OFF);
			m_wait(10);
		}
		else {
				
		}
	}	
}

ISR(INT2_vect){
	m_rf_read(buffer , p_length);
	new = 1;	
}

void TX_ADC(){
	m_green(TOGGLE);
	m_rf_send(RX_add, pot, p_length);
}

void setupUSB(){
	m_usb_init();
	m_green(ON);
	while (!m_usb_isconnected());
	m_usb_tx_string("ready\n");
	m_green(OFF);
}
void setupADC(){
	clear(ADMUX,REFS1);  	// set Vref = internal 2.56V
	set(ADMUX,REFS0); 

	set(ADCSRA,ADPS2);	// set ADC prescaller to 128 -> a 125Khz
	set(ADCSRA,ADPS1);
	set(ADCSRA,ADPS0);
	
	set(DIDR0,ADC0D);
	set(DIDR0,ADC1D);
	set(DIDR0,ADC4D);
	set(DIDR0,ADC5D);

}


void deal_with_new(void ){
	m_red(TOGGLE);
	new = 0;
	int i;
	int a;
	for (i=0; i < p_length ; i=i+2){
		m_usb_tx_string("\tpot ");
		m_usb_tx_int(i/2 );
		m_usb_tx_string(": \t");
		a = *(int*)&buffer[i];
		m_usb_tx_int(a);
	}
	m_usb_tx_string("\n\r");
}

void update_ADC(){
	clear(ADCSRB, MUX5);		// set pin to F0
	clear(ADMUX, MUX2);
	clear(ADMUX, MUX1);
	clear(ADMUX, MUX0);
	
	set(ADCSRA, ADEN);		// start conversion process
	set(ADCSRA, ADSC);	
	
	smallDelay(ADC_DELAY);	
	while(!check(ADCSRA,ADIF));		// wait to finish
	set(ADCSRA,ADIF);
	pot[0]=ADCL;
	pot[1]=ADCH;


	clear(ADCSRB, MUX5);		// set pin to F1
	clear(ADMUX, MUX2);
	clear(ADMUX, MUX1);
	set(ADMUX, MUX0);
	
	set(ADCSRA, ADEN);		// start conversion process
	set(ADCSRA, ADSC);	
	
	smallDelay(ADC_DELAY);	
	while(!check(ADCSRA,ADIF));		// wait to finish
	set(ADCSRA,ADIF);
	pot[2]=ADCL;
	pot[3]=ADCH;



	clear(ADCSRB, MUX5);		// set pin to F1
	set(ADMUX, MUX2);
	clear(ADMUX, MUX1);
	clear(ADMUX, MUX0);
	
	set(ADCSRA, ADEN);		// start conversion process
	set(ADCSRA, ADSC);	

	smallDelay(ADC_DELAY);
	while(!check(ADCSRA,ADIF));		// wait to finish
	set(ADCSRA,ADIF);
	pot[4]=ADCL;
	pot[5]=ADCH;


	clear(ADCSRB, MUX5);		// set pin to F1
	set(ADMUX, MUX2);
	clear(ADMUX, MUX1);
	set(ADMUX, MUX0);
	
	set(ADCSRA, ADEN);		// start conversion process
	set(ADCSRA, ADSC);	
	
	smallDelay(ADC_DELAY);	
	while(!check(ADCSRA,ADIF));		// wait to finish
	set(ADCSRA,ADIF);
	pot[6]=ADCL;
	pot[7]=ADCH;


}

