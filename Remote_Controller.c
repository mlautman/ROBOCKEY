#define 	pack_length 	4
#define 	chan		1
#define	 	RX_address 	0x1D
#define 	TX_address 	0x17


#include <avr/io.h>
#include "m_imu.h"
#include "m_bus.h"
#include "m_usb.h"
#include "m_general.h"
#include "m_wireless.h"


void setUSB(void);
void deal_with_new(void );
void setupADC(void);
void update_ADC(void);
void TX_ADC(void);

bool isTX = true;
bool isRX = false;

char 	pot[4];
char 	new		 	=	 0;
char 	buffer[pack_length]	=	{0};

int main(void)
{
	m_clockdivide(0);
	m_wireless_init( RX_address);
	m_change_buddy(TX_address);
	if (isTX){
		setupADC();
	}

	while (1){
		if ( new == 1 ){
			deal_with_new();
		}
		if (isTX){
			update_ADC();
			TX_ADC();
		}
		else { 
		}
	}	
}

ISR(INT2_vect){
	m_rf_read(buffer , pack_length);
	new = 1;	
}


void setUSB(){
	m_usb_init();
	m_green(ON);
	while (!m_usb_isconnected());
	m_usb_tx_string("ready\n");
	m_green(OFF);
}
void setupADC(){
	set(ADMUX,REFS1);  	// set Vref = internal 2.56V
	set(ADMUX,REFS0); 

	set(ADCSRA,ADPS2);	// set ADC prescaller to 128 -> a 125Khz
	set(ADCSRA,ADPS1);
	set(ADCSRA,ADPS0);
	
	set(DIDR0,ADC0D);
	set(DIDR0,ADC1D);
	set(DIDR0,ADC2D);
	set(DIDR0,ADC3D);

}


void deal_with_new(void ){
	new = 0;
	int i;
	for (i=0; i < pack_length ; i++){
		m_usb_tx_char(buffer[i]);
	}
}

void update_ADC(){
	clear(ADCSRB, MUX5);		// set pin to F0
	clear(ADMUX, MUX2);
	clear(ADMUX, MUX1);
	clear(ADMUX, MUX0);
	
	set(ADCSRA, ADEN);		// start conversion process
	set(ADCSRA, ADSC);	
	
	while(!check(ADCSRA,ADIF));		// wait to finish
	pot[0]=ADC;

	clear(ADCSRB, MUX5);		// set pin to F1
	clear(ADMUX, MUX2);
	clear(ADMUX, MUX1);
	set(ADMUX, MUX0);
	
	set(ADCSRA, ADEN);		// start conversion process
	set(ADCSRA, ADSC);	
	
	while(!check(ADCSRA,ADIF));		// wait to finish
	pot[1]=ADC;

	clear(ADCSRB, MUX5);		// set pin to F1
	clear(ADMUX, MUX2);
	set(ADMUX, MUX1);
	clear(ADMUX, MUX0);
	
	set(ADCSRA, ADEN);		// start conversion process
	set(ADCSRA, ADSC);	
	
	while(!check(ADCSRA,ADIF));		// wait to finish
	pot[2]=ADC;

	clear(ADCSRB, MUX5);		// set pin to F1
	clear(ADMUX, MUX2);
	set(ADMUX, MUX1);
	set(ADMUX, MUX0);
	
	set(ADCSRA, ADEN);		// start conversion process
	set(ADCSRA, ADSC);	
	
	while(!check(ADCSRA,ADIF));		// wait to finish
	pot[3]=ADC;

}
void TX_ADC(){
	m_rf_send(TX_address, pot, 4);
}
