/*
 * Wireless.c
 *
 * Created: 2012/10/14 9:36:47
 *  Author: Genius
 */ 


#include "m_general.h"
#include "m_bus.h"
#include "m_rf.h"
#include "m_usb.h"
#include "wireless.h"

volatile int ConvertFinishFlag=0;
volatile char buffer[packLength]={0};

int main(void)
{
	char value[packLength]={0};
	int m;
	m_clockdivide(0);
	wirelessinitial();
	m_usb_init();
	while(!m_usb_isconnected()); // wait for a connection
	sei();
	while(1) {
		if (m_usb_rx_available()) {
			memset(value,0,packLength);
			m=0;
			while(m_usb_rx_available()) {
				if (m<packLength) {
					value[m] = m_usb_rx_char();
					m++;
				}else{ break; }
			}
			m_usb_tx_char('O');
			m_usb_tx_char('K');
			m_usb_tx_char('\n');
			m_wait(100);
			wireless_string(value,packLength);
		}
		
					
		if (ConvertFinishFlag==1) {
			for (int i=0;i<packLength;i++) {
				if (buffer[i]) {
					m_usb_tx_char(buffer[i]);
				}
			}
			ConvertFinishFlag=0;
		}

    }
}
ISR(INT2_vect )
{
	int success;
	success = m_rf_read(buffer, packLength);
	if ((success==1)&&(buffer[0]!=0)) {
		m_green(ON);
		ConvertFinishFlag=1;
	}else{
		m_green(OFF);
	}
}
