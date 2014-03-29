
#include <avr/io.h>
#include "m_imu.h"
#include "m_bus.h"
#include "m_usb.h"
#include "m_general.h"


int main(void)
{
	m_clockdivide(0);
	unsigned char accel_scale = 0;
	unsigned char gyro_scale = 0;
	int data[9] = {0,0,0,0,0,0,0,0,0};
	m_usb_init();
	m_imu_init(accel_scale, gyro_scale);
	m_green(ON);
	while (!m_usb_isconnected());
	m_usb_tx_string("ready\n");
	m_green(OFF);
	while(1)
    	{
		m_wait(100);
		m_imu_raw(data);
		m_usb_tx_int(data[0]);
		m_usb_tx_char('\t');
		m_usb_tx_int(data[1]);
		m_usb_tx_char('\t');
		m_usb_tx_int(data[2]);
		m_usb_tx_char('\t');
		m_usb_tx_int(data[3]);
		m_usb_tx_char('\t');
		m_usb_tx_int(data[4]);
		m_usb_tx_char('\t');
		m_usb_tx_int(data[5]);
		m_usb_tx_char('\n');
		m_usb_tx_char('\r');
    }
}
