#include "localize.h"

#define 	STARS		4


const int ALLSTARS[12] ={ 1165 ,   874, 0 , 0 ,   1450 ,0,  -1056 , 248 , 0 , 0  ,  -1450 ,0 } ; /// <--- acutal star locations in 1/10 mm


void localize (char* wii_data){
	int bad_value_cnt=1;	
	if ( get_bad_values(wii_data) > 2 ){
		// ---------
		// implement drive to middle and get better look code
	} else {
	}

}



int get_bad_values(char* wii_packet){
	int bad_value_cnt=0;
	int i;
	for (i = 0: i < STARS * 3 ; i = i+3){
		if (wii_packet[i]==1023){
			bad_value_cnt++;
		}
	}
	return bad_value_cnt;

}
