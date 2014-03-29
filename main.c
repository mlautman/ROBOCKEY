/*
 * main..c
 *
 * Created: 14/11/2012 08:40:57 PM
 *  Authors: Michael Lautman, Oliver Pacchiana, Praveer Nidamaluri
 */ 

#include "m_general.h"
#include "m_bus.h"
#include "m_rf.h"
#include "m_usb.h"
////#include "m_port.h"
#include "m_wii.h"
#include <stdlib.h>
#include <math.h>

#define 	p_len		12
#define 	chan  	 	1
#define 	rf_address 	0x44
#define 	PORT_ADD	0x20

#define 	commTest	0xA0
#define 	play		0xA1
#define 	goalA		0xA2
#define 	goalB		0xA3
#define 	pause		0xA4
#define 	halfTime   	0xA6
#define 	gameOver   	0xA7


//constants like star number and number of used led's
#define 	STARS		4
#define 	X_BOT		512
#define 	Y_BOT		384
#define 	PI  		3.14159
#define 	RAD_DEG_RATIO	57

//variables that shouldn't change in code. 
#define 	H_LEDS		0 		//led's in use on the 10seg led
//#define 	maxPWM 	 	150		// <<<< max OCR1B and OCR1C values
#define         minPWM          190             // min OCR1B and OCR1C until it can no longer hold puck
#define 	D_t  		0.0	    	//  <---- change in time between readings 
//#define 	wallAvoidDist	10
#define black 1
#define white 0

bool play_cmd = false;


int maxPWM = 900;
int pwmMin = minPWM; //minimum pwm before stall
int kp_partial_theta = 15.0; //adjusting for an off angle
int spin_speed = 200;
int adc_spin_speed = 600;
int last_theta_angle = 0; 
int robot = white; //black wheels
char spinLeft = 3;
char keepSpin = 0;
bool atLocation = false;
bool atAngle = false;
//#define JStrokePWMFull 700
//#define JStrokePWMPartial 350
//#define nonJStrokePWM 550

///////////////////////////////////////////////////////////////
////		setup funcitons
///////////////////////////////////////////////////////////////
void gen_setup(void);
void setup_timer_1(void);
void setup_timer_3(void);
void setup_pins(void);

///////////////////////////////////////////////////////////////
////		usb debug functions and variables
///////////////////////////////////////////////////////////////
void matlab_output(int*, float* , unsigned int* );
bool usb_conn_test=false;
bool Matlab_pos_tracking = false;
bool go_to_location_test = false;
bool debug_com = false;
bool debug_goto= false;
bool debug_find_puck = false  ;
bool get_unstuck = false ;
bool debug_timer = false;

bool no_comm_version = false ;


///////////////////////////////////////////////////////////////
////		Wireless stuff 
///////////////////////////////////////////////////////////////
// vars
char rf_buffer[ p_len ] = {0};
volatile bool comm_cmd = false;
volatile unsigned char state_req = 0x00;   //    <---- TEST IF WE CAN REMOVE UNSIGNED 
bool new_play =false; 

// funcitons
void get_command( void );

///////////////////////////////////////////////////////////////
////		goToLocation parameters and vars 
///////////////////////////////////////////////////////////////

//avoid top if facing top  and above wall_avoid_dist
// funcitons
//bool gotoLocation(int,int,int*,float);
bool drivePuck(int , int  , int*,float) ;
bool goToAngle(int,  float*);
bool sprintToLocation(int , int  , int* , float*) ;


///////////////////////////////////////////////////////////////
////		PuckFinding parameters and vars
///////////////////////////////////////////////////////////////

// vars
bool findpuck = false;
int photo_d1 = 0;
int photo_ar = 0;
int photo_al = 0;
char has_puck = 0;
bool searching = false;
volatile char photo_d = 0;

int 	lastPhotoDiff	= 	0;
int last_photo_ar = 0;
int last_photo_al = 0;

char direction = 0; 

// funcitons
void setup_ADC(void );
void setup_puckfinding(void);
int readADC(char);
void find_puck(void);		// Main Find Puck function
void ADC_go_to_puck(void);



///////////////////////////////////////////////////////////////
////		State Machine Stuff
///////////////////////////////////////////////////////////////
#define 	PT_super	0x00
#define		ADC_super	0x01
#define		New_Loc_super	0x02
#define 	To_Goal_super	0x03
#define         state_debug     0x04

//#define 	PT_LF		0x20
//#define 	PT_LS		0x10
//#define	PT_RF		0x02
//#define	PT_RS		0x01
//#define	PT_F		0x11
//#define 	PT_C		0xAA

// vars
char Super_state	= 0x00 ;
char last_Super_state = 0x00;
char PT_state		= 0x00 ;
char Location_State 	= 0x00 ;
int goalX		= 180 ; 
int goalY 		= 5;
// functions 

///////////////////////////////////////////////////////////////
////		localization stuff
///////////////////////////////////////////////////////////////
// vars
char scale = 1; 
int position[2] = {0};
int positionX = 0;
int positionY = 0;
float b_angle[1]={0.0};
int last_pos[2] = {0};
float velocity = 0;

int pos_change = 7;
// funcitons
void localize(unsigned int*, int*, float*);

void set_motors( int  , int  );
void rotate(int );
int lastLeft = 0;
int lastRight = 0;
int goalAngle = 0;

int rxOG = 0;
int ryOG = 0;
int psiOG = 0; 

//error in angle, factoring in rotational velocity should be taken care of by kalman filter
int phi_robotOG = 0; // compute angle from robot to target (in global coordinates)	
int theta_angleOG = 0;
///////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
//					code   
//
///////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////

unsigned int timer_3_cnt = 0; 

char time_in_X =100; 


int main(void) {
	m_clockdivide(0);

	gen_setup();

	unsigned int wii_data[12] = {0};

	sei();
	m_rf_open(chan,rf_address,p_len);
	m_wii_read(wii_data);
	localize( wii_data , position, b_angle); // <--- takes in wii_data and updates position in place to house {x,y,angle} of robot 

	int enemy_goalX; 
	int enemy_goalY = 0;
	int own_goalX;
	int own_goalY = 0;
	int goalXPosition = 120;

	if ((int)(position[0])<0){ 	enemy_goalX = goalXPosition; 
		own_goalX = -goalXPosition;}

	else { 			enemy_goalX = -goalXPosition; 
		own_goalX = goalXPosition;}


	find_puck();
	set_motors(800,800);
	//bool  toggle = true;

	int lastX = 0;
	int lastY = 0;
	int lastAng = 0;

	while (1) {
		//find angle to own Goal
		rxOG = own_goalX - (int)position[0];
		ryOG = own_goalY - (int)position[1];
		psiOG = (int)(b_angle[0] *  RAD_DEG_RATIO) - 90; 
		if (psiOG<-180) { psiOG +=360; }
		if (psiOG>180) { psiOG -=360; }

		//error in angle, factoring in rotational velocity should be taken care of by kalman filter
		phi_robotOG = (int)(atan2(ryOG,rxOG) * RAD_DEG_RATIO); // compute angle from robot to target (in global coordinates)	
		theta_angleOG = (psiOG-phi_robotOG);
		//make sure the data within -180 to 180 degrees for ease of calculation
		if (theta_angleOG>180) { theta_angleOG-=360; }
		if (theta_angleOG<=-180) { theta_angleOG+=360;}

		/*if (debug_goto) {
		  m_usb_tx_string("\n\r");
		  m_usb_tx_int(theta_angleOG);
		// m_usb_tx_string("\t");
		//m_usb_tx_int(OCR1B);
		m_usb_tx_string("\n\r"); 
		}*/

		last_Super_state = Super_state;
		if (comm_cmd  == true)	{ get_command(); }
		if (check(TIFR3,OCF3A)){		 ///// timer 3 runs at 100Hz
			set(TIFR3,OCF3A);			
			if (timer_3_cnt == 1000){ timer_3_cnt = 0 ;   } else{ timer_3_cnt++;} // timer_3_cnt counts 10 seconds and then rolls over

			m_wii_read(wii_data);
			localize( wii_data , position, b_angle); // <--- takes in wii_data and updates position in place to house {x,y,angle} of robot 
			m_rf_open(chan,rf_address,p_len);


			find_puck();
			if (get_unstuck && timer_3_cnt%100 == 0){
				if ( (( lastX < position[0]+pos_change) && lastX > position[0]-pos_change)   
						&& ( lastY < position[1]+pos_change && (lastY > position[1]-pos_change)  ) ){
					//	&& ( ((int)lastAng < (int)b_angle*100 +20) && ((int)lastAng > (int)b_angle*100 - 20) )){
					Super_state = New_Loc_super;
					set_motors(-700,-700);
					m_wait(200);
					set_motors(0,0);
				}
				lastX = (int)position[0]; 
				lastY = (int)position[1];
				lastAng = (int)b_angle*100.0;
				}

				switch (Super_state){			
					case PT_super: 
						break;
					case ADC_super: 
						ADC_go_to_puck();
						break;
					case To_Goal_super: 
						drivePuck(enemy_goalX,  enemy_goalY ,position, b_angle[0]);
						break; 
					case New_Loc_super:	 
						break;
				}
				if (Super_state != To_Goal_super) keepSpin = 0;

				if ( Matlab_pos_tracking ){ matlab_output( position, b_angle , wii_data ); }
			}
		}
	}

	void find_puck(void){
		//TODO make this usable

		/* IR pt  L C P R	 */
		photo_d1 = ~(PINB & 0x0F) &0x0F ;


		if ((photo_d1 & 0b00000100) != 0b00000000){	
			//		if (Super_state == PT_super);//  if (direction == 1){rotate(-1);} else rotate(1);
			Super_state = ADC_super; 
		}
		if ((photo_d1 & 0b00000010) != 0b00000000){ 	Super_state = To_Goal_super;  }
		if ( (photo_d1 & 0b00000110) == 0 ) {		Super_state = PT_super; }

		switch (photo_d1){
			case 0b00000000:	rotate(0);  break; 
			case 0b00000001: 	rotate(1); break; 
			case 0b00001000: 	rotate(-1); break; 
			case 0b00001001: 	if (position[1]>0.0 ) // robot is to the top of the board
							if( (int)(10*position[2]) > -15  && (int)(10*position[2]) <  15 ) { rotate(1);}
							else { rotate(-1);}
						else {
							if ((int)(10*position[2]) > -15  && (int)(10*position[2]) < 15 ){ rotate(-1);}
							else { rotate(1); }
						}break;
		}	

		if (debug_find_puck){
			photo_ar = readADC(0);
			photo_al = readADC(1);
			m_usb_tx_string("\t digital left: \t "); 	m_usb_tx_int(((photo_d1 >> 3) & 0x01));
			m_usb_tx_string("\t digital cent: \t "); 	m_usb_tx_int(((photo_d1 >> 2) & 0x01));
			m_usb_tx_string("\t digital right: \t "); 	m_usb_tx_int(((photo_d1 >> 0) & 0x01));
			m_usb_tx_string("\t digital near: \t "); 	m_usb_tx_int(((photo_d1 >> 1) & 0x01));
			m_usb_tx_string("\t analog L: \t "); 		m_usb_tx_int( photo_al );
			m_usb_tx_string("\t analog R: \t "); 		m_usb_tx_int( photo_ar);
			m_usb_tx_string("\t");
			m_usb_tx_int(Super_state);
			m_usb_tx_string("\n\r");
		}
	}	 
	void rotate( int dir ){
		if ( dir ==0 ) dir = direction;
		direction = dir;
		if (dir >0)  	{ set_motors(  spin_speed, -spin_speed );}
		else  	{ set_motors( -spin_speed,  spin_speed );}
	}


	int readADC(char a){
		set(ADCSRA,ADEN);

		if(a==0){
			// Set ADC input as F0 (left)
			clear(ADCSRB,MUX5); clear(ADMUX,MUX2); clear(ADMUX,MUX1); clear(ADMUX,MUX0);	
			// Begin conversion		 
			set(ADCSRA,ADSC);								
			// Wait until conversion is finished.
			while(check(ADCSRA,ADSC));							
			// Disable ADC subsystem
			clear(ADCSRA,ADEN);								
			//k = ADC;
			return ADC; }

		else if(a==1){
			// Set ADC input as F1 (right)
			clear(ADCSRB,MUX5); clear(ADMUX,MUX2); clear(ADMUX,MUX1); set(ADMUX,MUX0);	
			// Begin conversion		 
			set(ADCSRA,ADSC);								
			// Wait until conversion is finished.
			while(check(ADCSRA,ADSC));							
			// Disable ADC subsystem
			clear(ADCSRA,ADEN);			
			return ADC; }	
		else {return -1;} 
	}
	void set_motors( int left, int  right ){
		//OCR1B is right motor
		//OCR1C is left motor
		//clear is forward, set is backward
		int right_lag_offset = 0;
		int left_lag_offset = 0;
		if (robot == black) { //black wheels
			right_lag_offset = 0;//22
			left_lag_offset = (float)left*0.025;//0
		}
		if (robot == white) { //white wheels
			right_lag_offset = 0;
			left_lag_offset = (float)left*.0476+15;
		}
		lastRight = right;
		lastLeft = left;
		if (no_comm_version || play_cmd){

			if (right < 0 ){    
				right -= right_lag_offset;
				OCR1B = -right; }
			else { 
				right += right_lag_offset;
				OCR1B  = right ;  };
			if ( left < 0 ){ 
				left -= left_lag_offset;
				OCR1C = -left; }
			else { 
				left += left_lag_offset;
				OCR1C  = left;
			}
			if ( left >= 0 ) clear(PORTC, 6);
			else set(PORTC,6);

			if (  right >= 0 ) { clear(PORTC, 7); }
			else set(PORTC,7);
		}
		if ( !play_cmd && !no_comm_version ) {OCR1B = 0; OCR1C = 0;}

		//OCR1B = 600;
		//OCR1C = 630;
		//clear(PORTC, 7); clear(PORTC, 6);

		/*  	if (debug_goto) {
			m_usb_tx_string("\n\r");
			m_usb_tx_int(left);
			m_usb_tx_string("\t");
			m_usb_tx_int(right);
			m_usb_tx_string("\n\r"); 
			}*/


	}

	void ADC_go_to_puck(){
		float 	kp_puck_offset		=  	1;
		float kd_puck_offset = 0;//100;
		float kp_puck_slow = 0;
		float kd_puck_slow = 0;
		if (abs(theta_angleOG)<120){
			kp_puck_slow = 0;//.2;
			kd_puck_slow = 25;
		}

		//perform a J-Stroke if facing own goal, go straight at puck if facing other goal
		photo_ar = readADC(0);
		photo_al = readADC(1);
		int diff = (photo_al-photo_ar);
		//int d_diff = diff - lastPhotoDiff;

		//slow down as you approach the puck
		int d_photo = abs(photo_ar -last_photo_ar)+abs(photo_al-last_photo_al); //difference in photo val
		int d_offset_photo = abs(photo_al - last_photo_al) + abs(last_photo_ar - photo_ar);
		int kd_photo = d_photo*kd_puck_slow;
		int kp_photo = kp_puck_slow*(photo_ar+photo_al);
		int slow = kd_photo + kp_photo;

		//differential turn to puck
		int kp_diff = diff*kp_puck_offset;
		int kd_diff = d_offset_photo*kd_puck_offset;
		int offset = kp_diff;


		last_photo_ar = photo_ar;
		last_photo_al = photo_al;

		lastPhotoDiff =  diff;

		int adcPWM = 700; 
//		adcPWM -= (int)(float)(photo_ar + photo_al) /* (velocity)*/    ; 
		//int offset = (int)((float) diff * kp_puck);

		int photo_left; int photo_right;
		if (offset< 0 ) {
			photo_left = adcPWM - slow;
			photo_right = adcPWM -  abs(kp_diff) + kd_diff - slow;
		}else	{ photo_left = adcPWM - abs(kp_diff ) + kd_diff - slow;
			photo_right = adcPWM - slow ;
		}if (photo_left < pwmMin) photo_left = pwmMin;
		if (photo_right < pwmMin) photo_right = pwmMin;
		set_motors(photo_left,photo_right);
		if (debug_goto) {
			m_usb_tx_string("\n\tphoto_ar: ");
			m_usb_tx_int(photo_ar);
			m_usb_tx_string("\tphoto_al: ");
			m_usb_tx_int(photo_al);
			m_usb_tx_string("\tkd_photo: ");
			m_usb_tx_int(kd_photo);
			m_usb_tx_string("\tkp_photo: ");
			m_usb_tx_int(kp_photo);
			m_usb_tx_string("\n\r"); 
		}
	}


	///////////////////////////////////////////////////////////////////////////////////////////////////////////////
	///////////////////////////////////////////////////////////////////////////////////////////////////////////////
	//
	//					go to stuff   
	//
	///////////////////////////////////////////////////////////////////////////////////////////////////////////////
	///////////////////////////////////////////////////////////////////////////////////////////////////////////////
	bool goToAngle(int toGoAngle, float* locationData) {
		//spin in place until facing angle. motors will turn at same speed, but at opposite directions

		int motorDiffkp  = 10;
		int motorDiffMax = 200;
		int maxAngle = 35;

		//get psi so it is between 180 and -180, 0 being at one goal

		int psi = (int)(locationData[0] *  RAD_DEG_RATIO) - 90; 
		if (psi<-180) { psi +=360; }
		if (psi>180) { psi -=360; }

		//error in angle, factoring in rotational velocity should be taken care of by kalman filter
		int theta_angle = (psi-toGoAngle);

		int atAngleError = 10;

		if (theta_angle > 180 ) { theta_angle -= 360; } 
		if (theta_angle < -180) { theta_angle += 360; }

		//motorDiff is the difference in PWM between the two motors
		int motorDiff = (int)((float)(motorDiffkp)*(float)(abs(theta_angle))/(float)maxAngle+minPWM-20);

		if (abs(theta_angle) > maxAngle) { motorDiff = motorDiffMax; }

		if (theta_angle >0) {  set_motors( motorDiff, -motorDiff ); }
		else {			set_motors( -motorDiff, motorDiff ); }

		if ( abs(theta_angle) < atAngleError) { atAngle = true; return true;}
		else {atAngle = false; return false;}

	}

	bool drivePuck(int toGoX, int toGoY , int* locationData, float locationDataAngle) {
		//location data is [xposition,yposition,angle w/x-axis]

		//get vector to desired location  
		int rx = toGoX - (int)locationData[0];
		int ry = toGoY - (int)locationData[1];

		int atLocationError = 5;
		if ( abs(rx) < atLocationError && abs(ry) < atLocationError){
			return true ;
		}

		//psi is angle robot is facing
		//get psi so it is between 180 and -180, 0 being at one goal	
		int psi = (int)(locationDataAngle * RAD_DEG_RATIO) - 90;
		if (psi<-180) { psi +=360; }
		if (psi>180) { psi -=360; }

		//tao is angle off from vertical, 0 if facing north or south, negative if facing northeast or southwest
		int tao;
		if (psi > 0)  { tao = psi - 90; }
		else 	{ tao = psi + 90; }

		int phi_robot = (int)(atan2(ry,rx) * RAD_DEG_RATIO); // compute angle from robot to target (in global coordinates)
		int theta_angle = (psi-phi_robot); //error in angle, factoring in rotational velocity should be taken care of by kalman filter

		//make sure the data within -180 to 180 degrees for ease of calculation
		if (theta_angle>180) { theta_angle-=360; }
		if (theta_angle<=-180) { theta_angle+=360;}

		//turn on wall avoidance if necessary. Always turn away toward center
		char wallAvoidFlag = 0;
		int wallAvoidDist = 0;//relative to center; avoid wall if locationData[1] is greater than this
		int wallAvoidDistBottom = wallAvoidDist;
		//avoid top if facing top  and above wall_avoid_dist
		float wallDis = 60;
		int wallDisMax = 30;
		int wallLocation = 60;
		//avoid top wall if within certain distance, and facing wall
		if ((int)locationData[1]>wallAvoidDist && psi>0 ) { 
			wallAvoidFlag = 1; 
			wallDis = ((float)wallLocation-(float)locationData[1])/cos((float)tao/(float)RAD_DEG_RATIO);	 }

		//avoid bottom wall
		else if ((int)locationData[1]< -wallAvoidDistBottom && psi<0 ) { 
			wallAvoidFlag = 2; 
			wallDis = ((float)wallLocation-(float)abs((float)locationData[1]))/(cos((float)tao/(float)RAD_DEG_RATIO));} 
		else {
			wallAvoidFlag = 0; 
			wallDis = wallDisMax;}//wallAvoidDist; }

	//adjust speeds to as to avoid walls and self-goals
	//pwmMax is lessened as you approach a wall, and as you face your own goal
	//pwmMin is lessened as you face your own goal, so as to turn faster
	int wall_adjusted = wallDisMax-wallDis; //is highest at wall, 0 far from wall
	if (wall_adjusted < 0) { wall_adjusted = 0;}

	if (keepSpin == 0) {
		//turn toward center if in wall avoidance mode
		if (wallAvoidFlag == 1 || wallAvoidFlag == 2) {
			if (tao>0) {	 spinLeft = 1; }
			if (tao<0) {    	 spinLeft = 0; }
		}
		//turn regularly since not in wall avoidance mode
		else if (theta_angle > 0 ) { spinLeft = 0; }
		else { spinLeft = 1; }
	}

	//float kp_full_theta  = 365.0;//2.5;//how quickly the full motor gets turned down with angle
	//float kp_full_dis = 0;//how quickly the full motor gets turned down with distance
	//float kp_partial_theta = 2.0;//how quickly the partial motor gets turned down with angle
	int fullSpeedAngle = 30;
	int turnmax = minPWM + 50;
	int turnmin = minPWM;
	int fullMotor = turnmax;
	int partialMotor = turnmin;

	//if within fullSpeedAngle degrees of target, full speed and rapid adjustment
	if (abs(theta_angle) < fullSpeedAngle) {
		fullMotor = maxPWM;
		partialMotor = (int)(fullMotor - kp_partial_theta*(float)abs(theta_angle)); //adjust partial motor for facing angle
		keepSpin = 0;
	}
	else keepSpin = 1; //keep the spin direction if not facing the opposing goal. This eliminates snaking.

	if (wallDis<0) { wallDis = 0;}

	//errors in negative partialMotor, etc.
	if (partialMotor < minPWM) { partialMotor = minPWM; }
	if (partialMotor > maxPWM) {partialMotor = maxPWM; }
	//errors in negative partialMotor, etc.
	if (fullMotor < minPWM) { fullMotor = minPWM; }
	if (fullMotor > maxPWM) {fullMotor = maxPWM;}

	//set the port outputs correctly for the given direction
	//if (theta_angle*last_theta_angle<0){
	if (spinLeft == 1) { 
		set_motors( partialMotor, fullMotor );
		//printf("OCR1B = %i \nOCR1C = %i \n",OCR1B,OCR1C);
	}

	else { 
		set_motors( fullMotor, partialMotor );
		//printf("OCR1B = %i \nOCR1C = %i \n",OCR1B,OCR1C);
	}
	//	}

	/*
	   if (debug_goto){
	   m_usb_tx_string("\n\r");
	   m_usb_tx_int(locationData[0]);
	   m_usb_tx_string("\t");
	   m_usb_tx_int(locationData[1]);
	   m_usb_tx_string("\t");
	   m_usb_tx_int(psi);
	   m_usb_tx_string("\n\r"); 
	   }
	 */

	last_theta_angle = theta_angle;

	return false;
}


bool sprintToLocation(int toGoX, int toGoY , int* location, float* locationData) {
	int sprintSpeed = maxPWM; //speed to travel from afar
	int closeDis = 50; //start slowing down
	float closeRatio = .3; //percentage of sprintSpeed to come if close
	int atAngleError = 45;
	int atLocationError = 8;
	//turns to angle, then goes directly to location
	//get vector to desired location  
	int rx = toGoX - (int)location[0];
	int ry = toGoY - (int)location[1];

	int dis = (int)(sqrt(((float)rx)*rx + ((float)ry)*ry));

	//psi is angle robot is facing
	//get psi so it is between 180 and -180, 0 being at one goal	
	int psi = (int)(locationData[0] *  RAD_DEG_RATIO) - 90; 
	if (psi<-180) { psi +=360; }
	if (psi>180) { psi -=360; }

	//error in angle, factoring in rotational velocity should be taken care of by kalman filter
	int phi_robot = (int)(atan2(ry,rx) * RAD_DEG_RATIO); // compute angle from robot to target (in global coordinates)	
	int theta_angle = (psi-phi_robot);

	/*
	   if (debug_goto){
	   m_usb_tx_string("\n\r dis: ");
	   m_usb_tx_int((int)dis);
	   m_usb_tx_string("\t x: ");
	   m_usb_tx_int((int)locationData[0]);
	   m_usb_tx_string("\t y: ");
	   m_usb_tx_int((int)locationData[1]);
	   m_usb_tx_string("\n\r"); 
	   }
	 */

	if ( dis < atLocationError){
		set_motors(0,0);
		atLocation = true;
		return true; }
	else if ( abs(theta_angle) > atAngleError){
		goToAngle(phi_robot,locationData);
		atLocation = false;
		return false; 
	}

	else {
		int fullMotor = sprintSpeed;
		int partialMotor = (int)(fullMotor - kp_partial_theta*(float)abs(theta_angle));  //adjust partial motor for facing angle
		if (dis<closeDis){partialMotor = partialMotor*closeRatio; fullMotor = fullMotor*closeRatio;}
		//errors in negative partialMotor, etc.
		if (partialMotor < minPWM) { partialMotor = minPWM; }
		if (partialMotor > maxPWM) {partialMotor = maxPWM; }
		//errors in negative partialMotor, etc.
		if (fullMotor < minPWM) { fullMotor = minPWM; }
		if (fullMotor > maxPWM) {fullMotor = maxPWM;}

		if (theta_angle < 0)
			set_motors(partialMotor,fullMotor);
		else
			set_motors(fullMotor,partialMotor);
		atLocation = false;
		return false;}
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
//					comm   
//
///////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////

void get_command(){

	comm_cmd = false;
	state_req = rf_buffer[0];

	m_red(OFF);
	//	int i; 
	//for ( i = 0 ; i < H_LEDS ; i++){
	//	m_port_clear(PORT_ADD,PORTH,i);
	//}

	if(state_req == commTest){  m_red(2);					}
	else if(state_req == play){ set_motors(800,800); 	new_play=true;			}
	else if(state_req == pause){ set_motors(0,0);}
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
//					localization   
//
///////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////
void localize(unsigned int* mWii_packet, int* localizationData, float* bot_angle){
	// SUBROUTINE VARIABLES

	velocity = sqrt((float)(last_pos[0]*last_pos[0] + localizationData[0]*localizationData[0]));

	last_pos[0] = localizationData[0];
	last_pos[1] = localizationData[1];
	int X[STARS] = {0}; 	int Y[STARS] = {0}; 
	long Dists[6]= {1};  	//  dists[0] = s0s1 [1] = s0s2  [2] = s0s3 [3] = s1s2 [4] = s1s3 [5] = s2s3 

	int s_map[STARS] = {0};	// star map...will probably get rid of in future versions 
	int avail_stars = 0;

	// fills X[] and Y[] with all of the good star readings
	int cnt;
	m_red(OFF);
	for (cnt = 0 ; cnt< STARS;cnt++){
		if (mWii_packet[cnt*3] == 1023){  /* m_red(ON);*/  }
		//else {
		X[avail_stars]=mWii_packet[ 3*cnt ]     -  X_BOT;
		Y[avail_stars]=mWii_packet[ 3*cnt + 1]  -  Y_BOT;
		avail_stars++;
		//}
	}	

	// computes distances between all available stars 
	int distIndex=0; 	int index;	 int other;

	for (index = 0; index< avail_stars ; index++){
		for(other = index+1; other < avail_stars; other ++){
			Dists[distIndex] = pow((X[index] - X[other]),2) + pow(( Y[index] - Y[other] ),2);
			distIndex++;
		}
	}
	// finds largest inter star distance 
	int minIndex=-1;	 int maxIndex=-1;
	long  max = 0;		 long  min = 90000; 
	for (index = 0 ; index < 6 ; index++){
		if (Dists[index] < min)  { min  = Dists[index]; 	minIndex = index; }
		if ( Dists[index] > max ){ max  = Dists[index]; 	maxIndex  = index; }
	}

	/* 0 : 1	 0 : 2	 0 : 3	 1 : 2	 1 : 3	 2 : 3 */
	switch (maxIndex){
		case 0: switch (minIndex) {
				case 1: case 2:		s_map[1] = 0; s_map[3] = 1; break; 
				case 3: case 4: s_map[1] = 1; s_map[3] = 0; break; 
				default: m_red(ON); } break;
		case 1: switch (minIndex) {
				case 0: case 2: s_map[1] = 0; s_map[3] = 2; break;
				case 3: case 5:		s_map[1] = 2; s_map[3] = 0; break; 
				default: m_red(ON); } break;
		case 2: switch (minIndex) {
				case 0: case 1: s_map[1] = 0; s_map[3] = 3; break; 
				case 4: case 5: s_map[1] = 3; s_map[3] = 0; break;
				default: m_red(ON); } break;
		case 3: switch (minIndex) {
				case 0: case 4: s_map[1] = 1; s_map[3] = 2; break; 
				case 1: case 5: s_map[1] = 2; s_map[3] = 1; break;
				default: m_red(ON); }	break;
		case 4: switch (minIndex) {
				case 0: case 3: s_map[1] = 1; s_map[3] = 3; break;
				case 2: case 5: s_map[1] = 3; s_map[3] = 1; break;
				default: m_red(ON) } break;
		case 5: switch (minIndex){
				case 1: case 3: s_map[1]=2; s_map[3]=3; break;
				case 2: case 4: s_map[1] = 3; s_map[3] = 2; break;
				default: m_red(ON); } break;
		default: m_red(ON);	
	}

	float s_Vert = sqrt(max);  		// find the pixel dist b/w stars 2 & 4
	float scale  = s_Vert/29.0;		// find the ration of pixel/cm

	// find the location of global center in local coordinates.
	long x_cent, y_cent;
	x_cent = ( X[ s_map[1] ] + X[s_map[3]] ) / 2.0 ;		// define the local position of the origin
	y_cent = ( Y[ s_map[1] ] + Y[s_map[3]] ) / 2.0 ;		// define the local position of the origin

	// find the dist from bot to global center
	long r = (long) sqrt( (float)x_cent*x_cent +(float)y_cent*y_cent)/scale;

	// find the angle between the local frame and the global frame
	float theta = - atan2(  X[ s_map[1] ] - X[s_map[3]]  , Y[ s_map[1] ] - Y[s_map[3]] );

	// find the angle between the local x-axis and the vector to the global origin
	float alpha = atan2( y_cent , x_cent );

	// find the angle between the global x-axis and the vector to the origin
	float phi = -theta +alpha;

	int XBOT =(int) r*cos(phi);		// determine the bot's global position
	int YBOT =(int) -r*sin(phi);		// ^

	position[0] =(int) XBOT;// x_cent/scale;
	position[1] =(int) YBOT; //y_cent/scale;
	bot_angle[0]= theta - PI/2;

}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
//					Interrupt handlers  
//
///////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////

/*ISR( TIMER3_COMPA_vect ){
  }*/

ISR( INT2_vect) { 

	m_rf_read(rf_buffer,p_len);
	state_req = rf_buffer[0];

	//	int i; 
	//for ( i = 0 ; i < H_LEDS ; i++){
	//	m_port_clear(PORT_ADD,PORTH,i);
	//}

	if(state_req == commTest){  m_green(2);		}
	else if(state_req == play){ m_green(1); 	play_cmd=true;			}
	else if(state_req == pause){	 m_green(0); play_cmd=false;}

	m_green(TOGGLE);
}	



///////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
//					SETUP functions 
//
///////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////

void gen_setup(void){
	usb_conn_test = Matlab_pos_tracking ||  go_to_location_test || debug_com || debug_goto || debug_find_puck; 
	if (usb_conn_test ){
		m_usb_init();
		m_green(ON);
		while (!m_usb_isconnected());
		m_green(OFF);
	} 
	m_bus_init();
	m_wii_open();	
	// m_port_init(PORT_ADD);

	setup_pins();

	setup_puckfinding();		//Puck Finding Intialization

	setup_timer_1();
	setup_timer_3();
	setup_ADC();


}
void setup_pins(){
	set(DDRC,6);		set(DDRC,7);		//setup motor direction lines as outputs 
	clear(PORTC,6);		clear(PORTC,7);		//set default motor dir to be forward 
	set(DDRB,6); 		set(DDRB,7); 		// Controll B6 with the timer

}

void setup_puckfinding(void){

	//	photo_d1 = ~(PINB & 0x0F) &0x0F ;	//negate photo_d1
	//	set(PCICR,PCIE0);			// Enable pin change interrupts
	//	PCMSK0 = PCMSK0 | 0x0F;			// Unmask pins to enable pin change interrupts
	DDRB = DDRB  & 0xC0 ;			// Set pins as inputs for photo transistor
	PORTB = PORTB |  0x3F;	 		// Enable internal pull-up resistors
}



///////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////				timer 3			(timer for other stuff) 
///////////////////////////////////////////////////////////////////////////////////////////////////////////////

void  setup_timer_3(){

	OCR3A =156  ; 				//0x61B = 1563 >> 10Hz clock

	set(TCCR3B, CS32); 	clear(TCCR3B, CS31); 	set(TCCR3B, CS30);	// <-- clock Prescaller 16MHz/1024
	clear(TCCR3B, WGM33);	set(TCCR3B, WGM32);
	clear(TCCR3A, WGM31);	clear(TCCR3A, WGM30);	//(mode 4) UP to OCR3A
	//	set(TIMSK3,TOIE3);
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////				timer 1 
///////////////////////////////////////////////////////////////////////////////////////////////////////////////
void  setup_timer_1(){
	OCR1A = 1015;		OCR1B=0; 		OCR1C=0;
	// Setting up clock divider		
	clear(TCCR1B,CS12); 	set(TCCR1B,CS11); 	set(TCCR1B, CS10);
	// Set mode15 on timer 1
	set(TCCR1B,WGM13); 	set(TCCR1B,WGM12); 	set(TCCR1A,WGM11);	 set(TCCR1A,WGM10); 	
	// Match b/w TCNT1 and OCR1x clears, set at rollover
	set(TCCR1A,COM1B1); 	clear(TCCR1A,COM1B0);	
	set(TCCR1A,COM1C1); 	clear(TCCR1A,COM1C0);	
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////				ADC setup 1 
///////////////////////////////////////////////////////////////////////////////////////////////////////////////
void setup_ADC(){
	// Set Voltage Reference to Vcc
	clear(ADMUX,REFS1);	set(ADMUX,REFS0);
	// set ADC clock prescalar to 16Mhz/128 = 125kHz
	set(ADCSRA,ADPS2); 	set(ADCSRA,ADPS1); 	set(ADCSRA,ADPS0);
	// Disable input circuitry for ADC0 - 7
	DIDR0 = DIDR0 | 0x03;		
	// Switch to single conversion mode
	clear(ADCSRA,ADATE);	

}




///////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
//					debug functions 
//
///////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////

void matlab_output( int* position,float* angle, unsigned int* wii_data){
	if( m_usb_rx_available() ){
		m_usb_tx_int( ( int)position[0] );
		m_usb_tx_char('\n');
		m_usb_tx_int( ( int)position[1] );
		m_usb_tx_char('\n');
		m_usb_tx_int( ( int )(1000*angle[0]) );
		m_usb_tx_char('\n');
		m_usb_tx_uint(wii_data[0]);
		m_usb_tx_char('\n');
		m_usb_tx_uint(wii_data[1]);
		m_usb_tx_char('\n');
		m_usb_tx_uint(wii_data[3]);
		m_usb_tx_char('\n');
		m_usb_tx_uint(wii_data[4]);
		m_usb_tx_char('\n');
		m_usb_tx_uint(wii_data[6]);
		m_usb_tx_char('\n');
		m_usb_tx_uint(wii_data[7]);
		m_usb_tx_char('\n');
		m_usb_tx_uint(wii_data[9]);
		m_usb_tx_char('\n');
		m_usb_tx_uint(wii_data[10]);
		m_usb_tx_char('\n');

		m_usb_rx_flush();
	}
}

