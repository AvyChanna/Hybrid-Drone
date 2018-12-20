// Do not remove the include below
#include "PlutoPilot.h"
#include "Control.h"
#include "Print.h"
#include "Sensor.h"
#include "App.h"
#include "Flight.h"
#include "Motor.h"
#include "Led.h"

#define ABS(x) ((x) > 0 ? (x) : -(x))

double initAccX =0;
double initAccY = 0;
double velX = 0;
double velY = 0;
double distX = 0;
double distY = 0;
double dist = 0;

int16_t m3;
int16_t m2;
int16_t m3f;
int16_t m2f;
int16_t throttle_value;
int16_t yaw_value;
int16_t roll_value;
int16_t pitch_value;
int16_t yaw_angle;
int16_t dir;
int16_t err;

bool isRover;

float sqrt(float number) {
   long i;
   float x2, y;
   const float threehalfs = 1.5F;

   x2 = number * 0.5F;
   y  = number;
   i  = * ( long * ) &y;                     // floating point bit level hacking [sic]
   i  = 0x5f3759df - ( i >> 1 );             // Newton's approximation
   y  = * ( float * ) &i;
   y  = y * ( threehalfs - ( x2 * y * y ) ); // 1st iteration
   y  = y * ( threehalfs - ( x2 * y * y ) ); // 2nd iteration
   y  = y * ( threehalfs - ( x2 * y * y ) ); // 3rd iteration

   return 1/y;
}

int16_t dead(int16_t in) {
	if(in>1450 && in<1550) return 1500;
	else return in;
}

//The setup function is called once at Pluto's hardware startup
void plutoInit()
{
// Add your hardware initialization code here
}



//The function is called once before plutoPilot when you activate Developer Mode
void onPilotStart()
{
  // do your one time stuffs here
	initAccX = Accelerometer.getX();
	initAccY = Accelerometer.getY();
	Control.disableFlightStatus(true);
	dir = Flight.getAngle(AG_YAW);

	ledOp(L_LEFT, OFF);
	isRover = true;
}



// The loop function is called in an endless loop
void plutoPilot()
{


	//Rover mode
	if(isRover) {
		throttle_value = Control.getRcData(RC_THROTTLE);
		yaw_value = Control.getRcData(RC_YAW);
		roll_value = Control.getRcData(RC_ROLL);
		pitch_value = Control.getRcData(RC_PITCH);
		yaw_angle = Flight.getAngle(AG_YAW);

		//Feedback error calculation
		if(yaw_value>1600 || yaw_value<1400 ) dir = yaw_angle;
		err = dir - yaw_angle;
		if(err > 180) err -= 360;
		else if(err < -180) err += 360;


		m2 = ((dead(pitch_value) - 1500)*2 + (dead(yaw_value) - 1500) + 10*err)/1.5;
		m3 = -(((dead(pitch_value) - 1500)*2 - (dead(yaw_value) - 1500) - 10*err)/1.5);

		if(m2>0) Motor.setDirection(M2, FORWARD);
		else Motor.setDirection(M2, BACKWARD);
		if(m3<0) Motor.setDirection(M3, FORWARD);
		else Motor.setDirection(M3, BACKWARD);

		m2f = 1000 + ABS(m2);
		m3f = 1000 + ABS(m3);

		Motor.set(M2, m2f);
		Motor.set(M3, m3f);
	}
	if(App.isArmSwitchOn()){


		//Distance Calculation
		velX += (Accelerometer.getX() - initAccX)*0.1;
		velY += (Accelerometer.getY() - initAccY)*0.1;
		distX += velX*0.1;
		distY += velY*0.1;
		dist = sqrt(distX*distX + distY*distY);
		if(dist > 400) {
			ledOp(L_LEFT, ON);
			Motor.set(M2, 1000);
			Motor.set(M3, 1000);
			isRover = false;
			Control.arm();
			Control.setCommand(COMMAND_TAKE_OFF);
		}
	}
	else {
		velX = 0;
		velY = 0;
		distX = 0;
		distY = 0;
		dist = 0;
	}

}



//The function is called once after plutoPilot when you deactivate Developer Mode
void onPilotFinish()
{

// do your cleanup stuffs here
		velX = 0;
		velY = 0;
		distX = 0;
		distY = 0;
		dist = 0;
		Motor.set(M2, 1000);
		Motor.set(M3, 1000);

}




