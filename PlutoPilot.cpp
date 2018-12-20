#include "PlutoPilot.h"
#include "Control.h"
#include "Flight.h"
#include "Motor.h"
#include "Print.h"
#include "App.h"
#include "Sensor.h"

int abs(int x) {return (x>0)?x:-(x);}

int current_heading;
int current_roll;
int current_pitch;
int set_heading;
bool is_armed;
int heading_error;

int m3,m2;
int m3_pwm,m2_pwm;

void plutoInit()
{
    current_roll = 0;
    current_heading = 0;
    heading_error = 0;
    current_pitch = 0;
    set_heading = 0;
    is_armed = false;
}


void onPilotStart()
{
    set_heading = Flight.getAngle(AG_YAW);
    current_roll = 0;
    current_heading = 0;
    heading_error = 0;
    current_pitch = 0;
    set_heading = 0;
    is_armed = false;
    Motor.set(M2, 1000);
	Motor.set(M3, 1000);
}

void plutoPilot()
{
    if(App.isArmSwitchOn())
    {
        Motor.set(M2, 1000);
	    Motor.set(M3, 1000);
        if(!is_armed)
        {
            is_armed = Control.arm();
            if (is_armed) {
				Flight.setRelativeAltholdHeight(200);
			}
        }
    }
    else
    {
        current_roll = Control.getRcData(RC_ROLL);
        current_pitch = Control.getRcData(RC_PITCH);
        if( abs(current_roll - 1500) < 100 )
        {
            current_roll = 1500;
            heading_error = set_heading - Flight.getAngle(AG_YAW);
        }
        else
        {
            heading_error = 0;
            set_heading = Flight.getAngle(AG_YAW);
        }
        if( abs(current_pitch - 1500) < 100)
            current_pitch = 1500;

        if(heading_error > 180)
            heading_error -= 360;
        if(heading_error < -180)
            heading_error += 360;

        current_roll -= 10*heading_error;
        m2 = 4*current_pitch + current_roll - 4500;
        m3 = 4*current_pitch - current_roll - 7500;
        m2 = (int)((float)m2*10.0/43.0); //  -4300, 4300, -1000, 1000
        m3 = (int)((float)m2*10.0/43.0);

        if (m2 < 0)
            Motor.setDirection(M3, BACKWARD);
        else
            Motor.setDirection(M3, FORWARD);

        if (m3 < 0)
            Motor.setDirection(M2, FORWARD);
        else
            Motor.setDirection(M2, BACKWARD);

        m2_pwm = 1000 + abs(m2);
        m3_pwm = 1000 + abs(m3);

//        Print.monitor("\nheading_error=0");
        Print.monitor("\ncurrent_heading=",Flight.getAngle(AG_YAW));
//        Print.monitor("\nX=",Magnetometer.getX());
//        Print.monitor("\nY=",Magnetometer.getY());
//        Print.monitor("\nZ=",Magnetometer.getZ());


        Motor.set(M2, m2_pwm);
        Motor.set(M3, m3_pwm);
    }
}

void onPilotFinish()
{
    current_roll = 0;
    current_heading = 0;
    heading_error = 0;
    current_pitch = 0;
    set_heading = 0;
    is_armed = false;

    Motor.set(M2, 1000);
	Motor.set(M3, 1000);
}
