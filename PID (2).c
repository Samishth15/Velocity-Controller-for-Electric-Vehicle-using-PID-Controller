#include "PID.h"

void PIDController_Init(PIDController *pid) {

	/* Clear controller variables */
	pid->integrator = 0.0f;
	pid->prevError  = 0.0f;

	pid->prevMeasurement = 0.0f;

	pid->out = 0.0f;
}

float PIDController_Update(PIDController *pid, float setpoint, float measurement) {

	/*
	* Error signal
	*/
   float error = setpoint - measurement; 
	
	if(error>500)  //If you don't want large torque current in-flow or jerks
	{
		error = 500;
	}

	/*
	* Proportional
	*/
    float proportional = pid->Kp * error;  //P = Kp*Error


	/*
	* Integral
	*/
    pid->integrator = pid->integrator + 0.5f * pid->Ki * pid->T * (error + pid->prevError); // previous val + Ki * error -> T

	/* Anti-wind-up via integrator clamping */
    if (pid->integrator > pid->limMaxInt) {

        pid->integrator = pid->limMaxInt;   //If steady state val increases more than capacity, enter highest val

    } else if (pid->integrator < pid->limMinInt) {

        pid->integrator = pid->limMinInt;   //If steady is lesser than lowest capacity

    }

	/*


	/*
	* Compute output and apply limits
	*/
    pid->out = proportional + pid->integrator; //kd = 0
		//dac input = pid.out
    if (pid->out > pid->limMax) {

        pid->out = pid->limMax;

    } else if (pid->out < pid->limMin) {

        pid->out = pid->limMin;

    }

	/* Store error and measurement for later use */
    pid->prevError       = error;
    pid->prevMeasurement = measurement;

	/* Return controller output */
    return pid->out;

}
