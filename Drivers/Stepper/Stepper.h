/*
 * Stepper.h
 *
 *  Created on: 22.02.2019
 *      Author: Nils
 */

#ifndef STEPPER_STEPPER_H_
#define STEPPER_STEPPER_H_

#include "gpio.h"

const uint8_t buffersize = 15;
const uint8_t splines = 5;

class Stepper{
public:

	uint16_t dir;
	uint16_t step;
	uint16_t enable;

	int32_t position = 0;
	int32_t pos_max;
	int32_t pos_min;


	uint32_t nextsteptime = 0;

	uint64_t time = 0;
	uint16_t time2 = 0;
	uint16_t time3 = 0;


	double v_ist=0;

	double max_vel;
	double max_accel;

	bool enabled = 0;

	uint32_t t_diff;
	GPIO_TypeDef* GPIOx;
	GPIO_TypeDef* GPIOy;


	double lastvel = 0;
	double lastpos = 0;


	int8_t actualbuffer = 0;

	int16_t steps = 0;
	int16_t runinterval[buffersize] = {0};
	int16_t runsteps[buffersize] = {0};
	uint8_t rundir[buffersize];
	int32_t pospoint[buffersize] = {0};
	uint64_t timestamp[buffersize] = {0, 0};
	double posvel[buffersize] = { 0 };
	uint8_t actualpospoint = 1;
	uint8_t lastcalculated = 1;
	bool newset = 0;


	/*Spline Variables*/
	uint8_t actual_spline = 0;
	bool spline_enable[splines] = { 0, 0, 0, 0, 0};
	int32_t s_end[splines] = {0};
	double v_end[splines] = {0};
	uint32_t t_start[splines] = { 0 };
	uint32_t t_end[splines] = { 0 };
	double b0[splines], b1[splines], b2[splines], b3[splines], b4[splines], b5[splines];

	bool running = 0;

	bool inverse = 0;


	Stepper(uint16_t dir, uint16_t step, GPIO_TypeDef* GPIOx, uint16_t enable,
			GPIO_TypeDef* GPIOy, double max_speed, double max_accel, double max_jerk
			,bool inverse, int32_t min_pos, int32_t max_pos);

	//void setvelocity(double vel);

	void run();

	void setdir();

	void setdir2(uint8_t dir);	//0 pos , 1 neg

	void toggleEnable();

	void moveLinear();

	void setvelocity(double ve);

	void setpositionvmax(double se, double ve);
	void setposition(double se, double vmax);
	void setpositionte(double se, uint32_t time);
	void setpositionve(double se, double ve);
	void setpositionvete(double se, double ve, uint32_t time);
	void setspline(double s0, double se, double v0, double ve, uint32_t time_start, uint32_t time_end, uint8_t mode, uint8_t spline);

	void calculatepos(uint32_t time);

	uint32_t checktime(double posstart, double posend);

	void update();
};



#endif /* STEPPER_STEPPER_H_ */
