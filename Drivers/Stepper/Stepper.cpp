/*
 * Stepper.cpp
 *
 *  Created on: 22.02.2019
 *      Author: Nils
 */
#include "Stepper.h"
#include "math.h"
#include "System.h"

#include "tim.h"


#include "App.h"

//extern uint32_t micros;
//TIM_HandleTypeDef htim1;

uint16_t counter = 0;

double sgn(double in)
{
	if(in>=0)
		return 1;
	else
		return -1;
}

Stepper::Stepper(uint16_t dir, uint16_t step, GPIO_TypeDef* GPIOx, uint16_t enable,
		GPIO_TypeDef* GPIOy, double max_speed, double max_accel, double max_jerk
		,bool inverse, int32_t min_pos, int32_t max_pos)
{
	this->dir = dir;
	this->step = step;
	this->enable = enable;
	this->GPIOx =GPIOx;
	this->GPIOy =GPIOy;
	this->max_vel = max_speed;
	this->max_accel = max_accel;
	this->inverse = inverse;

	pos_max = max_pos;
	pos_min = min_pos;

	HAL_GPIO_WritePin(GPIOy, enable, GPIO_PIN_SET);

	t_diff =sqrt(2/max_accel);
}

void Stepper::toggleEnable()
{
	if(enabled) HAL_GPIO_WritePin(GPIOy, enable, GPIO_PIN_SET), enabled=0, System::print("Disabled\n");
	else HAL_GPIO_WritePin(GPIOy, enable, GPIO_PIN_RESET), enabled=1, System::print("Enabled\n");
}

void Stepper::moveLinear()
{
	if(running){

		if(time2 == 1000){
			time2 = 0;
			time3 = 0;
			actualpospoint = (actualpospoint+1) %buffersize;
			steps = 0;
		}

		time3++;

		if(time3 == runinterval[actualpospoint] && steps<runsteps[actualpospoint])
		{
			if(rundir[actualpospoint] == 1){
				setdir2(1);
				position++;
			}
			if(rundir[actualpospoint] == 2){
				setdir2(2);
				position--;
			}
			HAL_GPIO_WritePin(GPIOx, step, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOx, step, GPIO_PIN_RESET);
			time3 = 0;
			steps++;
		}

		time2++;

		int16_t buffer = (int)lastcalculated - actualpospoint;
		if(buffer < 0) buffer +=buffersize;

		if(buffer < 3){
			running = 0;
		}
	}
}



void Stepper::setdir(){
	if(inverse){
		if(v_ist>0)HAL_GPIO_WritePin(GPIOx, dir, GPIO_PIN_SET);
		else HAL_GPIO_WritePin(GPIOx, dir, GPIO_PIN_RESET);

	}
	else
	{
		if(v_ist>0)HAL_GPIO_WritePin(GPIOx, dir, GPIO_PIN_RESET);
		else HAL_GPIO_WritePin(GPIOx, dir, GPIO_PIN_SET);
	}
}

void Stepper::setdir2(uint8_t direction){
	if(inverse){
		if(direction==1)HAL_GPIO_WritePin(GPIOx, dir, GPIO_PIN_SET);
		if(direction==2)HAL_GPIO_WritePin(GPIOx, dir, GPIO_PIN_RESET);
	}
	else{
		if(direction==1)HAL_GPIO_WritePin(GPIOx, dir, GPIO_PIN_RESET);
		if(direction==2)HAL_GPIO_WritePin(GPIOx, dir, GPIO_PIN_SET);
	}
}

void Stepper::setvelocity(double ve)
{

	if(abs(ve - posvel[lastcalculated])>0.1)
	{
		newset = 1;

		if(ve > max_vel) ve = max_vel;

		/*number of next and following*/
		uint8_t spline = (actual_spline+1)%splines;
		uint8_t spline_next = (actual_spline+2)%splines;

		double s0;
		double v0;
		double a0;
		double ae;

		double c0, c1, c2, c3, c4, c5;

		double d_s=0;

		/*calc maximal distance*/
		if(ve>0) d_s = pos_max - position;
		if(ve<0) d_s = pos_min - position;

		s0 = pospoint[lastcalculated];
		v0 = posvel[lastcalculated];
		a0 = 0;//posaccel[lastcalculated];
		ae = 0;

		/*distance to accelerate and decelerate*/
		double distance = 1000;
		distance = sgn(ve)*abs(pow(ve,2)/max_accel) - sgn(v0)*pow(v0,2)/(2*max_accel);

		System::print("Distance:%f, ds:%f \t v0: %f \t ve: %f\n", distance, d_s, v0, ve );

		if(ve==0 || abs(d_s) > abs(distance))
		{
			System::print("calc Vel\n");

			double buffer;
			uint32_t te;
			buffer = abs(ve-v0)/max_accel;
			te = (uint32_t)buffer;

			/*calc spline factors*/
			c0 = s0;
			c1 = v0;
			c2 = a0/2;
			c3 = -(ae+4*c2)/(3*te) + (ve-c1)/(pow(te,2));
			c4 = (2*c2+ae)/(4*pow(te,2)) + (c1-ve)/(2*pow(te,3));
			c5 = 0;

			/* endposition && endspeed */
			s_end[spline] = (int32_t)(c0 + c1*te + c2*pow(te,2) + c3*pow(te,3) +c4*pow(te,4) + c5*pow(te,5));
			v_end[spline] = ve;

			System::print("Checkdistance : %f, Realdis: %f, maxpos: %f\n", (double)distance, (double)s_end[spline], (double)pos_max);

			b0[spline] = c0;
			b1[spline] = c1;
			b2[spline] = c2;
			b3[spline] = c3;
			b4[spline] = c4;
			b5[spline] = c5;

			/* start && end time*/
			t_start[spline] = timestamp[lastcalculated];
			t_end[spline] = t_start[spline] + te;

//			if(ve == 0)stopindicator[spline] = 0;

			/*Constant Vel function*/
			if(ve != 0){
				t_start[spline_next] = t_end[spline];

				/*position to decelerate*/
				buffer = 0;
				if(ve>0){
					//pos max
					buffer = (pos_max - pow(ve,2)/max_accel);
					s_end[spline_next] = buffer;
				}
				if(ve<0){
					//pos min
					buffer = (pos_min + pow(ve,2)/max_accel);
					s_end[spline_next] = buffer;
				}
				/*time to decelerate*/
				t_end[spline_next] = t_start[spline_next] + abs(s_end[spline_next] - s_end[spline])/abs(ve);// - abs(ve/max_accel);

				System::print("t_end vconst: %f \n" ,(double)t_end[spline_next]);

				v_end[spline_next] = ve;

				b0[spline_next] = s_end[spline];
				b1[spline_next] = v_end[spline];
				b2[spline_next] = 0;
				b3[spline_next] = 0;
				b4[spline_next] = 0;
				b5[spline_next] = 0;

				spline_enable[spline_next] = 1;
			}

			spline_enable[spline] = 1;
		}
		else{	/*if distance is to short just move to border position*/
			if(ve>0) setposition(pos_max, ve);
			if(ve<0) setposition(pos_min, ve);
			if(ve==0){

				double buffer = pospoint[lastcalculated] + v0/max_accel;
				if(buffer > pos_max)buffer = pos_max;
				if(buffer < pos_min)buffer = pos_min;

				setposition((int32_t) buffer, max_vel);
			}
		}
	}

}

void Stepper::setposition(double se, double vmax)
{
	if(abs(se - position)>10){
		newset = 1;

		uint8_t spline = (actual_spline+1)%splines;

		double s0;
		double v0;
		double a0;
		double ve = 0;
		double ae = 0;

		double c0, c1, c2, c3, c4, c5;

		s0 = pospoint[lastcalculated];
		v0 = posvel[lastcalculated];
		a0 = 0; //posaccel[lastcalculated];

		System::print("calc Pos  s0: %f  se: %f\n", s0, se);

		double s_accel = pow(vmax, 2)/max_accel;

		int32_t d_s = se - s0;
		//d_s -= pow(v0,2)/(2*max_accel);
		if(d_s > 0) d_s -= sgn(v0)*pow(v0,2)/(2*max_accel);
		if(d_s < 0) d_s -= sgn(v0)*pow(v0,2)/(2*max_accel);

		int64_t te;
		double buffer = 0;
		//TODO

		if(abs(d_s)< s_accel){
		/* distance only accaleration __/\__*/
			buffer = sqrt(abs(d_s)/max_accel)*1.85;
		}
		else{
		/* distance accaleration and "const" velocity __*/
		/*										   __/	\__*/
			buffer = vmax/max_accel*1.85 + (abs(d_s)-s_accel)/vmax*1.85;
		}
		System::print("singend te: %f\n", buffer);
		te = (uint32_t)abs(buffer);
		if(te==0) te =3;

		c0 = s0;
		c1 = v0;
		c2 = a0/2;
		c3 = (ae-3*a0)/(2*te) - (4*ve+6*v0)/(pow(te,2)) + 10*(se-s0)/(pow(te,3));
		c4 = (3*a0-2*ae)/(pow(te,2)) + (8*v0+7*ve)/(pow(te,3)) + 15*(s0-se)/(pow(te,4));
		c5 = (ae-a0)/(2*pow(te,3)) - 3*(c1+ve)/(pow(te,4)) + 6*(se-c0)/(pow(te,5));

		s_end[spline] = se;
		v_end[spline] = ve;

		t_start[spline] = timestamp[lastcalculated];
		t_end[spline] = t_start[spline] + te;

		b0[spline] = c0;
		b1[spline] = c1;
		b2[spline] = c2;
		b3[spline] = c3;
		b4[spline] = c4;
		b5[spline] = c5;

		spline_enable[spline] = 1;

		System::print("Grad 5 Spline Variables\n");
		System::print("s0: %f \t v0: %f a0: %f\n", s0, v0, a0);
		System::print("b0: %f \t b1: %f \t b2: %f \t b3: %f \t b4: %f \t b5: %f \n", c0, c1, c2, c3, c4, c5);
		System::print("t_start: %f \t t_end: %f \t s_end: %f \n", (double)t_start[spline], (double)t_end[spline], (double)s_end[spline]);
	}
}

void Stepper::calculatepos(uint32_t time_p)
{
	//uint32_t end, diff;
	//uint32_t start = micros;

	double t1, t2, t3, t4, t5;

	uint8_t posno = (lastcalculated+1)%buffersize;
	t1 = (time_p - t_start[actual_spline]);

	t2=pow(t1,2);
	t3=pow(t1,3);
	t4=pow(t1,4);
	t5=pow(t1,5);

	pospoint[posno] = b0[actual_spline] + b1[actual_spline]*t1 + b2[actual_spline]*t2
			+ b3[actual_spline]*t3 +b4[actual_spline]*t4 + b5[actual_spline]*t5;
	posvel[posno] = b1[actual_spline] + 2*b2[actual_spline]*t1 + 3*b3[actual_spline]*t2
			+4*b4[actual_spline]*t3 + 5*b5[actual_spline]*t4;
	posaccel[posno] = 2*b2[actual_spline] + 6*b3[actual_spline]*t1 +12*b4[actual_spline]*t2 + 20*b5[actual_spline]*t3;


	int32_t buffer = pospoint[posno] - pospoint[(posno+buffersize-1)%buffersize];
	if(buffer > 0) rundir[posno] = 1;
	if(buffer < 0 ) rundir[posno] = 2, buffer = -buffer;
	if(buffer == 0) runsteps[posno] = 0, rundir[posno] = 0;
	else{
		runsteps[posno] = buffer;
		runinterval[posno] = ((double)1000/buffer);
	}

	timestamp[posno] = time_p;
	lastcalculated = posno;

	//System::print("lst %d \t pos: %.0f \t /_|: %d \t t: %0.f\t", posno, (double)pospoint[posno], runsteps[posno], (double)time_p);
	//System::print("buf: %d \t %d\n", buffer, position);

}

void Stepper::update()
{
	//counter ++;
	if(enabled && newset){

		uint32_t t;
		t = timestamp[lastcalculated]+1;

		int16_t buffer = (int)lastcalculated - actualpospoint;
		if(buffer < 0) buffer +=buffersize;

		//if(buffer < 8)
		while(buffer < 8 && newset)
		{
			uint8_t next_spline = (actual_spline+1)%splines;
			if(buffer > 6){
				//System::print("running, actualspline: %d\n", actual_spline);
				running = 1;
			}
			if((t >= t_start[next_spline]) && spline_enable[next_spline]){
				spline_enable[actual_spline] = 0;
				actual_spline = next_spline;

				System::print("nextspline: %d \n", actual_spline);

			}
			if(b1[actual_spline] && !b2[actual_spline] && !b3[actual_spline]
					&& !b4[actual_spline] && !b5[actual_spline] && t_end[actual_spline]-t < 1000){
				if(v_end[actual_spline] > 0)
					setposition(pos_max, v_end[actual_spline]);
				else
					setposition(pos_min, v_end[actual_spline]);
			}
			if(t >= t_end[actual_spline]){
				newset = 0;
			}

			//System::print("buf: %d \t", buffer);

			calculatepos(t);
			t++;
			buffer = (int)lastcalculated - actualpospoint;
			if(buffer < 0) buffer +=buffersize;
		}
	}
}
