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

double sign(double in)
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

		actualbuffer = buffer;

		if(buffer < 3){
			System::print("Stop\n");
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
	if(abs(ve - posvel[lastcalculated])>1 && lastvel != ve)
	{
		lastvel = ve;
		if(ve>0)lastpos = pos_max;
		if(ve<0) lastpos = pos_min;


		newset = 1;

		if(ve > max_vel) ve = max_vel;
		if(ve < -max_vel) ve = -max_vel;
		/*number of next and following*/
		uint8_t spline = (actual_spline+1)%splines;
		uint8_t spline_next1 = (actual_spline+2)%splines;
		uint8_t spline_next2 = (actual_spline+3)%splines;

		double s0, v0, a0, ae;
		double c0, c1, c2, c3, c4, c5;


		double d_s;

		s0 = pospoint[lastcalculated];
		v0 = posvel[lastcalculated];
		a0 = 0;
		ae = 0;

		if(isnan(v0)) v0 = 0;

		double t1 = (uint32_t)abs((ve - v0)/max_accel*1.4);
		double t2 = (uint32_t)abs((ve)/max_accel*1.4);

		double d_s1 = (int32_t)(t1*(v0 + (ve - v0)/2));
		double d_s2 = (int32_t)(t2*ve/2);

		d_s = s0 + d_s1 + d_s2;


		/*distance to accelerate and decelerate*/

		System::print("Endposmin :%f, \t v0: %f \t ve: %f\n", d_s, v0, ve );

		bool const_vel = 0;
		if(ve == 0)const_vel = 1;
		if(ve > 0 && d_s < pos_max)const_vel = 1;
		if(ve < 0 && d_s > pos_min)const_vel = 1;

		if(const_vel)
		{
			//System::print("calc Vel\n");

			uint32_t te;
			te = t1;

			double se = s0 + d_s1;

		    c0 = s0;
		    c1 = v0;
		    c2 = a0/2;
		    c3 = (ae-3*a0)/(2*te) - (4*ve+6*v0)/(pow(te,2)) + 10*(se-s0)/(pow(te,3));
		    c4 = (3*a0-2*ae)/(pow(te,2)) + (8*v0+7*ve)/(pow(te,3)) + 15*(s0-se)/(pow(te,4));
		    c5 = (ae-a0)/(2*pow(te,3)) - 3*(c1+ve)/(pow(te,4)) + 6*(se-c0)/(pow(te,5));

			/* endposition && endspeed */
			s_end[spline] = (int32_t)(c0 + c1*te + c2*pow(te,2) + c3*pow(te,3) +c4*pow(te,4) + c5*pow(te,5));
			v_end[spline] = ve;


			b0[spline] = c0;
			b1[spline] = c1;
			b2[spline] = c2;
			b3[spline] = c3;
			b4[spline] = c4;
			b5[spline] = c5;

			/* start && end time*/
			t_start[spline] = timestamp[lastcalculated];
			t_end[spline] = t_start[spline] + te;

			if(ve==0) lastpos = s_end[spline];

			System::print("1: t_start: %u t_end: %u s_0: %f s_e: %f\n", t_start[spline], t_end[spline], s0, se);

			/*Constant Vel function*/
			if(ve != 0){
				t_start[spline_next1] = t_end[spline];

				/*position to decelerate*/
				if(ve>0){
					s_end[spline_next1] = pos_max-abs(d_s2);
				}
				if(ve<0){
					s_end[spline_next1] = pos_min+abs(d_s2);
				}
				/*time to decelerate*/
				double diff = (s_end[spline] - s_end[spline_next1]);
				t_end[spline_next1] = t_start[spline_next1]+(uint32_t)abs(diff/ve);

				//System::print("t_end vconst: %f \n" ,(double)t_end[spline_next1]);

				v_end[spline_next1] = ve;

				b0[spline_next1] = s_end[spline];
				b1[spline_next1] = ve;
				b2[spline_next1] = 0;
				b3[spline_next1] = 0;
				b4[spline_next1] = 0;
				b5[spline_next1] = 0;


				t_start[spline_next2] = t_end[spline_next1];
				t_end[spline_next2] = t_start[spline_next2] + t2;

				System::print("2: t_start: %u t_end: %d s_0: %u s_e: %d\n", t_start[spline_next1], t_end[spline_next1], s_end[spline], s_end[spline_next1]);

				v0=ve;
				s0=s_end[spline_next1];

				if(ve>0){
					s_end[spline_next2] = pos_max;
				}
				if(ve<0){
					s_end[spline_next2] = pos_min;
				}
				ve=0;
				v_end[spline_next2] = 0;

				se=s_end[spline_next2];

				te = t2;

			    c0 = s_end[spline_next1];
			    c1 = v_end[spline_next1];
			    c2 = a0/2;
			    c3 = (ae-3*a0)/(2*te) - (4*ve+6*v0)/(pow(te,2)) + 10*(se-s0)/(pow(te,3));
			    c4 = (3*a0-2*ae)/(pow(te,2)) + (8*v0+7*ve)/(pow(te,3)) + 15*(s0-se)/(pow(te,4));
			    c5 = (ae-a0)/(2*pow(te,3)) - 3*(c1+ve)/(pow(te,4)) + 6*(se-c0)/(pow(te,5));

				b0[spline_next2] = c0;
				b1[spline_next2] = c1;
				b2[spline_next2] = c2;
				b3[spline_next2] = c3;
				b4[spline_next2] = c4;
				b5[spline_next2] = c5;

				System::print("3: t_start: %u t_end: %u s_0: %f s_e: %f\n", t_start[spline_next2], t_end[spline_next2], s0, se);


				spline_enable[spline_next1] = 1;
				spline_enable[spline_next2] = 1;
			}
			else{
				spline_enable[spline_next1] = 0;
				spline_enable[spline_next2] = 0;
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
	if((abs(se - pospoint[lastcalculated])>10 || (abs(posvel[lastcalculated])> 0.5))&& se != lastpos){

		if(se > pos_max)se= pos_max;
		if(se < pos_min)se= pos_min;
		lastpos = se;
		lastvel = 0;

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

		if(isnan(v0)) v0 = 0;

		System::print("calc Pos  s0: %f  se: %f\n", s0, se);

		double s_add = sign(v0)*pow(v0,2)/max_accel;

		int32_t d_s = se - s0;

		int32_t d_s2 = se - (s0 + (int32_t)s_add);


		double t_base = abs(v0)/max_accel*1.4;

		double multi = 2;

		int64_t te;
		double buffer = 0;

	    if(sign(d_s) == sign(d_s2)){
	        double t = 2*(vmax-abs(v0))/max_accel;
	        double s = t*(v0 + sign(v0)*((vmax-abs(v0))/2));
	        if(abs(s)>abs(d_s2)){
	            t=t*pow((s)/(d_s2),2);
	        }
	        else{
	            t = t+ (abs(d_s2) - abs(s))/vmax;
	        }
	        buffer = (t + t_base*multi);
	    }
	    else{
	        buffer = sqrt(abs(d_s2)/max_accel)*multi + t_base;
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

//		System::print("Grad 5 Spline Variables\n");
//		System::print("s0: %f \t v0: %f a0: %f\n", s0, v0, a0);
//		System::print("b0: %f \t b1: %f \t b2: %f \t b3: %f \t b4: %f \t b5: %f \n", c0, c1, c2, c3, c4, c5);
//		System::print("t_start: %f \t t_end: %f \t s_end: %f \n", (double)t_start[spline], (double)t_end[spline], (double)s_end[spline]);
	}
}

void Stepper::setpositionte(double se, uint32_t time)
{

}

void Stepper::setpositionve(double se, double ve, uint32_t time)
{

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
	//posaccel[posno] = 2*b2[actual_spline] + 6*b3[actual_spline]*t1 +12*b4[actual_spline]*t2 + 20*b5[actual_spline]*t3;


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

//			if(b1[actual_spline] && !b2[actual_spline] && !b3[actual_spline]
//					&& !b4[actual_spline] && !b5[actual_spline] && t_end[actual_spline]-t < 4){
//				if(v_end[actual_spline] > 0)
//					setposition(pos_max, v_end[actual_spline]);
//				else
//					setposition(pos_min, v_end[actual_spline]);
//			}
			if(t >= t_end[actual_spline] && !spline_enable[next_spline]){
				newset = 0;
				System::print("Stop-calculating : t=%u\n", t);
			}
			if((t >= t_start[next_spline]) && spline_enable[next_spline]){
				spline_enable[actual_spline] = 0;
				actual_spline = next_spline;

				System::print("nextspline: %d \n", actual_spline);

			}

			//System::print("buf: %d \t", buffer);

			calculatepos(t);
			t++;
			buffer = (int)lastcalculated - actualpospoint;
			if(buffer < 0) buffer +=buffersize;
		}
	}
}
