#include "App.h"
#include "stdio.h"
//#include "array"

#include "System.h"
#include "gpio.h"
#include "usart.h"

#include "math.h"
#include "tim.h"

#include "Stepper.h"

#include "spi.h"

#include "nRF905.h"
#include "nRF905_config.h"
#include "nRF905_defs.h"

#define RXADDR 0xFE1337AD
#define TXADDR 0xFE1337AC // Address of device to send to


TIM_HandleTypeDef htim1;


Stepper M1(DIR1_Pin, STEP1_Pin, STEP1_GPIO_Port, M_EN_Pin, M_EN_GPIO_Port, 40, 0.5, 0.0001, 0, (int32_t)-130/0.01125, (int32_t)130/0.01125);
Stepper M2(DIR2_Pin, STEP2_Pin, STEP2_GPIO_Port, M_EN_Pin, M_EN_GPIO_Port, 40, 0.5, 0.0001, 0, (int32_t)-92/0.01125, (int32_t)92/0.01125);
Stepper M3(DIR3_Pin, STEP3_Pin, STEP3_GPIO_Port, M_EN_Pin, M_EN_GPIO_Port, 50, 0.4, 0.0001, 0, (int32_t)0, 50000);

int32_t posbuffer[3];
uint32_t timebuffer[3];

uint8_t worked = 0;

uint16_t sendpos = 200;
uint16_t sendposcounter = 0;

uint8_t rxdata[10];
uint8_t txdata[10];

uint8_t gotdata = 0;

uint16_t I2C_counter = 0;


uint8_t runmode = 0;
bool setmode = 0;
bool tobuffer[3] = { 0 };

const uint8_t axis = 3;
const uint8_t linepoints = 11;
int32_t positions[axis][linepoints];
uint8_t pos_number[axis][linepoints];
uint8_t pos_mode[axis][linepoints];
uint8_t points[axis] = { 0, 0 , 0 };
double posvel[axis][linepoints] = { 0 };
uint32_t minimaltime = 0;
uint32_t sectiontime = 400;
uint8_t lastspline[3] = { 0, 0, 0};
uint32_t timeZero[3];
uint8_t iterations[3];
uint8_t basespline[3];

//TODO Add posbuffer and timeout

void Timer_IT1()
{
	M1.moveLinear();
	M2.moveLinear();
	M3.moveLinear();
}


void App_Start()
{
	System::print("Stepper Driver \n");

	HAL_TIM_Base_Start_IT(&htim1);

    HAL_SPI_Init(&hspi2);


    M1.toggleEnable();
    M2.toggleEnable();
    M3.toggleEnable();

    M1.toggleEnable();
    M2.toggleEnable();
    M3.toggleEnable();

    M3.position = 25000;
    M3.pospoint[M3.lastcalculated] = 25000;
    M3.lastpos = 25000;

    HAL_Delay(200);

	nRF905_init();

	// Set address of this device
	nRF905_setListenAddress(RXADDR);

	// Put into receive mode
	nRF905_RX();

	HAL_Delay(1);

	System::print("Button %u\n", RXADDR);

	uint8_t regs[NRF905_REGISTER_COUNT];
	nRF905_getConfigRegisters(regs);

	uint16_t channel = ((uint16_t)(regs[1] & 0x01)<<8) | regs[0];
	uint32_t freq = (422400UL + (channel * 100UL)) * (1 + ((regs[1] & ~NRF905_MASK_BAND) >> 1));

	System::print("Channel: %u", channel);
	System::print("Freq: %u", freq);
	System::print("KHz");
	System::print("\nAuto retransmit: %u\n", !!(regs[1] & ~NRF905_MASK_AUTO_RETRAN));
	System::print("Low power RX: %u\n", !!(regs[1] & ~NRF905_MASK_LOW_RX));

	System::print("");


    HAL_Delay(200);

    //M1.setvelocity(M1.max_vel);
    //M2.setvelocity(0.06);
    //M1.setposition(M1.pos_max, M1.max_vel);

    while(1){

    	if(dataReady() && (!M1.running || M1.actualbuffer>9) && (!M2.running || M2.actualbuffer>9) &&
    			(!M3.running || M3.actualbuffer>9)){

    		//System::print("Buffer: %d   %d    %d\n", M1.actualbuffer, M2.actualbuffer, M3.actualbuffer);
    		//System::print("data Incoming\n");
    		if(getandcheckdata()){
    			workdata();
    		}
    	}

		if(runmode == 1) testrun();
		if(runmode == 2) cycle();
		if(runmode == 3) pause1();
		if(runmode == 4) start();
		if(runmode == 5) pause2();
		if(runmode == 6) stop();

    	M1.update();
    	M2.update();
    	M3.update();


    	HAL_Delay(1);

    }

}

void workdata()
{
	uint8_t motor = rxdata[0]>>4;
	uint8_t mode = rxdata[0] & 0x0F;


	/*Set Velocity*/
	if(mode == 1){
		if(M1.enabled){
			int8_t vel = rxdata[1];
//			System::print("Set Velocity %d\n", vel);
			double vel1 = vel;
			if(vel1!=0)vel1 = vel1/100;
			if(motor == 0) M1.setvelocity(vel1*M1.max_vel);
			if(motor == 1) M2.setvelocity(vel1*M2.max_vel);
			if(motor == 2) M3.setvelocity(vel1*M3.max_vel);
		}
	}

	/*Set Position*/
	if(mode == 2){
		if(M1.enabled){
			int32_t position = 0;
			int8_t vel = rxdata[5];
			position = rxdata[1]<<24 | rxdata[2]<<16 | rxdata[3]<<8 | rxdata[4];
			System::print("Set Position: %d Motor: %u\n", position, motor);
			if(rxdata[6] == 0){
				if(motor == 0) M1.setpositionvmax(position, (double)vel/100*M1.max_vel);
				if(motor == 1) M2.setpositionvmax(position, (double)vel/100*M2.max_vel);
				if(motor == 2) M3.setpositionvmax(position, (double)vel/100*M3.max_vel);
			}
			if(rxdata[6] == 1)
			{
				posbuffer[motor] = position;
				if(motor == 2) setsyncronmove();
			}
		}
	}

	/*Set Curve Parameter*/
	if(mode == 3){

	}

	/*Set Time*/
	if(mode == 4){

	}

	/*Set PictureNumber*/
	if(mode == 5){

	}

	/*Set Curve Movement (START, STOP, PAUSE, CYCLE, TESTRUN)*/
	if(mode == 6){
		runmode = rxdata[1];
		setmode = 1;
	}

	/*Receive Linepoints*/
	if(mode == 7){
		//TODO		(optional: count points and request again if point is missing)
		uint8_t pointnumber = rxdata[1];
		pos_number[motor][pointnumber] = rxdata[2];
		pos_mode[motor][pointnumber] = rxdata[3];
		positions[motor][pointnumber] = rxdata[4]<<24 | rxdata[5]<<16 | rxdata[6]<<8 | rxdata[7];
		points[motor] = rxdata[8];

		System::print("M: %u  Posnr: %u  Pos: %d  Mode: %u\n", motor, pos_number[motor][pointnumber], positions[motor][pointnumber], pos_mode[motor][pointnumber]);

		if(motor == 2 && rxdata[1] == rxdata[8] -1){
			calcmintime();
		}
	}

	/*Toggle Motor Enable*/
	if(mode == 9){
		if(motor == 9){
			System::print("Motors ON\n");
			M1.toggleEnable();
			M2.toggleEnable();
			M3.toggleEnable();
		}
	}

	/*Retrun actual Position*/
	if(mode == 10){
//		System::print("Retrun Position\n");
		for(uint8_t i=0; i<10; i++){
			txdata[i] = 0;
		}

		if(M1.enabled) txdata[6] = 1;
		else txdata[6] = 2;

		txdata[0] = 10;
		if(motor == 0){
			txdata[1] = 0;
			txdata[2] = M1.position >> 24;
			txdata[3] = M1.position >> 16;
			txdata[4] = M1.position >> 8;
			txdata[5] = M1.position;
		}
		if(motor == 1){
			txdata[1] = 1;
			txdata[2] = M2.position >> 24;
			txdata[3] = M2.position >> 16;
			txdata[4] = M2.position >> 8;
			txdata[5] = M2.position;
		}
		if(motor == 2){
			txdata[1] = 2;
			txdata[2] = M3.position >> 24;
			txdata[3] = M3.position >> 16;
			txdata[4] = M3.position >> 8;
			txdata[5] = M3.position;
		}


		for(uint8_t i=0; i<9; i++){
			txdata[9] += txdata[i];
		}

		nRF905_TX(TXADDR, txdata, 10, NRF905_NEXTMODE_RX);
	}

	if(mode != 10){
		for(uint8_t i=0; i<10; i++){
			txdata[i] = 0;
		}
		txdata[0] = 200;
		txdata[8] = 0xAA;
		txdata[9] = txdata[0] + txdata[8];

		nRF905_TX(TXADDR, txdata, 10, NRF905_NEXTMODE_RX);
	}

}


uint8_t getandcheckdata()
{
	nRF905_read(rxdata, 10);

	uint8_t dataOK = 0;

	uint8_t checksum = 0;
	for(uint8_t i=0; i<9; i++){
		checksum += rxdata[i];
	}

//	System::print("Gotdata\n");
	if(checksum == rxdata[9]){
		dataOK = 1;
//		System::print("DataOK\n");
	}

	return dataOK;
}


void setsyncronmove()
{
	timebuffer[0] = M1.checktime(M1.pospoint[M1.lastcalculated],posbuffer[0]);
	timebuffer[1] = M2.checktime(M2.pospoint[M2.lastcalculated],posbuffer[1]);
	timebuffer[2] = M3.checktime(M3.pospoint[M3.lastcalculated],posbuffer[2]);

//	System::print("t1: %u t2: %u t3: %u \n", timebuffer[0], timebuffer[1], timebuffer[2]);

	uint8_t slowest;
	if(timebuffer[0]>timebuffer[1])slowest = 0;
	else slowest = 1;
	if(timebuffer[slowest]<timebuffer[2])slowest = 2;

	M1.update();
	M2.update();
	M3.update();

	if(slowest == 0){
		M1.setpositionvmax(posbuffer[0], M1.max_vel);
		M2.setpositionte(posbuffer[1], timebuffer[slowest]);
		M3.setpositionte(posbuffer[2], timebuffer[slowest]);
	}
	if(slowest == 1){
		M1.setpositionte(posbuffer[0], timebuffer[slowest]);
		M2.setpositionvmax(posbuffer[1], M2.max_vel);
		M3.setpositionte(posbuffer[2], timebuffer[slowest]);
	}
	if(slowest == 2){
		M1.setpositionte(posbuffer[0], timebuffer[slowest]);
		M2.setpositionte(posbuffer[1], timebuffer[slowest]);
		M3.setpositionvmax(posbuffer[2], M3.max_vel);
	}
}

uint32_t calcmintime()
{
	uint32_t mintime = 0;
	uint32_t calctime = 0;

	uint8_t i,j;
	for(i = 0; i<3; i++){
		//uint8_t pointno = 0;
		for(j = 0; j < 11; j++){
			if(pos_mode[i][j] == 1)
			{
				posvel[i][j] = positions[i][j]-positions[i][j-1];
				//System::print("posvel: %f \n", posvel[i][j+1]);
				posvel[i][j] = posvel[i][j]/(pos_number[i][j]-pos_number[i][j-1]);
				//System::print("posvel: %f \n", posvel[i][j+1]);
				posvel[i][j-1] = posvel[i][j];
				//System::print("posvel: %f \n", posvel[i][j]);
			}
			else{
				posvel[i][j] = 0;
			}
		}
		for(j = 0; j <points[i]-1; j++)
		{
			if(i == 0){
				calctime = M1.checktime(positions[i][j],positions[i][j+1]);
				calctime = calctime/(pos_number[i][j+1]-pos_number[i][j]);
			}
			if(i == 1){
				calctime = M2.checktime(positions[i][j],positions[i][j+1]);
				calctime = calctime/(pos_number[i][j+1]-pos_number[i][j]);
			}
			if(i == 2){
				calctime = M3.checktime(positions[i][j],positions[i][j+1]);
				calctime = calctime/(pos_number[i][j+1]-pos_number[i][j]);
			}
			if(calctime > mintime)mintime = calctime;
		}
	}

	minimaltime = mintime;
	System::print("Mintime: %u \n", mintime);

	return mintime;
}


void testrun()
{
	if(setmode)
	{
		if(!M1.running && !M2.running && !M3.running){
			/*Set position to Startposition*/
			if(abs(M1.position-positions[0][0]) > 4  || abs(M2.position-positions[1][0]) > 4 || abs(M3.position-positions[2][0]) > 4){
				M1.setpositionvmax(positions[0][0], M1.max_vel);
				M2.setpositionvmax(positions[1][0], M2.max_vel);
				M3.setpositionvmax(positions[2][0], M3.max_vel);
			}
			else{
				/*Motor 1*/
				uint8_t m = 0;
				uint8_t n = 0;
				lastspline[m] = M1.actual_spline;
				timeZero[m] = M1.timestamp[M1.lastcalculated];
				basespline[m] = M1.actual_spline;

				M1.setspline(positions[m][n], positions[m][n+1], posvel[m][n]/minimaltime, posvel[m][n+1]/minimaltime,
						timeZero[m]+pos_number[m][n]*minimaltime, timeZero[m]+pos_number[m][n+1]*minimaltime,
						pos_mode[m][n+1], (basespline[m]+n+1)%splines);

				//System::print("v0: %f, ve: %f\n", posvel[m][n]/minimaltime, posvel[m][n+1]/minimaltime);

				n=1;
				M1.setspline(positions[m][n], positions[m][n+1], posvel[m][n]/minimaltime, posvel[m][n+1]/minimaltime,
						timeZero[m]+pos_number[m][n]*minimaltime, timeZero[m]+pos_number[m][n+1]*minimaltime,
						pos_mode[m][n+1], (basespline[m]+n+1)%splines);

				//System::print("v0: %f, ve: %f\n", posvel[m][n]/minimaltime, posvel[m][n+1]/minimaltime);

				iterations[m] = 2;

				/*Motor 2*/
				m = 1;
				n = 0;
				lastspline[m] = M2.actual_spline;
				timeZero[m] = M2.timestamp[M2.lastcalculated];
				basespline[m] = M2.actual_spline;

				M2.setspline(positions[m][n], positions[m][n+1], posvel[m][n]/minimaltime, posvel[m][n+1]/minimaltime,
						timeZero[m]+pos_number[m][n]*minimaltime, timeZero[m]+pos_number[m][n+1]*minimaltime,
						pos_mode[m][n+1], (basespline[m]+n+1)%splines);

				n=1;
				M2.setspline(positions[m][n], positions[m][n+1], posvel[m][n]/minimaltime, posvel[m][n+1]/minimaltime,
						timeZero[m]+pos_number[m][n]*minimaltime, timeZero[m]+pos_number[m][n+1]*minimaltime,
						pos_mode[m][n+1], (basespline[m]+n+1)%splines);
				iterations[m] = 2;

				/*Motor 3*/
				m = 2;
				n = 0;
				lastspline[m] = M3.actual_spline;
				timeZero[m] = M3.timestamp[M3.lastcalculated];
				basespline[m] = M3.actual_spline;

				M3.setspline(positions[m][n], positions[m][n+1], posvel[m][n]/minimaltime, posvel[m][n+1]/minimaltime,
						timeZero[m]+pos_number[m][n]*minimaltime, timeZero[m]+pos_number[m][n+1]*minimaltime,
						pos_mode[m][n+1], (basespline[m]+n+1)%splines);

				n=1;
				M3.setspline(positions[m][n], positions[m][n+1], posvel[m][n]/minimaltime, posvel[m][n+1]/minimaltime,
						timeZero[m]+pos_number[m][n]*minimaltime, timeZero[m]+pos_number[m][n+1]*minimaltime,
						pos_mode[m][n+1], (basespline[m]+n+1)%splines);
				iterations[m] = 2;

				setmode = 0;
			}
		}
	}
	if(lastspline[0] != M1.actual_spline && iterations[0] < points[0]){

		uint8_t m = 0;
		uint8_t n = iterations[0];

		M1.setspline(positions[m][n], positions[m][n+1], posvel[m][n]/minimaltime, posvel[m][n+1]/minimaltime,
				timeZero[m]+pos_number[m][n]*minimaltime, timeZero[m]+pos_number[m][n+1]*minimaltime,
				pos_mode[m][n+1], (basespline[m]+n+1)%splines);

		//System::print("v0: %f, ve: %f\n", posvel[m][n]/minimaltime, posvel[m][n+1]/minimaltime);

		iterations[m]++;
		lastspline[m] = M1.actual_spline;
	}
	if(lastspline[1] != M2.actual_spline && iterations[1] < points[1]){
		uint8_t m = 1;
		uint8_t n = iterations[m];

		M2.setspline(positions[m][n], positions[m][n+1], posvel[m][n]/minimaltime, posvel[m][n+1]/minimaltime,
				timeZero[m]+pos_number[m][n]*minimaltime, timeZero[m]+pos_number[m][n+1]*minimaltime,
				pos_mode[m][n+1], (basespline[m]+n+1)%splines);

		iterations[m]++;
		lastspline[m] = M2.actual_spline;
	}
	if(lastspline[2] != M3.actual_spline && iterations[2] < points[2]){
		uint8_t m = 2;
		uint8_t n = iterations[m];

		M3.setspline(positions[m][n], positions[m][n+1], posvel[m][n]/minimaltime, posvel[m][n+1]/minimaltime,
				timeZero[m]+pos_number[m][n]*minimaltime, timeZero[m]+pos_number[m][n+1]*minimaltime,
				pos_mode[m][n+1], (basespline[m]+n+1)%splines);

		iterations[m]++;
		lastspline[m] = M3.actual_spline;
	}
}

void cycle()
{

}
void pause1()
{

}
void start()
{

}
void pause2()
{

}
void stop()
{

}
