#ifndef SRC_APP_H_
#define SRC_APP_H_

#include <stdint.h>
#include <usart.h>


#ifdef __cplusplus
extern "C" {
#endif

void InterruptPIN(uint16_t GPIO_PIN);

void Timer_IT1();

void App_Start();

void workdata();

void setsyncronmove();

uint8_t getandcheckdata();

uint32_t calcmintime();

void testrun();
void cycle();
void start();
void stop();

void setspline(uint8_t Motor, int32_t posa, int32_t posb,
		double vela, double velb, uint32_t timea, uint32_t timeb,
		uint8_t mode, uint8_t spline);

#ifdef __cplusplus
}
#endif

#endif /* SRC_APP_H_ */
