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
void pause1();
void start();
void pause2();
void stop();

#ifdef __cplusplus
}
#endif

#endif /* SRC_APP_H_ */
