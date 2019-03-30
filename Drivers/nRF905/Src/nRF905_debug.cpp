/*
 * nRF905_debug.cpp
 *
 *  Created on: 20.06.2018
 *      Author: Nils
 */
//#include "stm32f0xx_hal.h"
#include "main.h"
#include "System.h"

#include "nRF905.h"
#include "nRF905_defs.h"

#define RXADDR 0xFE4CA6E5 // Address of this device

#define RXADDR1 0x586F2E10 // Address of this device
#define TXADDR1 0xFE4CA6E5 // Address of device to send to

#define RXADDR2 0xFE4CA6E5 // Address of this device
#define TXADDR2 0x586F2E10 // Address of device to send to

void nRF905_debug()
{



	nRF905_init();

	// Set address of this device
	nRF905_setListenAddress(RXADDR);

	// Put into receive mode
	nRF905_RX();

	while(1)
	{

		uint8_t regs[NRF905_REGISTER_COUNT];
		nRF905_getConfigRegisters(regs);
		System::print("Raw: ");

		uint8_t dataValid = 0;

		for(uint8_t i=0;i<NRF905_REGISTER_COUNT;i++)
		{
			System::print("%u ", regs[i]);
			if(regs[i] == 0xFF || regs[i] == 0x00)
				dataValid++;
		}

		System::print("");

		// Registers were all 0xFF or 0x00,  this is probably bad
		if(dataValid >= NRF905_REGISTER_COUNT)
		{
			System::print("All registers read as 0xFF or 0x00! Is the nRF905 connected correctly?");
			HAL_Delay(1000);
			return;
		}

		char* str;
		uint8_t data;

		uint16_t channel = ((uint16_t)(regs[1] & 0x01)<<8) | regs[0];
		uint32_t freq = (422400UL + (channel * 100UL)) * (1 + ((regs[1] & ~NRF905_MASK_BAND) >> 1));

		System::print("Channel: %u", channel);
		System::print("Freq: %u", freq);
		System::print("KHz");
		System::print("Auto retransmit: %u\n", !!(regs[1] & ~NRF905_MASK_AUTO_RETRAN));
		System::print("Low power RX: %u\n", !!(regs[1] & ~NRF905_MASK_LOW_RX));

		// TX power
		data = regs[1] & ~NRF905_MASK_PWR;
		switch(data)
		{
			case NRF905_PWR_n10:
				data = -10;
				break;
			case NRF905_PWR_n2:
				data = -2;
				break;
			case NRF905_PWR_6:
				data = 6;
				break;
			case NRF905_PWR_10:
				data = 10;
				break;
			default:
				data = -127;
				break;
		}
		System::print("TX Power: %d dBm \n", (signed char)data);

		// Freq band
		data = regs[1] & ~NRF905_MASK_BAND;
		switch(data)
		{
			case NRF905_BAND_433:
				str = (char*)"433";
				break;
			default:
				str = (char*)"868/915";
				break;
		}
		System::print("Band: %s MHz\n", str);
		System::print("TX Address width: %u \n", regs[2] >> 4);
		System::print("RX Address width: %u \n", regs[2] & 0x07);


		System::print("RX Payload size: %u \n", regs[3]);
		System::print("TX Payload size: %u \n", regs[4]);

		System::print("RX Address [0]: %u \n", regs[5]);

		System::print("RX Address [1]: %u \n", regs[6]);

		System::print("RX Address [2]: %u \n", regs[7]);

		System::print("RX Address [3]: %u \n", regs[8]);

		System::print("RX Address: ", ((unsigned long)regs[8]<<24 | (unsigned long)regs[7]<<16 | (unsigned long)regs[6]<<8 | (unsigned long)regs[5]));


		// CRC mode
		data = regs[9] & ~NRF905_MASK_CRC;
		switch(data)
		{
			case NRF905_CRC_16:
				str = (char*)"16bit";
				break;
			case NRF905_CRC_8:
				str = (char*)"8bit";
				break;
			default:
				str = (char*)"Disabled";
				break;
		}
		System::print("CRC Mode: %s \n", str);

		// Xtal freq
		data = regs[9] & ~NRF905_MASK_CLK;
		switch(data)
		{
			case NRF905_CLK_4MHZ:
				data = 4;
				break;
			case NRF905_CLK_8MHZ:
				data = 8;
				break;
			case NRF905_CLK_12MHZ:
				data = 12;
				break;
			case NRF905_CLK_16MHZ:
				data = 16;
				break;
			case NRF905_CLK_20MHZ:
				data = 20;
				break;
			default:
				data = 0;
				break;
		}
		System::print("Xtal freq: %u MHz \n", data);

		// Clock out freq
		data = regs[9] & ~NRF905_MASK_OUTCLK;
		switch(data)
		{
			case NRF905_OUTCLK_4MHZ:
				str = (char*)"4MHz";
				break;
			case NRF905_OUTCLK_2MHZ:
				str = (char*)"2MHz";
				break;
			case NRF905_OUTCLK_1MHZ:
				str = (char*)"1MHz";
				break;
			case NRF905_OUTCLK_500KHZ:
				str = (char*)"500KHz";
				break;
			default:
				str = (char*)"Disabled";
				break;
		}
		System::print("Clock out freq: %s \n", str);

		System::print("---------------------\n");

		HAL_Delay(1000);
	}
}


void nRF905_testserver()
{
	nRF905_init();

	// Set address of this device
	nRF905_setListenAddress(RXADDR1);

	// Put into receive mode
	nRF905_RX();

	HAL_Delay(1);

	System::print("PingServer\n");

	uint8_t regs[NRF905_REGISTER_COUNT];
	nRF905_getConfigRegisters(regs);

	System::print("");

	// Registers were all 0xFF or 0x00,  this is probably bad

	uint16_t channel = ((uint16_t)(regs[1] & 0x01)<<8) | regs[0];
	uint32_t freq = (422400UL + (channel * 100UL)) * (1 + ((regs[1] & ~NRF905_MASK_BAND) >> 1));

	System::print("Channel: %u", channel);
	System::print("Freq: %u", freq);
	System::print("KHz");
	System::print("Auto retransmit: %u\n", !!(regs[1] & ~NRF905_MASK_AUTO_RETRAN));
	System::print("Low power RX: %u\n", !!(regs[1] & ~NRF905_MASK_LOW_RX));


	while(1)
	{

		static uint32_t pings=0;
		static uint32_t invalids=0;

		System::print("Waiting for ping...\n");

		// Wait for data
		while(!dataReady())
		{
			HAL_Delay(1);
		}

//		if(packetStatus != PACKET_OK)
//		{
//			invalids++;
//			System::print("Invalid packet!\n");
//			packetStatus = PACKET_NONE;
//			nRF905_RX();
//		}
//		else
//		{
		pings++;

		// Make buffer for data
		uint8_t buffer[NRF905_MAX_PAYLOAD];
		nRF905_read(buffer, sizeof(buffer));

		System::print("Got ping, sending reply...\n");


		char data[NRF905_MAX_PAYLOAD] = {0};
		sprintf(data, "junk");

		// Send back the data, once the transmission has completed go into receive mode
		while(!nRF905_TX(TXADDR1, data, sizeof(buffer), NRF905_NEXTMODE_RX));
		//while(!nRF905_TX(TXADDR1, data, sizeof(buffer), NRF905_NEXTMODE_RX));

		System::print("Reply sent\n");

		// Print out ping contents
		System::print("Data from server: %s\n", buffer);
//		}

		System::print("Totals: %d Ping,  %d Invalid \n ----------\n", pings, invalids);
	}
}

void nRF905_testclient()
{
	nRF905_init();

	// Set address of this device
	nRF905_setListenAddress(RXADDR2);

	// Put into receive mode
	nRF905_RX();

	HAL_Delay(1);

	System::print("PingClient\n");

	uint8_t regs[NRF905_REGISTER_COUNT];
	nRF905_getConfigRegisters(regs);

	System::print("");

	// Registers were all 0xFF or 0x00,  this is probably bad

	uint16_t channel = ((uint16_t)(regs[1] & 0x01)<<8) | regs[0];
	uint32_t freq = (422400UL + (channel * 100UL)) * (1 + ((regs[1] & ~NRF905_MASK_BAND) >> 1));

	System::print("Channel: %u", channel);
	System::print("Freq: %u", freq);
	System::print("KHz");
	System::print("Auto retransmit: %u\n", !!(regs[1] & ~NRF905_MASK_AUTO_RETRAN));
	System::print("Low power RX: %u\n", !!(regs[1] & ~NRF905_MASK_LOW_RX));



	while(1)
	{
		static uint8_t counter;
		static uint32_t sent;
		static uint32_t replies;
		static uint32_t timeouts;
		static uint32_t TIMEOUT = 1000;

		// Make data
		char data[NRF905_MAX_PAYLOAD] = {0};
		sprintf(data, "test %u", counter);
		System::print("%s test %u\n", data, counter);
		counter++;

		System::print("Sending data: %s\n", data);


		uint32_t startTime = 0;

		// Send the data (send fails if other transmissions are going on, keep trying until success) and enter RX mode on completion
		while(!nRF905_TX(TXADDR2, data, sizeof(data), NRF905_NEXTMODE_RX));
		sent++;

		System::print("Data sent, waiting for reply...\n");


		// Wait for reply with timeout
		uint32_t sendStartTime = 0;
		while(1)
		{
			if(dataReady())
			{
				// If success toggle LED and send ping time over UART
				uint16_t totalTime = startTime;

				static uint8_t ledState;
				ledState = !ledState;

				replies++;

				System::print("Ping time: %u ms\n", totalTime);

				// Get the ping data
				uint8_t replyData[NRF905_MAX_PAYLOAD];
				nRF905_read(replyData, sizeof(replyData));

				// Print out ping contents
				System::print("Data from server: %s \n", replyData);
				break;
			}
			else if(sendStartTime > TIMEOUT)
			{
				System::print("Ping timed out\n");
				timeouts++;
				break;
			}
			HAL_Delay(1);
			sendStartTime++;
		}

		System::print("Totals: %d , Sent: %d , Replies: %d, Timeouts: %d\n ----------\n", counter, sent, replies, timeouts);


		HAL_Delay(200);
	}
}
