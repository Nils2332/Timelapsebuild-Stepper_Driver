//#include "stm32f4xx_hal.h"
#include "nRF905.h"
#include "nRF905_config.h"
#include "nRF905_defs.h"
#include "gpio.h"
#include "spi.h"
#include "main.h"
#include "stdlib.h"


static SPI_HandleTypeDef* spi = &hspi2;

uint8_t POWERED_UP(void)
{
	uint8_t status = (HAL_GPIO_ReadPin(PWRUP_GPIO_Port, PWRUP_Pin));
	return status;
}

void POWER_UP(void)
{
	HAL_GPIO_WritePin(PWRUP_GPIO_Port, PWRUP_Pin, GPIO_PIN_SET);
}
void POWER_DOWN(void)
{
	HAL_GPIO_WritePin(PWRUP_GPIO_Port, PWRUP_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(TRX_CE_GPIO_Port, TRX_CE_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(TX_EN_GPIO_Port, TX_EN_Pin, GPIO_PIN_RESET);
}
void STANDBY(void)
{
	HAL_GPIO_WritePin(TRX_CE_GPIO_Port, TRX_CE_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(TX_EN_GPIO_Port, TX_EN_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(PWRUP_GPIO_Port, PWRUP_Pin, GPIO_PIN_SET);
	HAL_Delay(3);
}
void TXMODE(void)
{
	HAL_GPIO_WritePin(TRX_CE_GPIO_Port, TRX_CE_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(TX_EN_GPIO_Port, TX_EN_Pin, GPIO_PIN_SET);
}
void RXMODE(void)
{
	HAL_GPIO_WritePin(TX_EN_GPIO_Port, TX_EN_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(TRX_CE_GPIO_Port, TRX_CE_Pin, GPIO_PIN_SET);
}
void spiSelect(void)
{
	HAL_GPIO_WritePin(CSN_GPIO_Port, CSN_Pin, GPIO_PIN_RESET);
}
void spiDeselect(void)
{
	HAL_GPIO_WritePin(CSN_GPIO_Port, CSN_Pin, GPIO_PIN_SET);
}

void spiTransferNR(uint8_t data)
{
	uint8_t datain[1] = { data };
	HAL_SPI_Transmit(spi, datain, 1, 10);
}

uint8_t spiTransfer(void)
{
	uint8_t dataout[1];
	HAL_SPI_TransmitReceive(spi, (uint8_t *)NRF905_CMD_NOP, dataout, 1, 10);
	return dataout[0];
}

static uint8_t readConfigRegister(uint8_t reg)
{
	uint8_t val = 0;

	spiSelect();
	spiTransferNR(NRF905_CMD_R_CONFIG | reg);
	val = spiTransfer();
	spiDeselect();

	return val;
}

static void writeConfigRegister(uint8_t reg, uint8_t val)
{
	spiSelect();
	spiTransferNR(NRF905_CMD_W_CONFIG | reg);
	spiTransferNR(val);
	spiDeselect();
}

static void setConfigReg1(uint8_t val, uint8_t mask, uint8_t reg)
{
	writeConfigRegister(reg, (readConfigRegister(NRF905_REG_CONFIG1) & mask) | val);
}

static void setConfigReg2(uint8_t val, uint8_t mask, uint8_t reg)
{
	writeConfigRegister(reg, (readConfigRegister(NRF905_REG_CONFIG2) & mask) | val);
}

static const uint8_t config[]= {
	NRF905_CMD_W_CONFIG,
	NRF905_CHANNEL,
	NRF905_AUTO_RETRAN | NRF905_LOW_RX | NRF905_PWR | NRF905_BAND | ((NRF905_CHANNEL>>8) & 0x01),
	(NRF905_ADDR_SIZE<<4) | NRF905_ADDR_SIZE,
	NRF905_PAYLOAD_SIZE, // RX payload size
	NRF905_PAYLOAD_SIZE, // TX payload size
	0xE7, 0xE7, 0xE7, 0xE7, // Default receive address
	NRF905_CRC | NRF905_CLK_FREQ | NRF905_OUTCLK
};

static void defaultConfig(void)
{

	// Set control registers
	spiSelect();
	for(uint8_t i=0;i<sizeof(config);i++)
		spiTransferNR(config[i]);
	spiDeselect();

	// Default transmit address
	spiSelect();
	spiTransferNR(NRF905_CMD_W_TX_ADDRESS);
	for(uint8_t i=0;i<4;i++)
		spiTransferNR(0xE7);
	spiDeselect();

	// Clear transmit payload
	spiSelect();
	spiTransferNR(NRF905_CMD_W_TX_PAYLOAD);
		for(uint8_t i=0;i<NRF905_MAX_PAYLOAD;i++)
			spiTransferNR(0x00);
	spiDeselect();


}

static void setAddress(uint32_t address, uint8_t cmd)
{
	spiSelect();
	spiTransferNR(cmd);
	for(uint8_t i=0;i<4;i++)
		spiTransferNR(address>>(8 * i));
	spiDeselect();
}

static uint8_t readStatus(void)
{
	uint8_t status;
	spiSelect();
	status = spiTransfer();
	spiDeselect();
	return status;
}

uint8_t dataReady(void)
{
#if NRF905_DR_SW
	return (readStatus() & (1<<NRF905_STATUS_DR));
#else
	return HAL_GPIO_ReadPin(DR_GPIO_Port, DR_Pin)
#endif
}

static uint8_t addressMatched(void)
{
#if NRF905_AM_SW
	return (readStatus() & (1<<NRF905_STATUS_AM));
#else
	return HAL_GPIO_ReadPin(AM_GPIO_Port, AM_Pin)
#endif
}

void nRF905_init()
{
	spiDeselect();

	POWER_DOWN();
	STANDBY();
	nRF905_RX();
	defaultConfig();
}

void nRF905_setChannel(uint16_t channel)
{
	if(channel > 511)
		channel = 511;

	uint8_t reg = (readConfigRegister(NRF905_REG_CONFIG1) & NRF905_MASK_CHANNEL) | (channel>>8);

	spiSelect();
	spiTransferNR(NRF905_CMD_W_CONFIG | NRF905_REG_CHANNEL);
	spiTransferNR(channel);
	spiTransferNR(reg);
	spiDeselect();
}

void nRF905_setBand(nRF905_band_t band)
{
	uint8_t reg = (readConfigRegister(NRF905_REG_CONFIG1) & NRF905_MASK_BAND) | band;

	spiSelect();
	spiTransferNR(NRF905_CMD_W_CONFIG | NRF905_REG_CONFIG1);
	spiTransferNR(reg);
	spiDeselect();
}

void nRF905_setAutoRetransmit(nRF905_auto_retran_t val)
{
	setConfigReg1(val, NRF905_MASK_AUTO_RETRAN, NRF905_REG_AUTO_RETRAN);
}

void nRF905_setLowRxPower(nRF905_low_rx_t val)
{
	setConfigReg1(val, NRF905_MASK_LOW_RX, NRF905_REG_LOW_RX);
}

void nRF905_setTransmitPower(nRF905_pwr_t val)
{
	setConfigReg1(val, NRF905_MASK_PWR, NRF905_REG_PWR);
}

void nRF905_setCRC(nRF905_crc_t val)
{
	setConfigReg2(val, NRF905_MASK_CRC, NRF905_REG_CRC);
}

void nRF905_setClockOut(nRF905_outclk_t val)
{
	setConfigReg2(val, NRF905_MASK_OUTCLK, NRF905_REG_OUTCLK);
}

void nRF905_setPayloadSize(uint8_t size)
{
	spiSelect();
	if(size > NRF905_MAX_PAYLOAD)
		size = NRF905_MAX_PAYLOAD;

	spiTransferNR(NRF905_CMD_W_CONFIG | NRF905_REG_RX_PAYLOAD_SIZE);
	spiTransferNR(size);
	spiTransferNR(size);
	spiDeselect();
}

void nRF905_setAddressSize(nRF905_addr_size_t size)
{
	spiSelect();
	spiTransferNR(NRF905_CMD_W_CONFIG | NRF905_REG_ADDR_WIDTH);
	spiTransferNR((size<<4) | size);
	spiDeselect();
}

uint8_t nRF905_receiveBusy()
{
	return addressMatched();
}

uint8_t nRF905_airwayBusy()
{
#if NRF905_COLLISION_AVOID
	return HAL_GPIO_ReadPin(CD_GPIO_Port, CD_Pin);
#else
	return 0;
#endif
}

void nRF905_setListenAddress(uint32_t address)
{
	setAddress(address, NRF905_CMD_W_CONFIG | NRF905_REG_RX_ADDRESS);
}

uint8_t nRF905_TX(uint32_t sendTo, void* data, uint8_t len, nRF905_nextmode_t nextMode)
{
	// TODO check DR is low?

	while(dataReady())
	{
		HAL_Delay(1);
	}

#if NRF905_COLLISION_AVOID
		if(nRF905_airwayBusy())
			return 0;
#endif

	setAddress(sendTo, NRF905_CMD_W_TX_ADDRESS);

	if(!POWERED_UP())
	{
		STANDBY();
		HAL_Delay(3);
	}


	HAL_GPIO_WritePin(TX_EN_GPIO_Port, TX_EN_Pin, GPIO_PIN_SET);

	// Load new payload
	if(data != NULL)
	{
		spiSelect();
		spiTransferNR(NRF905_CMD_W_TX_PAYLOAD);
		for(uint8_t i=0;i<len;i++)
			spiTransferNR(((uint8_t*)data)[i]);
		spiDeselect();

//		if(!POWERED_UP())
//		{
//			STANDBY();
//			POWER_UP();
//			HAL_Delay(3);
//		}
	}

#if NRF905_COLLISION_AVOID
		if(nRF905_airwayBusy())
			return 0;
#endif

	// Put into transmit mode
	//TXMODE();
	HAL_GPIO_WritePin(TRX_CE_GPIO_Port, TRX_CE_Pin, GPIO_PIN_SET);
	HAL_Delay(1);
	HAL_GPIO_WritePin(TRX_CE_GPIO_Port, TRX_CE_Pin, GPIO_PIN_RESET);

	if(nextMode == NRF905_NEXTMODE_RX)
	{
		// The datasheets says that the radio can switch straight to RX mode after
		// a transmission is complete by clearing TX_EN while transmitting, but
		// if this is done within ~700us the transmission seems to get corrupt.
		HAL_Delay(1);
		RXMODE();
	}
	else if(nextMode == NRF905_NEXTMODE_STANDBY)
	{
		HAL_Delay(1);
		STANDBY();
	}
	// else NRF905_NEXTMODE_TX

	return 1;
}

void nRF905_RX()
{
	NRF905_NO_INTERRUPT()
	{
		POWER_UP();
		RXMODE();
	}
}

void nRF905_read(void* data, uint8_t len)
{
	if(len > NRF905_MAX_PAYLOAD)
		len = NRF905_MAX_PAYLOAD;

	spiSelect();
	spiTransferNR(NRF905_CMD_R_RX_PAYLOAD);

	// Get received payload
	for(uint8_t i=0;i<len;i++)
		((uint8_t*)data)[i] = spiTransfer();

	// Must make sure all of the payload has been read, otherwise DR never goes low
	//uint8_t remaining = NRF905_MAX_PAYLOAD - len;
	//while(remaining--)
	//	spi_transfer_nr(NRF905_CMD_NOP);

}

void nRF905_powerDown()
{
	POWER_DOWN();
}

void nRF905_powerUp()
{
	uint8_t wasPoweredUp = POWERED_UP();
	STANDBY();
	POWER_UP();
	if(!wasPoweredUp)
		HAL_Delay(3);
}

void nRF905_standby()
{
	STANDBY();
	POWER_UP();
	HAL_Delay(3);
}

void nRF905_getConfigRegisters(void* regs)
{
	spiSelect();
	spiTransferNR(NRF905_CMD_R_CONFIG);
	for(uint8_t i=0;i<NRF905_REGISTER_COUNT;i++)
	{
		((uint8_t*)regs)[i] = spiTransfer();
	}
	spiDeselect();
}
