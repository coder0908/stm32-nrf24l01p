/*
 * nrf24l01plus_driver.c
 *
 *  Created on: Apr 26, 2025
 *      Author: coder0908
 */


#include "nrf24l01plus_driver.h"

#define _BV(bit) (1 << bit)

static void en_cs(struct Nrf24 *rd)
{
	VMD_ASSERT_PARAM(rd);
	HAL_GPIO_WritePin(rd->csPort, rd->csPin, GPIO_PIN_RESET);
}

static void dis_cs(struct Nrf24 *rd)
{
	VMD_ASSERT_PARAM(rd);
	HAL_GPIO_WritePin(rd->csPort, rd->csPin, GPIO_PIN_SET);
}

static void en_ce(struct Nrf24 *rd)
{
	VMD_ASSERT_PARAM(rd);
	HAL_GPIO_WritePin(rd->cePort, rd->cePin, GPIO_PIN_SET);
}

static void dis_ce(struct Nrf24 *rd)
{
	VMD_ASSERT_PARAM(rd);
	HAL_GPIO_WritePin(rd->cePort, rd->cePin, GPIO_PIN_RESET);
}

//LSB first
static bool nrf24_write_spi(struct Nrf24 *rd, uint8_t reg, uint8_t *buf, uint16_t size, uint8_t *status)
{
	VMD_ASSERT_PARAM(rd);
	VMD_ASSERT_PARAM(buf);
	VMD_ASSERT_PARAM(size <= 32);

	HAL_StatusTypeDef spiStatus = HAL_ERROR;

	reg |= NRF24_CMD_W_REGISTER;

	en_cs(rd);

	spiStatus = HAL_SPI_TransmitReceive(rd->hspi, &reg, (status == NULL)?&(rd->statusReg):status, 1, 300);
	if (spiStatus != HAL_OK) {
		dis_cs(rd);
		return false;
	}

	spiStatus = HAL_SPI_TransmitReceive(rd->hspi, buf, rd->tmpBuf, size, 1000);
	dis_cs(rd);
	if (spiStatus != HAL_OK) {
		return false;
	}


	return true;
}

//LSB first
static bool nrf24_read_spi(struct Nrf24 *rd, uint8_t reg, uint8_t *buf, uint16_t size, uint8_t *status)
{
	VMD_ASSERT_PARAM((rd));
	VMD_ASSERT_PARAM((buf));
	VMD_ASSERT_PARAM(size <= 32);

	HAL_StatusTypeDef spiStatus = HAL_ERROR;

	reg &= NRF24_CMD_R_REGISTER;
	
	en_cs(rd);

	spiStatus = HAL_SPI_TransmitReceive(rd->hspi, &reg, (status == NULL)?&(rd->statusReg):status, 1, 300);
	if (spiStatus != HAL_OK) {
		dis_cs(rd);
		return false;
	}

	spiStatus = HAL_SPI_TransmitReceive(rd->hspi, rd->nopBuf, buf, size, 1000);
	dis_cs(rd);
	if (spiStatus != HAL_OK) {
		return false;
	}


	return true;
}

//ex) if you want to write 0bxxddddxx, data = 0bdddd, moreSigBitIdx = 5, lessSigBitIdx = 2
static bool nrf24_write_regByte(struct Nrf24 *rd, uint8_t reg, uint8_t data, uint8_t moreSigBitIdx, uint8_t lessSigBitIdx)
{
	bool ret = false;
	uint8_t mask = 0;
	uint8_t tmpReg = 0;

	VMD_ASSERT_PARAM(lessSigBitIdx < moreSigBitIdx);
	VMD_ASSERT_PARAM(moreSigBitIdx <= 7);
	VMD_ASSERT_PARAM(lessSigBitIdx <= 7);

	for (int i = lessSigBitIdx; i <= moreSigBitIdx; i++) {
		mask |= _BV(i);
	}
	data <<= lessSigBitIdx;
	data &= mask;

	ret = nrf24_read_spi(rd, reg, &tmpReg, 1, NULL);
	if (!ret) {
		return ret;
	}

	tmpReg &= ~mask;
	tmpReg |= data;

	return nrf24_write_spi(rd, reg, &tmpReg, 1, NULL);
}

//ex) if moreSigBitIdx = 5 and lessSigBitIdx = 3, then data = (0bxxdddxxx >> lessSigBitIdx)
static bool nrf24_read_regByte(struct Nrf24 *rd, uint8_t reg, uint8_t *data, uint8_t moreSigBitIdx, uint8_t lessSigBitIdx)
{
	bool ret = false;
	uint8_t mask = 0;
	uint8_t tmpReg = 0;

	VMD_ASSERT_PARAM(lessSigBitIdx < moreSigBitIdx);
	VMD_ASSERT_PARAM(moreSigBitIdx <= 7);
	VMD_ASSERT_PARAM(lessSigBitIdx <= 7);
	VMD_ASSERT_PARAM(data);

	for (int i = lessSigBitIdx; i <= moreSigBitIdx; i++) {
		mask |= _BV(i);
	}

	ret = nrf24_read_spi(rd, reg, &tmpReg, 1, NULL);
	if (!ret ) {
		return false;
	}

	tmpReg &= mask;
	tmpReg >>= lessSigBitIdx;

	*data = tmpReg;

	return ret;
}

static bool nrf24_write_regBit(struct Nrf24 *rd, uint8_t reg, bool en, uint8_t bitIdx)
{
	bool ret = false;
	uint8_t tmpReg = 0;

	VMD_ASSERT_PARAM(bitIdx <= 7);

	ret = nrf24_read_spi(rd, reg, &tmpReg, 1, NULL);
	if (!ret) {
		return ret;
	}

	tmpReg &= ~(1 << bitIdx);
	tmpReg |= en << bitIdx;

	return nrf24_write_spi(rd, reg, &tmpReg, 1, NULL);
}

static bool nrf24_read_regBit(struct Nrf24 *rd, uint8_t reg, bool *isEn, uint8_t bitIdx)
{
	bool ret = false;
	uint8_t tmpReg = 0;

	VMD_ASSERT_PARAM(bitIdx <= 7);
	VMD_ASSERT_PARAM((isEn));

	ret = nrf24_read_spi(rd, reg, &tmpReg, 1, NULL);
	if (!ret) {
		return false;
	}

	*isEn = tmpReg & _BV(bitIdx);

	return ret;
}

bool nrf24_init(struct Nrf24 *rd, SPI_HandleTypeDef *hspi, GPIO_TypeDef *cePort, uint16_t cePin, GPIO_TypeDef *csPort, uint16_t csPin)
{
	VMD_ASSERT_PARAM(rd);
	VMD_ASSERT_PARAM(cePort);
	VMD_ASSERT_PARAM(csPort);
	VMD_ASSERT_PARAM(hspi);

	rd->hspi = hspi;
	rd->cePort = cePort;
	rd->cePin = cePin;
	rd->csPort = csPort;
	rd->csPin = csPin;
	
	dis_cs(rd);
	dis_ce(rd);

	for (int i = 0; i < 32; i++) {
		rd->nopBuf[i] = NRF24_CMD_NOP;
	}
	
	return nrf24_en_power(rd, false);
}

bool nrf24_begin(struct Nrf24 *rd)
{
	bool isRx;
	bool ret = false;
	uint8_t statusReg;


	ret = nrf24_get_pmode(rd, &isRx);
	if (!ret) {
		return ret;
	}

	if (isRx) {
		en_ce(rd);
	}

	nrf24_flush_rxBuf(rd);
	nrf24_flush_txBuf(rd);
	nrf24_clear_irq(rd, true, true, true);

	ret = nrf24_read_status(rd, &statusReg);
	if (statusReg != 14) {
		return false;
	}

	return nrf24_en_power(rd, true);
}

bool nrf24_deinit(struct Nrf24 *rd)
{
	bool ret = false;

	ret = nrf24_en_power(rd, false);
	if (!ret) {
		return ret;
	}

	dis_ce(rd);
	dis_cs(rd);

	return true;
}



//toggle ce pin. max 4ms
bool nrf24_write_txPld(struct Nrf24 *rd, uint8_t *pld, uint16_t size)
{
	const uint8_t reg = NRF24_CMD_W_TX_PAYLOAD;
	HAL_StatusTypeDef spiStatus = HAL_ERROR;

	VMD_ASSERT_PARAM(size >= 1);
	VMD_ASSERT_PARAM(size <= 32);
	VMD_ASSERT_PARAM((rd));
	VMD_ASSERT_PARAM((pld));

	en_cs(rd);

	spiStatus = HAL_SPI_TransmitReceive(rd->hspi, &reg, &rd->statusReg, 1, 300);
	if (spiStatus != HAL_OK) {
		dis_cs(rd);
		return false;
	}

	spiStatus = HAL_SPI_TransmitReceive(rd->hspi, pld, rd->tmpBuf, size, 1000);
	dis_cs(rd);
	if (spiStatus != HAL_OK) {
		return false;
	}


	en_ce(rd);
	HAL_Delay(1);	//toggle cePin to transmit actually
	dis_ce(rd);

	return true;
}

//recommand to call nrf24_is_rx_empty() to check is fifo available
bool nrf24_read_rxPld(struct Nrf24 *rd, uint8_t *pld, uint16_t size)
{
	const uint8_t reg = NRF24_CMD_R_RX_PAYLOAD;
	HAL_StatusTypeDef spiStatus = HAL_ERROR;

	VMD_ASSERT_PARAM(size >= 1);
	VMD_ASSERT_PARAM(size <= 32);
	VMD_ASSERT_PARAM((rd));
	VMD_ASSERT_PARAM(pld);

	en_cs(rd);

	spiStatus = HAL_SPI_TransmitReceive(rd->hspi, &reg, &rd->statusReg, 1, 300);
	if (spiStatus != HAL_OK) {
		dis_cs(rd);
		return false;
	}

	spiStatus = HAL_SPI_TransmitReceive(rd->hspi, rd->nopBuf, pld, size, 1000);
	dis_cs(rd);
	if (spiStatus != HAL_OK) {
		return false;
	}


	return true;
}

//indicate "don't transmit ack(specific packet)" to receiver.
//requirement : nrf24_en_dyn_ack()
bool nrf24_write_txPldNoAck(struct Nrf24 *rd, uint8_t *pld, uint16_t size)
{
	const uint8_t reg = NRF24_CMD_W_TX_PAYLOAD_NOACK;
	HAL_StatusTypeDef spiStatus = HAL_ERROR;

	VMD_ASSERT_PARAM(size >= 1);
	VMD_ASSERT_PARAM(size <= 32);
	VMD_ASSERT_PARAM((rd));
	VMD_ASSERT_PARAM(pld);

	en_cs(rd);

	spiStatus = HAL_SPI_TransmitReceive(rd->hspi, &reg, &rd->statusReg, 1, 300);
	if (spiStatus != HAL_OK) {
		dis_cs(rd);
		return false;
	}

	spiStatus = HAL_SPI_TransmitReceive(rd->hspi, pld, rd->tmpBuf, size, 1000);
	dis_cs(rd);
	if (spiStatus != HAL_OK) {
		return false;
	}


	en_ce(rd);
	HAL_Delay(1);
	dis_ce(rd);

	return true;
}

//requirement : nrf24_en_ack_pld()
bool nrf24_write_ackPld(struct Nrf24 *rd, uint8_t pipe, const uint8_t *pld, uint8_t size)
{
	uint8_t reg = NRF24_CMD_W_ACK_PAYLOAD;
	HAL_StatusTypeDef spiStatus = HAL_ERROR;

	VMD_ASSERT_PARAM(size >= 1);
	VMD_ASSERT_PARAM(size <= 32);
	VMD_ASSERT_PARAM((rd));
	VMD_ASSERT_PARAM(pld);
	VMD_ASSERT_PARAM(pipe <= 5);

	reg |= pipe;

	en_cs(rd);

	spiStatus = HAL_SPI_TransmitReceive(rd->hspi, &reg, &rd->statusReg, 1, 300);
	if (spiStatus != HAL_OK) {
		dis_cs(rd);
		return false;
	}

	spiStatus = HAL_SPI_TransmitReceive(rd->hspi, pld, rd->tmpBuf, size, 1000);
	dis_cs(rd);

	if (spiStatus != HAL_OK) {
		return false;
	}

	return true;
}

//don't call while transmitting
bool nrf24_reuse_txPld(struct Nrf24 *rd)
{
	HAL_StatusTypeDef spiStatus = HAL_ERROR;
	const uint8_t reg = NRF24_CMD_REUSE_TX_PL;

	VMD_ASSERT_PARAM(rd);

	en_cs(rd);
	spiStatus = HAL_SPI_TransmitReceive(rd->hspi, &reg, &rd->statusReg, 1, 300);
	dis_cs(rd);

	if (spiStatus != HAL_OK) {
		return false;
	}

	return true;
}

//requirement : nrf24_set_dpl(), nrf24_en_dpl()
bool nrf24_read_pldWidth(struct Nrf24 *rd, uint8_t *pldWidth)
{
	HAL_StatusTypeDef spiStatus = HAL_ERROR;
	const uint8_t reg = NRF24_CMD_R_RX_PL_WID;

	VMD_ASSERT_PARAM((rd));
	VMD_ASSERT_PARAM(pldWidth);

	en_cs(rd);

	spiStatus = HAL_SPI_TransmitReceive(rd->hspi, &reg, &rd->statusReg, 1, 300);
	if (spiStatus != HAL_OK) {
		dis_cs(rd);
		return false;
	}

	spiStatus = HAL_SPI_TransmitReceive(rd->hspi, &reg, pldWidth, 1, 300);
	dis_cs(rd);

	if (spiStatus != HAL_OK) {
		return false;
	}	

	if (*pldWidth > 32) {
		nrf24_flush_rxBuf(rd);
		return false;
	}

	return true;
}

bool nrf24_read_status(struct Nrf24 *rd, uint8_t *status)
{
	HAL_StatusTypeDef spiStatus = HAL_ERROR;
	const uint8_t reg = NRF24_CMD_NOP;

	VMD_ASSERT_PARAM(rd);
	VMD_ASSERT_PARAM(status);

	en_cs(rd);
	spiStatus = HAL_SPI_TransmitReceive(rd->hspi, &reg, status, 1, 300);
	dis_cs(rd);

	if (spiStatus != HAL_OK) {
		return false;
	}

	return true;
}

//use when  max_rt irq asserted
bool nrf24_flush_txBuf(struct Nrf24 *rd)
{
	HAL_StatusTypeDef spiStatus = HAL_ERROR;
	const uint8_t reg = NRF24_CMD_FLUSH_TX;

	VMD_ASSERT_PARAM((rd));

	en_cs(rd);

	spiStatus = HAL_SPI_TransmitReceive(rd->hspi, &reg, &rd->statusReg, 1, 300);
	dis_cs(rd);
	if (spiStatus != HAL_OK) {
		return false;
	}

	return true;
}

//use when nrf24_r_pld_width() return over 32
bool nrf24_flush_rxBuf(struct Nrf24 *rd)
{
	HAL_StatusTypeDef spiStatus = HAL_ERROR;
	const uint8_t reg = NRF24_CMD_FLUSH_RX;

	VMD_ASSERT_PARAM((rd));

	en_cs(rd);

	spiStatus = HAL_SPI_TransmitReceive(rd->hspi, &reg, &rd->statusReg, 1, 300);
	dis_cs(rd);
	if (spiStatus != HAL_OK) {
		return false;
	}

	return true;
}

bool nrf24_en_irq(struct Nrf24 *rd, bool rx_dr, bool tx_ds, bool max_rt)
{
	return nrf24_write_regByte(rd, NRF24_REG_CONFIG, (!rx_dr) << 2 | (!tx_ds) << 1| (!max_rt) , 6, 4);
}

bool nrf24_isEn_irq(struct Nrf24 *rd, bool *rx_dr, bool *tx_ds, bool *max_rt)
{
	uint8_t configReg = 0;
	bool ret = false;

	ret = nrf24_read_regByte(rd, NRF24_REG_CONFIG, &configReg, 6, 4);

	if (rx_dr) {
		*rx_dr = !(configReg & _BV(2));
	}

	if (tx_ds) {
		*tx_ds = !(configReg & _BV(1));
	}

	if (max_rt) {
		*max_rt = !(configReg & _BV(0));
	}

	return ret;
}

//if irq asserted, use nrf24_clear_irq()
bool nrf24_read_irq(struct Nrf24 *rd, bool *rx_dr, bool *tx_ds, bool *max_rt)
{
	uint8_t configReg = 0;
	bool ret = false;

	ret = nrf24_read_regByte(rd, NRF24_REG_STATUS, &configReg, 6, 4);

	if (rx_dr) {
		*rx_dr = configReg & _BV(2);
	}

	if (tx_ds) {
		*tx_ds = configReg & _BV(1);
	}

	if (max_rt) {
		*max_rt = configReg & _BV(0);
	}

	return ret;
}

bool nrf24_clear_irq(struct Nrf24 *rd, bool rx_dr, bool tx_ds, bool max_rt)
{
	bool ret = false;

	ret = nrf24_write_regBit(rd, NRF24_REG_STATUS, rx_dr, 6);
	if (!ret) {
		return ret;
	}

	ret = nrf24_write_regBit(rd, NRF24_REG_STATUS, tx_ds, 5);
	if (!ret) {
		return ret;
	}

	return nrf24_write_regBit(rd, NRF24_REG_STATUS, max_rt, 4);

}

bool nrf24_set_crcLen(struct Nrf24 *rd, uint8_t crcLen)
{
	bool ret = false;

	switch(crcLen) {
	case 1:
		ret = nrf24_write_regBit(rd, NRF24_REG_CONFIG, true, 3);
		if (!ret) {
			return ret;
		}
		ret = nrf24_write_regBit(rd, NRF24_REG_CONFIG, false, 2);
		if (!ret) {
			return ret;
		}
		break;
	case 2:
		ret = nrf24_write_regBit(rd, NRF24_REG_CONFIG, true, 3);
		if (!ret) {
			return ret;
		}
		ret = nrf24_write_regBit(rd, NRF24_REG_CONFIG, true, 2);
		if (!ret) {
			return ret;
		}
		break;
	case 0:
		ret = nrf24_write_regBit(rd, NRF24_REG_CONFIG, false, 3);
		if (!ret) {
			return ret;
		}
		break;
	default:
		return false;
	}

	return true;
}

bool nrf24_get_crcLen(struct Nrf24 *rd, uint8_t *crcLen)
{
	bool ret = false;
	bool isEn_crc;
	bool is_crcLen2byte;

	VMD_ASSERT_PARAM(crcLen);

	ret = nrf24_read_regBit(rd, NRF24_REG_CONFIG, &isEn_crc, 3);
	if (!ret) {
		return ret;
	}

	ret = nrf24_read_regBit(rd, NRF24_REG_CONFIG, &is_crcLen2byte, 2);
	if (!ret) {
		return ret;
	}

	if (!isEn_crc) {
		*crcLen = 0;
	} else if (is_crcLen2byte) {
		*crcLen = 2;
	} else {
		*crcLen = 1;
	}

	return ret;
}

bool nrf24_en_power(struct Nrf24 *rd, bool en)
{
	return nrf24_write_regBit(rd, NRF24_REG_CONFIG, en, 1);
}

bool nrf24_isEn_power(struct Nrf24 *rd, bool *isEn)
{
	return nrf24_read_regBit(rd, NRF24_REG_CONFIG, isEn, 1);
}

bool nrf24_set_pmode(struct Nrf24 *rd, bool isRx)
{
	return nrf24_write_regBit(rd, NRF24_REG_CONFIG, isRx, 0);
}

bool nrf24_get_pmode(struct Nrf24 *rd, bool *isRx)
{
	return nrf24_read_regBit(rd, NRF24_REG_CONFIG, isRx, 0);
}

bool nrf24_en_autoAck(struct Nrf24 *rd, uint8_t pipe, bool en)
{
	VMD_ASSERT_PARAM(pipe <= 5);
	return nrf24_write_regBit(rd, NRF24_REG_EN_AA, en, pipe);
}

bool nrf24_isEn_autoAck(struct Nrf24 *rd, uint8_t pipe, bool *isEn)
{	
	VMD_ASSERT_PARAM(pipe <= 5);
	return nrf24_read_regBit(rd, NRF24_REG_EN_AA, isEn, pipe);
}

bool nrf24_en_rxAddr(struct Nrf24 *rd, uint8_t pipe, bool en)
{
	VMD_ASSERT_PARAM(pipe <= 5);
	return nrf24_write_regBit(rd, NRF24_REG_EN_RXADDR, en, pipe);
}
bool nrf24_isEn_rxAddr(struct Nrf24 *rd, uint8_t pipe, bool *isEn)
{
	VMD_ASSERT_PARAM(pipe <= 5);
	return nrf24_read_regBit(rd, NRF24_REG_EN_RXADDR, isEn, pipe);
}

bool nrf24_set_addrWidth(struct Nrf24 *rd, uint8_t addrWidth)
{
	VMD_ASSERT_PARAM(addrWidth >= 3 && addrWidth <= 5);

	return nrf24_write_regByte(rd, NRF24_REG_SETUP_AW, addrWidth - 2, 1, 0);
}

bool nrf24_get_addrWidth(struct Nrf24 *rd, uint8_t *addrWidth)
{
	bool ret = false;

	VMD_ASSERT_PARAM(addrWidth);

	ret = nrf24_read_regByte(rd, NRF24_REG_SETUP_AW, addrWidth, 1, 0);
	if (!ret ) {
		return false;
	}
	
	*addrWidth += 2;
	if (*addrWidth > 5 || *addrWidth < 3) {
		return false;
	}

	return ret;
}

bool nrf24_set_ard(struct Nrf24 *rd, uint16_t ard)
{
	VMD_ASSERT_PARAM(ard >= 250 && ard <= 4000);

	ard /= 250;
	ard--;

	return nrf24_write_regByte(rd, NRF24_REG_SETUP_RETR, ard, 7, 4 );
}

bool nrf24_get_ard(struct Nrf24 *rd, uint16_t *ard)
{
	bool ret = false;
	uint8_t tmp = 0;

	VMD_ASSERT_PARAM(ard);

	ret = nrf24_read_regByte(rd, NRF24_REG_SETUP_RETR, &tmp, 7, 4);
	if (!ret) {
		return ret;
	}

	*ard = tmp;
	*ard += 1;
	*ard *= 250;

	if (*ard < 250 || *ard > 4000) {
		return false;
	}

	return ret;
}

bool nrf24_set_arc(struct Nrf24 *rd, uint8_t arc)
{
	VMD_ASSERT_PARAM(arc <= 15);
	return nrf24_write_regByte(rd, NRF24_REG_SETUP_RETR, arc, 3, 0);
}

bool nrf24_get_arc(struct Nrf24 *rd, uint8_t *arc)
{
	bool ret = false;

	ret = nrf24_read_regByte(rd, NRF24_REG_SETUP_RETR, arc, 3, 0);
	if (!ret) {
		return ret;
	}

	if (*arc > 15) {
		return false;
	}

	return ret;
}

bool nrf24_set_channel(struct Nrf24 *rd, uint16_t mhz)
{
	VMD_ASSERT_PARAM(mhz >= 2400 && mhz <= 2525);
	return nrf24_write_regByte(rd, NRF24_REG_RF_CH, mhz - 2400, 6, 0);
}

bool nrf24_get_channel(struct Nrf24 *rd, uint16_t *mhz)
{
	bool ret = false;
	uint8_t tmp = 0;

	VMD_ASSERT_PARAM(mhz);

	ret = nrf24_read_regByte(rd, NRF24_REG_RF_CH, &tmp, 6, 0);
	if (!ret) {
		return ret;
	}

	*mhz = tmp;
	*mhz += 2400;

	if (*mhz < 2400 || *mhz > 2525)
		return false;

	return ret;
}

bool nrf24_set_dataRate(struct Nrf24 *rd, enum Nrf24_DataRate dataRate)
{
	bool ret = false;

	ret = nrf24_write_regBit(rd, NRF24_REG_RF_SETUP, false, 5);
	if (!ret) {
		return ret;
	}

	ret = nrf24_write_regBit(rd, NRF24_REG_RF_SETUP, false, 3);
	if (!ret) {
		return ret;
	}

	if (dataRate == NRF24_2MBPS) {
		return nrf24_write_regBit(rd, NRF24_REG_RF_SETUP, true, 3);
	} else if(dataRate == NRF24_250KBPS) {
		return nrf24_write_regBit(rd, NRF24_REG_RF_SETUP, true, 5);
	} else if(dataRate != NRF24_1MBPS) {
		return false;
	}

	return ret;

}

bool nrf24_get_dataRate(struct Nrf24 *rd, enum Nrf24_DataRate *dataRate)
{
	bool ret = false;
	bool lowBit = 0;
	bool highBit = 0;

	VMD_ASSERT_PARAM(dataRate);

	ret = nrf24_read_regBit(rd, NRF24_REG_RF_SETUP, &lowBit, 5);
	if (!ret) {
		return ret;
	}

	ret = nrf24_read_regBit(rd, NRF24_REG_RF_SETUP, &highBit, 3);
	if (!ret) {
		return ret;
	}


	if (lowBit && highBit) {
		return false;
	} else if (lowBit) {
		*dataRate = NRF24_250KBPS;
	} else if (highBit) {
		*dataRate = NRF24_2MBPS;
	} else {
		*dataRate = NRF24_1MBPS;
	}

	return ret;
}

bool nrf24_set_paPower(struct Nrf24 *rd, enum Nrf24_PaPower paPower)
{
	return nrf24_write_regByte(rd, NRF24_REG_RF_SETUP, paPower, 2, 1);
}

bool nrf24_get_paPower(struct Nrf24 *rd, enum Nrf24_PaPower *paPower)
{
	return nrf24_read_regByte(rd, NRF24_REG_RF_SETUP, paPower, 2, 1);
}

bool nrf24_read_pipeNum(struct Nrf24 *rd, uint8_t *pipe)
{
	bool ret = false;

	ret = nrf24_read_regByte(rd, NRF24_REG_STATUS, pipe, 3, 1);
	if (!ret) {
		return ret;
	}

	if (*pipe > 5) {
		return false;
	}

	return ret;
}

bool nrf24_is_txBufFull(struct Nrf24 *rd, bool *is_full)
{	
	return nrf24_read_regBit(rd, NRF24_REG_STATUS, is_full, 0);
}

bool nrf24_is_txBufEmpty(struct Nrf24 *rd, bool *is_empty)
{
	return nrf24_read_regBit(rd, NRF24_REG_FIFO_STATUS, is_empty, 4);
}

bool nrf24_is_rxBufFull(struct Nrf24 *rd, bool *is_full)
{
	return nrf24_read_regBit(rd, NRF24_REG_FIFO_STATUS, is_full, 1);
}

bool nrf24_is_rxBufEmpty(struct Nrf24 *rd, bool *is_empty)
{
	return nrf24_read_regBit(rd, NRF24_REG_FIFO_STATUS, is_empty, 0);
}

//packet loss count. +1 plost_cnt when max_rt irq asserted
//if plostCnt == 15 then transmit operation halted
//write value in RF_CH register with nrf24_set_channel() to reset plos_cnt.
bool nrf24_read_plosCnt(struct Nrf24 *rd, uint8_t *plosCnt)
{
	bool ret = false;

	VMD_ASSERT_PARAM(plosCnt);

	ret = nrf24_read_regByte(rd, NRF24_REG_OBSERVE_TX, plosCnt, 7, 4);
	if (!ret) {
		return ret;
	}

	if (*plosCnt > 15) {
		return false;
	}

	return ret;
}

bool nrf24_read_arc(struct Nrf24 *rd, uint8_t *arc)
{
	bool ret = false;

	VMD_ASSERT_PARAM(arc);

	ret = nrf24_read_regByte(rd, NRF24_REG_OBSERVE_TX, arc, 3, 0);
	if (!ret) {
		return ret;
	}

	if (*arc > 15) {
		return false;
	}

	return ret;
}

bool nrf24_set_rxAddr(struct Nrf24 *rd, uint8_t pipe, uint8_t *rxAddr, uint8_t width)
{
	bool ret = false;
	uint8_t read_addr[5] = {0,};

	VMD_ASSERT_PARAM(rxAddr);
	VMD_ASSERT_PARAM(width >= 3 && width <= 5);
	VMD_ASSERT_PARAM(pipe <= 5);

	if (pipe == 0 || pipe == 1) {
		return nrf24_write_spi(rd, NRF24_REG_RX_ADDR_P0 + pipe, rxAddr, width, NULL);
	}

	ret = nrf24_read_spi(rd, NRF24_REG_RX_ADDR_P1, read_addr, width, NULL);
	if (!ret) {
		return ret;
	}

	for (int i = 1; i < width; i++) {
		if (read_addr[i] != rxAddr[i]) {
			return false;
		}
	}

	return nrf24_write_spi(rd, NRF24_REG_RX_ADDR_P0 + pipe, rxAddr, 1, NULL);
}

bool nrf24_get_rxAddr(struct Nrf24 *rd, uint8_t pipe, uint8_t *rxAddr, uint8_t width)
{
	bool ret = false;

	VMD_ASSERT_PARAM(width >= 3 && width <= 5);
	VMD_ASSERT_PARAM(pipe <= 5);

	if (pipe == 0) {
		return nrf24_read_spi(rd, NRF24_REG_RX_ADDR_P0, rxAddr, width, NULL);
	}

	ret = nrf24_read_spi(rd, NRF24_REG_RX_ADDR_P1, rxAddr, width, NULL);
	if (!ret) {
		return ret;
	}

	if (pipe != 1) {
		ret = nrf24_read_spi(rd, NRF24_REG_RX_ADDR_P0 + pipe, rxAddr, 1, NULL);
	}

	return ret;
}

bool nrf24_set_txAddr(struct Nrf24 *rd, uint8_t *txAddr, uint8_t width)
{
	VMD_ASSERT_PARAM(width >= 3 && width <= 5);
	return nrf24_write_spi(rd, NRF24_REG_TX_ADDR, txAddr, width, NULL);
}

bool nrf24_get_txAddr(struct Nrf24 *rd, uint8_t *txAddr, uint8_t width)
{
	VMD_ASSERT_PARAM(width >= 3 && width <= 5);
	return nrf24_read_spi(rd, NRF24_REG_TX_ADDR, txAddr, width, NULL);
}

bool nrf24_set_rxPldWidth(struct Nrf24 *rd, uint8_t pipe, uint8_t pldWidth)
{
	VMD_ASSERT_PARAM(pldWidth <= 32);
	VMD_ASSERT_PARAM(pipe <= 5);

	return nrf24_write_regByte(rd, NRF24_REG_RX_PW_P0 + pipe, pldWidth, 5, 0);
}

bool nrf24_get_rxPldWidth(struct Nrf24 *rd, uint8_t pipe, uint8_t *pldWidth)
{
	bool ret = false;

	VMD_ASSERT_PARAM(pipe <= 5);

	ret = nrf24_read_regByte(rd, NRF24_REG_RX_PW_P0 + pipe, pldWidth, 5, 0);
	if (!ret) {
		return ret;
	}

	if (*pldWidth > 32) {
		nrf24_flush_rxBuf(rd);
		return false;
	}

	return ret;
}

//requirement: nrf24_en_auto_ack
bool nrf24_set_DPL(struct Nrf24 *rd, uint8_t pipe, bool en)
{
	VMD_ASSERT_PARAM(pipe <= 5);
	return nrf24_write_regBit(rd, NRF24_REG_DYNPD, en, pipe);
}

bool nrf24_get_DPL(struct Nrf24 *rd, uint8_t pipe, bool *isEn)
{
	VMD_ASSERT_PARAM(pipe <= 5);
	return nrf24_read_regBit(rd, NRF24_REG_DYNPD, isEn, pipe);

}

bool nrf24_en_DPL(struct Nrf24 *rd, bool en)
{
	return nrf24_write_regBit(rd, NRF24_REG_FEATURE, en, 2);
}

bool nrf24_isEn_DPL(struct Nrf24 *rd, bool *isEn)
{
	return nrf24_read_regBit(rd, NRF24_REG_FEATURE, isEn, 2);
}

//requirement : nrf24_set_dpl(), nrf24_en_dpl()
//specific pipe that dpl feature on.
bool nrf24_en_ackPld(struct Nrf24 *rd, bool en)
{
	return nrf24_write_regBit(rd, NRF24_REG_FEATURE, en, 1);
}

bool nrf24_isEn_ackPld(struct Nrf24 *rd, bool *isEn)
{
	return nrf24_read_regBit(rd, NRF24_REG_FEATURE, isEn, 1);
}

bool nrf24_en_dynAck(struct Nrf24 *rd, bool en)
{
	return nrf24_write_regBit(rd, NRF24_REG_FEATURE, en, 0);
}

bool nrf24_isEn_dynAck(struct Nrf24 *rd, bool *isEn)
{
	return nrf24_read_regBit(rd, NRF24_REG_FEATURE, isEn, 0);
}


bool nrf24_init_arduinoStyle(struct Nrf24 *rd)
{
	bool ret = false;

	ret = nrf24_set_arc(rd, 15);
	if (!ret)
		return ret;

	ret = nrf24_set_ard(rd, 250 * 5);
	if (!ret)
		return ret;

	ret = nrf24_set_dataRate(rd, NRF24_1MBPS);
	if (!ret)
		return ret;

	ret = nrf24_en_ackPld(rd, false);
	if (!ret)
		return ret;

	ret = nrf24_en_DPL(rd, false);
	if (!ret)
		return ret;

	ret = nrf24_en_dynAck(rd, false);
	if (!ret)
		return ret;

	for (int i = 0; i < 6; i++) {
		ret = nrf24_set_DPL(rd, i, false);
		if (!ret)
			return ret;
	}
	for (int i = 0; i < 6; i++) {
		ret = nrf24_en_autoAck(rd, i, true);
		if (!ret)
			return ret;
	}

	ret = nrf24_en_rxAddr(rd, 0, true);
	if (!ret)
		return ret;

	ret = nrf24_en_rxAddr(rd, 1, true);
	if (!ret)
		return ret;

	for (int i = 0; i < 6; i++)
		ret = nrf24_set_rxPldWidth(rd, i, 32);

	ret = nrf24_set_addrWidth(rd, 5);
	if (!ret)
		return ret;

	ret = nrf24_set_channel(rd, 2476);
	if (!ret)
		return ret;

	ret = nrf24_clear_irq(rd, true, true, true);
	if (!ret)
		return ret;

	ret = nrf24_flush_txBuf(rd);
	if (!ret)
		return ret;

	ret = nrf24_flush_rxBuf(rd);
	if (!ret)
		return ret;

	ret = nrf24_set_crcLen(rd, 2);
	if (!ret)
		return ret;

	return ret;
}
