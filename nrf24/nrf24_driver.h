/*
 * nrf24.h
 *
 *  Created on: Apr 26, 2025
 *      Author: coder0908
 */

#ifndef __NRF24L01PLUS_DRIVER_H__
#define __NRF24L01PLUS_DRIVER_H__

#include <stdint.h>
#include <stdbool.h>
#include "stm32f4xx_hal.h"
#include "vmd.h"

#define NRF24_MAX_PLD_WIDTH 32

/*COMMAND*/
#define NRF24_CMD_R_REGISTER		0b00011111
#define NRF24_CMD_W_REGISTER		0b00100000
#define NRF24_CMD_R_RX_PAYLOAD		0b01100001
#define NRF24_CMD_W_TX_PAYLOAD		0b10100000
#define NRF24_CMD_FLUSH_TX		0b11100001
#define NRF24_CMD_FLUSH_RX		0b11100010
#define NRF24_CMD_REUSE_TX_PL		0b11100011
#define NRF24_CMD_R_RX_PL_WID		0b01100000
#define NRF24_CMD_W_ACK_PAYLOAD		0b10101000
#define NRF24_CMD_W_TX_PAYLOAD_NOACK	0b10110000
#define NRF24_CMD_NOP			0xff

/*REGISTER*/
#define NRF24_REG_CONFIG		0x00
#define NRF24_REG_EN_AA			0x01
#define NRF24_REG_EN_RXADDR		0x02
#define NRF24_REG_SETUP_AW		0x03
#define NRF24_REG_SETUP_RETR		0x04
#define NRF24_REG_RF_CH			0x05
#define NRF24_REG_RF_SETUP		0x06
#define NRF24_REG_STATUS		0x07
#define NRF24_REG_OBSERVE_TX		0x08
#define NRF24_REG_RPD			0x09
#define NRF24_REG_RX_ADDR_P0		0x0A
#define NRF24_REG_RX_ADDR_P1		0x0B
#define NRF24_REG_RX_ADDR_P2		0x0C
#define NRF24_REG_RX_ADDR_P3		0x0D
#define NRF24_REG_RX_ADDR_P4		0x0E
#define NRF24_REG_RX_ADDR_P5		0x0F
#define NRF24_REG_TX_ADDR		0x10
#define NRF24_REG_RX_PW_P0		0x11
#define NRF24_REG_RX_PW_P1		0x12
#define NRF24_REG_RX_PW_P2		0x13
#define NRF24_REG_RX_PW_P3		0x14
#define NRF24_REG_RX_PW_P4		0x15
#define NRF24_REG_RX_PW_P5		0x16
#define NRF24_REG_FIFO_STATUS		0x17
#define NRF24_REG_DYNPD			0x1C
#define NRF24_REG_FEATURE		0x1D


struct Nrf24 {
	/*public*/
	GPIO_TypeDef 	*cePort;
	uint16_t  	cePin;

	GPIO_TypeDef *csPort;
	uint16_t csPin;

	SPI_HandleTypeDef *hspi;

	/* private */
	uint8_t statusReg;
	uint8_t tmpBuf[32];
	uint8_t nopBuf[32];
};

enum Nrf24_PaPower {
	NRF24_PA_M18DBM = 0,
	NRF24_PA_M12DBM,
	NRF24_PA_M6DBM,
	NRF24_PA_0DBM
};

enum Nrf24_DataRate {
	NRF24_1MBPS = 0,
	NRF24_2MBPS,
	NRF24_250KBPS
};



bool nrf24_init(struct Nrf24 *rd);
bool nrf24_deinit(struct Nrf24 *rd);

bool nrf24_begin(struct Nrf24 *rd);

bool nrf24_write_txPld(struct Nrf24 *rd, uint8_t *txPld, uint16_t size);
bool nrf24_read_rxPld(struct Nrf24 *rd, uint8_t *rx_pld, uint16_t size);
bool nrf24_write_txPldNoAck(struct Nrf24 *rd, uint8_t *txPld, uint16_t size);
bool nrf24_write_ackPld(struct Nrf24 *rd, uint8_t pipe, const uint8_t *ackPld, uint8_t size);

bool nrf24_reuse_txPld(struct Nrf24 *rd);

bool nrf24_read_pldWidth(struct Nrf24 *rd, uint8_t *pldWidth);
bool nrf24_read_status(struct Nrf24 *rd, uint8_t *status);

bool nrf24_flush_txBuf(struct Nrf24 *rd);
bool nrf24_flush_rxBuf(struct Nrf24 *rd);

bool nrf24_en_irq(struct Nrf24 *rd, bool RX_DR, bool TX_DS, bool MAX_RT);
bool nrf24_isEn_irq(struct Nrf24 *rd, bool *RX_DR, bool *TX_DS, bool *MAX_RT);

bool nrf24_read_irq(struct Nrf24 *rd, bool *rx_dr, bool *tx_ds, bool *MAX_RT);
bool nrf24_clear_irq(struct Nrf24 *rd, bool rx_dr, bool tx_ds, bool MAX_RT);

bool nrf24_set_crcLen(struct Nrf24 *rd, uint8_t crcLen);
bool nrf24_get_crcLen(struct Nrf24 *rd, uint8_t *crcLen);

bool nrf24_en_power(struct Nrf24 *rd, bool en);
bool nrf24_isEn_power(struct Nrf24 *rd, bool *isEn);

bool nrf24_set_pmode(struct Nrf24 *rd, bool isRx);
bool nrf24_get_pmode(struct Nrf24 *rd, bool *isRx);

bool nrf24_en_autoAck(struct Nrf24 *rd, uint8_t pipe, bool en);
bool nrf24_isEn_autoAck(struct Nrf24 *rd, uint8_t pipe, bool *isEn);

bool nrf24_en_rxAddr(struct Nrf24 *rd, uint8_t pipe, bool en);
bool nrf24_isEn_rxAddr(struct Nrf24 *rd, uint8_t pipe, bool *isEn);

bool nrf24_set_addrWidth(struct Nrf24 *rd, uint8_t addrWidth);
bool nrf24_get_addrWidth(struct Nrf24 *rd, uint8_t *addrWidth);

bool nrf24_set_ARD(struct Nrf24 *rd, uint16_t ARD);
bool nrf24_get_ARD(struct Nrf24 *rd, uint16_t *ARD);	

bool nrf24_set_ARC(struct Nrf24 *rd, uint8_t ARC);
bool nrf24_get_ARC(struct Nrf24 *rd, uint8_t *ARC);

bool nrf24_set_channel(struct Nrf24 *rd, uint16_t Mhz);
bool nrf24_get_channel(struct Nrf24 *rd, uint16_t *Mhz);

bool nrf24_set_dataRate(struct Nrf24 *rd, enum Nrf24_DataRate dataRate);
bool nrf24_get_dataRate(struct Nrf24 *rd, enum Nrf24_DataRate *dataRate);

bool nrf24_set_paPower(struct Nrf24 *rd, enum Nrf24_PaPower paPower);
bool nrf24_get_paPower(struct Nrf24 *rd, enum Nrf24_PaPower *paPower);

bool nrf24_read_pipeNum(struct Nrf24 *rd, uint8_t *pipe);

bool nrf24_is_txBufFull(struct Nrf24 *rd, bool *isFull);
bool nrf24_is_rxBufFull(struct Nrf24 *rd, bool *isFull);	

bool nrf24_is_rxBufEmpty(struct Nrf24 *rd, bool *isEmpty);
bool nrf24_is_txBufEmpty(struct Nrf24 *rd, bool *isEmpty);

bool nrf24_read_plosCnt(struct Nrf24 *rd, uint8_t *plosCnt);
bool nrf24_read_ARCnt(struct Nrf24 *rd, uint8_t *ARC);

bool nrf24_set_rxAddr(struct Nrf24 *rd, uint8_t pipe, uint8_t *rxAddr, uint8_t width);
bool nrf24_get_rxAddr(struct Nrf24 *rd, uint8_t pipe, uint8_t *rxAddr, uint8_t width);

bool nrf24_set_txAddr(struct Nrf24 *rd, uint8_t *tx_addr, uint8_t width);
bool nrf24_get_txAddr(struct Nrf24 *rd, uint8_t *tx_addr, uint8_t width);

bool nrf24_set_rxPldWidth(struct Nrf24 *rd, uint8_t pipe, uint8_t pldWidth);
bool nrf24_get_rxPldWidth(struct Nrf24 *rd, uint8_t pipe, uint8_t *pldWidth);

bool nrf24_set_DPL(struct Nrf24 *rd, uint8_t pipe, bool en);
bool nrf24_get_DPL(struct Nrf24 *rd, uint8_t pipe, bool *isEn);
bool nrf24_en_DPL(struct Nrf24 *rd, bool en);
bool nrf24_isEn_DPL(struct Nrf24 *rd, bool *en);

bool nrf24_en_ackPld(struct Nrf24 *rd, bool en);
bool nrf24_isEn_ackPld(struct Nrf24 *rd, bool *isEn);

bool nrf24_en_dynAck(struct Nrf24 *rd, bool en);
bool nrf24_isEn_dynAck(struct Nrf24 *rd, bool *isEn);

bool nrf24_init_arduinoStyle(struct Nrf24 *rd);

#endif /* __NRF24L01PLUS_DRIVER_H__*/
