/*
 * nrf24.h
 *
 *  Created on: Apr 26, 2025
 *      Author: coder0908
 */

#ifndef __NRF24_H__
#define __NRF24_H__

#include <stdint.h>
#include <stdbool.h>
#include "stm32f4xx_hal.h"
#include "vmd.h"

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


struct nrf24 {
	/*public*/
	GPIO_TypeDef 	*ce_port;
	uint16_t  	ce_pin;

	GPIO_TypeDef *cs_port;
	uint16_t cs_pin;

	SPI_HandleTypeDef *hspi;

	/* private */
	uint8_t status_reg;
	uint8_t tmp_buf[32];
	uint8_t nop_buf[32];
};

enum nrf24_pa_power {
	NRF24_PA_M18DBM = 0,
	NRF24_PA_M12DBM,
	NRF24_PA_M6DBM,
	NRF24_PA_0DBM
};

enum nrf24_data_rate {
	NRF24_1MBPS = 0,
	NRF24_2MBPS,
	NRF24_250KBPS
};



enum vmd_ret nrf24_init(struct nrf24 *rd);
enum vmd_ret nrf24_deinit(struct nrf24 *rd);

enum vmd_ret nrf24_begin(struct nrf24 *rd);

enum vmd_ret nrf24_w_tx_pld(struct nrf24 *rd, uint8_t *tx_pld, uint16_t size);
enum vmd_ret nrf24_r_rx_pld(struct nrf24 *rd, uint8_t *rx_pld, uint16_t size);
enum vmd_ret nrf24_w_tx_pld_no_ack(struct nrf24 *rd, uint8_t *tx_pld, uint16_t size);
enum vmd_ret nrf24_w_ack_pld(struct nrf24 *rd, uint8_t pipe, uint8_t *ack_pld, uint8_t size);

enum vmd_ret nrf24_reuse_tx_pld(struct nrf24 *rd);

enum vmd_ret nrf24_r_pld_width(struct nrf24 *rd, uint8_t *pld_width);
enum vmd_ret nrf24_r_status(struct nrf24 *rd, uint8_t *status);

enum vmd_ret nrf24_flush_tx(struct nrf24 *rd);
enum vmd_ret nrf24_flush_rx(struct nrf24 *rd);

enum vmd_ret nrf24_set_irq(struct nrf24 *rd, bool RX_DR, bool TX_DS, bool MAX_RT);
enum vmd_ret nrf24_get_irq(struct nrf24 *rd, bool *RX_DR, bool *TX_DS, bool *MAX_RT);

enum vmd_ret nrf24_is_asserted_irq(struct nrf24 *rd, bool *rx_dr, bool *tx_ds, bool *max_rt);
enum vmd_ret nrf24_clear_irq(struct nrf24 *rd, bool rx_dr, bool tx_ds, bool max_rt);

enum vmd_ret nrf24_set_crc(struct nrf24 *rd, uint8_t crc_byte_len);
enum vmd_ret nrf24_get_crc(struct nrf24 *rd, uint8_t *crc_byte_len);

enum vmd_ret nrf24_en_power_up(struct nrf24 *rd, bool en);
enum vmd_ret nrf24_isen_power_up(struct nrf24 *rd, bool *is_en);

enum vmd_ret nrf24_set_pmode(struct nrf24 *rd, bool is_rx);
enum vmd_ret nrf24_get_pmode(struct nrf24 *rd, bool *is_rx);

enum vmd_ret nrf24_set_auto_ack(struct nrf24 *rd, uint8_t pipe, bool en);
enum vmd_ret nrf24_get_auto_ack(struct nrf24 *rd, uint8_t pipe, bool *is_en);

enum vmd_ret nrf24_en_rx_addr(struct nrf24 *rd, uint8_t pipe, bool en);
enum vmd_ret nrf24_isen_rx_addr(struct nrf24 *rd, uint8_t pipe, bool *is_en);

enum vmd_ret nrf24_set_addr_width(struct nrf24 *rd, uint8_t addr_width);
enum vmd_ret nrf24_get_addr_width(struct nrf24 *rd, uint8_t *addr_width);

enum vmd_ret nrf24_set_ard(struct nrf24 *rd, uint16_t auto_re_trmit_delay);
enum vmd_ret nrf24_get_ard(struct nrf24 *rd, uint16_t *auto_re_trmit_delay);

enum vmd_ret nrf24_set_arc(struct nrf24 *rd, uint8_t auto_re_trmit_cnt);
enum vmd_ret nrf24_get_arc(struct nrf24 *rd, uint8_t *auto_re_trmit_cnt);

enum vmd_ret nrf24_set_channel(struct nrf24 *rd, uint16_t Mhz);
enum vmd_ret nrf24_get_channel(struct nrf24 *rd, uint16_t *Mhz);

enum vmd_ret nrf24_set_data_rate(struct nrf24 *rd, enum nrf24_data_rate data_rate);
enum vmd_ret nrf24_get_data_rate(struct nrf24 *rd, enum nrf24_data_rate *data_rate);

enum vmd_ret nrf24_set_pa_power(struct nrf24 *rd, enum nrf24_pa_power pa_pwr);
enum vmd_ret nrf24_get_pa_power(struct nrf24 *rd, enum nrf24_pa_power *pa_pwr);

enum vmd_ret nrf24_r_pipe_num(struct nrf24 *rd, uint8_t *pipe);

enum vmd_ret nrf24_is_tx_full(struct nrf24 *rd, bool *is_full);
enum vmd_ret nrf24_is_rx_full(struct nrf24 *rd, bool *is_full);

enum vmd_ret nrf24_is_rx_empty(struct nrf24 *rd, bool *is_empty);
enum vmd_ret nrf24_is_tx_empty(struct nrf24 *rd, bool *is_empty);

enum vmd_ret nrf24_r_plos_cnt(struct nrf24 *rd, uint8_t *plos_cnt);
enum vmd_ret nrf24_r_arc_cnt(struct nrf24 *rd, uint8_t *auto_re_trmit_cnt);

enum vmd_ret nrf24_set_rx_addr(struct nrf24 *rd, uint8_t pipe, uint8_t *rx_addr, uint8_t width);
enum vmd_ret nrf24_get_rx_addr(struct nrf24 *rd, uint8_t pipe, uint8_t *rx_addr, uint8_t width);

enum vmd_ret nrf24_set_tx_addr(struct nrf24 *rd, uint8_t *tx_addr, uint8_t width);
enum vmd_ret nrf24_get_tx_addr(struct nrf24 *rd, uint8_t *tx_addr, uint8_t width);

enum vmd_ret nrf24_set_rx_pld_width(struct nrf24 *rd, uint8_t pipe, uint8_t pld_width);
enum vmd_ret nrf24_get_rx_pld_width(struct nrf24 *rd, uint8_t pipe, uint8_t *pld_width);

enum vmd_ret nrf24_set_dpl(struct nrf24 *rd, uint8_t pipe, bool en);
enum vmd_ret nrf24_get_dpl(struct nrf24 *rd, uint8_t pipe, bool *is_en);
enum vmd_ret nrf24_en_dpl(struct nrf24 *rd, bool en);
enum vmd_ret nrf24_isen_dpl(struct nrf24 *rd, bool *en);

enum vmd_ret nrf24_en_ack_pld(struct nrf24 *rd, bool en);
enum vmd_ret nrf24_isen_ack_pld(struct nrf24 *rd, bool *is_en);

enum vmd_ret nrf24_en_dyn_ack(struct nrf24 *rd, bool en);
enum vmd_ret nrf24_isen_dyn_ack(struct nrf24 *rd, bool *is_en);

#endif /* __NRF24_H__*/
