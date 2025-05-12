/*
 * nrf24.c
 *
 *  Created on: Apr 26, 2025
 *      Author: coder0908
 */


#include <nrf24_driver.h>

#define _BV(bit) (1 << bit)

static void en_cs(struct nrf24 *rd)
{
	VMD_ASSERT_PARAM(NOT_NULL(rd));
	HAL_GPIO_WritePin(rd->cs_port, rd->cs_pin, GPIO_PIN_RESET);
}

static void dis_cs(struct nrf24 *rd)
{
	VMD_ASSERT_PARAM(NOT_NULL(rd));
	HAL_GPIO_WritePin(rd->cs_port, rd->cs_pin, GPIO_PIN_SET);
}

static void en_ce(struct nrf24 *rd)
{
	VMD_ASSERT_PARAM(NOT_NULL(rd));
	HAL_GPIO_WritePin(rd->ce_port, rd->ce_pin, GPIO_PIN_SET);
}

static void dis_ce(struct nrf24 *rd)
{
	VMD_ASSERT_PARAM(NOT_NULL(rd));
	HAL_GPIO_WritePin(rd->ce_port, rd->ce_pin, GPIO_PIN_RESET);
}

//LSB first
static enum vmd_ret write_spi(struct nrf24 *rd, uint8_t reg, uint8_t *tx_buf, uint16_t size, uint8_t *status)
{
	VMD_ASSERT_PARAM(NOT_NULL(rd));
	VMD_ASSERT_PARAM(NOT_NULL(tx_buf));
	VMD_ASSERT_PARAM(size <= 32);

	HAL_StatusTypeDef spi_reslt;

	reg |= NRF24_CMD_W_REGISTER;

	en_cs(rd);

	spi_reslt = HAL_SPI_TransmitReceive(rd->hspi, &reg, (status == NULL)?&(rd->status_reg):status, 1, 300);
	if (spi_reslt != HAL_OK) {
		return VMD_FAIL;
	}

	spi_reslt = HAL_SPI_TransmitReceive(rd->hspi, tx_buf, rd->tmp_buf, size, 1000);
	if (spi_reslt != HAL_OK) {
		return VMD_FAIL;
	}

	dis_cs(rd);

	return VMD_SUCC;
}

//LSB first
static enum vmd_ret read_spi(struct nrf24 *rd, uint8_t reg, uint8_t *rx_buf, uint16_t size, uint8_t *status)
{
	VMD_ASSERT_PARAM(NOT_NULL(rd));
	VMD_ASSERT_PARAM(NOT_NULL(rx_buf));
	VMD_ASSERT_PARAM(size <= 32);

	HAL_StatusTypeDef spi_reslt;

	reg &= NRF24_CMD_R_REGISTER;
	
	en_cs(rd);

	spi_reslt = HAL_SPI_TransmitReceive(rd->hspi, &reg, (status == NULL)?&(rd->status_reg):status, 1, 300);
	if (spi_reslt != HAL_OK) {
		return VMD_FAIL;
	}

	spi_reslt = HAL_SPI_TransmitReceive(rd->hspi, rd->nop_buf, rx_buf, size, 1000);
	if (spi_reslt != HAL_OK) {
		return VMD_FAIL;
	}

	dis_cs(rd);

	return VMD_SUCC;
}

//ex) if you want to write 0bxxddddxx, data = 0bdddd, more_sig_bit_idx = 5, least_sig_bit_idx = 2
static enum vmd_ret write_byte(struct nrf24 *rd, uint8_t reg, uint8_t data, uint8_t more_sig_bit_idx, uint8_t least_sig_bit_idx)
{
	enum vmd_ret ret;
	uint8_t mask = 0;
	uint8_t read_reg = 0;

	VMD_ASSERT_PARAM( least_sig_bit_idx < more_sig_bit_idx);
	VMD_ASSERT_PARAM(more_sig_bit_idx <= 7);
	VMD_ASSERT_PARAM(least_sig_bit_idx <= 7);

	for (int i = least_sig_bit_idx; i <= more_sig_bit_idx; i++) {
		mask |= _BV(i);
	}
	data <<= least_sig_bit_idx;
	data &= mask;

	ret = read_spi(rd, reg, &read_reg, 1, NULL);
	if (ret != VMD_SUCC) {
		return ret;
	}

	read_reg &= ~mask;
	read_reg |= data;

	return write_spi(rd, reg, &read_reg, 1, NULL);
}

//ex) more_sig_bit_idx = 5, least_sig_bit_idx = 3, then data = (0bxxdddxxx >> least_sig_bit_idx)
static enum vmd_ret read_byte(struct nrf24 *rd, uint8_t reg, uint8_t *data, uint8_t more_sig_bit_idx, uint8_t least_sig_bit_idx)
{
	enum vmd_ret ret;
	uint8_t mask = 0;
	uint8_t read_reg = 0;

	VMD_ASSERT_PARAM( least_sig_bit_idx < more_sig_bit_idx);
	VMD_ASSERT_PARAM(more_sig_bit_idx <= 7);
	VMD_ASSERT_PARAM(least_sig_bit_idx <= 7);
	VMD_ASSERT_PARAM(NOT_NULL(data));

	for (int i = least_sig_bit_idx; i <= more_sig_bit_idx; i++) {
		mask |= _BV(i);
	}

	ret = read_spi(rd, reg, &read_reg, 1, NULL);

	read_reg &= mask;
	read_reg >>= least_sig_bit_idx;

	*data = read_reg;

	return ret;
}

static enum vmd_ret write_bit(struct nrf24 *rd, uint8_t reg, bool en, uint8_t bit_idx)
{
	enum vmd_ret ret;
	uint8_t read_reg = 0;

	VMD_ASSERT_PARAM(bit_idx <= 7);

	ret = read_spi(rd, reg, &read_reg, 1, NULL);
	if (ret != VMD_SUCC) {
		return ret;
	}

	read_reg &= ~(1 << bit_idx);
	read_reg |= en << bit_idx;

	return write_spi(rd, reg, &read_reg, 1, NULL);
}

static enum vmd_ret read_bit(struct nrf24 *rd, uint8_t reg, bool *is_en, uint8_t bit_idx)
{
	enum vmd_ret ret;
	uint8_t read_reg = 0;

	VMD_ASSERT_PARAM(bit_idx <= 7);
	VMD_ASSERT_PARAM(NOT_NULL(is_en));

	ret = read_spi(rd, reg, &read_reg, 1, NULL);

	*is_en = read_reg & _BV(bit_idx);

	return ret;
}

enum vmd_ret nrf24_init(struct nrf24 *rd)
{
	VMD_ASSERT_PARAM(NOT_NULL(rd));
	
	dis_cs(rd);
	dis_ce(rd);

	for (int i = 0; i < 32; i++) {
		rd->nop_buf[i] = NRF24_CMD_NOP;
	}
	
	return nrf24_en_power_up(rd, false);
}

enum vmd_ret nrf24_begin(struct nrf24 *rd)
{
	bool is_rx;
	enum vmd_ret ret;

	ret = nrf24_get_pmode(rd, &is_rx);
	if (ret != VMD_SUCC) {
		return ret;
	}

	if (is_rx) {
		en_ce(rd);
	}

	return nrf24_en_power_up(rd, true);
}

enum vmd_ret nrf24_deinit(struct nrf24 *rd)
{
	enum vmd_ret ret;

	ret = nrf24_en_power_up(rd, false);
	if (ret != VMD_SUCC) {
		return ret;
	}

	dis_ce(rd);
	dis_cs(rd);

	return VMD_SUCC;
}



//actual transmitting
//toggle ce pin. max 4ms
//if enabled "enhanced shockburst" you can keep the pin on
enum vmd_ret nrf24_w_tx_pld(struct nrf24 *rd, uint8_t *tx_pld, uint16_t size)
{
	const uint8_t reg = NRF24_CMD_W_TX_PAYLOAD;
	HAL_StatusTypeDef spi_reslt;

	VMD_ASSERT_PARAM(size >= 1);
	VMD_ASSERT_PARAM(size <= 32);
	VMD_ASSERT_PARAM(NOT_NULL(rd));
	VMD_ASSERT_PARAM(NOT_NULL(tx_pld));

	en_cs(rd);

	spi_reslt = HAL_SPI_TransmitReceive(rd->hspi, &reg, &rd->status_reg, 1, 300);
	if (spi_reslt != HAL_OK) {
		return VMD_FAIL;
	}

	spi_reslt = HAL_SPI_TransmitReceive(rd->hspi, tx_pld, rd->tmp_buf, size, 1000);
	if (spi_reslt != HAL_OK) {
		return VMD_FAIL;
	}

	dis_cs(rd);

	en_ce(rd);
	HAL_Delay(1);
	dis_ce(rd);

	return VMD_SUCC;
}

//actual receiveing
//recommand to call nrf24_is_rx_empty() to check is fifo available
enum vmd_ret nrf24_r_rx_pld(struct nrf24 *rd, uint8_t *rx_pld, uint16_t size)
{
	const uint8_t reg = NRF24_CMD_R_RX_PAYLOAD;
	HAL_StatusTypeDef spi_reslt;

	VMD_ASSERT_PARAM(size >= 1);
	VMD_ASSERT_PARAM(size <= 32);
	VMD_ASSERT_PARAM(NOT_NULL(rd));
	VMD_ASSERT_PARAM(NOT_NULL(rx_pld));

	en_cs(rd);

	spi_reslt = HAL_SPI_TransmitReceive(rd->hspi, &reg, &rd->status_reg, 1, 300);
	if (spi_reslt != HAL_OK) {
		return VMD_FAIL;
	}

	spi_reslt = HAL_SPI_TransmitReceive(rd->hspi, rd->nop_buf, rx_pld, size, 1000);
	if (spi_reslt != HAL_OK) {
		return VMD_FAIL;
	}

	dis_cs(rd);

	return VMD_SUCC;
}

//indicate "don't transmit ack(specific packet)" to receiver.
//requirement : nrf24_en_dyn_ack()
enum vmd_ret nrf24_w_tx_pld_no_ack(struct nrf24 *rd, uint8_t *tx_pld, uint16_t size)
{
	const uint8_t reg = NRF24_CMD_W_TX_PAYLOAD_NOACK;
	HAL_StatusTypeDef spi_reslt;

	VMD_ASSERT_PARAM(size >= 1);
	VMD_ASSERT_PARAM(size <= 32);
	VMD_ASSERT_PARAM(NOT_NULL(rd));
	VMD_ASSERT_PARAM(NOT_NULL(tx_pld));

	en_cs(rd);

	spi_reslt = HAL_SPI_TransmitReceive(rd->hspi, &reg, &rd->status_reg, 1, 300);
	if (spi_reslt != HAL_OK) {
		return VMD_FAIL;
	}

	spi_reslt = HAL_SPI_TransmitReceive(rd->hspi, tx_pld, rd->tmp_buf, size, 1000);
	if (spi_reslt != HAL_OK) {
		return VMD_FAIL;
	}

	dis_cs(rd);

	en_ce(rd);
	HAL_Delay(1);
	dis_ce(rd);

	return VMD_SUCC;
}

//use in RX mode.
//requirement : nrf24_en_ack_pld()
enum vmd_ret nrf24_w_ack_pld(struct nrf24 *rd, uint8_t pipe, const uint8_t *ack_pld, uint8_t size)
{
	uint8_t reg = NRF24_CMD_W_ACK_PAYLOAD;
	HAL_StatusTypeDef spi_reslt;

	VMD_ASSERT_PARAM(size >= 1);
	VMD_ASSERT_PARAM(size <= 32);
	VMD_ASSERT_PARAM(NOT_NULL(rd));
	VMD_ASSERT_PARAM(NOT_NULL(ack_pld));

	VMD_ASSERT_PARAM(pipe <= 5);

	reg |= pipe;

	en_cs(rd);

	spi_reslt = HAL_SPI_TransmitReceive(rd->hspi, &reg, &rd->status_reg, 1, 300);
	if (spi_reslt != HAL_OK) {
		return VMD_FAIL;
	}

	spi_reslt = HAL_SPI_TransmitReceive(rd->hspi, ack_pld, rd->tmp_buf, size, 1000);
	if (spi_reslt != HAL_OK) {
		return VMD_FAIL;
	}

	dis_cs(rd);

	return VMD_SUCC;
}

//don't call while transmitting
enum vmd_ret nrf24_reuse_tx_pld(struct nrf24 *rd)
{
	HAL_StatusTypeDef spi_reslt;
	const uint8_t reg = NRF24_CMD_REUSE_TX_PL;

	VMD_ASSERT_PARAM(NOT_NULL(rd));

	en_cs(rd);

	spi_reslt = HAL_SPI_TransmitReceive(rd->hspi, &reg, &rd->status_reg, 1, 300);
	if (spi_reslt != HAL_OK) {
		return VMD_FAIL;
	}

	dis_cs(rd);

	return VMD_SUCC;
}

//requirement : nrf24_set_dpl()
//description :Read RX payload width for the top in RX_FIFO
//use in PRX(if enabled dpl specific packet) or PTX(if enabled ack_pld specific packet)
enum vmd_ret nrf24_r_pld_width(struct nrf24 *rd, uint8_t *pld_width)
{
	HAL_StatusTypeDef spi_reslt;
	const uint8_t reg = NRF24_CMD_R_RX_PL_WID;

	VMD_ASSERT_PARAM(NOT_NULL(rd));
	VMD_ASSERT_PARAM(NOT_NULL(pld_width));

	en_cs(rd);

	spi_reslt = HAL_SPI_TransmitReceive(rd->hspi, &reg, &rd->status_reg, 1, 300);
	if (spi_reslt != HAL_OK) {
		return VMD_FAIL;
	}

	spi_reslt = HAL_SPI_TransmitReceive(rd->hspi, &reg, pld_width, 1, 300);
	if (spi_reslt != HAL_OK) {
		return VMD_FAIL;
	}	

	dis_cs(rd);

	if (*pld_width > 32) {
		nrf24_flush_rx(rd);
		return VMD_FAIL;
	}

	return VMD_SUCC;
}

enum vmd_ret nrf24_r_status(struct nrf24 *rd, uint8_t *status)
{
	HAL_StatusTypeDef spi_reslt;
	const uint8_t reg = NRF24_CMD_NOP;

	VMD_ASSERT_PARAM(NOT_NULL(rd));
	VMD_ASSERT_PARAM(NOT_NULL(status));

	en_cs(rd);

	spi_reslt = HAL_SPI_TransmitReceive(rd->hspi, &reg, status, 1, 300);
	if (spi_reslt != HAL_OK) {
		return VMD_FAIL;
	}

	dis_cs(rd);

	return VMD_SUCC;
}

//use when  MAX_RT irq asserted
enum vmd_ret nrf24_flush_tx(struct nrf24 *rd)
{
	HAL_StatusTypeDef spi_reslt;
	const uint8_t reg = NRF24_CMD_FLUSH_TX;

	VMD_ASSERT_PARAM(NOT_NULL(rd));

	en_cs(rd);

	spi_reslt = HAL_SPI_TransmitReceive(rd->hspi, &reg, &rd->status_reg, 1, 300);
	if (spi_reslt != HAL_OK) {
		return VMD_FAIL;
	}

	dis_cs(rd);

	return VMD_SUCC;
}

//use when nrf24_r_pld_width() return over 32
enum vmd_ret nrf24_flush_rx(struct nrf24 *rd)
{
	HAL_StatusTypeDef spi_reslt;
	const uint8_t reg = NRF24_CMD_FLUSH_RX;

	VMD_ASSERT_PARAM(NOT_NULL(rd));

	en_cs(rd);

	spi_reslt = HAL_SPI_TransmitReceive(rd->hspi, &reg, &rd->status_reg, 1, 300);
	if (spi_reslt != HAL_OK) {
		return VMD_FAIL;
	}

	dis_cs(rd);

	return VMD_SUCC;
}

enum vmd_ret nrf24_set_irq(struct nrf24 *rd, bool RX_DR, bool TX_DS, bool MAX_RT)
{
	return write_byte(rd, NRF24_REG_CONFIG, (!RX_DR) << 2 | (!TX_DS) << 1| (!MAX_RT) , 6, 4);
}

enum vmd_ret nrf24_get_irq(struct nrf24 *rd, bool *RX_DR, bool *TX_DS, bool *MAX_RT)
{
	uint8_t read_bits = 0;
	enum vmd_ret ret;

	ret = read_byte(rd, NRF24_REG_CONFIG, &read_bits, 6, 4);

	if (NOT_NULL(RX_DR)) {
		*RX_DR = !(read_bits & _BV(2));
	}

	if (NOT_NULL(TX_DS)) {
		*TX_DS = !(read_bits & _BV(1));
	}

	if (NOT_NULL(MAX_RT)) {
		*MAX_RT = !(read_bits & _BV(0));
	}

	return ret;
}

//use nrf24_clear_irq(), if irq asserted
enum vmd_ret nrf24_is_asserted_irq(struct nrf24 *rd, bool *rx_dr, bool *tx_ds, bool *max_rt)
{
	uint8_t read_bits = 0;
	enum vmd_ret ret;

	ret = read_byte(rd, NRF24_REG_STATUS, &read_bits, 6, 4);

	if (NOT_NULL(rx_dr)) {
		*rx_dr = read_bits & _BV(2);
	}

	if (NOT_NULL(tx_ds)) {
		*tx_ds = read_bits & _BV(1);
	}

	if (NOT_NULL(max_rt)) {
		*max_rt = read_bits & _BV(0);
	}

	return ret;
}

enum vmd_ret nrf24_clear_irq(struct nrf24 *rd, bool rx_dr, bool tx_ds, bool max_rt)
{
	enum vmd_ret ret;

	ret = write_bit(rd, NRF24_REG_STATUS, rx_dr, 6);
	if (ret != VMD_SUCC) {
		return ret;
	}

	ret = write_bit(rd, NRF24_REG_STATUS, tx_ds, 5);
	if (ret != VMD_SUCC) {
		return ret;
	}

	return write_bit(rd, NRF24_REG_STATUS, max_rt, 4);

}

enum vmd_ret nrf24_set_crc(struct nrf24 *rd, uint8_t crc_byte_len)
{
	enum vmd_ret ret;

	switch(crc_byte_len) {
	case 1:
		ret = write_bit(rd, NRF24_REG_CONFIG, true, 3);
		if (ret != VMD_SUCC) {
			return ret;
		}
		ret = write_bit(rd, NRF24_REG_CONFIG, false, 2);
		if (ret != VMD_SUCC) {
			return ret;
		}
		break;
	case 2:
		ret = write_bit(rd, NRF24_REG_CONFIG, true, 3);
		if (ret != VMD_SUCC) {
			return ret;
		}
		ret = write_bit(rd, NRF24_REG_CONFIG, true, 2);
		if (ret != VMD_SUCC) {
			return ret;
		}
		break;
	case 0:
		ret = write_bit(rd, NRF24_REG_CONFIG, false, 3);
		if (ret != VMD_SUCC) {
			return ret;
		}
		break;
	default:
		return VMD_FAIL;
	}

	return VMD_SUCC;
}

enum vmd_ret nrf24_get_crc(struct nrf24 *rd, uint8_t *crc_byte_len)
{
	enum vmd_ret ret;
	bool isen_crc;
	bool is_crc_2byte;

	ret = read_bit(rd, NRF24_REG_CONFIG, &isen_crc, 3);
	if (ret != VMD_SUCC) {
		return ret;
	}

	ret = read_bit(rd, NRF24_REG_CONFIG, &is_crc_2byte, 2);
	if (ret != VMD_SUCC) {
		return ret;
	}

	if (!isen_crc) {
		*crc_byte_len = 0;
	} else if (is_crc_2byte) {
		*crc_byte_len = 2;
	} else {
		*crc_byte_len = 1;
	}

	return ret;
}

enum vmd_ret nrf24_en_power_up(struct nrf24 *rd, bool en)
{
	return write_bit(rd, NRF24_REG_CONFIG, en, 1);
}

enum vmd_ret nrf24_isen_power_up(struct nrf24 *rd, bool *is_en)
{
	return read_bit(rd, NRF24_REG_CONFIG, is_en, 1);
}

enum vmd_ret nrf24_set_pmode(struct nrf24 *rd, bool is_rx)
{
	return write_bit(rd, NRF24_REG_CONFIG, is_rx, 0);
}

enum vmd_ret nrf24_get_pmode(struct nrf24 *rd, bool *is_rx)
{
	return read_bit(rd, NRF24_REG_CONFIG, is_rx, 0);
}

enum vmd_ret nrf24_set_auto_ack(struct nrf24 *rd, uint8_t pipe, bool en)
{
	VMD_ASSERT_PARAM(pipe <= 5);
	return write_bit(rd, NRF24_REG_EN_AA, en, pipe);
}

enum vmd_ret nrf24_get_auto_ack(struct nrf24 *rd, uint8_t pipe, bool *is_en)
{	
	VMD_ASSERT_PARAM(pipe <= 5);
	return read_bit(rd, NRF24_REG_EN_AA, is_en, pipe);
}

enum vmd_ret nrf24_en_rx_addr(struct nrf24 *rd, uint8_t pipe, bool en)
{
	VMD_ASSERT_PARAM(pipe <= 5);
	return write_bit(rd, NRF24_REG_EN_RXADDR, en, pipe);
}
enum vmd_ret nrf24_isen_rx_addr(struct nrf24 *rd, uint8_t pipe, bool *is_en)
{
	VMD_ASSERT_PARAM(pipe <= 5);
	return read_bit(rd, NRF24_REG_EN_RXADDR, is_en, pipe);
}

enum vmd_ret nrf24_set_addr_width(struct nrf24 *rd, uint8_t addr_width)
{
	VMD_ASSERT_PARAM(addr_width >= 3 && addr_width <= 5);

	return write_byte(rd, NRF24_REG_SETUP_AW, addr_width - 2, 1, 0);
}

enum vmd_ret nrf24_get_addr_width(struct nrf24 *rd, uint8_t *addr_width)
{
	enum vmd_ret ret;

	VMD_ASSERT_PARAM(NOT_NULL(addr_width));

	ret = read_byte(rd, NRF24_REG_SETUP_AW, addr_width, 1, 0);
	
	*addr_width += 2;
	if (*addr_width > 5) {
		return VMD_FAIL;
	}

	return ret;
}

enum vmd_ret nrf24_set_ard(struct nrf24 *rd, uint16_t auto_retr_delay)
{
	VMD_ASSERT_PARAM(auto_retr_delay >= 250 && auto_retr_delay <= 4000);

	auto_retr_delay /= 250;
	auto_retr_delay--;

	return write_byte(rd, NRF24_REG_SETUP_RETR, auto_retr_delay, 7, 4 );
}

enum vmd_ret nrf24_get_ard(struct nrf24 *rd, uint16_t *auto_retr_delay)
{
	enum vmd_ret ret;
	uint8_t tmp;

	VMD_ASSERT_PARAM(NOT_NULL(auto_retr_delay));

	ret = read_byte(rd, NRF24_REG_SETUP_RETR, &tmp, 7, 4);
	if (ret != VMD_SUCC) {
		return ret;
	}

	*auto_retr_delay = tmp;
	*auto_retr_delay += 1;
	*auto_retr_delay *= 250;

	if (*auto_retr_delay < 250 || *auto_retr_delay > 4000) {
		return VMD_FAIL;
	}

	return ret;
}

enum vmd_ret nrf24_set_arc(struct nrf24 *rd, uint8_t auto_retr_cnt)
{
	VMD_ASSERT_PARAM(auto_retr_cnt <= 15);
	return write_byte(rd, NRF24_REG_SETUP_RETR, auto_retr_cnt, 3, 0);
}

enum vmd_ret nrf24_get_arc(struct nrf24 *rd, uint8_t *auto_retr_cnt)
{

	enum vmd_ret ret;

	ret = read_byte(rd, NRF24_REG_SETUP_RETR, auto_retr_cnt, 3, 0);
	if (ret != VMD_SUCC) {
		return ret;
	}

	if (*auto_retr_cnt > 15) {
		return VMD_FAIL;
	}

	return ret;
}

enum vmd_ret nrf24_set_channel(struct nrf24 *rd, uint16_t Mhz)
{
	VMD_ASSERT_PARAM(Mhz >= 2400 && Mhz <= 2525);
	return write_byte(rd, NRF24_REG_RF_CH, Mhz - 2400, 6, 0);
}

enum vmd_ret nrf24_get_channel(struct nrf24 *rd, uint16_t *Mhz)
{
	enum vmd_ret ret = true;
	uint8_t tmp = 0;

	VMD_ASSERT_PARAM(NOT_NULL(Mhz));

	ret = read_byte(rd, NRF24_REG_RF_CH, &tmp, 6, 0);
	if (ret != VMD_SUCC) {
		return ret;
	}

	*Mhz = tmp;
	*Mhz += 2400;

	if (*Mhz < 2400 || *Mhz > 2525)
		return VMD_FAIL;

	return ret;
}

enum vmd_ret nrf24_set_data_rate(struct nrf24 *rd, enum nrf24_data_rate data_rate)
{
	enum vmd_ret ret;

	ret = write_bit(rd, NRF24_REG_RF_SETUP, false, 5);
	if (ret != VMD_SUCC) {
		return ret;
	}

	ret = write_bit(rd, NRF24_REG_RF_SETUP, false, 3);
	if (ret != VMD_SUCC) {
		return ret;
	}

	if (data_rate == NRF24_2MBPS) {
		return write_bit(rd, NRF24_REG_RF_SETUP, true, 3);
	} else if(data_rate == NRF24_250KBPS) {
		return write_bit(rd, NRF24_REG_RF_SETUP, true, 5);
	} else if(data_rate != NRF24_1MBPS) {
		return VMD_FAIL;
	}

	return ret;

}

enum vmd_ret nrf24_get_data_rate(struct nrf24 *rd, enum nrf24_data_rate *data_rate)
{
	enum vmd_ret ret;
	bool bit_low = 0;
	bool bit_high = 0;

	VMD_ASSERT_PARAM(NOT_NULL(data_rate));

	ret = read_bit(rd, NRF24_REG_RF_SETUP, &bit_low, 5);
	if (ret != VMD_SUCC) {
		return ret;
	}

	ret = read_bit(rd, NRF24_REG_RF_SETUP, &bit_high, 3);
	if (ret != VMD_SUCC) {
		return ret;
	}

	*data_rate = NRF24_1MBPS;

	if (bit_low && bit_high) {
		return VMD_FAIL;
	} else if (bit_low) {
		*data_rate = NRF24_250KBPS;
	} else if (bit_high) {
		*data_rate = NRF24_2MBPS;
	}

	return ret;
}

enum vmd_ret nrf24_set_pa_power(struct nrf24 *rd, enum nrf24_pa_power pa_pwr)
{
	return write_byte(rd, NRF24_REG_RF_SETUP, pa_pwr, 2, 1);
}

enum vmd_ret nrf24_get_pa_power(struct nrf24 *rd, enum nrf24_pa_power *pa_pwr)
{
	return read_byte(rd, NRF24_REG_RF_SETUP, pa_pwr, 2, 1);
}

enum vmd_ret nrf24_r_pipe_num(struct nrf24 *rd, uint8_t *pipe)
{
	enum vmd_ret ret;
	ret = read_byte(rd, NRF24_REG_STATUS, pipe, 3, 1);
	if (ret != VMD_SUCC) {
		return ret;
	}

	if (*pipe > 5) {
		return VMD_FAIL;
	}

	return ret;
}

enum vmd_ret nrf24_is_tx_full(struct nrf24 *rd, bool *is_full)
{	
	return read_bit(rd, NRF24_REG_STATUS, is_full, 0);
}

enum vmd_ret nrf24_is_tx_empty(struct nrf24 *rd, bool *is_empty)
{
	return read_bit(rd, NRF24_REG_FIFO_STATUS, is_empty, 4);
}

enum vmd_ret nrf24_is_rx_full(struct nrf24 *rd, bool *is_full)
{
	return read_bit(rd, NRF24_REG_FIFO_STATUS, is_full, 1);
}

enum vmd_ret nrf24_is_rx_empty(struct nrf24 *rd, bool *is_empty)
{
	return read_bit(rd, NRF24_REG_FIFO_STATUS, is_empty, 0);
}

//packet loss count. each max_rt asserted(arc) == +1 plost_cnt
//write value in RF_CH register to call nrf24_set_channel() to reset plos_cnt.
enum vmd_ret nrf24_r_plos_cnt(struct nrf24 *rd, uint8_t *plos_cnt)
{
	enum vmd_ret ret;

	VMD_ASSERT_PARAM(NOT_NULL(plos_cnt));

	ret = read_byte(rd, NRF24_REG_OBSERVE_TX, plos_cnt, 7, 4);
	if (ret != VMD_SUCC) {
		return ret;
	}

	if (*plos_cnt > 15) {
		return VMD_FAIL;
	}

	return ret;
}

//not get set value. Read realtime value!
enum vmd_ret nrf24_r_arc_cnt(struct nrf24 *rd, uint8_t *arc_cnt)
{
	enum vmd_ret ret = true;

	VMD_ASSERT_PARAM(NOT_NULL(arc_cnt));

	ret = read_byte(rd, NRF24_REG_OBSERVE_TX, arc_cnt, 3, 0);
	if (ret != VMD_SUCC) {
		return ret;
	}

	if (*arc_cnt > 15) {
		return VMD_FAIL;
	}

	return ret;
}

enum vmd_ret nrf24_set_rx_addr(struct nrf24 *rd, uint8_t pipe, uint8_t *rx_addr, uint8_t width)
{
	enum vmd_ret ret;
	uint8_t read_addr[5] = {0,};

	VMD_ASSERT_PARAM(width >= 3 && width <= 5);
	VMD_ASSERT_PARAM(pipe <= 5);

	if (pipe == 0 || pipe == 1) {
		return write_spi(rd, NRF24_REG_RX_ADDR_P0 + pipe, rx_addr, width, NULL);
	}

	ret = read_spi(rd, NRF24_REG_RX_ADDR_P1, read_addr, width, NULL);
	if (ret != VMD_SUCC) {
		return ret;
	}

	for (int i = 1; i < width; i++) {
		if (read_addr[i] != rx_addr[i]) {
			return VMD_FAIL;
		}
	}

	return write_spi(rd, NRF24_REG_RX_ADDR_P0 + pipe, rx_addr, 1, NULL);
}

enum vmd_ret nrf24_get_rx_addr(struct nrf24 *rd, uint8_t pipe, uint8_t *rx_addr, uint8_t width)
{
	enum vmd_ret ret;

	VMD_ASSERT_PARAM(width >= 3 && width <= 5);
	VMD_ASSERT_PARAM(pipe <= 5);

	if (pipe == 0) {
		return read_spi(rd, NRF24_REG_RX_ADDR_P0, rx_addr, width, NULL);
	}

	ret = read_spi(rd, NRF24_REG_RX_ADDR_P1, rx_addr, width, NULL);
	if (ret != VMD_SUCC) {
		return ret;
	}

	if (pipe != 1) {
		ret = read_spi(rd, NRF24_REG_RX_ADDR_P0 + pipe, rx_addr, 1, NULL);
	}

	return ret;
}

enum vmd_ret nrf24_set_tx_addr(struct nrf24 *rd, uint8_t *tx_addr, uint8_t width)
{
	VMD_ASSERT_PARAM(width >= 3 && width <= 5);
	return write_spi(rd, NRF24_REG_TX_ADDR, tx_addr, width, NULL);
}

enum vmd_ret nrf24_get_tx_addr(struct nrf24 *rd, uint8_t *tx_addr, uint8_t width)
{
	VMD_ASSERT_PARAM(width >= 3 && width <= 5);
	return read_spi(rd, NRF24_REG_TX_ADDR, tx_addr, width, NULL);
}

enum vmd_ret nrf24_set_rx_pld_width(struct nrf24 *rd, uint8_t pipe, uint8_t pld_width)
{
	VMD_ASSERT_PARAM(pld_width <= 32);
	VMD_ASSERT_PARAM(pipe <= 5);

	return write_byte(rd, NRF24_REG_RX_PW_P0 + pipe, pld_width, 5, 0);
}

enum vmd_ret nrf24_get_rx_pld_width(struct nrf24 *rd, uint8_t pipe, uint8_t *pld_width)
{
	enum vmd_ret ret;

	VMD_ASSERT_PARAM(pipe <= 5);

	ret = read_byte(rd, NRF24_REG_RX_PW_P0 + pipe, pld_width, 5, 0);
	if (ret != VMD_SUCC) {
		return ret;
	}

	if (*pld_width > 32) {
		ret = nrf24_flush_rx(rd);	//todo: check
		// if (ret != VMD_SUCC) {
		// 	return ret;
		// }
		return VMD_FAIL;
	}

	return ret;
}
//requirement: nrf24_en_auto_ack, nrf24_en_dpl(),
enum vmd_ret nrf24_set_dpl(struct nrf24 *rd, uint8_t pipe, bool en)
{
	VMD_ASSERT_PARAM(pipe <= 5);
	return write_bit(rd, NRF24_REG_DYNPD, en, pipe);
}

enum vmd_ret nrf24_get_dpl(struct nrf24 *rd, uint8_t pipe, bool *is_en)
{
	VMD_ASSERT_PARAM(pipe <= 5);
	return read_bit(rd, NRF24_REG_DYNPD, is_en, pipe);

}

enum vmd_ret nrf24_en_dpl(struct nrf24 *rd, bool en)
{
	return write_bit(rd, NRF24_REG_FEATURE, en, 2);
}

enum vmd_ret nrf24_isen_dpl(struct nrf24 *rd, bool *is_en)
{
	return read_bit(rd, NRF24_REG_FEATURE, is_en, 2);
}

//requirement : nrf24_set_dpl()
//specific pipe that dpl feature on.
enum vmd_ret nrf24_en_ack_pld(struct nrf24 *rd, bool en)
{
	return write_bit(rd, NRF24_REG_FEATURE, en, 1);
}
enum vmd_ret nrf24_isen_ack_pld(struct nrf24 *rd, bool *is_en)
{
	return read_bit(rd, NRF24_REG_FEATURE, is_en, 1);
}

enum vmd_ret nrf24_en_dyn_ack(struct nrf24 *rd, bool en)
{
	return write_bit(rd, NRF24_REG_FEATURE, en, 0);
}

enum vmd_ret nrf24_isen_dyn_ack(struct nrf24 *rd, bool *is_en)
{
	return read_bit(rd, NRF24_REG_FEATURE, is_en, 0);
}
