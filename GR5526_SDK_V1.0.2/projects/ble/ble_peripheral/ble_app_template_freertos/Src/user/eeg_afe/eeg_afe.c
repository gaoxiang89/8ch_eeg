#include "eeg_afe.h"
#include <stdlib.h>
#include <string.h>
#include "app_log.h"

#define TEST_SIGNAL 0

static bool m_is_initialized;

static void afe_device_reset(void);
static int afe_write_cmd(uint8_t command);
static int afe_write_reg(uint8_t reg, uint8_t rval);
static int afe_read_reg(uint8_t reg, uint8_t *rval);

/* 用于读取数据 */
static uint8_t spi_tx_dumy[EEG_AFE_DATA_LENGTH];

int afe_write_cmd(uint8_t command)
{
	int ret = 0;
	uint8_t rd_value;

	ret = eeg_afe_hw_spi_transmit_receive(&command, &rd_value, 1);

	return ret;
}

int afe_write_reg(uint8_t reg, uint8_t rval)
{
	int ret = 0;
	uint8_t write_buf[3] = { EEG_AFE_CMD_WREG | reg, 0x0, rval };
	uint8_t read_buf[3] = { 0 };

	ret = eeg_afe_hw_spi_transmit_receive(write_buf, read_buf, 3);
	if (ret != 0) {
		APP_LOG_INFO("afe write reg failed, %d\r\n", ret);
		return ret;
	}

	return ret;
}

int afe_read_reg(uint8_t reg, uint8_t *rval)
{
	int ret = 0;
	uint8_t write_buf[3] = { EEG_AFE_CMD_RREG | reg, 0x0 };
	uint8_t read_buf[3] = { 0, 0, 0 };

	ret = eeg_afe_hw_spi_transmit_receive(write_buf, read_buf, 3);
	if (ret != 0) {
		APP_LOG_INFO("afe read reg failed, %d\r\n", ret);
		return ret;
	}

	APP_LOG_DEBUG("reg %02x, value %02x\r\n", reg, read_buf[2]);
	*rval = read_buf[2];

	return 0;
}

int eeg_afe_read_samples_data(uint8_t *rval, uint8_t len)
{
	int ret = 0;

	ret = eeg_afe_hw_spi_transmit_receive(spi_tx_dumy, rval, len);
	if (ret != 0) {
		APP_LOG_INFO("afe read reg failed, %d\r\n", ret);
		return ret;
	}

	return ret;
}

static void afe_device_reset(void)
{
#ifdef EEG_AFE_RESET_PIN_ENABLE
	eeg_afe_hw_reset_pin_set(0);
	eeg_afe_delay_ms(10);
	eeg_afe_hw_reset_pin_set(1);
	eeg_afe_delay_ms(100);
#else
	afe_write_cmd(EEG_AFE_CMD_RESET);
#endif
}

int eeg_afe_init(void)
{
	uint8_t id = 0;
	int ret = 0;

	eeg_afe_hw_init();
	afe_device_reset();

	afe_write_cmd(EEG_AFE_CMD_SDATAC);
	ret = afe_read_reg(EEG_REG_ID_ADDR, &id);
	if (ret) {
		APP_LOG_INFO("ads1292 read id err!please check it!\n");
		m_is_initialized = false;
		return ret;
	}
	m_is_initialized = true;

	APP_LOG_DEBUG("ads1292 id %02x\r\n", id);

	afe_write_cmd(EEG_AFE_CMD_SDATAC);
	afe_write_reg(EEG_REG_CONFIG1_ADDR, 0x01);
#if (TEST_SIGNAL)	
	afe_write_reg(EEG_REG_CONFIG2_ADDR, 0xE3);
#else
	afe_write_reg(EEG_REG_CONFIG2_ADDR, 0xE0);
#endif
	afe_write_reg(EEG_REG_LOFF_ADDR, 0xA0);
#if (TEST_SIGNAL)	
	afe_write_reg(EEG_REG_CH1SET_ADDR, 0x05);
	afe_write_reg(EEG_REG_CH2SET_ADDR, 0x05);
#else
	afe_write_reg(EEG_REG_CH1SET_ADDR, 0x00);
	afe_write_reg(EEG_REG_CH2SET_ADDR, 0x00);
#endif
	afe_write_reg(EEG_REG_RLD_SENS_ADDR, 0x23);
	afe_write_reg(EEG_REG_LOFF_SENS_ADDR, 0x13);
	afe_write_reg(EEG_REG_RESP1_ADDR, 0x02);
	afe_write_reg(EEG_REG_RESP2_ADDR, 0x03);
	if (id >= ADX920) {
		afe_write_reg(0x0f, 0x40 | 0x03); // ISET enable, N * 2.2nA
	}

	afe_write_cmd(EEG_AFE_CMD_SDATAC);
	for (int i = 0; i < 0x1f; i++) {
		ret = afe_read_reg(i, &id);
	}

	afe_write_cmd(EEG_AFE_CMD_RDATAC);
	eeg_afe_delay_ms(1);
	afe_write_cmd(EEG_AFE_CMD_START);
	eeg_afe_delay_ms(1);

	return 0;
}

void eeg_afe_lead_off_sense_set(int all_pull_up)
{
	if (all_pull_up) {
		afe_write_cmd(EEG_AFE_CMD_SDATAC);
		afe_write_reg(0x0f, 0x40 | 0x03); // ISET enable, 3 * 2.2nA
		afe_write_reg(EEG_REG_LOFF_SENS_ADDR, 0x15);
		afe_write_cmd(EEG_AFE_CMD_RDATAC);
	} else if (all_pull_up == LEAD_OFF_SENSE_OFF) {
		afe_write_cmd(EEG_AFE_CMD_SDATAC);
		afe_write_reg(0x0f, 0x40 | 0x03); // ISET enable, 3 * 2.2nA
		afe_write_reg(EEG_REG_LOFF_SENS_ADDR, 0x13);
		afe_write_cmd(EEG_AFE_CMD_RDATAC);
	} else {
		afe_write_cmd(EEG_AFE_CMD_SDATAC);
		afe_write_reg(0x0f, 0x40 | 0x03); // ISET enable, 3 * 2.2nA
		afe_write_reg(EEG_REG_LOFF_SENS_ADDR, 0x13);
		afe_write_cmd(EEG_AFE_CMD_RDATAC);		
	}


	
}

/* 发送1064个clk进入低功耗 */
static uint8_t dummy[1068];
void eeg_afe_deinit(void)
{
	afe_write_cmd(EEG_AFE_CMD_STOP);
	afe_write_reg(EEG_REG_CONFIG1_ADDR, 0x01);
	afe_write_reg(EEG_REG_CONFIG2_ADDR, 0x00);
	afe_write_reg(EEG_REG_RLD_SENS_ADDR, 0x00);
	afe_write_cmd(EEG_AFE_CMD_STANDBY);

	// 拉低power down 和 选择外部clk
	eeg_afe_hw_reset_pin_set(0);

	eeg_afe_hw_spi_transmit_receive(dummy, dummy, 1068);

	eeg_afe_hw_deinit();

	m_is_initialized = false;
}

bool eeg_afe_is_initialized(void)
{
	return m_is_initialized;
}