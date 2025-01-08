#include <errno.h>
#include "ads1299.h"
#include <stdlib.h>
#include <string.h>
#include "app_log.h"

#define REG_CHxSET_ADDR(idx) ((idx) + 0x05)

const static uint8_t reg_name[REG_MAX][20] = {
	{"ID"},
	{"CONFIG1"},
	{"CONFIG2"},
	{"CONFIG3"},
	{"LOFF"},
	{"CH1SET"},
	{"CH2SET"},
	{"CH3SET"},
	{"CH4SET"},
	{"CH5SET"},
	{"CH6SET"},
	{"CH7SET"},
	{"CH8SET"},
	{"BAIS_SENSP"},
	{"BAIS_SENSN"},
	{"LOFF_SENSP"},
	{"LOFF_SENSN"},
	{"LOFF_FLIP"},
	{"LOFF_STATP"},
	{"LOFF_STATN"},
	{"GPIO"},
	{"MISC1"},
	{"MISC2"},
	{"CONFIG4"},
};

static int ads1299_write_cmd(uint8_t command)
{
	int ret = 0;
	uint8_t rd_value;

	ret = ads1299_hw_spi_transmit_receive(&command, &rd_value, 1);

	return ret;
}

static int ads1299_write_reg(uint8_t reg, uint8_t rval)
{
	int ret = 0;
	uint8_t write_buf[3] = {CMD_WREG | reg, 0x0, rval};
	uint8_t read_buf[3] = {0, 0, 0};

	ret = ads1299_hw_spi_transmit_receive(write_buf, read_buf, 3);
	ads1299_delay_us(100);
	if (ret != 0)
	{
		APP_LOG_INFO("afe read reg failed, %d\r\n", ret);
		return ret;
	}

	return 0;
}

static int ads1299_read_reg(uint8_t reg, uint8_t *rval)
{
	int ret = 0;
	uint8_t write_buf[3] = {CMD_RREG | reg, 0x0, 0x0};
	uint8_t read_buf[3] = {0, 0, 0};

	ret = ads1299_hw_spi_transmit_receive(write_buf, read_buf, 3);
	if (ret != 0)
	{
		APP_LOG_INFO("afe read reg failed, %d\r\n", ret);
		return ret;
	}

	*rval = read_buf[2];

	return 0;
}

static void ads1299_device_reset(void)
{
	ads1299_delay_ms(200);
#ifdef ADS1299_RESET_PIN_ENABLE
	ads1299_hw_reset_pin_set(0);
	ads1299_delay_ms(20);
	ads1299_hw_reset_pin_set(1);
	ads1299_delay_ms(500);
#else
	ads1299_write_cmd(CMD_RESET);
	ads1299_delay_ms(500);
#endif
}

void default_reg_set(void)
{
	ads1299_write_cmd(CMD_STOP);
	ads1299_delay_ms(1);
	ads1299_write_cmd(CMD_SDATAC);
	ads1299_delay_ms(1);

	ads1299_write_reg(REG_CONFIG1, CONFIG1_const | HIGH_RES_4k_SPS);
	ads1299_write_reg(REG_CONFIG3, CONFIG3_const | REF_BUF_EN | BIAS_REF_INTERNAL | BIAS_BUF_EN | INT_TEST_NONE);
	ads1299_write_reg(REG_LOFF, LOFF_const | COMP_TH_90 | ILEAD_OFF_6nA | FLEAD_OFF_DC);

#if (CONFIG_ADC_ADS1299_TEST_SIGNAL == 1)
	ads1299_write_reg(REG_CONFIG2, CONFIG2_const | INT_TEST_4HZ);
	ads1299_write_reg(REG_CH1SET, CHnSET_const | GAIN_6X | TEST_SIGNAL);
	ads1299_write_reg(REG_CH2SET, CHnSET_const | GAIN_6X | TEST_SIGNAL);
	ads1299_write_reg(REG_CH3SET, CHnSET_const | GAIN_6X | TEST_SIGNAL);
	ads1299_write_reg(REG_CH4SET, CHnSET_const | GAIN_6X | TEST_SIGNAL);
	ads1299_write_reg(REG_CH5SET, CHnSET_const | GAIN_6X | TEST_SIGNAL);
	ads1299_write_reg(REG_CH6SET, CHnSET_const | GAIN_6X | TEST_SIGNAL);
	ads1299_write_reg(REG_CH7SET, CHnSET_const | GAIN_6X | TEST_SIGNAL);
	ads1299_write_reg(REG_CH8SET, CHnSET_const | GAIN_6X | TEST_SIGNAL);
#else
	ads1299_write_reg(REG_CONFIG2, CONFIG2_const | INT_TEST_DC);
	ads1299_write_reg(REG_CH1SET, CHnSET_const | GAIN_6X | ELECTRODE_INPUT);
	ads1299_write_reg(REG_CH2SET, CHnSET_const | GAIN_6X | ELECTRODE_INPUT);
	ads1299_write_reg(REG_CH3SET, CHnSET_const | GAIN_6X | ELECTRODE_INPUT);
	ads1299_write_reg(REG_CH4SET, CHnSET_const | GAIN_6X | ELECTRODE_INPUT);
	ads1299_write_reg(REG_CH5SET, CHnSET_const | GAIN_6X | ELECTRODE_INPUT);
	ads1299_write_reg(REG_CH6SET, CHnSET_const | GAIN_6X | ELECTRODE_INPUT);
	ads1299_write_reg(REG_CH7SET, CHnSET_const | GAIN_6X | ELECTRODE_INPUT);
	ads1299_write_reg(REG_CH8SET, CHnSET_const | GAIN_6X | ELECTRODE_INPUT);
#endif
	ads1299_write_reg(REG_BIAS_SENSP, BIAS_SENSP_const | BIAS1P);
	ads1299_write_reg(REG_BIAS_SENSN, BIAS_SENSN_const | BIAS1N);
	ads1299_write_reg(REG_LOFF_SENSP, LOFF_SENSP_const);
	ads1299_write_reg(REG_LOFF_SENSN, LOFF_SENSN_const);
	ads1299_write_reg(REG_LOFF_FLIP, LOFF_FLIP_const);

	ads1299_write_reg(REG_LOFF_STATP, 0);
	ads1299_write_reg(REG_LOFF_STATN, 0);
	ads1299_write_reg(REG_GPIO, GPIO_const);
	ads1299_write_reg(REG_MISC1, MISC1_const);
	ads1299_write_reg(REG_MISC2, MISC2_const);

	ads1299_write_reg(REG_CONFIG4, CONFIG4_const | PD_LOFF_COMP);
}

static void ads1299_all_reg_read(void)
{
	uint8_t reg;

	for (int i = 1; i < REG_MAX; i++)
	{
		ads1299_read_reg(i, &reg);
		APP_LOG_INFO("%s %02x", reg_name[i], reg);
	}
}

int ads1299_init(void)
{
	uint8_t id = 0;
	int ret = 0;

	ads1299_hw_init();
	ads1299_device_reset();

	ads1299_write_cmd(CMD_SDATAC);
	ads1299_write_cmd(CMD_SDATAC);
	ret = ads1299_read_reg(REG_ID, &id);
	if (ret)
	{
		APP_LOG_INFO("ads1299 read id err!please check it!\n");
		return ret;
	}

	APP_LOG_DEBUG("ads1299 id %02x\r\n", id);

	ads1299_write_cmd(CMD_SDATAC);
	ads1299_all_reg_read();
	ads1299_delay_ms(10);
	default_reg_set();
	ads1299_delay_ms(10);
	ads1299_all_reg_read();
ads1299_delay_ms(1);
	ads1299_hw_spi_high_speed();

	ads1299_write_cmd(CMD_START);
	
	ads1299_delay_ms(1);
	ads1299_write_cmd(CMD_START);
	ads1299_delay_ms(1);
	ads1299_write_cmd(CMD_RDATAC);

    

	return 0;
}

/* 用于读取数据 */
static uint8_t spi_tx_dumy[ADS1299_READ_SAMPLE_BYTES];

int ads1299_read_samples_data(uint8_t *data, uint8_t len)
{
	int ret = 0;

	ret = eeg_afe_hw_spi_dma_receive(spi_tx_dumy, data, len);
	// ret = ads1299_hw_spi_transmit_receive(spi_tx_dumy, data, len);
	if (ret != 0)
	{
		APP_LOG_INFO("afe read reg failed, %d\r\n", ret);
	}

	return ret;
}