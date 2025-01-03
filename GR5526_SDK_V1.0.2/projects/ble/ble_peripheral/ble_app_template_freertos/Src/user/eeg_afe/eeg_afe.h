#ifndef EEG_AFE_H_
#define EEG_AFE_H_

#include <stdint.h>
#include <stdbool.h>

/* support printf for gr5515 */
#include "board.h"

#define EEG_AFE_CMD_WAKEUP	0x02
#define EEG_AFE_CMD_STANDBY 0x04
#define EEG_AFE_CMD_RESET	0x06
#define EEG_AFE_CMD_START	0x08
#define EEG_AFE_CMD_STOP	0x0A
#define EEG_AFE_CMD_RDATAC	0x10
#define EEG_AFE_CMD_SDATAC	0x11
#define EEG_AFE_CMD_RDATA	0x12
#define EEG_AFE_CMD_RREG	0x20
#define EEG_AFE_CMD_WREG	0x40

#define EEG_REG_ID_ADDR			 0x00
#define EEG_REG_CONFIG1_ADDR	 0x01
#define EEG_REG_CONFIG2_ADDR	 0x02
#define EEG_REG_LOFF_ADDR		 0x03
#define EEG_REG_CH1SET_ADDR		 0x04
#define EEG_REG_CH2SET_ADDR		 0x05
#define EEG_REG_RLD_SENS_ADDR	 0x06
#define EEG_REG_LOFF_SENS_ADDR	 0x07
#define EEG_REG_LOFF_STAT_ADDR	 0x08
#define EEG_REG_RESP1_ADDR		 0x09
#define EEG_REG_RESP2_ADDR		 0x0a
#define EEG_REG_GPIO_ADDR		 0x0b
#define EEG_REG_CHxSET_ADDR(idx) ((idx) + EEG_REG_CH1SET_ADDR)

#define EEG_AFE_ADC_WIDTH	 3
#define EEG_AFE_MAX_CHANNELS (1 + 1) // 1channel status, n channel eeg, n = 1 or 2
#define EEG_AFE_DATA_LENGTH	 (EEG_AFE_MAX_CHANNELS * EEG_AFE_ADC_WIDTH)

#define EEG_AFE_RESET_PIN_ENABLE 1

#define LEAD_OFF_SENSE_NORMAL 1
#define LEAD_OFF_SENSE_RLD    2
#define LEAD_OFF_SENSE_OFF    3

typedef enum {
	UNKNOW,
	ADS1292 = 0x53,
	ADX920 = 0xf2,
	ADX921 = 0xf3,
} ads1x9x_id_t;

typedef void (*eeg_afe_drdy_cb_t)(void);

void eeg_afe_hw_init(void);

void eeg_afe_hw_deinit(void);

void eeg_afe_hw_reset_pin_set(int value);

void eeg_afe_delay_ms(uint32_t ms);

int eeg_afe_hw_spi_transmit_receive(uint8_t *p_tx_data, uint8_t *p_rx_data, uint32_t size);

int eeg_afe_init(void);

void eeg_afe_deinit(void);

int eeg_afe_read_samples_data(uint8_t *data, uint8_t len);

void eeg_afe_drdy_cb_register(eeg_afe_drdy_cb_t cb);

void eeg_afe_lead_off_sense_set(int all_pull_up);

bool eeg_afe_is_initialized(void);

#endif /* EEG_AFE_H_ */
