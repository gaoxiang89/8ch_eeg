#ifndef ADS1299_H_
#define ADS1299_H_
#ifdef __cplusplus
extern "C"
{
#endif

#include "ads1299_registers.h"
#include <stdint.h>

#define ADS1299_SAMPLE_BYTES 3                                                        // 每个通道的数据大小（字节数），对应24位
#define ADS1299_NUM_CHANNELS 8                                                        // 最大通道数
#define ADS1299_READ_SAMPLE_BYTES ((1 + ADS1299_NUM_CHANNELS) * ADS1299_SAMPLE_BYTES) // 一次采样的总字节数

#define ADS1299_NORMAL_MODE 0
#define ADS1299_TEST_MODE 1

#define ADS1299_LOFF_NO_SET 0
#define ADS1299_LOFF_OFF 1
#define ADS1299_LOFF_DC 2
#define ADS1299_LOFF_AC_7P8 3
#define ADS1299_LOFF_AC_31P2 4
#define ADS1299_LOFF_AC_Fdr4 5

#define ADS1299_INPUT_NORMAL 1
#define ADS1299_INPUT_SHORT 2
#define ADS1299_INPUT_TEST 3

#define ADS1299_RESET_PIN_ENABLE 1

    typedef void (*ads1299_drdy_cb_t)(void);
    int ads1299_init(void);

    void ads1299_hw_init(void);

    void ads1299_hw_deinit(void);

    int ads1299_hw_spi_transmit_receive(uint8_t *p_tx_data, uint8_t *p_rx_data, uint32_t size);

    void ads1299_drdy_cb_register(ads1299_drdy_cb_t cb);

    int ads1299_read_samples_data(uint8_t *data, uint8_t len);

    void ads1299_hw_reset_pin_set(int value);

    void ads1299_delay_ms(uint32_t ms);

#ifdef __cplusplus
}
#endif
#endif /* ADS1299_H_ */
