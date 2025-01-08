#include "ads1299/ads1299.h"

#include "app_log.h"
#include <app_io.h>
#include "board_bc.h"

static uint8_t rd_eeg_buf[ADS1299_READ_SAMPLE_BYTES];
#define EEG_READ_DEBUG 1
#define EEG_READ_IN_ISR 0

void eeg_read_debug_pin_init(void)
{
#if (EEG_READ_DEBUG)
    app_drv_err_t ret = 0;
    app_io_init_t io_init = APP_IO_DEFAULT_CONFIG;
    app_io_pin_state_t pin_val;

    io_init.pull = APP_IO_NOPULL;
    io_init.mode = APP_IO_MODE_OUTPUT;
    io_init.pin = AFE_DEBUG_PIN;
    io_init.mux = APP_IO_MUX;
    ret = app_io_init(AFE_DEBUG_TYPE, &io_init);
    if (ret != APP_DRV_SUCCESS)
    {
        APP_LOG_INFO("AFE_START_PIN init failed.\r\n");
    }
    app_io_write_pin(AFE_DEBUG_TYPE, AFE_DEBUG_PIN, APP_IO_PIN_RESET);
#endif
}
void eeg_read_debug_pin_set(int value)
{
#if (EEG_READ_DEBUG)
    app_io_write_pin(AFE_DEBUG_TYPE, AFE_DEBUG_PIN, value ? APP_IO_PIN_SET : APP_IO_PIN_RESET);
#endif
}

volatile bool start_read_in_isr = false;
volatile bool eeg_drdy = false;
void eeg_drdy_cb(void)
{

#if (EEG_READ_IN_ISR)
    if (!start_read_in_isr)
    {
        return;
    }
    eeg_read_debug_pin_set(APP_IO_PIN_SET);
    ads1299_read_samples_data(rd_eeg_buf, ADS1299_READ_SAMPLE_BYTES);
    eeg_read_debug_pin_set(APP_IO_PIN_RESET);

    return;
#else
    eeg_drdy = true;
#endif
}

void eeg_reader_task(void *arg)
{
    static uint32_t count = 0;
    uint8_t eeg_buf[ADS1299_READ_SAMPLE_BYTES];

    if (eeg_drdy)
    {
        eeg_read_debug_pin_set(APP_IO_PIN_SET);
        ads1299_read_samples_data(rd_eeg_buf, ADS1299_READ_SAMPLE_BYTES);
        eeg_read_debug_pin_set(APP_IO_PIN_RESET);
        eeg_drdy = false;
    } else {
        if (count++ > 100000)
        {
            count = 0;
            ads1299_read_samples_data(rd_eeg_buf, ADS1299_READ_SAMPLE_BYTES);
        }
    }
}

void eeg_reader_init(void)
{
    ads1299_init();
    ads1299_drdy_cb_register(eeg_drdy_cb);
    ads1299_read_samples_data(rd_eeg_buf, ADS1299_READ_SAMPLE_BYTES);
    eeg_read_debug_pin_init();
}
