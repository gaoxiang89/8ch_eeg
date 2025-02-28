#include "ads1299/ads1299.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "app_log.h"
#include <app_io.h>
#include "board_bc.h"

static uint8_t rd_eeg_buf[ADS1299_READ_SAMPLE_BYTES];
static SemaphoreHandle_t eeg_drdy_sem;
#define EEG_READ_DEBUG 1
#define EEG_READ_IN_ISR 0

#if (EEG_READ_IN_ISR)
static QueueHandle_t xEEGQueue;
#endif

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
void eeg_drdy_cb(void)
{
    BaseType_t ret = pdTRUE;
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

#if (EEG_READ_IN_ISR)
    if (!start_read_in_isr) {
        return;
    }
    eeg_read_debug_pin_set(APP_IO_PIN_SET);
    ads1299_read_samples_data(rd_eeg_buf, ADS1299_READ_SAMPLE_BYTES);
    eeg_read_debug_pin_set(APP_IO_PIN_RESET);

    xQueueSendFromISR(xEEGQueue, &rd_eeg_buf, &xHigherPriorityTaskWoken);
	portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
	return;
#else
    if (eeg_drdy_sem)
    {
        ret = xSemaphoreGiveFromISR(eeg_drdy_sem, &xHigherPriorityTaskWoken);
        if (xHigherPriorityTaskWoken != pdFALSE)
        {
            portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
        }
    }
#endif
}

void eeg_reader_task(void *arg)
{
    int count = 0;
    uint8_t eeg_buf[ADS1299_READ_SAMPLE_BYTES];

#if (EEG_READ_IN_ISR)
	xEEGQueue = xQueueCreate(20, ADS1299_READ_SAMPLE_BYTES);
#endif
    ads1299_init();
    ads1299_drdy_cb_register(eeg_drdy_cb);
    ads1299_read_samples_data(rd_eeg_buf, ADS1299_READ_SAMPLE_BYTES);
    eeg_read_debug_pin_init();
    start_read_in_isr = true;
    APP_LOG_INFO("eeg reader init\r\n");
    while (1)
    {
#if (EEG_READ_IN_ISR)
		if (pdTRUE != xQueueReceive(xEEGQueue, &eeg_buf, pdMS_TO_TICKS(100))) {
			ads1299_read_samples_data(eeg_buf, ADS1299_READ_SAMPLE_BYTES);
		}
#else
        xSemaphoreTake(eeg_drdy_sem, pdMS_TO_TICKS(5000));
        eeg_read_debug_pin_set(APP_IO_PIN_SET);
        ads1299_read_samples_data(rd_eeg_buf, ADS1299_READ_SAMPLE_BYTES);
        eeg_read_debug_pin_set(APP_IO_PIN_RESET);
#endif
        if (count++ >= 250)
        {
            // APP_LOG_INFO("eeg reader, count: %d\r\n", count);
            count = 0;
        }
    }

    vTaskDelete(NULL);
}

void eeg_reader_init(void)
{
    xTaskCreate(eeg_reader_task, "eeg_reader_task", 1024, NULL, 5, NULL);
    eeg_drdy_sem = xSemaphoreCreateBinary();
}
