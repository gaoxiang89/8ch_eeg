#include "eeg_afe.h"

#include "FreeRTOS.h"
#include "task.h"

#include "board.h"
#include <app_spi.h>
#include <app_spi_dma.h>
#include <app_io.h>
#include "app_pwr_mgmt.h"
#include "app_log.h"

#define SPI_CLOCK_PRESCALER		64u /* The SPI CLOCK Freq = Peripheral CLK/SPI_CLOCK_PRESCALER, 500KHz */
#define SPI_SOFT_CS_MODE_ENABLE 1u	 /* suggest to enable SOFT CS MODE */

/* master spi parameters */
app_spi_params_t spi_params = {
    .id = APP_SPI_ID_MASTER,
    .pin_cfg = {
       .cs = {
           .type   = APP_SPIM_CS_IO_TYPE,
           .mux    = APP_SPIM_CS_PINMUX,
           .pin    = APP_SPIM_CS_PIN,
           .mode   = APP_IO_MODE_MUX,
           .pull   = APP_IO_PULLUP,
           .enable = APP_SPI_PIN_ENABLE,
       },
       .clk  = {
           .type   = APP_SPIM_CLK_IO_TYPE,
           .mux    = APP_SPIM_CLK_PINMUX,
           .pin    = APP_SPIM_CLK_PIN,
           .mode   = APP_IO_MODE_MUX,
           .pull   = APP_IO_PULLUP,
           .enable = APP_SPI_PIN_ENABLE,
       },
       .mosi = {
           .type   = APP_SPIM_MOSI_IO_TYPE,
           .mux    = APP_SPIM_MOSI_PINMUX,
           .pin    = APP_SPIM_MOSI_PIN,
           .mode   = APP_IO_MODE_MUX,
           .pull   = APP_IO_PULLUP,
           .enable = APP_SPI_PIN_ENABLE,
       },
       .miso = {
           .type   = APP_SPIM_MISO_IO_TYPE,
           .mux    = APP_SPIM_MISO_PINMUX,
           .pin    = APP_SPIM_MISO_PIN,
           .mode   = APP_IO_MODE_MUX,
           .pull   = APP_IO_PULLUP,
           .enable = APP_SPI_PIN_ENABLE,
       },
    },
    .dma_cfg = {
        .tx_dma_instance = DMA0,
        .rx_dma_instance = DMA0,
        .tx_dma_channel  = DMA_Channel2,
        .rx_dma_channel  = DMA_Channel3,
    },
    .init = {
        .data_size      = SPI_DATASIZE_8BIT,
        .clock_polarity = SPI_POLARITY_LOW,
        .clock_phase    = SPI_PHASE_2EDGE,
        .baudrate_prescaler = SPI_CLOCK_PRESCALER,
        .ti_mode        = SPI_TIMODE_DISABLE,
        .slave_select   = SPI_SLAVE_SELECT_0,
    },
 
    .is_soft_cs = SPI_SOFT_CS_MODE_ENABLE,
};

static eeg_afe_drdy_cb_t drdy_cb = NULL;
static uint8_t eeg_buf[EEG_AFE_DATA_LENGTH];

volatile uint8_t g_master_tdone = 0;
volatile uint8_t g_master_rdone = 0;
volatile uint8_t g_master_trdone = 0;

static void app_spi_master_callback(app_spi_evt_t *p_evt)
{
	if (p_evt->type == APP_SPI_EVT_TX_CPLT) {
		g_master_tdone = 1;
	}
	if (p_evt->type == APP_SPI_EVT_RX_CPLT) {
		g_master_rdone = 1;
	}
	if (p_evt->type == APP_SPI_EVT_TX_RX_CPLT) {
		g_master_trdone = 1;
	}
	if (p_evt->type == APP_SPI_EVT_ERROR) {
		g_master_tdone = 1;
		g_master_rdone = 1;
	}
}

static void hw_gpio_init(void)
{
	app_drv_err_t ret = 0;
	app_io_init_t io_init = APP_IO_DEFAULT_CONFIG;
	app_io_pin_state_t pin_val;

#ifdef AFE_RESET_PIN
	io_init.pull = APP_IO_NOPULL;
	io_init.mode = APP_IO_MODE_OUTPUT;
	io_init.pin = AFE_RESET_PIN;
	io_init.mux = APP_IO_MUX;
	ret = app_io_init(APP_IO_TYPE_NORMAL, &io_init);
	if (ret != APP_DRV_SUCCESS) {
		APP_LOG_INFO("AFE_RESET_PIN init failed.\r\n");
	}
	app_io_write_pin(AFE_RESET_TYPE, AFE_RESET_PIN, APP_IO_PIN_SET);
#endif

#ifdef AFE_START_PIN
	io_init.pull = APP_IO_NOPULL;
	io_init.mode = APP_IO_MODE_OUTPUT;
	io_init.pin = AFE_START_PIN;
	io_init.mux = APP_IO_MUX;
	ret = app_io_init(APP_IO_TYPE_NORMAL, &io_init);
	if (ret != APP_DRV_SUCCESS) {
		APP_LOG_INFO("AFE_START_PIN init failed.\r\n");
	}
	app_io_write_pin(AFE_START_TYPE, AFE_START_PIN, APP_IO_PIN_RESET);
#endif

	ret = app_spi_init(&spi_params, app_spi_master_callback);
	if (ret != 0) {
		APP_LOG_INFO("SPI master initial failed! Please check the input paraments.\r\n");
	}

	ret = app_spi_dma_init(&spi_params);
	if (ret != 0) {
		APP_LOG_INFO("SPI master dma initial failed! Please check the input paraments.\r\n");
	}

	APP_LOG_DEBUG("SPI master inital success!\r\n");
}

void drdy_callback(app_io_evt_t *p_evt)
{
	if (p_evt->pin == AFE_DRDY_PIN) {
		if (drdy_cb) {
			drdy_cb();
		}
	}
}

static void drdy_gpio_init(void)
{
	app_drv_err_t ret = 0;
	app_io_init_t io_init = APP_IO_DEFAULT_CONFIG;
	app_io_pin_state_t pin_val;

	io_init.pull = APP_IO_PULLUP;
	io_init.mode = APP_IO_MODE_IT_FALLING;
	io_init.pin = AFE_DRDY_PIN;
	io_init.mux = APP_IO_MUX;

	ret = app_io_event_register_cb(AFE_DRDY_TYPE, &io_init, drdy_callback, NULL);
	if (ret != APP_DRV_SUCCESS) {
		APP_LOG_INFO("AFE_DRDY_PIN init failed.\r\n");
	}

    // NVIC_SetPriority(EXT0_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 0, 15));
}

void eeg_afe_hw_init(void)
{
	hw_gpio_init();
	drdy_gpio_init();
	eeg_afe_delay_ms(100);
}

void eeg_afe_hw_deinit(void)
{
	app_spi_deinit(spi_params.id);
	app_spi_dma_deinit(spi_params.id);
}

void eeg_afe_hw_reset_pin_set(int value)
{
#ifdef AFE_RESET_PIN
	if (value == 0) {
		app_io_write_pin(AFE_RESET_TYPE, AFE_RESET_PIN, APP_IO_PIN_RESET);
	} else {
		app_io_write_pin(AFE_RESET_TYPE, AFE_RESET_PIN, APP_IO_PIN_SET);
	}
#endif
}

void eeg_afe_delay_ms(uint32_t ms)
{
	vTaskDelay(ms);
}

int eeg_afe_hw_spi_transmit_receive(uint8_t *p_tx_data, uint8_t *p_rx_data, uint32_t size)
{
	app_drv_err_t ret = 0;

	ret = app_spi_transmit_receive_sync(APP_SPI_ID_MASTER, p_tx_data, p_rx_data, size, 1000);
	if (ret != APP_DRV_SUCCESS) {
		APP_LOG_INFO("eeg afe spi transmit receive faild, ret %d\r\n", ret);
	}

	return ret;
}

void eeg_afe_drdy_cb_register(eeg_afe_drdy_cb_t cb)
{
	drdy_cb = cb;
}