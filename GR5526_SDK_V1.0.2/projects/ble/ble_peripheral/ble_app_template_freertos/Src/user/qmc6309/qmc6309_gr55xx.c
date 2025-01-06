

#include "app_log.h"
#include "app_i2c.h"
#include "board_bc.h"
#include "FreeRTOS.h"
#include "task.h"
#include "qmc6309.h"


static app_i2c_params_t master_params = {
    .id = APP_I2C_3_ID,
    .role = APP_I2C_ROLE_MASTER,
    .pin_cfg = {
        .scl = {
            .type = APP_I2C_3_SCL_IO_TYPE,
            .mux = APP_I2C_3_SCL_PINMUX,
            .pin = APP_I2C_3_SCL_PIN,
            .pull = APP_IO_PULLUP,
        },
        .sda = {
            .type = APP_I2C_3_SDA_IO_TYPE,
            .mux = APP_I2C_3_SDA_PINMUX,
            .pin = APP_I2C_3_SDA_PIN,
            .pull = APP_IO_PULLUP,
        },
    },
    .dma_cfg = {
        .tx_dma_instance = DMA0,
        .rx_dma_instance = DMA0,
        .tx_dma_channel = DMA_Channel2,
        .rx_dma_channel = DMA_Channel3,
    },
    .init = {
        .speed = I2C_SPEED_100K,
        .own_address = 0x00,
        .addressing_mode = I2C_ADDRESSINGMODE_7BIT,
        .general_call_mode = I2C_GENERALCALL_DISABLE,
    },
};

/*
 * GLOBAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */
static volatile uint8_t g_master_tdone = 0;
static volatile uint8_t g_master_rdone = 0;

/*
 * LOCAL FUNCTION DEFINITIONS
 *****************************************************************************************
 */
static void APP_I2C_3_evt_handler(app_i2c_evt_t *p_evt)
{
    switch (p_evt->type)
    {
    case APP_I2C_EVT_ERROR:
        g_master_tdone = 1;
        g_master_rdone = 1;
        break;

    case APP_I2C_EVT_TX_CPLT:
        g_master_tdone = 1;
        break;

    case APP_I2C_EVT_RX_DATA:
        g_master_rdone = 1;
        break;
    }
}

void qmc6309_i2c_scan(void)
{
    uint8_t address;
    uint16_t ret;
    uint8_t dummy = 0;

    printf("Scanning I2C3 bus...\r\n");
    for (address = 0; address < 128; address++)
    {
        ret = app_i2c_transmit_sync(APP_I2C_3_ID, address, &dummy, 1, 1000);
        if (ret == APP_DRV_SUCCESS)
        {
            printf("I2C3 device found at address 0x%02X\r\n", address);
        }

        vTaskDelay(100);
    }
    printf("I2C3 bus scan completed.\r\n");
}

int qmc6309_hw_init(void)
{
    uint16_t ret = app_i2c_init(&master_params, APP_I2C_3_evt_handler);
    if (ret != 0)
    {
        printf("qmc6309 I2C master initial failed! Please check the input parameters.\r\n");
    }
    else
    {
        printf("qmc6309 I2C master initialized successfully.\r\n");
    }

    return ret;
}

int qmc6309_i2c_write(uint16_t address, uint8_t *p_data, uint16_t size)
{
    int ret = 0;

    ret = app_i2c_transmit_sync(APP_I2C_3_ID, address, p_data, size, 1000);

    return ret;
}

int qmc6309_i2c_read(uint16_t address, uint8_t *p_data, uint16_t size)
{
    int ret = 0;

    ret = app_i2c_receive_sync(APP_I2C_3_ID, address, p_data, size, 1000);

    return ret;
}

void qmc6309_delay_ms(unsigned int ms)
{
    vTaskDelay(pdMS_TO_TICKS(ms));
}