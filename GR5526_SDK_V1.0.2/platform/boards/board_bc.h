#ifndef __BOARD_SK_H__
#define __BOARD_SK_H__

#include "app_key.h"
#include "app_uart.h"

/*******HCI UART IO CONFIG***********************/
#define APP_HCI_UART_ID                     APP_UART_ID_3
#define APP_HCI_UART_FLOW_ON                0
#define APP_HCI_UART_BAUDRATE               115200
#define APP_HCI_UART_TRN_PORT               APP_IO_TYPE_GPIOA
#define APP_HCI_UART_FLOW_PORT              APP_IO_TYPE_GPIOA
#define APP_HCI_UART_TX_PIN                 APP_IO_PIN_4
#define APP_HCI_UART_RX_PIN                 APP_IO_PIN_5
#define APP_HCI_UART_CTS_PIN                APP_IO_PIN_8
#define APP_HCI_UART_RTS_PIN                APP_IO_PIN_9
#define APP_HCI_UART_TX_PINMUX              APP_IO_MUX_3
#define APP_HCI_UART_RX_PINMUX              APP_IO_MUX_3
#define APP_HCI_UART_CTS_PINMUX             APP_IO_MUX_4
#define APP_HCI_UART_RTS_PINMUX             APP_IO_MUX_4
#define APP_HCI_UART_TRIGGER_PIN            AON_GPIO_PIN_1

/*******UART DRIVER IO CONFIG*******************/
#define APP_UART_ID                         APP_UART_ID_2
#define APP_UART_BAUDRATE                   115200
#define APP_UART_TX_IO_TYPE                 APP_IO_TYPE_GPIOA
#define APP_UART_RX_IO_TYPE                 APP_IO_TYPE_GPIOA
#define APP_UART_TX_PIN                     APP_IO_PIN_8
#define APP_UART_RX_PIN                     APP_IO_PIN_9
#define APP_UART_TX_PINMUX                  APP_IO_MUX_1
#define APP_UART_RX_PINMUX                  APP_IO_MUX_1
#define APP_UART_TX_PULL                    APP_IO_NOPULL
#define APP_UART_RX_PULL                    APP_IO_PULLUP

#define APP_UART1_ID                        APP_UART_ID_1
#define APP_UART1_BAUDRATE                  115200
#define APP_UART1_TX_IO_TYPE                APP_IO_TYPE_GPIOC
#define APP_UART1_RX_IO_TYPE                APP_IO_TYPE_GPIOC
#define APP_UART1_TX_PIN                    APP_IO_PIN_0
#define APP_UART1_RX_PIN                    APP_IO_PIN_1
#define APP_UART1_TX_PINMUX                 APP_IO_MUX_4
#define APP_UART1_RX_PINMUX                 APP_IO_MUX_4

/*******I2C IO CONFIG***************************/
#define APP_I2C_3_ID                        APP_I2C_ID_3
#define APP_I2C_3_SCL_PIN                   APP_IO_PIN_11
#define APP_I2C_3_SDA_PIN                   APP_IO_PIN_12
#define APP_I2C_3_SCL_IO_TYPE               APP_IO_TYPE_GPIOA
#define APP_I2C_3_SDA_IO_TYPE               APP_IO_TYPE_GPIOA
#define APP_I2C_3_SCL_PINMUX                APP_IO_MUX_2
#define APP_I2C_3_SDA_PINMUX                APP_IO_MUX_2

#define APP_I2C_0_ID                        APP_I2C_ID_0
#define APP_I2C_0_SCL_PIN                   APP_IO_PIN_2
#define APP_I2C_0_SDA_PIN                   APP_IO_PIN_3
#define APP_I2C_0_SCL_IO_TYPE               APP_IO_TYPE_GPIOA
#define APP_I2C_0_SDA_IO_TYPE               APP_IO_TYPE_GPIOA
#define APP_I2C_0_SCL_PINMUX                APP_IO_MUX_0
#define APP_I2C_0_SDA_PINMUX                APP_IO_MUX_0

/*******TP I2C IO CONFIG***************************/
#define APP_TP_RST_IO_TYPE                  APP_IO_TYPE_GPIOB
#define APP_TP_RST_IO_MUX                   APP_IO_MUX_7
#define APP_TP_RST_IO_PIN                   APP_IO_PIN_13

#define APP_TP_INT_IO_TYPE                  APP_IO_TYPE_GPIOC
#define APP_TP_INT_IO_MUX                   APP_IO_MUX_7
#define APP_TP_INT_IO_PIN                   APP_IO_PIN_0

#define APP_TP_SCL_IO_TYPE                  APP_IO_TYPE_GPIOB
#define APP_TP_SCL_IO_MUX                   APP_IO_MUX_3
#define APP_TP_SCL_IO_PIN                   APP_IO_PIN_14

#define APP_TP_SDA_IO_TYPE                  APP_IO_TYPE_GPIOB
#define APP_TP_SDA_IO_MUX                   APP_IO_MUX_3
#define APP_TP_SDA_IO_PIN                   APP_IO_PIN_15

#define APP_TP_I2C_ID                       APP_I2C_ID_1
#define APP_TP_PERIPH_DEVICE                PERIPH_DEVICE_NUM_I2C1

/*******EEG SPI IO CONFIG***************************/
#define APP_SPIM_CS_PIN                     APP_IO_PIN_13
#define APP_SPIM_CLK_PIN                    APP_IO_PIN_10
#define APP_SPIM_MOSI_PIN                   APP_IO_PIN_11
#define APP_SPIM_MISO_PIN                   APP_IO_PIN_12
#define APP_SPIM_CS_IO_TYPE                 APP_IO_TYPE_GPIOB
#define APP_SPIM_CLK_IO_TYPE                APP_IO_TYPE_GPIOB
#define APP_SPIM_MOSI_IO_TYPE               APP_IO_TYPE_GPIOB
#define APP_SPIM_MISO_IO_TYPE               APP_IO_TYPE_GPIOB
#define APP_SPIM_CS_PINMUX                  APP_IO_MUX_3
#define APP_SPIM_CLK_PINMUX                 APP_IO_MUX_3
#define APP_SPIM_MOSI_PINMUX                APP_IO_MUX_3
#define APP_SPIM_MISO_PINMUX                APP_IO_MUX_3

/*******EEG AFE IO CONFIG***************************/
#define AFE_START_PIN  APP_IO_PIN_9
#define AFE_START_TYPE APP_IO_TYPE_GPIOB

#define AFE_RESET_PIN  APP_IO_PIN_8
#define AFE_RESET_TYPE APP_IO_TYPE_GPIOB

#define AFE_DRDY_PIN   APP_IO_PIN_7
#define AFE_DRDY_TYPE  APP_IO_TYPE_GPIOB

/*******I2S IO CONFIG***************************/
#define APP_I2S_MASTER_WS_PIN               APP_IO_PIN_6
#define APP_I2S_MASTER_SDO_PIN              APP_IO_PIN_7
#define APP_I2S_MASTER_SDI_PIN              APP_IO_PIN_8
#define APP_I2S_MASTER_SCLK_PIN             APP_IO_PIN_9
#define APP_I2S_MASTER_WS_IO_TYPE           APP_IO_TYPE_GPIOA
#define APP_I2S_MASTER_SDO_IO_TYPE          APP_IO_TYPE_GPIOA
#define APP_I2S_MASTER_SDI_IO_TYPE          APP_IO_TYPE_GPIOA
#define APP_I2S_MASTER_SCLK_IO_TYPE         APP_IO_TYPE_GPIOA
#define APP_I2S_MASTER_WS_PINMUX            APP_IO_MUX_3
#define APP_I2S_MASTER_SDO_PINMUX           APP_IO_MUX_3
#define APP_I2S_MASTER_SDI_PINMUX           APP_IO_MUX_3
#define APP_I2S_MASTER_SCLK_PINMUX          APP_IO_MUX_3

#define APP_I2S_SLAVE_WS_PIN                APP_IO_PIN_3
#define APP_I2S_SLAVE_SDO_PIN               APP_IO_PIN_2
#define APP_I2S_SLAVE_SDI_PIN               APP_IO_PIN_1
#define APP_I2S_SLAVE_SCLK_PIN              APP_IO_PIN_0
#define APP_I2S_SLAVE_WS_IO_TYPE            APP_IO_TYPE_MSIO
#define APP_I2S_SLAVE_SDO_IO_TYPE           APP_IO_TYPE_MSIO
#define APP_I2S_SLAVE_SDI_IO_TYPE           APP_IO_TYPE_MSIO
#define APP_I2S_SLAVE_SCLK_IO_TYPE          APP_IO_TYPE_MSIO
#define APP_I2S_SLAVE_WS_PINMUX             APP_IO_MUX_4
#define APP_I2S_SLAVE_SDO_PINMUX            APP_IO_MUX_4
#define APP_I2S_SLAVE_SDI_PINMUX            APP_IO_MUX_4
#define APP_I2S_SLAVE_SCLK_PINMUX           APP_IO_MUX_4

#define APP_I2S_LOOPBACK_IO_MUX             APP_IO_MUX_3
#define APP_I2S_LOOPBACK_WS_PIN             APP_IO_PIN_6
#define APP_I2S_LOOPBACK_SDO_PIN            APP_IO_PIN_7
#define APP_I2S_LOOPBACK_SDI_PIN            APP_IO_PIN_8
#define APP_I2S_LOOPBACK_SCLK_PIN           APP_IO_PIN_9
#define APP_I2S_LOOPBACK_WS_TYPE            APP_IO_TYPE_GPIOA
#define APP_I2S_LOOPBACK_SDO_TYPE           APP_IO_TYPE_GPIOA
#define APP_I2S_LOOPBACK_SDI_TYPE           APP_IO_TYPE_GPIOA
#define APP_I2S_LOOPBACK_SCLK_TYPE          APP_IO_TYPE_GPIOA

/*******PDM IO CONFIG***************************/
#define APP_PDM_LOOPBACK_IO_MUX             APP_IO_MUX_1
#define APP_PDM_LOOPBACK_CLK_PIN            APP_IO_PIN_2
#define APP_PDM_LOOPBACK_DATA_PIN           APP_IO_PIN_3
#define APP_PDM_LOOPBACK_CLK_TYPE           APP_IO_TYPE_GPIOA
#define APP_PDM_LOOPBACK_DATA_TYPE          APP_IO_TYPE_GPIOA

/*******GPADC IO CONFIG***************************/
#define APP_GPADC_P_INPUT_PIN               APP_IO_PIN_9
#define APP_GPADC_N_INPUT_PIN               APP_IO_PIN_8

/*******I2C IO CONFIG***************************/
#define APP_I2C1_IO_TYPE                    APP_IO_TYPE_AON
#define APP_I2C1_SCL_PIN                    APP_IO_PIN_2
#define APP_I2C1_SDA_PIN                    APP_IO_PIN_3

/*******UC1701 DRIVER IO CONFIG*****************/
#define DISPLAY_DRIVER_TYPE_HW_SPI
//#define DISPLAY_DRIVER_TYPE_SW_IO
#define DISPLAY_SPIM_CS0_PIN                APP_IO_PIN_3
#define DISPLAY_CMD_AND_DATA_PIN            APP_IO_PIN_5
#define DISPLAY_SPIM_CLK_PIN                APP_IO_PIN_7
#define DISPLAY_SPIM_MOSI_PIN               APP_IO_PIN_6
#define DISPLAY_BACK_LIGHT_PIN              APP_IO_PIN_2
#define DISPLAY_SPIM_GPIO_TYPE              APP_IO_TYPE_GPIOA

/*******PWM IO CONFIG***************************/
#define APP_PWM0_MODULE                     APP_PWM_ID_0
#define APP_PWM0_GPIO_MUX                   APP_IO_MUX_0
#define APP_PWM0_CHANNEL_A                  APP_IO_PIN_0
#define APP_PWM0_CHANNEL_B                  APP_IO_PIN_1
#define APP_PWM0_CHANNEL_C                  APP_IO_PIN_2
#define APP_PWM0_GPIO_TYPE                  APP_IO_TYPE_MSIO

#define APP_PWM1_MODULE                     APP_PWM_ID_1
#define APP_PWM1_GPIO_MUX                   APP_IO_MUX_0
#define APP_PWM1_CHANNEL_A                  APP_IO_PIN_4
#define APP_PWM1_CHANNEL_B                  APP_IO_PIN_5
#define APP_PWM1_CHANNEL_C                  APP_IO_PIN_6
#define APP_PWM1_GPIO_TYPE                  APP_IO_TYPE_AON

/*******QSPI IO CONFIG***************************/
#define APP_QSPI0_MODULE                    QSPI0
#define APP_QSPI0_PORT                      GPIO1
#define APP_QSPI0_GPIO_MUX                  APP_IO_MUX_0
#define APP_QSPI0_CS_IO_TYPE                APP_IO_TYPE_GPIOB
#define APP_QSPI0_CLK_IO_TYPE               APP_IO_TYPE_GPIOB
#define APP_QSPI0_IO0_IO_TYPE               APP_IO_TYPE_GPIOB
#define APP_QSPI0_IO1_IO_TYPE               APP_IO_TYPE_GPIOB
#define APP_QSPI0_IO2_IO_TYPE               APP_IO_TYPE_GPIOB
#define APP_QSPI0_IO3_IO_TYPE               APP_IO_TYPE_GPIOB
#define APP_QSPI0_CS_PIN                    APP_IO_PIN_10
#define APP_QSPI0_CLK_PIN                   APP_IO_PIN_5
#define APP_QSPI0_IO0_PIN                   APP_IO_PIN_6
#define APP_QSPI0_IO1_PIN                   APP_IO_PIN_7
#define APP_QSPI0_IO2_PIN                   APP_IO_PIN_8
#define APP_QSPI0_IO3_PIN                   APP_IO_PIN_9

#define APP_QSPI1_MODULE                    QSPI1
#define APP_QSPI1_PORT                      GPIO0
#define APP_QSPI1_GPIO_MUX                  APP_IO_MUX_0
#define APP_QSPI1_CS_IO_TYPE                APP_IO_TYPE_GPIOA
#define APP_QSPI1_CLK_IO_TYPE               APP_IO_TYPE_GPIOA
#define APP_QSPI1_IO0_IO_TYPE               APP_IO_TYPE_GPIOA
#define APP_QSPI1_IO1_IO_TYPE               APP_IO_TYPE_GPIOA
#define APP_QSPI1_IO2_IO_TYPE               APP_IO_TYPE_GPIOA
#define APP_QSPI1_IO3_IO_TYPE               APP_IO_TYPE_GPIOA
#define APP_QSPI1_CS_PIN                    APP_IO_PIN_10
#define APP_QSPI1_CLK_PIN                   APP_IO_PIN_15
#define APP_QSPI1_IO0_PIN                   APP_IO_PIN_14
#define APP_QSPI1_IO1_PIN                   APP_IO_PIN_13
#define APP_QSPI1_IO2_PIN                   APP_IO_PIN_12
#define APP_QSPI1_IO3_PIN                   APP_IO_PIN_11

#define APP_QSPI2_MODULE                    QSPI2
#define APP_QSPI2_PORT                      GPIO1
#define APP_QSPI2_GPIO_MUX                  APP_IO_MUX_0
#define APP_QSPI2_CS_IO_TYPE                APP_IO_TYPE_GPIOB
#define APP_QSPI2_CLK_IO_TYPE               APP_IO_TYPE_GPIOB
#define APP_QSPI2_IO0_IO_TYPE               APP_IO_TYPE_GPIOB
#define APP_QSPI2_IO1_IO_TYPE               APP_IO_TYPE_GPIOB
#define APP_QSPI2_IO2_IO_TYPE               APP_IO_TYPE_GPIOB
#define APP_QSPI2_IO3_IO_TYPE               APP_IO_TYPE_GPIOB
#define APP_QSPI2_CS_PIN                    APP_IO_PIN_11
#define APP_QSPI2_CLK_PIN                   APP_IO_PIN_0
#define APP_QSPI2_IO0_PIN                   APP_IO_PIN_1
#define APP_QSPI2_IO1_PIN                   APP_IO_PIN_2
#define APP_QSPI2_IO2_PIN                   APP_IO_PIN_3
#define APP_QSPI2_IO3_PIN                   APP_IO_PIN_4

#define APP_FLASH_GPIO_MUX                  APP_QSPI0_GPIO_MUX
#define APP_FLASH_CS_IO_TYPE                APP_QSPI0_CS_IO_TYPE
#define APP_FLASH_CLK_IO_TYPE               APP_QSPI0_CLK_IO_TYPE
#define APP_FLASH_IO0_IO_TYPE               APP_QSPI0_IO0_IO_TYPE
#define APP_FLASH_IO1_IO_TYPE               APP_QSPI0_IO1_IO_TYPE
#define APP_FLASH_IO2_IO_TYPE               APP_QSPI0_IO2_IO_TYPE
#define APP_FLASH_IO3_IO_TYPE               APP_QSPI0_IO3_IO_TYPE
#define APP_FLASH_CS_PIN                    APP_QSPI0_CS_PIN
#define APP_FLASH_CLK_PIN                   APP_QSPI0_CLK_PIN
#define APP_FLASH_IO0_PIN                   APP_QSPI0_IO0_PIN
#define APP_FLASH_IO1_PIN                   APP_QSPI0_IO1_PIN
#define APP_FLASH_IO2_PIN                   APP_QSPI0_IO2_PIN
#define APP_FLASH_IO3_PIN                   APP_QSPI0_IO3_PIN
#define APP_FLASH_QSPI_ID                   APP_QSPI_ID_0


#define PWR_HOLD_PIN APP_IO_PIN_6
#define PWR_HOLD_TYPE APP_IO_TYPE_AON

#define PWR_KEY_PIN APP_IO_PIN_7
#define PWR_KEY_TYPE APP_IO_TYPE_AON

#define CES_UP_KEY_PIN APP_IO_PIN_3
#define CES_UP_KEY_TYPE APP_IO_TYPE_AON

#define CES_DN_KEY_PIN APP_IO_PIN_2
#define CES_DN_KEY_TYPE APP_IO_TYPE_AON

#define VDD33_EN_PIN APP_IO_PIN_15
#define VDD33_EN_TYPE APP_IO_TYPE_GPIOB

#define AFE_PWR_EN_PIN APP_IO_PIN_7
#define AFE_PWR_EN_TYPE APP_IO_TYPE_MSIO

#define LED1_PIN APP_IO_PIN_6
#define LED1_TYPE APP_IO_TYPE_GPIOA
#define LED2_PIN APP_IO_PIN_7
#define LED2_TYPE APP_IO_TYPE_GPIOA




/**
 * @defgroup BSP_MAROC Defines
 * @{
 */
    #define BSP_KEY_UP_ID       0x00    /**< ID for UP KEY. */
    #define BSP_KEY_OK_ID       0x01    /**< ID for OK KEY. */

    #define UART_TX_BUFF_SIZE   0x400   /**< Size of app uart tx buffer. */
/** @} */

/**
 * @defgroup BSP_ENUM Enumerations
 * @{
 */
typedef enum
{
    BSP_LED_NUM_0,
    BSP_LED_NUM_1,
} bsp_led_num_t;
/** @} */

/**
 * @defgroup BSP_FUNCTION Functions
 * @{
 */
/**
 *****************************************************************************************
 * @brief Initialize boards key.
 *****************************************************************************************
 */
void bsp_key_init(void);

/**
 *****************************************************************************************
 * @brief App key event handler
 *****************************************************************************************
 */
void app_key_evt_handler(uint8_t key_id, app_key_click_type_t key_click_type);

/**
 *****************************************************************************************
 * @brief Initialize app uart.
 *****************************************************************************************
 */
void bsp_uart_init(void);

/**
 *****************************************************************************************
 * @brief Uart data send.
 *****************************************************************************************
 */
void bsp_uart_send(uint8_t *p_data, uint16_t length);

/**
 *****************************************************************************************
 * @brief Uart data flush.
 *****************************************************************************************
 */
void bsp_uart_flush(void);

/**
 *****************************************************************************************
 * @brief App uart event handler.
 *****************************************************************************************
 */
void app_uart_evt_handler(app_uart_evt_t *p_evt);

/**
 *****************************************************************************************
 * @brief Initialize boards led.
 *****************************************************************************************
 */
void bsp_led_init(void);

/**
 *****************************************************************************************
 * @brief Open boards led.
 *
 * @param[in] led_num: Number of led needed open.
 *****************************************************************************************
 */
void bsp_led_open(bsp_led_num_t led_num);

/**
 *****************************************************************************************
 * @brief Close boards led.
 *
 * @param[in] led_num: Number of led needed close.
 *****************************************************************************************
 */
void bsp_led_close(bsp_led_num_t led_num);

/**
 *****************************************************************************************
 * @brief BSP log init.
 *****************************************************************************************
 */
void bsp_log_init(void);

/**
 *****************************************************************************************
 * @brief Board init.
 *****************************************************************************************
 */
void board_init(void);

#endif  /* __BOARD_SK_H__ */
