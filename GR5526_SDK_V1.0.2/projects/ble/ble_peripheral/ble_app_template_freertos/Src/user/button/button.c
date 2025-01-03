#include "button.h"
#include "flexible_button.h"

#include "board_bc.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "timers.h"
#include "app_io.h"
#include "app_gpiote.h"
#include "app_log.h"
#include "errno.h"
#include "system_manager.h"
#include <stdint.h>

#define SCAN_INTERVAL		 (1000 / FLEX_BTN_SCAN_FREQ_HZ)
#define SHORT_PRESS_TIME	 100
#define LONG_PRESS_TIME_1	 1500
#define LONG_PRESS_TIME		 2000
#define LONG_PRESS_HOLD_TIME 5000

static flex_button_t m_flex_btns[BTN_MAX];
static bool first_pressed = true;
static uint32_t scan_btn_count = 0;

static app_io_init_t button_pins[BTN_MAX] = {
	{ .pin = PWR_KEY_PIN,  .mux = APP_IO_MUX_7, .pull = APP_IO_PULLUP },
	{ .pin = CES_UP_KEY_PIN,  .mux = APP_IO_MUX_7, .pull = APP_IO_PULLUP },
	{ .pin = CES_DN_KEY_PIN,  .mux = APP_IO_MUX_7, .pull = APP_IO_PULLUP },
};

static bool is_init_wakeup = false;

typedef struct {
	uint32_t id;
	uint32_t count;
	bool start;
} button_factory_test_t;

static button_factory_test_t btn_factory_test = { .id = 0, .count = 0, .start = false };

static uint8_t btn_read(void *arg)
{
	flex_button_t *btn = (flex_button_t *)arg;

	return app_io_read_pin(APP_IO_TYPE_AON, button_pins[btn->id].pin);
}

static void btn_evt_cb(void *arg)
{
	flex_button_t *btn = (flex_button_t *)arg;
	flex_button_event_t btn_event = flex_button_event_read(btn);

	if (btn->id == BTN_POWER) {
		if (btn_event == FLEX_BTN_PRESS_CLICK) {
		} else if (btn_event == FLEX_BTN_PRESS_SHORT_START) {
		} else if (btn_event == FLEX_BTN_PRESS_SHORT_UP) {
		} else if (btn_event == FLEX_BTN_PRESS_LONG_START) {
		} else if (btn_event == FLEX_BTN_PRESS_LONG_UP) {
			if (first_pressed) {
				flex_button_update_long_press_time(btn, FLEX_MS_TO_SCAN_CNT(LONG_PRESS_TIME));
				first_pressed = false;
			}
		} else if (btn_event == FLEX_BTN_PRESS_LONG_HOLD) {
		} else if (btn_event == FLEX_BTN_PRESS_LONG_HOLD_UP) {
			if (first_pressed) {
				flex_button_update_long_press_time(btn, FLEX_MS_TO_SCAN_CNT(LONG_PRESS_TIME));
				first_pressed = false;
			}
		}
	} else if (btn->id == BTN_CES_UP) {
	}
}

static void button_gpio_init(void)
{
	for (int i = 0; i < BTN_MAX; i++) {
		app_io_init(APP_IO_TYPE_AON, &button_pins[i]);
	}
}

void button_power_up_skip(void)
{
	scan_btn_count = 0xffff;
}

void button_power_up_process(void)
{
	static uint32_t btn_no_press_count = 0;

	if (scan_btn_count < FLEX_MS_TO_SCAN_CNT(LONG_PRESS_TIME_1)) {
		scan_btn_count++;
		if (button_read(BTN_POWER) == 1) {
			btn_no_press_count++;
		}

		if (btn_no_press_count > FLEX_MS_TO_SCAN_CNT(500)) {
			scan_btn_count = 0xffff;
		}
	}
}

void button_factory_process(void)
{
//	if (btn_factory_test.start) {
//		btn_factory_test.count++;
//		if (button_read(btn_factory_test.id) == 0) {
//			at_cmd_print(SHELL_RESP_OK, strlen(SHELL_RESP_OK));
//			btn_factory_test.start = false;
//			btn_factory_test.count = 0;
//		}

//		if (btn_factory_test.count >= 15 * FLEX_BTN_SCAN_FREQ_HZ) {
//			at_cmd_print(SHELL_RESP_ERROR, strlen(SHELL_RESP_ERROR));
//			btn_factory_test.start = false;
//			btn_factory_test.count = 0;
//		}
//	}
}

void vButtonTask(void *arg)
{
	TickType_t last_wake_time;

	button_init();

	last_wake_time = xTaskGetTickCount();

	while (!is_init_wakeup) {
		flex_button_scan();

		button_power_up_process();

		button_factory_process();

		vTaskDelayUntil(&last_wake_time, SCAN_INTERVAL);
	}

	APP_LOG_DEBUG("button task is deleted\r\n");
	vTaskDelete(NULL);
}

void button_init(void)
{
	button_gpio_init();

	for (int i = 0; i < BTN_MAX; i++) {
		m_flex_btns[i].id = i;
		m_flex_btns[i].usr_button_read = btn_read;
		m_flex_btns[i].cb = btn_evt_cb;
		m_flex_btns[i].pressed_logic_level = 0;
		m_flex_btns[i].short_press_start_tick = FLEX_MS_TO_SCAN_CNT(SHORT_PRESS_TIME);
		m_flex_btns[i].long_press_start_tick = FLEX_MS_TO_SCAN_CNT(LONG_PRESS_TIME_1);
		m_flex_btns[i].long_hold_start_tick = FLEX_MS_TO_SCAN_CNT(LONG_PRESS_HOLD_TIME);

		flex_button_register(&m_flex_btns[i]);
	}
}

int button_read(button_index_t id)
{
	return app_io_read_pin(APP_IO_TYPE_AON, button_pins[id].pin);
}

int button_check(button_index_t id)
{
	if (btn_factory_test.start) {
		return -2;
	}

	btn_factory_test.start = true;
	btn_factory_test.count = 0;
	btn_factory_test.id = id;

	return 0;
}
