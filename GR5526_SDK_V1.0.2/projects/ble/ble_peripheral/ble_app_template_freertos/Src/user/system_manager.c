#include "system_manager.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "queue.h"
#include "user_periph_setup.h"
#include "button/button.h"
#include "task_param.h"
#include "board_bc.h"
#include "app_log.h"
#include "user_app.h"

static sys_state_t current_state;
static sys_event_t current_event;

#define SYS_EVT_QUEUE_SIZE 16
static QueueHandle_t event_queue;

typedef struct
{
	TaskFunction_t pvTaskCode;
	const char *pcName;
	uint16_t usStackDepth;
	void *pvParameters;
	UBaseType_t uxPriority;
	task_param_t ctx;
} task_table_entry_t;

task_table_entry_t app_task_table[] = {
	{ NULL, "eeg_read_task", 1024, NULL, 6, { .state = TASK_RUNNING } },
};


#ifndef ARRAY_SIZE
#define ARRAY_SIZE(arr) (sizeof(arr) / sizeof((arr)[0]))
#endif

static volatile bool m_app_created = false;

static void power_off_prepare(void);

void app_tasks_create(void)
{
	if (m_app_created)
	{
		return;
	}

	m_app_created = true;

	for (int i = 0; i < ARRAY_SIZE(app_task_table); i++)
	{
		xTaskCreate(app_task_table[i].pvTaskCode, app_task_table[i].pcName, app_task_table[i].usStackDepth,
					(void *)&app_task_table[i].ctx, app_task_table[i].uxPriority, NULL);
	}
}

void app_tasks_delete(void)
{
	m_app_created = false;

	for (int i = 0; i < ARRAY_SIZE(app_task_table); i++)
	{
		app_task_table[i].ctx.state = TASK_EXIT;
		APP_LOG_INFO("app task state: exit\r\n");
	}
}

static void sys_manager_handle_event(sys_state_t *current_state, sys_event_t event)
{
	APP_LOG_INFO("state %d, handle event: %d \r\n", *current_state, current_event);
	switch (*current_state)
	{
	case STATE_OFF:
		if (event == EVENT_BUTTON_SHORT_START)
		{
			*current_state = STATE_TURNING_ON;
		}
		else if (event == EVENT_PLUG_IN)
		{
			*current_state = STATE_CHARGING;
		}
		break;

	case STATE_CHARGING:
		if (event == EVENT_UNPLUG)
		{
			*current_state = STATE_OFF;
			APP_LOG_ERROR("STATE_CHARGING --> STATE_OFF\r\n");
		}
		else if (event == EVENT_ENTER_WORK_MODE)
		{
			*current_state = STATE_ON;
			APP_LOG_ERROR("STATE_CHARGING --> STATE_ON\r\n");
			app_tasks_create();
		}
		break;

	case STATE_TURNING_ON:
		if (event == EVENT_BUTTON_CLICK || event == EVENT_BUTTON_SHORT_UP)
		{
			/* 按键未按足够时间就松开，进入关机状态 */
			*current_state = STATE_OFF;
			APP_LOG_ERROR("STATE_TURNING_ON --> STATE_OFF \r\n");
		}
		else if (event == EVENT_BUTTON_LONG_START)
		{
			*current_state = STATE_ON;
			APP_LOG_ERROR("STATE_TURNING_ON --> STATE_ON \r\n");
			app_tasks_create();
		}
		else if (event == EVENT_PLUG_IN)
		{
			*current_state = STATE_CHARGING;
			APP_LOG_ERROR("STATE_TURNING_ON --> STATE_CHARGING \r\n");
		}
		break;

	case STATE_ON:
		if (event == EVENT_BUTTON_LONG_START || event == EVENT_LEAD_OFF_TIMEOUT)
		{
			*current_state = STATE_TURNING_OFF;
		}
		else if (event == EVENT_PLUG_IN)
		{
			*current_state = STATE_CHARGING;
			APP_LOG_INFO("STATE_ON --> STATE_CHARGING\r\n");
		}
		else if (event == EVENT_MAIN_OTA_ENTRY || event == EVENT_BT_OTA_ENTRY)
		{
		}
		else if (event == EVENT_BT_PAIR_ENTRY)
		{
			*current_state = STATE_PAIRING;
			APP_LOG_INFO("STATE_ON --> STATE_PAIRING");
		}
		else if (event == EVENT_BLE_DISCONNECTED)
		{
		}
		else if (event == EVENT_BT_CONNECTED)
		{
		}
		break;

	case STATE_TURNING_OFF:
		if (event == EVENT_BUTTON_LONG_UP)
		{
			*current_state = STATE_OFF;
			APP_LOG_INFO("STATE_TURNING_OFF --> STATE_OFF");
		}
		else if (event == EVENT_BUTTON_LONG_HOLD || event == EVENT_BUTTON_LONG_HOLD_UP)
		{
			*current_state = STATE_PAIRING;
			APP_LOG_INFO("STATE_TURNING_OFF --> STATE_PAIRING");
		}
		break;

	case STATE_PAIRING:
		if (event == EVENT_BUTTON_SHORT_UP || event == EVENT_BT_PAIR_TIMEOUT)
		{
			*current_state = STATE_PAIRING_FAILED;
		}
		else if (event == EVENT_BUTTON_LONG_START)
		{
			*current_state = STATE_OFF;
		}
		else if (event == EVENT_BT_CONNECTED)
		{
			*current_state = STATE_ON;
		}
		break;

	case STATE_PAIRING_FAILED:
		if (event == EVENT_BUTTON_LONG_START)
		{
			*current_state = STATE_TURNING_OFF;
		}
		else if (event == EVENT_BT_CONNECTED)
		{
			*current_state = STATE_ON;
		}
		break;

	default:
		break;
	}

	if (event == EVENT_FORCE_POWER_OFF && (*current_state != STATE_CHARGING))
	{
		APP_LOG_INFO("Force Power Off\r\n");
		*current_state = STATE_OFF;
	}
}

void sys_manager_process(void)
{
	switch (current_state)
	{
	case STATE_OFF:
		break;

	case STATE_CHARGING:
		break;

	case STATE_TURNING_ON:
		break;

	case STATE_ON:

		break;

	case STATE_TURNING_OFF:

		break;

	case STATE_PAIRING:
		break;

	case STATE_PAIRING_FAILED:

		break;

	default:
		APP_LOG_INFO("Unknown State\r\n");
		break;
	}
}

int sys_manager_event_set(sys_event_t event)
{
	BaseType_t ret = pdTRUE;

	ret = xQueueSend(event_queue, &event, 50);
	if (ret != pdTRUE)
	{
		APP_LOG_INFO("sys event set failed\r\n");
	}

	return 0;
}



void vSystemManagerTask(void *arg)
{
	event_queue = xQueueCreate(SYS_EVT_QUEUE_SIZE, sizeof(sys_event_t));
	if (!event_queue)
	{
		APP_LOG_INFO("event_queue create failed\r\n");
	}

	xTaskCreate(vButtonTask, "button_task", 512, NULL, 1, NULL);

	current_event = EVENT_NONE;

	while (1)
	{
		if (xQueueReceive(event_queue, &current_event, 100) == pdTRUE)
		{
			sys_manager_handle_event(&current_state, current_event);
		}

		sys_manager_process();

		if (current_state == STATE_OFF)
		{
			power_off_prepare();
			break;
		}
	}

	APP_LOG_INFO("system manager task will delete\r\n");
	vTaskDelete(NULL);
}

sys_state_t sys_manager_state_get(void)
{
	return current_state;
}

void power_off_prepare(void)
{
	app_tasks_delete();

	pwr_mgmt_mode_set(PMR_MGMT_SLEEP_MODE);
}

void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName)
{
	APP_LOG_ERROR("task stack overflow, %s\r\n", pcTaskName);
}

void vApplicationMallocFailedHook(void)
{
	APP_LOG_ERROR("malloc failed\r\n");
}
