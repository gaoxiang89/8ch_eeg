#ifndef SYSTEM_MANAGER_H_
#define SYSTEM_MANAGER_H_
#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
typedef enum {
	STATE_OFF,
	STATE_CHARGING,
	STATE_TURNING_ON,
	STATE_ON,
	STATE_TURNING_OFF,
	STATE_PAIRING,
	STATE_PAIRING_FAILED,
} sys_state_t;

typedef enum {
	EVENT_NONE,
	EVENT_BUTTON_CLICK,
	EVENT_BUTTON_SHORT_START,
	EVENT_BUTTON_SHORT_UP,
	EVENT_BUTTON_LONG_START,
	EVENT_BUTTON_LONG_UP,
	EVENT_BUTTON_LONG_HOLD,
	EVENT_BUTTON_LONG_HOLD_UP,
	EVENT_PLUG_IN,
	EVENT_UNPLUG,
	EVENT_LEAD_OFF_TIMEOUT,
	EVENT_BLE_CONNECTED,
	EVENT_BLE_DISCONNECTED,
	EVENT_BT_CONNECTED,
	EVENT_BT_DISCONNECTED,
	EVENT_BT_PAIR_ENTRY,
	EVENT_BT_PAIR_TIMEOUT,	
	EVENT_FORCE_POWER_OFF,
	EVENT_ENTER_WORK_MODE,
	EVENT_MAIN_OTA_ENTRY,
	EVENT_BT_OTA_ENTRY,
} sys_event_t;

void vSystemManagerTask(void *arg);

int sys_manager_event_set(sys_event_t event);

sys_state_t sys_manager_state_get(void);


#ifdef __cplusplus
}
#endif
#endif /* SYSTEM_MANAGER_H_ */
