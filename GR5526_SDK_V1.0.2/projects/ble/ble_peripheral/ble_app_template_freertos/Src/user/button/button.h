#ifndef BUTTON_H_
#define BUTTON_H_
#ifdef __cplusplus
extern "C"
{
#endif

	typedef enum
	{
		BTN_POWER = 0,
		BTN_CES_UP,
		BTN_CES_DN,
		BTN_MAX,
	} button_index_t;

	void vButtonTask(void *arg);

	void button_init(void);

	void button_init_wakeup(void);

	int button_read(button_index_t id);

	int button_check(button_index_t id);

	void button_power_up_skip(void);

#ifdef __cplusplus
}
#endif
#endif /* BUTTON_H_ */
