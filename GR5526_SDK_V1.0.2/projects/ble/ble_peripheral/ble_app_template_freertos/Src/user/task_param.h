#ifndef TASK_PARAM_H_
#define TASK_PARAM_H_
#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>

typedef enum {
	TASK_RUNNING,
	TASK_EXIT,
} task_run_state_t;

typedef struct {
	task_run_state_t state;
} task_param_t;

#ifdef __cplusplus
}
#endif
#endif /* TASK_PARAM_H_ */
