#ifndef SRC_FC_TIMERS_SCHEDULER_H_
#define SRC_FC_TIMERS_SCHEDULER_H_

uint8_t schedulerInit(void);
uint8_t schedulerAddTask(void (*func)(void), float frequencyHz, uint8_t priority);
void dispatchScheduler(void);

#endif
