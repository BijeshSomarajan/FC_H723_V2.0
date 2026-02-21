#include "stm32h7xx_ll_tim.h"
#include "stm32h7xx_ll_bus.h"
#include "stm32h7xx_ll_system.h"
#include "stm32h7xx_ll_utils.h"
#include "stm32h7xx_ll_cortex.h"
#include "Scheduler.h"
#include "string.h"

typedef struct {
	void (*taskFunc)(void);
	uint32_t periodTicks;
	uint32_t nextRunTicks;
	uint8_t priority;
	uint8_t active;
} SchedulerTask;

/* -----------------------------------------------------------
 * 🧩 Scheduler configuration constants
 * -----------------------------------------------------------*/
#define SCHEDULER_MAX_TASKS     16U
#define SCHEDULER_MAX_PRIORITY  255U

/* TIM5 setup: Use 100us tick for slow scheduler tasks */
static const uint32_t schedulerTim5Prescaler = 274UL;
static const uint32_t schedulerTim5Period = 99UL; /* 100 µs = 10 kHz base tick */

static volatile uint32_t schedulerTick = 0U;
static SchedulerTask schedulerTaskList[SCHEDULER_MAX_TASKS];
static uint8_t schedulerTaskCount = 0U;

static void schedulerSortTasksByPriority(void);

/* -----------------------------------------------------------
 * @brief  Initialize TIM5 for 10 kHz scheduler tick (100 µs)
 * -----------------------------------------------------------*/
uint8_t schedulerInit(void) {
	LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM5);
	LL_TIM_DisableCounter(TIM5);
	LL_TIM_SetPrescaler(TIM5, schedulerTim5Prescaler);
	LL_TIM_SetAutoReload(TIM5, schedulerTim5Period);
	LL_TIM_SetCounterMode(TIM5, LL_TIM_COUNTERMODE_UP);
	LL_TIM_SetClockDivision(TIM5, LL_TIM_CLOCKDIVISION_DIV1);
	LL_TIM_SetUpdateSource(TIM5, LL_TIM_UPDATESOURCE_REGULAR);
	LL_TIM_EnableIT_UPDATE(TIM5);

	// Set a lower priority for this slow scheduler's interrupt (e.g. priority 5, sub 0)
	NVIC_SetPriority(TIM5_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 5, 0));
	NVIC_EnableIRQ(TIM5_IRQn);

	LL_TIM_SetCounter(TIM5, 0U);
	LL_TIM_EnableCounter(TIM5);
	LL_TIM_GenerateEvent_UPDATE(TIM5);

	return 1;
}

/* -----------------------------------------------------------
 * @brief  TIM5 interrupt handler — increments scheduler tick
 * -----------------------------------------------------------*/
void TIM5_IRQHandler(void) {
	// This ISR is extremely fast and just increments a volatile variable
	if (LL_TIM_IsActiveFlag_UPDATE(TIM5)) {
		LL_TIM_ClearFlag_UPDATE(TIM5);
		schedulerTick++;
	}
}



/* -----------------------------------------------------------
 * @brief  Optimized Sort (Insertion Sort is better for small, mostly sorted arrays)
 * -----------------------------------------------------------*/
static void schedulerSortTasksByPriority(void) {
	int i, j;
	SchedulerTask key;
	for (i = 1; i < schedulerTaskCount; i++) {
		key = schedulerTaskList[i];
		j = i - 1;
		while ( j >= 0 && schedulerTaskList[j].priority > key.priority ) {
			schedulerTaskList[j + 1] = schedulerTaskList[j];
			j = j - 1;
		}
		schedulerTaskList[j + 1] = key;
	}
}


/* -----------------------------------------------------------
 * @brief  Add a new periodic task
 * -----------------------------------------------------------*/
uint8_t schedulerAddTask(void (*func)(void), float frequencyHz, uint8_t priority) {
	if (schedulerTaskCount >= SCHEDULER_MAX_TASKS) {
		return 0xFFU;
	}
	if (frequencyHz <= 0.0f) {
		return 0xFFU;
	}
	// This section is safe because it's only intended to be called during initialization
	// when the scheduler is not actively dispatching/interrupting yet.
	SchedulerTask *t = &schedulerTaskList[schedulerTaskCount];
	t->taskFunc = func;
	t->priority = priority;
	t->periodTicks = (uint32_t) (10000.0f / frequencyHz); // Use 100us base
	if (t->periodTicks == 0U) {
		t->periodTicks = 1U;
	}
	t->nextRunTicks = schedulerTick + t->periodTicks;
	t->active = 1U;
	schedulerTaskCount++;
	// Sort in place after adding the new task
	schedulerSortTasksByPriority();
	return (schedulerTaskCount - 1U);
}

/* -----------------------------------------------------------
 * @brief  Run due tasks — call this from main loop
 *         Reads tick counter using a safe (atomic) copy operation.
 * -----------------------------------------------------------*/
void dispatchScheduler(void) {
	for (uint8_t i = 0U; i < schedulerTaskCount; i++) {
		SchedulerTask *t = &schedulerTaskList[i];
		if (t->active == 0U) {
			continue;
		}
		// Atomic read of the volatile 32-bit schedulerTick on a 32-bit CPU is safe
		// (Assuming you have configured the H7 core's memory access appropriately via MPU/SCB)
		const uint32_t currentTick = schedulerTick;
		if (currentTick < t->nextRunTicks) {
			continue;
		}
		// All tasks must be fast (non-blocking) in a cooperative scheduler
		t->taskFunc();
		t->nextRunTicks += t->periodTicks;
	}
}
