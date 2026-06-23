#ifndef SRC_FC_TIMERS_GPTIMER_H_
#define SRC_FC_TIMERS_GPTIMER_H_

/* User callback type */
typedef void (*GPTimerCallback_t)(void);

//Used by Attitude AGT sensor read
uint8_t initGPTimer24(uint32_t frequencyHz, GPTimerCallback_t callback, uint8_t priority);
void startGPTimer24(void);
void stopGPTimer24(void);

//Used by Attitude Mag sensor read
uint8_t initGPTimer4(uint32_t frequencyHz, GPTimerCallback_t callback, uint8_t priority);
void startGPTimer4(void);
void stopGPTimer4(void);

//Used by Altitude sensor read
uint8_t initGPTimer3(uint32_t frequencyHz, GPTimerCallback_t callback, uint8_t priority);
void startGPTimer3(void);
void stopGPTimer3(void);

//Used by Motor control
uint8_t initGPTimer6(uint32_t frequencyHz, GPTimerCallback_t callback, uint8_t priority);
void startGPTimer6(void);
void stopGPTimer6(void);

//Used by Attitude control
uint8_t initGPTimer7(uint32_t frequencyHz, GPTimerCallback_t callback, uint8_t priority);
void startGPTimer7(void);
void stopGPTimer7(void);

#endif /* SRC_FC_TIMERS_GPTIMER_H_ */
