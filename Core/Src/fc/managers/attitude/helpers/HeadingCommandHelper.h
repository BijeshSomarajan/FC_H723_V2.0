#ifndef SRC_FC_MANAGERS_ATTITUDE_HELPERS_HEADINGCOMMANDHELPER_H_
#define SRC_FC_MANAGERS_ATTITUDE_HELPERS_HEADINGCOMMANDHELPER_H_
#include <sys/_stdint.h>

#define HEADING_COMMAND_ROTATION_RATE     20.0f   // deg/s
#define HEADING_COMMAND_DONE_THRESHOLD    5.0f   // deg, measured nose within this of the target
#define HEADING_COMMAND_TIMEOUT_MARGIN    3.0f   // s, added to 2x the ideal turn time before the command is abandoned

void setAbsoluteHeadingCommand(float headingDegree);
void setRelativeHeadingCommand(float deltaDeg);
void cancelHeadingCommand(void);
void handleHeadingCommand(float dt);
uint8_t isHeadingCommandComplete(void);

#endif /* SRC_FC_MANAGERS_ATTITUDE_HELPERS_HEADINGCOMMANDHELPER_H_ */
