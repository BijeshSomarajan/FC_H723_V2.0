#ifndef SRC_FC_MANAGERS_POSITION_COMMON_POSITIONCOMMON_H_
#define SRC_FC_MANAGERS_POSITION_COMMON_POSITIONCOMMON_H_

#include <sys/_stdint.h>

typedef struct _POSITION_CORDINATE_DATA POSITION_CORDINATE_DATA;
struct _POSITION_CORDINATE_DATA {

	float xPositionRaw, yPositionRaw,zPositionRaw;
	float xPosition, yPosition, zPosition;

	float xVelocity, yVelocity, zVelocity;

	float xAcceleration, yAcceleration, zAcceleration;
	float xAccelerationBias, yAccelerationBias, zAccelerationBias;

	float positionProcessDt;
	float positionXYUpdateDt;
	float positionZUpdateDt;
};
extern POSITION_CORDINATE_DATA positionCordinateData;

typedef struct _POSITION_COMMAND_DATA POSITION_COMMAND_DATA;
struct _POSITION_COMMAND_DATA {
	float pitchCommand, rollCommand;
	float positionCommandDt;
};
extern POSITION_COMMAND_DATA positionCommandData;

#endif /* SRC_FC_MANAGERS_POSITION_COMMON_POSITIONCOMMON_H_ */
