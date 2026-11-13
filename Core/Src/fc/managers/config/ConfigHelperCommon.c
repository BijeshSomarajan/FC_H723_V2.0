#include "ConfigHelperCommon.h"

#include <sys/_stdint.h>

#define CONFIG_PACKET_BUFFER_SIZE 10
int16_t configDataPacketCount = 0;
ConfigDataPacket configDataPacketBuffer[CONFIG_PACKET_BUFFER_SIZE];

void addConfigDataPacket(ConfigDataPacket dataPacket) {
	if (configDataPacketCount < CONFIG_PACKET_BUFFER_SIZE) {
		configDataPacketBuffer[configDataPacketCount] = dataPacket;
		configDataPacketCount++;
	}
}

uint8_t hasConfigDataPacket() {
	if (configDataPacketCount > 0) {
		return 1;
	} else {
		return 0;
	}
}

ConfigDataPacket getConfigDataPacket() {
	ConfigDataPacket dataPacket = configDataPacketBuffer[0];
	if (configDataPacketCount > 0) {
		for (int32_t indx = 0; indx < configDataPacketCount - 1; indx++) {
			configDataPacketBuffer[indx] = configDataPacketBuffer[indx + 1];
		}
		configDataPacketCount--;
	}
	return dataPacket;
}
