#include "ConfigHelperMobileApp.h"

#include <stdio.h>
#include <sys/_stdint.h>
#include "../../logger/Logger.h"
#include "../../sensors/rc/devices/crsf/CRSFFcConfig.h"
#include "../../status/FCStatus.h"
#include "ConfigHelperCommon.h"

/* Single source of truth for the number of int32 values per sub-command */
#define CFG_N_AG_BIAS        6
#define CFG_N_MAG_BIAS       13
#define CFG_N_RC_BIAS        4
#define CFG_N_ATT_MASTER_PID 6
#define CFG_N_ATT_RATE_PID   12
#define CFG_N_ALT_MASTER_PID 4
#define CFG_N_ALT_RATE_PID   5
#define CFG_N_POS_MASTER_PID 4
#define CFG_N_POS_RATE_PID   5
#define CFG_N_STICK_GAIN     3
#define CFG_N_DEVICE         3
#define CFG_N_MISC           8
#define CFG_N_EST_PROCESS    5
#define CFG_N_EST_MEASUREMENT 7
#define CFG_MAX_VALUES       13   /* size of ConfigDataPacket.data[] */

uint8_t getConfigValueCount(uint8_t subCmd) {
	switch (subCmd) {
	case SUB_CMD_AG_BIAS:
		return CFG_N_AG_BIAS;
	case SUB_CMD_MAG_BIAS:
		return CFG_N_MAG_BIAS;
	case SUB_CMD_RC_BIAS:
		return CFG_N_RC_BIAS;
	case SUB_CMD_ATT_MASTER_PID:
		return CFG_N_ATT_MASTER_PID;
	case SUB_CMD_ATT_RATE_PID:
		return CFG_N_ATT_RATE_PID;
	case SUB_CMD_ALT_MASTER_PID:
		return CFG_N_ALT_MASTER_PID;
	case SUB_CMD_ALT_RATE_PID:
		return CFG_N_ALT_RATE_PID;
	case SUB_CMD_POS_MASTER_PID:
		return CFG_N_POS_MASTER_PID;
	case SUB_CMD_POS_RATE_PID:
		return CFG_N_POS_RATE_PID;
	case SUB_CMD_STICK_GAIN:
		return CFG_N_STICK_GAIN;
	case SUB_CMD_DEVICE:
		return CFG_N_DEVICE;
	case SUB_CMD_MISC:
		return CFG_N_MISC;
	case SUB_CMD_EST_PROCESS:
		return CFG_N_EST_PROCESS;
	case SUB_CMD_EST_MEASUREMENT:
		return CFG_N_EST_MEASUREMENT;
	default:
		return 0;
	}
}

void mergeConfigDataFromMobileApp(int32_t *data, ConfigDataPacket *pkt) {
	if (pkt->cmd != CMD_SAVE_CONFIG) {
		return;
	}
	/* Ignore short/unknown packets so a partial save never half-updates the config */
	if (pkt->length != getConfigValueCount(pkt->subCmd)) {
		return;
	}
	const int32_t *v = pkt->data; /* values arrive in the same order as sendConfigDataToMobileApp */
	switch (pkt->subCmd) {
	case SUB_CMD_AG_BIAS: {
		data[CALIB_PROP_AX_BIAS_ADDR] = v[0];
		data[CALIB_PROP_AY_BIAS_ADDR] = v[1];
		data[CALIB_PROP_AZ_BIAS_ADDR] = v[2];
		data[CALIB_PROP_GX_BIAS_ADDR] = v[3];
		data[CALIB_PROP_GY_BIAS_ADDR] = v[4];
		data[CALIB_PROP_GZ_BIAS_ADDR] = v[5];
		break;
	}
	case SUB_CMD_MAG_BIAS: {
		data[CALIB_PROP_MX_BIAS_ADDR] = v[0];
		data[CALIB_PROP_MY_BIAS_ADDR] = v[1];
		data[CALIB_PROP_MZ_BIAS_ADDR] = v[2];
		data[CALIB_PROP_M_SI_00_ADDR] = v[3];
		data[CALIB_PROP_M_SI_01_ADDR] = v[4];
		data[CALIB_PROP_M_SI_02_ADDR] = v[5];
		data[CALIB_PROP_M_SI_10_ADDR] = v[6];
		data[CALIB_PROP_M_SI_11_ADDR] = v[7];
		data[CALIB_PROP_M_SI_12_ADDR] = v[8];
		data[CALIB_PROP_M_SI_20_ADDR] = v[9];
		data[CALIB_PROP_M_SI_21_ADDR] = v[10];
		data[CALIB_PROP_M_SI_22_ADDR] = v[11];
		data[CALIB_PROP_HEADING_BIAS_ADDR] = v[12];
		break;
	}
	case SUB_CMD_RC_BIAS: {
		data[CALIB_PROP_RC_THROTTLE_OFFSET_ADDR] = v[0];
		data[CALIB_PROP_RC_PITCH_OFFSET_ADDR] = v[1];
		data[CALIB_PROP_RC_ROLL_OFFSET_ADDR] = v[2];
		data[CALIB_PROP_RC_YAW_OFFSET_ADDR] = v[3];
		break;
	}
	case SUB_CMD_ATT_MASTER_PID: {
		data[CALIB_PROP_PID_KP_PITCH_ADDR] = v[0];
		data[CALIB_PROP_PID_KP_ROLL_ADDR] = v[1];
		data[CALIB_PROP_PID_KP_YAW_ADDR] = v[2];
		data[CALIB_PROP_PID_PITCH_LIMIT_ADDR] = v[3];
		data[CALIB_PROP_PID_ROLL_LIMIT_ADDR] = v[4];
		data[CALIB_PROP_PID_YAW_LIMIT_ADDR] = v[5];
		break;
	}
	case SUB_CMD_ATT_RATE_PID: {
		data[CALIB_PROP_RATE_PID_KP_PITCH_ADDR] = v[0];
		data[CALIB_PROP_RATE_PID_KP_ROLL_ADDR] = v[1];
		data[CALIB_PROP_RATE_PID_KP_YAW_ADDR] = v[2];
		data[CALIB_PROP_RATE_PID_KI_PITCH_ADDR] = v[3];
		data[CALIB_PROP_RATE_PID_KI_ROLL_ADDR] = v[4];
		data[CALIB_PROP_RATE_PID_KI_YAW_ADDR] = v[5];
		data[CALIB_PROP_RATE_PID_KD_PITCH_ADDR] = v[6];
		data[CALIB_PROP_RATE_PID_KD_ROLL_ADDR] = v[7];
		data[CALIB_PROP_RATE_PID_KD_YAW_ADDR] = v[8];
		data[CALIB_PROP_RATE_PID_PITCH_LIMIT_ADDR] = v[9];
		data[CALIB_PROP_RATE_PID_ROLL_LIMIT_ADDR] = v[10];
		data[CALIB_PROP_RATE_PID_YAW_LIMIT_ADDR] = v[11];
		break;
	}
	case SUB_CMD_ALT_MASTER_PID: {
		data[CALIB_PROP_ALT_HOLD_PID_KP_ADDR] = v[0];
		data[CALIB_PROP_ALT_HOLD_PID_LIMIT_ADDR] = v[1];
		data[CALIB_PROP_ALT_HOLD_SPEED_ADDR] = v[2];
		data[CALIB_PROP_ALT_HOLD_DOB_FF_GAIN_ADDR] = v[3];
		break;
	}
	case SUB_CMD_ALT_RATE_PID: {
		data[CALIB_PROP_ALT_HOLD_RATE_PID_KP_ADDR] = v[0];
		data[CALIB_PROP_ALT_HOLD_RATE_PID_KI_ADDR] = v[1];
		data[CALIB_PROP_ALT_HOLD_RATE_PID_KD_ADDR] = v[2];
		data[CALIB_PROP_ALT_HOLD_RATE_PID_LIMIT_ADDR] = v[3];
		data[CALIB_PROP_ALT_HOLD_CONTROL_LIMIT_ADDR] = v[4];
		break;
	}
	case SUB_CMD_POS_MASTER_PID: {
		data[CALIB_PROP_POS_HOLD_PID_KP_ADDR] = v[0];
		data[CALIB_PROP_POS_HOLD_PID_LIMIT_ADDR] = v[1];
		data[CALIB_PROP_POS_HOLD_CRUISE_SPEED_ADDR] = v[2];
		data[CALIB_PROP_POS_HOLD_RTH_SPEED_ADDR] = v[3];
		break;
	}
	case SUB_CMD_POS_RATE_PID: {
		data[CALIB_PROP_POS_HOLD_RATE_PID_KP_ADDR] = v[0];
		data[CALIB_PROP_POS_HOLD_RATE_PID_KI_ADDR] = v[1];
		data[CALIB_PROP_POS_HOLD_RATE_PID_KD_ADDR] = v[2];
		data[CALIB_PROP_POS_HOLD_RATE_PID_LIMIT_ADDR] = v[3];
		data[CALIB_PROP_POS_HOLD_BRAKE_DECEL_ADDR] = v[4];
		break;
	}
	case SUB_CMD_STICK_GAIN: {
		data[CALIB_PROP_RC_PITCH_RATE_P_ADDR] = v[0];
		data[CALIB_PROP_RC_ROLL_RATE_P_ADDR] = v[1];
		data[CALIB_PROP_RC_YAW_RATE_P_ADDR] = v[2];
		break;
	}
	case SUB_CMD_DEVICE: {
		data[CALIB_PROP_MODEL_VERSION_ADDR] = v[0];
		data[CALIB_PROP_BAT_TYPE_ADDR] = v[1];
		data[CALIB_PROP_BAT_NOM_VOLT_ADDR] = v[2];
		break;
	}
	case SUB_CMD_MISC: {
		data[CALIB_PROP_THRUST_EXPO_ADDR] = v[0];
		data[CALIB_PROP_RC_LIFTOFF_THROTTLE_ADDR] = v[1];
		data[CALIB_PROP_ALT_HOLD_LIFT_OFF_ALT_ADDR] = v[2];
		data[CALIB_PROP_MISSION_MAX_ALT_CLIMB_ADDR] = v[3];
		data[CALIB_PROP_TILT_COMP_GAIN_ADDR] = v[4];
		data[CALIB_PROP_TILT_COMP_FALL_TAU_ADDR] = v[5];
		data[CALIB_PROP_VENTURI_GAIN_ADDR] = v[6];
		data[CALIB_PROP_VENTURI_FALL_TAU_ADDR] = v[7];
		break;
	}
	case SUB_CMD_EST_PROCESS: {
		data[CALIB_PROP_EST_XY_Q_POS_ADDR] = v[0];
		data[CALIB_PROP_EST_XY_Q_VEL_ADDR] = v[1];
		data[CALIB_PROP_EST_Z_Q_POS_ADDR] = v[2];
		data[CALIB_PROP_EST_Z_Q_VEL_ADDR] = v[3];
		data[CALIB_PROP_EST_DYNAMIC_Q_ENABLED_ADDR] = v[4];
		break;
	}
	case SUB_CMD_EST_MEASUREMENT: {
		data[CALIB_PROP_EST_Z_GNSS_RP_BASE_ADDR] = v[0];
		data[CALIB_PROP_EST_Z_GNSS_RV_BASE] = v[1];
		data[CALIB_PROP_EST_Z_TERRAIN_RP_BASE] = v[2];
		data[CALIB_PROP_EST_Z_BARO_RP_BASE_ADDR] = v[3];
		data[CALIB_PROP_EST_Z_BARO_RP_GAIN_ADDR] = v[4];
		data[CALIB_PROP_EST_Z_BARO_DYNAMIC_R_ENABLED_ADDR] = v[5];
		data[CALIB_PROP_EST_Z_BARO_CRUISE_ADAPT_ENABLED_ADDR] = v[6];
		break;
	}
	default: {
		break;
	}
	}
}

void sendConfigDataToMobileApp(int32_t *data, int32_t length, int32_t cmd, int32_t subCmd) {
	(void) length;
	switch (subCmd) {
	case SUB_CMD_AG_BIAS: {
		int32_t values[CFG_N_AG_BIAS] = { data[CALIB_PROP_AX_BIAS_ADDR], data[CALIB_PROP_AY_BIAS_ADDR], data[CALIB_PROP_AZ_BIAS_ADDR], data[CALIB_PROP_GX_BIAS_ADDR], data[CALIB_PROP_GY_BIAS_ADDR], data[CALIB_PROP_GZ_BIAS_ADDR] };
		sendCRSFConfigData(values, CFG_N_AG_BIAS, cmd, subCmd);
		break;
	}
	case SUB_CMD_MAG_BIAS: {
		int32_t values[CFG_N_MAG_BIAS] = { data[CALIB_PROP_MX_BIAS_ADDR], data[CALIB_PROP_MY_BIAS_ADDR], data[CALIB_PROP_MZ_BIAS_ADDR], data[CALIB_PROP_M_SI_00_ADDR], data[CALIB_PROP_M_SI_01_ADDR], data[CALIB_PROP_M_SI_02_ADDR], data[CALIB_PROP_M_SI_10_ADDR], data[CALIB_PROP_M_SI_11_ADDR],
				data[CALIB_PROP_M_SI_12_ADDR], data[CALIB_PROP_M_SI_20_ADDR], data[CALIB_PROP_M_SI_21_ADDR], data[CALIB_PROP_M_SI_22_ADDR], data[CALIB_PROP_HEADING_BIAS_ADDR] };
		sendCRSFConfigData(values, CFG_N_MAG_BIAS, cmd, subCmd);
		break;
	}
	case SUB_CMD_RC_BIAS: {
		int32_t values[CFG_N_RC_BIAS] = { data[CALIB_PROP_RC_THROTTLE_OFFSET_ADDR], data[CALIB_PROP_RC_PITCH_OFFSET_ADDR], data[CALIB_PROP_RC_ROLL_OFFSET_ADDR], data[CALIB_PROP_RC_YAW_OFFSET_ADDR] };
		sendCRSFConfigData(values, CFG_N_RC_BIAS, cmd, subCmd);
		break;
	}
	case SUB_CMD_ATT_MASTER_PID: {
		int32_t values[CFG_N_ATT_MASTER_PID] = { data[CALIB_PROP_PID_KP_PITCH_ADDR], data[CALIB_PROP_PID_KP_ROLL_ADDR], data[CALIB_PROP_PID_KP_YAW_ADDR], data[CALIB_PROP_PID_PITCH_LIMIT_ADDR], data[CALIB_PROP_PID_ROLL_LIMIT_ADDR], data[CALIB_PROP_PID_YAW_LIMIT_ADDR] };
		sendCRSFConfigData(values, CFG_N_ATT_MASTER_PID, cmd, subCmd);
		break;
	}
	case SUB_CMD_ATT_RATE_PID: {
		int32_t values[CFG_N_ATT_RATE_PID] = { data[CALIB_PROP_RATE_PID_KP_PITCH_ADDR], data[CALIB_PROP_RATE_PID_KP_ROLL_ADDR], data[CALIB_PROP_RATE_PID_KP_YAW_ADDR], data[CALIB_PROP_RATE_PID_KI_PITCH_ADDR], data[CALIB_PROP_RATE_PID_KI_ROLL_ADDR], data[CALIB_PROP_RATE_PID_KI_YAW_ADDR],
				data[CALIB_PROP_RATE_PID_KD_PITCH_ADDR], data[CALIB_PROP_RATE_PID_KD_ROLL_ADDR], data[CALIB_PROP_RATE_PID_KD_YAW_ADDR], data[CALIB_PROP_RATE_PID_PITCH_LIMIT_ADDR], data[CALIB_PROP_RATE_PID_ROLL_LIMIT_ADDR], data[CALIB_PROP_RATE_PID_YAW_LIMIT_ADDR] };
		sendCRSFConfigData(values, CFG_N_ATT_RATE_PID, cmd, subCmd);
		break;
	}
	case SUB_CMD_ALT_MASTER_PID: {
		int32_t values[CFG_N_ALT_MASTER_PID] = { data[CALIB_PROP_ALT_HOLD_PID_KP_ADDR], data[CALIB_PROP_ALT_HOLD_PID_LIMIT_ADDR], data[CALIB_PROP_ALT_HOLD_SPEED_ADDR] ,data[CALIB_PROP_ALT_HOLD_DOB_FF_GAIN_ADDR] };
		sendCRSFConfigData(values, CFG_N_ALT_MASTER_PID, cmd, subCmd);
		break;
	}
	case SUB_CMD_ALT_RATE_PID: {
		int32_t values[CFG_N_ALT_RATE_PID] = { data[CALIB_PROP_ALT_HOLD_RATE_PID_KP_ADDR], data[CALIB_PROP_ALT_HOLD_RATE_PID_KI_ADDR], data[CALIB_PROP_ALT_HOLD_RATE_PID_KD_ADDR], data[CALIB_PROP_ALT_HOLD_RATE_PID_LIMIT_ADDR], data[CALIB_PROP_ALT_HOLD_CONTROL_LIMIT_ADDR] };
		sendCRSFConfigData(values, CFG_N_ALT_RATE_PID, cmd, subCmd);
		break;
	}
	case SUB_CMD_POS_MASTER_PID: {
		int32_t values[CFG_N_POS_MASTER_PID] = { data[CALIB_PROP_POS_HOLD_PID_KP_ADDR], data[CALIB_PROP_POS_HOLD_PID_LIMIT_ADDR], data[CALIB_PROP_POS_HOLD_CRUISE_SPEED_ADDR], data[CALIB_PROP_POS_HOLD_RTH_SPEED_ADDR] };
		sendCRSFConfigData(values, CFG_N_POS_MASTER_PID, cmd, subCmd);
		break;
	}
	case SUB_CMD_POS_RATE_PID: {
		int32_t values[CFG_N_POS_RATE_PID] = { data[CALIB_PROP_POS_HOLD_RATE_PID_KP_ADDR], data[CALIB_PROP_POS_HOLD_RATE_PID_KI_ADDR], data[CALIB_PROP_POS_HOLD_RATE_PID_KD_ADDR], data[CALIB_PROP_POS_HOLD_RATE_PID_LIMIT_ADDR], data[CALIB_PROP_POS_HOLD_BRAKE_DECEL_ADDR] };
		sendCRSFConfigData(values, CFG_N_POS_RATE_PID, cmd, subCmd);
		break;
	}
	case SUB_CMD_STICK_GAIN: {
		int32_t values[CFG_N_STICK_GAIN] = { data[CALIB_PROP_RC_PITCH_RATE_P_ADDR], data[CALIB_PROP_RC_ROLL_RATE_P_ADDR], data[CALIB_PROP_RC_YAW_RATE_P_ADDR] };
		sendCRSFConfigData(values, CFG_N_STICK_GAIN, cmd, subCmd);
		break;
	}
	case SUB_CMD_DEVICE: {
		int32_t values[CFG_N_DEVICE] = { data[CALIB_PROP_MODEL_VERSION_ADDR], data[CALIB_PROP_BAT_TYPE_ADDR], data[CALIB_PROP_BAT_NOM_VOLT_ADDR] };
		sendCRSFConfigData(values, CFG_N_DEVICE, cmd, subCmd);
		break;
	}
	case SUB_CMD_MISC: {
		int32_t values[CFG_N_MISC] = { data[CALIB_PROP_THRUST_EXPO_ADDR], data[CALIB_PROP_RC_LIFTOFF_THROTTLE_ADDR], data[CALIB_PROP_ALT_HOLD_LIFT_OFF_ALT_ADDR], data[CALIB_PROP_MISSION_MAX_ALT_CLIMB_ADDR], data[CALIB_PROP_TILT_COMP_GAIN_ADDR], data[CALIB_PROP_TILT_COMP_FALL_TAU_ADDR],
				data[CALIB_PROP_VENTURI_GAIN_ADDR], data[CALIB_PROP_VENTURI_FALL_TAU_ADDR] };
		sendCRSFConfigData(values, CFG_N_MISC, cmd, subCmd);
		break;
	}
	case SUB_CMD_EST_PROCESS: {
		int32_t values[CFG_N_EST_PROCESS] = { data[CALIB_PROP_EST_XY_Q_POS_ADDR], data[CALIB_PROP_EST_XY_Q_VEL_ADDR], data[CALIB_PROP_EST_Z_Q_POS_ADDR], data[CALIB_PROP_EST_Z_Q_VEL_ADDR], data[CALIB_PROP_EST_DYNAMIC_Q_ENABLED_ADDR] };
		sendCRSFConfigData(values, CFG_N_EST_PROCESS, cmd, subCmd);
		break;
	}
	case SUB_CMD_EST_MEASUREMENT: {
		int32_t values[CFG_N_EST_MEASUREMENT] = { data[CALIB_PROP_EST_Z_GNSS_RP_BASE_ADDR], data[CALIB_PROP_EST_Z_GNSS_RV_BASE], data[CALIB_PROP_EST_Z_TERRAIN_RP_BASE], data[CALIB_PROP_EST_Z_BARO_RP_BASE_ADDR], data[CALIB_PROP_EST_Z_BARO_RP_GAIN_ADDR],
				data[CALIB_PROP_EST_Z_BARO_DYNAMIC_R_ENABLED_ADDR], data[CALIB_PROP_EST_Z_BARO_CRUISE_ADAPT_ENABLED_ADDR] };
		sendCRSFConfigData(values, CFG_N_EST_MEASUREMENT, cmd, subCmd);
		break;
	}
	default:
		break;
	}
}

static int32_t readInt32BE(const uint8_t *p) {
	uint32_t v = ((uint32_t) p[0] << 24) | ((uint32_t) p[1] << 16) | ((uint32_t) p[2] << 8) | (uint32_t) p[3];
	return (int32_t) v;
}

static void debugConfigPacket(const ConfigDataPacket *pkt) {
	char buf[160];
	int n = snprintf(buf, sizeof(buf), "CFG src=%u cmd=0x%02X sub=%u n=%u :", (unsigned) pkt->source, (unsigned) pkt->cmd, (unsigned) pkt->subCmd, (unsigned) pkt->length);

	for (uint8_t i = 0; i < pkt->length && n > 0 && n < (int) sizeof(buf); i++) {
		n += snprintf(&buf[n], sizeof(buf) - n, "%s%ld", (i == 0) ? " " : ",", (long) pkt->data[i]);
	}
	if (n < (int) sizeof(buf) - 1) {
		buf[n++] = '\n';
		buf[n] = '\0';
	}
	logString(buf);
}

ConfigDataPacket deSeralizeMblAppData(const uint8_t *payload) {
	ConfigDataPacket configDataPacket;
	uint8_t configCommand = payload[3];
	uint8_t configSubCommand = payload[4];

	switch (configCommand) {
	case CMD_SAVE_CONFIG: {
		uint8_t count = getConfigValueCount(configSubCommand);
		const uint8_t *p = &payload[5]; /* values start right after subCmd */
		configDataPacket.cmd = configCommand;
		configDataPacket.subCmd = configSubCommand;
		configDataPacket.length = count; /* 0 for an unknown sub-command */
		for (uint8_t i = 0; i < count; i++) {
			configDataPacket.data[i] = readInt32BE(p);
			p += 4;
		}
		break;
	}
	default:
		configDataPacket.cmd = configCommand;
		configDataPacket.subCmd = configSubCommand;
		configDataPacket.length = 0;
		break;
	}

	configDataPacket.source = CONFIG_SOURCE_MOBILE_APP;
	return configDataPacket;
}

void _fcMblAppConfigCallback(const uint8_t *payload) {
	// payload[0..2] = destination, origin, sequence , command , subcommand
	ConfigDataPacket configDataPacket = deSeralizeMblAppData(payload);
	debugConfigPacket(&configDataPacket);
	addConfigDataPacket(configDataPacket);
}

uint8_t initConfigHelperMobileApp() {
	logString("[Config Helper Mobile App] : UART -> Init , Success!\n");
	registerCRSFFcConfigCallBack(_fcMblAppConfigCallback);
	logString("[Config Helper Mobile App] : FCConfig Callback Registered, Success!\n");
	return 1;
}
