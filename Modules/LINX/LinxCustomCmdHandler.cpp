#include <LinxCustomCmdHandler.h>

extern Energocycle_settings_s esettings;
extern Energocycle_status_s estatus;
extern EC_FLASH_settings eflash_settings;

const unsigned char CUSTOM_CMD_DATA_LENGTH[6] {
    0x01,
    0x01,
    0x26,
    0x24,
    0x10
};

const unsigned char CUSTOM_RSP_DATA_LENGTH[6] {
    0x00,
    0x11,
    0x00,
    0x00,
    0x00,
    0x00
};

LinxCustomCmdHandler::LinxCustomCmdHandler() {

}

LinxCustomCmdHandler::~LinxCustomCmdHandler() {

}

void LinxCustomCmdHandler::float_to_uca(float value, unsigned char *data) {
    unsigned char *ptr = (unsigned char *) &value;
    for (unsigned char i = 0; i < 4; ++i) {
        *data = *ptr;
        data++;
        ptr++;
    }
}

float LinxCustomCmdHandler::uca_to_float(const unsigned char *data) {
    float result = 0.0;
    unsigned char *ptr = (unsigned char *) &result;
    for (unsigned char i = 0; i < 4; ++i) {
        *ptr = *data;
        data++;
        ptr++;
    }
    return result;
}

void LinxCustomCmdHandler::settings_unpack(unsigned char *data, Energocycle_settings_s *es) {
    es->mode = (Energocycle_mode) *(data + 0);
    es->res1 = *(data + 1);
    es->res2 = *(data + 2);
    es->res3 = *(data + 3);

    es->Unom = uca_to_float(data + 4);
    es->Umax = uca_to_float(data + 8);
    es->Umin = uca_to_float(data + 12);
    es->Inom = uca_to_float(data + 16);
    es->Imax = uca_to_float(data + 20);
    es->Imin = uca_to_float(data + 24);
    es->Tmax = uca_to_float(data + 28);
    es->Tmin = uca_to_float(data + 32);

    es->Cycles = ((*(data + 36)) << 8) | *(data + 37);
}

void LinxCustomCmdHandler::pid_settings_unpack(unsigned char *data, EC_PID_settings *settings) {
	settings->U_PID_settings.Kp = uca_to_float(data + 0);
	settings->U_PID_settings.Ki = uca_to_float(data + 4);
	settings->U_PID_settings.Kd = uca_to_float(data + 8);

	settings->I_PID_settings.Kp = uca_to_float(data + 12);
	settings->I_PID_settings.Ki = uca_to_float(data + 16);
	settings->I_PID_settings.Kd = uca_to_float(data + 20);

	settings->T_PID_settings.Kp = uca_to_float(data + 24);
	settings->T_PID_settings.Ki = uca_to_float(data + 28);
	settings->T_PID_settings.Kd = uca_to_float(data + 32);
}

void LinxCustomCmdHandler::status_packetize(Energocycle_status_s *es, unsigned char *data) {
    //*(data + 0) = es->status;
    *(data + 0) = es->state;
    *(data + 1) = es->error;
    *(data + 2) = es->res;

    float_to_uca(es->U, (data + 3));
    float_to_uca(es->I, (data + 7));
    float_to_uca(es->T, (data + 11));

    *(data + 15) = (es->Cycles & 0xFF00) >> 8;
    *(data + 16) = es->Cycles & 0x00FF;
}

LinxStatus LinxCustomCmdHandler::CUSTOM_CMD_OSTANOV_f(unsigned char data_rx_size, unsigned char* data_rx, unsigned char* data_tx_size, unsigned char* data_tx) {
	LINX_CUSTOM_CMD cmd = CUSTOM_CMD_OSTANOV;

	//SEGGER_RTT_printf(0, "CUSTOM_CMD_OSTANOV\n");

    return L_OK;
}

LinxStatus LinxCustomCmdHandler::CUSTOM_CMD_STATUS_f(unsigned char data_rx_size, unsigned char* data_rx, unsigned char* data_tx_size, unsigned char* data_tx) {
	LINX_CUSTOM_CMD cmd = CUSTOM_CMD_STATUS;

	//SEGGER_RTT_printf(0, "CUSTOM_CMD_STATUS\n");

    status_packetize(&estatus, data_tx);
    *data_tx_size = CUSTOM_RSP_DATA_LENGTH[cmd - CMD_OFFSET];

    return L_OK;
}

LinxStatus LinxCustomCmdHandler::CUSTOM_CMD_RUN_f(unsigned char data_rx_size, unsigned char* data_rx, unsigned char* data_tx_size, unsigned char* data_tx) {
	LINX_CUSTOM_CMD cmd = CUSTOM_CMD_RUN;

	LinxStatus status;
	if (data_rx_size == CUSTOM_CMD_DATA_LENGTH[cmd - CMD_OFFSET]) {
		settings_unpack(data_rx, &esettings);
#ifdef SEGGER_DEBUG
		SEGGER_RTT_printf(0, "CUSTOM_CMD_RUN \n"
				"mode=%u; Cycles=%u;\n"
				"Unom=%u.%02u; Umax=%u.%02u; Umin=%u.%02u; \n"
				"Inom=%u.%02u; Imax=%u.%02u; Imin=%u.%02u; \n"
				"Tmax=%u.%02u; Tmin=%u.%02u\n", esettings.mode,	esettings.Cycles,
				FLOAT_PRINTF(esettings.Unom),
				FLOAT_PRINTF(esettings.Umax),
				FLOAT_PRINTF(esettings.Umin),
				FLOAT_PRINTF(esettings.Inom),
				FLOAT_PRINTF(esettings.Imax),
				FLOAT_PRINTF(esettings.Imin),
				FLOAT_PRINTF(esettings.Tmax),
				FLOAT_PRINTF(esettings.Tmin));
#endif

		status = L_OK;
	} else {
		status = L_UNKNOWN_ERROR;
	}

    return status;
}

LinxStatus LinxCustomCmdHandler::CUSTOM_CMD_PID_f(unsigned char data_rx_size, unsigned char* data_rx, unsigned char* data_tx_size, unsigned char* data_tx) {
	LINX_CUSTOM_CMD cmd = CUSTOM_CMD_PID;

	LinxStatus status;
	if (data_rx_size == CUSTOM_CMD_DATA_LENGTH[cmd - CMD_OFFSET]) {
		pid_settings_unpack(data_rx, &eflash_settings.epid_settings);
#ifdef SEGGER_DEBUG
		SEGGER_RTT_printf(0, "CUSTOM_CMD_PID \n"
				"T Kp=%u.%02u; T Ki=%u.%02u; T Kd=%u.%02u; \n"
				"U Kp=%u.%02u; U Ki=%u.%02u; U Kd=%u.%02u; \n"
				"I Kp=%u.%02u; I Ki=%u.%02u; I Kd=%u.%02u; \n",
				FLOAT_PRINTF(eflash_settings.epid_settings.T_PID_settings.Kp),
				FLOAT_PRINTF(eflash_settings.epid_settings.T_PID_settings.Ki),
				FLOAT_PRINTF(eflash_settings.epid_settings.T_PID_settings.Kd),
				FLOAT_PRINTF(eflash_settings.epid_settings.U_PID_settings.Kp),
				FLOAT_PRINTF(eflash_settings.epid_settings.U_PID_settings.Ki),
				FLOAT_PRINTF(eflash_settings.epid_settings.U_PID_settings.Kd),
				FLOAT_PRINTF(eflash_settings.epid_settings.I_PID_settings.Kp),
				FLOAT_PRINTF(eflash_settings.epid_settings.I_PID_settings.Ki),
				FLOAT_PRINTF(eflash_settings.epid_settings.I_PID_settings.Kd));
#endif
		FLASH_WriteSettings();
		status = L_OK;
	} else {
		status = L_UNKNOWN_ERROR;
	}

    return L_OK;
}

LinxStatus LinxCustomCmdHandler::CUSTOM_CMD_CALIBRATION_f(unsigned char data_rx_size, unsigned char* data_rx, unsigned char* data_tx_size, unsigned char* data_tx) {
	LINX_CUSTOM_CMD cmd = CUSTOM_CMD_CALIBRATION;

	//SEGGER_RTT_printf(0, "CUSTOM_CMD_CALIBRATION\n");

    return L_OK;
}
