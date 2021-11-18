#ifndef LINX_LINXCUSTOMCMDHANDLER_H_
#define LINX_LINXCUSTOMCMDHANDLER_H_

#include "SEGGER_RTT.h"

#include "Config.h"
#include "EnergocycleSettings.h"
#include "Flash.h"
#include "LinxDevice.h"


enum LINX_CUSTOM_CMD {
    CUSTOM_CMD_OSTANOV = 0xFE01,
    CUSTOM_CMD_STATUS = 0xFE02,
    CUSTOM_CMD_RUN = 0xFE03,
    CUSTOM_CMD_PID = 0xFE04,
    CUSTOM_CMD_CALIBRATION = 0xFE05,
    CUSTOM_CMD_CALIBRATION_SET = 0xFE06
};

#define CMD_OFFSET CUSTOM_CMD_OSTANOV
#define CALL_MEMBER_FN(object, ptr) (object->*ptr)

class LinxCustomCmdHandler {
public:
	LinxCustomCmdHandler();
	virtual ~LinxCustomCmdHandler();

	void float_to_uca(float value, unsigned char *data);
	float uca_to_float(const unsigned char *data);

	void settings_unpack(unsigned char *data, Energocycle_settings_s *es);
	void status_packetize(Energocycle_status_s *es, unsigned char *data);
	void pid_settings_unpack(unsigned char *data, EC_PID_settings *settings);

	LinxStatus CUSTOM_CMD_OSTANOV_f(unsigned char data_rx_size, unsigned char* data_rx, unsigned char* data_tx_size, unsigned char* data_tx);
	LinxStatus CUSTOM_CMD_STATUS_f(unsigned char data_rx_size, unsigned char* data_rx, unsigned char* data_tx_size, unsigned char* data_tx);
	LinxStatus CUSTOM_CMD_RUN_f(unsigned char data_rx_size, unsigned char* data_rx, unsigned char* data_tx_size, unsigned char* data_tx);
	LinxStatus CUSTOM_CMD_PID_f(unsigned char data_rx_size, unsigned char* data_rx, unsigned char* data_tx_size, unsigned char* data_tx);
	LinxStatus CUSTOM_CMD_CALIBRATION_f(unsigned char data_rx_size, unsigned char* data_rx, unsigned char* data_tx_size, unsigned char* data_tx);

};

#endif /* LINX_LINXCUSTOMCMDHANDLER_H_ */
