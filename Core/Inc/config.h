#ifndef INC_CONFIG_H_
#define INC_CONFIG_H_

#define SEGGER_DEBUG 1
#define FLOAT_PRINTF(x) (int)(x), (int)(((x) - (int)(x))*100)
//#define ENC28J60_ENABLE_UDP_NTP
//#define ENC28J60_ENABLE_UART_DEBUG
//#define ENC28J60_ENABLE_SYSVIEW_DEBUG
//#define ENC28J60_ENABLE_LED_DEBUG
//#define ASC_BluePill_PINOUT

#define MAC_ADDR   {0x00,0x15,0x42,0xBF,0xF0,0x51}
#define IP_ADDR {192,168,0,55}
#define IP_GATE {192,168,0,1}
#define IP_MASK {255,255,255,0}
#define LOCAL_PORT_TCP 80

#endif /* INC_CONFIG_H_ */

