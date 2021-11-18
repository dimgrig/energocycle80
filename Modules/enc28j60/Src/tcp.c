#include "tcp.h"
//--------------------------------------------------
//extern UART_HandleTypeDef ENC28J60_huart;
//-----------------------------------------------
extern char str1[60];
extern uint8_t net_buf[ENC28J60_MAXFRAME];
extern uint8_t macaddr[6];
extern uint8_t ipaddr[4];

extern uint16_t _len_data;
extern uint16_t _max_data_length;
extern uint8_t _data[2048];
extern float temp;

extern StreamBufferHandle_t tcp_rx_stream_buffer;
extern StreamBufferHandle_t tcp_tx_stream_buffer;
//--------------------------------------------------
uint8_t tcp_send(uint8_t *ip_addr, uint16_t port, uint8_t op)
{
	uint16_t sz_data=0;
  uint8_t res=0;
  uint16_t len=0;
	static uint32_t num_seg=0;
	enc28j60_frame_ptr *frame=(void*) net_buf;
	ip_pkt_ptr *ip_pkt = (void*)(frame->data);
	tcp_pkt_ptr *tcp_pkt = (void*)(ip_pkt->data);
	if (op==TCP_OP_SYNACK)
	{
		//�������� ��������� ������ TCP
		tcp_pkt->port_dst = be16toword(port);
		tcp_pkt->port_src = be16toword(LOCAL_PORT_TCP);
		tcp_pkt->num_ask = be32todword(be32todword(tcp_pkt->bt_num_seg) + 1);
		tcp_pkt->bt_num_seg = rand();
		tcp_pkt->fl = TCP_SYN | TCP_ACK;
		tcp_pkt->size_wnd = be16toword(8192);
		tcp_pkt->urg_ptr = 0;
		len = sizeof(tcp_pkt_ptr)+4;
		tcp_pkt->len_hdr = len << 2;
		tcp_pkt->data[0]=2;//Maximum Segment Size (2)
		tcp_pkt->data[1]=4;//Length
		tcp_pkt->data[2]=0x05;
		tcp_pkt->data[3]=0x82;
		tcp_pkt->cs = 0;
		tcp_pkt->cs=checksum((uint8_t*)tcp_pkt-8, len+8, 2);
		//�������� ��������� ������ IP
		sprintf(str1,"len:%d\r\n", len);
		HAL_UART_Transmit_MACRO(&huart1,(uint8_t*)str1,strlen(str1),0x1000);
		len+=sizeof(ip_pkt_ptr);
		sprintf(str1,"len:%d\r\n", len);
		HAL_UART_Transmit_MACRO(&huart1,(uint8_t*)str1,strlen(str1),0x1000);
		ip_pkt->len=be16toword(len);
		ip_pkt->id = 0;
		ip_pkt->ts = 0;
		ip_pkt->verlen = 0x45;
		ip_pkt->fl_frg_of=0;
		ip_pkt->ttl=128;
		ip_pkt->cs = 0;
		ip_pkt->prt=IP_TCP;
		memcpy(ip_pkt->ipaddr_dst,ip_addr,4);
		memcpy(ip_pkt->ipaddr_src,ipaddr,4);
		ip_pkt->cs = checksum((void*)ip_pkt,sizeof(ip_pkt_ptr),0);
		//�������� ��������� Ethernet
		memcpy(frame->addr_dest,frame->addr_src,6);
		memcpy(frame->addr_src,macaddr,6);
		frame->type=ETH_IP;
		len+=sizeof(enc28j60_frame_ptr);
		enc28j60_packetSend((void*)frame,len);
#ifdef ENC28J60_ENABLE_UART_DEBUG
		sprintf(str1,"len:%d\r\n", len);
		HAL_UART_Transmit_MACRO(&huart1,(uint8_t*)str1,strlen(str1),0x1000);
		HAL_UART_Transmit_MACRO(&huart1,(uint8_t*)"SYN ACK\r\n",9,0x1000);
#endif
	}
	else if (op==TCP_OP_ACK_OF_FIN)
	{
		//�������� ��������� ������ TCP
		tcp_pkt->port_dst = be16toword(port);
		tcp_pkt->port_src = be16toword(LOCAL_PORT_TCP);
		num_seg = tcp_pkt->num_ask;
		tcp_pkt->num_ask = be32todword(be32todword(tcp_pkt->bt_num_seg) + 1);
		//��������� 0 � USART, ����� ��������� ���
#ifdef ENC28J60_ENABLE_UART_DEBUG
		HAL_UART_Transmit_MACRO(&huart1,(uint8_t*)0,1,0x1000);
#endif
		tcp_pkt->bt_num_seg = num_seg;
		tcp_pkt->fl = TCP_ACK;
		tcp_pkt->size_wnd = be16toword(8192);
		tcp_pkt->urg_ptr = 0;
		len = sizeof(tcp_pkt_ptr);
		tcp_pkt->len_hdr = len << 2;
		tcp_pkt->cs = 0;
		tcp_pkt->cs=checksum((uint8_t*)tcp_pkt-8, len+8, 2);
		//�������� ��������� ������ IP
#ifdef ENC28J60_ENABLE_UART_DEBUG
		sprintf(str1,"len:%d\r\n", len);
		HAL_UART_Transmit_MACRO(&huart1,(uint8_t*)str1,strlen(str1),0x1000);
#endif
		len+=sizeof(ip_pkt_ptr);
#ifdef ENC28J60_ENABLE_UART_DEBUG
		sprintf(str1,"len:%d\r\n", len);
		HAL_UART_Transmit_MACRO(&huart1,(uint8_t*)str1,strlen(str1),0x1000);
#endif
		ip_pkt->len=be16toword(len);
		ip_pkt->id = 0;
		ip_pkt->ts = 0;
		ip_pkt->verlen = 0x45;
		ip_pkt->fl_frg_of=0;
		ip_pkt->ttl=128;
		ip_pkt->cs = 0;
		ip_pkt->prt=IP_TCP;
		memcpy(ip_pkt->ipaddr_dst,ip_addr,4);
		memcpy(ip_pkt->ipaddr_src,ipaddr,4);
		ip_pkt->cs = checksum((void*)ip_pkt,sizeof(ip_pkt_ptr),0);
		//�������� ��������� Ethernet
		memcpy(frame->addr_dest,frame->addr_src,6);
		memcpy(frame->addr_src,macaddr,6);
		frame->type=ETH_IP;
		len+=sizeof(enc28j60_frame_ptr);
		enc28j60_packetSend((void*)frame,len);
#ifdef ENC28J60_ENABLE_UART_DEBUG
		HAL_UART_Transmit_MACRO(&huart1,(uint8_t*)"ACK OF FIN\r\n",12,0x1000);
#endif
		tcp_pkt->fl = TCP_FIN|TCP_ACK;
    len = sizeof(tcp_pkt_ptr);
    tcp_pkt->cs = 0;
    tcp_pkt->cs=checksum((uint8_t*)tcp_pkt-8, len+8, 2);
    len+=sizeof(ip_pkt_ptr);
    len+=sizeof(enc28j60_frame_ptr);
    enc28j60_packetSend((void*)frame,len);
	}
	else if (op==TCP_OP_ACK_OF_DATA)
	{
		//�������� ��������� ������ TCP
		sz_data = be16toword(ip_pkt->len)-20-(tcp_pkt->len_hdr>>2);
		tcp_pkt->port_dst = be16toword(port);
		tcp_pkt->port_src = be16toword(LOCAL_PORT_TCP);
		num_seg = tcp_pkt->num_ask;
		tcp_pkt->num_ask = be32todword(be32todword(tcp_pkt->bt_num_seg) + sz_data);
#ifdef ENC28J60_ENABLE_UART_DEBUG
		sprintf(str1,"sz_data:%u\r\n", sz_data);
		HAL_UART_Transmit_MACRO(&huart1,(uint8_t*)str1,strlen(str1),0x1000);
#endif
		tcp_pkt->bt_num_seg = num_seg;
		tcp_pkt->fl = TCP_ACK;
		tcp_pkt->size_wnd = be16toword(8192);
		tcp_pkt->urg_ptr = 0;
		len = sizeof(tcp_pkt_ptr);
		tcp_pkt->len_hdr = len << 2;
		tcp_pkt->cs = 0;
		tcp_pkt->cs=checksum((uint8_t*)tcp_pkt-8, len+8, 2);
		//�������� ��������� ������ IP
#ifdef ENC28J60_ENABLE_UART_DEBUG
		sprintf(str1,"len:%d\r\n", len);
		HAL_UART_Transmit_MACRO(&huart1,(uint8_t*)str1,strlen(str1),0x1000);
#endif
		len+=sizeof(ip_pkt_ptr);
#ifdef ENC28J60_ENABLE_UART_DEBUG
		sprintf(str1,"len:%d\r\n", len);
		HAL_UART_Transmit_MACRO(&huart1,(uint8_t*)str1,strlen(str1),0x1000);
#endif
		ip_pkt->len=be16toword(len);
		ip_pkt->id = 0;
		ip_pkt->ts = 0;
		ip_pkt->verlen = 0x45;
		ip_pkt->fl_frg_of=0;
		ip_pkt->ttl=128;
		ip_pkt->cs = 0;
		ip_pkt->prt=IP_TCP;
		memcpy(ip_pkt->ipaddr_dst,ip_addr,4);
		memcpy(ip_pkt->ipaddr_src,ipaddr,4);
		ip_pkt->cs = checksum((void*)ip_pkt,sizeof(ip_pkt_ptr),0);
		//�������� ��������� Ethernet
		memcpy(frame->addr_dest,frame->addr_src,6);
		memcpy(frame->addr_src,macaddr,6);
		frame->type=ETH_IP;
		len+=sizeof(enc28j60_frame_ptr);
		enc28j60_packetSend((void*)frame,len);
		//���� ������ "Hello!!!", �� �������� �����

		uint16_t new_len = 0;
		int res = tcp_server_handler(ip_pkt, tcp_pkt, &new_len);
		if (res) {
			tcp_pkt->fl = TCP_ACK|TCP_PSH;
#ifdef ENC28J60_ENABLE_UART_DEBUG
			sprintf(str1,"hdr_len:%d\r\n",sizeof(tcp_pkt_ptr));
			HAL_UART_Transmit_MACRO(&huart1,(uint8_t*)str1,strlen(str1),0x1000);
#endif
			len = sizeof(tcp_pkt_ptr);
			tcp_pkt->len_hdr = len << 2;
			//len+=strlen((char*)tcp_pkt->data);
			len+=new_len;
			tcp_pkt->cs = 0;
			tcp_pkt->cs=checksum((uint8_t*)tcp_pkt-8, len+8, 2);
			//�������� ��������� ������ IP
			len+=sizeof(ip_pkt_ptr);
			ip_pkt->len=be16toword(len);
			ip_pkt->cs = 0;
			ip_pkt->cs = checksum((void*)ip_pkt,sizeof(ip_pkt_ptr),0);
			len+=sizeof(enc28j60_frame_ptr);
			enc28j60_packetSend((void*)frame,len);
		}
	}
  return res;
}
//--------------------------------------------------
uint8_t tcp_read(enc28j60_frame_ptr *frame, uint16_t len)
{
  uint8_t res=0;
	uint16_t len_data=0;
	uint16_t i=0;
	ip_pkt_ptr *ip_pkt = (void*)(frame->data);
	tcp_pkt_ptr *tcp_pkt = (void*)(ip_pkt->data);
	len_data = be16toword(ip_pkt->len)-20-(tcp_pkt->len_hdr>>2);
#ifdef ENC28J60_ENABLE_UART_DEBUG
	sprintf(str1,"%d.%d.%d.%d-%d.%d.%d.%d %d tcp\r\n",
		ip_pkt->ipaddr_src[0],ip_pkt->ipaddr_src[1],ip_pkt->ipaddr_src[2],ip_pkt->ipaddr_src[3],
		ip_pkt->ipaddr_dst[0],ip_pkt->ipaddr_dst[1],ip_pkt->ipaddr_dst[2],ip_pkt->ipaddr_dst[3], len_data);
	HAL_UART_Transmit_MACRO(&huart1,(uint8_t*)str1,strlen(str1),0x1000);
#endif
	//���� ���� ������, �� ������� �� � ������������ ���������
	if (len_data)
	{
		//for (i=0;i<len_data;i++)
		//{
		//	HAL_UART_Transmit_MACRO(&huart1,tcp_pkt->data+i,1,0x1000);
		//}
#ifdef ENC28J60_ENABLE_UART_DEBUG
		HAL_UART_Transmit_MACRO(&huart1,tcp_pkt->data,len_data,0x1000);
		HAL_UART_Transmit_MACRO(&huart1,(uint8_t*)"\r\n",2,0x1000);
#endif
		//���� ������� ���� �������������, �� ���������� ���� ������
		if (tcp_pkt->fl&TCP_ACK)
		{
			tcp_send(ip_pkt->ipaddr_src, be16toword(tcp_pkt->port_src), TCP_OP_ACK_OF_DATA);
		}
	}
	if (tcp_pkt->fl == TCP_SYN)
	{
		tcp_send(ip_pkt->ipaddr_src, be16toword(tcp_pkt->port_src), TCP_OP_SYNACK);
	}
	else if (tcp_pkt->fl == (TCP_FIN|TCP_ACK))
	{
		tcp_send(ip_pkt->ipaddr_src, be16toword(tcp_pkt->port_src), TCP_OP_ACK_OF_FIN);
	}
	else if (tcp_pkt->fl == (TCP_PSH|TCP_ACK))
	{
		//���� ������ ���
		if(!len_data)
		{
			tcp_send(ip_pkt->ipaddr_src, be16toword(tcp_pkt->port_src), TCP_OP_ACK_OF_FIN);
		}
	}
	else if (tcp_pkt->fl == TCP_ACK)
	{
#ifdef ENC28J60_ENABLE_UART_DEBUG
		HAL_UART_Transmit_MACRO(&huart1,(uint8_t*)"ACK\r\n",5,0x1000);
#endif
	}
  return res;
}

uint8_t tcp_server_handler(ip_pkt_ptr *ip_pkt, tcp_pkt_ptr *tcp_pkt, uint16_t *new_len) {

	tcp_stream_create_buffer();
	uint8_t bufferRX[ TCP_STREAM_BUFF_SIZE ];
	uint8_t bufferTX[ TCP_STREAM_BUFF_SIZE ];
	memset(bufferRX, 0, TCP_STREAM_BUFF_SIZE);
	memset(bufferTX, 0, TCP_STREAM_BUFF_SIZE);
	uint8_t result = 0;
	uint8_t length = 0;

	memcpy(bufferRX, tcp_pkt->data, TCP_STREAM_BUFF_SIZE);

	uint8_t sof_index = 0;
	uint8_t packet_size = 0;

	for (uint8_t i = 0; i < TCP_STREAM_BUFF_SIZE; ++i) {
		sof_index = i;
		if (bufferRX[i] == (char) 0xFF) {
			;
		} else {
			continue;
		}

		if ((sof_index + 1) <= TCP_STREAM_BUFF_SIZE) {
			packet_size = bufferRX[sof_index + 1];
		} else {
			break;
		}

		if ((sof_index + packet_size) <= TCP_STREAM_BUFF_SIZE) {
			length = bufferRX[sof_index + 1];
#ifdef SEGGER_DEBUG
			//SEGGER_RTT_printf(0, "TCP recv %d %d\n", sof_index, packet_size);
			//SEGGER_SYSVIEW_PrintfHost("TCP recv %d\n", length);
#endif
			i = sof_index + packet_size - 1;
			xStreamBufferSend(tcp_rx_stream_buffer, bufferRX+sof_index, length, 10);
		} else {
			break;
		}
	}

	length  = xStreamBufferReceive(tcp_tx_stream_buffer, (uint8_t*)bufferTX, sizeof(bufferTX), portMAX_DELAY);
	if ( length > 0 )
	{
#ifdef SEGGER_DEBUG
		//SEGGER_RTT_printf(0, "TCP send %d\n", length);
		//SEGGER_SYSVIEW_PrintfHost("TCP send %d\n", length);
#endif
		memcpy((char*)tcp_pkt->data, bufferTX, bufferTX[1]);
		if ((uint16_t)bufferTX[1] % 2 == 1) {
			*new_len = (uint16_t)bufferTX[1] + 1;
		} else {
			*new_len = (uint16_t)bufferTX[1];
		}

		result = 1;
	}

	return result;
}
//--------------------------------------------------
