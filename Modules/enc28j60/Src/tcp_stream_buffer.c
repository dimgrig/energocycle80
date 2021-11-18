#include "tcp_stream_buffer.h"

StreamBufferHandle_t tcp_rx_stream_buffer = NULL;
StreamBufferHandle_t tcp_tx_stream_buffer = NULL;

__inline void tcp_stream_create_buffer( void )
{
    if (tcp_rx_stream_buffer == NULL) {
    	tcp_rx_stream_buffer = xStreamBufferCreate(TCP_STREAM_BUFF_SIZE, 1);
    }
    if (tcp_tx_stream_buffer == NULL) {
    	tcp_tx_stream_buffer = xStreamBufferCreate(TCP_STREAM_BUFF_SIZE, 1);
    }
}
