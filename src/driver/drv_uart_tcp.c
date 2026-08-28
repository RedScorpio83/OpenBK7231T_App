#include "../new_common.h"
#include "../new_pins.h"
#include "../new_cfg.h"
#include "../quicktick.h"
#include "../cmnds/cmd_public.h"
#include "../logging/logging.h"
#include "errno.h"
#include <lwip/sockets.h>
#include "drv_uart.h"

#if ENABLE_DRIVER_UART_TCP

#define DEFAULT_BUF_SIZE		512
#define DEFAULT_UART_TCP_PORT	8888
#define INVALID_SOCK			-1

extern int Main_HasWiFiConnected();
static int g_conn_channel = -1;
static int g_baudRate = 2400;
static int listen_sock = INVALID_SOCK;
static int client_sock = INVALID_SOCK;
static xTaskHandle g_trx_thread = NULL;
static bool g_utcp_running = false;

void UART_TCP_Deinit();

void UART_TCP_TRX_Thread(void* arg)
{
	int reuse = 1;
	unsigned char rx_buf[256];
	unsigned char tx_buf[256];
	struct sockaddr_in server_addr =
	{
		.sin_family = AF_INET,
		.sin_addr =
		{
			.s_addr = INADDR_ANY,
		},
		.sin_port = htons(DEFAULT_UART_TCP_PORT),
	};

	ADDLOG_INFO(LOG_FEATURE_DRV, "UART TCP Single-Thread Bridge starting on port %d", DEFAULT_UART_TCP_PORT);

	if(listen_sock != INVALID_SOCK) close(listen_sock);
	if(client_sock != INVALID_SOCK) close(client_sock);

	listen_sock = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
	if(listen_sock < 0)
	{
		ADDLOG_ERROR(LOG_FEATURE_DRV, "UART TCP Unable to create listen socket");
		goto exit_thread;
	}

	int flags = fcntl(listen_sock, F_GETFL, 0);
	fcntl(listen_sock, F_SETFL, flags | O_NONBLOCK);
	setsockopt(listen_sock, SOL_SOCKET, SO_REUSEADDR, (const char*)&reuse, sizeof(reuse));

	if(bind(listen_sock, (struct sockaddr*)&server_addr, sizeof(server_addr)) != 0)
	{
		ADDLOG_ERROR(LOG_FEATURE_DRV, "UART TCP bind failed");
		goto exit_thread;
	}

	if(listen(listen_sock, 2) != 0)
	{
		ADDLOG_ERROR(LOG_FEATURE_DRV, "UART TCP listen failed");
		goto exit_thread;
	}

	g_utcp_running = true;
	ADDLOG_INFO(LOG_FEATURE_DRV, "UART TCP Listening on port %d", DEFAULT_UART_TCP_PORT);

	while(g_utcp_running)
	{
		// 1. Poll incoming client connection
		if(client_sock == INVALID_SOCK)
		{
			struct sockaddr_storage source_addr;
			socklen_t addr_len = sizeof(source_addr);
			client_sock = accept(listen_sock, (struct sockaddr*)&source_addr, &addr_len);
			if(client_sock != INVALID_SOCK)
			{
				struct timeval tv;
				tv.tv_sec = 0;
				tv.tv_usec = 10000; // 10ms timeout
				setsockopt(client_sock, SOL_SOCKET, SO_RCVTIMEO, (const char*)&tv, sizeof(tv));
				setsockopt(client_sock, SOL_SOCKET, SO_SNDTIMEO, (const char*)&tv, sizeof(tv));
				int nodelay = 1;
				setsockopt(client_sock, IPPROTO_TCP, TCP_NODELAY, (const char*)&nodelay, sizeof(nodelay));

				if(g_conn_channel >= 0) CHANNEL_Set(g_conn_channel, 1, CHANNEL_SET_FLAG_SKIP_MQTT | CHANNEL_SET_FLAG_SILENT);
				ADDLOG_INFO(LOG_FEATURE_DRV, "UART TCP Client connected!");
			}
		}

		// 2. Bidirectional forwarding
		if(client_sock != INVALID_SOCK)
		{
			// TCP -> UART
			errno = 0;
			int r = recv(client_sock, rx_buf, sizeof(rx_buf), 0);
			if(r > 0)
			{
				for(int i = 0; i < r; i++)
				{
					UART_SendByte(rx_buf[i]);
				}
			}
			else if(r == 0)
			{
				ADDLOG_INFO(LOG_FEATURE_DRV, "UART TCP Client disconnected gracefully");
				close(client_sock);
				client_sock = INVALID_SOCK;
				if(g_conn_channel >= 0) CHANNEL_Set(g_conn_channel, 0, CHANNEL_SET_FLAG_SKIP_MQTT | CHANNEL_SET_FLAG_SILENT);
			}
			else
			{
				int err = errno;
				if(err != 0 && err != EAGAIN && err != EWOULDBLOCK && err != EINTR)
				{
					ADDLOG_INFO(LOG_FEATURE_DRV, "UART TCP Client error %d", err);
					close(client_sock);
					client_sock = INVALID_SOCK;
					if(g_conn_channel >= 0) CHANNEL_Set(g_conn_channel, 0, CHANNEL_SET_FLAG_SKIP_MQTT | CHANNEL_SET_FLAG_SILENT);
				}
			}

			// UART -> TCP
			if(client_sock != INVALID_SOCK)
			{
				int ulen = UART_GetDataSize();
				if(ulen > 0)
				{
					if(ulen > sizeof(tx_buf)) ulen = sizeof(tx_buf);
					for(int i = 0; i < ulen; i++)
					{
						tx_buf[i] = UART_GetByte(i);
					}
					UART_ConsumeBytes(ulen);
					errno = 0;
					int s_ret = send(client_sock, tx_buf, ulen, 0);
					if(s_ret < 0)
					{
						int err = errno;
						if(err != 0 && err != EAGAIN && err != EWOULDBLOCK && err != EINTR)
						{
							close(client_sock);
							client_sock = INVALID_SOCK;
							if(g_conn_channel >= 0) CHANNEL_Set(g_conn_channel, 0, CHANNEL_SET_FLAG_SKIP_MQTT | CHANNEL_SET_FLAG_SILENT);
						}
					}
				}
			}
		}

		// Yield 5ms
		rtos_delay_milliseconds(5);
	}

exit_thread:
	UART_TCP_Deinit();
	rtos_delete_thread(NULL);
}

void UART_TCP_Init()
{
	g_baudRate = Tokenizer_GetArgIntegerDefault(1, 2400);
	g_conn_channel = Tokenizer_GetArgIntegerDefault(3, -1);
	int flowcontrol = Tokenizer_GetArgIntegerDefault(4, 0);

	UART_InitUART(g_baudRate, 0, flowcontrol > 0 ? true : false);
	UART_InitReceiveRingBuffer(DEFAULT_BUF_SIZE * 2);

	if(g_trx_thread != NULL)
	{
		g_utcp_running = false;
		rtos_delay_milliseconds(50);
		rtos_delete_thread(&g_trx_thread);
		g_trx_thread = NULL;
	}

	OSStatus err = rtos_create_thread(&g_trx_thread, BEKEN_APPLICATION_PRIORITY,
		"UART_TCP_TRX",
		(beken_thread_function_t)UART_TCP_TRX_Thread,
		2048,
		(beken_thread_arg_t)0);
	if(err != kNoErr)
	{
		ADDLOG_ERROR(LOG_FEATURE_DRV, "create UART_TCP_TRX failed with %i!", err);
	}
}

void UART_TCP_Deinit()
{
	g_utcp_running = false;
	if(listen_sock != INVALID_SOCK)
	{
		close(listen_sock);
		listen_sock = INVALID_SOCK;
	}
	if(client_sock != INVALID_SOCK)
	{
		close(client_sock);
		client_sock = INVALID_SOCK;
	}
	if(g_conn_channel >= 0)
	{
		CHANNEL_Set(g_conn_channel, 0, CHANNEL_SET_FLAG_SKIP_MQTT | CHANNEL_SET_FLAG_SILENT);
	}
}

#endif
