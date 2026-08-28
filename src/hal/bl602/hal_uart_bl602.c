#ifdef PLATFORM_BL602

#include "../../new_pins.h"
#include "../../new_cfg.h"
#include "../../cmnds/cmd_public.h"
#include "../../cmnds/cmd_local.h"
#include "../../logging/logging.h"

#include "../hal_uart.h"
#include <vfs.h>
#include <device/vfs_uart.h>
#include <bl_uart.h>
#include <bl602_uart.h>
#include <aos/yloop.h>

uint8_t g_id = 1;
int fd_console = -1;

static void console_cb_read(int fd, void* param)
{
	char buffer[64];
	int ret;
	int i;

	ret = aos_read(fd, buffer, sizeof(buffer));
	if(ret > 0)
	{
		for(i = 0; i < ret; i++)
		{
			UART_AppendByteToReceiveRingBuffer(buffer[i]);
		}
	}
}

void HAL_UART_SendByte(byte b)
{
	if(fd_console >= 0)
	{
		aos_write(fd_console, &b, 1);
	}
}

int HAL_UART_Init(int baud, int parity, bool hwflowc, int txOverride, int rxOverride)
{
	if(fd_console < 0)
	{
		if(CFG_HasFlag(OBK_FLAG_USE_SECONDARY_UART))
		{
			fd_console = aos_open("/dev/ttyS1", 0);
		}
		else
		{
			fd_console = aos_open("/dev/ttyS0", 0);
		}
		if(fd_console >= 0)
		{
			aos_ioctl(fd_console, IOCTL_UART_IOC_BAUD_MODE, baud);
			aos_poll_read_fd(fd_console, console_cb_read, (void*)0x12345678);
		}
	}
	return 1;
}

#endif
