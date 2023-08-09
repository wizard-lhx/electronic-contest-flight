#ifndef _DRV_UART_H_
#define _DRV_UART_H_
#include "sysconfig.h"

// 修改此宏定义可以在自己写的发送函数里调用底层的发送一字节函数
#define UartSendLXIMU	DrvUart5SendBuf
#define LX_Send_To_RasPi DrvUart2SendBuf
#define LX_Send_To_OpenMV_Front DrvUart1SendBuf
#define LX_Send_To_Car DrvUart3SendBuf

void DrvUart1Init(uint32_t baudrate);
void DrvUart1SendBuf(u8 *data, u8 len);
void DrvUart2Init(uint32_t baudrate);
void DrvUart2SendBuf(u8 *data, u8 len);
void DrvUart3Init(uint32_t baudrate);
void DrvUart3SendBuf(u8 *data, u8 len);
void DrvUart4Init(uint32_t baudrate);
void DrvUart4SendBuf(u8 *data, u8 len);
void DrvUart5Init(uint32_t baudrate);
void DrvUart5SendBuf(u8 *data, u8 len);

void DrvUartDataCheck(void);
#endif
