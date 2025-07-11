#ifndef __COM_H__
#define __COM_H__
#include <stdbool.h>
#include "main.h"
#include "usart.h"
#include "FreeRTOS.h"
#include "cmsis_os.h"
#define COM_TX_BUF_DEPTH 128
#define COM_RX_BUF_DEPTH 256

typedef struct
{
  UART_HandleTypeDef *huart;
  DMA_HandleTypeDef *hdma_usart_rx;

  // 发送相关变量
  SemaphoreHandle_t TX_IDLE_semHandle; // 发送空闲标志
  QueueHandle_t TX_QueueHandle;        // 发送队列
  uint8_t TX_buffer[COM_TX_BUF_DEPTH]; // 发送缓存

  // 接收相关变量
  QueueHandle_t RX_QueueHandle;        // 接收队列
  uint8_t RX_buffer[COM_RX_BUF_DEPTH]; // 接收缓存
  uint16_t RX_data_counter;            // 接收到的数据字节数
  void (*decode_rx_byte)(uint8_t byte); // 串口接收解帧函数
} COM_WITH_DMA_IDLE_t;  // 串口，启用DMA收发和接收空闲中断

bool Init_COM_with_DMA_idle(COM_WITH_DMA_IDLE_t *p_COM, char *COM_name, UART_HandleTypeDef *huart, DMA_HandleTypeDef *hdma_usart_rx, void (*decode_rx_byte)(uint8_t byte));
bool COM_send_data(COM_WITH_DMA_IDLE_t *p_COM, uint8_t* data, uint16_t data_length);

extern COM_WITH_DMA_IDLE_t COM1;  // 接飞控
// extern COM_WITH_DMA_IDLE_t COM2;
// extern COM_WITH_DMA_IDLE_t COM3;
// extern COM_WITH_DMA_IDLE_t COM4;

#endif
