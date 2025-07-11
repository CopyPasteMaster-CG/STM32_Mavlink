#include "COM.h"
#include "main.h"
// 函数原型
void COM_TX_task(void *argument);
void COM_RX_task(void *argument);
extern uint8_t rec_buf[64];
// COM口实体
COM_WITH_DMA_IDLE_t COM1;  // 接飞控
// COM_WITH_DMA_IDLE_t COM2;
// COM_WITH_DMA_IDLE_t COM3;
// COM_WITH_DMA_IDLE_t COM4;


// if (qian == 1) 
//		{
//        if (channels[0].state == 0) channels[0].state = 1;
//        if (channels[1].state == 0) channels[1].state = 1;
//    }
//    if (hou == 1)
//		{
//        if (channels[2].state == 0) channels[2].state = 1;
//        if (channels[3].state == 0) channels[3].state = 1;
//    }
//    if (zuo == 1) 
//		{
//        if (channels[0].state == 0) channels[0].state = 1;
//        if (channels[2].state == 0) channels[2].state = 1;
//    }
//    if (you == 1)
//		{
//        if (channels[1].state == 0) channels[1].state = 1;
//        if (channels[3].state == 0) channels[3].state = 1;
//    }
//    if (paotou1 == 1)
//		{
//        if (channels[0].state == 0) channels[0].state = 1;
//    }
//    if (paotou2 == 1)
//		{
//        if (channels[1].state == 0) channels[1].state = 1;
//    }
//    if (paotou3 == 1) 
//		{
//        if (channels[2].state == 0) channels[2].state = 1;
//    }
//    if (paotou4 == 1) 
//		{
//        if (channels[3].state == 0) channels[3].state = 1;
//    }
//    if (all == 1)
//		{
//        for (uint8_t i = 0; i < 4; i++) 
//				{
//            if (channels[i].state == 0) channels[i].state = 1;
//        }
//    }


/// @brief 初始化串口，使用DMA收发，接收为空闲中断。注意，Init_COM_with_DMA_idle必须尽早调用，前面不能加延时，否则串口接收异常
/// @param p_COM 指向COM口实体的指针
/// @param COM_name 串口的名称，格式“COMx”，只允许有4个字节
/// @param huart 串口句柄
/// @param hdma_usart_rx 接收DMA句柄
/// @param decode_rx_byte 解帧处理回调函数
/// @return 初始化是否成功
bool Init_COM_with_DMA_idle(COM_WITH_DMA_IDLE_t *p_COM, char *COM_name, UART_HandleTypeDef *huart, DMA_HandleTypeDef *hdma_usart_rx, void (*decode_rx_byte)(uint8_t byte))
{
  // 输入参数检查
  if(strlen(COM_name) != 4)  // COM_name只允许有4个字节，即COMx
  {
    //SEGGER_RTT_printf(0, RTT_CTRL_TEXT_RED "\n Init COM failed, COM name length error! Should be 4\n");
    return false;
  }

  // 结构体数据清零
  memset(p_COM, 0, sizeof(COM_WITH_DMA_IDLE_t));

  // 初始化串口变量
  p_COM->huart = huart;
  p_COM->hdma_usart_rx = hdma_usart_rx;
  p_COM->TX_IDLE_semHandle = xSemaphoreCreateBinary();
  p_COM->TX_QueueHandle = xQueueCreate(COM_TX_BUF_DEPTH, sizeof(uint8_t));
  p_COM->RX_QueueHandle = xQueueCreate(COM_RX_BUF_DEPTH, sizeof(uint8_t));
  p_COM->decode_rx_byte = decode_rx_byte;
  xSemaphoreGive(p_COM->TX_IDLE_semHandle); // 信号量刚创建默认是没有被give的

  // 创建串口发送处理任务
  char COM_TX_task_name[] = "COMx_TX_TASK";
  COM_TX_task_name[3] = COM_name[3];
  BaseType_t pd =xTaskCreate(COM_TX_task,
              COM_TX_task_name,
              configMINIMAL_STACK_SIZE * 1,
              (void *)p_COM,
              tskIDLE_PRIORITY + 0,
              NULL);
			if(pd==pdPASS)
			printf("send_ok\r\n");
			
			printf("Free heap: %lu bytes\n", xPortGetFreeHeapSize());
			
  // 创建串口接收处理任务
  char COM_RX_task_name[] = "COMx_RX_TASK";
  COM_RX_task_name[3] = COM_name[3];
  pd =xTaskCreate(COM_RX_task,
              COM_RX_task_name,
              configMINIMAL_STACK_SIZE * 1,
              (void *)p_COM,
              tskIDLE_PRIORITY + 0,
              NULL);
							
			if(pd==pdPASS )
			printf("rec_ok\r\n");
  // 使能串口DMA空闲中断接收
  HAL_UARTEx_ReceiveToIdle_DMA(p_COM->huart, p_COM->RX_buffer, COM_RX_BUF_DEPTH);
  __HAL_DMA_DISABLE_IT(p_COM->hdma_usart_rx, DMA_IT_HT);  // 不使用DMA自身的中断

  return true;
}

/// @brief 通过串口发送一串数据
/// @param p_COM 指向COM口实体的指针
/// @param data 指向要发送的数据的指针
/// @param data_length 要发送的数据的字节数
/// @return 是否发送成功
bool COM_send_data(COM_WITH_DMA_IDLE_t *p_COM, uint8_t* data, uint16_t data_length)
{
  // 判断发送队列是否有足够的空间
  if(uxQueueSpacesAvailable(p_COM->TX_QueueHandle) < data_length)
  {
    printf("COM send data failed, queue spaces not enough!\r\n");
		//SEGGER_RTT_printf(0, RTT_CTRL_TEXT_RED "\n COM send data failed, queue spaces not enough!\n");
    return false;
  }

  // 发送队列空间充足，则将要发送的数据放入发送队列
  for (int i = 0; i < data_length; i++)
  {
    xQueueSend(p_COM->TX_QueueHandle, data + i, 0);
  }
  
  return true;
}

/// @brief 串口数据发送处理任务
/// @param argument 指向COM口实体的指针
void COM_TX_task(void *argument)
{
  COM_WITH_DMA_IDLE_t *p_COM = (COM_WITH_DMA_IDLE_t *)argument;

  BaseType_t xResult;
  for (;;)
  {
    xResult = xSemaphoreTake(p_COM->TX_IDLE_semHandle, portMAX_DELAY); // 如果正在发送数据，阻塞在此处
    if (xResult == pdTRUE)                                             // 当前没有正在发送数据
    {
      if (uxQueueMessagesWaiting(p_COM->TX_QueueHandle)) // 发送队列中有数据
      {
        int counter = 0;
        while (xQueueReceive(p_COM->TX_QueueHandle, p_COM->TX_buffer + counter, 0)) // 提取发送队列中的数据到发送缓存中
        {
          counter++;
          if (counter >= COM_TX_BUF_DEPTH)
            break;
        }
        HAL_UART_Transmit_DMA(p_COM->huart, p_COM->TX_buffer, counter); // 将数据发送出去
      }
      else // 队列中无数据
      {
        xSemaphoreGive(p_COM->TX_IDLE_semHandle); // 归还信号量
        vTaskDelay(1);                            // 延迟1ms，1ms后再来检查
      }
    }
  }
}

/// @brief 串口数据接收处理任务
/// @param argument 指向COM口实体的指针
void COM_RX_task(void *argument)
{
  COM_WITH_DMA_IDLE_t *p_COM = (COM_WITH_DMA_IDLE_t *)argument;

  uint8_t rx_byte;

  for (;;)
  {
    xQueueReceive(p_COM->RX_QueueHandle, &rx_byte, portMAX_DELAY); // 从队列中取出一个字节的数据，如果队列中没有数据就阻塞
    (*(p_COM->decode_rx_byte))(rx_byte);                           // 解帧
  }
}

/// @brief 串口接收空闲中断
/// @param huart 串口
/// @param Size 接收到的数据字节数
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{

  COM_WITH_DMA_IDLE_t *p_COM = NULL;
  static signed portBASE_TYPE xHigherPriorityTaskWoken;
  if (huart->Instance == USART2)
  {
    p_COM = &COM1;
  }
  // else if (huart->Instance == USART2)
  // {
  //   p_COM = &COM2;
  // }
  // else if (huart->Instance == USART3)
  // {
  //   p_COM = &COM3;
  // }
  // else if (huart->Instance == UART4)
  // {
  //   p_COM = &COM4;
  // }

  // 将接收缓存中的数据放入接收队列中
  for (int i = 0; i < Size; i++)
  {
//		
//     if (p_COM == &COM1)
//     {
//       
//			 printf("0x%02X\r", p_COM->RX_buffer[i]);
//     }
		//printf("0x%02X\r", p_COM->RX_buffer[i]);
    xQueueSendFromISR(p_COM->RX_QueueHandle, p_COM->RX_buffer + i, &xHigherPriorityTaskWoken);
  }


	//HAL_UART_Transmit(p_COM->huart, p_COM->RX_buffer, Size, 0xffff);
  // 重新使能串口DMA接收空闲中断
  HAL_UARTEx_ReceiveToIdle_DMA(p_COM->huart, p_COM->RX_buffer, COM_RX_BUF_DEPTH);
  __HAL_DMA_DISABLE_IT(p_COM->hdma_usart_rx, DMA_IT_HT);  // 不使用DMA自身的中断
  // 任务切换
  portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}




/// @brief 串口接收中断处理函数
/// @param UartHandle 串口实例
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *UartHandle) // 串口接收中断处理函数
{
		
}

/// @brief 串口发送完成中断
/// @param UartHandle 串口句柄
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *UartHandle)
{
  static signed portBASE_TYPE xHigherPriorityTaskWoken;

  if (UartHandle->Instance == USART2)
  {
    xSemaphoreGiveFromISR(COM1.TX_IDLE_semHandle, &xHigherPriorityTaskWoken);
  }
  // else if(UartHandle->Instance == USART2)
  // {
  //   xSemaphoreGiveFromISR(COM2.TX_IDLE_semHandle, &xHigherPriorityTaskWoken);
  // }
  // else if (UartHandle->Instance == USART3)
  // {
  //   xSemaphoreGiveFromISR(COM3.TX_IDLE_semHandle, &xHigherPriorityTaskWoken);
  // }
  // else if (UartHandle->Instance == UART4)
  // {
  //   xSemaphoreGiveFromISR(COM4.TX_IDLE_semHandle, &xHigherPriorityTaskWoken);
  // }

  // 任务切换
  portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *UartHandle)
{
  COM_WITH_DMA_IDLE_t *p_COM = NULL;

  if (UartHandle->Instance == USART2)
  {
    __HAL_UNLOCK(&huart1);  // 解锁串口

    // 重新使能串口DMA接收空闲中断
    p_COM = &COM1;
    HAL_UARTEx_ReceiveToIdle_DMA(p_COM->huart, p_COM->RX_buffer, COM_RX_BUF_DEPTH);
    __HAL_DMA_DISABLE_IT(p_COM->hdma_usart_rx, DMA_IT_HT);  // 不使用DMA自身的中断
  }
  // else if (UartHandle->Instance == USART2)
  // {
  //   __HAL_UNLOCK(&huart2);  // 解锁串口

  //   // 重新使能串口DMA接收空闲中断
  //   p_COM = &COM2;
  //   HAL_UARTEx_ReceiveToIdle_DMA(p_COM->huart, p_COM->RX_buffer, COM_RX_BUF_DEPTH);
  //   __HAL_DMA_DISABLE_IT(p_COM->hdma_usart_rx, DMA_IT_HT);  // 不使用DMA自身的中断
  // }
  // else if (UartHandle->Instance == USART3)
  // {
  //   __HAL_UNLOCK(&huart3);  // 解锁串口

  //   // 重新使能串口DMA接收空闲中断
  //   p_COM = &COM3;
  //   HAL_UARTEx_ReceiveToIdle_DMA(p_COM->huart, p_COM->RX_buffer, COM_RX_BUF_DEPTH);
  //   __HAL_DMA_DISABLE_IT(p_COM->hdma_usart_rx, DMA_IT_HT);  // 不使用DMA自身的中断
  // }
  // else if (UartHandle->Instance == UART4)
  // {
  //   __HAL_UNLOCK(&huart4);  // 解锁串口

  //   // 重新使能串口DMA接收空闲中断
  //   p_COM = &COM4;
  //   HAL_UARTEx_ReceiveToIdle_DMA(p_COM->huart, p_COM->RX_buffer, COM_RX_BUF_DEPTH);
  //   __HAL_DMA_DISABLE_IT(p_COM->hdma_usart_rx, DMA_IT_HT);  // 不使用DMA自身的中断
  // }
}
