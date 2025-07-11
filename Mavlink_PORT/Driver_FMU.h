#ifndef __DRIVER_FMU_H__
#define __DRIVER_FMU_H__

#include "main.h"
#include "COM.h"

/// @brief 计算通信频率
typedef struct _FREQ_calc_
{
	uint32_t last_got_frame_time_ms;  // 最后一次收到此帧的时间，单位：ms
	uint32_t period_ms;  // 周期
	float freq_Hz;  // 频率
}FREQ_calc_t;



#define OPEN_INTERVAL 100
#define CLOSE_INTERVAL 500

typedef struct 
 {
	uint8_t enable_switch;   
	uint8_t open_all;      
	uint8_t front;    
	uint8_t rear;     
	uint8_t left;    
	uint8_t right;    
	uint8_t turret1;  
	uint8_t turret2; 
	uint8_t turret3;  
	uint8_t turret4; 
} local_paotou_mavlink_t;



typedef struct 
{
  uint8_t state;          // 0=关闭,1-4=闪烁,5=持续
  uint32_t start_time;    // 状态启动时间
	
  GPIO_TypeDef* port;     // 输出端口
  uint16_t pin;           // 输出引脚
} ChannelCtl;
void process_channels();
void update_freq_calc(FREQ_calc_t *p_freq_calc);
void switch_pin(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin);
void FMU_Init(void);
void FMU_decode(uint8_t data_in);
void FMU_task(void *argument);
void send_paotou_task();
void send_paotou_stream(local_paotou_mavlink_t local_msg );
#endif
