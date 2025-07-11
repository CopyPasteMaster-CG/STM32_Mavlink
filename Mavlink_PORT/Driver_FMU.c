#include "Driver_FMU.h"
#include "mavlink.h"
#include "usart.h"
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
extern DMA_HandleTypeDef hdma_usart2_rx;
extern DMA_HandleTypeDef hdma_usart1_tx;

#define MAVLINK_COM COM1  // 接mavlink协议的串口
#define MAVLINK_SYSTEM_ID 1
#define MAVLINK_COMPONENT_ID 0

// 全局变量
bool already_got_heartbeat = false;  // 是否已经收到mavlink心跳帧
bool already_request_mav_stream = false;  // 是否已经请求数据流

mavlink_heartbeat_t heartbeat;
mavlink_gps_raw_int_t gps_raw_int;
mavlink_attitude_t attitude;
mavlink_global_position_int_t global_position_int;

mavlink_paotou_mavlink_t paotou_mavlink;

FREQ_calc_t freq_calc_heartbeat;
FREQ_calc_t freq_calc_gps_raw_int;
FREQ_calc_t freq_calc_attitude;
FREQ_calc_t freq_calc_global_position_int;
FREQ_calc_t freq_calc_paotou_mavlink;

/// @brief 更新频率计算结构体
/// @param p_freq_calc 频率计算结构体
void update_freq_calc(FREQ_calc_t *p_freq_calc)
{
	p_freq_calc->period_ms = xTaskGetTickCount() - p_freq_calc->last_got_frame_time_ms;
	if (p_freq_calc->period_ms == 0)
	{
		p_freq_calc->freq_Hz = 0;
	}
	else
	{
		p_freq_calc->freq_Hz = 1000.0f / (float)p_freq_calc->period_ms;
	}

	p_freq_calc->last_got_frame_time_ms = xTaskGetTickCount();
}

uint8_t current_flight_mode = 0;  // 当前飞行模式
        // STABILIZE =     0,  // manual airframe angle with manual throttle
        // ACRO =          1,  // manual body-frame angular rate with manual throttle
        // ALT_HOLD =      2,  // manual airframe angle with automatic throttle
        // AUTO =          3,  // fully automatic waypoint control using mission commands
        // GUIDED =        4,  // fully automatic fly to coordinate or fly at velocity/direction using GCS immediate commands
        // LOITER =        5,  // automatic horizontal acceleration with automatic throttle
        // RTL =           6,  // automatic return to launching point
        // CIRCLE =        7,  // automatic circular flight with automatic throttle
        // LAND =          9,  // automatic landing with horizontal position control
        // DRIFT =        11,  // semi-autonomous position, yaw and throttle control
        // SPORT =        13,  // manual earth-frame angular rate control with manual throttle
        // FLIP =         14,  // automatically flip the vehicle on the roll axis
        // AUTOTUNE =     15,  // automatically tune the vehicle's roll and pitch gains
        // POSHOLD =      16,  // automatic position hold with manual override, with automatic throttle
        // BRAKE =        17,  // full-brake using inertial/GPS system, no pilot input
        // THROW =        18,  // throw to launch mode using inertial/GPS system, no pilot input
        // AVOID_ADSB =   19,  // automatic avoidance of obstacles in the macro scale - e.g. full-sized aircraft
        // GUIDED_NOGPS = 20,  // guided mode but only accepts attitude and altitude
        // SMART_RTL =    21,  // SMART_RTL returns to home by retracing its steps
        // FLOWHOLD  =    22,  // FLOWHOLD holds position with optical flow without rangefinder
        // FOLLOW    =    23,  // follow attempts to follow another vehicle or ground station
        // ZIGZAG    =    24,  // ZIGZAG mode is able to fly in a zigzag manner with predefined point A and point B
        // SYSTEMID  =    25,  // System ID mode produces automated system identification signals in the controllers
        // AUTOROTATE =   26,  // Autonomous autorotation
        // AUTO_RTL =     27,  // Auto RTL, this is not a true mode, AUTO will report as this mode if entered to perform a DO_LAND_START Landing sequence
        // TURTLE =       28,  // Flip over after crash

uint8_t gnss_fix_type = 0;  // 当前定位状态
        // NO_GPS = 0,                  // No GPS connected/detected
        // NO_FIX = 1,                  // Receiving valid GPS messages but no lock
        // GPS_OK_FIX_2D = 2,           // Receiving valid messages and 2D lock
        // GPS_OK_FIX_3D = 3,           // Receiving valid messages and 3D lock
        // GPS_OK_FIX_3D_DGPS = 4,      // Receiving valid messages and 3D lock with differential improvements
        // GPS_OK_FIX_3D_RTK_FLOAT = 5, // Receiving valid messages and 3D RTK Float
        // GPS_OK_FIX_3D_RTK_FIXED = 6, // Receiving valid messages and 3D RTK Fixed

uint8_t gnss_sat_num = 0;  // 参与定位卫星数
uint16_t gnss_hdop = 0xFFFF;  // GNSS 水平定位因子

float roll_deg = 0;  // 横滚角，单位：度
float pitch_deg = 0;  // 俯仰角，单位：度
float yaw_deg = 0;  // 航向角，单位：度，范围：-180度~180度。正北为0，正东为90度，正西为-90度

int32_t latitude = 0;  // 维度，单位：原始值（度）乘以10的7次方
int32_t longitude = 0;  // 经度，单位：原始值（度）乘以10的7次方

int32_t alt_msl_mm = 0;  // 海拔高度，单位：mm
int32_t alt_above_home_mm = 0;  // 相对于home点的高度，单位：mm

int16_t speed_north_cm_s = 0;  // 北向速度，单位：cm/s
int16_t speed_east_cm_s = 0;  // 东向速度，单位：cm/s
int16_t speed_down_cm_s = 0;  // 垂直速度，单位：cm/s，注意，竖直向下为正


volatile uint8_t enable = 0;
volatile uint8_t qian = 0;
volatile uint8_t hou = 0;
volatile uint8_t zuo = 0;
volatile uint8_t you = 0;
volatile uint8_t paotou1 = 0;
volatile uint8_t paotou2 = 0;
volatile uint8_t paotou3 = 0;
volatile uint8_t paotou4 = 0;
volatile uint8_t all = 0;


ChannelCtl channels[4] = 
{
    {0, 0, OUT1_GPIO_Port, OUT1_Pin},
    {0, 0, OUT2_GPIO_Port, OUT2_Pin},
    {0, 0, OUT3_GPIO_Port, OUT3_Pin},
    {0, 0, OUT4_GPIO_Port, OUT4_Pin}
};

uint8_t number=0;

uint8_t state[4]={1,1,1,1};  


//void process_channels()
//{
//	if (enable != 1)
//	return;
//	uint8_t i;
//	uint8_t temp=return_switch();
//		switch (temp)
//			{
//			case 1:
//				if(state[0]==1&&state[1]==1)
//				{
//					HAL_GPIO_WritePin(OUT1_GPIO_Port,OUT1_Pin,GPIO_PIN_SET);
//					HAL_GPIO_WritePin(OUT2_GPIO_Port,OUT2_Pin,GPIO_PIN_SET);
//					vTaskDelay(OPEN_INTERVAL);
//					HAL_GPIO_WritePin(OUT1_GPIO_Port,OUT1_Pin,GPIO_PIN_RESET);
//					HAL_GPIO_WritePin(OUT2_GPIO_Port,OUT2_Pin,GPIO_PIN_RESET);
//					vTaskDelay(CLOSE_INTERVAL);
//					HAL_GPIO_WritePin(OUT1_GPIO_Port,OUT1_Pin,GPIO_PIN_SET);
//					HAL_GPIO_WritePin(OUT2_GPIO_Port,OUT2_Pin,GPIO_PIN_SET);
//					vTaskDelay(OPEN_INTERVAL);
//					HAL_GPIO_WritePin(OUT1_GPIO_Port,OUT1_Pin,GPIO_PIN_RESET);
//					HAL_GPIO_WritePin(OUT2_GPIO_Port,OUT2_Pin,GPIO_PIN_RESET);
//					state[0]=state[1]=0;
//				}
//				break;
//			case 2:
//				if(state[2]==1&&state[3]==1)
//				{
//					HAL_GPIO_WritePin(OUT3_GPIO_Port,OUT3_Pin,GPIO_PIN_SET);
//					HAL_GPIO_WritePin(OUT4_GPIO_Port,OUT4_Pin,GPIO_PIN_SET);
//					vTaskDelay(OPEN_INTERVAL);
//					HAL_GPIO_WritePin(OUT3_GPIO_Port,OUT3_Pin,GPIO_PIN_RESET);
//					HAL_GPIO_WritePin(OUT4_GPIO_Port,OUT4_Pin,GPIO_PIN_RESET);
//					vTaskDelay(CLOSE_INTERVAL);
//					HAL_GPIO_WritePin(OUT3_GPIO_Port,OUT3_Pin,GPIO_PIN_SET);
//					HAL_GPIO_WritePin(OUT4_GPIO_Port,OUT4_Pin,GPIO_PIN_SET);
//					vTaskDelay(OPEN_INTERVAL);
//					HAL_GPIO_WritePin(OUT3_GPIO_Port,OUT3_Pin,GPIO_PIN_RESET);
//					HAL_GPIO_WritePin(OUT4_GPIO_Port,OUT4_Pin,GPIO_PIN_RESET);
//					state[2]=state[3]=0;
//				}
//				break;
//			case 3:
//				if(state[0]==1&&state[2]==1)
//				{
//					HAL_GPIO_WritePin(OUT1_GPIO_Port,OUT1_Pin,GPIO_PIN_SET);
//					HAL_GPIO_WritePin(OUT3_GPIO_Port,OUT3_Pin,GPIO_PIN_SET);
//					vTaskDelay(OPEN_INTERVAL);
//					HAL_GPIO_WritePin(OUT1_GPIO_Port,OUT1_Pin,GPIO_PIN_RESET);
//					HAL_GPIO_WritePin(OUT3_GPIO_Port,OUT3_Pin,GPIO_PIN_RESET);
//					vTaskDelay(CLOSE_INTERVAL);
//					HAL_GPIO_WritePin(OUT1_GPIO_Port,OUT1_Pin,GPIO_PIN_SET);
//					HAL_GPIO_WritePin(OUT3_GPIO_Port,OUT3_Pin,GPIO_PIN_SET);
//					vTaskDelay(OPEN_INTERVAL);
//					HAL_GPIO_WritePin(OUT1_GPIO_Port,OUT1_Pin,GPIO_PIN_RESET);
//					HAL_GPIO_WritePin(OUT3_GPIO_Port,OUT3_Pin,GPIO_PIN_RESET);
//					state[0]=state[2]=0;
//				}
//				break;
//			case 4:
//				if(state[1]==1&&state[3]==1)
//				{
//					HAL_GPIO_WritePin(OUT2_GPIO_Port,OUT2_Pin,GPIO_PIN_SET);
//					HAL_GPIO_WritePin(OUT4_GPIO_Port,OUT4_Pin,GPIO_PIN_SET);
//					vTaskDelay(OPEN_INTERVAL);
//					HAL_GPIO_WritePin(OUT2_GPIO_Port,OUT2_Pin,GPIO_PIN_RESET);
//					HAL_GPIO_WritePin(OUT4_GPIO_Port,OUT4_Pin,GPIO_PIN_RESET);
//					vTaskDelay(CLOSE_INTERVAL);
//					HAL_GPIO_WritePin(OUT2_GPIO_Port,OUT2_Pin,GPIO_PIN_SET);
//					HAL_GPIO_WritePin(OUT4_GPIO_Port,OUT4_Pin,GPIO_PIN_SET);
//					vTaskDelay(OPEN_INTERVAL);
//					HAL_GPIO_WritePin(OUT2_GPIO_Port,OUT2_Pin,GPIO_PIN_RESET);
//					HAL_GPIO_WritePin(OUT4_GPIO_Port,OUT4_Pin,GPIO_PIN_RESET);
//					state[1]=state[3]=0;
//				}
//				break;
//			case 5:
//				if(state[0]==1)
//				{
//					HAL_GPIO_WritePin(OUT1_GPIO_Port,OUT1_Pin,GPIO_PIN_SET);
//					vTaskDelay(OPEN_INTERVAL);
//					HAL_GPIO_WritePin(OUT1_GPIO_Port,OUT1_Pin,GPIO_PIN_RESET);
//					vTaskDelay(CLOSE_INTERVAL);
//					HAL_GPIO_WritePin(OUT1_GPIO_Port,OUT1_Pin,GPIO_PIN_SET);
//					vTaskDelay(OPEN_INTERVAL);
//					HAL_GPIO_WritePin(OUT1_GPIO_Port,OUT1_Pin,GPIO_PIN_RESET);
//					state[0]=0;
//				}
//				break;
//			case 6:
//				if(state[1]==1)
//				{
//					HAL_GPIO_WritePin(OUT2_GPIO_Port,OUT2_Pin,GPIO_PIN_SET);
//					vTaskDelay(OPEN_INTERVAL);
//					HAL_GPIO_WritePin(OUT2_GPIO_Port,OUT2_Pin,GPIO_PIN_RESET);
//					vTaskDelay(CLOSE_INTERVAL);
//					HAL_GPIO_WritePin(OUT2_GPIO_Port,OUT2_Pin,GPIO_PIN_SET);
//					vTaskDelay(OPEN_INTERVAL);
//					HAL_GPIO_WritePin(OUT2_GPIO_Port,OUT2_Pin,GPIO_PIN_RESET);
//					state[1]=0;
//				}
//				break;
//			case 7:
//				if(state[2]==1)
//				{
//					HAL_GPIO_WritePin(OUT3_GPIO_Port,OUT3_Pin,GPIO_PIN_SET);
//					vTaskDelay(OPEN_INTERVAL);
//					HAL_GPIO_WritePin(OUT3_GPIO_Port,OUT3_Pin,GPIO_PIN_RESET);
//					vTaskDelay(CLOSE_INTERVAL);
//					HAL_GPIO_WritePin(OUT3_GPIO_Port,OUT3_Pin,GPIO_PIN_SET);
//					vTaskDelay(OPEN_INTERVAL);
//					HAL_GPIO_WritePin(OUT3_GPIO_Port,OUT3_Pin,GPIO_PIN_RESET);
//					state[2]=0;
//				}
//				break;
//			case 8:
//				if(state[3]==1)
//				{
//				HAL_GPIO_WritePin(OUT4_GPIO_Port,OUT4_Pin,GPIO_PIN_SET);
//				vTaskDelay(OPEN_INTERVAL);
//				HAL_GPIO_WritePin(OUT4_GPIO_Port,OUT4_Pin,GPIO_PIN_RESET);
//				vTaskDelay(CLOSE_INTERVAL);
//				HAL_GPIO_WritePin(OUT4_GPIO_Port,OUT4_Pin,GPIO_PIN_SET);
//				vTaskDelay(OPEN_INTERVAL);
//				HAL_GPIO_WritePin(OUT4_GPIO_Port,OUT4_Pin,GPIO_PIN_RESET);
//				state[3]=0;
//				}
//				break;
//			case 9:
//				if(state[0]==1&&state[1]==1&&state[2]==1&&state[3]==1)
//				{
//				HAL_GPIO_WritePin(OUT2_GPIO_Port,OUT2_Pin,GPIO_PIN_SET);
//				HAL_GPIO_WritePin(OUT4_GPIO_Port,OUT4_Pin,GPIO_PIN_SET);
//				HAL_GPIO_WritePin(OUT1_GPIO_Port,OUT1_Pin,GPIO_PIN_SET);
//				HAL_GPIO_WritePin(OUT3_GPIO_Port,OUT3_Pin,GPIO_PIN_SET);
//				vTaskDelay(OPEN_INTERVAL);
//				HAL_GPIO_WritePin(OUT2_GPIO_Port,OUT2_Pin,GPIO_PIN_RESET);
//				HAL_GPIO_WritePin(OUT4_GPIO_Port,OUT4_Pin,GPIO_PIN_RESET);
//				HAL_GPIO_WritePin(OUT1_GPIO_Port,OUT1_Pin,GPIO_PIN_RESET);
//				HAL_GPIO_WritePin(OUT3_GPIO_Port,OUT3_Pin,GPIO_PIN_RESET);
//				vTaskDelay(CLOSE_INTERVAL);
//				HAL_GPIO_WritePin(OUT2_GPIO_Port,OUT2_Pin,GPIO_PIN_SET);
//				HAL_GPIO_WritePin(OUT4_GPIO_Port,OUT4_Pin,GPIO_PIN_SET);
//				HAL_GPIO_WritePin(OUT1_GPIO_Port,OUT1_Pin,GPIO_PIN_SET);
//				HAL_GPIO_WritePin(OUT3_GPIO_Port,OUT3_Pin,GPIO_PIN_SET);
//				vTaskDelay(OPEN_INTERVAL);
//				HAL_GPIO_WritePin(OUT2_GPIO_Port,OUT2_Pin,GPIO_PIN_RESET);
//				HAL_GPIO_WritePin(OUT4_GPIO_Port,OUT4_Pin,GPIO_PIN_RESET);
//				HAL_GPIO_WritePin(OUT1_GPIO_Port,OUT1_Pin,GPIO_PIN_RESET);
//				HAL_GPIO_WritePin(OUT3_GPIO_Port,OUT3_Pin,GPIO_PIN_RESET);
//				state[0]=state[1]=state[2]=state[3]=0;
//				}
//				break;
//		}
//}

void process_channels()
{
	if (enable ==0)
		return;
	
	if (all !=number)
	{
    if (qian == 1) 
		{
      channels[0].state = 1;
			channels[1].state = 1;
    }
    if (hou == 1)
		{
         channels[2].state = 1;
         channels[3].state = 1;
    }
    if (zuo == 1) 
		{
         channels[0].state = 1;
         channels[2].state = 1;
    }
    if (you == 1)
		{
         channels[1].state = 1;
         channels[3].state = 1;
    }
    if (paotou1 == 1)
		{
         channels[0].state = 1;
    }
    if (paotou2 == 1)
		{
       channels[1].state = 1;
    }
    if (paotou3 == 1) 
		{
        channels[2].state = 1;
    }
    if (paotou4 == 1) 
		{
         channels[3].state = 1;
    }
		
//    if (all == 1)
//		{
//        for (uint8_t i = 0; i < 4; i++) 
//				{
//            channels[i].state = 1;
//        }
//    }
		
		number=all;
	}
		
	for (uint8_t i = 0; i < 4; i++)
	{
		uint32_t now = HAL_GetTick(); 
		uint32_t elapsed = now - channels[i].start_time;
		if(channels[i].state==5)
			continue;
			switch (channels[i].state)
			{
			case 1:
				HAL_GPIO_WritePin(channels[i].port, channels[i].pin, GPIO_PIN_SET);
				channels[i].state = 2;
				channels[i].start_time = now;
				break;
			case 2:
				if (elapsed >= OPEN_INTERVAL)
				{
					HAL_GPIO_WritePin(channels[i].port, channels[i].pin, GPIO_PIN_RESET);
					channels[i].state = 3;
					channels[i].start_time = now;
				}
				break;
			case 3:
				if (elapsed >= CLOSE_INTERVAL)
				{
					HAL_GPIO_WritePin(channels[i].port, channels[i].pin, GPIO_PIN_SET); // 替换实际引脚
					channels[i].state = 4;
					channels[i].start_time =now;
				}
				break;
			case 4:
				if (elapsed >= OPEN_INTERVAL)
				{
					HAL_GPIO_WritePin(channels[i].port, channels[i].pin, GPIO_PIN_RESET); // 替换实际引脚
					channels[i].state = 5;
					//channels[i].start_time = HAL_GetTick(); 
				}
				break;
			}
		
	}	

}



/// @brief 将弧度转换为角度
/// @param rad 弧度值
/// @return 角度值
float degrees(float rad)
{
    return rad * (180.0f / 3.14159265f);
}

/// @brief 飞控接口初始化
/// @param  空

void FMU_Init(void)
{
	Init_COM_with_DMA_idle(&COM1, "COM1", &huart2, &hdma_usart2_rx, &FMU_decode);
}

mavlink_status_t status;  // mavlink解析状态。注意，此变量不可以定义为局部变量，否则无法记下当前的解析状态
mavlink_message_t rec_mav_msg;  // 存放接收到的mavlink信息
int chan = MAVLINK_COMM_0;  // mavlink通道


/// @brief 解析飞控发送来的数据
/// @param data_in 接收到的数据
void FMU_decode(uint8_t data_in)
{
	uint8_t msg_state = mavlink_parse_char(chan, data_in, &rec_mav_msg, &status);


	if (msg_state==1)
	{
		switch (rec_mav_msg.msgid)
		{
			case MAVLINK_MSG_ID_HEARTBEAT:
			//printf("Got one heartbeat frame\r\n");
				//printf("Received message with ID %d, sequence: %d from component %d of system %d", rec_mav_msg.msgid, rec_mav_msg.seq, rec_mav_msg.compid, rec_mav_msg.sysid);
			//SEGGER_RTT_printf(0, RTT_CTRL_TEXT_RED "Got one heartbeat frame\r\n");
			mavlink_msg_heartbeat_decode(&rec_mav_msg, &heartbeat);
			current_flight_mode = heartbeat.custom_mode;  // 心跳帧中包含当前的飞行模式
			already_got_heartbeat = true;
			update_freq_calc(&freq_calc_heartbeat);
			break;

//		case MAVLINK_MSG_ID_GPS_RAW_INT:
//			mavlink_msg_gps_raw_int_decode(&rec_mav_msg, &gps_raw_int);
//			gnss_fix_type = gps_raw_int.fix_type;
//			gnss_sat_num = gps_raw_int.satellites_visible;
//			gnss_hdop = gps_raw_int.eph;
//			update_freq_calc(&freq_calc_gps_raw_int);
//			break;

		case MAVLINK_MSG_ID_ATTITUDE:
			mavlink_msg_attitude_decode(&rec_mav_msg, &attitude);
			roll_deg = degrees(attitude.roll);
			pitch_deg = degrees(attitude.pitch);
			yaw_deg = degrees(attitude.yaw);
			update_freq_calc(&freq_calc_attitude);
			break;

		case MAVLINK_MSG_ID_GLOBAL_POSITION_INT:
			mavlink_msg_global_position_int_decode(&rec_mav_msg, &global_position_int);
			latitude = global_position_int.lat;
			longitude = global_position_int.lon;
			alt_msl_mm = global_position_int.alt;
			alt_above_home_mm = global_position_int.relative_alt;
			speed_north_cm_s = global_position_int.vx;
			speed_east_cm_s = global_position_int.vy;
			speed_down_cm_s = global_position_int.vz;
			update_freq_calc(&freq_calc_global_position_int);
			break;
		
		case MAVLINK_MSG_ID_PAOTOU_MAVLINK:
			mavlink_msg_paotou_mavlink_decode(&rec_mav_msg, &paotou_mavlink);
			enable = paotou_mavlink.enable;
			qian = paotou_mavlink.qian;
			hou = paotou_mavlink.hou;
			zuo = paotou_mavlink.zuo;
			you = paotou_mavlink.you;
			paotou1 = paotou_mavlink.tongdao1;
			paotou2 = paotou_mavlink.tongdao2;
			paotou3 = paotou_mavlink.tongdao3;
			paotou4 = paotou_mavlink.tongdao4;
			all=paotou_mavlink.all;
		//	printf("使能：%d 全抛：%d 前方：%d 后方：%d 左方：%d 右方：%d  抛1:%d 抛2: %d 抛3:  %d  抛4 : %d \r\n", enable, all, qian, hou, zuo, you, paotou1, paotou2, paotou3, paotou4);
			update_freq_calc(&freq_calc_paotou_mavlink);
		//	printf("抛投数据发送频率：%f\r\n",freq_calc_paotou_mavlink.freq_Hz);
			break;

		default:
			// SEGGER_RTT_printf(0, RTT_CTRL_TEXT_RED "Got one frame, ID: %X\r\n", rec_mav_msg.msgid);
			break;
		}
	}
}

/// @brief 通过串口发送mavlink帧
/// @param message 要发送的mavlink帧
void send_mavlink_msg(const mavlink_message_t *msg)
{
	uint8_t buf[300];
	// Translate message to buffer
	uint16_t len = mavlink_msg_to_send_buffer(buf, msg);
	COM_send_data(&MAVLINK_COM, buf, len);
}

/// @brief 向mavlink设备请求数据流
/// @param req_stream_id 数据流的ID
/// @param req_message_rate 数据流的频率，单位Hz. 如果要关闭此流，则此值设置为0
void request_mavlink_stream(uint8_t req_stream_id, uint16_t req_message_rate)
{
	mavlink_message_t message;
	mavlink_request_data_stream_t rds;
	rds.req_message_rate = req_message_rate;
	rds.target_system = MAVLINK_SYSTEM_ID;
	rds.target_component = MAVLINK_COMPONENT_ID;
	rds.req_stream_id = req_stream_id;
	rds.start_stop = (req_message_rate == 0) ? 0 : 1;
	mavlink_msg_request_data_stream_encode(MAVLINK_SYSTEM_ID, MAVLINK_COMPONENT_ID, &message, &rds);
	send_mavlink_msg(&message);
}

void send_paotou_stream(local_paotou_mavlink_t local_msg )
{
	mavlink_message_t message;
	mavlink_paotou_mavlink_t paotou_msg;
	paotou_msg.enable=local_msg.enable_switch;
	paotou_msg.qian=local_msg.front;
	paotou_msg.hou=local_msg.rear;
	paotou_msg.zuo=local_msg.left;
	paotou_msg.you=local_msg.right;
	paotou_msg.tongdao1=local_msg.turret1;
	paotou_msg.tongdao2=local_msg.turret2;
	paotou_msg.tongdao3=local_msg.turret3;
	paotou_msg.tongdao4=local_msg.turret4;
	paotou_msg.all=local_msg.open_all; 
	mavlink_msg_paotou_mavlink_encode(MAVLINK_SYSTEM_ID, MAVLINK_COMPONENT_ID, &message, &paotou_msg);
	send_mavlink_msg(&message);
}

void send_paotou_task()
{
	TickType_t xLasTime = xTaskGetTickCount();
	while(1)
	{
		local_paotou_mavlink_t paotou_msg;
		paotou_msg.enable_switch=1;
		paotou_msg.front=1;
		paotou_msg.rear=1;
		paotou_msg.left=1;
		paotou_msg.right=1;
		paotou_msg.turret1=1;
		paotou_msg.turret2=1;
		paotou_msg.turret3=1;
		paotou_msg.turret4=1;
		paotou_msg.open_all=1;
		send_paotou_stream(paotou_msg);
		//printf("使能：%d 全抛：%d 前方：%d 后方：%d 左方：%d 右方：%d  抛1:%d 抛2: %d 抛3:  %d  抛4 : %d \r\n", enable, all, qian, hou, zuo, you, paotou1, paotou2, paotou3, paotou4);
		//printf("使能：%d 全抛：%d 前方：%d 后方：%d 左方：%d 右方：%d  抛1:%d 抛2: %d 抛3:  %d  抛4 : %d \r\n", enable, all, qian, hou, zuo, you, paotou1, paotou2, paotou3, paotou4);
		vTaskDelayUntil(&xLasTime,500);
	}
}

void FMU_task(void *argument)
{
	FMU_Init();
	TickType_t xLastWakeTime = xTaskGetTickCount();
	//xTaskCreate(send_paotou_task,"send_paotou_task",configMINIMAL_STACK_SIZE*2,NULL,tskIDLE_PRIORITY+1,NULL);
	for (;;)
	{
//		HAL_GPIO_TogglePin(OUT1_GPIO_Port,OUT1_Pin);
//		HAL_GPIO_TogglePin(OUT2_GPIO_Port,OUT2_Pin);
//		HAL_GPIO_TogglePin(OUT3_GPIO_Port,OUT3_Pin);
//		HAL_GPIO_TogglePin(OUT4_GPIO_Port,OUT4_Pin);
//		uint8_t i;
//		for(i=0;i<4;i++)
//		{
//			HAL_GPIO_TogglePin(channels[i].port,channels[i].pin);
//		}
		process_channels();
		
		printf("使能：%d 全抛：%d 前方：%d 后方：%d 左方：%d 右方：%d  抛1:%d 抛2: %d 抛3:  %d  抛4 : %d \r\n", enable, all, qian, hou, zuo, you, paotou1, paotou2, paotou3, paotou4);
		
		if (already_got_heartbeat)  // 已经收到心跳帧
		{
			//printf("rec heartbeat\r\n");
			if (already_request_mav_stream == false)  // 还未请求数据流
			{
				//request_mavlink_stream(MAV_DATA_STREAM_EXTENDED_STATUS, 2);
				//request_mavlink_stream(MAV_DATA_STREAM_EXTRA1, 5);
				//request_mavlink_stream(MAV_DATA_STREAM_POSITION, 5);		

				
				already_request_mav_stream = true;	
			}
		}
		vTaskDelayUntil(&xLastWakeTime, 200);
	}
}
