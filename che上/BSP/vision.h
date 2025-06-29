#ifndef _bspvision_h
#define _bspvision_h
#include "main.h"
#include "type.h"
#include  "usbd_cdc_if.h"

extern uint8_t myUsbRxData[ ] ;
extern uint16_t myUsbRxNum ;


typedef	__packed struct
{
	char start;      // 0 帧头，取 's'
  char type;       // 1 消息类型 0xA0
  char find_bool;  // 2 是否追踪
  float yaw;       // 3 - 6 偏航角
  float pitch;     // 7 - 10 俯仰角
  //float distance;  // 11 - 14 距离
  char  empty[20];           // 11 - 30 预留空位
  char end;        // 31 帧尾，取 'e'	
}vision_rx;


typedef __packed struct
{
		char start;                 // 0 帧头，取 's'
    char type;                  // 1 消息类型 0xB0
    float yaw;                  // 2 - 5 偏航角
    float pitch;                // 6 - 9 俯仰角
    uint8_t enemy_team_color;  // 10 敌方颜色 0 ：红 1 ：蓝
    uint8_t mode;              // 11 模式 0 ：自瞄 1 ：符
    uint8_t rune_flag;         // 12 符模式 '0'：不可激活 '1'：小符 '2':大符
    char  empty[18];            // 13 - 30 预留空位
    char end;        // 31 帧尾，取 'e'
}vision_tx;


extern vision_rx Vision_Read_Data	;
extern vision_tx Vision_Send_Data	;


void vision_receive_test(void);
uint8_t vision_read(uint8_t* data,uint32_t len);
void vision_send(void);
void vision_clear_R(void);
void vision_clear_S(void);

#endif