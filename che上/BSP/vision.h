#ifndef _bspvision_h
#define _bspvision_h
#include "main.h"
#include "type.h"
#include  "usbd_cdc_if.h"

extern uint8_t myUsbRxData[ ] ;
extern uint16_t myUsbRxNum ;


typedef	__packed struct
{
	char start;      // 0 ֡ͷ��ȡ 's'
  char type;       // 1 ��Ϣ���� 0xA0
  char find_bool;  // 2 �Ƿ�׷��
  float yaw;       // 3 - 6 ƫ����
  float pitch;     // 7 - 10 ������
  //float distance;  // 11 - 14 ����
  char  empty[20];           // 11 - 30 Ԥ����λ
  char end;        // 31 ֡β��ȡ 'e'	
}vision_rx;


typedef __packed struct
{
		char start;                 // 0 ֡ͷ��ȡ 's'
    char type;                  // 1 ��Ϣ���� 0xB0
    float yaw;                  // 2 - 5 ƫ����
    float pitch;                // 6 - 9 ������
    uint8_t enemy_team_color;  // 10 �з���ɫ 0 ���� 1 ����
    uint8_t mode;              // 11 ģʽ 0 ������ 1 ����
    uint8_t rune_flag;         // 12 ��ģʽ '0'�����ɼ��� '1'��С�� '2':���
    char  empty[18];            // 13 - 30 Ԥ����λ
    char end;        // 31 ֡β��ȡ 'e'
}vision_tx;


extern vision_rx Vision_Read_Data	;
extern vision_tx Vision_Send_Data	;


void vision_receive_test(void);
uint8_t vision_read(uint8_t* data,uint32_t len);
void vision_send(void);
void vision_clear_R(void);
void vision_clear_S(void);

#endif