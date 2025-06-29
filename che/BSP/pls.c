#include<stdio.h>
#include "pls.h"
#include "main.h"
uint8_t Lazer_rx_buf[40] = {0};
uint16_t Temp_dis = 0;

KALMAN klm;



fram_t Dis_frame_rx = { 0 };
#define crc_mul 0x04C11DB7  //生成多项式
#define uint8_t unsigned char
#define uint32_t unsigned int

uint32_t crc32_mpeg_2(uint8_t* data, uint8_t length)
{
    uint8_t i;
    uint32_t crc = 0xffffffff;  // Initial value
    while (length--)
    {
        crc ^= (uint32_t)(*data++) << 24;// crc ^=(uint32_t)(*data)<<24; data++;
        for (i = 0; i < 8; ++i)
        {
            if (crc & 0x80000000)
                crc = (crc << 1) ^ 0x04C11DB7;
            else
                crc <<= 1;
        }
    }
    return crc;
}
/*
   CRC校验
*/
uint32_t Verify_CRC32_Check_Sum(uint8_t* pchMessage, uint32_t dwLength)
{
    uint32_t wExpected = 0;
    wExpected = crc32_mpeg_2(pchMessage, dwLength - 4);

    return ((wExpected & 0x000000ff) == pchMessage[dwLength - 4] && 
        ((wExpected >> 8) & 0x000000ff)== pchMessage[dwLength - 3] &&
        ((wExpected >> 16) & 0x000000ff) == pchMessage[dwLength - 2] &&
        ((wExpected >> 24) & 0x000000ff) == pchMessage[dwLength - 1]
        );
}

void READ_Data(uint8_t * Rx_data)
{
  if(Verify_CRC32_Check_Sum(Rx_data  , LEN_DATA))
	{
	   Temp_dis = Rx_data[5]<<8 | Rx_data[4];
		 Dis_frame_rx.data.Distance = Kal_Man(Temp_dis);
	}	
}

void Dis_init(UART_HandleTypeDef *huart)
{
	Kal_Man_init();
	
	uint8_t init_data[10] = {0xF5 , 0XE1 , 0x01 ,0x00 ,0x00, 0x00 , 0X12 ,0X17, 0XE4 ,0X7B};
	HAL_UART_Transmit(huart,init_data,10,0xffff);
  HAL_Delay(30);
	init_data[0] = 0XF5;
	init_data[1] = 0xE0;
	init_data[2] = 0X01;
	init_data[3] = 0X00;
	init_data[4] = 0X00;
	init_data[5] = 0X00;
	init_data[6] = 0x9F;
	init_data[7] = 0x70;
	init_data[8] = 0xE9;
	init_data[9] = 0x32;
	HAL_UART_Transmit(huart,init_data,10,0xffff);
  HAL_Delay(30);

}

void Kal_Man_init(void)
{
	klm.p_last = 2000;//随便赋值
	klm.x_last = 1;  //上次测量值
	klm.R = 0.5;  //测量误差
	klm.Q = 2;  //估计误差
}

/*一阶滤波*/
/*一阶滤波*/
/*一阶滤波*/
/*一阶滤波*/
/*一阶滤波*/
unsigned long Kal_Man(unsigned long res)//采集值
{
   /*上次估计*/
   klm.x_pre = klm.x_last;


   klm.p_pre = klm.p_last + klm.Q;


   klm.kg     = klm.p_pre / (klm.p_pre+ klm.R);

   klm.x_now = (1 - klm.kg) * klm.x_pre + klm.kg * res;

   klm.p_now = (1 - klm.kg) * klm.p_pre;

   klm.p_last = klm.p_now;

   klm.x_last = klm.x_now;


   return klm.x_now;

}













