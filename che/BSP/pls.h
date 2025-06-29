#ifndef __pls_h
#define __pls_h
#include "stm32f4xx_hal.h"
#define 	LEN_DATA 	  24				//Êý¾Ý³¤

typedef struct{

   float kg;

   float Q;

   float R;

   float x_pre;

   float p_pre;

   float p_last;

   float p_now;

   float x_now;

   float x_last;

   float x_pid;

}KALMAN;



typedef  struct
{
    uint16_t Distance;
    uint8_t Temp[2];
    uint8_t Amplitude[2];
    uint8_t Background_Light[2];
    uint8_t TOF[2];
}fram_data_t;


typedef  struct
{
    uint8_t frame_header[4];
    fram_data_t data;
    uint32_t  frame_tail;	
}fram_t;


void READ_Data(uint8_t * Rx_data);
uint32_t Verify_CRC32_Check_Sum(uint8_t* pchMessage, uint32_t dwLength);
uint32_t crc32_mpeg_2(uint8_t* data, uint8_t length);
void Dis_init(UART_HandleTypeDef *huart);
void Kal_Man_init(void);
unsigned long Kal_Man(unsigned long res);
extern fram_t Dis_frame_rx ;
extern uint8_t Lazer_rx_buf[40];
#endif
