
/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       INS_task.c/h
  * @brief      use bmi088 to calculate the euler angle. no use ist8310, so only
  *             enable data ready pin to save cpu time.enalbe bmi088 data ready
  *             enable spi DMA to save the time spi transmit
  *             主要利用陀螺仪bmi088，磁力计ist8310，完成姿态解算，得出欧拉角，
  *             提供通过bmi088的data ready 中断完成外部触发，减少数据等待延迟
  *             通过DMA的SPI传输节约CPU时间.
  * @note
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. done
  *  V2.0.0     Nov-11-2019     RM              1. support bmi088, but don't support mpu6500
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  //---------------------------下面是抽象的许铁头在讲话↓-----------------------------------//
  *2022.7.02 天气热的要命 心情很不美丽
  *1、现在的代码yaw轴存在零飘情况 如果启用磁力计能解决，但是磁力计收敛很慢并且前期的零
  *飘十分抽象（该车一次debug值更新零飘1度多，十分抽象）
  *2、启用磁力计就是初始化磁力计并且把ist8310_read_mag(ist8310_real_data.mag)函数取消注释
  *如果你好奇 可以取消注释看看效果 然后就能理解我为什么不启用了（阿门）
  *3、对比官方代码会发现官方例程里gyro_update_flag用了IMU_NOTIFY_SHFITS
  *但是相信我 不是这里抄错了用成了IMU_UPDATE_SHFITS 如果你好奇为什么 可以改回去试试（微笑.jpg）
  *4、那个官方所言的解决零飘的代码没个卵用 你只要细看代码 就会发现值根本没传出来（不过官方宏定义了
  *这个函数并且在校准函数里用了他 但是并没有调用那个校准函数 总之就是官方摆烂例程没写也不知道他
  *到底在干什么于是放弃使用了）
  *5、珍爱生命 远离陀螺仪 阿门
  */
//2023.9 将陀螺仪改为南航版本
#include "IMU_Task.h"


#include "main.h"

#include "cmsis_os.h"
//#include "FreeRTOS.h"
//#include "task.h"
#include "bsp_imu_pwm.h"
#include "bsp_spi.h"
#include "BMI088driver.h"
#include "ist8310driver.h"
#include "pidimu.h"
#include "arm_math.h"
//#include "vision.h"
//#include "usbd_cdc_if.h"
#include "MahonyAHRS.h"
#include "math.h"




#define IMU_temp_PWM(pwm)  imu_pwm_set(pwm)                    //pwm给定
#define BMI088_BOARD_INSTALL_SPIN_MATRIX    \
    {1.0f, 0.0f, 0.0f},                     \
    {0.0f, 1.0f, 0.0f},                     \
    {0.0f, 0.0f, 1.0f}                     \


#define IST8310_BOARD_INSTALL_SPIN_MATRIX   \
    {1.0f, 0.0f, 0.0f},                     \
    {0.0f, 1.0f, 0.0f},                     \
    {0.0f, 0.0f, 1.0f}                      \
	
/**
  * @brief          旋转陀螺仪,加速度计和磁力计,并计算零漂,因为设备有不同安装方式
  * @param[out]     gyro: 加上零漂和旋转
  * @param[out]     accel: 加上零漂和旋转
  * @param[out]     mag: 加上零漂和旋转
  * @param[in]      bmi088: 陀螺仪和加速度计数据
  * @param[in]      ist8310: 磁力计数据
  * @retval         none
  */
static void imu_cali_slove(fp32 gyro[3], fp32 accel[3], fp32 mag[3], bmi088_real_data_t *bmi088, ist8310_real_data_t *ist8310);

/**
  * @brief          control the temperature of bmi088
  * @param[in]      temp: the temperature of bmi088
  * @retval         none
  */
/**
  * @brief          控制bmi088的温度
  * @param[in]      temp:bmi088的温度
  * @retval         none
  */
static void imu_temp_control(fp32 temp);

/**
  * @brief          open the SPI DMA accord to the value of imu_update_flag
  * @param[in]      none
  * @retval         none
  */
/**
  * @brief          根据imu_update_flag的值开启SPI DMA
  * @param[in]      temp:bmi088的温度
  * @retval         none
  */
static void imu_cmd_spi_dma(void);

void AHRS_init(fp32 quat[4], fp32 accel[3], fp32 mag[3]);
void AHRS_update(fp32 quat[4], fp32 time, fp32 gyro[3], fp32 accel[3], fp32 mag[3]);
void get_angle(fp32 quat[4], fp32 *yaw, fp32 *pitch, fp32 *roll);
void mpu_offset_clc(void);

extern SPI_HandleTypeDef hspi1;


static TaskHandle_t INS_task_local_handler;

uint8_t gyro_dma_rx_buf[SPI_DMA_GYRO_LENGHT];
uint8_t gyro_dma_tx_buf[SPI_DMA_GYRO_LENGHT] = {0x82,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};


uint8_t accel_dma_rx_buf[SPI_DMA_ACCEL_LENGHT];
uint8_t accel_dma_tx_buf[SPI_DMA_ACCEL_LENGHT] = {0x92,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};

uint8_t accel_temp_dma_rx_buf[SPI_DMA_ACCEL_TEMP_LENGHT];
uint8_t accel_temp_dma_tx_buf[SPI_DMA_ACCEL_TEMP_LENGHT] = {0xA2, 0xFF, 0xFF, 0xFF};





volatile uint8_t gyro_update_flag = 0;
volatile uint8_t accel_update_flag = 0;
volatile uint8_t accel_temp_update_flag = 0;
volatile uint8_t mag_update_flag = 0;
volatile uint8_t imu_start_dma_flag = 0;
volatile uint8_t imu_read_flag=0;

volatile bool imu_init_finish_flag = 0;
volatile bool gimbal_return_finish_flag = 0;

bmi088_real_data_t bmi088_real_data;
bmi088_real_data_t bmi088_offset_data;

fp32 gyro_scale_factor[3][3] = {BMI088_BOARD_INSTALL_SPIN_MATRIX};
fp32 gyro_offset[3];
fp32 gyro_cali_offset[3];

fp32 accel_scale_factor[3][3] = {BMI088_BOARD_INSTALL_SPIN_MATRIX};
fp32 accel_offset[3];
fp32 accel_cali_offset[3];

ist8310_real_data_t ist8310_real_data;
fp32 mag_scale_factor[3][3] = {IST8310_BOARD_INSTALL_SPIN_MATRIX};
fp32 mag_offset[3];
fp32 mag_cali_offset[3];
imu_chass IMU_Chass_MSG;

static uint8_t first_temperate;
static fp32 imu_temp_PID[3] = {TEMPERATURE_PID_KP, TEMPERATURE_PID_KI, TEMPERATURE_PID_KD};
static pid_type_def imu_temp_pid;
static const float timing_time = 0.001f;   //tast run time , unit s.任务运行的时间 单位 s

//加速度计低通滤波
static fp32 accel_fliter_1[3] = {0.0f, 0.0f, 0.0f};
static fp32 accel_fliter_2[3] = {0.0f, 0.0f, 0.0f};
static fp32 accel_fliter_3[3] = {0.0f, 0.0f, 0.0f};
static const fp32 fliter_num[3] = {1.929454039488895f, -0.93178349823448126f, 0.002329458745586203f};

fp32 INS_gyro[3] = {0.0f, 0.0f, 0.0f};
fp32 INS_accel[3] = {0.0f, 0.0f, 0.0f};
fp32 INS_mag[3] = {0.0f, 0.0f, 0.0f};
fp32 INS_quat[4] = {0.0f, 0.0f, 0.0f, 0.0f};
fp32 INS_angle[3] = {0.0f, 0.0f, 0.0f};      //euler angle, unit rad.欧拉角 单位 rad
fp32 INS_angle_deg[3] = {0.0f, 0.0f, 0.0f};

fp32 IMU_angle[3] = {0.0f, 0.0f, 0.0f};

fp32 imu_vision[3]={0.0f,0.0f,0.0f};

float *IMU_kf_result;
float yaw_add_one_tick=0.000109032771;//-6.80427474e-05f;
float yaw_offset;


/**
  * @brief          imu task, init bmi088, ist8310, calculate the euler angle
  * @param[in]      pvParameters: NULL
  * @retval         none
  */
/**
  * @brief          imu任务, 初始化 bmi088, ist8310, 计算欧拉角
  * @param[in]      pvParameters: NULL
  * @retval         none
  */
void IMUFun(void const *pvParameters)
{
    uint16_t count = 0;
    portTickType currentTime;
    
    osDelay(INS_TASK_INIT_TIME);
    
    while(BMI088_init())
    {
        osDelay(100);
    }
    
     while(ist8310_init())
    {
        osDelay(100);
    }
    BMI088_read(bmi088_real_data.gyro, bmi088_real_data.accel, &bmi088_real_data.temp);
    imu_cali_slove(INS_gyro, INS_accel, INS_mag, &bmi088_real_data, &ist8310_real_data);
    
    PID_init(&imu_temp_pid, PID_POSITION, imu_temp_PID, TEMPERATURE_PID_MAX_OUT, TEMPERATURE_PID_MAX_IOUT);
    AHRS_init(INS_quat, INS_accel, INS_mag);

//    accel_fliter_1[0] = accel_fliter_2[0] = accel_fliter_3[0] = bmi088_real_data.accel[0];//p
//    accel_fliter_1[1] = accel_fliter_2[1] = accel_fliter_3[1] = bmi088_real_data.accel[1];//yoll
//    accel_fliter_1[2] = accel_fliter_2[2] = accel_fliter_3[2] = bmi088_real_data.accel[2];//z

    //get the handle of task
    //获取当前任务的任务句柄，
    INS_task_local_handler = xTaskGetHandle(pcTaskGetName(NULL));

    //set spi frequency
    hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;

    if (HAL_SPI_Init(&hspi1) != HAL_OK)
    {
        Error_Handler();
    }

    SPI1_DMA_init((uint32_t)gyro_dma_tx_buf, (uint32_t)gyro_dma_rx_buf, SPI_DMA_GYRO_LENGHT);

    imu_start_dma_flag = 1;
    bmi088_offset_data.gyro[0] =-0.00105551549; 
	bmi088_offset_data.gyro[1] = 0.00144814304;
	bmi088_offset_data.gyro[2] = 0.000177296082;
    
    
    while(1)
    {
        IMU_task_add++;
          currentTime = xTaskGetTickCount(); //当前系统时间
        //wait spi DMA tansmit done
        //等待SPI DMA传输
			while (ulTaskNotifyTake(pdTRUE, portMAX_DELAY) != pdPASS)
        {
        }

        if(!imu_init_finish_flag)
            count++;

        if(!imu_init_finish_flag && count > 500)
        {
            imu_init_finish_flag = 1;
            count = 700;
        }

       if(gyro_update_flag & (1 << IMU_UPDATE_SHFITS))
        {
            gyro_update_flag &= ~(1 << IMU_UPDATE_SHFITS);
            BMI088_gyro_read_over(gyro_dma_rx_buf + BMI088_GYRO_RX_BUF_DATA_OFFSET, bmi088_real_data.gyro);
						bmi088_real_data.gyro[0] -= bmi088_offset_data.gyro[0];
						bmi088_real_data.gyro[1] -= bmi088_offset_data.gyro[1];
						bmi088_real_data.gyro[2] -= bmi088_offset_data.gyro[2];
        }
        if(accel_update_flag & (1 << IMU_UPDATE_SHFITS))
        {
            accel_update_flag &= ~(1 << IMU_UPDATE_SHFITS);
            BMI088_accel_read_over(accel_dma_rx_buf + BMI088_ACCEL_RX_BUF_DATA_OFFSET, bmi088_real_data.accel, &bmi088_real_data.time);

        }

        if(accel_temp_update_flag & (1 << IMU_UPDATE_SHFITS))
        {
            accel_temp_update_flag &= ~(1 << IMU_UPDATE_SHFITS);
            BMI088_temperature_read_over(accel_temp_dma_rx_buf + BMI088_ACCEL_RX_BUF_DATA_OFFSET, &bmi088_real_data.temp);
            imu_temp_control(bmi088_real_data.temp);
        }

        imu_cali_slove(INS_gyro, INS_accel, INS_mag, &bmi088_real_data, &ist8310_real_data);
        //加速度计低通滤波
        accel_fliter_1[0] = accel_fliter_2[0];
        accel_fliter_2[0] = accel_fliter_3[0];

        accel_fliter_3[0] = accel_fliter_2[0] * fliter_num[0] + accel_fliter_1[0] * fliter_num[1] + INS_accel[0] * fliter_num[2];

        accel_fliter_1[1] = accel_fliter_2[1];
        accel_fliter_2[1] = accel_fliter_3[1];

        accel_fliter_3[1] = accel_fliter_2[1] * fliter_num[0] + accel_fliter_1[1] * fliter_num[1] + INS_accel[1] * fliter_num[2];

        accel_fliter_1[2] = accel_fliter_2[2];
        accel_fliter_2[2] = accel_fliter_3[2];

        accel_fliter_3[2] = accel_fliter_2[2] * fliter_num[0] + accel_fliter_1[2] * fliter_num[1] + INS_accel[2] * fliter_num[2];

        AHRS_update(INS_quat, timing_time, INS_gyro, accel_fliter_3, INS_mag);
        get_angle(INS_quat, INS_angle + INS_YAW_ADDRESS_OFFSET, INS_angle + INS_PITCH_ADDRESS_OFFSET, INS_angle + INS_ROLL_ADDRESS_OFFSET);
				//消零飘
				static int pre_tick = 0;
				int tick = HAL_GetTick();
				if (pre_tick)
				yaw_offset += yaw_add_one_tick * (tick - pre_tick);
				IMU_angle[0] = 180.0f*INS_angle[0]/PI+yaw_offset;   //yaw_offset; //yaw
				IMU_angle[1] = 180.0f*INS_angle[1]/PI; //pitch
				IMU_angle[2] = 180.0f*INS_angle[2]/PI; //roll     
				pre_tick = tick;   
				AngleLoop_f(&IMU_angle[0],360);	
				//        /*更新底盘陀螺仪信息*/
				IMU_Chass_MSG.ol_angle[0] = INS_angle[0];           //yaw
				IMU_Chass_MSG.ol_angle[1] = INS_angle[1];           //pitch
				IMU_Chass_MSG.ol_angle[2] = INS_angle[2];           //roll

				
				IMU_Chass_MSG.angle[0]=IMU_angle[0];
				IMU_Chass_MSG.angle[1]=IMU_angle[1];
				IMU_Chass_MSG.angle[2]=IMU_angle[2];
				
				imu_vision[0]=IMU_angle[0]*0.017453292f;
				imu_vision[1]=IMU_angle[1]*0.017453292f;
                
				imu_vision[2]=IMU_angle[2]*0.017453292f;
				
				IMU_Chass_MSG.accel[0] = accel_fliter_3[0];
				IMU_Chass_MSG.accel[1] = accel_fliter_3[1];
				IMU_Chass_MSG.accel[2] = accel_fliter_3[2];

				for(uint8_t i = 0; i < 4; i++)
				{
					IMU_Chass_MSG.quat_[i] = INS_quat[i];             
				}				
				for(uint8_t i = 0; i < 3; i++)
				{
					IMU_Chass_MSG.omg_rad[i] = bmi088_real_data.gyro[i];    //0 roll    1 pitch     2 yaw
				}					
				IMU_Chass_MSG.temp = bmi088_real_data.temp;
        /*------------------*/
//		//虚拟串口发送
//        vision_transmit();
//		CDC_Transmit_FS( vision_transmit_buff, VISION_BUFLEN );
        vTaskDelayUntil(&currentTime, 1);
    }
}

/**
  * @brief          旋转陀螺仪,加速度计和磁力计,并计算零漂,因为设备有不同安装方式
  * @param[out]     gyro: 加上零漂和旋转
  * @param[out]     accel: 加上零漂和旋转
  * @param[out]     mag: 加上零漂和旋转
  * @param[in]      bmi088: 陀螺仪和加速度计数据
  * @param[in]      ist8310: 磁力计数据
  * @retval         none
  */
static void imu_cali_slove(fp32 gyro[3], fp32 accel[3], fp32 mag[3], bmi088_real_data_t *bmi088, ist8310_real_data_t *ist8310)
{
    for (uint8_t i = 0; i < 3; i++)
    {
        gyro[i] = bmi088->gyro[0] * gyro_scale_factor[i][0] + bmi088->gyro[1] * gyro_scale_factor[i][1] + bmi088->gyro[2] * gyro_scale_factor[i][2];
        accel[i] = bmi088->accel[0] * accel_scale_factor[i][0] + bmi088->accel[1] * accel_scale_factor[i][1] + bmi088->accel[2] * accel_scale_factor[i][2] + accel_offset[i];
        mag[i] = ist8310->mag[0] * mag_scale_factor[i][0] + ist8310->mag[1] * mag_scale_factor[i][1] + ist8310->mag[2] * mag_scale_factor[i][2] + mag_offset[i];
    }
}

void AHRS_init(fp32 quat[4], fp32 accel[3], fp32 mag[3])
{
    quat[0] = 1.0f;
    quat[1] = 0.0f;
    quat[2] = 0.0f;
    quat[3] = 0.0f;

}

/**
  * @brief          计算陀螺仪零漂
  * @param[out]     gyro_offset:计算零漂
  * @param[in]      gyro:角速度数据
  * @param[out]     offset_time_count: 自动加1
  * @retval         none
  */
void gyro_offset_calc(fp32 gyro_offset[3], fp32 gyro[3], uint16_t *offset_time_count)
{
    if (gyro_offset == NULL || gyro == NULL || offset_time_count == NULL)
    {
        return;
    }

        gyro_offset[0] = gyro_offset[0] + 0.00005f * gyro[0];
        gyro_offset[1] = gyro_offset[1] + 0.00005f * gyro[1];
        gyro_offset[2] = gyro_offset[2] + 0.00005f * gyro[2];
        (*offset_time_count)++;
}


/**
  * @brief          calculate gyro zero drift
  * @param[out]     cali_scale:scale, default 1.0
  * @param[out]     cali_offset:zero drift, collect the gyro ouput when in still
  * @param[out]     time_count: time, when call gyro_offset_calc 
  * @retval         none
  */
/**
  * @brief          校准陀螺仪
  * @param[out]     陀螺仪的比例因子，1.0f为默认值，不修改
  * @param[out]     陀螺仪的零漂，采集陀螺仪的静止的输出作为offset
  * @param[out]     陀螺仪的时刻，每次在gyro_offset调用会加1,
  * @retval         none
  */
void INS_cali_gyro(fp32 cali_scale[3], fp32 cali_offset[3], uint16_t *time_count)
{
        if( *time_count == 0)
        {
            gyro_offset[0] = gyro_cali_offset[0];
            gyro_offset[1] = gyro_cali_offset[1];
            gyro_offset[2] = gyro_cali_offset[2];
        }
        gyro_offset_calc(gyro_offset, INS_gyro, time_count);

        cali_offset[0] = gyro_offset[0];
        cali_offset[1] = gyro_offset[1];
        cali_offset[2] = gyro_offset[2];
        cali_scale[0] = 1.0f;
        cali_scale[1] = 1.0f;
        cali_scale[2] = 1.0f;

}

void INS_set_cali_gyro(fp32 cali_scale[3], fp32 cali_offset[3])
{
    gyro_cali_offset[0] = cali_offset[0];
    gyro_cali_offset[1] = cali_offset[1];
    gyro_cali_offset[2] = cali_offset[2];
    gyro_offset[0] = gyro_cali_offset[0];
    gyro_offset[1] = gyro_cali_offset[1];
    gyro_offset[2] = gyro_cali_offset[2];
}

void AHRS_update(fp32 quat[4], fp32 time, fp32 gyro[3], fp32 accel[3], fp32 mag[3])
{
    MahonyAHRSupdateIMU(quat, gyro[0], gyro[1], gyro[2], accel[0], accel[1], accel[2]);
}

//进行四元素解算求得欧拉角
void get_angle(fp32 q[4], fp32 *yaw, fp32 *pitch, fp32 *roll)
{
    *yaw = atan2f(2.0f * (q[0] * q[3] + q[1] * q[2]), 2.0f * (q[0] * q[0] + q[1] * q[1]) - 1.0f);
    *pitch = asinf(-2.0f * (q[1] * q[3] - q[0] * q[2]));
    *roll = atan2f(2.0f * (q[0] * q[1] + q[2] * q[3]), 2.0f * (q[0] * q[0] + q[3] * q[3]) - 1.0f);
}
/**
  * @brief          control the temperature of bmi088
  * @param[in]      temp: the temperature of bmi088
  * @retval         none
  */
/**
  * @brief          控制bmi088的温度
  * @param[in]      temp:bmi088的温度
  * @retval         none
  */
static void imu_temp_control(fp32 temp)
{
    uint16_t tempPWM;
    static uint8_t temp_constant_time = 0;
    if (first_temperate)
    {
        PID_calc(&imu_temp_pid, temp,45.0f);
        if (imu_temp_pid.out < 0.0f)
        {
            imu_temp_pid.out = 0.0f;
        }
        tempPWM = (uint16_t)imu_temp_pid.out;
        IMU_temp_PWM(tempPWM);
    }
    else
    {
        //在没有达到设置的温度，一直最大功率加热
        //in beginning, max power
        if (temp > 45.0f)
        {
            temp_constant_time++;
            if (temp_constant_time > 200)
            {
                //达到设置温度，将积分项设置为一半最大功率，加速收敛
                //
                first_temperate = 1;
                imu_temp_pid.Iout = MPU6500_TEMP_PWM_MAX / 2.0f;
            }
        }

        IMU_temp_PWM(MPU6500_TEMP_PWM_MAX - 1);
    }
}


void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if(GPIO_Pin == INT1_ACCEL_Pin)
    {
        accel_update_flag |= 1 << IMU_DR_SHFITS;
        accel_temp_update_flag |= 1 << IMU_DR_SHFITS;
        if(imu_start_dma_flag)
        {
            imu_cmd_spi_dma();
        }
    }
    else if(GPIO_Pin == INT1_GYRO_Pin)
    {
        gyro_update_flag |= 1 << IMU_DR_SHFITS;
        if(imu_start_dma_flag)
        {
            imu_cmd_spi_dma();
        }
    }
    else if(GPIO_Pin == DRDY_IST8310_Pin)
    {
        mag_update_flag |= 1 << IMU_DR_SHFITS;

        if(mag_update_flag &= 1 << IMU_DR_SHFITS)
        {
            mag_update_flag &= ~(1 << IMU_DR_SHFITS);
            mag_update_flag |= (1 << IMU_SPI_SHFITS);

//            ist8310_read_mag(ist8310_real_data.mag);
        }
    }
    else if(GPIO_Pin == GPIO_PIN_0)
    {
        //wake up the task
        //唤醒任务
        if (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED)
        {
            static BaseType_t xHigherPriorityTaskWoken;
            vTaskNotifyGiveFromISR(INS_task_local_handler, &xHigherPriorityTaskWoken);
            portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
        }
    }
    else if(GPIO_Pin == GPIO_PIN_1)
    {
        gimbal_return_finish_flag = 1;
    }
}

/**
  * @brief          open the SPI DMA accord to the value of imu_update_flag
  * @param[in]      none
  * @retval         none
  */
/**
  * @brief          根据imu_update_flag的值开启SPI DMA
  * @param[in]      temp:bmi088的温度
  * @retval         none
  */
static void imu_cmd_spi_dma(void)
{
    //开启陀螺仪的DMA传输
    if( (gyro_update_flag & (1 << IMU_DR_SHFITS) ) && !(hspi1.hdmatx->Instance->CR & DMA_SxCR_EN) && !(hspi1.hdmarx->Instance->CR & DMA_SxCR_EN)
            && !(accel_update_flag & (1 << IMU_SPI_SHFITS)) && !(accel_temp_update_flag & (1 << IMU_SPI_SHFITS)))
    {
        gyro_update_flag &= ~(1 << IMU_DR_SHFITS);
        gyro_update_flag |= (1 << IMU_SPI_SHFITS);

        HAL_GPIO_WritePin(CS1_GYRO_GPIO_Port, CS1_GYRO_Pin, GPIO_PIN_RESET);
        SPI1_DMA_enable((uint32_t)gyro_dma_tx_buf, (uint32_t)gyro_dma_rx_buf, SPI_DMA_GYRO_LENGHT);
        return;
    }
    //开启加速度计的DMA传输
    if((accel_update_flag & (1 << IMU_DR_SHFITS)) && !(hspi1.hdmatx->Instance->CR & DMA_SxCR_EN) && !(hspi1.hdmarx->Instance->CR & DMA_SxCR_EN)
            && !(gyro_update_flag & (1 << IMU_SPI_SHFITS)) && !(accel_temp_update_flag & (1 << IMU_SPI_SHFITS)))
    {
        accel_update_flag &= ~(1 << IMU_DR_SHFITS);
        accel_update_flag |= (1 << IMU_SPI_SHFITS);

        HAL_GPIO_WritePin(CS1_ACCEL_GPIO_Port, CS1_ACCEL_Pin, GPIO_PIN_RESET);
        SPI1_DMA_enable((uint32_t)accel_dma_tx_buf, (uint32_t)accel_dma_rx_buf, SPI_DMA_ACCEL_LENGHT);
        return;
    }

    if((accel_temp_update_flag & (1 << IMU_DR_SHFITS)) && !(hspi1.hdmatx->Instance->CR & DMA_SxCR_EN) && !(hspi1.hdmarx->Instance->CR & DMA_SxCR_EN)
            && !(gyro_update_flag & (1 << IMU_SPI_SHFITS)) && !(accel_update_flag & (1 << IMU_SPI_SHFITS)))
    {
        accel_temp_update_flag &= ~(1 << IMU_DR_SHFITS);
        accel_temp_update_flag |= (1 << IMU_SPI_SHFITS);

        HAL_GPIO_WritePin(CS1_ACCEL_GPIO_Port, CS1_ACCEL_Pin, GPIO_PIN_RESET);
        SPI1_DMA_enable((uint32_t)accel_temp_dma_tx_buf, (uint32_t)accel_temp_dma_rx_buf, SPI_DMA_ACCEL_TEMP_LENGHT);
        return;
    }
}

void IMU_IRQHandler(void)
{

	if(__HAL_DMA_GET_FLAG(hspi1.hdmarx, __HAL_DMA_GET_TC_FLAG_INDEX(hspi1.hdmarx)) != RESET)
    {
        __HAL_DMA_CLEAR_FLAG(hspi1.hdmarx, __HAL_DMA_GET_TC_FLAG_INDEX(hspi1.hdmarx));

        //gyro read over
        //陀螺仪读取完毕
        if(gyro_update_flag & (1 << IMU_SPI_SHFITS))
        {
            gyro_update_flag &= ~(1 << IMU_SPI_SHFITS);
            gyro_update_flag |= (1 << IMU_UPDATE_SHFITS);

            HAL_GPIO_WritePin(CS1_GYRO_GPIO_Port, CS1_GYRO_Pin, GPIO_PIN_SET);
        }

        //accel read over
        //加速度计读取完毕
        if(accel_update_flag & (1 << IMU_SPI_SHFITS))
        {
            accel_update_flag &= ~(1 << IMU_SPI_SHFITS);
            accel_update_flag |= (1 << IMU_UPDATE_SHFITS);

            HAL_GPIO_WritePin(CS1_ACCEL_GPIO_Port, CS1_ACCEL_Pin, GPIO_PIN_SET);
        }
        //temperature read over
        //温度读取完毕
        if(accel_temp_update_flag & (1 << IMU_SPI_SHFITS))
        {
            accel_temp_update_flag &= ~(1 << IMU_SPI_SHFITS);
            accel_temp_update_flag |= (1 << IMU_UPDATE_SHFITS);

            HAL_GPIO_WritePin(CS1_ACCEL_GPIO_Port, CS1_ACCEL_Pin, GPIO_PIN_SET);
        }

        imu_cmd_spi_dma();

        if(gyro_update_flag & (1 << IMU_UPDATE_SHFITS))
        {
            __HAL_GPIO_EXTI_GENERATE_SWIT(GPIO_PIN_0);
        }
    }
}

/**
  * @brief          calculate IMU Zero_drift
  * @param[in]      none
  * @retval         none
  */
/**
  * @brief          计算IMU零漂
  * @param[in]      none
  * @retval         none
  */
void mpu_offset_clc(void)
{
	static uint16_t i =0 ,j=0;
	while( i < 3000 || j < 3000 )
	{
			while (imu_read_flag == 0)
			{
			}
			imu_read_flag = 0;
			if(gyro_update_flag & (1 << IMU_UPDATE_SHFITS))
			{
				if(i<3000)
				{
					gyro_update_flag &= ~(1 << IMU_UPDATE_SHFITS);
					BMI088_gyro_read_over(gyro_dma_rx_buf + BMI088_GYRO_RX_BUF_DATA_OFFSET, bmi088_real_data.gyro);
					bmi088_offset_data.gyro[0] += bmi088_real_data.gyro[0];
					bmi088_offset_data.gyro[1] += bmi088_real_data.gyro[1];
					bmi088_offset_data.gyro[2] += bmi088_real_data.gyro[2];
				}	
				else if(i==3000)
				{
					bmi088_offset_data.gyro[0] = bmi088_offset_data.gyro[0] / 3000.0f;
					bmi088_offset_data.gyro[1] = bmi088_offset_data.gyro[1] / 3000.0f;
					bmi088_offset_data.gyro[2] = bmi088_offset_data.gyro[2] / 3000.0f;
				}
				i++;		
			}

			if(accel_update_flag & (1 << IMU_UPDATE_SHFITS))
			{
				if(j<3000)
				{
					accel_update_flag &= ~(1 << IMU_UPDATE_SHFITS);
					BMI088_accel_read_over(accel_dma_rx_buf + BMI088_ACCEL_RX_BUF_DATA_OFFSET, bmi088_real_data.accel, &bmi088_real_data.time);
					bmi088_offset_data.accel[0] += bmi088_real_data.accel[0];
					bmi088_offset_data.accel[1] += bmi088_real_data.accel[1];
					bmi088_offset_data.accel[2] += bmi088_real_data.accel[2];
				}	
				else if(j==3000)
				{
					bmi088_offset_data.accel[0] = bmi088_offset_data.accel[0] / 3000.0f;
					bmi088_offset_data.accel[1] = bmi088_offset_data.accel[1] / 3000.0f;
					bmi088_offset_data.accel[2] = bmi088_offset_data.accel[2] / 3000.0f;
				}
				j++;
			}
	}
}
