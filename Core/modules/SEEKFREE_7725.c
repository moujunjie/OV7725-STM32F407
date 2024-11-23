/*********************************************************************************************************************
 * COPYRIGHT NOTICE
 * Copyright (c) 2018,逐飞科技
 * All rights reserved.
 * 技术讨论QQ群：一群：179029047(已满)  二群：244861897(已满)  三群：824575535
 *
 * 以下所有内容版权均属逐飞科技所有，未经允许不得用于商业用途，
 * 欢迎各位使用并传播本程序，修改内容时必须保留逐飞科技的版权声明。
 *
 * @file       		小钻风(二值化摄像头)
 * @company	   		成都逐飞科技有限公司
 * @author     		逐飞科技(QQ3184284598)
 * @version    		查看doc内version文件 版本说明
 * @Software 		MDK 5.24
 * @Taobao   		https://seekfree.taobao.com/
 * @date       		2020-01-03
 * @note		
					OV7725接线定义：
					------------------------------------
					查看2-HAL OV7725 UART PC.ioc文件
					------------------------------------
 ********************************************************************************************************************/



#include "SEEKFREE_7725.h"
#include "SEEKFREE_IIC.h"
#include "tim.h"
#include "stm32f4xx_it.h"
#include "usart.h"

uint8_t image_bin[OV7725_SIZE];                                   //定义存储接收图像的数组
uint8_t ov7725_finish_flag = 0;
uint8_t OV7725_IDCode = 0;

//-------------------------------------------------------------------------------------------------------------------
//  @brief      小钻风摄像头内部寄存器初始化(内部使用，用户无需调用)
//  @param      NULL
//  @return     uint8_t			返回0则出错，返回1则成功
//  @since      v1.0
//  Sample usage:
//-------------------------------------------------------------------------------------------------------------------
uint8_t ov7725_reg_init(void)
{
    simiic_write_reg ( OV7725_DEV_ADD, OV7725_COM7, 0x80 );	//复位摄像头
    HAL_Delay(50);

    while( OV7725_IDCode != OV7725_ID )
	{
		OV7725_IDCode = simiic_read_reg( OV7725_DEV_ADD, OV7725_VER,SCCB);
		HAL_Delay(1);
		//校验摄像头ID号
	}

    if(OV7725_IDCode == OV7725_ID)
    {
        //ID号确认无误   然后配置寄存器
        simiic_write_reg(OV7725_DEV_ADD, OV7725_COM4, 0xC1);
        simiic_write_reg(OV7725_DEV_ADD, OV7725_CLKRC, 0x01);
        simiic_write_reg(OV7725_DEV_ADD, OV7725_COM2, 0x03);
        simiic_write_reg(OV7725_DEV_ADD, OV7725_COM3, 0xD0);
        simiic_write_reg(OV7725_DEV_ADD, OV7725_COM7, 0x40);
        simiic_write_reg(OV7725_DEV_ADD, OV7725_HSTART, 0x3F);
        simiic_write_reg(OV7725_DEV_ADD, OV7725_HSIZE, 0x50);
        simiic_write_reg(OV7725_DEV_ADD, OV7725_VSTRT, 0x03);
        simiic_write_reg(OV7725_DEV_ADD, OV7725_VSIZE, 0x78);
        simiic_write_reg(OV7725_DEV_ADD, OV7725_HREF, 0x00);
        simiic_write_reg(OV7725_DEV_ADD, OV7725_SCAL0, 0x0A);
        simiic_write_reg(OV7725_DEV_ADD, OV7725_AWB_Ctrl0, 0xE0);
        simiic_write_reg(OV7725_DEV_ADD, OV7725_DSPAuto, 0xff);
        simiic_write_reg(OV7725_DEV_ADD, OV7725_DSP_Ctrl2, 0x0C);
        simiic_write_reg(OV7725_DEV_ADD, OV7725_DSP_Ctrl3, 0x00);
        simiic_write_reg(OV7725_DEV_ADD, OV7725_DSP_Ctrl4, 0x00);


        if(OV7725_W == 80)              simiic_write_reg(OV7725_DEV_ADD, OV7725_HOutSize, 0x14);
        else if(OV7725_W == 160)        simiic_write_reg(OV7725_DEV_ADD, OV7725_HOutSize, 0x28);
        else if(OV7725_W == 240)        simiic_write_reg(OV7725_DEV_ADD, OV7725_HOutSize, 0x3c);
        else if(OV7725_W == 320)        simiic_write_reg(OV7725_DEV_ADD, OV7725_HOutSize, 0x50);

        if(OV7725_H == 60)              simiic_write_reg(OV7725_DEV_ADD, OV7725_VOutSize, 0x1E);
        else if(OV7725_H == 120)        simiic_write_reg(OV7725_DEV_ADD, OV7725_VOutSize, 0x3c);
        else if(OV7725_H == 180)        simiic_write_reg(OV7725_DEV_ADD, OV7725_VOutSize, 0x5a);
        else if(OV7725_H == 240)        simiic_write_reg(OV7725_DEV_ADD, OV7725_VOutSize, 0x78);


        simiic_write_reg(OV7725_DEV_ADD, OV7725_REG28, 0x01);
        simiic_write_reg(OV7725_DEV_ADD, OV7725_EXHCH, 0x10);
        simiic_write_reg(OV7725_DEV_ADD, OV7725_EXHCL, 0x1F);
        simiic_write_reg(OV7725_DEV_ADD, OV7725_GAM1, 0x0c);
        simiic_write_reg(OV7725_DEV_ADD, OV7725_GAM2, 0x16);
        simiic_write_reg(OV7725_DEV_ADD, OV7725_GAM3, 0x2a);
        simiic_write_reg(OV7725_DEV_ADD, OV7725_GAM4, 0x4e);
        simiic_write_reg(OV7725_DEV_ADD, OV7725_GAM5, 0x61);
        simiic_write_reg(OV7725_DEV_ADD, OV7725_GAM6, 0x6f);
        simiic_write_reg(OV7725_DEV_ADD, OV7725_GAM7, 0x7b);
        simiic_write_reg(OV7725_DEV_ADD, OV7725_GAM8, 0x86);
        simiic_write_reg(OV7725_DEV_ADD, OV7725_GAM9, 0x8e);
        simiic_write_reg(OV7725_DEV_ADD, OV7725_GAM10, 0x97);
        simiic_write_reg(OV7725_DEV_ADD, OV7725_GAM11, 0xa4);
        simiic_write_reg(OV7725_DEV_ADD, OV7725_GAM12, 0xaf);
        simiic_write_reg(OV7725_DEV_ADD, OV7725_GAM13, 0xc5);
        simiic_write_reg(OV7725_DEV_ADD, OV7725_GAM14, 0xd7);
        simiic_write_reg(OV7725_DEV_ADD, OV7725_GAM15, 0xe8);
        simiic_write_reg(OV7725_DEV_ADD, OV7725_SLOP, 0x20);
        simiic_write_reg(OV7725_DEV_ADD, OV7725_LC_RADI, 0x00);
        simiic_write_reg(OV7725_DEV_ADD, OV7725_LC_COEF, 0x13);
        simiic_write_reg(OV7725_DEV_ADD, OV7725_LC_XC, 0x08);
        simiic_write_reg(OV7725_DEV_ADD, OV7725_LC_COEFB, 0x14);
        simiic_write_reg(OV7725_DEV_ADD, OV7725_LC_COEFR, 0x17);
        simiic_write_reg(OV7725_DEV_ADD, OV7725_LC_CTR, 0x05);
        simiic_write_reg(OV7725_DEV_ADD, OV7725_BDBase, 0x99);
        simiic_write_reg(OV7725_DEV_ADD, OV7725_BDMStep, 0x03);
        simiic_write_reg(OV7725_DEV_ADD, OV7725_SDE, 0x04);
        simiic_write_reg(OV7725_DEV_ADD, OV7725_BRIGHT, 0x00);
        simiic_write_reg(OV7725_DEV_ADD, OV7725_CNST, 0x40);
        simiic_write_reg(OV7725_DEV_ADD, OV7725_SIGN, 0x06);
        simiic_write_reg(OV7725_DEV_ADD, OV7725_UVADJ0, 0x11);
        simiic_write_reg(OV7725_DEV_ADD, OV7725_UVADJ1, 0x02);
        return 1;
    }
    else        return 0;//错误
}



//-------------------------------------------------------------------------------------------------------------------
//  @brief      小钻风摄像头场中断回调函数
//  @param      NULL
//  @return     void
//  @since      v1.0
//  Sample usage:					该函数在stm32f1xx_it.c文件中的EXTI0_IRQHandler函数中进行回调
//-------------------------------------------------------------------------------------------------------------------
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    //使能TIM触发DMA链路。
    __HAL_TIM_ENABLE_DMA(&htim8, TIM_DMA_TRIGGER);
     //重置DMA
    HAL_DMA_Start_IT(&hdma_tim8_ch4_trig_com, (uint32_t)&GPIOE->IDR, (uint32_t)image_bin, OV7725_SIZE);
}


//-------------------------------------------------------------------------------------------------------------------
//  @brief      小钻风摄像头DMA完成中断
//  @param      NULL
//  @return     void
//  @since      v1.0
//  Sample usage:					//该函数已在SEEKFREE_7725.c中的ov7725_port_init函数中注册。
                                    //该函数在stm32f1xx_it.c文件中的DMA1_Channel4_IRQHandler函数中进行回调
//-------------------------------------------------------------------------------------------------------------------
void DMA_transfer_complete_callback(DMA_HandleTypeDef * _hdma)
{
    //图像传输完成标志位置1
    ov7725_finish_flag = 1;			
    //失能TIM触发DMA链路。
    __HAL_TIM_DISABLE_DMA(&htim8, TIM_DMA_TRIGGER);
}


//-------------------------------------------------------------------------------------------------------------------
//  @brief      小钻风摄像头采集程序初始化(内部使用，用户无需调用)
//  @param      NULL
//  @return     void
//  @since      v1.0
//  Sample usage:
//-------------------------------------------------------------------------------------------------------------------
void ov7725_port_init(void)
{
    //dma传输完成中断回调函数注册
    while(HAL_OK != HAL_DMA_RegisterCallback(&hdma_tim8_ch4_trig_com,HAL_DMA_XFER_CPLT_CB_ID,DMA_transfer_complete_callback))
    {
        HAL_DMA_Abort_IT(&hdma_tim8_ch4_trig_com);
    }
    
    //打开DMA中断
    HAL_DMA_Start_IT(&hdma_tim8_ch4_trig_com, (uint32_t)&GPIOE->IDR, (uint32_t)image_bin, OV7725_SIZE);
    
    //使能TIM触发DMA链路
    __HAL_TIM_ENABLE_DMA(&htim8, TIM_DMA_TRIGGER);
}



//-------------------------------------------------------------------------------------------------------------------
//  @brief      小钻风摄像头初始化(调用之后设置好相关中断函数即可采集图像)
//  @param      NULL
//  @return     0
//  @since      v1.0
//  Sample usage:
//-------------------------------------------------------------------------------------------------------------------
uint8_t ov7725_init(void)
{
    uint8_t f = ov7725_reg_init();          //摄像头寄存器配置

    if(f == 1)
    {
        put_char(&huart1,'A');
    }
    ov7725_port_init();         //TIM和DMA使能
    return 0;
}





//-------------------------------------------------------------------------------------------------------------------
//  @brief      串口输出8位数据
//  @param      USARTx			串口号
//  @param      Data			8位数据
//  @return     void
//  @since      v1.0
//  Sample usage:				调用该函数前请先初始化串口
//-------------------------------------------------------------------------------------------------------------------
void put_char(UART_HandleTypeDef* USARTx, uint8_t Data)
{
    while((USARTx->Instance->SR & 0X40)==0);
    USARTx->Instance->DR = Data;
}


        
//-------------------------------------------------------------------------------------------------------------------
//  @brief      小钻风摄像头未解压图像发送至上位机查看图像
//  @param      *imgaddr			压缩图像数据地址
//  @param      *imgsize			图像大小(直接填写OV7725_SIZE)
//  @return     void
//  @since      v1.0
//  Sample usage:					调用该函数前请先初始化串口
//-------------------------------------------------------------------------------------------------------------------
void seekfree_sendimg_7725(void *imgaddr, uint32_t imgsize)
{
//建议不要使用HAL库自带的串口发送函数，由于HAL库自带的串口发送函数多余的操作较多，可能导致串口发送数据不完整。
//建议不要使用HAL库自带的串口发送函数，由于HAL库自带的串口发送函数多余的操作较多，可能导致串口发送数据不完整。
//建议不要使用HAL库自带的串口发送函数，由于HAL库自带的串口发送函数多余的操作较多，可能导致串口发送数据不完整。

//使用HAL库串口发送函数发送图像数据。
//    uint8_t send_buff[4] = {0x00,0xFF,0x01,0x01};
//    HAL_UART_Transmit(&huart1,send_buff,4,0xffff);
//    HAL_UART_Transmit(&huart1,imgaddr,imgsize/2,10000);
  
//使用寄存器发送图像数据。
	uint32_t i;
	/*put_char(&huart1,0x00);
    put_char(&huart1,0xff);
    put_char(&huart1,0x01);
    put_char(&huart1,0x01);*/
    for(i=0; i<(imgsize); i++)
    {
        if(((uint8_t *)(imgaddr))[i] == 0)
        {
            put_char(&huart1,0xFE);
        }else{
            put_char(&huart1,0x00);
        }

    }
    put_char(&huart1,0xFF);
}





//-------------------------------------------------------------------------------------------------------------------
//  @brief      小钻风摄像头数据解压函数
//  @param      *data1				源地址
//  @param      *data2				目的地址
//  @return     void
//  @since      v1.0
//  Sample usage:					Image_Decompression(da1,dat2[0]);//将一维数组dat1的内容解压到二维数组dat2里.
//-------------------------------------------------------------------------------------------------------------------
void Image_Decompression(uint8_t *data1,uint8_t *data2)
{
    uint8_t  temp[2] = {0,255};
    uint16_t lenth = OV7725_SIZE;
    uint8_t  i = 8;
    while(lenth--)
    {
        i = 8;
        while(i--)
        {
            *data2++ = temp[(*data1 >> i) & 0x01];
        }
        data1++;
    }
}


