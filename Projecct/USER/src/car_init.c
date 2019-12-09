/*********************************************************************************************************************
 * 
 * @file       		car_init.c
 *  		
 * @core			S9KEA128
 * @date       		2018
 ********************************************************************************************************************/

#include "headfile.h"


void car_init()                         //初始化PWM频率为15K，占空比为0  
{    
    //这里一共四个通道，本历程使用通道2与通道3控制左电机的正反转，剩下4和5通道控制右电机的正反转
    ftm_pwm_init(ftm2,ftm_ch2,14000,0);//右电机 100为正正转  /这里需要注意，单片机是无法直接驱动电机的，单片机只能给出控制信号，将控制信号给驱动模块，驱动模块输出电压使得电机转动    
    ftm_pwm_init(ftm2,ftm_ch3,14000,0);//可查看我们店铺MOS管驱动模块
    ftm_pwm_init(ftm2,ftm_ch4,14000,0);//左电机 100为正正转               //桌大大的推文中，建议电磁组电机频率选用13K-17K                                                                
    ftm_pwm_init(ftm2,ftm_ch5,14000,0);//占空比精度可以在KEA128_ftm.h文件内找到FTM2_PRECISON宏定义进行设置
    //ftm_pwm_init(ftm0,ftm_ch0,200,200);//
    ftm_pwm_init(ftm0,ftm_ch0,300,500);//舵机初始化                                                              
    //初始化AD通道
   //采集电磁信息，需要有外部电路的支持，具体电路课查看我们店铺的运放模块和电感电容。
    //AD通道连接运放的输出，即可采集磁场信息
    adc_init(AD_CHANNEL1);
    adc_init(AD_CHANNEL2);
    adc_init(AD_CHANNEL3);
    adc_init(AD_CHANNEL4);
    adc_init(AD_CHANNEL5);
    adc_init(AD_CHANNEL6);
    adc_init(AD_CHANNEL7);
    adc_init(AD_CHANNEL8);
    
    
    
    



}

void key_init(void)      //按键初始化
{
    gpio_init(KEY1,GPI,0);  //初始化IO口
    gpio_init(KEY2,GPI,0);
    gpio_init(KEY3,GPI,0);
    gpio_init(KEY4,GPI,0);
    gpio_init(KEY5,GPI,0);  //初始化IO口
    gpio_init(KEY6,GPI,0);
    gpio_init(KEY7,GPI,0);
    gpio_init(KEY8,GPI,0);
    gpio_init(KEY9,GPI,0);
    
//    port_pull(KEY1);        //设置IO口内部上拉，或者外部上拉
//    port_pull(KEY2);
//    port_pull(KEY3);
//    port_pull(KEY4);
//    port_pull(KEY5);        //设置IO口内部上拉，或者外部上拉
//    port_pull(KEY6);
//    port_pull(KEY7);
//    port_pull(KEY8);
//    port_pull(KEY9);    
}

