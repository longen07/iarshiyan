/*********************************************************************************************************************
 * COPYRIGHT NOTICE
 * Copyright (c) 2017,逐飞科技
 * All rights reserved.
 * 技术讨论QQ群：179029047
 * 以下所有内容版权均属逐飞科技所有，未经允许不得用于商业用途，
 * 欢迎各位使用并传播本程序，修改内容时必须保留逐飞科技的版权声明。
 * @file       		main
 * @company	   		成都逐飞科技有限公司
 * @author     		逐飞科技(QQ3184284598)
 * @version    		v2.0
 * @Software 		IAR 7.7 or MDK 5.23
 * @Target core		S9KEA128AMLK
 * @Taobao   		https://seekfree.taobao.com/
 * @date       		2017-11-6
 ********************************************************************************************************************/
/*********************************************************************************************************************
* @author               14届智能车-越野组成员 孙小伟，陈扬写
* @date       		2019-9-1
 ********************************************************************************************************************/
#include "headfile.h"
#include "show.h"
extern int Velocity;
static uint8 ok=1;
uint16 PID_Send,Flash_Send,APP_RX;//蓝牙遥控速度和APP接收的数据  
extern float Velocity_KP1,Velocity_KI1;//速度控制PID参数
extern int Motor_A,Motor_B,Target_A,Target_B,Velocity;  //电机舵机控制相关     
extern int Encoder_Left,Encoder_Right; //左右编码器的脉冲计数
extern uint16 baud;
extern int Sensor,PWMA1,PWMA2,PWMB1,PWMB2;
extern int8 look,look1;
extern float Turn,Turn1;
//int8 test;
int BiZhan1=24,BiZhan2=20,BiZhan3=10,BiZhan4=9;//BiZhan1=22
int cycle_direction_flag=1;
int cycle_f_flag=1,a_flag=1,cycle_ok_flag=1;

int BiZhan1_cc=0;
void Cycle_L_R(void)
{
  //cycle_deal();//在main的while前面，不能重复赋值
  if(Cycle_Flag)
  {
    //gpio_set(E1,1);
    if(a_flag==1)
    { 
        if(cycle_f_flag)
        {
          cycle_direction_flag=1;
          cycle_f_flag=0;
        }
       else
       {
          cycle_direction_flag=0;
          cycle_f_flag=1;
       }
    }
    else
    {
      
    }
    //if((ad_result[2]>(ad_result[3]+100)))
    if(cycle_direction_flag)
     {
       ftm_pwm_duty(ftm0,ftm_ch0,(uint16)(Turn_center-32));//30
       delay(140);
     } 
    else 
      //if((ad_result[3]>(ad_result[2]+100)))
      {
       ftm_pwm_duty(ftm0,ftm_ch0,(uint16)(Turn_center+25));//30
       delay(140);
      }
      gpio_set(E1,0);
    //gpio_set(E1,0);
  }
  Cycle_Flag=0;
}

void BiZhan(void) 
{
  int i,last_4=600;
if(BiZhan_Flag)
{     
  
  
       if(BiZhan1_cc==0)
         BiZhan_right=0;
       else 
         BiZhan_right=1;//右拐 
           BiZhan1_cc++;
       gpio_set(G1,0);//亮
      // gpio_set(E1,0);//蜂鸣器
             if(BiZhan_right==1)
             {
                   for(i=0;i<BiZhan1;i+=1)//while(cj_last<30)//
                   {
                     ftm_pwm_duty(ftm0,ftm_ch0,Turnmax);
                     delay(5);
                   }
                   
                   for(i=0;i<BiZhan2;i+=1)
                   {
                     ftm_pwm_duty(ftm0,ftm_ch0,Turn_center-82);
                     delay(10);
                     ad_result[0]=adc_ave(AD_CHANNEL7,ADC_12bit,5);
                     ad_result[1]=adc_ave(AD_CHANNEL2,ADC_12bit,5); 
                     ad_result[4]=adc_ave(AD_CHANNEL4,ADC_12bit,5);
                     if(((ad_result[0]-ad_result[1])>200)||((ad_result[1]-ad_result[0])>200))break;
                     if((ad_result[0]>200)&&(ad_result[1]>200))break;
                     if(ad_result[4]>last_4)break;
                   }
                   
                   for(int i=0;i<BiZhan3;i+=1)
                   {
                     ftm_pwm_duty(ftm0,ftm_ch0,Turn_center-70);
                     delay(10);
                     ad_result[0]=adc_ave(AD_CHANNEL7,ADC_12bit,5);
                     ad_result[1]=adc_ave(AD_CHANNEL2,ADC_12bit,5); 
                     ad_result[4]=adc_ave(AD_CHANNEL4,ADC_12bit,5);
                     if(((ad_result[0]-ad_result[1])>200)||((ad_result[1]-ad_result[0])>200))break;
                     if((ad_result[0]>200)&&(ad_result[1]>200))break;
                     if(ad_result[4]>last_4)break;           
                   }  
                   
                   for(int i=0;i<BiZhan4;i+=1)
                   {
                     ftm_pwm_duty(ftm0,ftm_ch0,Turn_center);
                     delay(10);
                     ad_result[0]=adc_ave(AD_CHANNEL7,ADC_12bit,5);
                     ad_result[1]=adc_ave(AD_CHANNEL2,ADC_12bit,5); 
                     ad_result[4]=adc_ave(AD_CHANNEL4,ADC_12bit,5);
                     if(((ad_result[0]-ad_result[1])>200)||((ad_result[1]-ad_result[0])>200))break;
                     if((ad_result[0]>200)&&(ad_result[1]>200))break;
                     if(ad_result[4]>last_4)break;
                   }
                   
                   for(int i=0;i<25;i+=1)
                   {
                     ftm_pwm_duty(ftm0,ftm_ch0,Turn_center+20);
                     delay(10);
                     ad_result[0]=adc_ave(AD_CHANNEL7,ADC_12bit,5);
                     ad_result[1]=adc_ave(AD_CHANNEL2,ADC_12bit,5); 
                     ad_result[4]=adc_ave(AD_CHANNEL4,ADC_12bit,5);
                     if(((ad_result[0]-ad_result[1])>200)||((ad_result[1]-ad_result[0])>200))break;
                     if((ad_result[0]>200)&&(ad_result[1]>200))break;
                     if(ad_result[4]>last_4)break;
                   }
                   
             }
             
             else
             {
                   for(i=0;i<BiZhan1;i+=1)//while(cj_last<30)//
                   {
                     ftm_pwm_duty(ftm0,ftm_ch0,Turnmin);
                     delay(5);
                   }
                   
                   for(i=0;i<BiZhan2;i+=1)
                   {
                     ftm_pwm_duty(ftm0,ftm_ch0,Turn_center+82);
                     delay(10);
                     ad_result[0]=adc_ave(AD_CHANNEL7,ADC_12bit,5);
                     ad_result[1]=adc_ave(AD_CHANNEL2,ADC_12bit,5); 
                     ad_result[4]=adc_ave(AD_CHANNEL4,ADC_12bit,5);
                     if(((ad_result[0]-ad_result[1])>200)||((ad_result[1]-ad_result[0])>200))break;
                     if((ad_result[0]>200)&&(ad_result[1]>200))break;
                     if(ad_result[4]>last_4)break;
                   }
                   
                   for(int i=0;i<BiZhan3;i+=1)
                   {
                     ftm_pwm_duty(ftm0,ftm_ch0,Turn_center+70);
                     delay(10);
                     ad_result[0]=adc_ave(AD_CHANNEL7,ADC_12bit,5);
                     ad_result[1]=adc_ave(AD_CHANNEL2,ADC_12bit,5); 
                     ad_result[4]=adc_ave(AD_CHANNEL4,ADC_12bit,5);
                     if(((ad_result[0]-ad_result[1])>200)||((ad_result[1]-ad_result[0])>200))break;
                     if((ad_result[0]>200)&&(ad_result[1]>200))break;
                     if(ad_result[4]>last_4)break;                
                   }  
                   
                   for(int i=0;i<BiZhan4;i+=1)
                   {
                     ftm_pwm_duty(ftm0,ftm_ch0,Turn_center);
                     delay(10);
                     ad_result[0]=adc_ave(AD_CHANNEL7,ADC_12bit,5);
                     ad_result[1]=adc_ave(AD_CHANNEL2,ADC_12bit,5); 
                     ad_result[4]=adc_ave(AD_CHANNEL4,ADC_12bit,5);
                     if(((ad_result[0]-ad_result[1])>200)||((ad_result[1]-ad_result[0])>200))break;
                     if((ad_result[0]>200)&&(ad_result[1]>200))break;
                     if(ad_result[4]>last_4)break;
                   }
                   
                   for(int i=0;i<25;i+=1)
                   {
                     ftm_pwm_duty(ftm0,ftm_ch0,Turn_center-20);
                     delay(10);
                     ad_result[0]=adc_ave(AD_CHANNEL7,ADC_12bit,5);
                     ad_result[1]=adc_ave(AD_CHANNEL2,ADC_12bit,5); 
                     ad_result[4]=adc_ave(AD_CHANNEL4,ADC_12bit,5);
                     if(((ad_result[0]-ad_result[1])>200)||((ad_result[1]-ad_result[0])>200))break;
                     if((ad_result[0]>200)&&(ad_result[1]>200))break;
                     if(ad_result[4]>last_4)break;
                   }
             }
            
             
      // EnableInterrupts;
      
}
BiZhan_Flag=0; 
gpio_set(G1,1);//关

}


int main(void)
{   
    
    DisableInterrupts ;     //禁止全部中断
    gpio_init(I1,GPO,0);
    get_clk();              //获取时钟频率 必须执行、   
    OLED_Init();//OLED与单片机接线请查看OLED文件最上方注 释    
    ftminit();//编码器初始化
    key_init();//按键初始化
    gpio_init(I0,GPI,0);
    
    car_init();//传感器采值
    //FLASH_Init();
    
    gpio_init(G0, GPO, 1);//PTG0 led /gpio_init(A5,GPO,0);  main函数检测
    gpio_init(G1, GPO, 1);//PTG1 led                         pwm赋值检测
    gpio_init(G2, GPO, 1);//PTG2 led                       单片机蓝牙串口检测
    gpio_init(G3, GPO, 1);//最下面第四个led灯              中断里控制函数检测
    
    gpio_init(E1, GPO, 0);//E1为蜂鸣器引脚  蜂鸣器为高电平响 低电平不响
    set_irq_priority(PIT_CH1_IRQn,1);//设置中断优先级
    
     
    
     gpio_init(I4,GPI,1); //停车检测IO端口  I4为干簧管检测停车端口 高电平有效 没有这句话事实还是可以的？
     //port_pull(I4);    
     irq_init();//初始化外部中断
     set_irq_priority(IRQ_IRQn,0);//设置优先级,根据自己的需求设置 可设置范围为 0 - 3
     enable_irq(IRQ_IRQn);//打开外部中断IRQ的中断开关 

    SC_black_Init();
    EnableInterrupts;   //开总中断（凡是用到中断的，都需要的）
      
    if(cycle_ok_flag)
    { 
      key_deal();
      key_scan();
      cycle_deal();//设置圆环模式
      oledshow();
    }
    
       
    while(1)
    {   
    //gpio_set(E1,1);//蜂鸣器响
    Cycle_L_R();
//     BiZhan();
     ////////
      gpio_set(G2,0);
      delay(10);
      gpio_set(G2,1);
      /////////
      key_deal2();//屏幕切换关闭和打开    
    
     if(ok==1)
   {
      oledshow();			 
     key_scan();
      key_deal();	 
//          
//   //printf("{B%d:%d:%d:%d:%d}$", Encoder_Left,Target_B,Encoder_Right,Target_A,80);//打印到APP上面 显示波形                                     //波形0      波形1       波形2     波形3      波形4       
   }    
//      else
//         { }                
         
                   
     }   
 }




void key_deal2(void)
{   
    static uint8 key9_last_status=1;
    uint8 key9_status;
    uint8 key9_flag = 0;
    key9_status = gpio_get(KEY9);
    if((0==key9_last_status) && (1==key9_status))   key9_flag = 1;//检测到上升沿，置位按键标志位
    key9_last_status = key9_status;        
    
    if(key9_flag==1)       //按键是否有效
    { key9_flag = 0;  //按键已使用，清零标志位
      ok=!ok;  
      OLED_Fill(0x00);
    }
}



