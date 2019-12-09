/*********************************************************************************************************************
 * COPYRIGHT NOTICE
 * Copyright (c) 2017,��ɿƼ�
 * All rights reserved.
 * ��������QQȺ��179029047
 * @author     		��ɿƼ�(QQ3184284598)
 * @version    		v2.0
 * @Software 		IAR 7.7 or MDK 5.23
 * @Target core		S9KEA128AMLK
 * @Taobao   		https://seekfree.taobao.com/
 * @date       		2017-11-6
 ********************************************************************************************************************/

#include "isr.h"
#include  "calculate.h"  
#include "show.h"
#include "KEA128_uart.h"
#include "string.h"
#include "math.h"
#include "KEA128_pit.h"

uint8 TIME0flag_5ms = 0 ;


int Encoder_A_EXTI,Flag_Direction; 
unsigned char Usart3_Receive;

extern UART_Type * uart[3];
extern uint16 baud;
extern uint16 Bluetooth_Velocity,PID_Send,Flash_Send,APP_RX; 
extern float Velocity_KP1,Velocity_KI1;	       //�ٶȿ���PID����
extern float Velocity_KP2,Velocity_KI2;	       //�ٶȿ���PID����
extern float Turn_P,Turn_I,Turn_D;        //�¼ӵ� �����������ֻ����Կ��Ʋ���
extern int temp2;
extern float Turn;
extern uint16 ad_result[8];
extern float Direction;
uint8 stop_ready=0;
int BiZhan_Flag=0;
short int cj_count=0;
float cj_last=30;
int cj_flag=1,speed_fq_flag=1,d_cj_flag=0,cj_limit_flag=0;//  �ٶȷ����ر�
int f_pid_count=0,f_pid_flag=0,pid_change_flag=0;

float Turn_P_Z=3,Turn_D_Z=110,//ֱ��Turn_P_Z=7.9,Turn_D_Z=200,
       Turn_P_W=8.7,Turn_D_W=200;//���Turn_P_W=8.7,Turn_D_W=200;


uint32 timer = 0xffffffff;
void PIT_CH0_IRQHandler(void)
{
    PIT_FlAG_CLR(pit0);   
}

void PIT_CH1_IRQHandler(void)//5ms
{ 
  static int cj_time=0;
  uint32 time=0;
 // static int speed_fq_time=0,speed_fq_time_flag=0;
  gpio_set(G3,0);//��
  if(!(BiZhan_Flag||Cycle_Flag))
  { 
      cj_time++;
      if(cj_time>8)
      {   
          gpio_set(I1,1);  //����������ź�

      }
  
      if(pid_change_flag)
       {
         Turn_P=Turn_P_W;//ֱ��
         Turn_D=Turn_D_W;
       }
        else if(( (ad_result[2]<200)&&(ad_result[3]<200))||((Turn<(Turnmax-100))&&(Turn>(Turnmin+100))) )  //ֱ�����PD//160
               {
                     if(ad_result[4]>700)
                     {
                         Turn_P=Turn_P_Z;//ֱ��
                         Turn_D=Turn_D_Z;
                     }
                    else
                   {              
                         Turn_P=Turn_P_W;//���
                         Turn_D=Turn_D_W;//���
                         f_pid_flag=1;
                   }
               }             
               else
               {              
                 Turn_P=Turn_P_W;//���
                 Turn_D=Turn_D_W;//���
                 f_pid_flag=1;
               }
      
      if(f_pid_flag)
      {
          f_pid_count++;
     
          Turn_P=Turn_P_W;//
          Turn_D=Turn_D_W;
          if(f_pid_count>35)//44
          {
            f_pid_count=0;
            f_pid_flag=0;
          }
      }
  }
  PIT_FlAG_CLR(pit1);  //�жϱ�־λ����

  temp2=ftm_count_get(ftm1);
        
  Encoder_Right=Read_Encoder();//temp2;
        
  if(!(BiZhan_Flag||Cycle_Flag))
  {
    ADcontrol();
  }
        
  PDcontrol(); 
  
  if(!(BiZhan_Flag||Cycle_Flag))
  {
       if(cj_time>8)//8
       {
         gpio_set(I1,0);
         cj_time=0;
          
       }       
  }
  
    if((Encoder_Right<=0)&&(BiZhan_Flag==1))//&&(Direction>60))
    {
      //d_cj_flag = 1 ;
        
    }
    
    if(gpio_get(I0))   //Ϊ�˷�ֹ�������Ĵ����ж�����������˵͵�ƽ
   {
     time = pit_time_get(pit0)/1000u;
     if(time > 200u)
     {
      gpio_init(I0, GPI, 0);
     }    
   }
  gpio_set(G3,1);//��       
          
          
          
//     if(stop_ready&&tt<2000) tt++; //����t��ֵ��ֹt����������ױ�־λ�ļ��  �����ⲿ�жϵ��ж� 
//     if(i++>50)     
//     {
//       gpio_turn(G0);
//       i = 0;
}



void IRQ_IRQHandler(void)  //�ⲿ�жϱ��ϱ�־//ͣ���������ⲿ�жϸɻɹܼ��  I4�˿�
{  
  int abc;
  int cj_length=200;//185
  if(gpio_get(I0))
   {
     pit_time_start(pit0);
     irq_init_1();   //�½��ش����ж�
   }
   else
   {
     timer = pit_time_get(pit0)/1000u; 
     pit_close(pit0);
     irq_init();     //�����ش����ж�
   }
  
  if(d_cj_flag)//��ʱ�����Ʊ���
    cj_limit_flag=0;//�����Ʊ���
  else
    cj_limit_flag=1; //���Ʊ���
    
  if(cj_limit_flag)
     abc=(Turn>390)&&(Turn<510)&&cj_flag;
  else
     abc=1;
if(abc)
//if(1)
 {   
      cj_last=timer; 
    //  gpio_set(G2,0);//��      
      //d_cj_flag = 0;
     if((cj_last<cj_length)&&(cj_last>7)) 
     {
          BiZhan_Flag=1;
          d_cj_flag = 0;
        //  gpio_set(E1,1);//��������
         if((cj_last<140))
         {
            d_cj_flag = 1 ;
            BiZhan_Flag = 0;
         }
      }
      else
        d_cj_flag = 0;     
   //   gpio_set(G2,1);//��  
 }   
  else 
    cj_last=cj_length+1;

//  }
////static uint16 n = 0;	 
////    systick_delay_ms(50);
////    k++;
////    gpio_turn(G3);//G2�ڵ�ƽ��ת
////    if(tt==0)
////    // stop_flag=1;  
////    stop_ready=1;
////    if(tt>1600)
////    {
////      stop_flag=1;  
////                 }
  
    
    CLEAR_IRQ_FLAG;
}
/**************************************************************************
�������ܣ�����2�����ж�
��ڲ�������
����  ֵ����
**************************************************************************/

//����APP������ Ȼ������Ӧ�Ĵ���
//void UART2_IRQHandler(void)//����2�����жϷ������
//{     
//    
//  if((uart[uart2]->S1 & UART_S1_RDRF_MASK))//���յ�����
//  {	  
//       gpio_turn(G2);//GO�ڵ�ƽ��ת
//       static unsigned char Flag_PID,i,j,Receive[50];
//       static unsigned int  Data;
//        Usart3_Receive=uart[uart2]->D; 
//        APP_RX=Usart3_Receive;
//        if(Usart3_Receive>=0x41&&Usart3_Receive<=0x48)  
//        Flag_Direction=Usart3_Receive-0x40;
//        else 	if(Usart3_Receive<10)  
//        Flag_Direction=Usart3_Receive;	
//        else 	if(Usart3_Receive==0X5A)      
//        Flag_Direction=0;	
//                  
//                          //��������APP���Խ���ͨѶ
//        if(Usart3_Receive==0x7B) Flag_PID=1;   //APP����ָ����ʼλ
//        if(Usart3_Receive==0x7D) Flag_PID=2;   //APP����ָ��ֹͣλ
//
//           if(Flag_PID==1)  //�ɼ�����
//           {
//                  Receive[i]=Usart3_Receive;
//                  i++;
//           }
//           if(Flag_PID==2)  //��������
//           {
//                 if(Receive[3]==0x50) 	       PID_Send=1;//��ӦAPP-��ȡ�豸����
//                 else  if(Receive[3]==0x57)   Flash_Send=1;//��Ӧapp-���õ��籣�����������д��flash��
//                 else  if(Receive[1]!=0x23) 
//                 {								
//                        for(j=i;j>=4;j--)//���ݴ�����Ҫ�Ķ�
//                        {
//                          Data+=(Receive[j-1]-48)*Mypow(10,i-j);
//                       
//                        }
//                        switch(Receive[1])
//                         {
//                                 case 0x30:  Bluetooth_Velocity=Data;break;//��ӦAPP-����-����0
//                                 case 0x31:  Velocity_KP1=Data;break;//��ӦAPP-����-����1
//                                 case 0x32:  Velocity_KI1=Data;break;//��ӦAPP-����-����2
//                                 case 0x33:  Turn_P=Data;break;//��ӦAPP-����-����3
//                                 case 0x34:  Turn_D=Data;break;                                 
//                                 case 0x35:  Turn_I=Data;break;
//                                 case 0x36:  Turn_P1=Data;break;
//                                 case 0x37:  Turn_D1=Data;break; //Ԥ��
//                                 case 0x38:  break; //Ԥ��
//                         }
//                 }				 
//                 Flag_PID=0;//��ر�־λ����
//                 i=0;
//                 j=0;
//                 Data=0;
//                 memset(Receive, 0, sizeof(uint16)*50);//��������
//           } 	 			
// }  
//  
//}




void KBI0_IRQHandler(void)
{
    
  
    CLEAN_KBI0_FLAG;
    
}



/*
�жϺ������ƣ��������ö�Ӧ���ܵ��жϺ���
Sample usage:��ǰ���������ڶ�ʱ�� ͨ��0���ж�
void PIT_CH0_IRQHandler(void)
{
    ;
}
�ǵý����жϺ������־λ

FTMRE_IRQHandler      
PMC_IRQHandler        
IRQ_IRQHandler        
I2C0_IRQHandler       
I2C1_IRQHandler       
SPI0_IRQHandler       
SPI1_IRQHandler       
UART0_IRQHandler 
UART1_IRQHandler 
UART2_IRQHandler 
ADC0_IRQHandler       
ACMP0_IRQHandler      
FTM0_IRQHandler       
FTM1_IRQHandler       
FTM2_IRQHandler       
RTC_IRQHandler        
ACMP1_IRQHandler      
PIT_CH0_IRQHandler    
PIT_CH1_IRQHandler    
KBI0_IRQHandler       
KBI1_IRQHandler       
Reserved26_IRQHandler 
ICS_IRQHandler        
WDG_IRQHandler        
PWT_IRQHandler        
MSCAN_Rx_IRQHandler   
MSCAN_Tx_IRQHandler   
*/



