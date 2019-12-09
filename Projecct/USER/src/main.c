/*********************************************************************************************************************
 * COPYRIGHT NOTICE
 * Copyright (c) 2017,��ɿƼ�
 * All rights reserved.
 * ��������QQȺ��179029047
 * �����������ݰ�Ȩ������ɿƼ����У�δ��������������ҵ��;��
 * ��ӭ��λʹ�ò������������޸�����ʱ���뱣����ɿƼ��İ�Ȩ������
 * @file       		main
 * @company	   		�ɶ���ɿƼ����޹�˾
 * @author     		��ɿƼ�(QQ3184284598)
 * @version    		v2.0
 * @Software 		IAR 7.7 or MDK 5.23
 * @Target core		S9KEA128AMLK
 * @Taobao   		https://seekfree.taobao.com/
 * @date       		2017-11-6
 ********************************************************************************************************************/
/*********************************************************************************************************************
* @author               14�����ܳ�-ԽҰ���Ա ��Сΰ������д
* @date       		2019-9-1
 ********************************************************************************************************************/
#include "headfile.h"
#include "show.h"
extern int Velocity;
static uint8 ok=1;
uint16 PID_Send,Flash_Send,APP_RX;//����ң���ٶȺ�APP���յ�����  
extern float Velocity_KP1,Velocity_KI1;//�ٶȿ���PID����
extern int Motor_A,Motor_B,Target_A,Target_B,Velocity;  //�������������     
extern int Encoder_Left,Encoder_Right; //���ұ��������������
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
  //cycle_deal();//��main��whileǰ�棬�����ظ���ֵ
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
         BiZhan_right=1;//�ҹ� 
           BiZhan1_cc++;
       gpio_set(G1,0);//��
      // gpio_set(E1,0);//������
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
gpio_set(G1,1);//��

}


int main(void)
{   
    
    DisableInterrupts ;     //��ֹȫ���ж�
    gpio_init(I1,GPO,0);
    get_clk();              //��ȡʱ��Ƶ�� ����ִ�С�   
    OLED_Init();//OLED�뵥Ƭ��������鿴OLED�ļ����Ϸ�ע ��    
    ftminit();//��������ʼ��
    key_init();//������ʼ��
    gpio_init(I0,GPI,0);
    
    car_init();//��������ֵ
    //FLASH_Init();
    
    gpio_init(G0, GPO, 1);//PTG0 led /gpio_init(A5,GPO,0);  main�������
    gpio_init(G1, GPO, 1);//PTG1 led                         pwm��ֵ���
    gpio_init(G2, GPO, 1);//PTG2 led                       ��Ƭ���������ڼ��
    gpio_init(G3, GPO, 1);//��������ĸ�led��              �ж�����ƺ������
    
    gpio_init(E1, GPO, 0);//E1Ϊ����������  ������Ϊ�ߵ�ƽ�� �͵�ƽ����
    set_irq_priority(PIT_CH1_IRQn,1);//�����ж����ȼ�
    
     
    
     gpio_init(I4,GPI,1); //ͣ�����IO�˿�  I4Ϊ�ɻɹܼ��ͣ���˿� �ߵ�ƽ��Ч û����仰��ʵ���ǿ��Եģ�
     //port_pull(I4);    
     irq_init();//��ʼ���ⲿ�ж�
     set_irq_priority(IRQ_IRQn,0);//�������ȼ�,�����Լ����������� �����÷�ΧΪ 0 - 3
     enable_irq(IRQ_IRQn);//���ⲿ�ж�IRQ���жϿ��� 

    SC_black_Init();
    EnableInterrupts;   //�����жϣ������õ��жϵģ�����Ҫ�ģ�
      
    if(cycle_ok_flag)
    { 
      key_deal();
      key_scan();
      cycle_deal();//����Բ��ģʽ
      oledshow();
    }
    
       
    while(1)
    {   
    //gpio_set(E1,1);//��������
    Cycle_L_R();
//     BiZhan();
     ////////
      gpio_set(G2,0);
      delay(10);
      gpio_set(G2,1);
      /////////
      key_deal2();//��Ļ�л��رպʹ�    
    
     if(ok==1)
   {
      oledshow();			 
     key_scan();
      key_deal();	 
//          
//   //printf("{B%d:%d:%d:%d:%d}$", Encoder_Left,Target_B,Encoder_Right,Target_A,80);//��ӡ��APP���� ��ʾ����                                     //����0      ����1       ����2     ����3      ����4       
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
    if((0==key9_last_status) && (1==key9_status))   key9_flag = 1;//��⵽�����أ���λ������־λ
    key9_last_status = key9_status;        
    
    if(key9_flag==1)       //�����Ƿ���Ч
    { key9_flag = 0;  //������ʹ�ã������־λ
      ok=!ok;  
      OLED_Fill(0x00);
    }
}



