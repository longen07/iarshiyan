/*********************************************************************************************************************
 * 
 * @file       		car_init.c
 *  		
 * @core			S9KEA128
 * @date       		2018
 ********************************************************************************************************************/

#include "headfile.h"


void car_init()                         //��ʼ��PWMƵ��Ϊ15K��ռ�ձ�Ϊ0  
{    
    //����һ���ĸ�ͨ����������ʹ��ͨ��2��ͨ��3��������������ת��ʣ��4��5ͨ�������ҵ��������ת
    ftm_pwm_init(ftm2,ftm_ch2,14000,0);//�ҵ�� 100Ϊ����ת  /������Ҫע�⣬��Ƭ�����޷�ֱ����������ģ���Ƭ��ֻ�ܸ��������źţ��������źŸ�����ģ�飬����ģ�������ѹʹ�õ��ת��    
    ftm_pwm_init(ftm2,ftm_ch3,14000,0);//�ɲ鿴���ǵ���MOS������ģ��
    ftm_pwm_init(ftm2,ftm_ch4,14000,0);//���� 100Ϊ����ת               //�����������У�����������Ƶ��ѡ��13K-17K                                                                
    ftm_pwm_init(ftm2,ftm_ch5,14000,0);//ռ�ձȾ��ȿ�����KEA128_ftm.h�ļ����ҵ�FTM2_PRECISON�궨���������
    //ftm_pwm_init(ftm0,ftm_ch0,200,200);//
    ftm_pwm_init(ftm0,ftm_ch0,300,500);//�����ʼ��                                                              
    //��ʼ��ADͨ��
   //�ɼ������Ϣ����Ҫ���ⲿ��·��֧�֣������·�β鿴���ǵ��̵��˷�ģ��͵�е��ݡ�
    //ADͨ�������˷ŵ���������ɲɼ��ų���Ϣ
    adc_init(AD_CHANNEL1);
    adc_init(AD_CHANNEL2);
    adc_init(AD_CHANNEL3);
    adc_init(AD_CHANNEL4);
    adc_init(AD_CHANNEL5);
    adc_init(AD_CHANNEL6);
    adc_init(AD_CHANNEL7);
    adc_init(AD_CHANNEL8);
    
    
    
    



}

void key_init(void)      //������ʼ��
{
    gpio_init(KEY1,GPI,0);  //��ʼ��IO��
    gpio_init(KEY2,GPI,0);
    gpio_init(KEY3,GPI,0);
    gpio_init(KEY4,GPI,0);
    gpio_init(KEY5,GPI,0);  //��ʼ��IO��
    gpio_init(KEY6,GPI,0);
    gpio_init(KEY7,GPI,0);
    gpio_init(KEY8,GPI,0);
    gpio_init(KEY9,GPI,0);
    
//    port_pull(KEY1);        //����IO���ڲ������������ⲿ����
//    port_pull(KEY2);
//    port_pull(KEY3);
//    port_pull(KEY4);
//    port_pull(KEY5);        //����IO���ڲ������������ⲿ����
//    port_pull(KEY6);
//    port_pull(KEY7);
//    port_pull(KEY8);
//    port_pull(KEY9);    
}

