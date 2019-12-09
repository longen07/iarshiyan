#include "show.h"
#include "calculate.h"
  /**************************************************************************
���ߣ�ƽ��С��֮��
�ҵ��Ա�С�꣺http://shop114407458.taobao.com/
**************************************************************************/
#define SERVO_INIT 750


unsigned char i;          //��������
unsigned char Send_Count; //������Ҫ���͵����ݸ���
float Vol;
//extern int Voltage;//��ص�ѹ������صı���
extern int Encoder_Left,Encoder_Right; //���ұ��������������
extern int Motor_A,Motor_B,Servo,Target_A,Target_B;  //�������������     
extern int Sensor,PWMA1,PWMA2,PWMB1,PWMB2;
extern float Velocity_KP1,Velocity_KI1;	       //�ٶȿ���PID����
extern float Turn_P,Turn_I,Turn_D;        //�¼ӵ� �����������ֻ����Կ��Ʋ��� 
extern float Velocity,Velocity_Set,Angle_Set;
extern float Turn;

extern uint16 Bluetooth_Velocity,PID_Send,Flash_Send,APP_RX;





/**************************************************************************
�������ܣ�OLED��ʾ
��ڲ�������
����  ֵ����
**************************************************************************/
void oled_show(void)
{/*
	
		if(Flag_Way==3)
		{
			OLED_ShowString(00,0,"L");
			OLED_ShowNumber(90,0,Sensor_Left,4,12);	
			OLED_ShowString(40,0,"M");
			OLED_ShowNumber(50,0,Sensor_Middle,4,12);
			OLED_ShowString(80,0,"R");
			OLED_ShowNumber(10,0,Sensor_Right,4,12);
			OLED_ShowString(0,10,"Z");
			OLED_ShowNumber(20,10,Sensor,4,12);		  
		}									
		                      OLED_ShowString(00,20,"EncoLEFT");    //����������
		if( Encoder_Left<0)		OLED_ShowString(80,20,"-"),
		                      OLED_ShowNumber(95,20,-Encoder_Left,5,12);
		else                 	OLED_ShowString(80,20,"+"),
		                      OLED_ShowNumber(95,20, Encoder_Left,5,12);

		                      OLED_ShowString(00,30,"EncoRIGHT");
		if(Encoder_Right<0)	  OLED_ShowString(80,30,"-"),
		                      OLED_ShowNumber(95,30,-Encoder_Right,5,12);
		else               		OLED_ShowString(80,30,"+"),
		                      OLED_ShowNumber(95,30,Encoder_Right,5,12);	


		                      OLED_ShowString(00,40,"VOLTAGE");
		                      OLED_ShowString(68,40,".");
		                      OLED_ShowString(90,40,"V");
		                      OLED_ShowNumber(55,40,Voltage/100,2,12);
		                      OLED_ShowNumber(78,40,Voltage%100,2,12);
		 if(Voltage%100<10) 	OLED_ShowNumber(72,40,0,2,12);

                        if(Flag_Stop==0)
                        OLED_ShowString(103,40,"O-N");
                        if(Flag_Stop==1)
                        OLED_ShowString(103,40,"OFF");


                        OLED_ShowString(00,50,"MODE-");
                        if(Flag_Way==0)               OLED_ShowString(40,50,"APP");
                        else if(Flag_Way==1)          OLED_ShowString(40,50,"PS2");
                        else if(Flag_Way==2)	      OLED_ShowString(40,50,"CCD");
                        else if(Flag_Way==3)	      OLED_ShowString(40,50,"ELE");

                //	OLED_ShowString(80,50,"S");
                //	OLED_ShowNumber(95,50,Servo,4,12);
		//=============ˢ��=======================//
		OLED_Refresh_Gram();	*/
	}
/**************************************************************************
�������ܣ���APP��������
��ڲ�������
����  ֵ����
��    �ߣ�ƽ��С��֮��
**************************************************************************/
void APP_Show(void)
{    
	  static uint8 flag=0;
//	  int app_2,app_3,app_4;
//	  app_4=0;//(Voltage-740)*2/3;if(app_4<0)app_4=0;if(app_4>100)app_4=100;   //�Ե�ѹ���ݽ��д���
//	  app_3=(int)(Encoder_Right*1.1); if(app_3<0)app_3=-app_3;//�Ա��������ݾ������ݴ������ͼ�λ�
//	  app_2=(int)(Encoder_Left*1.1);  if(app_2<0)app_2=-app_2;//
	  flag=!flag;
        if(PID_Send==1)//���û���APP��ѡ�񡰻�ȡ�豸������ �ͻ�ִ�����if 
	{
		printf("{C%d:%d:%d:%d:%d:%d:%d:%d:%d}$",(int)Bluetooth_Velocity,
                       (int)Velocity_KP1,(int)Velocity_KI1,(int)Turn_P,(int)Turn_I,(int)Turn_D,0,0,0);//��ӡ��APP����  ��APP�ϵ��ȡ�豸������	
		PID_Send=0;	
	}	
//        else  if(flag==0)// 1��������   2��������  3������  4.�Ƕȣ�δʹ�ã�
//        printf("{A%d:%d:%d:%d}$",(uint8)app_2,(uint8)app_3,app_4,(int)((Servo-SERVO_INIT)*0.16)); //��ӡ��APP����
//	else
	if(flag==0)
        printf("{B%d:%d:%d:%d:%d}$", Encoder_Left,Target_A,Encoder_Right,PWMA1,PWMB1);//��ӡ��APP���� ��ʾ����
                                      //����0      ����1       ����2     ����3      ����4

}

