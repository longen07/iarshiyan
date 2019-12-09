#include "show.h"
#include "calculate.h"
  /**************************************************************************
作者：平衡小车之家
我的淘宝小店：http://shop114407458.taobao.com/
**************************************************************************/
#define SERVO_INIT 750


unsigned char i;          //计数变量
unsigned char Send_Count; //串口需要发送的数据个数
float Vol;
//extern int Voltage;//电池电压采样相关的变量
extern int Encoder_Left,Encoder_Right; //左右编码器的脉冲计数
extern int Motor_A,Motor_B,Servo,Target_A,Target_B;  //电机舵机控制相关     
extern int Sensor,PWMA1,PWMA2,PWMB1,PWMB2;
extern float Velocity_KP1,Velocity_KI1;	       //速度控制PID参数
extern float Turn_P,Turn_I,Turn_D;        //新加的 增加三个以手机可以控制参数 
extern float Velocity,Velocity_Set,Angle_Set;
extern float Turn;

extern uint16 Bluetooth_Velocity,PID_Send,Flash_Send,APP_RX;





/**************************************************************************
函数功能：OLED显示
入口参数：无
返回  值：无
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
		                      OLED_ShowString(00,20,"EncoLEFT");    //编码器数据
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
		//=============刷新=======================//
		OLED_Refresh_Gram();	*/
	}
/**************************************************************************
函数功能：向APP发送数据
入口参数：无
返回  值：无
作    者：平衡小车之家
**************************************************************************/
void APP_Show(void)
{    
	  static uint8 flag=0;
//	  int app_2,app_3,app_4;
//	  app_4=0;//(Voltage-740)*2/3;if(app_4<0)app_4=0;if(app_4>100)app_4=100;   //对电压数据进行处理
//	  app_3=(int)(Encoder_Right*1.1); if(app_3<0)app_3=-app_3;//对编码器数据就行数据处理便于图形化
//	  app_2=(int)(Encoder_Left*1.1);  if(app_2<0)app_2=-app_2;//
	  flag=!flag;
        if(PID_Send==1)//若用户在APP上选择“获取设备参数” 就会执行这个if 
	{
		printf("{C%d:%d:%d:%d:%d:%d:%d:%d:%d}$",(int)Bluetooth_Velocity,
                       (int)Velocity_KP1,(int)Velocity_KI1,(int)Turn_P,(int)Turn_I,(int)Turn_D,0,0,0);//打印到APP上面  （APP上点获取设备参数）	
		PID_Send=0;	
	}	
//        else  if(flag==0)// 1：左码盘   2：右码盘  3：电量  4.角度（未使用）
//        printf("{A%d:%d:%d:%d}$",(uint8)app_2,(uint8)app_3,app_4,(int)((Servo-SERVO_INIT)*0.16)); //打印到APP上面
//	else
	if(flag==0)
        printf("{B%d:%d:%d:%d:%d}$", Encoder_Left,Target_A,Encoder_Right,PWMA1,PWMB1);//打印到APP上面 显示波形
                                      //波形0      波形1       波形2     波形3      波形4

}

