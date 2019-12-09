#include "headfile.h"////////////////////////////////////////////////////1111111111111111111
#include  "calculate.h"
#include  "math.h"
#include "show.h"
uint16 AD_valu[5];
uint8 key1_flag = 0,key2_flag = 0,key3_flag = 0,key4_flag = 0,key5_flag = 0,key6_flag = 0,key7_flag = 0,key8_flag = 0;
float Direction,last_Direction;
float sensor_to_one[2];
double sum,sum1;
uint16 ad_result[8],ad_guihua[8];                      //adc的值
uint16 baud=0;
int Motor_A,Motor_B,Servo,Target_A,Target_B,Velocity;  //电机舵机控制相关 
int Encoder_Left,Encoder_Right,speed_ave;  
int PWMA1,PWMA2,PWMB1,PWMB2;
float Velocity_KP1=4,Velocity_KI1=50;	

uint16 Blue_V_decrease=0,Blue_V_increase=0,Blue_V_temp=10;
//float Velocity_KP1=21,Velocity_KI1=46;	       //速度控制PID参数	
//float Turn_P1=7,Turn_I1=2,Turn_D1=2;
//int16 Turn_P=35,Turn_D=500;
// float Turn_I=4,Turn_P=256,Turn_D=172;//230
int temp1,temp2;
int8 look,look1,look2;
uint16 t=0;
uint8 flag_ring=0;
float flag_cricle=4;
int16 ADmax;
int8 stop_flag=0,stop_target=0;
uint8 t1=30,t2=100;
uint16 stop_count=0; 
//float ff=1680;
uint32 k=0;
/*陈扬定义*///**************************************************************************
const uint16 Turnmax=590;const uint16 Turnmin=310;
float Turn1,Sensor,Sensor1,Turny;
float Turn=0;
int Cycle_Flag=0,BiZhan_right=1;
uint16 Turn_center=465;//456//460;

float Turn_P=8.7,Turn_D=200;//Turn_P=6.7,Turn_D=120;

uint16 max_v[4],min_v[4];
extern int BiZhan1;
uint16 Bluetooth_Velocity=230;
extern int cycle_direction_flag,cycle_f_flag,a_flag,cycle_ok_flag;
int cy_timer=0,cy_timer_flag=0;//圆环

int cycle_chose=0;

int aaa=0;
float Bias,Last_Bias;
int fq=30;//速度分区30
/*陈扬定义*/
//**************************************************************************
/*江定义定义*/



//**************************************************************************



//float Value_stage(float turn)
//  {
//	  static float ValueOld, ValueNew ,ValueFilter;
//		ValueOld = ValueFilter;
//		ValueNew = turn;		
//		if(ValueNew>=ValueOld)
//			ValueFilter = ((ValueNew-ValueOld)>stage?(ValueOld+stage):ValueNew);
//		else
//			ValueFilter = ((ValueNew-ValueOld)<-stage?(ValueOld-stage):ValueNew);
//                return ValueFilter;   
//  }	
/**************************************************************************
函数功能：小车运动数学模型
入口参数：速度和转角
返回  值：无
**************************************************************************/
void Kinematic_Analysis(float velocity,float turn)  //后期加速来慢慢调  目标速度60 60调稳立刻圆环 暂时不加速
{   
    float speed = velocity;   
    static int i =0;
    if((d_cj_flag == 0)) 
    {
              if(!(BiZhan_Flag||Cycle_Flag)&&(speed_fq_flag==1))
            { 
 /*大打角*/    if((Turn>=Turn_center+5*fq)||(Turn<=Turn_center-5*fq))
                         speed=speed-125;//125 105
                 //speed =speed-60;
                else if((Turn>=Turn_center+4*fq)||(Turn<=Turn_center-4*fq))
                         speed =speed-100;//160 -100 ; 
                    //speed =speed-90;
                 else if((Turn>=Turn_center+3*fq)||(Turn<=Turn_center-3*fq))
                         speed =speed-50;//190 -50 ;
                   // speed =speed-180;
                 else if((Turn>=Turn_center+2*fq)||(Turn<=Turn_center-2*fq))
                         speed =speed-20;//210 20
                 else if((Turn>=Turn_center+fq)||(Turn<=Turn_center-fq))
                         speed =speed-10;//220 10
  /*小打角*/     else //if((Turn<Turn_center+fq)||(Turn>Turn_center-fq))
                         speed =speed;
            }
             else
                speed=90;
              
              if(speed<90)
                speed=90;
    }
    
   /* else if(BiZhan_Flag==1)
          {
            speed=90;//
          }*/   
  else
  {
      speed=-60;
      i++;
      if(i>2000)//200
      {
          d_cj_flag=0;
          i = 0 ;
      }   
  }
    Target_A=(int)(speed); 
    Target_B=(int)(speed); 		 
}
/**************************************************************************
函数功能：转角位置式PID处理
入口参数：无
返回  值：无
**************************************************************************/
void Get_RC(void)
{
  
  
  if(Blue_V_decrease==1)
  {
      Bluetooth_Velocity-=Blue_V_temp;
      Blue_V_decrease=0;
      Blue_V_increase=1;
  }
  
  Velocity=Bluetooth_Velocity;	  //电磁巡线模式下的速度      
  Bias=100-Sensor;  //提取偏差        
      if((Bias<=4)&&(Bias>=-4))
      {
         Bias = 0;
      }
      
      else if((Bias<=-97))
      {
         Bias=-100;
      }
      else if((Bias>=97))
      {
         Bias=100;
      }

      Direction=Bias*Turn_P*0.1+(Bias-Last_Bias)*Turn_D*0.1;
      if((Bias==-100))
        {
        Direction=-80;
        }
      if((Bias==100))
        {
       Direction=80;
        }

  
  if((ad_result[0]<((min_v[0])+120))&&(ad_result[1]<(min_v[1]+120)))
  { 
  Direction=last_Direction;  
  }
  
  //出赛道
  if((ad_result[0]<((min_v[0])+200))&&(ad_result[1]<(min_v[1]+200)))
  {
     // if(ad_result[0]<(ad_result[1]-20))
      if(ad_result[1]>(ad_result[0]+5))  
      {
         Direction=last_Direction-1000;//向右打死
      }  
      else if(ad_result[0]>(ad_result[1]+65))
      {
        Direction=last_Direction;//+1000;  //向左打死
      }
  }
  
//直角弯  
  if((ad_result[2]>300)||(ad_result[3]>300))//500///*&&(Turn>(Turnmax-40)||Turn<(Turnmin+40))*/)
  {
      if((ad_result[0]>(max_v[0]-100))||(ad_result[1]>(max_v[1]-100)))//else*///200
      {
      }
    else  if((ad_result[2]>(ad_result[3]+350))&&(ad_result[3]<450))//zuo350
      {    
         Direction=last_Direction+1000;
      }
      else if((ad_result[3]>(ad_result[2]+350))&&(ad_result[2]<450))//you
      {        
       
        Direction=last_Direction-1000; 
        
      }
  }   
  //舵机调角****************
    if(d_cj_flag == 0)//短测距
    {
      
      Turn=-((int)Direction)+Turn_center;////8*8   
      
    }
    else
    {     
          Turn=((int)Direction)+Turn_center;            
    }
  if((Encoder_Right<=2)&&(BiZhan_Flag==1))
  {
    Turn=Turn_center; 
  }
  //舵机调角****************
  Last_Bias=Bias;   //上一次的偏差
  last_Direction=Direction;

  if(Turn>=Turnmax)//舵机限幅
  {
      Turn=Turnmax;
  }
  
  if(Turn<=Turnmin)
  {
      Turn=Turnmin;
  }

  ftm_pwm_duty(ftm0,ftm_ch0,(uint16)Turn);     
//Bias1=100-Sensor1;
//  look1=100-Sensor1;         
// Turn1=Bias1*(Turn_P+20)*0.01+(Bias1-Last_Bias1)*(Turn_D-100)*0.1;             
//Last_Bias1=Bias1; 
}
/**************************************************************************
闭环 闭环 闭环 函数功能：所有的控制代码都在这里面
         5ms定时中断由MPU6050的INT引脚触发
         严格保证采样和数据处理的时间同步				 
**************************************************************************/
void ADcontrol()  //闭环控制
{ 
  ad_result[0] = adc_ave(AD_CHANNEL7,ADC_12bit,10); //最左 剩
  ad_result[2] = adc_ave(AD_CHANNEL8,ADC_12bit,10); //最左 
   
  ad_result[1] = adc_ave(AD_CHANNEL2,ADC_12bit,10); //最右 剩     
  ad_result[3] = adc_ave(AD_CHANNEL1,ADC_12bit,10); //最右
  
  ad_result[4] = adc_ave(AD_CHANNEL4,ADC_12bit,10);
   if(ad_result[4]>aaa)
     aaa=ad_result[4];
   //采集中间电感的数据Get_Adc(2)Sensor_Middle
     //ad_value[0]=ad_result[5];
 ///*=========================冒泡排序升序==========================*///        
//     for(j=0;j<2;j++)//比较几次
//       {
//         for(k=0;k<2-j;k++)  //五个电感只需要交换4次
//         {
//           if(ad_value[k]>ad_value[k+1]) //前面的比后面的大则进行交换
//           {
//             temp=ad_value[k+1];
//             ad_value[k+1]=ad_value[k];
//             ad_value[k]= temp;
//           }
//         }
//       } 
         //ADmax=ad_value[2];
  
      for(int i=0;i<2;i++)  
    {
        sensor_to_one[i] = (float)(ad_result[i] - min_v[i])/(float)(max_v[i] - min_v[i]); 
        if(sensor_to_one[i]<=0.0)  sensor_to_one[i]=0.001;
        if(sensor_to_one[i]>1.0)  sensor_to_one[i]=1.0;        
        ad_guihua[i] = (int)(1000 * sensor_to_one[i]);     //AD[i]为归一化后的值  范围为0-1000
     } 
     
//      Encoder_Left=Read_Encoder(1);                                   //===读取编码器的值							 //为了保证M法测速的时间基准，首先读取编码器数据
//      Encoder_Right=Read_Encoder(2);     
      sum=ad_guihua[0]*1+ad_guihua[1]*199;          //归一化处理
      Sensor=(sum/(ad_guihua[0]+ad_guihua[1]));      //处理圆环电感归一化      
////      sum1=ad_result[2]*1+ad_result[3]*199;          //归一化处理
////      Sensor1=(sum1/(ad_result[2]+ad_result[3]));          //求偏差
      Get_RC();  
//      Kinematic_Analysis(Velocity,Turn);  //小车运动学分析  
//     if(Target_A<6) Target_A=6;
//     if(Target_B<6) Target_B=6;     
      //圆环
   if(cy_timer_flag == 1)
   {
      cy_timer++;
      if(cy_timer>1500)
      {
        cy_timer_flag = 0;
        cy_timer = 0;
        cj_flag=1;
      }
   }   
  //圆环后不测时间
  if(cy_timer_flag == 0)
   {     
     if(((ad_result[0]>max_v[0]-200)&&(ad_result[1]>max_v[1]-250)))
     {
       if((ad_result[4]>2000))
       {
         cj_flag=0;
         Cycle_Flag=1; 
         gpio_set(E1,1);
         cy_timer_flag=1;
       }
     } 
   }
} 
 
void PDcontrol()
{	 
      if(1) 
    {  
        Kinematic_Analysis(Velocity,Turn);   //个人认为取非得效果会好一点 
        if(Target_A<-50) Target_A=-50;
        if(Target_B<6) Target_B=6;
    }
      else 
    {  
       Velocity=0; //先刹车但方向仍然可以控制 
       Kinematic_Analysis(Velocity,Turn);
       if(Target_A<600) Target_A=0;
       if(Target_B<600) Target_B=0;
       //if(stop_count<100)  stop_count++;
    } 
      
      if(1)
    //  if(((ad_result[0]+ad_result[1])>600)&&stop_target==0)            //保护程序                								
    {	          
         Motor_A=Incremental_PI_B(Encoder_Right,Target_A); 
         //Motor_B=Incremental_PI_A(Encoder_Left,Target_B);                   //===速度闭环控制计算电机A最终PWM	                					
         Set_Pwm(Motor_A,Motor_B);//===速度闭环控制计算电机B最终PWM                                  	 //===赋值给PWM寄存器                 
         //test=Target_A-Encoder_Right;      
    }						
    else	
     {    
       Motor_A=0,Motor_B=0;                                             
       Set_Pwm(Motor_A,Motor_B);                                   	     //===赋值给PWM寄存器 
     }             
}        

// OLED显示屏        
void oledshow()
{      
                              OLED_Print_Num1(40,0,ad_result[4]);
      OLED_Print_Num1(0,0,ad_result[0]);                         OLED_Print_Num1(80,0,ad_result[1]);
                               
                              OLED_Print_Num1(40,1,(int)Turn);
      OLED_Print_Num1(0,1,ad_result[2]);                         OLED_Print_Num1(80,1,ad_result[3]);
         
                              OLED_Print_Num1(40,2,(int)cj_last);
      OLED_Print_Num1(0,2,Velocity); 
      
                        //      OLED_Print_Num1(40,4,(short int)(Turn_P*10));

                          //    OLED_Print_Num1(40,5,(short int)(Turn_D));  
                            OLED_Print_Num1(40,5,fq);
      OLED_Print_Num1(0,5,cycle_chose);       // OLED_Print_Num1(40,6,BiZhan1);
     
   
      OLED_Print_Num1(0,7,max_v[0]);                             OLED_Print_Num1(80,7,max_v[1]); 
      OLED_Print_Num1(0,6,min_v[0]);                             OLED_Print_Num1(80,6,min_v[1]);
      
    //  OLED_Print_Num1(0,5,ad_guihua[0]);
    //  OLED_Print_Num1(80,5,ad_guihua[1]);            
      //OLED_Print_Num1(60,4,Encoder_Right); 
//      OLED_Print_Num1(20,4,Encoder_Left);
//       OLED_Print_Num1(40,5,look2);
//       OLED_Print_Num1(80,5,Turn_P);//KI就是KD  
//      OLED_Print_Num1(40,6,t2);    
}
/**************************************************************************
函数功能：赋值给PWM寄存器
入口参数：左轮PWM、右轮PWM
返回  值：无
**************************************************************************/
void Set_Pwm(int motor_a,int motor_b)
{
    	if(motor_a<0)			
        {
          PWMA1=0,PWMA2=-motor_a;          
        }
	else 	            
        {
          PWMA2=0,PWMA1=motor_a;        
        }		
        
        if((Encoder_Right<10)&&(PWMA2>=300))
        {
          PWMA2 = 600;          
        }
        if((Encoder_Right<10)&&(PWMA1>=300))
        {
          PWMA1 = 600;
        }
        //停车保护
//        if((ad_result[0] - min_v[0]<120)&&(ad_result[1] - min_v[1]<120))
//        {
//        PWMA2 = 0;
//        PWMA1 = 0;
//        }
          //ftm_pwm_duty(ftm2,ftm_ch2,300);
          //ftm_pwm_duty(ftm2,ftm_ch3,0);
          ftm_pwm_duty(ftm2,ftm_ch2,PWMA1);
          ftm_pwm_duty(ftm2,ftm_ch3,PWMA2);
        
}
/**************************************************************************
函数功能：绝对值函数
入口参数：int
返回  值：unsigned int
**************************************************************************/
int myabss(int a)
{        
         uint16 temp;
	   if(a<0)  temp=-a;  
	  else temp=a;
	  return temp;
}


void ftminit()//编码器初始化
{
  //ftm的计数引脚可查看KEA128_port_cfg.h文件内的FTM0_COUNT_PIN与FTM1_COUNT_PIN的宏定义得知
    //一个ftm同一时间只能实现一种功能，如果用于测速就不能用于输出PWM
   // ftm_count_init(ftm0);   //对E0引脚输入的脉冲进行计数    E0接编码器LSB 
    ftm_count_init(ftm1);   //对E7引脚输入的脉冲进行计数    E7接编码器LSB
   pit_init_ms(pit1,5);   // 定时计数器0用于就算单位时间内的脉冲数
    enable_irq(PIT_CH1_IRQn);

    gpio_init(C5,GPI,0);    //用于判断方向                  C5接编码器DIR
    port_pull(C5);          //IO上拉
   
    gpio_init(H5,GPI,0);    //用于判断方向                  H5接编码器DIR
    port_pull(H5);          //IO上拉
}
/**************************************************************************77777777788888888888888
函数功能：单位时间读取编码器计数
入口参数：定时器
返回  值：速度值
**************************************************************************/
int Read_Encoder()
{
    int Encoder_TIM; 
    temp2 = ftm_count_get(ftm1);ftm_count_clean(ftm1)  ;
    if(gpio_get(H5))     Encoder_TIM = temp2;//速度取负
    else                 Encoder_TIM = -temp2;      //右轮速度                		
    return Encoder_TIM;
}
/**************************************************************************
函数功能：增量PD控制器
入口参数：编码器测量值，目标速度
返回  值：电机PWM
根据增量式离散PID公式 
pwm+=Kp[e（k）-e(k-1)]+Ki*e(k)+Kd[e(k)-2e(k-1)+e(k-2)]
e(k)代表本次偏差 
e(k-1)代表上一次的偏差  以此类推 
pwm代表增量输出
在我们的速度控制闭环系统里面，只使用PI控制
pwm+=Kp[e（k）-e(k-1)]+Ki*e(k)
**************************************************************************/
int Incremental_PI_A (int Encoder,int Target)
{ 	
	 static int Bias,Pwm,Last_bias;
	 Bias=Target-Encoder;                //计算偏差
	 Pwm+=(Velocity_KP1/10)*Bias+(Velocity_KI1/10)*(Bias-Last_bias);   //增量式PI控制器        
         if(Pwm>800) Pwm=800;
         if(Pwm<-280) Pwm=-280;
	 Last_bias=Bias;	                   //保存上一次偏差 
	 return Pwm;                         //增量输出
}
int Incremental_PI_B (int Encoder,int Target)//增量式
{ 	
	 static int Bias,Pwm,Last_bias;
	 Bias=Target-Encoder;                //计算偏差
	 Pwm+=(Velocity_KP1/10)*Bias+(Velocity_KI1/10)*(Bias-Last_bias);   //增量式PI控制器  
         if(Pwm>800) Pwm=800;	//PWM限幅
          if(Pwm<-300) Pwm=-300;	
	 Last_bias=Bias;	                   //保存上一次偏差 
	 return Pwm;                         //增量输出
}

void key_scan(void)
{
    static uint8 key1_last_status=1,key2_last_status=1,key3_last_status=1,key4_last_status=1,key5_last_status=1,key6_last_status=1,key7_last_status=1,key8_last_status=1;       
    uint8 key1_status, key2_status, key3_status,key4_status,key5_status,key6_status,key7_status,key8_status;
    
    key1_status = gpio_get(KEY1);//获取当前引脚状态
    key2_status = gpio_get(KEY2);
    key3_status = gpio_get(KEY3);
    key4_status = gpio_get(KEY4);
    key5_status = gpio_get(KEY5);
    key6_status = gpio_get(KEY6);
    key7_status = gpio_get(KEY7);
    key8_status = gpio_get(KEY8);
   
    if((0==key1_last_status) && (1==key1_status))   key1_flag = 1;//检测到上升沿，置位按键标志位
    if((0==key2_last_status) && (1==key2_status))   key2_flag = 1;//检测到上升沿，置位按键标志位
    if((0==key3_last_status) && (1==key3_status))   key3_flag = 1;//检测到上升沿，置位按键标志位
    if((0==key4_last_status) && (1==key4_status))   key4_flag = 1;//检测到上升沿，置位按键标志位
    if((0==key5_last_status) && (1==key5_status))   key5_flag = 1;//检测到上升沿，置位按键标志位
    if((0==key6_last_status) && (1==key6_status))   key6_flag = 1;//检测到上升沿，置位按键标志位
    if((0==key7_last_status) && (1==key7_status))   key7_flag = 1;//检测到上升沿，置位按键标志位
    if((0==key8_last_status) && (1==key8_status))   key8_flag = 1;//检测到上升沿，置位按键标志位
       
    key1_last_status = key1_status;//记录本次引脚状态
    key2_last_status = key2_status;
    key3_last_status = key3_status;
    key4_last_status = key4_status;
    key5_last_status = key5_status;//记录本次引脚状态
    key6_last_status = key6_status;
    key7_last_status = key7_status;
    key8_last_status = key8_status;
}
//人工识别苯环
void cycle_deal(void)
{
    if ( cycle_chose == 0 )//左
    {
      a_flag=0;
      //cycle_f_flag=0;
      cycle_direction_flag=1;//左
    }
    else if (cycle_chose == 1)//右
    {     
      a_flag=0;
    //  cycle_f_flag=0;
      cycle_direction_flag=0;  //右
    }
    else if (cycle_chose == 2)//先左后右
    {
      a_flag=1;
      cycle_f_flag=1;//先左
     // cycle_direction_flag=0;  
    }
    else if (cycle_chose == 3)//先右后左
    {
      a_flag=1;
      cycle_f_flag=0;//先右
     // cycle_direction_flag=0;  
    }
   
}

void key_deal(void)
{  
   
    if(key1_flag)       //按键是否有效
    {   
       cycle_chose++;
       if(cycle_chose>3)
         cycle_chose=0;   
       key1_flag = 0;  //按键已使用，清零标志位                    //右1
       //cycle_direction_flag=!cycle_direction_flag;
    }
    
    if(key2_flag==1)    //按键是否有效
    {
        key2_flag = 0;  //按键已使用，清零标志位
    }
   
   while(key3_flag==1)       //按键是否有效
    {
          key_scan();
            
            if(key2_flag==1)    //按键是否有效
          {
              key2_flag = 0;  //按键已使用，清零标志位
              Bluetooth_Velocity=Bluetooth_Velocity+10;
          }
          if(key1_flag)       //按键是否有效
          {     
              key1_flag = 0;  //按键已使用，清零标志位                    //右1
              Bluetooth_Velocity=Bluetooth_Velocity-10;        
          }
            
          if(key8_flag)
          {
            key3_flag = 0;  //按键已使用，清零标志位                         //右3
          }  
      oledshow();
    }
    
    while(key4_flag==1)       //按键是否有效
    {
          key_scan();
          if(key2_flag)
          {
            fq+=1;
            key2_flag = 0;  //按键已使用，清零标志位                         //右3
          }

          if(key1_flag)
          {
            fq-=1;
            key1_flag = 0;  //按键已使用，清零标志位                         //右3
          }
          if(key8_flag)
          {
            key4_flag = 0;  //按键已使用，清零标志位                         //右3
          } 
      oledshow();
    }
    
    while(key5_flag==1)       //D控制
    {
          key_scan();
          if(key8_flag)
          {
            key5_flag = 0;  //按键已使用，清零标志位                         //右3
          }  
      oledshow();
    }
    
   while(key6_flag==1)       //避障第一打角
    {
          key_scan();
            if(key2_flag==1)    //按键是否有效
          {
              key2_flag = 0;  //按键已使用，清零标志位
              BiZhan1+=1;
          }
          if(key1_flag)       //按键是否有效
          {     
              key1_flag = 0;  //按键已使用，清零标志位                    //右1
              BiZhan1-=1;         
          }
            
          if(key8_flag)
          {
            key6_flag = 0;  //按键已使用，清零标志位                         //右3
          }  
      oledshow();
    }
   
    if(key7_flag==1)    //按键是否有效
    {
       key7_flag = 0;  //按键已使用，清零标志位                       //左3
       BiZhan_right=!BiZhan_right;//反向避障
    }  
   
   if(key8_flag==1)    //按键是否有效
    {
       key8_flag = 0;  //按键已使用，清零标志位 //左4           
       
       cycle_ok_flag=0;//退出设置圆环模式
    }
}

//判断电感最大值最小值
void SC_black_Init(void)
{
  
   uint16  i,j;
   min_v[0] = 0;  max_v[0] = 100;
   min_v[1] = 0;  max_v[1] = 100;
   
   if(1)
   {
     min_v[2] = 0;  max_v[2] = 2;
     min_v[3] = 0;  max_v[3] = 2;
       delay(500);
       gpio_set(G0,0);
       delay(500);
       
       for(i=0;i<20;i++) 
       {
           AD_valu[0] =adc_ave(AD_CHANNEL7,ADC_12bit,10);  //PTC0  通道      
           AD_valu[1] =adc_ave(AD_CHANNEL2,ADC_12bit,10);  //PTC1  通道     
           AD_valu[2] =adc_ave(AD_CHANNEL8,ADC_12bit,10);  //PTC0  通道      
           AD_valu[3] =adc_ave(AD_CHANNEL1,ADC_12bit,10);  //PTC1  通道        
          // AD_valu[4] =adc_ave(AD_CHANNEL4,ADC_12bit,10);
//           AD_valu[2] = ad_ave(ADC1,AD15,ADC_10bit,6); //PTE25 通道     
//           AD_valu[3] = ad_ave(ADC1,AD14,ADC_10bit,6) * 2; //PTE24 通道 
//           AD_valu[4] = ad_ave(ADC0,AD15,ADC_10bit,6); //PTE24 通道 
//           AD_valu[5] = ad_ave(ADC0,AD13,ADC_10bit,6); //PTE24 通道 
           for(j=0;j<4;j++) 
           {     
               if(AD_valu[j] > max_v[j])  
               max_v[j] = AD_valu[j];
           }
            for(j=0;j<4;j++) 
           {     
               if(AD_valu[j] <min_v[j])  
               min_v[j] = AD_valu[j];
           }
//           max_v[4] = max_v[5] = (max_v[1]+max_v[2])/2;
//           max_v[4] = max_v[5] = (max_v[0]+max_v[3])/2;
           delay(3);           //延时	
           oledshow();
       }  
        gpio_set(G0,1);
 
   }
}  