#include "common.h"
#include "include.h"

void LowerComputer(void);
void UpperMonitor(void);
void GetAngle(void);
void Filter(void);
void Calculate_Speed(void);
void Go(float Speed);
void fast(void);
void slow(void);

float Ang_IGyro=-1,skt=-1,Speed_L=0,Ang=0,Ang_Error=0,Ang_Acc,P=0,D=0;
int Check_Speed , Check_Speed_Last , D_Check_Speed , I_Check_Speed , LeftWheel_Count=0 , RightWheel_Count=0;
 //   当前速度        前一速度           速度的D        速度的I          左轮测速              右轮测速
short aacx,aacy,aacz,gyrox,gyroy,gyroz;         //加速度和陀螺仪传感器原始数据
int ADC_value[5]={0},ADC_value_Last[5]={0};                           //从左到右
float L_Speed,R_Speed,CHB;                      //左轮输出，右轮输出，差和比
float CHB_Last=0;
float temp=0;
float var[6];                                   //上位机通讯变量
uint8_t txt[30]="X:";                           //下位机通讯变量

float Speed_Set=16;                            //速度设定值    //14，16速度可以  //18出界
float Ang_Set=114;                             //102 102.5 103 103.5 110（前倒）
float Wandao = 0;                                     //弯道减速参数
int a,b,Kadc,K,flag;


/******下位机*********/
void LowerComputer(void)
{
     sprintf((char*)txt,"Ang_Acc:%04d",(int)Ang_Acc);
     LCD_P6x8Str(0,0,txt);
     sprintf((char*)txt,"Ang_IGyro:%04d",(int)Ang_IGyro);
     LCD_P6x8Str(0,1,txt);
     sprintf((char*)txt,"skt:%04d",(int)skt);
     LCD_P6x8Str(0,2,txt);
}

/******上位机*********/
void UpperMonitor(void)
{
     var[0]=Ang_Acc;  //加速度求得角度
     var[1]=skt;      //角速度求得角度
     var[2]=Ang;      //滤波后角度
     var[3]=P;
     var[4]=D;
     var[5]=Speed_L;
     vcan_sendware((uint8_t *)var, sizeof(var));
}
void fast(void)
{
     Ang_Set+=0.5;     //速度小 Ang↑ Ang_ACC↓ Speed_L↓  加速               //0.5
     if(Ang_Set>116)          Ang_Set=116; 
}                                

void slow(void)
{
     Ang_Set-=0.8;   //0.05    //速度大 减速 抬头 
     if(Ang_Set<112.5)          Ang_Set=112.5;  //110.5
}
       
void GetAngle(void)
{
    MPU_Get_Accelerometer(&aacx,&aacy,&aacz);	//得到加速度传感器数据
    MPU_Get_Gyroscope(&gyrox,&gyroy,&gyroz);	//得到陀螺仪数据
    /*加速度求角度*/
    aacx=aacx-1122;
    aacz=aacz-1122;                 //修复安装误差 （减去的数是实验出来的）
    Ang_Acc=atan2(aacx,aacz)*53;    //用三角函数计算出角度 乘这个系数让它放大一点
    Ang_Acc=Ang_Acc-Ang_Set;       //加一个常数修复加速度计安装误差，得到更准的角度（从上位机找）
    /*角速度求角度*/
    gyrox=gyrox+54;                          //这个20不用调
    gyroy=gyroy-18;
    Ang_IGyro=Ang_IGyro+gyroy*(-0.00015);    //积分得到的角度（这个系数通过上位机调出，直到积分得到角度大致符合真实的角度）
    skt=skt+gyroy*(-0.00015);  //skt用来看融合滤波前的Ang_IGyro值
    Filter();
    LowerComputer();   //发送到下位机
    //Ang_Set=111;

//    if((Check_Speed>-1.5)&&(Check_Speed<1.5))//////////////////////////////////////////////////////////////////////////////////////////////////
//        Ang_Set=111;
//    else if(Check_Speed>1.5)
//        Ang_Set=110;
//    else if(Check_Speed<-1.5)
//        Ang_Set=114;

//    if(flag==0)
//    {
       if(Check_Speed>0)slow();
       if(Check_Speed<0)fast();
//    }
    
//Ang_Set=113.5;
    
    P=Ang*250;//速度值：慢了为负，快了为正    //角度：前倾为正，后倾为负////////////////////////////////////////////////////
    D=gyroy*0.1;
    Speed_L = P - D;   //Kp*Ang（回复力) +  Kd*y方向角速度（阻尼力）
    //LowerComputer();   //发送到下位机
    //UpperMonitor();    //发送到上位机    
    Go(Speed_L);
}

void Filter(void)
{
    Ang_Error=Ang_Acc-Ang_IGyro;
    Ang_IGyro=Ang_IGyro+Ang_Error*0.01;   //绿的那个参数越大曲线跟随效应越好但是越大毛刺越多(需要调)
    Ang=Ang_IGyro;
}

void Go(float Speed)///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
{    
    if(Speed>0)                   //前倾 电机正转 向前走 转向+直立
    {
       Speed=1000-Speed-20;       //-20修复电机死区//右偏，左转    
       if((CHB>-10)&&(CHB<10))
       {
          Wandao = 0.5;
       }
       else if(((CHB>-30)&&(CHB<-10))||((CHB>10)&&(CHB<30)))               //弯道系数大
       {
          //slow();
          CHB-=8;
          Wandao = 2;
       }
       else if(((CHB>-60)&&(CHB<-30))||((CHB>30)&&(CHB<60)))               //弯道系数大
       {
          //slow();
          CHB-=8;
          Wandao = 2;
       }
       else                         //直道系数小
       {
          //slow();
          Wandao = 1.5;
       }
     //Wandao = 2;
     CHB=0.5*CHB;
     if(CHB>0)                   //前倾 电机正转 向前走 转向+直立
     {
        L_Speed=Speed+Wandao*CHB;
        R_Speed=(int)Speed-Wandao*CHB;
     }
     else                       //右偏，左转
     {
        L_Speed=(int)Speed+Wandao*CHB;
        R_Speed=Speed-Wandao*CHB;
     }
     if(L_Speed>=1000)L_Speed=999;
     if(R_Speed>=1000)R_Speed=999;
     ftm_pwm_duty(FTM2,FTM_CH0,(uint32)L_Speed);
     ftm_pwm_duty(FTM2,FTM_CH3,(uint32)R_Speed);
     ftm_pwm_duty(FTM2,FTM_CH1,1000);
     ftm_pwm_duty(FTM2,FTM_CH2,1000);
  }
  if(Speed<0)                   //后倾 电机反转 向后走 仅直立
  {
     Speed=1000+Speed-20;       //-20修复电机死区
     ftm_pwm_duty(FTM2,FTM_CH0,1000);
     ftm_pwm_duty(FTM2,FTM_CH3,1000);     
     ftm_pwm_duty(FTM2,FTM_CH1,(uint32)(Speed));
     ftm_pwm_duty(FTM2,FTM_CH2,(uint32)(Speed));
  }
}

/**pit1中断服务函数：每1ms调整一次姿态****/
void pit1_irq(void)
{
     GetAngle();                 //调整姿态
     PIT_Flag_Clear(PIT1);       //清中断标志位
}

/****pit0中断服务函数：测速********/
void pit0_irq(void)
{
   LeftWheel_Count = ftm_pulse_get(FTM0);
   RightWheel_Count = ftm_pulse_get(FTM1);
   for(int k=0;k<15;k++)//十五次取平均
   {
     ADC_value[0]+=adc_once(ADC0_SE4,ADC_12bit);
     //ADC_value[1]+=adc_once(ADC0_SE5,ADC_12bit);
     //ADC_value[2]+=adc_once(ADC0_SE6,ADC_12bit);
     ADC_value[3]+=adc_once(ADC0_SE12,ADC_12bit);
     //ADC_value[4]+=adc_once(ADC0_SE13,ADC_12bit);
   }
   ADC_value[0]/=15;
   //ADC_value[1]/=15;
   //ADC_value[2]/=15;
   ADC_value[3]/=15;
   //ADC_value[4]/=15;
   
   
   
   
//   a=ADC_value[0]-ADC_value_Last[0];
//   if(a<0) a=-a;
//   b=ADC_value[3]-ADC_value_Last[3];
//   if(b<0) b=-b;
//   Kadc=a+b;
//   
   
   //if(Kadc<0) Kadc=1;
   //if(Kadc>1.8) Kadc=1.8;

   
   
//    if(Kadc>5)
//   {
//     K+=0.005;  
//     Ang_Set=109.5;
//     flag=1;
//   }
//   else flag=0;
//     K=100;
//   
//   if(Kadc>130) K=130;
//   
//   ADC_value_Last[0]=ADC_value[0];
//   ADC_value_Last[3]=ADC_value[3];
//   
   //CHB = K*(ADC_value[0]-ADC_value[3])/(ADC_value[0]+ADC_value[3]);///////////////////////////////////////////////////////////////////////
   CHB = 100*(ADC_value[0]-ADC_value[3])/(ADC_value[0]+ADC_value[3]);///////////////////////////////////////////////////////////////////////
   //CHB=0;
  
//   if(ADC_value[1]<450)
//  {
//    CHB=-40;
//    Ang_Set=110.5;
//  }
//  if(ADC_value[3]<450) 
//  {
//    CHB=40;
//    Ang_Set=110.5;
//  }
  
//  if((CHB>30)||(CHB<-30)) Ang_Set=111.5;
   Calculate_Speed();           //计算速度
   ftm_pulse_clean(FTM0);       //速度清零
   ftm_pulse_clean(FTM1);       //速度清零
   PIT_Flag_Clear(PIT0);        //清中断标志位
}

/***计算速度函数***/  
void Calculate_Speed(void)
{
  
   Check_Speed=(LeftWheel_Count+RightWheel_Count)/2;    //平均左右轮的数据，得到切向的速度
   Check_Speed=(int)(Check_Speed-Speed_Set);                   //当前速度-设定速度（反馈）
   D_Check_Speed=Check_Speed-Check_Speed_Last;          //速度的 D
   Check_Speed_Last=Check_Speed;                        //储存本次速度，下次用来算 D
   I_Check_Speed+=Check_Speed;                          //速度的I
   if(I_Check_Speed>15000)I_Check_Speed=15000;
 }

void main(void)
{
   MPU_Init();                          //初始化MPU6050
   LCD_Init();                          //下位机初始化
   adc_init(ADC0_SE4);                  //B0
   adc_init(ADC0_SE5);                  //B1
   adc_init(ADC0_SE6);                  //B2
   adc_init(ADC0_SE12);                 //F4
   adc_init(ADC0_SE13);                 //F5   
   pit_init_ms(PIT0,5);                 //定时20ms
   enable_irq(PIT_CH0_IRQn);            //使能PIT_CH0中断   
   pit_init_ms(PIT1,1);                 //定时1ms
   enable_irq(PIT_CH1_IRQn);            //使能PIT_CH0中断
   uart_init(UART0,115200);             //上位机初始化
   gpio_init(PTH5,GPI,0);               //初始化 PTG5管脚为输入 (第一路测速方向位，右轮)  
   gpio_init(PTH7,GPI,0);               //初始化 PTG6           (第二路测速方向位，左轮)
   ftm_pulse_init(FTM0, FTM_PS_1, PTE7);//初始化E7为输入脉冲捕获
   ftm_pulse_init(FTM1, FTM_PS_1, PTE0);//初始化E0为输入脉冲捕获
   ftm_pwm_init(FTM2, FTM_CH0,14000,0); //C0
   ftm_pwm_init(FTM2, FTM_CH1,14000,0); //C1
   ftm_pwm_init(FTM2, FTM_CH2,14000,0); //C2
   ftm_pwm_init(FTM2, FTM_CH3,14000,0); //C3
   EnableInterrupts;                    //开总中断（凡是用到中断的，都需要的）    
            
   while(1)
   {
    
   }
}

