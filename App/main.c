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
 //   ��ǰ�ٶ�        ǰһ�ٶ�           �ٶȵ�D        �ٶȵ�I          ���ֲ���              ���ֲ���
short aacx,aacy,aacz,gyrox,gyroy,gyroz;         //���ٶȺ������Ǵ�����ԭʼ����
int ADC_value[5]={0},ADC_value_Last[5]={0};                           //������
float L_Speed,R_Speed,CHB;                      //��������������������ͱ�
float CHB_Last=0;
float temp=0;
float var[6];                                   //��λ��ͨѶ����
uint8_t txt[30]="X:";                           //��λ��ͨѶ����

float Speed_Set=16;                            //�ٶ��趨ֵ    //14��16�ٶȿ���  //18����
float Ang_Set=114;                             //102 102.5 103 103.5 110��ǰ����
float Wandao = 0;                                     //������ٲ���
int a,b,Kadc,K,flag;


/******��λ��*********/
void LowerComputer(void)
{
     sprintf((char*)txt,"Ang_Acc:%04d",(int)Ang_Acc);
     LCD_P6x8Str(0,0,txt);
     sprintf((char*)txt,"Ang_IGyro:%04d",(int)Ang_IGyro);
     LCD_P6x8Str(0,1,txt);
     sprintf((char*)txt,"skt:%04d",(int)skt);
     LCD_P6x8Str(0,2,txt);
}

/******��λ��*********/
void UpperMonitor(void)
{
     var[0]=Ang_Acc;  //���ٶ���ýǶ�
     var[1]=skt;      //���ٶ���ýǶ�
     var[2]=Ang;      //�˲���Ƕ�
     var[3]=P;
     var[4]=D;
     var[5]=Speed_L;
     vcan_sendware((uint8_t *)var, sizeof(var));
}
void fast(void)
{
     Ang_Set+=0.5;     //�ٶ�С Ang�� Ang_ACC�� Speed_L��  ����               //0.5
     if(Ang_Set>116)          Ang_Set=116; 
}                                

void slow(void)
{
     Ang_Set-=0.8;   //0.05    //�ٶȴ� ���� ̧ͷ 
     if(Ang_Set<112.5)          Ang_Set=112.5;  //110.5
}
       
void GetAngle(void)
{
    MPU_Get_Accelerometer(&aacx,&aacy,&aacz);	//�õ����ٶȴ���������
    MPU_Get_Gyroscope(&gyrox,&gyroy,&gyroz);	//�õ�����������
    /*���ٶ���Ƕ�*/
    aacx=aacx-1122;
    aacz=aacz-1122;                 //�޸���װ��� ����ȥ������ʵ������ģ�
    Ang_Acc=atan2(aacx,aacz)*53;    //�����Ǻ���������Ƕ� �����ϵ�������Ŵ�һ��
    Ang_Acc=Ang_Acc-Ang_Set;       //��һ�������޸����ٶȼư�װ���õ���׼�ĽǶȣ�����λ���ң�
    /*���ٶ���Ƕ�*/
    gyrox=gyrox+54;                          //���20���õ�
    gyroy=gyroy-18;
    Ang_IGyro=Ang_IGyro+gyroy*(-0.00015);    //���ֵõ��ĽǶȣ����ϵ��ͨ����λ��������ֱ�����ֵõ��Ƕȴ��·�����ʵ�ĽǶȣ�
    skt=skt+gyroy*(-0.00015);  //skt�������ں��˲�ǰ��Ang_IGyroֵ
    Filter();
    LowerComputer();   //���͵���λ��
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
    
    P=Ang*250;//�ٶ�ֵ������Ϊ��������Ϊ��    //�Ƕȣ�ǰ��Ϊ��������Ϊ��////////////////////////////////////////////////////
    D=gyroy*0.1;
    Speed_L = P - D;   //Kp*Ang���ظ���) +  Kd*y������ٶȣ���������
    //LowerComputer();   //���͵���λ��
    //UpperMonitor();    //���͵���λ��    
    Go(Speed_L);
}

void Filter(void)
{
    Ang_Error=Ang_Acc-Ang_IGyro;
    Ang_IGyro=Ang_IGyro+Ang_Error*0.01;   //�̵��Ǹ�����Խ�����߸���ЧӦԽ�õ���Խ��ë��Խ��(��Ҫ��)
    Ang=Ang_IGyro;
}

void Go(float Speed)///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
{    
    if(Speed>0)                   //ǰ�� �����ת ��ǰ�� ת��+ֱ��
    {
       Speed=1000-Speed-20;       //-20�޸��������//��ƫ����ת    
       if((CHB>-10)&&(CHB<10))
       {
          Wandao = 0.5;
       }
       else if(((CHB>-30)&&(CHB<-10))||((CHB>10)&&(CHB<30)))               //���ϵ����
       {
          //slow();
          CHB-=8;
          Wandao = 2;
       }
       else if(((CHB>-60)&&(CHB<-30))||((CHB>30)&&(CHB<60)))               //���ϵ����
       {
          //slow();
          CHB-=8;
          Wandao = 2;
       }
       else                         //ֱ��ϵ��С
       {
          //slow();
          Wandao = 1.5;
       }
     //Wandao = 2;
     CHB=0.5*CHB;
     if(CHB>0)                   //ǰ�� �����ת ��ǰ�� ת��+ֱ��
     {
        L_Speed=Speed+Wandao*CHB;
        R_Speed=(int)Speed-Wandao*CHB;
     }
     else                       //��ƫ����ת
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
  if(Speed<0)                   //���� �����ת ����� ��ֱ��
  {
     Speed=1000+Speed-20;       //-20�޸��������
     ftm_pwm_duty(FTM2,FTM_CH0,1000);
     ftm_pwm_duty(FTM2,FTM_CH3,1000);     
     ftm_pwm_duty(FTM2,FTM_CH1,(uint32)(Speed));
     ftm_pwm_duty(FTM2,FTM_CH2,(uint32)(Speed));
  }
}

/**pit1�жϷ�������ÿ1ms����һ����̬****/
void pit1_irq(void)
{
     GetAngle();                 //������̬
     PIT_Flag_Clear(PIT1);       //���жϱ�־λ
}

/****pit0�жϷ�����������********/
void pit0_irq(void)
{
   LeftWheel_Count = ftm_pulse_get(FTM0);
   RightWheel_Count = ftm_pulse_get(FTM1);
   for(int k=0;k<15;k++)//ʮ���ȡƽ��
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
   Calculate_Speed();           //�����ٶ�
   ftm_pulse_clean(FTM0);       //�ٶ�����
   ftm_pulse_clean(FTM1);       //�ٶ�����
   PIT_Flag_Clear(PIT0);        //���жϱ�־λ
}

/***�����ٶȺ���***/  
void Calculate_Speed(void)
{
  
   Check_Speed=(LeftWheel_Count+RightWheel_Count)/2;    //ƽ�������ֵ����ݣ��õ�������ٶ�
   Check_Speed=(int)(Check_Speed-Speed_Set);                   //��ǰ�ٶ�-�趨�ٶȣ�������
   D_Check_Speed=Check_Speed-Check_Speed_Last;          //�ٶȵ� D
   Check_Speed_Last=Check_Speed;                        //���汾���ٶȣ��´������� D
   I_Check_Speed+=Check_Speed;                          //�ٶȵ�I
   if(I_Check_Speed>15000)I_Check_Speed=15000;
 }

void main(void)
{
   MPU_Init();                          //��ʼ��MPU6050
   LCD_Init();                          //��λ����ʼ��
   adc_init(ADC0_SE4);                  //B0
   adc_init(ADC0_SE5);                  //B1
   adc_init(ADC0_SE6);                  //B2
   adc_init(ADC0_SE12);                 //F4
   adc_init(ADC0_SE13);                 //F5   
   pit_init_ms(PIT0,5);                 //��ʱ20ms
   enable_irq(PIT_CH0_IRQn);            //ʹ��PIT_CH0�ж�   
   pit_init_ms(PIT1,1);                 //��ʱ1ms
   enable_irq(PIT_CH1_IRQn);            //ʹ��PIT_CH0�ж�
   uart_init(UART0,115200);             //��λ����ʼ��
   gpio_init(PTH5,GPI,0);               //��ʼ�� PTG5�ܽ�Ϊ���� (��һ·���ٷ���λ������)  
   gpio_init(PTH7,GPI,0);               //��ʼ�� PTG6           (�ڶ�·���ٷ���λ������)
   ftm_pulse_init(FTM0, FTM_PS_1, PTE7);//��ʼ��E7Ϊ�������岶��
   ftm_pulse_init(FTM1, FTM_PS_1, PTE0);//��ʼ��E0Ϊ�������岶��
   ftm_pwm_init(FTM2, FTM_CH0,14000,0); //C0
   ftm_pwm_init(FTM2, FTM_CH1,14000,0); //C1
   ftm_pwm_init(FTM2, FTM_CH2,14000,0); //C2
   ftm_pwm_init(FTM2, FTM_CH3,14000,0); //C3
   EnableInterrupts;                    //�����жϣ������õ��жϵģ�����Ҫ�ģ�    
            
   while(1)
   {
    
   }
}

