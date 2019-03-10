#include "common.h"
#include "OLED_0_96.h"

/**********************  初始化  **************************/
int counttime = 0;
int flag = 1;
int flag2 = 0;
float realOutput = 0.0;
float stableOutput = 430;
char spring_oled1[20];
char spring_oled2[20];
char spring_oled3[20];
char spring_oled4[20];
char spring_oled5[20];
char spring_oled6[20];
char spring_oled7[20];
uint8 data_getstring[2];
uint16_t AD0 = 0, AD1 = 0 ,AD4 = 0, AD5 = 0;
uint16_t count;
float pre_offset = 0, offset=0;
uint8 status = 0;
/**********************  初始化  **************************/




/**********************  舵机PID  **************************/
typedef struct{
  float pGain;
  float iGain;
  float dGain;
  float ilimit;
  float istate;
  float perr;
  float errdat;
  float pidout;
}PIDP;

void pid_init(PIDP* pid){
  pid->istate = 0;
  pid->perr = 0;
  pid->errdat = 0;
  pid->pidout = 0;
}
void updat_pid(PIDP* pid){
  float ilimit1;
  ilimit1 = -1 * pid->ilimit;
  pid->istate += ((pid->iGain)*(pid->errdat));
  if((pid->istate)>(pid->ilimit))
    pid->istate = pid->ilimit;
  if((pid->istate)<(ilimit1))
    pid->istate = ilimit1;
  pid->pidout = pid->pGain * pid->errdat + pid->istate + ((pid->errdat)-(pid->perr)) * pid->dGain;
  pid->perr = pid->errdat;  
}

PIDP steer = {1.20, 0.04, 2.6,10,0,0,0,0};
void sd5_init(){
  FTM_PWM_Init(ftm0,ftm_ch0,A0,300,430);
  FTM_PWM_Duty(ftm0,ftm_ch0,430);
  pid_init(&steer);
}

void sd5_control(){
  updat_pid(&steer);
  if(((steer.pidout)<=125)&&((steer.pidout)>=-125)){
    realOutput = 430+(steer.pidout);
    //FTM_PWM_Duty(ftm0,ftm_ch0,450+(steer.pidout));
  }
  else{
    if((steer.pidout)<0){
      (steer.pidout)=-125;
       realOutput = 430+(steer.pidout);
      //FTM_PWM_Duty(ftm0,ftm_ch0,450+(steer.pidout));
    }
    else{
      (steer.pidout)=125;
       realOutput = 430+(steer.pidout);
      //FTM_PWM_Duty(ftm0,ftm_ch0,450+(steer.pidout));
    }
  }
  status = GPIO_Get(C5);
  if((AD0+AD1)>1550 && status){
    if(flag){
      GPIO_Set(I1,HIGH);
      FTM_PWM_Duty(ftm0,ftm_ch0,430);
      Soft_Delay_ms(410);
      FTM_PWM_Duty(ftm0,ftm_ch0,500);
      Soft_Delay_ms(400); 
    }
    if(!flag){
      GPIO_Set(I1,LOW);
      FTM_PWM_Duty(ftm0,ftm_ch0,500);
      Soft_Delay_ms(300);
      FTM_PWM_Duty(ftm0,ftm_ch0,430);
      Soft_Delay_ms(380); 
    }
    flag = 1 - flag;
  }else if((AD0+AD1)>1550 && !status){ 
    if(flag){
    GPIO_Set(I1,HIGH);
    FTM_PWM_Duty(ftm0,ftm_ch0,430);
    Soft_Delay_ms(455);
    FTM_PWM_Duty(ftm0,ftm_ch0,360);
    Soft_Delay_ms(400);
    }
    if(!flag){
      GPIO_Set(I1,LOW);
      FTM_PWM_Duty(ftm0,ftm_ch0,360);
      Soft_Delay_ms(250);
      FTM_PWM_Duty(ftm0,ftm_ch0,430);
      Soft_Delay_ms(320); 
    }
    flag = 1 - flag;
  }else if((steer.errdat)>16 || (steer.errdat)<-16){
    FTM_PWM_Duty(ftm0,ftm_ch0,realOutput);
  }else{
    FTM_PWM_Duty(ftm0,ftm_ch0,stableOutput);
  }
}
/**********************  舵机PID  **************************/



/**********************  控制代码  **************************/
void Control()
{
  //补充你的控制代码
  //FTM_PWM_Duty(ftm0,ftm_ch0,offset+500.0);
  
    //读取AD值
  AD0 = ADC_Read(ADC0_SE1);
  AD1 = ADC_Read(ADC0_SE3);
  
  //steer.errdat = (float) 1250.0*(AD1 - AD0)*1.0/(AD1+ AD0 +1);
  sd5_control();
  //FTM_PWM_Duty(ftm0,ftm_ch0,360);

  //FTM_PWM_Duty(ftm0,ftm_ch0,480);
  
  //读取AD值
  //AD0 = ADC_Read(ADC0_SE1);
  //AD1 = ADC_Read(ADC0_SE3);
  //AD4 = ADC_Read(ADC0_SE9);
  //AD5 = ADC_Read(ADC0_SE10);

  //舵机控制 建议使用位置式PD控制 请参考相应的手册
  //pre_offset = offset;
  //offset =(float) 1250.0*(AD0 - AD1)*1.0/(AD1+ AD0 +1);
  
  
  count=FTM_Pulse_Get(ftm1);//编码器数值读取
 // FTM_Count_Clean(ftm1);//编码器数值清零
  
  //电机控制，建议对电机与舵机的占空比限幅，电机0~125%，舵机根据安装情况设置
  FTM_PWM_Duty(ftm2,ftm_ch0,120);
  

}
/**********************  控制代码  **************************/




/**********************  中断操作  **************************/
void PIT_Interrupt(uint8 ch)
{
  
    steer.errdat = (float) 60.0*(AD1 - AD0)*1.0/(AD1+ AD0 +1);


  
  //翻转核心板灯观察工作状态       
    GPIO_Turn(G2);    GPIO_Turn(G3);
    Control();
    
 
}
/**********************  中断操作  **************************/



void OLED_Myshow(void)
{
  OLED_Clear(0x00);
  OLED_Show_String(8,16,0,0,1,spring_oled1,0);
 OLED_Show_String(8,16,0,16,1,spring_oled2,0);
  OLED_Refresh_Gram();
}

int main(void)
{
  
  while((1280*ex_clk_khz) != (256*ics_clk_khz));//确保时钟配置无误
  OLED_Init();
  Soft_Delay_ms(125);
  OLED_Clear(0x00);
  
  //核心板3色RGB LED初始化,高电平灭,低电平亮
  GPIO_Init(C5,GPI,LOW);
  GPIO_Init(I1,GPO,LOW);
  GPIO_Init(G1,GPO,LOW);
  GPIO_Turn(G1);
  GPIO_Init(G2,GPO,LOW);
  GPIO_Init(G3,GPO,HIGH);

  //FTM_PWM_Init(ftm0,ftm_ch0,A0,50,0);//舵机

 // FTM_Pulse_Init(ftm0,  FTM_PS_1, TCLK1);//编码器  此处注意，使用DEF车模，初始化两个编码器，删除舵机初始化与控制，使用AB车模，删除此句编码器初始化，否则无法正常工作。
  FTM_Pulse_Init(ftm1,  FTM_PS_1, TCLK2);

  FTM_PWM_Init(ftm2,ftm_ch0,F0,14000,0); //PWM1
  FTM_PWM_Init(ftm2,ftm_ch1,F1,14000,0); //PWM2
  FTM_PWM_Init(ftm2,ftm_ch4,G6,14000,0); //PWM3
  FTM_PWM_Init(ftm2,ftm_ch5,G7,14000,0); //PWM4
  

  //ADC
  ADC_Init(ADC0_SE1, ADC_12bit); //A1
  ADC_Init(ADC0_SE2, ADC_12bit); //A6
  ADC_Init(ADC0_SE3, ADC_12bit); //A7
  ADC_Init(ADC0_SE9, ADC_12bit); //C1
  ADC_Init(ADC0_SE10, ADC_12bit);//C2

//  GPIO_Init(I1,GPO,1);
  
  sd5_init();
  
  //PIT定时器
  PIT_Init1(pit0,1000);                  //单位us,0.1ms 
  PIT_SetCallback(PIT_Interrupt);	
  //Disable_Interrupt(INT_PIT_CH0);
  Enable_Interrupt(INT_PIT_CH0); 
  

  //初始化蓝牙
  UART_Init(uart2,9600,RXTX_D6D7);
  
  

  while(1)
  {
    
      
    sprintf(spring_oled1, "%4d",AD0);//
    sprintf(spring_oled2, "%4d",AD1);//
    sprintf(spring_oled3, "%4.0f",steer.errdat);//
    sprintf(spring_oled4, "%4.0f",realOutput);//
    sprintf(spring_oled5, "%4.0f", counttime);//
    sprintf(spring_oled6, "%4.0f",steer.istate);//
    sprintf(spring_oled7, "%4.0f",((steer.errdat)-(steer.perr)));//
    UART_Putstr(uart2,"L_");
    UART_Putstr(uart2,spring_oled1);
    UART_Putstr(uart2,"  ");
    UART_Putstr(uart2,"R_");
    UART_Putstr(uart2,spring_oled2 );
    UART_Putstr(uart2,"  ");
    UART_Putstr(uart2,"errd_");
    UART_Putstr(uart2,spring_oled3 );
    UART_Putstr(uart2,"  ");
    UART_Putstr(uart2,"realOutput_");
    UART_Putstr(uart2,spring_oled4 ); 
    UART_Putstr(uart2,"  ");
    UART_Putstr(uart2,"pre_");
    UART_Putstr(uart2,spring_oled5 ); 
    UART_Putstr(uart2,"  ");
    UART_Putstr(uart2,"istat_");
    UART_Putstr(uart2,spring_oled6 ); 
    UART_Putstr(uart2,"  ");
    UART_Putstr(uart2,"err-pre_");
    UART_Putstr(uart2,spring_oled7 ); 
    UART_Putchar(uart2,'\n');
    //OLED_Myshow();
    //OLED_Show_String(10,10,1,1,1,"aaa",1);
//FTM_PWM_Duty(ftm0,ftm_ch0,500);


  }
}
