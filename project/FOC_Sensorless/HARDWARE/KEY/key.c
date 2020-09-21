#include "key.h"
#include "delay.h"


//////////////////////////////////////////////////////////////////////////////////	 
//开发板
//按键输入 驱动代码		   
//////////////////////////////////////////////////////////////////////////////////	 

//按键初始化函数 

void KEY_Init(void)
{
    
    GPIO_InitTypeDef GPIO_InitStructure;
    
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA|RCC_AHBPeriph_GPIOB|RCC_AHBPeriph_GPIOC,ENABLE);//使能GPIOA,GPIOB,GPIOC时钟
    
    
    GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_10|GPIO_Pin_11;//PB10,PB11,K3,K4
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU; //PB10,PB11设置成上拉输入 
    GPIO_Init(GPIOB, &GPIO_InitStructure);//初始化GPIOB.10,11
    
} 
//按键处理函数
//返回按键值
//mode:0,不支持连续按;1,支持连续按;
//返回值：
//0，没有任何按键按下
//KEY_ONOFF_PRES，KEY3按下
//KEY_BRAKE_PRES，KEY4按下
u8 KEY_Scan(u8 mode)
{	 
    static u8 key_up=1;//按键按松开标志
	  static u8 key_onoff=1;
	  static u8 key_brake=1;
	
    if(mode)key_up=1;  //when mode:0,不支持连续按  
    if(key_up&&(key_onoff==0||key_brake==0))
    {
        key_up=0;
       
        if(KEY_ONOFF==0)//if yes, means this key has pressed
				{
					key_onoff = KEY_ONOFF;//update this key's status
					key_brake = KEY_BRAKE;//update this key's status
					return KEY_ONOFF_PRES;
				}
        else if(KEY_BRAKE==0)//if yes, means this key has pressed
				{
					key_onoff = KEY_ONOFF;//update this key's status
					key_brake = KEY_BRAKE;//update this key's status
					return KEY_BRAKE_PRES;
				}
    }
		else if(KEY_ONOFF==1&&KEY_BRAKE==1)//if yes, means all keys have released
		{			
			key_up=1; //all keys have released
    }	
		
    key_onoff = KEY_ONOFF;
		key_brake = KEY_BRAKE; 		
    return 0;// 无按键按下
}






