#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/string.h>
#include <linux/leds.h>
#include <linux/slab.h>
#include <linux/cdev.h>
#include <linux/device.h>   /*class_create*/ 
#include <asm/uaccess.h>	/* copy_*_user */
#include <linux/fs.h>
#include <linux/i2c.h>
#include <linux/errno.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/miscdevice.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/gpio.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>




static int breathleds_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id);
static int breathleds_i2c_remove(struct i2c_client *client);

static struct i2c_client * breathleds_i2c_client = NULL;

static int led_ear_sta = 1;
static int led_chest_sta = 1;
static char led_ear_color =0;// 'B';
static char led_chest_color =0;// 'B';
static char led_ear_freq = 'L';
static char led_chest_freq = 'L';
typedef unsigned short U16;
typedef bool BOOL;
typedef unsigned char U8;

#define BREATHNAME     "breath_led"  //breathled_ear
static const struct i2c_device_id breathleds_i2c_id[] = {{"breath_led",0},{"breathled_chest",0},{}};   
//static struct i2c_board_info __initdata breathleds_i2c_hw1={ I2C_BOARD_INFO("breathled_chest", (0xb2>>1))};
//static struct i2c_board_info __initdata breathleds_i2c_hw2={ I2C_BOARD_INFO("breathled_ear", (0xb6>>1))};

static const struct of_device_id led_of_match[] = {
	{.compatible = "mediatek,breath_led"},
	
};

static struct i2c_driver breathleds_i2c_driver = {                       
    .probe = breathleds_i2c_probe,                                   
    .remove = breathleds_i2c_remove,    
     .id_table = breathleds_i2c_id, 
    .driver= 
    {
	.owner	= THIS_MODULE,
	.name	= BREATHNAME,
	 .of_match_table = led_of_match,
     },                 
                               
};
#define XUNHU_T818_BLINK
#define  AW2013_I2C_MAX_LOOP 3
#ifdef XUNHU_T818_BLINK
////modify by linhui 20150424
//#define FLASHLIGHT_I2C_BUSNUM 3
//static struct i2c_board_info __initdata aw2013_i2c_hw={ I2C_BOARD_INFO("aw2013", 0x45)};
//static struct i2c_client * aw2013_i2c_client = NULL;

static BOOL AW2013_i2c_write_reg_org(unsigned char reg,unsigned char data)
{
	BOOL ack=0;
	unsigned char ret;
	unsigned char wrbuf[2];

	wrbuf[0] = reg;
	wrbuf[1] = data;

	ret = i2c_master_send(breathleds_i2c_client, wrbuf, 2);
	if (ret != 2) {
		dev_err(&breathleds_i2c_client->dev,
		"%s: i2c_master_recv() failed, ret=%d\n",
		__func__, ret);
		ack = 1;
	}

	return ack;
}


BOOL AW2013_i2c_write_reg(unsigned char reg,unsigned char data)
{
	BOOL ack=0;
	unsigned char i;
	for (i=0; i<AW2013_I2C_MAX_LOOP; i++)
	{
		ack = AW2013_i2c_write_reg_org(reg,data);
		if (ack == 0) // ack success
			break;
		}
	return ack;
}


int s4aw2013_WriteReg(u16 addr, char * buf ,int count)
{
	unsigned char ret;
	 printk("************   8888iddr=0x%02x  \n",breathleds_i2c_client->addr);
	ret = i2c_master_send(breathleds_i2c_client, buf, count);
	
	if (ret != count) 
	{

		   printk("**********************   8888   ning s4aw2013_WriteReg failed  %s \r\n", __func__);

	
		dev_err(&breathleds_i2c_client->dev,"%s: i2c_master_recv() failed, ret=%d\n",
			__func__, ret);
	}
	 printk("*********5555***   8888  \n");
	return ret;
}
///////////
#define Imax          0x02   //LED Imax,0x00=omA,0x01=5mA,0x02=10mA,0x03=15mA,
#define Rise_time   0x02   //LED rise time,0x00=0.13s,0x01=0.26s,0x02=0.52s,0x03=1.04s,0x04=2.08s,0x05=4.16s,0x06=8.32s,0x07=16.64s
#define Hold_time   0x01   //LED max light time light 0x00=0.13s,0x01=0.26s,0x02=0.52s,0x03=1.04s,0x04=2.08s,0x05=4.16s
#define Fall_time     0x02   //LED fall time,0x00=0.13s,0x01=0.26s,0x02=0.52s,0x03=1.04s,0x04=2.08s,0x05=4.16s,0x06=8.32s,0x07=16.64s
#define Off_time      0x01   //LED off time ,0x00=0.13s,0x01=0.26s,0x02=0.52s,0x03=1.04s,0x04=2.08s,0x05=4.16s,0x06=8.32s,0x07=16.64s
#define Delay_time   0x00   //LED Delay time ,0x00=0s,0x01=0.13s,0x02=0.26s,0x03=0.52s,0x04=1.04s,0x05=2.08s,0x06=4.16s,0x07=8.32s,0x08=16.64s
#define Period_Num  0x00   //LED breath period number,0x00=forever,0x01=1,0x02=2.....0x0f=15


unsigned char AW2013_i2c_read_reg(unsigned char regaddr) 
{
	unsigned char rdbuf[1], wrbuf[1], ret, i;

	wrbuf[0] = regaddr;

	for (i=0; i<AW2013_I2C_MAX_LOOP; i++) 
	{
		ret = i2c_master_send(breathleds_i2c_client, wrbuf, 1);
		if (ret == 1)
			break;
	}
	
	ret = i2c_master_recv(breathleds_i2c_client, rdbuf, 1);
	
	if (ret != 1)
	{
		   printk("**********************   5555   ning AW2013_i2c_read_reg failed  %s \r\n", __func__);
	
		dev_err(&breathleds_i2c_client->dev,"%s: i2c_master_recv() failed, ret=%d\n",
			__func__, ret);
	}
	
    	return rdbuf[0];
		
}
void aw2013_breath_all(int led0,int led1,int led2)  //led on=0x01   ledoff=0x00
{  
	breathleds_i2c_client->addr=0x45;

//printk("xunhu-----------led0=%d----led1=%d---------led2=%d----------------------\n",led0,led1,led2);
	//write_reg(0x00, 0x55);
	AW2013_i2c_write_reg(0x00, 0x55);	
  	  msleep(20);
    //liubiao open for xunhu
    // Reset
	AW2013_i2c_write_reg(0x01, 0x01);		// enable LED 		

	AW2013_i2c_write_reg(0x31, Imax|0x70);	//config mode, IMAX = 5mA	
	AW2013_i2c_write_reg(0x32, Imax|0x70);	//config mode, IMAX = 5mA	
	AW2013_i2c_write_reg(0x33, Imax|0x70);	//config mode, IMAX = 5mA	

	AW2013_i2c_write_reg(0x34, 0xff);	// LED0 level,
	AW2013_i2c_write_reg(0x35, 0xff);	// LED1 level,
	AW2013_i2c_write_reg(0x36, 0xff);	// LED2 level,
											
	AW2013_i2c_write_reg(0x37, Rise_time<<4 | Hold_time);	  //led0  				
	AW2013_i2c_write_reg(0x38, Fall_time<<4 | Off_time);	  //led0 
	AW2013_i2c_write_reg(0x39, Delay_time<<4| Period_Num);  //led0 

	AW2013_i2c_write_reg(0x3a, Rise_time<<4 | Hold_time);	  //led1						
	AW2013_i2c_write_reg(0x3b, Fall_time<<4 | Off_time);	  //led1 
	AW2013_i2c_write_reg(0x3c, Delay_time<<4| Period_Num);  //led1  

	AW2013_i2c_write_reg(0x3d, Rise_time<<4 | Hold_time);	  //led2 			
	AW2013_i2c_write_reg(0x3e, Fall_time<<4 | Off_time);    //led2 
	AW2013_i2c_write_reg(0x3f, Delay_time<<4| Period_Num);  //

	AW2013_i2c_write_reg(0x30, led2<<2|led1<<1|led0);	      //led on=0x01 ledoff=0x00	
}

void led_flash_aw2013_test( unsigned int id )
{
	char buf[2];
	
	//AW2013_i2c_read_reg(0x55);
	
	printk("hwctl led_flash_aw2013_test \n");	
	buf[0]=0x01;
	buf[1]=0x01;
	s4aw2013_WriteReg(0x45,buf,2);

	buf[0]=0x31;
	buf[1]=0x71;
	s4aw2013_WriteReg(0x45,buf,2);

	buf[0]=0x34;
	buf[1]=0xff;
	s4aw2013_WriteReg(0x45,buf,2);

	buf[0]=0x37;
	buf[1]=0x53;
	s4aw2013_WriteReg(0x45,buf,2);

	buf[0]=0x38;
	buf[1]=0x55;
	s4aw2013_WriteReg(0x45,buf,2);

	buf[0]=0x39;
	buf[1]=0x00;
	s4aw2013_WriteReg(0x45,buf,2);

	buf[0]=0x30;
	buf[1]=0x01;
	s4aw2013_WriteReg(0x45,buf,2);	
}

////modify by linhui 

void led_flash_aw2013_power_on(int id)/////
{
	char buf[2];
//	unsigned int id =0;////0 blue led ,1 red,2 green,

	buf[0]=0x01;
	buf[1]=0x01;
	s4aw2013_WriteReg(0x45,buf,2);

	buf[0]=0x31+id;
	buf[1]=0x73;
	s4aw2013_WriteReg(0x45,buf,2);

	buf[0]=0x34+id;
	buf[1]=0xff;//0xc8;
	s4aw2013_WriteReg(0x45,buf,2);

	buf[0]=0x37+id*3;
	buf[1]=0x34;
	s4aw2013_WriteReg(0x45,buf,2);

	buf[0]=0x38+id*3;
	buf[1]=0x35;
	s4aw2013_WriteReg(0x45,buf,2);

	buf[0]=0x39+id*3;
	buf[1]=0x03;
	s4aw2013_WriteReg(0x45,buf,2);

	buf[0]=0x30;
	buf[1]=1<<id;
	s4aw2013_WriteReg(0x45,buf,2);
}


void led_flash_aw2013_charging_full(int id)
{
	//unsigned int id =2;/////green led

	char buf[2];
	buf[0]=0x00;
	buf[1]=0x54;/////reset led module
	s4aw2013_WriteReg(0x45,buf,2);

	buf[0]=0x01;
	buf[1]=0x01;
	s4aw2013_WriteReg(0x45,buf,2);

	buf[0]=0x31+id;
	buf[1]=0x02;
	s4aw2013_WriteReg(0x45,buf,2);

	buf[0]=0x34+id;
	buf[1]=0xff;
	s4aw2013_WriteReg(0x45,buf,2);

	buf[0]=0x30;
	buf[1]=1<<id;
	s4aw2013_WriteReg(0x45,buf,2);
}

void led_off_aw2013(void)
{
	char buf[2];
	buf[0]=0x30;
	buf[1]=0x00;
	s4aw2013_WriteReg(0x45,buf,2);

	buf[0]=0x01;
	buf[1]=0x00;
	s4aw2013_WriteReg(0x45,buf,2);
	}
#endif


//#define AW9106_RESET_PIN		    GPIO124                 //AW9106的RESET脚
#define IIC_ADDRESS_WRITE			0xB0        //I
#define IIC_ADDRESS_READ			0xB1        //IIC的读地址，={1011，0，AD1，AD0，1}，AD0,AD1接低则为0xB1 ，接高则是0xB7
#define AW9016_I2C_MAX_LOOP 		5


/****************************************************************************
 * DEBUG MACROS
 ***************************************************************************/
static int debug_enable_led = 1;
#define LEDS_DRV_DEBUG(format, args...) do { \
	if (debug_enable_led) \
	{\
		printk(KERN_WARNING format, ##args);\
	} \
} while (0)


void AW9106_delay_1us(U16 wTime)   //延时1us的函尸
{
// 注意，根据各平台的主频调整为1us .
    U16 i;
#if defined(MT6223) 
    for (i=0; i<wTime*30; i++) ;  
#elif defined(MT6235)
    for (i=0; i<wTime*8; i++) ;
#else
    for (i=0; i<wTime*15; i++) ;
#endif
}


static BOOL AW9016_i2c_write_reg_org(unsigned char reg,unsigned char data)
{
	BOOL ack=0;
	unsigned char ret;
	unsigned char wrbuf[2];

	wrbuf[0] = reg;
	wrbuf[1] = data;

	ret = i2c_master_send(breathleds_i2c_client, wrbuf, 2);
	if (ret != 2) {
		
		printk("%s: AW9016 i2c_master_recv() failed, ret=%d\n",
		__func__, ret);
		ack = 1;
	}

	return ack;
}

BOOL AW9016_i2c_write_reg(unsigned char reg,unsigned char data)
{
	BOOL ack=0;
	unsigned char i;
	for (i=0; i<AW9016_I2C_MAX_LOOP; i++)
	{
		ack = AW9016_i2c_write_reg_org(reg,data);
		if (ack == 0) // ack success
			break;
		}
	return ack;
}

int AW9016_WriteRegs(u16 addr, char * buf ,int count)
{
	unsigned char ret;
	 printk("************   8888iddr=0x%02x  \n",breathleds_i2c_client->addr);
	ret = i2c_master_send(breathleds_i2c_client, buf, count);
	
	if (ret != count) 
	{

		   printk("**********************   8888   ning s4aw2013_WriteReg failed  %s \r\n", __func__);

	
		dev_err(&breathleds_i2c_client->dev,"%s: i2c_master_recv() failed, ret=%d\n",
			__func__, ret);
	}
	 printk("*********5555***   8888  \n");
	return ret;
}

unsigned char AW9016_i2c_read_reg(unsigned char regaddr) 
{
	unsigned char rdbuf[1], wrbuf[1], ret, i;

	wrbuf[0] = regaddr;

	for (i=0; i<AW9016_I2C_MAX_LOOP; i++) 
	{
		ret = i2c_master_send(breathleds_i2c_client, wrbuf, 1);
		if (ret == 1)
			break;
	}
	
	ret = i2c_master_recv(breathleds_i2c_client, rdbuf, 1);
	
	if (ret != 1)
	{
		   printk("**********************   5555   ning AW2013_i2c_read_reg failed  %s \r\n", __func__);
	
		dev_err(&breathleds_i2c_client->dev,"%s: i2c_master_recv() failed, ret=%d\n",
			__func__, ret);
	}
	
    	return rdbuf[0];
		
}

static BOOL AW9106_i2c_write_reg(unsigned char reg,unsigned char data)
{
	return AW9016_i2c_write_reg(reg, data);

}

U8 AW9106_i2c_read_reg(U8 regaddr) 
{
	return AW9016_i2c_read_reg(regaddr);

}
void AW9106_SoftReset(void)
{		
	AW9106_i2c_write_reg(0x7f,0x00); //软复位，清除所有寄存器值，灭掉所有灯
	AW9106_delay_1us(30); 
}

#if 0

void AW9106_Hw_reset(void)
{   
	mt_set_gpio_mode(AW9106_RESET_PIN | 0x80000000, GPIO_MODE_00);
	mt_set_gpio_dir(AW9106_RESET_PIN | 0x80000000,GPIO_DIR_OUT);
	mt_set_gpio_out(AW9106_RESET_PIN | 0x80000000,0);
	AW9106_delay_1us(1000); //复位信号为低电平的持续时间必须至少20us才能正常复位  
	mt_set_gpio_out(AW9106_RESET_PIN | 0x80000000,1);
	AW9106_delay_1us(300); 
}



void AW9106_POWER_ON(void)
{    // AW9106 POWER-ON， 请客户不要改动此函数
     // 在AW9106_init()中，先进行POWER-ON，再进行客户自身的相关操作
	kal_uint16 count=0;
	//AW9106_i2c_initial();
	AW9106_Hw_reset();
	
}
#endif

/*
void AW9106_OnOff(BOOL OnOff)  //AW9106硬件使能。低电平复位，高电平正常工作。
{   
	//uem_stop_timer(AW9106_timer_ID);   //关闭timer
	//GPTI_StopItem(AW9106_timer_ID);
	//StopTimer(AW9106_timer_ID);
	 Paoma_cnt=0;
   	Breath_cnt=0;
	GPIO_ModeSetup(AW9106_RESET_PIN, 0);
	GPIO_InitIO(1, AW9106_RESET_PIN);     
	AW9106_i2c_initial();  
    AW9106_i2c_write_reg(0x7f,0x00); //软复位，可以不要	  
	GPIO_WriteIO(0, AW9106_RESET_PIN);
	AW9106_delay_1us(200); //复位信号为低电平的持续时间必须至少20us才能正常复位	
	if (OnOff ==1)
	{  
		GPIO_WriteIO(1, AW9106_RESET_PIN); 
	}
	AW9106_delay_1us(30); 
}
*/

//-------------------------------------------------------------------------------------------
//函数名: AW9106_AllOn
//调用此函数，则10路灯全亮，每路的电流分别由软件独立控制。
//-------------------------------------------------------------------------------------------
void AW9106_AllOn(void)
{
	printk("-------------------------AW9106_AllOn  Entry ------------------------- \r\n");

	AW9106_SoftReset();
	AW9106_i2c_write_reg(0x12,0x00);   //OUT配置为呼吸灯模式
	AW9106_i2c_write_reg(0x13,0x00);   //OUT配置为呼吸灯模式
	
	AW9106_i2c_write_reg(0x20,0x3f);//OUT0口调光，调光等级为0-255。OUT0~OUT5的调光指令依次为0x20~0x25. 写0关闭
	AW9106_i2c_write_reg(0x21,0x3f);
	AW9106_i2c_write_reg(0x22,0x3f);
	AW9106_i2c_write_reg(0x23,0x3f);
	AW9106_i2c_write_reg(0x24,0x3f);
	AW9106_i2c_write_reg(0x25,0x3f);

}

void AW9106_out0_fade(void)
{
	AW9106_i2c_write_reg(0x12,0x00);   //OUT4~5配置为呼吸灯模式这句如果之前设置过，可以不要
	AW9106_i2c_write_reg(0x13,0x00);   //OUT0~3配置为呼吸灯模式这句如果之前设置过，可以不要
	AW9106_i2c_write_reg(0x04,0x03); 	 //OUT4-OUT5自主呼吸BLINK模式使模?0为blink模式，1为FADE模式
	AW9106_i2c_write_reg(0x05,0x0e);   //OUT0-OUT3自主呼吸BLINK模式使能 0为blink模式，1为FADE模式?	AW9106_i2c_write_reg(0x15,0x09);   //淡进淡出时间设置这句如果之前设置过，可以不要

	AW9106_i2c_write_reg(0x03,0x00);   //先把03H清0
	AW9106_i2c_write_reg(0x03,0x01);   //从0写到1则产生淡进过程
	//AW9106_i2c_write_reg(0x03,0x00);   //从1写到0则产生淡出? 
}

//-------------------------------------------------------------------------------------------
//以下为AW9106实现几种效果的参考函数，用户请根据自己的效果需要来调整。
//-------------------------------------------------------------------------------------------

void AW9106_init(void)    //AW9106初始化，请客户在开机初始化时调用
{
	//AW9106_POWER_ON();   //AW9106 POWER-ON ，客户一般不要改动
	//先POWER-ON，以下再进行客户需要的操作
	//以下为客户初始化，由客户更改
	//AW9106_i2c_initial();
	//AW9106_Hw_reset();
	AW9106_SoftReset();	//GPTI_GetHandle(&AW9106_timer_ID);  ????daviekuo
	AW9106_i2c_write_reg(0x12,0x00);   //P0口配置为呼吸灯模式
	AW9106_i2c_write_reg(0x13,0x00);   //P1口配置为呼吸灯模式
	
	AW9106_i2c_write_reg(0x20,0x3f);//OUT0口调光，调光等级为0-255。OUT0~OUT5的调光指令依次为0x20~0x2f. 写0关闭
	AW9106_i2c_write_reg(0x21,0x3f);
	AW9106_i2c_write_reg(0x22,0x3f);
	AW9106_i2c_write_reg(0x23,0x3f);
	AW9106_i2c_write_reg(0x24,0x3f);
	AW9106_i2c_write_reg(0x25,0x3f);		
	//AW9106_test();  //读取AW9106内部寄存器的值，看与写进去的值是否一致，由此判断I2C接口是否通畅
}

//-------------------------------------------------------------------------------------------
//函数名: AW9106_init_pattern
//调用此函数，则6路灯全亮，为自主呼吸BLINK 模式
//-------------------------------------------------------------------------------------------
void AW9106_init_pattern(void)
{
 //AW9106基本的效果实现。6路自主呼吸
		printk(KERN_ERR"-------------------------AW9106_init_pattern  Entry ------------------------- \r\n");
	  AW9106_SoftReset();
	 if(1)//(strcmp(breathleds_i2c_client->name, "breathled_chest")==0)
	 {
	 //printk("daviekuo zzzzzzzzzz  0x%x\n", breathleds_i2c_client->addr);

	  AW9106_i2c_write_reg(0x12,0x00);	//OUT4~5配置为呼吸灯模式
	  AW9106_i2c_write_reg(0x13,0x00);	//OUT0~3配置为呼吸灯模式
	  AW9106_i2c_write_reg(0x14,0x3f);//自主呼吸使能 	 
 
	  AW9106_i2c_write_reg(0x04,0x00);	 //OUT4-OUT5自主呼吸BLINK模式使能		 out5/out2 R  out4/out1 G	out3/out0 B  
	  AW9106_i2c_write_reg(0x05,0x09);	 //OUT0-OUT3自主呼吸BLINK模式使能
 /*   
	  AW9106_i2c_write_reg(0x15,0x1b);	  //淡进淡出时间设置	 (512+1024)  +	(1024+512)
	  AW9106_i2c_write_reg(0x16,0x12);	  //全亮全暗时间设置 

	  AW9106_i2c_write_reg(0x15,0x02);	 //淡进淡出时间设置 	(2048 + 512)	+  (256 + 0)
	  AW9106_i2c_write_reg(0x16,0x20);	 //全亮全暗时间设置 

	  AW9106_i2c_write_reg(0x15,0x1b);	 //淡进淡出时间设置 	(512+1024)	+  (512+1024)
	  AW9106_i2c_write_reg(0x16,0x12);	 //全亮全暗时间设置 
 */ 	  
	  AW9106_i2c_write_reg(0x15,0x12);    //淡进淡出时间设置	  (256+512)  +	 (256+512)
	  AW9106_i2c_write_reg(0x16,0x20);    //全亮全暗时间设置 	  
	 
//	  AW9106_i2c_write_reg(0x11,0x82);	 //开始自主呼吸，并设置最大电流
	  AW9106_i2c_write_reg(0x11,0x00);   //开始自主呼吸，并设置最大电流		//由android laucher来设置初始状态
	  led_ear_freq = 'M';
	  led_chest_freq = 'M';
	 }
	 else
	 {
		 AW9106_SoftReset();
		 AW9106_i2c_write_reg(0x12,0x03);   //OUT4~5配置为呼吸灯模式
		 AW9106_i2c_write_reg(0x13,0x0f);   //OUT0~3配置为呼吸灯模式
	/*	 
		 AW9106_i2c_write_reg(0x04,0x03);   //OUT4-OUT5自主呼吸BLINK模式使能		out all
		 AW9106_i2c_write_reg(0x05,0x0f);   //OUT0-OUT3自主呼吸BLINK模式使能
	*/

		 AW9106_i2c_write_reg(0x04,0x01);   //OUT4-OUT5自主呼吸BLINK模式使能		out5/out2 R  out4/out1 G   out3/out0 B  
		 AW9106_i2c_write_reg(0x05,0x02);   //OUT0-OUT3自主呼吸BLINK模式使能
	/*	 
		 AW9106_i2c_write_reg(0x15,0x1b); 	 //淡进淡出时间设置		(512+1024)  +  (512+1024)
		 AW9106_i2c_write_reg(0x16,0x12);	 //全亮全暗时间设置 
		 AW9106_i2c_write_reg(0x15,0x12);	  //淡进淡出时间设置	 (256+512)  +	(256+512)
		 AW9106_i2c_write_reg(0x16,0x09);	  //全亮全暗时间设置 
	*/

		 AW9106_i2c_write_reg(0x02,0x1);	
		 AW9106_i2c_write_reg(0x03,0x2);	

		 //AW9106_i2c_write_reg(0x14,0x3f);//自主呼吸使能 

//		 AW9106_i2c_write_reg(0x11,0x82);	//开始自主呼吸，并设置最大电流
		 AW9106_i2c_write_reg(0x11,0x02);	//开始自主呼吸，并设置最大电流	   //由android laucher来设置初始状态
	 }
	 //如果不采用自主呼吸模式，则采用下列方式调光
	//AW9106_i2c_write_reg(0x14,0x00);//关闭自主呼吸使能
	//AW9106_i2c_write_reg(0x20,0x3f);//OUT0口调光，调光等级为0-255。OUT0~OUT5的调光指令依次为0x20~0x25. 写0关闭
	//AW9106_i2c_write_reg(0x21,0x3f);
	//AW9106_i2c_write_reg(0x22,0x3f);
	//AW9106_i2c_write_reg(0x23,0x3f);
	//AW9106_i2c_write_reg(0x24,0x3f);
	//AW9106_i2c_write_reg(0x25,0x3f);
		AW9106_delay_1us(60); 
	 //AW9106_test();  //读取AW9106内部寄存器的值，看与写进去的值是否一致，由此判断I2C接口是否通畅
}


void set_led_const_ctl(int led,int color)
{
	switch(color)
		{
			case 'R':
			case 'r':						//Extre speed
				AW9106_SoftReset();								
				AW9106_i2c_write_reg(0x14,0x3f);//自主呼吸使能		
				AW9106_i2c_write_reg(0x04,~0x02);   //OUT4-OUT5自主呼吸BLINK模式使能 	out5/out2 R  out4/out1 G   out3/out0 B	
				AW9106_i2c_write_reg(0x05,~0x04);   //OUT0-OUT3自主呼吸BLINK模式使能							
				AW9106_i2c_write_reg(0x02,0x00);
				AW9106_i2c_write_reg(0x03,0x00);
				break;
			case 'G':
			case 'g':						//Extre speed
				 AW9106_SoftReset();								
				 AW9106_i2c_write_reg(0x14,0x3f);//自主呼吸使能
				 AW9106_i2c_write_reg(0x04,~0x01);	//OUT4-OUT5自主呼吸BLINK模式使能		out5/out2 R  out4/out1 G   out3/out0 B	
				 AW9106_i2c_write_reg(0x05,~0x02);	//OUT0-OUT3自主呼吸BLINK模式使能				
				AW9106_i2c_write_reg(0x02,0x00);
				AW9106_i2c_write_reg(0x03,0x00);
				break;
			case 'B':
			case 'b':
	
			        AW9106_SoftReset(); 							
			        AW9106_i2c_write_reg(0x14,0x3f);//自主呼吸使能		
			         AW9106_i2c_write_reg(0x04,~0x00);	//OUT4-OUT5自主呼吸BLINK模式使能		out5/out2 R  out4/out1 G   out3/out0 B	
				AW9106_i2c_write_reg(0x05,~0x09);	//OUT0-OUT3自主呼吸BLINK模式使能	
				AW9106_i2c_write_reg(0x02,0x00);
				AW9106_i2c_write_reg(0x03,0x00);
				break;
		}
}
#if 1
static ssize_t frequency_store(struct device *dev, struct device_attribute *attr,  const char *buf, size_t size)
{	
	printk("daviekuo, store_frequenct in data %s\n",buf);
	//breathleds_i2c_client->addr = 0xb6>>1;
	if (buf != NULL && size != 0)
	{		
		char led = *(buf+0);
		char freq = *(buf+1);
		
		if(led == 'C' || led == 'c')
		{
			breathleds_i2c_client->addr = 0xb2>>1;
			led_chest_freq = freq;
		}	
		else if(led == 'E' || led == 'e')
		{
			breathleds_i2c_client->addr = 0xb6>>1; //AD0 AD1 =11  
			led_ear_freq = freq;
		}	
		switch(freq)
		{

			case 'L':
			case 'l':						//Low speed
				AW9106_i2c_write_reg(0x15,0x12);   //淡进淡出时间设置	  (2048 + 512)	  +  (0 + 512)
				AW9106_i2c_write_reg(0x16,0x20);   //全亮全暗时间设置 
				break;		

			case 'M':
			case 'm':						//Middle speed
				AW9106_i2c_write_reg(0x15,0x12);   //淡进淡出时间设置	  (256 + 512)	  +  (256 + 512)
				AW9106_i2c_write_reg(0x16,0x09);   //全亮全暗时间设置 
				break;		
				
			case 'H':
			case 'h':						//High speed
				AW9106_i2c_write_reg(0x15,0x09);   //淡进淡出时间设置	  (256 + 256)	  +  (0 + 256)
				AW9106_i2c_write_reg(0x16,0x08);   //全亮全暗时间设置 
				break;

			case 'C':
			case 'c':
				set_led_const_ctl(1,led_chest_color);
				set_led_const_ctl(1,led_ear_color);
			break;
			case 'E':
			case 'e':						//Extre speed
				break;
			default:
				return -2;					
		}
	}
	else
		return -1;
	return 1;//sprintf(buf, "E%cC%c\n", led_ear_freq, led_chest_freq);
}

static ssize_t frequency_show(struct device *dev, struct device_attribute *attr, char *buf)
{
		char data[20] = {0};
		data[0] = 'E';
		data[1] = led_ear_freq;
		data[2] = 'C';
		data[3] = led_chest_freq;
	
	/*	
		if(led_ear_color == 0)
			data[1] = '0';
		else
			data[1] = '1';
		if(led_chest_sta == 0)
			data[3] = '0';
		else
			data[3] = '1';
	*/
		data[4] = '\0';
	
		return sprintf(buf, "%s\n", data);
}



static ssize_t ledcolor_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
				  
{	
	printk("daviekuo, store_color in data %s\n",buf);
	//breathleds_i2c_client->addr = 0xb6>>1;

	if (buf != NULL && size != 0) 
	{		
		char led = *(buf+0);
		char clr = *(buf+1);
		
		if(led == 'C' || led == 'c')
		{
			breathleds_i2c_client->addr = 0xb2>>1;
			led_chest_color = clr;
		}	
		else if(led == 'E' || led == 'e')
		{
			breathleds_i2c_client->addr = 0xb6>>1;//AD0 AD1 =11  
			led_ear_color = clr;
		}	
		switch(clr)
		{

			case 'R':
			case 'r':
				AW9106_SoftReset();				
				AW9106_i2c_write_reg(0x12,0x00);    
				AW9106_i2c_write_reg(0x13,0x00);    
				AW9106_i2c_write_reg(0x14,0x24);
				AW9106_i2c_write_reg(0x04,0x02);   //OUT2 OUT5
				AW9106_i2c_write_reg(0x05,0x04);    	

				AW9106_i2c_write_reg(0x11,0x03);
				AW9106_i2c_write_reg(0x11,0x83);
			//	printk("daviekuo: RRRRRRRR\n");
	 			break;
			case 'G':
			case 'g':
				 AW9106_SoftReset();				
				 AW9106_i2c_write_reg(0x12,0x00);	 
				 AW9106_i2c_write_reg(0x13,0x00);	 
				 AW9106_i2c_write_reg(0x14,0x12);
				 AW9106_i2c_write_reg(0x04,0x01);	//OUT1 OUT4
				 AW9106_i2c_write_reg(0x05,0x02);	  
				AW9106_i2c_write_reg(0x11,0x03);
				AW9106_i2c_write_reg(0x11,0x83);
				
			break;
			case 'B':
			case 'b':
	//			 AW9106_Hw_reset();
			     AW9106_SoftReset(); 			
				 AW9106_i2c_write_reg(0x12,0x00);	 
				 AW9106_i2c_write_reg(0x13,0x00);	 
			     AW9106_i2c_write_reg(0x14,0x09); 
				 AW9106_i2c_write_reg(0x04,0x00);	 //OUT0 OUT3
				 AW9106_i2c_write_reg(0x05,0x09); 
				AW9106_i2c_write_reg(0x11,0x03);
				AW9106_i2c_write_reg(0x11,0x83);		
				break;		
				
			case 'X':						//RG
			case 'x':
				AW9106_SoftReset(); 			
				AW9106_i2c_write_reg(0x12,0x00);   //OUT4~5配置为呼吸灯模式
				AW9106_i2c_write_reg(0x13,0x00);   //OUT0~3配置为呼吸灯模式
				AW9106_i2c_write_reg(0x14,0x3f);//自主呼吸使能		
				AW9106_i2c_write_reg(0x04,0x03);   //OUT4-OUT5自主呼吸BLINK模式使能 	out5/out2 R  out4/out1 G   out3/out0 B	
				AW9106_i2c_write_reg(0x05,0x06);   //OUT0-OUT3自主呼吸BLINK模式使能 	
				//AW9106_i2c_write_reg(0x15,0x12);	//淡进淡出时间设置	  (256+512)  +	 (256+512)
				//AW9106_i2c_write_reg(0x16,0x09);	//全亮全暗时间设置				
				AW9106_i2c_write_reg(0x11,0x80);
				//printk("daviekuo: RRRRRRRR\n");
				break;
			case 'Y':						//BR
			case 'y':
				 AW9106_SoftReset();				
				 AW9106_i2c_write_reg(0x12,0x00);	//OUT4~5配置为呼吸灯模式
				 AW9106_i2c_write_reg(0x13,0x00);	//OUT0~3配置为呼吸灯模式
				 AW9106_i2c_write_reg(0x14,0x3f);//自主呼吸使能
				 AW9106_i2c_write_reg(0x04,0x02);	//OUT4-OUT5自主呼吸BLINK模式使能		out5/out2 R  out4/out1 G   out3/out0 B	
				 AW9106_i2c_write_reg(0x05,0x0d);	//OUT0-OUT3自主呼吸BLINK模式使能
				 //AW9106_i2c_write_reg(0x15,0x12);	  //淡进淡出时间设置	 (256+512)	+	(256+512)
				 //AW9106_i2c_write_reg(0x16,0x09);	  //全亮全暗时间设置 
				 AW9106_i2c_write_reg(0x11,0x80);
				printk("daviekuo: GGGGGGGG\n");
				
				break;
			case 'Z':
			case 'z':						//BG
	//			 AW9106_Hw_reset();
				 AW9106_SoftReset();			
				 AW9106_i2c_write_reg(0x12,0x00);	//OUT4~5配置为呼吸灯模式
				 AW9106_i2c_write_reg(0x13,0x00);	//OUT0~3配置为呼吸灯模式
				 AW9106_i2c_write_reg(0x14,0x3f);//自主呼吸使能 	
				 AW9106_i2c_write_reg(0x04,0x01);	//OUT4-OUT5自主呼吸BLINK模式使能		out5/out2 R  out4/out1 G   out3/out0 B	
				 AW9106_i2c_write_reg(0x05,0x0b);	//OUT0-OUT3自主呼吸BLINK模式使能
				 //AW9106_i2c_write_reg(0x15,0x12);	 //淡进淡出时间设置    (256+512)  +   (256+512)
				 //AW9106_i2c_write_reg(0x16,0x09);	 //全亮全暗时间设置 			 
				 AW9106_i2c_write_reg(0x11,0x80);				
				break;		
			case 'A':
			case 'a':						//White
	//			 AW9106_Hw_reset();
				 AW9106_SoftReset();			
				 AW9106_i2c_write_reg(0x12,0x00);	//OUT4~5配置为呼吸灯模式
				 AW9106_i2c_write_reg(0x13,0x00);	//OUT0~3配置为呼吸灯模式
				 AW9106_i2c_write_reg(0x14,0x3f);//自主呼吸使能 	
				 AW9106_i2c_write_reg(0x04,0x03);	//OUT4-OUT5自主呼吸BLINK模式使能		out5/out2 R  out4/out1 G   out3/out0 B	
				 AW9106_i2c_write_reg(0x05,0x0f);	//OUT0-OUT3自主呼吸BLINK模式使能
				 //AW9106_i2c_write_reg(0x15,0x12);	 //淡进淡出时间设置    (256+512)  +   (256+512)
				 //AW9106_i2c_write_reg(0x16,0x09);	 //全亮全暗时间设置 			 
				 AW9106_i2c_write_reg(0x11,0x80);				
				break;
			default:
				return -2;					
		}
		if(led == 'E' || led == 'e')
		{
			if(led_ear_freq == 'L' || led_ear_freq == 'l')
			{
				AW9106_i2c_write_reg(0x15,0x12);	  //淡进淡出时间设置	  (256+512)  +	 (256+512)
				AW9106_i2c_write_reg(0x16,0x09);	  //全亮全暗时间设置				
			}
			else if(led_ear_freq == 'M' || led_ear_freq == 'm')
			{
				AW9106_i2c_write_reg(0x15,0x12);   //淡进淡出时间设置	  (2048 + 512)	  +  (256 + 0)
				AW9106_i2c_write_reg(0x16,0x20);   //全亮全暗时间设置 
			}
			else if(led_ear_freq == 'H' || led_ear_freq == 'h')
			{
				AW9106_i2c_write_reg(0x15,0x09);   //淡进淡出时间设置	  (256 + 256)	  +  (0 + 256)
				AW9106_i2c_write_reg(0x16,0x08);   //全亮全暗时间设置 
			}

		}
		else if(led == 'C' || led == 'c')
		{
			if(led_chest_freq == 'L' || led_chest_freq == 'l')
			{
				AW9106_i2c_write_reg(0x15,0x12);	  //淡进淡出时间设置	  (256+512)  +	 (256+512)
				AW9106_i2c_write_reg(0x16,0x09);	  //全亮全暗时间设置				
			}
			else if(led_chest_freq == 'M' || led_chest_freq == 'm')
			{
				AW9106_i2c_write_reg(0x15,0x12);   //淡进淡出时间设置	  (2048 + 512)	  +  (256 + 0)
				AW9106_i2c_write_reg(0x16,0x20);   //全亮全暗时间设置 
			}
			else if(led_chest_freq == 'H' || led_chest_freq == 'h')
			{
				AW9106_i2c_write_reg(0x15,0x09);   //淡进淡出时间设置	  (256 + 256)	  +  (0 + 256)
				AW9106_i2c_write_reg(0x16,0x08);   //全亮全暗时间设置 
			}

		}
	}
	else
		return -1;
	//return sprintf(buf, "E%cC%c\n", led_ear_color, led_chest_color);
	return 1;
}

static ssize_t ledcolor_show(struct device *dev, struct device_attribute *attr, char *buf)
{
		char data[20] = {0};
		data[0] = 'E';
		data[1] = led_ear_color;
		data[2] = 'C';
		data[3] = led_chest_color;
	/*	
		if(led_ear_color == 0)
			data[1] = '0';
		else
			data[1] = '1';
		if(led_chest_sta == 0)
			data[3] = '0';
		else
			data[3] = '1';
	*/
		data[4] = '\0';
	
		return sprintf(buf, "%s\n", data);

}



static ssize_t onoff_store(struct device *dev, struct device_attribute *attr,const char *buf, size_t size)				   
{
		if (buf !=NULL && size != 0) {
			
		char  led = *(buf+0);
		char  sta = *(buf+1);
		
		if(led == 'E' || led == 'e')
		{
			breathleds_i2c_client->addr = 0xb6>>1;
			switch(sta)
			{
				case '1':
				/*	
					AW9106_i2c_write_reg(0x02,0x3);    
					AW9106_i2c_write_reg(0x03,0x6);  
					led_ear_sta = 1;
				*/
					//AW9106_i2c_write_reg(0x014,0x3f);
					//AW9106_i2c_write_reg(0x012,0x0);	  
					//AW9106_i2c_write_reg(0x013,0x0);							
					//AW9106_i2c_write_reg(0x11,0x80);
					led_ear_sta = 1;
					break;
				case '0':
					AW9106_SoftReset();
					//AW9106_i2c_write_reg(0x012,0x3);	  
					//AW9106_i2c_write_reg(0x013,0xf);		
	/*
					AW9106_i2c_write_reg(0x02,0x3);    
					AW9106_i2c_write_reg(0x03,0xf);    
					led_ear_sta = 0;
	*/
					//AW9106_i2c_write_reg(0x02,0x3);    
					//AW9106_i2c_write_reg(0x03,0xf);    
					//AW9106_i2c_write_reg(0x014,0x0);		
					//AW9106_i2c_write_reg(0x11,0x02);	//test
	
					led_ear_sta = 0;
					break;
				default:
					return -2;					
			}
		}
		else if(led == 'C' || led == 'c')
		{
			breathleds_i2c_client->addr = 0xb2>>1;
			switch(sta)
			{
				case '1':	
//					AW9106_i2c_write_reg(0x04,0x00);   //OUT4-OUT5自主呼吸BLINK模式使能 	out5/out2 R  out4/out1 G   out3/out0 B	
//					AW9106_i2c_write_reg(0x05,0x09);   //OUT0-OUT3自主呼吸BLINK模式使能					
					//AW9106_i2c_write_reg(0x02,0x3);    
					//AW9106_i2c_write_reg(0x03,0xf); 
					//AW9106_i2c_write_reg(0x012,0x0);	  
					//AW9106_i2c_write_reg(0x013,0x0);		

					//AW9106_i2c_write_reg(0x014,0x3f);	
					//AW9106_i2c_write_reg(0x11,0x80);	 
					led_chest_sta = 1;
					break;
				case '0':
					AW9106_SoftReset();
					//AW9106_i2c_write_reg(0x012,0x3);	  
					//AW9106_i2c_write_reg(0x013,0xf);		
   					//AW9106_i2c_write_reg(0x02,0x3);    
					//AW9106_i2c_write_reg(0x03,0xf); 
					//AW9106_i2c_write_reg(0x014,0x0);		
					led_chest_sta = 0;
					break;

									
				default:
					return -2;					
			}
	
		}
		else if(led == 'A' || led == 'a')
		{
			
		}
	
		}
		else
			return -1;
		//return sprintf(buf, "E%dC%d\n", led_ear_sta, led_chest_sta);
		return 1;

}

static ssize_t onoff_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	char data[20] ;
	data[0] = 'E';
	data[2] = 'C';
	
	if(led_ear_sta == 0)
		data[1] = '0';
	else
		data[1] = '1';
	if(led_chest_sta == 0)
		data[3] = '0';
	else
		data[3] = '1';
	data[4] = '\0';
		
	return sprintf(buf, "%s\n", data);

}

 static DEVICE_ATTR(onoff, S_IWUSR | S_IWGRP | S_IRUGO, onoff_show, onoff_store);
  static DEVICE_ATTR(frequency, S_IWUSR | S_IWGRP | S_IRUGO, frequency_show, frequency_store);
 static DEVICE_ATTR(color, S_IWUSR | S_IWGRP | S_IRUGO, ledcolor_show, ledcolor_store);

#endif
/****************************************************************************
 * driver functions
 ***************************************************************************/
 #if 0
static int breathleds_probe(struct platform_device *pdev)
{
	printk(" breathleds_probe---------\n");

	//int ret;
	//AW9106_Hw_reset();
	
	//i2c_register_board_info(BREATHLEDS_I2C_BUSNUM, &breathleds_i2c_hw1, 1);
	//i2c_register_board_info(BREATHLEDS_I2C_BUSNUM, &breathleds_i2c_hw2, 1);
	
	return 0;//  i2c_add_driver(&breathleds_i2c_driver);

	
}


static int breathleds_remove(struct platform_device *pdev)
{
    return 0;
}

static int breathleds_suspend(struct platform_device *pdev, pm_message_t mesg)
{
    return 0;
}

static int breathleds_resume(struct platform_device *pdev)
{
    return 0;
}
#endif
#define DEV_NAME   "breathleds"
static dev_t b_dev ;
static struct cdev *breathleds_cdev;
static struct class *breathleds_class = NULL;
struct device *breathleds_dev = NULL;

static long breathleds_unlocked_ioctl(struct file *pfile, unsigned int cmd, unsigned long param)
{
	 return 0;	
}


static int breathleds_release (struct inode *node, struct file *file)
{
 	return 0;
}

static int breathleds_open (struct inode *inode, struct file *file)
{
	printk(KERN_INFO"/dev/breathleds open success!\n");
	return 0;
}

//static int breathleds_write(struct file *pfile, const char __user *from, size_t len, loff_t * offset)	
static ssize_t breathleds_write(struct file * pfile, const char __user * puser, size_t len, loff_t * poff)
{
	//int reg_data = 0;
	char data[20] ;//= {0};
	char led,sta;
	int ret;
	ret=copy_from_user(data, puser, len);

	 led = data[0];
	 sta = data[1];
	printk("daviekuo, breathleds_write  %s\n",data);

	if (len != 0) {
	if(led == 'E' || led == 'e')
	{
		breathleds_i2c_client->addr = 0xb6>>1;
		switch(sta)
		{
			case '1':
			/*	
				AW9106_i2c_write_reg(0x02,0x3);    
				AW9106_i2c_write_reg(0x03,0x6);  
				led_ear_sta = 1;
			*/
			//	AW9106_i2c_write_reg(0x04,0x01);   //OUT4-OUT5自主呼吸BLINK模式使能		out5/out2 R  out4/out1 G   out3/out0 B  
		 	//	AW9106_i2c_write_reg(0x05,0x02);   //OUT0-OUT3自主呼吸BLINK模式使能						
				AW9106_i2c_write_reg(0x012,0x0);      
				AW9106_i2c_write_reg(0x013,0x0);        
				//AW9106_i2c_write_reg(0x014,0x3f);
				AW9106_i2c_write_reg(0x11,0x80);
				led_ear_sta = 1;
	 			break;
			case '0':
				AW9106_i2c_write_reg(0x012,0x3);      
				AW9106_i2c_write_reg(0x013,0xf);        
				//AW9106_i2c_write_reg(0x02,0x3);    
				//AW9106_i2c_write_reg(0x03,0xf);    
				//AW9106_i2c_write_reg(0x014,0x0);        
				//AW9106_i2c_write_reg(0x11,0x02);

				led_ear_sta = 0;
/*
				AW9106_i2c_write_reg(0x02,0x3);    
				AW9106_i2c_write_reg(0x03,0xf);    
				led_ear_sta = 0;
*/
				break;
			default:
				return -2;					
		}
	}
	else if(led == 'C' || led == 'c')
	{
		breathleds_i2c_client->addr = 0xb2>>1;
		switch(sta)
		{
			case '1':						
	 		//	AW9106_i2c_write_reg(0x04,0x01);   //OUT4-OUT5自主呼吸BLINK模式使能		out5/out2 R  out4/out1 G   out3/out0 B  
	 		//	AW9106_i2c_write_reg(0x05,0x02);   //OUT0-OUT3自主呼吸BLINK模式使能				
				AW9106_i2c_write_reg(0x012,0x0);      
				AW9106_i2c_write_reg(0x013,0x0);        
				//AW9106_i2c_write_reg(0x014,0x3f);      
				AW9106_i2c_write_reg(0x11,0x80);	 
				led_chest_sta = 1;
	 			break;
			case '0':
				AW9106_i2c_write_reg(0x012,0x3);      
				AW9106_i2c_write_reg(0x013,0xf);        
				//AW9106_i2c_write_reg(0x02,0x3);    
				//AW9106_i2c_write_reg(0x03,0xf);    
				//AW9106_i2c_write_reg(0x014,0x0);        
				led_chest_sta = 0;
				break;			
			default:
				return -2;					
		}

	}
	else if(led == 'A' || led == 'a')
	{
		
	}

	}
	else
		return -1;
	
	return 0;
}

static ssize_t breathleds_read (struct file *pfile, char __user *to, size_t len, loff_t *offset)
{	
	char data[20],ret;
	data[0] = 'E';
	data[2] = 'C';
	
	if(led_ear_sta == 0)
		data[1] = '0';
	else
		data[1] = '1';
	if(led_chest_sta == 0)
		data[3] = '0';
	else
		data[3] = '1';

	ret=copy_to_user(to, data, 4);
	
	return 0;

}
static struct file_operations breathleds_fops = {
	//.owner = THIS_MODULE,
	.open = breathleds_open,
	.write = breathleds_write,
	.read = breathleds_read,
	.release = breathleds_release,
	.unlocked_ioctl = breathleds_unlocked_ioctl,
};

static int breathleds_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int ret;
	unsigned int led_en;
	struct device_node *node;
	
	   printk("[%s] breathleds_i2c_probe\n", id->name);
	
	   printk("[%x] breathleds_i2c_probe\n", client->addr);
	
		breathleds_i2c_client = client;
			breathleds_i2c_client->addr = 0xb6>>1;	   
		   node = of_find_compatible_node(NULL, NULL, "mediatek,adc_rst");
		   led_en = of_get_named_gpio(node, "led_v21_gpio26", 0);
		  gpio_request(led_en, "led_en");
		  gpio_direction_output(led_en, 1);
		   gpio_set_value(led_en, 1);

		AW9106_SoftReset();
		ret = alloc_chrdev_region(&b_dev, 0, 1, DEV_NAME);
		   if (ret< 0) {
		   printk("motor  alloc_chrdev_region failed, %d", ret);
		  return ret;
	  }
	   breathleds_cdev= cdev_alloc();
	   if (breathleds_cdev == NULL) {
			   printk("breathleds cdev_alloc failed");
			   ret = -ENOMEM;
			   goto EXIT;
		   }
	  cdev_init(breathleds_cdev, &breathleds_fops);
	   breathleds_cdev->owner = THIS_MODULE;
	   ret = cdev_add(breathleds_cdev, b_dev, 1);
	   if (ret < 0) {
			printk("Attatch file motor operation failed, %d", ret);
		   goto EXIT;
	   }
		breathleds_class = class_create(THIS_MODULE, "yyd");
				if (IS_ERR(breathleds_class)) {
					printk("Failed to create class(breathleds)!\n");
					return PTR_ERR(breathleds_class);
				}
				
		breathleds_dev = device_create(breathleds_class, NULL, b_dev, NULL,DEV_NAME);
		if (IS_ERR(breathleds_dev))
			printk("Failed to create breathleds device\n");

		if (device_create_file(breathleds_dev, &dev_attr_onoff) < 0)
				printk("Failed to create device file(%s)!\n",
					  dev_attr_onoff.attr.name);   
		if (device_create_file(breathleds_dev, &dev_attr_color) < 0)
				printk("Failed to create device file(%s)!\n",
					  dev_attr_color.attr.name);
		if (device_create_file(breathleds_dev, &dev_attr_frequency) < 0)
				printk("Failed to create device file(%s)!\n",
					  dev_attr_frequency.attr.name);
	return 0;
EXIT:

 	if(breathleds_cdev != NULL)
  	{
		cdev_del(breathleds_cdev);
		breathleds_cdev = NULL;
 	}

   //  unregister_chrdev_region(breathleds_dev, 1);
	 return -1;
/*****************************************************************************
*****************************************************************************/
}
static int breathleds_i2c_remove(struct i2c_client *client)
{
	return 0;
}
#if 0  

static struct platform_device breathleds_device = {
	.name = "breathleds",
	.id = -1
};


// platform structure
static struct platform_driver breathleds_driver = {
    .probe		= breathleds_probe,
    .remove	= breathleds_remove,
   // .shutdown = breathleds_shutdown,
    .suspend	= breathleds_suspend,
    .resume	= breathleds_resume,
    .driver		= {
        .name	= "breathleds",
        .owner	= THIS_MODULE,
    }
};
#endif
static int __init breathleds_init(void)
{
	int ret=0;

	LEDS_DRV_DEBUG("[LED]%s\n", __func__);
	printk("daviekuo %s 1111111\n", __func__);
	
#if 0
	ret = platform_device_register(&breathleds_device);
	if (ret)
		printk("breathleds_device_init:dev:E%d\n", ret);
	
	ret = platform_driver_register(&breathleds_driver);

	if (ret) {
		printk("breathleds_init:drv:E%d\n", ret);
		return ret;
	}
#endif	
	 i2c_add_driver(&breathleds_i2c_driver);
	return ret;
}

static void __exit breathleds_exit(void)
{
	//platform_driver_unregister(&breathleds_driver);
}

module_param(debug_enable_led, int, 0644);

late_initcall(breathleds_init);
module_exit(breathleds_exit);

MODULE_AUTHOR("MediaTek Inc.");
MODULE_DESCRIPTION("BreathLed driver for MediaTek");
MODULE_LICENSE("GPL");


