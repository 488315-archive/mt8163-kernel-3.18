#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/miscdevice.h>
#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/io.h>
#include <linux/cdev.h>
#include<linux/kthread.h>
#include <asm/uaccess.h>
#include <asm/uaccess.h>	/* copy_*_user */
#include <linux/semaphore.h>  
#include <linux/device.h>   /*class_create*/ 
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/wakelock.h>
#include <linux/fs.h>
#include <linux/hrtimer.h>
#ifdef CONFIG_OF
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/regulator/consumer.h>
#include <linux/clk.h>
#include <linux/platform_device.h>
#include <linux/gpio.h>
#include <linux/pinctrl/consumer.h>
#include <linux/of_gpio.h>
#endif

#include <mt-plat/mt_boot.h>

/*----------------------------------------------------------------------------*/
#define DEV_NAME   "aw9523b"
static struct cdev *misc_cdev;
static dev_t misc_dev;
static struct class *misc_class = NULL;
static struct device *misc_device = NULL;
 static unsigned int IICSCL,IICSDA,IICSDA1;
// static struct hrtimer sstop_timer;
static struct task_struct * paoma_thread;

#define SDA_SET_OUT(sda)		gpio_direction_output(sda, 1)
#define SDA_SET_IN(sda)			gpio_direction_input(sda)
#define SDA_OUT_H(sda)		gpio_set_value(sda, 1)
#define SDA_OUT_L(sda)		gpio_set_value(sda, 0)
#define SDA_GET_IN(sda)		gpio_get_value(sda)
#define SCL_OUT_H(scl)		gpio_set_value(scl, 1)
#define SCL_OUT_L(scl)		gpio_set_value(scl, 0)
#define NOP2 	30000

#define AW_ADDR1 0xb6
#define AW_ADDR2 0xb6

// #define delay_nop_1us(wTime) { for (ggg=0; ggg<wTime*15; ggg++) ;}
 #define KAL_FALSE 0
#define KAL_TRUE 1

  typedef struct i2c_gpio{
	  u32 sda;
	  u32 scl;
	  u32 addr;
	  u32 speed;
	    u32 sda1;
  }I2C_GPIO_T;
static I2C_GPIO_T IIC_GPIO;
 static I2C_GPIO_T *IIC_DEV=&IIC_GPIO;
 static void find_gpio(I2C_GPIO_T *dev)
 {
	 struct device_node *node;
	//unsigned int led_en;
	
	 node = of_find_compatible_node(NULL, NULL, "mediatek,adc_rst");
	IICSCL  = of_get_named_gpio(node, "iic_slc_gpio52", 0);
	IICSDA   = of_get_named_gpio(node, "iic_sda_gpio29", 0);
	 IICSDA1  = of_get_named_gpio(node, "iic_sda1_gpio13", 0);
	 gpio_request(IICSDA, "IICSDA");
	 gpio_request(IICSCL, "IICSCL");
	 gpio_request(IICSDA1, "IICSDA1");
	printk("yyyyyyyy====%d,%d,%d\n",IICSCL,IICSDA,IICSDA1);
	 dev->scl=IICSCL;
	 dev->sda=IICSDA;
	 dev->sda1= IICSDA1;

		//  led_en = of_get_named_gpio(node, "led_v21_gpio26", 0);
		//  gpio_request(led_en, "led_en");
		//  gpio_direction_output(led_en, 1);
		//   gpio_set_value(led_en, 1);
 }

static void delay_nop_1us( int wTime)
 {
 	udelay(5);
 }

  static void i2c_init_gpio(I2C_GPIO_T *dev)
  {
	  gpio_direction_output(dev->scl, 1);
	  gpio_set_value(dev->scl, 1);
  
	  gpio_direction_output(dev->sda, 1);
	  gpio_set_value(dev->sda, 1);

	    gpio_direction_output(dev->sda1, 1);
	  gpio_set_value(dev->sda1, 1);
  
  }
 static  void i2c_start(I2C_GPIO_T *dev)
  {
	  u32 sda = dev->sda;
	  u32 scl = dev->scl;
  
	  SDA_OUT_H(sda);
	  SCL_OUT_H(scl);
	  delay_nop_1us(NOP2);
  
	  SDA_OUT_L(sda);
	  delay_nop_1us(NOP2);
	  //SDA_OUT_H(sda);
	  //delay_nop_1us(10);
  }

static void i2c_stop(I2C_GPIO_T *dev)
 {
	 u32 sda = dev->sda;
	 u32 scl = dev->scl;
 
	 SCL_OUT_L(scl);
	 delay_nop_1us(NOP2);
	 
	 SDA_OUT_L(sda);
	 delay_nop_1us(NOP2);
 
	 SCL_OUT_H(scl);
	 delay_nop_1us(NOP2);
	 
	 SDA_OUT_H(sda);
	 delay_nop_1us(NOP2);
 }
static  void IIC_NAck(I2C_GPIO_T *dev)
  {
		SCL_OUT_L(dev->scl);
	  SDA_SET_OUT(dev->sda);
	  SDA_OUT_H(dev->sda);
	  delay_nop_1us(NOP2);
	   SCL_OUT_H(dev->scl);
	  delay_nop_1us(NOP2);
	  SCL_OUT_L(dev->scl);
  }
  
 static void IIC_Ack(I2C_GPIO_T *dev)
  {
		SCL_OUT_L(dev->scl);
	  SDA_SET_OUT(dev->sda);
	  SDA_OUT_L(dev->sda);
	  delay_nop_1us(NOP2);
	   SCL_OUT_H(dev->scl);
	  delay_nop_1us(NOP2);
	  SCL_OUT_L(dev->scl);
  }

 static bool i2c_transfer_byte(I2C_GPIO_T *dev, u8 data)
  {
  
	  u8 tmp = data;
	  bool ack = KAL_FALSE;
	  u8 i;
	  int ret;
	  for(i = 0; i < 8; i++)
	  {
		  
		  SCL_OUT_L(dev->scl);
		  //delay_nop_1us(NOP);
		  if(tmp & 0x80)
		  {
			  SDA_OUT_H(dev->sda);			  
		  }
		  else
		  {
			  SDA_OUT_L(dev->sda);		  
		  }
		  delay_nop_1us(NOP2);
		  SCL_OUT_H(dev->scl);
		  delay_nop_1us(NOP2);
		  tmp = tmp << 1;
	  }
  
	  SCL_OUT_L(dev->scl);
	  //delay_nop_1us(NOP);
	  SDA_SET_IN(dev->sda);
	  delay_nop_1us(NOP2);
	  SCL_OUT_H(dev->scl);
	  delay_nop_1us(NOP2);
  
	  ret=SDA_GET_IN(dev->sda);
	  if(ret)
	  {
		  ack = KAL_FALSE;
	  }
	  else
	  {
		  ack = KAL_TRUE;
	  }   
	  SCL_OUT_L(dev->scl);
	  delay_nop_1us(NOP2);
  
	  SDA_SET_OUT(dev->sda);
  
	  return ack;
  }
 static unsigned char i2c_read_byte(I2C_GPIO_T *dev,bool ack)
  {
	  uint8_t i,receive=0;
	  SDA_SET_IN(dev->sda);
	  
	  for(i=0;i<8;i++ )
		  {
		   SCL_OUT_L(dev->scl);
		  delay_nop_1us(NOP2);
		  SCL_OUT_H(dev->scl);
		  delay_nop_1us(NOP2);
		  receive<<=1;
		  if(SDA_GET_IN(dev->sda))receive++;   
		   //delay_nop_1us(NOP2);
	  } 				   
	  if (ack)
		  IIC_Ack(dev); 
	  else
		  IIC_NAck(dev);
	  return receive;
  }
#if 1
 static uint8_t i2c_read_reg_org(I2C_GPIO_T *dev, u8 Read_regaddr[],u8 Read_regaddr_length,u8 ret_buf[],u8 read_len)
  {
	  uint8_t count = 0,i;
		  
		  i2c_start(dev);
		  i2c_transfer_byte(dev, dev->addr);
		  for(i=0;i<Read_regaddr_length;i++)
		  {
			i2c_transfer_byte(dev, Read_regaddr[i]);
		  }
		  i2c_start(dev);
		  i2c_transfer_byte(dev, dev->addr+1);
		  
			for(count=0;count<read_len;count++)
			{		   
			   if(count !=(read_len-1))ret_buf[count]=i2c_read_byte(dev,KAL_TRUE);
				  else	ret_buf[count]=i2c_read_byte(dev,KAL_FALSE);		  
			}	  
		  i2c_stop(dev);
#if 0
		  printk("i2c_read_byte==");
		  for(count=0;count<read_len;count++)
		  printk("%x,",ret_buf[count]);
		  printk("\n");
#endif		
		  return count;
  
  }
 #endif
  static uint8_t i2c_read_buf(I2C_GPIO_T *dev,u8 ret_buf[],u8 read_len)
  {
	  uint8_t count = 0;
		  
		  i2c_start(dev);
		  i2c_transfer_byte(dev, dev->addr+1);
		  
			for(count=0;count<read_len;count++)
			{		   
			   if(count !=(read_len-1))ret_buf[count]=i2c_read_byte(dev,KAL_TRUE);
				  else	ret_buf[count]=i2c_read_byte(dev,KAL_FALSE);		  
			}	  
		  i2c_stop(dev);
		
		  return count;
  
  }
 
 
 static uint8_t i2c_write_buf(I2C_GPIO_T *dev, u8 data[],u8 len)
  {
	  int i,ret;
  
	  i2c_start(dev);
	  i2c_transfer_byte(dev, dev->addr);
	  for(i=0;i<len;i++)
	  { 	  
			 ret=i2c_transfer_byte(dev, data[i]);
	  //   if(ret == KAL_TRUE)nwrite++;    
	   //  else return 0;
	  }
	  i2c_stop(dev);
	  return len;
  }
 static struct hrtimer led_time;

#define BREATH_MAX_STEP 69
 static u8 breath_step[BREATH_MAX_STEP]={0,0,0,3,6,9,12,
								  15,18,21,24,27,
								  30,34,38,42,46,
								  50,54,60,66,72,
								  78,84,92,102,115,
								  125,135,145,160,180,
								  210,255,255,255,210,
								  180,160,145,135,125,
								  115,102,92,84,78,
								  72,66,60,54,50,
								  46,42,38,34,30,
								  27,24,21,18,15,
								  12,9,6,3,0,0,0};

 static int AW9106_i2c_write_reg(unsigned char reg,unsigned char data)
 {
	
	unsigned char buf[2];
	buf[0]=reg;
	buf[1]=data;
	
	//IIC_DEV->sda=sda;
	i2c_write_buf(IIC_DEV,buf,2);
	 return 0;
 }
 static void AW9106_SoftReset(void)
 {		 
	 AW9106_i2c_write_reg(0x7f,0x00); 	
 }

 static void AW9523_breath_mode(int val)
{
   static int Breath_cnt2=0;
	    int i=0,reg_base=0x20;

		IIC_DEV->sda=IICSDA;
		for(i=0;i<12;i++)
		AW9106_i2c_write_reg(reg_base+i,breath_step[Breath_cnt2]/2);

		 IIC_DEV->sda=IICSDA1;		
		for(i=0;i<12;i++)
		AW9106_i2c_write_reg(reg_base+i,breath_step[Breath_cnt2]/2);

	if (Breath_cnt2<(BREATH_MAX_STEP-1))
		Breath_cnt2++;
	else
		Breath_cnt2=0;
}
//static int tt=0;
 enum hrtimer_restart led_time_hrtimer_func(struct hrtimer *timer)
{
	AW9523_breath_mode(1);
	hrtimer_forward_now(&led_time, ktime_set(0, 50*1000000));	//800ms
	
	return HRTIMER_RESTART;
}
#define mDELAY(x) msleep(x)
static void AW9523_init(void)
{
	IIC_DEV->sda=IICSDA;
	 AW9106_SoftReset();
	  AW9106_i2c_write_reg(0x12,0x00);	 //LED mode
	  AW9106_i2c_write_reg(0x13,0x00);
	 //   AW9106_i2c_write_reg(0x11,0x03);

	 IIC_DEV->sda=IICSDA1;
	 AW9106_SoftReset();
	 AW9106_i2c_write_reg(0x12,0x00);	//LED mode
	 AW9106_i2c_write_reg(0x13,0x00);

}
//#define DELAY 50
static int light=32,DELAY=110;
 static DEFINE_MUTEX(AW9523_mutex);

 static void AW9523_breath_front_loop(int8_t color)
  {
	  int i=0,reg_base=0x20;

	 mutex_lock(&AW9523_mutex);
	 if(color !=3)
	AW9523_init();
	if(color == 0x01)
	{	 
		 IIC_DEV->sda=IICSDA;
		for(i=0;i<12;i++)
		AW9106_i2c_write_reg(reg_base+i,0xff);
			
		IIC_DEV->sda=IICSDA1;
		 for(i=0;i<12;i++)
		  AW9106_i2c_write_reg(reg_base+i,0xff);		
		 
	}
	else if(color == 0x02)
	{
		 IIC_DEV->sda=IICSDA;
		for(i=0;i<12;i++)
		AW9106_i2c_write_reg(reg_base+i,0x0);

		 IIC_DEV->sda=IICSDA1;
		 for(i=0;i<12;i++)
		AW9106_i2c_write_reg(reg_base+i,0x0);
		 
	}
	else if(color == 0x04)
	{
		  hrtimer_init(&led_time, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
		led_time.function = led_time_hrtimer_func;
		hrtimer_start(&led_time, ktime_set(0, 50*1000000), HRTIMER_MODE_REL);
		
	}
	else if(color == 0x03)
	{
		
		IIC_DEV->sda=IICSDA;
		AW9106_i2c_write_reg(0x28,light);
		AW9106_i2c_write_reg(0x20,0xff);
		IIC_DEV->sda=IICSDA1;
		 AW9106_i2c_write_reg(0x24,light);
		AW9106_i2c_write_reg(0x20,0xff);
		mDELAY(DELAY);

		IIC_DEV->sda=IICSDA;
		AW9106_i2c_write_reg(0x29,light);
		AW9106_i2c_write_reg(0x21,0xff);	
		IIC_DEV->sda=IICSDA1;
		 AW9106_i2c_write_reg(0x23,light);
		 AW9106_i2c_write_reg(0x2b,0xff);
		mDELAY(DELAY);

		IIC_DEV->sda=IICSDA;
		AW9106_i2c_write_reg(0x2a,light);
		AW9106_i2c_write_reg(0x22,0xff);
		IIC_DEV->sda=IICSDA1;
		AW9106_i2c_write_reg(0x22,light);
		AW9106_i2c_write_reg(0x2a,0xff);
		mDELAY(DELAY);

		IIC_DEV->sda=IICSDA;
		AW9106_i2c_write_reg(0x2b,light);
		AW9106_i2c_write_reg(0x23,0xff);
		IIC_DEV->sda=IICSDA1;
		 AW9106_i2c_write_reg(0x21,light);
		AW9106_i2c_write_reg(0x29,0xff);
		mDELAY(DELAY);
//////////////////////////
		IIC_DEV->sda=IICSDA;
		AW9106_i2c_write_reg(0x20,light);
		AW9106_i2c_write_reg(0x24,0xff);
		IIC_DEV->sda=IICSDA1;
		 AW9106_i2c_write_reg(0x20,light);
		AW9106_i2c_write_reg(0x28,0xff);
		mDELAY(DELAY);

		IIC_DEV->sda=IICSDA;
		AW9106_i2c_write_reg(0x21,light);
		AW9106_i2c_write_reg(0x25,0xff);
		IIC_DEV->sda=IICSDA1;
		 AW9106_i2c_write_reg(0x2b,light);
		AW9106_i2c_write_reg(0x27,0xff);
		mDELAY(DELAY);

		IIC_DEV->sda=IICSDA;
		AW9106_i2c_write_reg(0x22,light);
		AW9106_i2c_write_reg(0x26,0xff);
		IIC_DEV->sda=IICSDA1;
		 AW9106_i2c_write_reg(0x2a,light);
		AW9106_i2c_write_reg(0x26,0xff);
		mDELAY(DELAY);
		IIC_DEV->sda=IICSDA;
		AW9106_i2c_write_reg(0x23,light);
		AW9106_i2c_write_reg(0x27,0xff);
		IIC_DEV->sda=IICSDA1;
		 AW9106_i2c_write_reg(0x29,light);
		AW9106_i2c_write_reg(0x25,0xff);
		mDELAY(DELAY);
/////////////////////////////////////////
		IIC_DEV->sda=IICSDA;
		AW9106_i2c_write_reg(0x24,light);
		AW9106_i2c_write_reg(0x28,0xff);
		IIC_DEV->sda=IICSDA1;
		 AW9106_i2c_write_reg(0x28,light);
		AW9106_i2c_write_reg(0x24,0xff);
		mDELAY(DELAY);

		IIC_DEV->sda=IICSDA;
		AW9106_i2c_write_reg(0x25,light);
		AW9106_i2c_write_reg(0x29,0xff);
		IIC_DEV->sda=IICSDA1;		 
		AW9106_i2c_write_reg(0x27,light);
		AW9106_i2c_write_reg(0x23,0xff);
		mDELAY(DELAY);

		IIC_DEV->sda=IICSDA;
		AW9106_i2c_write_reg(0x26,light);
		AW9106_i2c_write_reg(0x2a,0xff);
		IIC_DEV->sda=IICSDA1;
		 AW9106_i2c_write_reg(0x26,light);
		AW9106_i2c_write_reg(0x22,0xff);
		mDELAY(DELAY);

		IIC_DEV->sda=IICSDA;
		AW9106_i2c_write_reg(0x27,light);
		AW9106_i2c_write_reg(0x2b,0xff);
		IIC_DEV->sda=IICSDA1;
		 AW9106_i2c_write_reg(0x25,light);
		AW9106_i2c_write_reg(0x21,0xff);
		mDELAY(DELAY);
#if 0
///////////////////////////////////////
			IIC_DEV->sda=IICSDA;
			AW9106_i2c_write_reg(0x28,0x0);
			AW9106_i2c_write_reg(0x2c,0xff);
			IIC_DEV->sda=IICSDA1;
			 AW9106_i2c_write_reg(0x28,0x0);
			AW9106_i2c_write_reg(0x24,0xff);
			mDELAY(100);

			IIC_DEV->sda=IICSDA;
			AW9106_i2c_write_reg(0x29,0x0);
			AW9106_i2c_write_reg(0x2d,0xff);
			IIC_DEV->sda=IICSDA1;
			 AW9106_i2c_write_reg(0x27,0x0);
			AW9106_i2c_write_reg(0x23,0xff);
			mDELAY(100);

			IIC_DEV->sda=IICSDA;
			AW9106_i2c_write_reg(0x2a,0x0);
			AW9106_i2c_write_reg(0x2e,0xff);
			IIC_DEV->sda=IICSDA1;
		 	AW9106_i2c_write_reg(0x26,0x0);
			AW9106_i2c_write_reg(0x22,0xff);
			mDELAY(100);

			IIC_DEV->sda=IICSDA;
			AW9106_i2c_write_reg(0x2b,0x0);
			AW9106_i2c_write_reg(0x2f,0xff);
			IIC_DEV->sda=IICSDA1;
			 AW9106_i2c_write_reg(0x25,0x0);
			AW9106_i2c_write_reg(0x21,0xff);
			mDELAY(100);
#endif
	}
	 mutex_unlock(&AW9523_mutex);
       		  
  }

  static ssize_t yyd_misc_store(struct device *dev, struct device_attribute *attr,const char *buf, size_t count)
  {	
  int i=0;
	  find_gpio(IIC_DEV);
	i2c_init_gpio(IIC_DEV);
	IIC_DEV->addr=0xb6;
	if(buf[0] == '1')
	AW9523_breath_front_loop(1);
	else if(buf[0]== '2')
		AW9523_breath_front_loop(2);
	else if(buf[0]== '3')
		{
		 while(i++ <5)
		AW9523_breath_front_loop(3);
		}
	else if(buf[0]== '4')
		{
		 
		AW9523_breath_front_loop(4);
		}
	 return count;
 }
  static ssize_t yyd_misc_show(struct device *dev, struct device_attribute *attr, char *buf)
 {
	 u8 reg,buff[2];
	 buff[0]='2';
	  buff[1]='\0';
	 reg=0x10;
	i2c_read_reg_org(IIC_DEV,&reg,1,buff,1);
	if(buff[0]==0x12)buff[0]='g';
  	printk("55555555===%s\n",buff);
	return sprintf(buf, "%s\n", buff);	
	 return 0;  
 }
  static DEVICE_ATTR(yyd_misc, S_IWUSR | S_IWGRP | S_IRUGO, yyd_misc_show, yyd_misc_store);

 static int paoma_thread_func(void *data)
 {
	 AW9523_init();
 
	 for (;;) {
		 AW9523_breath_front_loop(3);
		 
		 printk("aw9523b_probe=222===");
		 if (kthread_should_stop())
			 break;
	 }
	 return 0;
 }

 static int misc_release (struct inode *node, struct file *file)
 {
	// gpio_free(IIC_DEV->scl);
	// gpio_free(IIC_DEV->sda);
	 printk("misc_release !\n");
	 return 0;	
 }
 static int misc_open (struct inode *inode, struct file *file)
 {
	// find_gpio(IIC_DEV);
	//i2c_init_gpio(IIC_DEV);
	 return nonseekable_open(inode, file);

 }
 
static ssize_t misc_write(struct file *pfile, const char __user *buf, size_t len, loff_t * offset)
{
	 char pbuf[len];	
	 static bool breath_flag=0,close_thread=1;	 
	 int tt;
	   if(copy_from_user(pbuf, buf,len))
	   {
		   return	 -EFAULT;  
	   }	  
	   printk("misc_write  !buf=%s\n",pbuf);
	if(pbuf[0]=='1' && pbuf[1]=='1')
	{
				
		if(pbuf[2]=='2')
		{
			AW9523_breath_front_loop(1);//all open

		}
		else if(pbuf[2]=='3')
		{
			 if(breath_flag == 1)
			 {
			 breath_flag=0;
			 hrtimer_cancel(&led_time);
			 }
			 if(close_thread)
			{
			  kthread_stop(paoma_thread);
			  close_thread=0;
			 }
			 
			AW9523_breath_front_loop(2);//all close

		}
		else if(pbuf[2]=='4')
		{
			breath_flag=1;
			//tt=simple_strtoul(&pbuf[4],NULL,10);
			if(close_thread ==0)
			AW9523_breath_front_loop(4);//breath mode
			//printk("misc_write	00buf=%d\n",tt);

		}
		else if(pbuf[2]=='5')
		{
			
			tt=simple_strtoul(&pbuf[4],NULL,10);
			
			DELAY=tt;
			printk("misc_write	00buf=%d\n",tt);

		}
		else if(pbuf[2]=='6')
		{
			
			tt=simple_strtoul(&pbuf[4],NULL,10);
			
			light=tt;
			printk("misc_write	00buf=%d\n",tt);

		}
		else if(pbuf[2]=='7')
		{
			if(close_thread ==0)
			{
			paoma_thread = kthread_run(paoma_thread_func, NULL, "paoma_thread");
			close_thread=1;
			}
		}
		
	}
	return len;
}

static ssize_t misc_read(struct file *pfile, char __user *to, size_t len, loff_t *offset)
{
	 char strbuf[len],ret;
	i2c_read_buf(IIC_DEV,strbuf,len);	
	ret=copy_to_user(to, strbuf, len);
	return 0;
}


static long misc_unlocked_ioctl (struct file *pfile, unsigned int cmd, unsigned long param)
{
			
	 return 0;
}

static const struct file_operations misc_fops = {
	//.owner = THIS_MODULE,
	.open = misc_open,
	.write = misc_write,
	.read = misc_read,
	.release = misc_release,
	.unlocked_ioctl = misc_unlocked_ioctl,
};

static int  misc_init(void)
{
	int ret = 0;

	ret = alloc_chrdev_region(&misc_dev, 0, 1, DEV_NAME);
			 if (ret< 0) {
			 printk("misc alloc_chrdev_region failed, %d", ret);
			return ret;
		}
		 misc_cdev= cdev_alloc();
		 if (misc_cdev == NULL) {
				 printk("misc cdev_alloc failed");
				 ret = -ENOMEM;
			 }
		cdev_init(misc_cdev, &misc_fops);
		 misc_cdev->owner = THIS_MODULE;
		 ret = cdev_add(misc_cdev, misc_dev, 1);
		 if (ret < 0) {
			  printk("Attatch file misc operation failed, %d", ret);			
		 }
			 
		 misc_class = class_create(THIS_MODULE, DEV_NAME);
			 if (IS_ERR(misc_class)) {
				 printk("Failed to create class(misc)!\n");
				 return PTR_ERR(misc_class);
			 }			 
		 misc_device = device_create(misc_class, NULL, misc_dev, NULL,DEV_NAME);
		 if (IS_ERR(misc_device))
			 printk("Failed to create misc_dev device\n");	
		
		 if (device_create_file(misc_device, &dev_attr_yyd_misc) < 0)
				printk("Failed to create device file(%s)!\n",
					  dev_attr_yyd_misc.attr.name);   
					
		return 0;
		
}
/*----------------------------------------------------------------------------*/

static int aw9523b_probe(struct platform_device *dev)
{

	misc_init();
	 find_gpio(IIC_DEV);
	i2c_init_gpio(IIC_DEV);
	IIC_DEV->addr=AW_ADDR1;

	if(NORMAL_BOOT == get_boot_mode())
		paoma_thread = kthread_run(paoma_thread_func, NULL, "paoma_thread");
	
	printk("aw9523b_probe====");
	return 0;
}
static void led_shutdown(struct platform_device * dev)
{
		IIC_DEV->sda=IICSDA;
		 AW9106_SoftReset();
		 	
		 IIC_DEV->sda=IICSDA1;
		 AW9106_SoftReset();
printk("led_shutdown--------\n");
}

static struct platform_driver aw_platform_driver = {
	.probe = aw9523b_probe,
	.shutdown    = led_shutdown,
	.driver = {
		   .name = "aw9523b",
		   .owner = THIS_MODULE,
	},
};
static struct platform_device aw_device = {
	.name	= "aw9523b",
};

static int __init aw_init(void)
{
	int ret=0;
    printk("aw9523b)\n");	
	ret =platform_device_register(&aw_device);
 	ret = platform_driver_register(&aw_platform_driver);
	if (ret) {
		printk("aw9523b77777");
		return ret;
	} 

	return ret;
}

static void __exit aw_exit(void)
{
  
}

module_init(aw_init);
module_exit(aw_exit);
MODULE_AUTHOR("Yongyida");
MODULE_DESCRIPTION("misc driver");
MODULE_LICENSE("GPL");

