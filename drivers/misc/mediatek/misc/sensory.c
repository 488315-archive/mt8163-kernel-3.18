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

/*----------------------------------------------------------------------------*/
#define DEV_NAME   "sensory"
static struct cdev *misc_cdev;
static dev_t misc_dev;
static struct class *misc_class = NULL;
static struct device *misc_device = NULL;
 static unsigned int IICSCL,IICSDA;
// static struct hrtimer sstop_timer;

#define SDA_SET_OUT(sda)		gpio_direction_output(sda, 1)
#define SDA_SET_IN(sda)			gpio_direction_input(sda)
#define SDA_OUT_H(sda)		gpio_set_value(sda, 1)
#define SDA_OUT_L(sda)		gpio_set_value(sda, 0)
#define SDA_GET_IN(sda)		gpio_get_value(sda)
#define SCL_OUT_H(scl)		gpio_set_value(scl, 1)
#define SCL_OUT_L(scl)		gpio_set_value(scl, 0)
#define NOP2 	30000


// #define delay_nop_1us(wTime) { for (ggg=0; ggg<wTime*15; ggg++) ;}
 #define KAL_FALSE 0
#define KAL_TRUE 1

  typedef struct i2c_gpio{
	  u32 sda;
	  u32 scl;
	  u32 addr;
	  u32 speed;
  }I2C_GPIO_T;
static I2C_GPIO_T IIC_GPIO;
 static I2C_GPIO_T *IIC_DEV=&IIC_GPIO;
 static void find_gpio(I2C_GPIO_T *dev)
 {
	 struct device_node *node;
		 
	 node = of_find_compatible_node(NULL, NULL, "mediatek,adc_rst");
	IICSCL  = of_get_named_gpio(node, "iic_slc_gpio121", 0);
	IICSDA   = of_get_named_gpio(node, "iic_sda_gpio122", 0);
	 
	 gpio_request(IICSDA, "IICSDA");
	 gpio_request(IICSCL, "IICSCL");
	printk("yyyyyyyy====%d,%d\n",IICSCL,IICSDA);
	 dev->scl=IICSCL;
	 dev->sda=IICSDA;
 }

static void delay_nop_1us( int wTime)
 {
 	udelay(12);
 }

  static void i2c_init_gpio(I2C_GPIO_T *dev)
  {
	  gpio_direction_output(dev->scl, 1);
	  gpio_set_value(dev->scl, 1);
  
	  gpio_direction_output(dev->sda, 1);
	  gpio_set_value(dev->sda, 1);
  
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


  static ssize_t yyd_misc_store(struct device *dev, struct device_attribute *attr,const char *buf, size_t count)
  {	
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

 static int misc_release (struct inode *node, struct file *file)
 {
	 gpio_free(IIC_DEV->scl);
	 gpio_free(IIC_DEV->sda);
	 printk("misc_release !\n");
	 return 0;	
 }
 static int misc_open (struct inode *inode, struct file *file)
 {
	 find_gpio(IIC_DEV);
	i2c_init_gpio(IIC_DEV);
	IIC_DEV->addr=0x78;
	 return nonseekable_open(inode, file);

 }
 
static ssize_t misc_write(struct file *pfile, const char __user *buf, size_t len, loff_t * offset)
{
	 char pbuf[len];	
	 	 
	   if(copy_from_user(pbuf, buf,len))
	   {
		   return	 -EFAULT;  
	   }	  
	  //  printk("misc_write111111%c,%c====%s\n",pbuf[0],pbuf[1],pbuf);

	   i2c_write_buf(IIC_DEV,pbuf,len);
		  
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

static int __init misc_init(void)
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

static void __exit misc_exit(void)
{

}

module_init(misc_init);
module_exit(misc_exit);
MODULE_AUTHOR("Yongyida");
MODULE_DESCRIPTION("misc driver");
MODULE_LICENSE("GPL");

