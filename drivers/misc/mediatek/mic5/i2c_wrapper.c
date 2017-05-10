#include <linux/module.h>
#include <linux/delay.h>
#include <linux/string.h>
#include <linux/ctype.h>
#include <linux/leds.h>
//#include <linux/leds-mt65xx.h>
#include <linux/workqueue.h>
#include <linux/wakelock.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/cdev.h>
#include <linux/device.h>   /*class_create*/ 
#include <asm/uaccess.h>	/* copy_*_user */

#include <linux/i2c.h>
//#include <mach/mt_gpio.h>
#include <linux/i2c.h>
#include "i2c_wrapper.h"

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


#define KAL_FALSE false
#define KAL_TRUE true


bool wrapper_i2c_write_reg_org(struct i2c_client *client, u8 regdata[],u8 length)
{
	bool ack = KAL_TRUE;
	unsigned char ret;
	unsigned char wrbuf[length];
	int i = 0;

	
	for (i=0; i<length; i++)
	{
		wrbuf[i] = regdata[i];
	}
	
	ret = i2c_master_send(client, wrbuf, length);
	if (ret != length) {
		I2C_WRP_ERR("%s: failed ret = %d\n", client->name, ret);
		ack = KAL_FALSE;
	}

	return ack;
}

bool wrapper_i2c_write_regdata(struct i2c_client *client, u8 reg,u8 data)
{
	bool ack = KAL_TRUE;
	unsigned char i;
	unsigned char wrbuf[2];

	wrbuf[0] = reg;
	wrbuf[1] = data;
	
	for (i=0; i<I2C_MAX_TRY; i++)
	{
		ack = wrapper_i2c_write_reg_org(client, wrbuf, 2);
		if (ack == KAL_TRUE) // ack success
			break;
	}
	return ack;
}

bool wrapper_i2c_write_data(struct i2c_client *client, u8 data[], u8 length)
{
	bool ack = KAL_FALSE;
	unsigned char i;

	for (i=0; i<I2C_MAX_TRY; i++)
	{
		ack = wrapper_i2c_write_reg_org(client, data, length);
		if (ack == KAL_TRUE) // ack success
			break;
	}
	return ack;
}



bool wrapper_i2c_write_regs(struct i2c_client *client, I2C_REGDATA_T regsdata[],u8 length)
{
	int i = 0;
	for (i = 0; i<length; i++)
	{
		if(wrapper_i2c_write_regdata(client,regsdata[i].reg, regsdata[i].data) == KAL_FALSE)
		{
			I2C_WRP_ERR("when write regsdata[%d] failed\n", i);
			return KAL_FALSE;
		}
		
	}

	return KAL_TRUE;
}

bool wrapper_i2c_read_regdata(struct i2c_client *client, u8 regaddr, u8 *data, u8 length)
{
	unsigned char  wrbuf[1], ret, i;
	bool ack = KAL_FALSE;

	wrbuf[0] = regaddr;
	

	for (i=0; i<I2C_MAX_TRY; i++) 
	{
		ret = i2c_master_send(client, wrbuf, 1);
		
		if (ret == 1){
			break;
		}
		else if(ret != 1 && i == I2C_MAX_TRY)
		{
			I2C_WRP_ERR("%s: failed\n", client->name);
			ack = KAL_FALSE;
			return ack;
		}
	}
	
	for (i=0; i<I2C_MAX_TRY; i++) 
	{
		ret = i2c_master_recv(client, data, length);

		if (ret != length)
		{
			I2C_WRP_ERR("%s: failed\n", client->name);
			ack = KAL_FALSE;
		}
	}
	ack = KAL_TRUE;
	
    return ack;
		
}

bool wrapper_i2c_read_data(struct i2c_client *client, u8 *data, u8 length) 
{
	bool ack = KAL_TRUE;
	unsigned char ret;
	unsigned char i = 0;
	
	for (i=0; i<I2C_MAX_TRY; i++)
	{
		ret = i2c_master_recv(client, data, length);
	}
	
	if (ret != length)
	{
		I2C_WRP_ERR("%s: failed\n", client->name);
		ack = KAL_FALSE;
	}
	
    return ack;
}
//=========================================================================
void delay_nop_1us(u16 wTime)
{
    u16 i;
#if defined(MT6223) 
    for (i=0; i<wTime*30; i++) ;  
#elif defined(MT6235)
    for (i=0; i<wTime*8; i++) ;
#else
    for (i=0; i<wTime*15; i++) ;
#endif
}

#define SDA_SET_OUT(sda)		gpio_direction_output(sda, 1);
#define SDA_SET_IN(sda)			gpio_direction_input(sda)
								

#define SDA_OUT_H(sda)		gpio_set_value(sda, 1);
#define SDA_OUT_L(sda)		gpio_set_value(sda, 0);
#define SDA_GET_IN(sda)		gpio_get_value(sda)

#define SCL_OUT_H(scl)		gpio_set_value(scl, 1);
#define SCL_OUT_L(scl)		gpio_set_value(scl, 0);

#define NOP 	10*4
#define NOP2 	NOP*2

void i2c_init_gpio(I2C_GPIO_T *dev)
{
	gpio_direction_output(dev->scl, 1);
	gpio_set_value(dev->scl, 1);

	gpio_direction_output(dev->sda, 1);
	gpio_set_value(dev->sda, 1);

}
void i2c_start(I2C_GPIO_T *dev)
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
void i2c_restart(I2C_GPIO_T *dev)
{
	u32 sda = dev->sda;
	u32 scl = dev->scl;

	SDA_OUT_H(sda);
	SCL_OUT_H(scl);
	delay_nop_1us(NOP2);

	SDA_OUT_L(sda);
	delay_nop_1us(NOP2);
	
}

void i2c_stop(I2C_GPIO_T *dev)
{
	u32 sda = dev->sda;
	u32 scl = dev->scl;

	SCL_OUT_L(scl);
	delay_nop_1us(NOP);
	
	SDA_OUT_L(sda);
	delay_nop_1us(NOP);

	SCL_OUT_H(scl);
	delay_nop_1us(NOP);
	
	SDA_OUT_H(sda);
	delay_nop_1us(NOP);
}
void IIC_NAck(I2C_GPIO_T *dev)
{
	  SCL_OUT_L(dev->scl);
	SDA_SET_OUT(dev->sda);
	SDA_OUT_H(dev->sda);
	delay_nop_1us(NOP2);
	 SCL_OUT_H(dev->scl);
	delay_nop_1us(NOP2);
	SCL_OUT_L(dev->scl);
}

void IIC_Ack(I2C_GPIO_T *dev)
{
	  SCL_OUT_L(dev->scl);
	SDA_SET_OUT(dev->sda);
	SDA_OUT_L(dev->sda);
	delay_nop_1us(NOP2);
	 SCL_OUT_H(dev->scl);
	delay_nop_1us(NOP2);
	SCL_OUT_L(dev->scl);
}

unsigned char i2c_read_byte(I2C_GPIO_T *dev,bool ack)
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

bool i2c_transfer_byte(I2C_GPIO_T *dev, u8 data)
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

uint8_t i2c_read_reg_org(I2C_GPIO_T *dev, u8 Read_regaddr[],u8 Read_regaddr_length,u8 ret_buf[],u8 read_len)
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
				else  ret_buf[count]=i2c_read_byte(dev,KAL_FALSE);			
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

uint8_t i2c_write_buf(I2C_GPIO_T *dev, u8 data[],u8 len)
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



