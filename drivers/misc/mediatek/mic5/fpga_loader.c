#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/ioport.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/pm.h>
#include <linux/pm_runtime.h>
#include <asm/io.h>
#include <linux/platform_device.h>
#include <linux/timer.h>
#include <linux/jiffies.h>
#include <linux/delay.h>
#include <linux/vmalloc.h>

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


#include "i2c_wrapper.h"

#define FPGA_ADDR 0x80;
static I2C_GPIO_T fpga_dev;
static I2C_GPIO_T *pfpga_dev;

static unsigned char fpga_refresh_command[] =
{
	0x79, 0x00, 0x00
};

static unsigned char fpga_none_command[] =
{
	0xff, 0xff, 0xff, 0xff
};

static const unsigned short i2c_fpga_addr[] = {(0x40), I2C_CLIENT_END};
//static const struct i2c_device_id i2c_driver_fpga_id[] = {{"i2c_fpga",0},{}};
//static struct i2c_board_info __initdata mic5_dev={ I2C_BOARD_INFO("i2c_fpga", (0x40))};


//MODULE_DEVICE_TABLE(i2c, i2c_driver_fpga_id);
//static struct delayed_work fpga_reset_work;

static struct i2c_msg * i2c_recev_msg;
static int current_msg_pos;



//#define GPIO_RST_PIN   (GPIO169|0x80000000)

int send_to_device(char * data, int count)
{
//	printk("Timothy:fpga_loader.c->send_to_device(), count = %d\n", count);
	int nwrite;
	int i;
		
		
	if(data[0] == 0) // STOP causes this, send stop
	{
//		printk("the data send to device is:");
//		for(i = 1; i < count; i++)
//		{
//			printk("%x ", data[i]);
//		}
//		printk("\n");
		//nwrite = i2c_master_send(fpga_client, &data[1], count-1);

		nwrite =i2c_write_buf(pfpga_dev,  &data[1],count-1);
		if(nwrite != count-1)
		{
			printk("Timothy:not write enough, nwrite = %d, count = %d\n", nwrite, count);
		}
		else
		{
			printk("Timothy:write success, count = %d\n", count);
		}
		return nwrite;
	}
	else if(data[0] == 1) // RESTART causes this, do not send stop, just put int in buf
	{
		if(current_msg_pos >= 8)
		{
			printk("Timothy:buf is overflow, quit");
			return -2;
		}
		printk("Timothy:the data add to buf is:");
		for(i = 1; i < count; i++)
		{
			printk("%x ", data[i]);
		}
		printk("\n");
		i2c_recev_msg[current_msg_pos].addr =i2c_fpga_addr[0];
		i2c_recev_msg[current_msg_pos].len = count -1;
		i2c_recev_msg[current_msg_pos].buf = (unsigned char *)kzalloc(i2c_recev_msg[current_msg_pos].len * sizeof(unsigned char), GFP_KERNEL);
		memcpy(i2c_recev_msg[current_msg_pos].buf, &data[1], i2c_recev_msg[current_msg_pos].len);
		current_msg_pos++;
		printk("Timothy:000000000000\n");
		return count - 1;
	}
	else if(data[0] == 2)
	{
		
		//nwrite = i2c_master_send(fpga_client, fpga_refresh_command, 3);
		nwrite =i2c_write_buf(pfpga_dev,  fpga_refresh_command,3);
		printk("Timothy:send refresh command now =%d\n",nwrite);
		return nwrite;
	}
	else if(data[0] == 3)
	{
		printk("Timothy:send none command now\n");
		//nwrite = i2c_master_send(fpga_client, fpga_none_command, 4);
		//nwrite = i2c_master_send(fpga_client, fpga_none_command, 4);
		nwrite =i2c_write_buf(pfpga_dev,  fpga_none_command,4);
		nwrite =i2c_write_buf(pfpga_dev,  fpga_none_command,4);
		return nwrite;
	}
	else
	{
		printk("Timothy:unknow flag for send");
		return -1;
	}

}

EXPORT_SYMBOL(send_to_device);

int receive_from_device(char * data, int count)
{
	
	int nread;
	int i;
	char *ret_buf;
	//char regbuf[4]={0xe0};
	ret_buf=data;
	printk("Timothy:fpga_loader.c->receive_from_device()\n");

	
	if(current_msg_pos >= 8)
	{
		printk("Timothy:buf is overflow, quit");
		return -2;
	}
	i2c_recev_msg[current_msg_pos].addr = i2c_fpga_addr[0];
	//i2c_recev_msg[current_msg_pos].flags = I2C_M_RD | I2C_M_NO_RD_ACK;
	i2c_recev_msg[current_msg_pos].len = count;
	i2c_recev_msg[current_msg_pos].buf = data;
	current_msg_pos++;
#if 0
	printk("Timothy:the data send before read is:");
	for(i = 0; i < current_msg_pos-1; i++)
	{
		for(j = 0; j < i2c_recev_msg[i].len; j++)
		{
			printk("%x ,", i2c_recev_msg[i].buf[j]);
		}
	}
	printk("\n");
#endif		
		
	//nread = i2c_transfer(fpga_client->adapter, i2c_recev_msg, current_msg_pos);
	      nread =i2c_read_reg_org(pfpga_dev,i2c_recev_msg[current_msg_pos-2].buf,i2c_recev_msg[current_msg_pos-2].len,ret_buf,count);		
#if 0		
	if(nread != current_msg_pos)
	{
		printk("Timothy:not read enough, nread = %d, current_msg_pos = %d\n", nread, current_msg_pos);
	}
	else
	{
		printk("Timothy:read success, current_msg_pos = %d\n", current_msg_pos);
	}
#endif
	printk("Timothy:the data read from i2c is:");
	for(i = 0; i < count; i++)
	{
		printk("%x, ", data[i]);
	}
	printk("\n");
	for(i = 0; i < current_msg_pos-1; i++)
	{
		if(i2c_recev_msg[i].buf)
		{
			kfree(i2c_recev_msg[i].buf);
			i2c_recev_msg[i].buf = NULL;
		}
	}
	current_msg_pos = 0;

	return count;
}



EXPORT_SYMBOL(receive_from_device);

void init_i2c_io(void)
{
	static struct device_node *node;
	node = of_find_compatible_node(NULL, NULL, "mediatek,fpga");
	
	   pfpga_dev=&fpga_dev;
	   pfpga_dev->sda=of_get_named_gpio(node, "misc_sda_gpio87", 0);
	  pfpga_dev->scl=of_get_named_gpio(node, "misc_clk_gpio88", 0);
	  pfpga_dev->addr=FPGA_ADDR;

	  gpio_request(pfpga_dev->sda, "fpga_sda");
	 gpio_request(pfpga_dev->scl, "fpga_clk");
	
	i2c_init_gpio(pfpga_dev);

	i2c_recev_msg = (struct i2c_msg *)kzalloc(8 * sizeof(struct i2c_msg), GFP_KERNEL);
	current_msg_pos = 0;

}
#if 0
static int i2c_driver_fpga_probe(struct i2c_client * client, const struct i2c_device_id* id)
{
#if 0  //later  open
		if(firstflag)
		{
		  pfpga_dev=&fpga_dev;
		   pfpga_dev->sda=(GPIO105|0x80000000);
		  pfpga_dev->scl=(GPIO106|0x80000000);
		  pfpga_dev->addr=FPGA_ADDR;
		  
		i2c_init_gpio(pfpga_dev);
		firstflag=false;
		}
#endif	
		
	printk("Timothy:fpga_loader.c->i2c_driver_fpga_probe()\ add=0x%x\n",client->addr);
	if((client->addr == i2c_fpga_addr[0]))
	{
		fpga_client = client;
		printk("Timothy:fpga (%x) init ok\n", client->addr);
	}	
  
	return 0;
}

static int i2c_driver_fpga_remove(struct i2c_client * client)
{
	printk("Timothy:fpga_loader.c->i2c_driver_fpga_remove()\n");
	return 0;
}


static int i2c_driver_fpga_detect(struct i2c_client * client, struct i2c_board_info * info)
{
	printk("Timothy:fpga_loader.c->i2c_driver_fpga_detect()\n");
	struct i2c_adapter * p_adapter;
	const char *type_name = "i2c_fpga";
	p_adapter = client->adapter;
	printk("Timothy:adapter->nr = %d\n", p_adapter->nr);
	if(2 == p_adapter->nr)
	{
		if(info->addr == i2c_fpga_addr[0])
		{
			printk("Timothy:detect fpga (%x) on i2c adapter (%d)\n", info->addr, p_adapter->nr);
			strlcpy(info->type, type_name, I2C_NAME_SIZE);
			return 0;
		}
	}
	return ENODEV;
}

static struct i2c_driver i2c_driver_fpga=
{
	.class 			= I2C_CLASS_HWMON,
	.probe 			= i2c_driver_fpga_probe,
	.remove 		= i2c_driver_fpga_remove,
	.id_table 		= i2c_driver_fpga_id,
	.driver			=
	{
		.name 	= "i2c_fpga",
		.owner 	= THIS_MODULE,
	},
	//.detect 		= i2c_driver_fpga_detect,
	.address_list	= i2c_fpga_addr
};

static int __init i2c_driver_fpga_init(void)
{
	printk("Timothy:fpga_loader.c->i2c_driver_fpga_init()\n");
	//fpga_hw_init();
	i2c_recev_msg = (struct i2c_msg *)kzalloc(8 * sizeof(struct i2c_msg), GFP_KERNEL);
	current_msg_pos = 0;
	i2c_register_board_info(2, &mic5_dev, 1);
	return i2c_add_driver(&i2c_driver_fpga);
}

static void __exit i2c_driver_fpga_exit(void)
{
	printk("Timothy:fpga_loader.c->i2c_driver_fpga_exit()\n");
	if(i2c_recev_msg)
	{
		kfree(i2c_recev_msg);
		i2c_recev_msg = NULL;
	}
	current_msg_pos = 0;
	i2c_del_driver(&i2c_driver_fpga);
}

module_init(i2c_driver_fpga_init);
module_exit(i2c_driver_fpga_exit);

MODULE_AUTHOR("Timothy");
MODULE_DESCRIPTION("I2C device fpga loader");
MODULE_LICENSE("GPL");

#endif
