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
//#include <mach/gpio.h>
#include <linux/gpio.h>
#include <linux/delay.h>

// codec init param
#include "cx20810_config.h"


#include <linux/wakelock.h>
//#include <linux/earlysuspend.h>

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
#include <linux/regulator/consumer.h>

#endif


#define I2C_CX20810_DRIVER_NAME "i2c_cx20810"
#define I2C_CX20810_DRIVER_NAME1 "i2c_cx20810_1"

static unsigned int RST1,RST2;

 unsigned int SPK_CTL;

//#define GPIO171_ADC_RST1_PIN  RST1 //(GPIO171|0x80000000)   //low power 3b
//#define GPIO128_ADC_RST2_PIN  RST2 // (GPIO128|0x80000000)   //high power 35

static struct i2c_driver i2c_driver_cx20810;

#define MAX_CX20810_NUM (3)

// g_client_cx20810[0] is on adapter 0 and its address is 0x35
// g_client_cx20810[1] is on adapter 1 and its address is 0x35
// g_client_cx20810[2] is on adapter 1 and its address is 0x3B
static struct i2c_client * g_client_cx20810[MAX_CX20810_NUM];
static const unsigned short i2c_cx20810_addr[] = {(0x35), (0x3B), I2C_CLIENT_END};
#if 0
static struct i2c_board_info __initdata cx20810_dev[]=
{ 	
	{I2C_BOARD_INFO(I2C_CX20810_DRIVER_NAME1, (0x3b))},
	{I2C_BOARD_INFO(I2C_CX20810_DRIVER_NAME, (0x35))},
};
#define I2C_NUM 3
#endif
static const struct i2c_device_id i2c_driver_cx20810_id[]=
{
    {I2C_CX20810_DRIVER_NAME, 0},
	{I2C_CX20810_DRIVER_NAME1, 0},
    {}
};

// function declaration
static int i2c_driver_cx20810_probe(struct i2c_client * client, const struct i2c_device_id* id);
static int i2c_driver_cx20810_remove(struct i2c_client * client);
//static int i2c_driver_cx20810_detect(struct i2c_client * client, struct i2c_board_info * info);
static int i2c_master_send_array_to_cx20810(const struct i2c_client *client, const char *buf, int length);
//static void cx20810_init(int index, int mode);
int cx20810_set_mode(int mode, int index);

// set cx20810 work mode  
int cx20810_set_mode(int mode, int index)
{
    //printk("Timothy:cx20810.c->cx20810_set_mode(), mode = %d, index = %d\n", mode, index);
 
    int ret=0;
    const char * param;
    int length;


    switch(mode)
    {
        case CX20810_NORMAL_MODE:
            param = codec_config_param_normal_mode;
            length = sizeof(codec_config_param_normal_mode);
            break;
        case CX20810_NORMAL_MODE_SIMPLE:
            param = codec_config_param_normal_mode_simple;
            length = sizeof(codec_config_param_normal_mode_simple);
            break;
        case CX20810_48K_16BIT_MODE:
            param = codec_config_param_48k_16bit_mode;
            length = sizeof(codec_config_param_48k_16bit_mode);
            break;
        case CX20810_96K_16BIT_MODE:
            param = codec_config_param_96k_16bit_mode;
            length = sizeof(codec_config_param_96k_16bit_mode);
            break;
        case CX20810_NIRMAL_MODE_CODEC3:
            param = codec3_config_param_normal_mode;
            length = sizeof(codec3_config_param_normal_mode);
            break;
        case CX20810_NIRMAL_MODE_CODEC3_SIMPLE:
            param = codec3_config_param_normal_mode_simple;
            length = sizeof(codec3_config_param_normal_mode_simple);
            break;
        default:
            return ret ;
            break;
    }

    // if client is null, return
    if(g_client_cx20810[index] == NULL)
    {
        printk("Timothy:cx20810(%d) is not detected yet\n", index);
        return -1;
    }

    ret = i2c_master_send_array_to_cx20810(g_client_cx20810[index], param, length);
    if(ret != 0)
    {
        printk("Timothy:cx82011[%x] init error!\n", g_client_cx20810[index]->addr);
        return -1;
    }
    else
    {
        printk("Timothy:cx20810[%x] init ok\n", g_client_cx20810[index]->addr);
		
        return 0;
    }
}
EXPORT_SYMBOL(cx20810_set_mode);

// send parameters to cx20810 as master
 unsigned char ADC_i2c_read_reg(unsigned char regaddr,int idex) 
{
	unsigned char rdbuf[1], wrbuf[1], ret, i;

	wrbuf[0] = regaddr;

	for (i=0; i<3; i++) 
	{
		ret = i2c_master_send( g_client_cx20810[idex] , wrbuf, 1);
		if (ret == 1)
			break;
	}
	
	ret = i2c_master_recv( g_client_cx20810[idex] , rdbuf, 1);
	
	if (ret != 1)
	{
		   printk("**********************   5555    failed  %s \r\n", __func__);
	
		dev_err(&g_client_cx20810[idex]->dev,"%s: i2c_master_recv() failed, ret=%d\n",
			__func__, ret);
	}
	
    	return rdbuf[0];
		
}

static int i2c_master_send_array_to_cx20810(const struct i2c_client *client, const char *buf, int length)
{
    
    int i;
    int nwrite;
    printk("Timothy:cx20810.c->i2c_master_send_array_to_cx20810()  client->addr=0x%x ,buf[0]=%d,buf[1]=%d\n", client->addr,buf[0],buf[1]);
    for(i = 0; i < (length / 2); i ++)
    {
        nwrite = i2c_master_send(client, buf + i * 2, 2);
        if(nwrite != 2)
        {     
            printk("Timothy:send to cx20810 error,,,nwrite=%d\n",nwrite);
            return -1;
        }
    }
    return 0;
}

int reset_mic_cx20810(void)
{
	int ret1=0 ,ret2=0;
	
	cx20810_set_mode(0,1);
	//ret1=ADC_i2c_read_reg(0x10,0);
	printk("111add35=%x\n", ret1);
	mdelay(650);
	cx20810_set_mode(0,0);
	//ret2=ADC_i2c_read_reg(0x10,1);
	printk("111add3b=%x\n",ret2 );

	if(ret1 ==0x5f && ret2 == 0x5f)
		return 1;
         return 0;
}
// initial cx20810
static void cx20810_init(int index, int mode)
{
    //printk("Timothy:cx20810.c->cx20810_init()\n");
    if(cx20810_set_mode(mode, index) == 0)
    {
        printk(KERN_ERR"KERN_ERR Timothy:cx20810 init success\n");
    }
    else
    {
        printk(KERN_ERR"KERN_ERR Timothy:cx20810 init fail\n");
    }
}
void spk_ctl_code63xx(int val) //mt_soc_code63xx.c
{
	gpio_direction_output(SPK_CTL, val);
	gpio_set_value(SPK_CTL, val);	
}
static void fpga_get_gpio_infor(void)
{	
	 struct device_node *node;
	
	node = of_find_compatible_node(NULL, NULL, "mediatek,adc_rst");
	RST1 = of_get_named_gpio(node, "adc_rst1_gpio126", 0);
	RST2 = of_get_named_gpio(node, "adc_rst2_gpio86", 0);
	SPK_CTL = of_get_named_gpio(node, "spk_f23_gpio46", 0);
	if(RST1>0)
	  gpio_request(RST1, "adc_rst1");
	else printk("rst1-----------\n");
	gpio_request(RST2, "adc_rst2");
	gpio_request(SPK_CTL, "spk_ctl");

	printk("111fpga_get_gpio_infor\n");
}
extern void init_i2c_io(void);

static int i2c_driver_cx20810_probe(struct i2c_client * client, const struct i2c_device_id* id)
{
	printk("Timothy:cx20810.c->i2c_driver_cx20810_probe(),client->addr=0x%x  %d\n",client->addr,client->adapter->nr);

	fpga_get_gpio_infor();
	init_i2c_io();
	spk_ctl_code63xx(0);

  if(client->adapter->nr == 2 && client->addr == i2c_cx20810_addr[0])
    {
       //     adc_set_gpio_output(RST2,1);
	// mdelay(30);
	//  adc_set_gpio_output(RST2,0);
	//  mdelay(30);
  	//  adc_set_gpio_output(RST2,1);
	//  mdelay(30);
	 
           g_client_cx20810[0] = client;
          cx20810_init(0, CX20810_NORMAL_MODE);
	printk("111add35=%x\n",ADC_i2c_read_reg(0x10,0) );
    }
    else if(client->adapter->nr == 2 && client->addr == i2c_cx20810_addr[1])  //0x3b--low vol
    {       
 	 //  adc_set_gpio_output(RST1,1);
	//   mdelay(30);
	 //   adc_set_gpio_output(RST1,0);
	//   mdelay(30);
	 //   adc_set_gpio_output(RST1,1);
	 //  mdelay(30);
            g_client_cx20810[1] = client;
            cx20810_init(1, CX20810_NORMAL_MODE);
	printk("111add3b=%x\n",ADC_i2c_read_reg(0x10,1) );
    }
    return 0;
}

static int i2c_driver_cx20810_remove(struct i2c_client * client)
{
    printk("Timothy:cx20810.c->i2c_driver_cx20810_remove()\n");
    return 0;
}

static int adc_gpio_init(struct platform_device *pdev)
{
	int ret = 0;
	struct regulator *reg;
#if 0
	static struct pinctrl *flashlight_pinctrl;
	static struct pinctrl_state *flashlight_hwen_high;
	static struct pinctrl_state *flashlight_hwen_low;

	flashlight_pinctrl = devm_pinctrl_get(&pdev->dev);
	if (IS_ERR(flashlight_pinctrl)) {
		printk("Cannot find flashlight pinctrl!");
		ret = PTR_ERR(flashlight_pinctrl);
	}
	/* Flashlight HWEN pin initialization */
	flashlight_hwen_high = pinctrl_lookup_state(flashlight_pinctrl, "rst_high");
	if (IS_ERR(flashlight_hwen_high)) {
		ret = PTR_ERR(flashlight_hwen_high);
		printk("%s : pinctrl err, flashlight_hwen_high\n", __func__);
	}

	flashlight_hwen_low = pinctrl_lookup_state(flashlight_pinctrl, "rst_low");
	if (IS_ERR(flashlight_hwen_low)) {
		ret = PTR_ERR(flashlight_hwen_low);
		printk("%s : pinctrl err, flashlight_hwen_low\n", __func__);
	}
	pinctrl_select_state(flashlight_pinctrl, flashlight_hwen_high);
#endif
	reg = regulator_get(&pdev->dev, "vmc");
	ret= regulator_set_voltage(reg, 3300000, 3300000);
	ret = regulator_enable(reg);

	reg = regulator_get(&pdev->dev, "vmch");
	ret= regulator_set_voltage(reg, 3000000, 3000000);
	ret = regulator_enable(reg);
	
	printk("adc_gpio_init--------\n");
	return ret;
}
static int adc_probe(struct platform_device *dev)
{
	adc_gpio_init(dev);
	i2c_add_driver(&i2c_driver_cx20810);	
	printk("adc_probe====");
	return 0;
}

#if 0
#ifdef CONFIG_HAS_EARLYSUSPEND
#ifdef CONFIG_EARLYSUSPEND
int yyd_lock_system=false;

  static void reset_cx2008(void)
{
	mt_set_gpio_out(GPIO128_ADC_RST2_PIN,1);
  	 mdelay(30);
	mt_set_gpio_out(GPIO128_ADC_RST2_PIN,0);
	mdelay(30);
	mt_set_gpio_out(GPIO128_ADC_RST2_PIN,1);
	mdelay(30);
	cx20810_init(0, CX20810_NORMAL_MODE);
	  printk("111add35=%x\n",ADC_i2c_read_reg(0x10,0) );

	  mt_set_gpio_out(GPIO171_ADC_RST1_PIN,1);
	  mdelay(30);
	  mt_set_gpio_out(GPIO171_ADC_RST1_PIN,0);
	   mdelay(30);
	   mt_set_gpio_out(GPIO171_ADC_RST1_PIN,1);
	   mdelay(30);  
            cx20810_init(1, CX20810_NORMAL_MODE);
	printk("111add3b=%x\n",ADC_i2c_read_reg(0x10,1) );

}
  static void m_suspend( struct early_suspend *h )
  {
  	if(yyd_lock_system)
	 mt_set_gpio_out(GPIO44_3_3V_PIN,0);
  }
  
  static void m_resume( struct early_suspend *h )
  {
 	 if(yyd_lock_system)
 	 {
 	 mt_set_gpio_out(GPIO44_3_3V_PIN,1);
	 mdelay(10);
	reset_cx2008();
 	 }
  }
  
  static struct early_suspend misc_early_suspend_handler = {
	  .level = EARLY_SUSPEND_LEVEL_STOP_DRAWING - 1,
	  .suspend = m_suspend,
	  .resume = m_resume,
  };
  
#endif
#endif
#endif
static const struct of_device_id tpd_of_match[] = {
	{.compatible = "mediatek,adc_config"},
	
};

static struct i2c_driver i2c_driver_cx20810=
{
  //  .class          = I2C_CLASS_HWMON,
    .probe          = i2c_driver_cx20810_probe,
    .remove         = i2c_driver_cx20810_remove,
    .id_table       = i2c_driver_cx20810_id,
    .driver         =
    {
        .of_match_table = tpd_of_match,
        .name   = I2C_CX20810_DRIVER_NAME,
        .owner  = THIS_MODULE,
    },

};
#ifdef CONFIG_OF
static const struct of_device_id FLASHLIGHT_of_match[] = {
	{.compatible = "mediatek,adc_rst1"},
	{},
};
#endif

static struct platform_driver adc_platform_driver = {
	.probe = adc_probe,
	
	.driver = {
		   .name = "adc_rst1",
		   .owner = THIS_MODULE,
#ifdef CONFIG_OF
		   .of_match_table = FLASHLIGHT_of_match,
#endif
	},
};

static int __init i2c_driver_cx20810_init(void)
{
	int ret=0;
    printk("Timothy:cx20810.c->i2c_driver_cx20810_init()\n");	
 	ret = platform_driver_register(&adc_platform_driver);
	if (ret) {
		printk("[flashlight_probe] platform_driver_register fail ~");
		return ret;
	} 
 	// i2c_register_board_info(3, cx20810_dev, 2);
#if 0
	#ifdef CONFIG_HAS_EARLYSUSPEND
	#ifdef CONFIG_EARLYSUSPEND
	register_early_suspend(&misc_early_suspend_handler);
	#endif
	#endif
#endif
  //  return i2c_add_driver(&i2c_driver_cx20810);
	return ret;
}

static void __exit i2c_driver_cx20810_exit(void)
{
    printk("Timothy:cx20810.c->i2c_driver_cx20810_exit()\n");
    i2c_del_driver(&i2c_driver_cx20810);
}



late_initcall(i2c_driver_cx20810_init);
module_exit(i2c_driver_cx20810_exit);

MODULE_AUTHOR("Timothy");
MODULE_DESCRIPTION("I2C device cx20810 loader");
MODULE_LICENSE("GPL");
