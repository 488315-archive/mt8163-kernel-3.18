#ifdef BUILD_LK
#include <platform/mt_gpio.h>
#include <platform/mt_i2c.h>
#include <platform/mt_pmic.h>
#include <string.h>
#else
#include <linux/string.h>
#include <linux/wait.h>
#include <linux/platform_device.h>
#include <linux/gpio.h>
#include <linux/pinctrl/consumer.h>
#include <linux/of_gpio.h>
#include <asm-generic/gpio.h>

#include <linux/kernel.h>
#include <linux/mm.h>
#include <linux/mm_types.h>
#include <linux/module.h>
#include <linux/types.h>
#include <linux/slab.h>
#include <linux/vmalloc.h>

#ifdef CONFIG_OF
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/regulator/consumer.h>
#include <linux/clk.h>
#endif
#endif
#include "lcm_drv.h"

/*static unsigned int GPIO_LCD_PWR_EN;*/
#if 0
static unsigned int GPIO_LCD_PWR_EN;
static unsigned int GPIO_LCD_RST_EN;

static void lcm_get_gpio_infor(void)
{
	static struct device_node *node;

	node = of_find_compatible_node(NULL, NULL, "mediatek,lcm");

	GPIO_LCD_PWR_EN = of_get_named_gpio(node, "lcm_power_gpio", 0);
	GPIO_LCD_RST_EN = of_get_named_gpio(node, "lcm_reset_gpio", 0);
}

static void lcm_set_gpio_output(unsigned int GPIO, unsigned int output)
{
	printk("lifei++++++++++++%ud\n",output);
	gpio_direction_output(GPIO, output);
	gpio_set_value(GPIO, output);
	printk("lifei--------------\n");
}

/* get LDO supply */
static int lcm_probe(struct device *dev)
{
	
	lcm_get_gpio_infor();

	return 0;
}

static const struct of_device_id lcm_of_ids[] = {
	{.compatible = "mediatek,lcm",},
	{}
};

static struct platform_driver lcm_driver = {
	.driver = {
		   .name = "mtk_lcm",
		   .owner = THIS_MODULE,
		   .probe = lcm_probe,
#ifdef CONFIG_OF
		   .of_match_table = lcm_of_ids,
#endif
		   },
};

static int __init lcm_init(void)
{
	pr_notice("LCM: Register lcm driver\n");
	if (platform_driver_register(&lcm_driver)) {
		pr_err("LCM: failed to register disp driver\n");
		return -ENODEV;
	}

	return 0;
}

static void __exit lcm_exit(void)
{
	platform_driver_unregister(&lcm_driver);
	pr_notice("LCM: Unregister lcm driver done\n");
}
late_initcall(lcm_init);
module_exit(lcm_exit);
MODULE_AUTHOR("mediatek");
MODULE_DESCRIPTION("Display subsystem Driver");
MODULE_LICENSE("GPL");

#endif


// ---------------------------------------------------------------------------
//  Local Constants
// ---------------------------------------------------------------------------

#define FRAME_WIDTH  										(720)
#define FRAME_HEIGHT 										(1280)

#define REGFLAG_DELAY             							0XFFE
#define REGFLAG_END_OF_TABLE      							0xFFF   // END OF REGISTERS MARKER

#define LCM_DSI_CMD_MODE									0

// ---------------------------------------------------------------------------
//  Local Variables
// ---------------------------------------------------------------------------

static LCM_UTIL_FUNCS lcm_util = {0};

#define SET_RESET_PIN(v)    								(lcm_util.set_reset_pin((v)))

#define UDELAY(n) 											(lcm_util.udelay(n))
#define MDELAY(n) 											(lcm_util.mdelay(n))


// ---------------------------------------------------------------------------
//  Local Functions
// ---------------------------------------------------------------------------

#define dsi_set_cmdq_V2(cmd, count, ppara, force_update)	lcm_util.dsi_set_cmdq_V2(cmd, count, ppara, force_update)
#define dsi_set_cmdq(pdata, queue_size, force_update)		lcm_util.dsi_set_cmdq(pdata, queue_size, force_update)
#define wrtie_cmd(cmd)										lcm_util.dsi_write_cmd(cmd)
#define write_regs(addr, pdata, byte_nums)					lcm_util.dsi_write_regs(addr, pdata, byte_nums)
#define read_reg											lcm_util.dsi_read_reg()
#define read_reg_v2(cmd, buffer, buffer_size)   				lcm_util.dsi_dcs_read_lcm_reg_v2(cmd, buffer, buffer_size)    

  
static struct LCM_setting_table lcm_initialization_setting[] = {
	
	/*
	Note :

	Data ID will depends on the following rule.
	
		count of parameters > 1	=> Data ID = 0x39
		count of parameters = 1	=> Data ID = 0x15
		count of parameters = 0	=> Data ID = 0x05

	Structure Format :

	{DCS command, count of parameters, {parameter list}}
	{REGFLAG_DELAY, milliseconds of time, {}},

	...

	Setting ending by predefined flag
	
	{REGFLAG_END_OF_TABLE, 0x00, {}}
	*/

	//{REGFLAG_DELAY, 200, {}},
	
	{0xE0, 1,{0x00}},
	
	{0xE1, 1,{0x93}},
	
	{0xE2, 1,{0x65}},
	
	{0xE3, 1,{0xF8}},
	

	{0xE0, 1,{0x04}},
	
	{0x2D, 1,{0x03}},
	
	{0xE0, 1,{0x00}},
	
	{0x80, 1,{0x01}},//03-4lane 02-3lane
	
	
	{0xE0, 1,{0x01}},
	
	{0x00, 1,{0x00}},
	
	{0x01, 1,{0x63}},
	
	{0x03, 1,{0x00}},
	
	{0x04, 1,{0x66}},
	
	
	{0x17, 1,{0x00}},
	
	{0x18, 1,{0xCF}},
	
	{0x19, 1,{0x03}},
	
	{0x1A, 1,{0x00}},
	
	{0x1B, 1,{0xCF}},
	
	{0x1C, 1,{0x03}},
	
	
	{0x1F, 1,{0x79}},
	
	{0x20, 1,{0x2D}},
	
	{0x21, 1,{0x2D}},
	
	{0x22, 1,{0x4F}},
	
	{0x26, 1,{0xF1}},
	
	
	{0x37, 1,{0x09}},
	
	{0x38, 1,{0x04}},
	
	{0x39, 1,{0x0C}},
	
	{0x3A, 1,{0x18}},
	
	{0x3C, 1,{0x78}},
	
	{0x40, 1,{0x04}},
	
	{0x41, 1,{0xA0}},
	
	
	{0x55, 1,{0x01}},
	
	{0x56, 1,{0x01}},
	
	{0x57, 1,{0x6D}},
	
	{0x58, 1,{0x0A}},
	
	{0x59, 1,{0x1A}},
	
	{0x5A, 1,{0x65}},
	
	{0x5B, 1,{0x14}},
	
	{0x5C, 1,{0x15}},
	
	
	{0x5D, 1,{0x70}},
	
	{0x5E, 1,{0x58}},
	
	{0x5F, 1,{0x48}},
	
	{0x60, 1,{0x3B}},
	
	{0x61, 1,{0x35}},
	
	{0x62, 1,{0x25}},
	
	{0x63, 1,{0x28}},
	
	{0x64, 1,{0x12}},
	
	{0x65, 1,{0x2C}},
	
	{0x66, 1,{0x2C}},
	
	{0x67, 1,{0x2E}},
	
	{0x68, 1,{0x4D}},
	
	{0x69, 1,{0x3B}},
	
	{0x6A, 1,{0x43}},
	
	{0x6B, 1,{0x35}},
	
	{0x6C, 1,{0x2F}},
	
	{0x6D, 1,{0x25}},
	
	{0x6E, 1,{0x13}},
	
	{0x6F, 1,{0x02}},
	
	{0x70, 1,{0x70}},
	
	{0x71, 1,{0x58}},
	
	{0x72, 1,{0x48}},
	
	{0x73, 1,{0x3B}},
	
	{0x74, 1,{0x35}},
	
	{0x75, 1,{0x25}},
	
	{0x76, 1,{0x28}},
	
	{0x77, 1,{0x12}},
	
	{0x78, 1,{0x2C}},
	
	{0x79, 1,{0x2C}},
	
	{0x7A, 1,{0x2E}},
	
	{0x7B, 1,{0x4D}},
	
	{0x7C, 1,{0x3B}},
	
	{0x7D, 1,{0x43}},
	
	{0x7E, 1,{0x35}},
	
	{0x7F, 1,{0x2F}},
	
	{0x80, 1,{0x25}},
	
	{0x81, 1,{0x13}},
	
	{0x82, 1,{0x02}},
	
	
	{0xE0, 1,{0x02}},
	
	{0x00, 1,{0x13}},
	{0x01, 1,{0x11}},
	
	{0x02, 1,{0x0B}},
	
	{0x03, 1,{0x09}},
	
	{0x04, 1,{0x07}},
	
	{0x05, 1,{0x05}},
	
	{0x06, 1,{0x1F}},
	
	{0x07, 1,{0x1F}},
	
	{0x08, 1,{0x1F}},
	
	{0x09, 1,{0x1F}},
	
	{0x0A, 1,{0x1F}},
	
	{0x0B, 1,{0x1F}},
	
	{0x0C, 1,{0x1F}},
	
	{0x0D, 1,{0x1F}},
	
	{0x0E, 1,{0x1F}},
	
	{0x0F, 1,{0x1F}},
	
	{0x10, 1,{0x1F}},
	
	{0x11, 1,{0x1F}},
	
	{0x12, 1,{0x01}},
	
	{0x13, 1,{0x03}},
	
	{0x14, 1,{0x1F}},
	
	{0x15, 1,{0x1F}},
	
	
	{0x16, 1,{0x12}},
	
	{0x17, 1,{0x10}},
	
	{0x18, 1,{0x0A}},
	
	{0x19, 1,{0x08}},
	
	{0x1A, 1,{0x06}},
	
	{0x1B, 1,{0x04}},
	
	{0x1C, 1,{0x1F}},
	
	{0x1D, 1,{0x1F}},
	
	{0x1E, 1,{0x1F}},
	
	{0x1F, 1,{0x1F}},
	
	{0x20, 1,{0x1F}},
	
	{0x21, 1,{0x1F}},
	
	{0x22, 1,{0x1F}},
	
	{0x23, 1,{0x1F}},
	
	{0x24, 1,{0x1F}},
	
	{0x25, 1,{0x1F}},
	
	{0x26, 1,{0x1F}},
	
	{0x27, 1,{0x1F}},
	
	{0x28, 1,{0x00}},
	
	{0x29, 1,{0x02}},
	
	{0x2A, 1,{0x1F}},
	
	{0x2B, 1,{0x1F}},
	
	
	{0x2C, 1,{0x00}},
	
	{0x2D, 1,{0x02}},
	
	{0x2E, 1,{0x08}},
	
	{0x2F, 1,{0x0a}},
	
	{0x30, 1,{0x04}},
	
	{0x31, 1,{0x06}},
	
	{0x32, 1,{0x1F}},
	
	{0x33, 1,{0x1F}},
	
	{0x34, 1,{0x1F}},
	
	{0x35, 1,{0x1F}},
	
	{0x36, 1,{0x1F}},
	
	{0x37, 1,{0x1F}},
	
	{0x38, 1,{0x1F}},
	
	{0x39, 1,{0x1F}},
	
	{0x3A, 1,{0x1F}},
	
	{0x3B, 1,{0x1F}},
	
	{0x3C, 1,{0x1F}},
	
	{0x3D, 1,{0x1F}},
	
	{0x3E, 1,{0x12}},
	
	{0x3F, 1,{0x10}},
	
	{0x40, 1,{0x1F}},
	
	{0x41, 1,{0x1F}},
	
	
	{0x42, 1,{0x01}},
	
	{0x43, 1,{0x03}},
	
	{0x44, 1,{0x09}},
	
	{0x45, 1,{0x0B}},
	
	{0x46, 1,{0x05}},
	
	{0x47, 1,{0x07}},
	
	{0x48, 1,{0x1F}},
	
	{0x49, 1,{0x1F}},
	
	{0x49, 1,{0x1F}},
	
	{0x4A, 1,{0x1F}},
	
	{0x4B, 1,{0x1F}},
	
	{0x4C, 1,{0x1F}},
	
	{0x4D, 1,{0x1F}},
	
	{0x4E, 1,{0x1F}},
	
	{0x4F, 1,{0x1F}},
	
	{0x50, 1,{0x1F}},
	
	{0x51, 1,{0x1F}},
	
	{0x52, 1,{0x1F}},
	
	{0x53, 1,{0x1F}},
	
	{0x54, 1,{0x13}},
	
	{0x55, 1,{0x11}},
	
	{0x56, 1,{0x1F}},
	
	{0x57, 1,{0x1F}},
	
	
	{0x58, 1,{0x40}},
	
	{0x59, 1,{0x00}},
	
	{0x5A, 1,{0x00}},
	
	{0x5B, 1,{0x30}},
	
	{0x5C, 1,{0x09}},
	
	{0x5D, 1,{0x30}},
	
	{0x5E, 1,{0x01}},
	
	{0x5F, 1,{0x02}},
	
	{0x60, 1,{0x30}},
	
	{0x61, 1,{0x01}},
	
	{0x62, 1,{0x02}},
	
	{0x63, 1,{0x03}},
	
	{0x64, 1,{0x64}},
	
	{0x65, 1,{0x75}},
	
	{0x66, 1,{0x0D}},
	
	{0x67, 1,{0x73}},
	
	{0x68, 1,{0x0A}},
	
	{0x69, 1,{0x06}},
	
	{0x6A, 1,{0x64}},
	
	{0x6B, 1,{0x08}},
	
	{0x6C, 1,{0x00}},
	
	{0x6D, 1,{0x00}},
	
	{0x6E, 1,{0x00}},
	
	{0x6F, 1,{0x00}},
	
	{0x70, 1,{0x00}},
	
	{0x71, 1,{0x00}},
	
	{0x72, 1,{0x06}},
	
	{0x73, 1,{0x86}},
	
	{0x74, 1,{0x00}},
	
	{0x75, 1,{0x07}},
	
	{0x76, 1,{0x00}},
	
	{0x77, 1,{0x5D}},
	
	{0x78, 1,{0x19}},
	
	{0x79, 1,{0x00}},
	
	{0x7A, 1,{0x05}},
	
	{0x7B, 1,{0x05}},
	
	{0x7C, 1,{0x00}},
	
	{0x7D, 1,{0x03}},
	
	{0x7E, 1,{0x86}},
	
	
	{0xE0, 1,{0x04}},
	
	{0x2B, 1,{0x2B}},
	
	{0x2E, 1,{0x44}},
	
	
	{0xE0, 1,{0x00}},
	
	{0xE6, 1,{0x02}},
	
	{0xE7, 1,{0x02}},
	
	
	{0x35, 1,{0x00}},	

	{REGFLAG_DELAY, 200, {}},

	{0x11, 0,{0x00}},

	{REGFLAG_DELAY, 200, {}},

	{0x29, 0,{0x00}},

	{REGFLAG_DELAY, 200, {}},
};



static struct LCM_setting_table lcm_sleep_out_setting[] = {
    // Sleep Out
	{0x11, 0, {0x00}},
    {REGFLAG_DELAY, 120, {}},

    // Display ON
	{0x29, 0, {0x00}},
   {REGFLAG_DELAY, 100, {}},
//	{REGFLAG_END_OF_TABLE, 0x00, {}}
};


static struct LCM_setting_table lcm_deep_sleep_mode_in_setting[] = {
	// Display off sequence
	{0x28, 0, {0x00}},

    // Sleep Mode On
	{0x10, 0, {0x00}},

	{REGFLAG_END_OF_TABLE, 0x00, {}}
};





static void push_table(struct LCM_setting_table *table, unsigned int count, unsigned char force_update)
{
	unsigned int i;

    for(i = 0; i < count; i++) {
		
        unsigned cmd;
        cmd = table[i].cmd;
		
        switch (cmd) {
			
            case REGFLAG_DELAY :
                MDELAY(table[i].count);
                break;
				
            case REGFLAG_END_OF_TABLE :
                break;
				
            default:
				dsi_set_cmdq_V2(cmd, table[i].count, table[i].para_list, force_update);
				//UDELAY(5);//soso add or it will fail to send register
       	}
    }
	
}


// ---------------------------------------------------------------------------
//  LCM Driver Implementations
// ---------------------------------------------------------------------------

static void lcm_set_util_funcs(const LCM_UTIL_FUNCS *util)
{
    memcpy(&lcm_util, util, sizeof(LCM_UTIL_FUNCS));
}


static void lcm_get_params(LCM_PARAMS *params)
{
		memset(params, 0, sizeof(LCM_PARAMS));
	
		params->type   = LCM_TYPE_DSI;

		params->width  = FRAME_WIDTH;
		params->height = FRAME_HEIGHT;

		// enable tearing-free
		params->dbi.te_mode = LCM_DBI_TE_MODE_DISABLED;
		params->dbi.te_edge_polarity = LCM_POLARITY_RISING;

		params->dsi.mode   = SYNC_EVENT_VDO_MODE;
	
		// DSI
		/* Command mode setting */
		params->dsi.LANE_NUM				= LCM_TWO_LANE;
		//The following defined the fomat for data coming from LCD engine.
		params->dsi.data_format.color_order = LCM_COLOR_ORDER_RGB;
		params->dsi.data_format.trans_seq   = LCM_DSI_TRANS_SEQ_MSB_FIRST;
		params->dsi.data_format.padding     = LCM_DSI_PADDING_ON_LSB;
		params->dsi.data_format.format      = LCM_DSI_FORMAT_RGB888;

		// Highly depends on LCD driver capability.
		// Not support in MT6573
		params->dsi.packet_size=256;

		// Video mode setting		
		params->dsi.intermediat_buffer_num = 2;

		params->dsi.PS=LCM_PACKED_PS_24BIT_RGB888;

		params->dsi.vertical_sync_active				= 2;
		params->dsi.vertical_backporch					= 12;
		params->dsi.vertical_frontporch					= 4;
		params->dsi.vertical_active_line				= FRAME_HEIGHT; 

		params->dsi.horizontal_sync_active				= 8;
		params->dsi.horizontal_backporch				= 64;
		params->dsi.horizontal_frontporch				= 64;
		params->dsi.horizontal_active_pixel				= FRAME_WIDTH;

		// Bit rate calculation
		//params->dsi.pll_div1=37;		// fref=26MHz, fvco=fref*(div1+1)	(div1=0~63, fvco=500MHZ~1GHz)
		//params->dsi.pll_div2=1; 		// div2=0~15: fout=fvo/(2*div2)
		params->dsi.PLL_CLOCK = 400;
}



static void dsp_lcm_init(void)
{		
  	 push_table(lcm_initialization_setting, sizeof(lcm_initialization_setting) / sizeof(struct LCM_setting_table), 1);
}

extern void spk_ctl_code63xx(int val) ;
extern bool system_shutdown_flag;
extern bool yyd_audio_shutdown_flag;
extern void lcm_set_gpio_output(unsigned int GPIO, unsigned int output);
extern unsigned int GPIO_LCD_RST_EN,GPIO_LCD_BACKLIGHT;

static void lcm_suspend(void)
{
	lcm_set_gpio_output(GPIO_LCD_RST_EN, 0);
	lcm_set_gpio_output(GPIO_LCD_BACKLIGHT, 0);
	if(system_shutdown_flag==true)
	{		
		spk_ctl_code63xx(0);
		 yyd_audio_shutdown_flag=true;
	}
	MDELAY(20);
 	   push_table(lcm_deep_sleep_mode_in_setting, sizeof(lcm_deep_sleep_mode_in_setting) / sizeof(struct LCM_setting_table), 1);
}


static void lcm_resume(void)
{
	lcm_set_gpio_output(GPIO_LCD_RST_EN, 1);	
	lcm_set_gpio_output(GPIO_LCD_BACKLIGHT, 1);	
	MDELAY(20);
	dsp_lcm_init();
	MDELAY(10);		
	push_table(lcm_sleep_out_setting, sizeof(lcm_sleep_out_setting) / sizeof(struct LCM_setting_table), 1);
}

static void lcm_init_power(void)
{

}

static void lcm_suspend_power(void)
{

}

static void lcm_resume_power(void)
{
}

LCM_DRIVER jd9365_dsi_hd_drv = 
{
    .name			= "jd9365_dsi_hd_drv",
	.set_util_funcs = lcm_set_util_funcs,
	.get_params     = lcm_get_params,
	.init           = dsp_lcm_init,
	.suspend        = lcm_suspend,
	.resume         = lcm_resume,
//	.compare_id    = lcm_compare_id,
	.init_power = lcm_init_power,
	.resume_power = lcm_resume_power,
	.suspend_power = lcm_suspend_power,
};

