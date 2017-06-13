#include <linux/irq.h>
#include <asm/irq.h>
#include <asm/io.h>
#include <linux/gpio.h>
#include <linux/timer.h>
#include <linux/kthread.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/types.h>

#include <linux/hrtimer.h>
#include <linux/jiffies.h>
#include <linux/input.h>
#include <linux/of_irq.h>
#include <linux/of.h>
#include <linux/string.h>
#include <linux/fs.h>
#include <linux/pinctrl/consumer.h>
#include <linux/of_gpio.h>
#include <linux/switch.h>
#include <linux/input.h>


#define TOUCH  1
#define NO_TOUCH  0
#define TP_NAME  "touchsensor"

static struct switch_dev touchsensor_dev;
static struct input_dev *touchsensor_input_dev;
static struct hrtimer dance_timer,touch_time;
static unsigned long left_time=0,right_time=0;
struct hrtimer audio_stop_timer;
static int timer_sta = 0;

static unsigned int tp_irq;
static struct pinctrl *pinctrl1;
static struct pinctrl_state *tp_int8, *tp_int9, *tp_int10, *tp_int11, *tp_int16, *tp_int17;
static unsigned int int8_gpio30,int9_gpio31,int10_gpio32,int11_gpio43,int16_gpio48,int17_gpio49;

#ifdef CONFIG_Y50_TOUCHSENSOR

#else
static unsigned long dis_time=0,left_touch=0,right_touch=0;
#endif

void commit_status(char *switch_name)
{
	
		touchsensor_dev.name=switch_name;
		
		switch_set_state(&touchsensor_dev, TOUCH);
		
		touchsensor_dev.state=NO_TOUCH;
		touchsensor_dev.name = TP_NAME;
		
		printk("touchsensor -------%s----\n",switch_name);	
}

static int tp_get_gpio_info(struct platform_device *pdev)
{
	int ret;
	
	pinctrl1 = devm_pinctrl_get(&pdev->dev);
	if (IS_ERR(pinctrl1)) {
		ret = PTR_ERR(pinctrl1);
		dev_err(&pdev->dev, "tp Cannot find touch pinctrl1!\n");
		return ret;
	}
	tp_int8 = pinctrl_lookup_state(pinctrl1, "tp_int8");
	if (IS_ERR(tp_int8)) {
		ret = PTR_ERR(tp_int8);
		dev_err(&pdev->dev, "tp Cannot find touch pinctrl default %d!\n", ret);
	}
	tp_int9 = pinctrl_lookup_state(pinctrl1, "tp_int9");
	if (IS_ERR(tp_int9)) {
		ret = PTR_ERR(tp_int9);
		dev_err(&pdev->dev, "tp Cannot find touch pinctrl default %d!\n", ret);
	}
	tp_int10 = pinctrl_lookup_state(pinctrl1, "tp_int10");
	if (IS_ERR(tp_int10)) {
		ret = PTR_ERR(tp_int10);
		dev_err(&pdev->dev, "tp Cannot find touch pinctrl default %d!\n", ret);
	}
	tp_int11 = pinctrl_lookup_state(pinctrl1, "tp_int11");
	if (IS_ERR(tp_int11)) {
		ret = PTR_ERR(tp_int11);
		dev_err(&pdev->dev, "tp Cannot find touch pinctrl default %d!\n", ret);
	}

	tp_int16 = pinctrl_lookup_state(pinctrl1, "tp_int16");
	if (IS_ERR(tp_int16)) {
		ret = PTR_ERR(tp_int16);
		dev_err(&pdev->dev, "tp Cannot find touch pinctrl default %d!\n", ret);
	}
	tp_int17 = pinctrl_lookup_state(pinctrl1, "tp_int17");
	if (IS_ERR(tp_int17)) {
		ret = PTR_ERR(tp_int17);
		dev_err(&pdev->dev, "tp Cannot find touch pinctrl default %d!\n", ret);
	}
	
	pinctrl_select_state(pinctrl1, tp_int9);
	pinctrl_select_state(pinctrl1, tp_int10);
	pinctrl_select_state(pinctrl1, tp_int11);
	pinctrl_select_state(pinctrl1, tp_int8);
	pinctrl_select_state(pinctrl1, tp_int16);
	pinctrl_select_state(pinctrl1, tp_int17);
	
	printk("tp_get_gpio_info-----\n");
	return ret;
}
static irqreturn_t tp_eint8_interrupt_handler(int irq, void *dev_id)
{
	
#ifdef CONFIG_Y50_TOUCHSENSOR	
		commit_status("left");
		left_time=jiffies;
#else
	commit_status("t_head");//du zi
#endif	
	printk("yydd8----------\n");

	return IRQ_HANDLED;
}

static irqreturn_t tp_eint9_interrupt_handler(int irq, void *dev_id)
{
	
#ifdef CONFIG_Y50_TOUCHSENSOR
		commit_status("right");
		right_time=jiffies;
#else	
		commit_status("yyd4");//cntoen
#endif
	printk("yydd9----------\n");

	return IRQ_HANDLED;
}
static irqreturn_t tp_eint10_interrupt_handler(int irq, void *dev_id)
{

#ifdef CONFIG_Y50_TOUCHSENSOR	
		commit_status("head");
#else	
		commit_status("yyd6");	 //volup
		
		input_report_key(touchsensor_input_dev, KEY_VOLUMEUP,1);
		input_sync(touchsensor_input_dev);
		input_report_key(touchsensor_input_dev, KEY_VOLUMEUP,0);
		input_sync(touchsensor_input_dev);

		 left_touch=jiffies;
		dis_time = abs(left_touch - right_touch);
		if(dis_time < HZ)
		{
			left_touch = 0;
			right_touch = 0;			
			if(timer_sta == 0)
			{
				timer_sta = 1;
				hrtimer_forward_now(&touch_time, ktime_set(2, 0));
				hrtimer_start(&touch_time, ktime_set(2, 0), HRTIMER_MODE_REL); 
			}
		}
#endif
	printk("yydd10-----------\n");

	return IRQ_HANDLED;
}
static irqreturn_t tp_eint11_interrupt_handler(int irq, void *dev_id)
{
#ifdef CONFIG_Y50_TOUCHSENSOR
			commit_status("back");
#else
		commit_status("yyd5");//voldown
	
		input_report_key(touchsensor_input_dev, KEY_VOLUMEDOWN,1);
		input_sync(touchsensor_input_dev);
		input_report_key(touchsensor_input_dev, KEY_VOLUMEDOWN,0);
		input_sync(touchsensor_input_dev);
		
		  right_touch=jiffies;
		   dis_time = abs(left_touch - right_touch);
		   if(dis_time < HZ)
		   {
			   left_touch = 0;
			   right_touch = 0; 		   
			   if(timer_sta == 0)
			   {
				   timer_sta = 1;
				   hrtimer_forward_now(&touch_time, ktime_set(2, 0));
				   hrtimer_start(&touch_time, ktime_set(2, 0), HRTIMER_MODE_REL); 
			   }
		   }
#endif	
	printk("yydd11-----------\n");

	return IRQ_HANDLED;
}

static irqreturn_t tp_eint16_interrupt_handler(int irq, void *dev_id)
{
#ifdef CONFIG_Y50_TOUCHSENSOR
	
#else
	
#endif	
printk("yydd16-----------\n");
	return IRQ_HANDLED;
}
static irqreturn_t tp_eint17_interrupt_handler(int irq, void *dev_id)
{
#ifdef CONFIG_Y50_TOUCHSENSOR
	
#else
	
#endif

printk("yydd17-------------\n");

	return IRQ_HANDLED;
}
extern bool audio_stop_flag;
extern bool yyd_main_server;

enum hrtimer_restart commit_audio_hrtimer_func(struct hrtimer *timer)
{	
	if(audio_stop_flag ==true && yyd_main_server==false)
	commit_status("5micoff");
	else if(audio_stop_flag ==false)
	commit_status("5micon");		
	hrtimer_forward_now(&audio_stop_timer, ktime_set(1, 0)); 
	
	return HRTIMER_RESTART;
}
enum hrtimer_restart double_touch_commit(struct hrtimer *timer)
{
	if((gpio_get_value(int10_gpio32) == 0) && (0 == gpio_get_value(int11_gpio43)))
	{
		commit_status("dance");		
	}
	timer_sta = 0;	
	return HRTIMER_NORESTART;
}

enum hrtimer_restart commit_delay_hrtimer_func(struct hrtimer *timer)
{
	if((jiffies -right_time) >4*HZ)right_time=0;
	if((jiffies -left_time) >4*HZ)left_time=0;

	if(right_time >0 && left_time>0 )
	{
		if( (jiffies -right_time) >3*HZ && (jiffies -right_time) <4*HZ && (jiffies -left_time) >3*HZ && (jiffies -left_time) <4*HZ && abs(left_time-right_time)<10)	
			commit_status("dance");

		 printk("commit_status  dance  jiffes=%ld,right_time=%ld,left_time=%ld\n",jiffies,right_time,left_time);
	}
	hrtimer_forward_now(&dance_timer, ktime_set(1, 0));	
	
	return HRTIMER_RESTART;
}

static int tp_probe(struct platform_device *dev)
{
	struct device_node *node = NULL;
	int ret,err;
	int debounce=17;
	//ktime_t ktime;
	touchsensor_dev.name = TP_NAME;
	touchsensor_dev.index = 0;
	touchsensor_dev.state = 0;
	
	ret=switch_dev_register(&touchsensor_dev);
	tp_get_gpio_info(dev);
	node = of_find_compatible_node(NULL, NULL, "mediatek,touchsensor");
	if (node) {
		
		int8_gpio30= of_get_named_gpio(node, "int8_gpio30", 0);
		int9_gpio31= of_get_named_gpio(node, "int9_gpio31", 0);
		int10_gpio32= of_get_named_gpio(node, "int10_gpio32", 0);
		int11_gpio43= of_get_named_gpio(node, "int11_gpio43", 0);
		int16_gpio48= of_get_named_gpio(node, "int16_gpio48", 0);
		int17_gpio49= of_get_named_gpio(node, "int17_gpio49", 0);

		gpio_request(int8_gpio30, "int8_gpio30");
		gpio_request(int9_gpio31, "int9_gpio31");
		gpio_request(int10_gpio32, "int10_gpio32");
		gpio_request(int11_gpio43, "int11_gpio43");
		gpio_request(int16_gpio48, "int16_gpio48");
		gpio_request(int17_gpio49, "int17_gpio49");

		gpio_set_debounce(int8_gpio30, debounce);
		gpio_set_debounce(int9_gpio31, debounce);
		gpio_set_debounce(int10_gpio32, debounce);
		gpio_set_debounce(int11_gpio43, debounce);
		gpio_set_debounce(int16_gpio48, debounce);
		gpio_set_debounce(int17_gpio49, debounce);
		
		tp_irq = irq_of_parse_and_map(node, 0);
		printk("tp_probe_irq = %d\n!", tp_irq);	
                  ret = request_irq(tp_irq, (irq_handler_t) tp_eint8_interrupt_handler, IRQF_TRIGGER_FALLING,
					"touchsensor-eint", NULL);
				  
		  tp_irq = irq_of_parse_and_map(node, 1);
		printk("tp_probe_irq = %d\n!", tp_irq);	
                  ret = request_irq(tp_irq, (irq_handler_t) tp_eint9_interrupt_handler, IRQF_TRIGGER_FALLING,
					"touchsensor-eint", NULL);

		tp_irq = irq_of_parse_and_map(node, 2);
		printk("tp_probe_irq = %d\n!", tp_irq);	
                  ret = request_irq(tp_irq, (irq_handler_t) tp_eint10_interrupt_handler, IRQF_TRIGGER_FALLING,
					"touchsensor-eint", NULL);

		 tp_irq = irq_of_parse_and_map(node, 3);
		printk("tp_probe_irq = %d\n!", tp_irq);	
                  ret = request_irq(tp_irq, (irq_handler_t) tp_eint11_interrupt_handler, IRQF_TRIGGER_FALLING,
					"touchsensor-eint", NULL);

		tp_irq = irq_of_parse_and_map(node, 4);
		printk("tp_probe_irq = %d\n!", tp_irq);	
                  ret = request_irq(tp_irq, (irq_handler_t) tp_eint16_interrupt_handler, IRQF_TRIGGER_FALLING,
					"touchsensor-eint", NULL);

		  tp_irq = irq_of_parse_and_map(node, 5);
		printk("tp_probe_irq = %d\n!", tp_irq);	
                  ret = request_irq(tp_irq, (irq_handler_t) tp_eint17_interrupt_handler, IRQF_TRIGGER_FALLING,
					"touchsensor-eint", NULL);
		if (ret > 0)
			{
				ret = -1;
				printk("tp_probe request_irq IRQ LINE NOT AVAILABLE!ret = %d.",ret);
			}

		}


	/*********************************************************/
	touchsensor_input_dev = input_allocate_device();	//分配设备
	__set_bit(EV_SYN, touchsensor_input_dev->evbit);  //注册设备支持event类型
	__set_bit(EV_KEY, touchsensor_input_dev->evbit);
	
	__set_bit(KEY_VOLUMEUP, touchsensor_input_dev->keybit);   
	__set_bit(KEY_VOLUMEDOWN, touchsensor_input_dev->keybit);  
	//__set_bit(KEY_PLAYPAUSE, touchsensor_input_dev->keybit);  
	
	touchsensor_input_dev->name ="TouchSensor";//
	err = input_register_device(touchsensor_input_dev);				
	if (err) {
		printk( "TouchSensor register input device failed (%d)\n", err);
		input_free_device(touchsensor_input_dev);
		return err;
	}
/*********************************************************/

#ifdef CONFIG_Y50_TOUCHSENSOR		
	hrtimer_init(&dance_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	dance_timer.function = commit_delay_hrtimer_func;
	hrtimer_start(&dance_timer, ktime_set(1, 0), HRTIMER_MODE_REL);
#endif
//---------audio timer---------------------------------
	hrtimer_init(&audio_stop_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	audio_stop_timer.function = commit_audio_hrtimer_func;
	//hrtimer_start(&audio_stop_timer, ktime_set(1, 0), HRTIMER_MODE_REL);
//--------------------------------------------------

//----------------------dual touch---------------
		hrtimer_init(&touch_time, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
		touch_time.function = double_touch_commit;
//--------------------------------------------
	printk("tp_probe====\n");
	return 0;
}

#ifdef CONFIG_OF
static const struct of_device_id TP_of_match[] = {
	{.compatible = "mediatek,touchsensor"},
	{},
};
#endif

static struct platform_driver tp_platform_driver = {
	.probe =tp_probe,
	
	.driver = {
		   .name = "touchsensor",
		   .owner = THIS_MODULE,
#ifdef CONFIG_OF
		   .of_match_table = TP_of_match,
#endif
	},
};

static int __init touchsensor_init(void)
{	
	platform_driver_register(&tp_platform_driver);

	printk("touchsensor   init    finish-------------\n");
	return  0;
}

static void __exit touchsensor_exit(void)
{
	
}	
	
late_initcall(touchsensor_init);
module_exit(touchsensor_exit);

MODULE_AUTHOR("Yongyida");
MODULE_DESCRIPTION("motor driver");
MODULE_LICENSE("GPL");
