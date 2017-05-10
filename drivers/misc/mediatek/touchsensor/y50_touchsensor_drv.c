#include <linux/switch.h>

#include <mach/gpio_const.h>
#include <cust_eint.h>
#include <cust_gpio_usage.h>
#include <mach/mt_gpio.h>
#include <mach/eint.h>

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

#include <linux/hrtimer.h>
#include <linux/jiffies.h>


#define TOUCH  1
#define NO_TOUCH  0
#define TP_NAME  "touchsensor"
static char debuf[10];

static struct switch_dev touchsensor_dev;
static struct hrtimer timer;
static struct hrtimer dance_timer;
static unsigned long left_time,right_time;
static struct task_struct *thread = NULL;

extern void mt_eint_mask(unsigned int eint_num);
extern void mt_eint_unmask(unsigned int eint_num);
extern void mt_eint_set_hw_debounce(unsigned int eint_num, unsigned int ms);
extern void mt_eint_set_polarity(unsigned int eint_num, unsigned int pol);
extern unsigned int mt_eint_set_sens(unsigned int eint_num, unsigned int sens);
extern void mt_eint_registration(unsigned int eint_num, unsigned int flow, void (EINT_FUNC_PTR)(void), unsigned int is_auto_umask);
extern void mt_eint_print_status(void);
extern void aw2013_breath_all(int led0,int led1,int led2);


static void commit_status(char *switch_name)
{
	touchsensor_dev.name=switch_name;
	
	switch_set_state(&touchsensor_dev, TOUCH);
	
	touchsensor_dev.state=NO_TOUCH;
	touchsensor_dev.name = TP_NAME;

	printk("touchsensor     commit_status--------%s----\n",switch_name);
}

static void touchsersor_eint3_func(void)
{

	commit_status("head");	//head
 	mt_eint_unmask(CUST_EINT_HALL_1_NUM);
}
static void touchsersor_eint4_func(void)
{
	commit_status("back");    //back
 	mt_eint_unmask(CUST_EINT_HALL_2_NUM);
}
static void touchsersor_eint11_func(void)
{
	right_time=jiffies;
	commit_status("right");	//right
	mt_eint_unmask(CUST_EINT_HALL_3_NUM);
}
static void touchsersor_eint12_func(void)
{	
	left_time=jiffies;
	commit_status("left");  //left
	mt_eint_unmask(CUST_EINT_HALL_4_NUM);
}
static void touchsersor_eint0_func(void)
{
	commit_status("pir");
	mt_eint_unmask(CUST_EINT_CHR_STAT_NUM);
}

enum hrtimer_restart commit_delay_hrtimer_func(struct hrtimer *timer)
{
	if((jiffies -right_time) >4*HZ)right_time=0;
	if((jiffies -left_time) >4*HZ)left_time=0;

	if( (jiffies -right_time) >3*HZ && (jiffies -right_time) <4*HZ && (jiffies -left_time) >3*HZ && (jiffies -left_time) <4*HZ && abs(left_time-right_time)<10)	
		commit_status("dance");
	hrtimer_forward_now(&dance_timer, ktime_set(1, 0));	
	

	return HRTIMER_RESTART;
}

enum hrtimer_restart delay_hrtimer_func(struct hrtimer *timer)
{
	

		mt_set_gpio_dir((GPIO3 | 0x80000000), GPIO_DIR_IN);
		mt_set_gpio_mode((GPIO3| 0x80000000), GPIO_MODE_00);
		mt_set_gpio_pull_enable((GPIO3 | 0x80000000), FALSE);
						
		mt_eint_set_hw_debounce(CUST_EINT_HALL_1_NUM, 100);
		mt_eint_registration(CUST_EINT_HALL_1_NUM, CUST_EINTF_TRIGGER_FALLING, touchsersor_eint3_func, 0);			
		mt_eint_unmask(CUST_EINT_HALL_1_NUM);

		mt_set_gpio_dir((GPIO4 | 0x80000000), GPIO_DIR_IN);
		mt_set_gpio_mode((GPIO4 | 0x80000000), GPIO_MODE_00);
		mt_set_gpio_pull_enable((GPIO4 | 0x80000000), FALSE);
						
		mt_eint_set_hw_debounce(CUST_EINT_HALL_2_NUM, 100);
		mt_eint_registration(CUST_EINT_HALL_2_NUM, CUST_EINTF_TRIGGER_FALLING, touchsersor_eint4_func, 0);			
		mt_eint_unmask(CUST_EINT_HALL_2_NUM);

		mt_set_gpio_dir((GPIO11 | 0x80000000), GPIO_DIR_IN);
		mt_set_gpio_mode((GPIO11 | 0x80000000), GPIO_MODE_00);
		mt_set_gpio_pull_enable((GPIO11 | 0x80000000), FALSE);
						
		mt_eint_set_hw_debounce(CUST_EINT_HALL_3_NUM, 100);
		mt_eint_registration(CUST_EINT_HALL_3_NUM, CUST_EINTF_TRIGGER_FALLING, touchsersor_eint11_func, 0); 			
		mt_eint_unmask(CUST_EINT_HALL_3_NUM);

		mt_set_gpio_dir((GPIO12 | 0x80000000), GPIO_DIR_IN);
		mt_set_gpio_mode((GPIO12 | 0x80000000), GPIO_MODE_00);
		mt_set_gpio_pull_enable((GPIO12 | 0x80000000), FALSE);
						
		mt_eint_set_hw_debounce(CUST_EINT_HALL_4_NUM, 100);
		mt_eint_registration(CUST_EINT_HALL_4_NUM, CUST_EINTF_TRIGGER_FALLING, touchsersor_eint12_func, 0); 			
		mt_eint_unmask(CUST_EINT_HALL_4_NUM);

		mt_set_gpio_dir((GPIO0 | 0x80000000), GPIO_DIR_IN);
		mt_set_gpio_mode((GPIO0 | 0x80000000), GPIO_MODE_00);
		mt_set_gpio_pull_enable((GPIO0 | 0x80000000), FALSE);
						
		mt_eint_set_hw_debounce(CUST_EINT_CHR_STAT_NUM, 100);
		mt_eint_registration(CUST_EINT_CHR_STAT_NUM, CUST_EINTF_TRIGGER_FALLING, touchsersor_eint0_func, 0);			
		mt_eint_unmask(CUST_EINT_CHR_STAT_NUM);

	ktime_t ktime;
	ktime = ktime_set(1, 0);	
	hrtimer_init(&dance_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	dance_timer.function = commit_delay_hrtimer_func;
	hrtimer_start(&dance_timer, ktime, HRTIMER_MODE_REL);

	printk("delay_hrtimer_func	finish-------------\n");

	return HRTIMER_NORESTART;
}


static int __init touchsersor_init(void)
{

	//int error;
	touchsensor_dev.name = TP_NAME;
	touchsensor_dev.index = 0;
	touchsensor_dev.state = 0;
	
	switch_dev_register(&touchsensor_dev);

	 mt_set_gpio_mode((GPIO78 | 0x80000000), GPIO_MODE_00); 
  	 mt_set_gpio_dir((GPIO78 | 0x80000000), GPIO_DIR_OUT) ;
	 mt_set_gpio_out((GPIO78 | 0x80000000), 1);	

				
				
	ktime_t ktime;

	ktime = ktime_set(25, 0);	 //60s later  register ext_int
	hrtimer_init(&timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	timer.function = delay_hrtimer_func;
	hrtimer_start(&timer, ktime, HRTIMER_MODE_REL);
				
	printk("touchsensor       init      finish-------------\n");
	return  0;
}

static void __exit touchsersor_exit(void)
{
	 switch_dev_unregister(&touchsensor_dev);
}	
	
module_init(touchsersor_init);
module_exit(touchsersor_exit);

//MODULE_AUTHOR("Yongyida");
//MODULE_DESCRIPTION("motor driver");
//MODULE_LICENSE("GPL");

//MODULE_AUTHOR("yongyida@yongyida.com>");
//MODULE_DESCRIPTION("touchsersor driver");
//MODULE_LICENSE("GPL");
