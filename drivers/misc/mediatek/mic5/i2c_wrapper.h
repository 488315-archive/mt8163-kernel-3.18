#ifndef I2C_WRAPPER_H_
#define I2C_WRAPPER_H_


#define I2C_MAX_TRY   1

#define I2C_WRP_DEBUG

#ifdef I2C_WRP_DEBUG

#define I2C_WRP_TAG			"[i2c_wrapper]"
#define I2C_WRP_LOG(fmt, arg...)	printk(KERN_ERR I2C_WRP_TAG fmt, ##arg)
#define I2C_WRP_MSG(fmt, arg...)	printk(fmt, ##arg)
#define I2C_WRP_ERR(fmt, arg...)	printk(KERN_ERR I2C_WRP_TAG "ERROR,%s, %d: "fmt"\n", __FUNCTION__, __LINE__, ##arg)
#define I2C_WRP_FUNC(fmt, arg...) printk(I2C_WRP_TAG "%s\n", __FUNCTION__)

#else

#define I2C_WRP_TAG			"[i2c_wrapper]"
#define I2C_WRP_LOG(fmt, arg...)		//		
#define I2C_WRP_MSG(fmt, arg...)		
#define I2C_WRP_ERR(fmt, arg...)	
#define I2C_WRP_FUNC(fmt, arg...)	
#endif
typedef struct i2c_regdata{
	u8 reg;
	u8 data;
}I2C_REGDATA_T;

typedef struct i2c_gpio{
	u32 sda;
	u32 scl;
	u32	addr;
	u32 speed;
}I2C_GPIO_T;

void delay_nop_1us(u16 wTime);
#if 0				
kal_bool wrapper_i2c_write_reg_org(struct i2c_client *client, u8 regdata[], u8 length);
kal_bool wrapper_i2c_write_regdata(struct i2c_client *client, u8 reg,u8 data);
kal_bool wrapper_i2c_write_data(struct i2c_client *client, u8 data[], u8 length);
kal_bool wrapper_i2c_write_regs(struct i2c_client *client, I2C_REGDATA_T regsdata[],u8 length);
kal_bool wrapper_i2c_read_regdata(struct i2c_client *client, u8 regaddr, u8 *data, u8 length);
kal_bool wrapper_i2c_read_data(struct i2c_client *client, u8 *data, u8 length);
kal_bool i2c_write_reg_org(I2C_GPIO_T *dev, u8 regdata[],u8 length);
kal_bool i2c_write_regdata(I2C_GPIO_T *dev, u8 reg,u8 data);
kal_bool i2c_write_regs(I2C_GPIO_T *dev, I2C_REGDATA_T regsdata[],u8 length);


#endif

void i2c_init_gpio(I2C_GPIO_T *dev);

uint8_t i2c_write_buf(I2C_GPIO_T *dev, u8 data[],u8 len);
uint8_t i2c_read_reg_org(I2C_GPIO_T *dev, u8 Read_regaddr[],u8 Read_regaddr_length,u8 ret_buf[],u8 read_len);
void i2c_start(I2C_GPIO_T *dev);
void i2c_stop(I2C_GPIO_T *dev);
unsigned char i2c_read_byte(I2C_GPIO_T *dev,bool ack);
bool i2c_transfer_byte(I2C_GPIO_T *dev, u8 data);


#endif /* I2C_WRAPPER_H_ */
