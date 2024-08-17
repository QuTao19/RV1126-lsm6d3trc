#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/ide.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/errno.h>
#include <linux/gpio.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/of_gpio.h>
#include <linux/semaphore.h>
#include <linux/timer.h>
#include <linux/i2c.h>
#include <asm/mach/map.h>
#include <asm/uaccess.h>
#include <asm/io.h>
#include <linux/fs.h>
#include "lsm6ds3trc.h"

#define LSM6DS3TRC_CNT 1
#define LSM6DS3TRC_NAME "lsm6ds3trc"

struct lsm6ds3trc_dev {
    dev_t devid;            // 设备号
    struct cdev cdev;       // cdev
    struct class *class;    // 类 
    struct device *device;  // 设备
    struct device_node *nd; // 设备节点
    int major;              // 主设备号
    void *private_data;     // 私有数据
	u8 acc_data[6];      // 加速度数据
	u8 gyr_data[6];      // 陀螺仪数据
	u8 temp_data[2];        // 温度数据
};
static struct lsm6ds3trc_dev lsm6ds3trcdev;

/* 
 * @function  : 从lsm6ds3tr读取多个寄存器数据
 * @param dev : lsm6ds3tr设备
 * @param reg : 要读取的寄存器首地址
 * @param val : 读取的数据
 * @param len : 读取数据长度
 * @return 	  : 操作结果
 */
static int lsm6ds3trc_read_regs(struct lsm6ds3trc_dev *dev, u8 reg, void *val, int len)
{
    int ret;
    struct i2c_msg msg[2];
    struct i2c_client *client = (struct i2c_client *)dev->private_data;

    /* msg[0]为发送要读取的首地址 */
    msg[0].addr = client->addr;     // ls6ds3tr的地址
    msg[0].flags = 0;               // 标记为发送数据
    msg[0].buf = &reg;              // 要读取的首地址
    msg[0].len = 1;                 // reg长度

    /* msg[1]为读取的数据 */
    msg[1].addr = client->addr;     // ls6ds3tr的地址
    msg[1].flags = I2C_M_RD;        // 标记为读取数据
    msg[1].buf = val;               // 读取的数据
    msg[1].len = len;               // 读取数据长度

    ret = i2c_transfer(client->adapter, msg, 2);
    if(ret == 2) {
		ret = 0;
	} else {
		printk("i2c rd failed=%d reg=%06x len=%d\n",ret, reg, len);
		ret = -EREMOTEIO;
	}
	return ret;
}

/* 
 * @function  : 向lsm6ds3tr写入多个寄存器数据
 * @param dev : lsm6ds3tr设备
 * @param reg : 要写入的寄存器首地址
 * @param val : 写入的数据
 * @param len : 写入的数据长度
 * @return    ：操作结果
 */
static s32 lsm6ds3trc_write_regs(struct lsm6ds3trc_dev *dev, u8 reg, u8 *buf, u8 len)
{
    u8 b[256];
    struct i2c_msg msg;
    struct i2c_client *client = (struct i2c_client *)dev->private_data;

    b[0] = reg;                 // 寄存器首地址
    memcpy(&b[1], buf, len);    // 将要写入的数据buf拷贝到数组b中

    msg.addr = client->addr;    // lsm6ds3tr的地址
    msg.flags = 0;              // 标记为写数据

    msg.buf = b;                // 要写入的数据
    msg.len = len + 1;          // 要写入的数据长度

    return i2c_transfer(client->adapter, &msg, 1);
}

/* 
 * @function  : 读取lsm6ds3tr指定寄存器的指定值
 * @param dev : lsm6ds3tr设备
 * @param reg : 要读取的寄存器地址
 * @return    : 读取到的数据
 */
static unsigned char lsm6ds3trc_read_reg(struct lsm6ds3trc_dev *dev, u8 reg)
{
    u8 data = 0;

	lsm6ds3trc_read_regs(dev, reg, &data, 1);
	return data;

#if 0
	struct i2c_client *client = (struct i2c_client *)dev->private_data;
	return i2c_smbus_read_byte_data(client, reg);
#endif

}

/* 
 * @function  : 向lsm6ds3tr指定寄存器写入指定值
 * @param dev : lsm6ds3tr设备
 * @param reg : 要写入的寄存器
 * @param data : 写入的数据
 * @return    : 无
 */
static void lsm6ds3trc_write_reg(struct lsm6ds3trc_dev *dev, u8 reg, u8 data)
{
    u8 buf = 0;
	buf = data;
	lsm6ds3trc_write_regs(dev, reg, &buf, 1);
}

/* 寄存器设置函数 */
bool lsm6ds3trc_getChipID(void)
{
	u8 buf = 0;
	buf = lsm6ds3trc_read_reg(&lsm6ds3trcdev, LSM6DS3TRC_WHO_AM_I);
	if (buf == 0x6a) 
		return true;
	else 
		return false;
}

void lsm6ds3trc_reset(void)
{
	u8 buf[1] = {0};

	buf[0] = 0x80;
	lsm6ds3trc_write_reg(&lsm6ds3trcdev, LSM6DS3TRC_CTRL3_C, buf[0]);
	mdelay(15);

	/* 复位 */
	buf[0] = lsm6ds3trc_read_reg(&lsm6ds3trcdev, LSM6DS3TRC_CTRL3_C);
	buf[0] |= 0x01;
	lsm6ds3trc_write_reg(&lsm6ds3trcdev, LSM6DS3TRC_CTRL3_C, buf[0]);
	while (buf[0] & 0x01) {
		buf[0] = lsm6ds3trc_read_reg(&lsm6ds3trcdev, LSM6DS3TRC_CTRL3_C);
	}
 
}

void lsm6ds3trc_setBDU(bool flag)
{
	u8 buf[1] = {0};
	buf[0] = lsm6ds3trc_read_reg(&lsm6ds3trcdev, LSM6DS3TRC_CTRL3_C);

	if (flag == true)
	{
		buf[0] |= 0x40;
		lsm6ds3trc_write_reg(&lsm6ds3trcdev, LSM6DS3TRC_CTRL3_C, buf[0]);
	} else {
		buf[0] &= 0xbf;
		lsm6ds3trc_write_reg(&lsm6ds3trcdev, LSM6DS3TRC_CTRL3_C, buf[0]);
	}

	buf[0] = lsm6ds3trc_read_reg(&lsm6ds3trcdev, LSM6DS3TRC_CTRL3_C);

}

void lsm6ds3trc_set_acc_rate(u8 rate)
{
	u8 buf[1] = {0};
	buf[0] = lsm6ds3trc_read_reg(&lsm6ds3trcdev, LSM6DS3TRC_CTRL1_XL);
	buf[0] |= rate;
	lsm6ds3trc_write_reg(&lsm6ds3trcdev, LSM6DS3TRC_CTRL1_XL, buf[0]);
}

void lsm6ds3trc_set_acc_scale(u8 scale)
{
	u8 buf[1] = {0};
	buf[0] = lsm6ds3trc_read_reg(&lsm6ds3trcdev, LSM6DS3TRC_CTRL1_XL);
	buf[0] |= scale;
	lsm6ds3trc_write_reg(&lsm6ds3trcdev, LSM6DS3TRC_CTRL1_XL, buf[0]);
}

void lsm6ds3trc_set_acc_bw(u8 bw, u8 ODR)
{
	u8 buf[1] = {0};
	buf[0] = lsm6ds3trc_read_reg(&lsm6ds3trcdev, LSM6DS3TRC_CTRL1_XL);
	buf[0] |= bw;
	lsm6ds3trc_write_reg(&lsm6ds3trcdev, LSM6DS3TRC_CTRL1_XL, buf[0]);

	buf[0] = lsm6ds3trc_read_reg(&lsm6ds3trcdev, LSM6DS3TRC_CTRL8_XL);
	buf[0] |= ODR;
	lsm6ds3trc_write_reg(&lsm6ds3trcdev, LSM6DS3TRC_CTRL8_XL, buf[0]);
}

void lsm6ds3trc_set_gyr_rate(u8 rate)
{
	u8 buf[1] = {0};
	buf[0] = lsm6ds3trc_read_reg(&lsm6ds3trcdev, LSM6DS3TRC_CTRL2_G);
	buf[0] |= rate;
	lsm6ds3trc_write_reg(&lsm6ds3trcdev, LSM6DS3TRC_CTRL2_G, buf[0]);
}

void lsm6ds3trc_set_gyr_scale(u8 scale)
{
	u8 buf[1] = {0};
	buf[0] = lsm6ds3trc_read_reg(&lsm6ds3trcdev, LSM6DS3TRC_CTRL2_G);
	buf[0] |= scale;
	lsm6ds3trc_write_reg(&lsm6ds3trcdev, LSM6DS3TRC_CTRL2_G, buf[0]);
}

void lsm6ds3trc_set_reg7(u8 reg7)
{
	u8 buf[1] = {0};
	buf[0] = lsm6ds3trc_read_reg(&lsm6ds3trcdev, LSM6DS3TRC_CTRL7_G);
	buf[0] |= reg7;
	lsm6ds3trc_write_reg(&lsm6ds3trcdev, LSM6DS3TRC_CTRL7_G, buf[0]);
}

void lsm6ds3trc_set_reg6(u8 reg6)
{
	u8 buf[1] = {0};
	buf[0] = lsm6ds3trc_read_reg(&lsm6ds3trcdev, LSM6DS3TRC_CTRL6_C);
	buf[0] |= reg6;
	lsm6ds3trc_write_reg(&lsm6ds3trcdev, LSM6DS3TRC_CTRL6_C, buf[0]);
}

void lsm6ds3trc_set_reg4(u8 reg4)
{
	u8 buf[1] = {0};
	buf[0] = lsm6ds3trc_read_reg(&lsm6ds3trcdev, LSM6DS3TRC_CTRL4_C);
	buf[0] |= reg4;
	lsm6ds3trc_write_reg(&lsm6ds3trcdev, LSM6DS3TRC_CTRL4_C, buf[0]);
}

u8 lsm6ds3trc_get_status(void)
{
	u8 buf = 0;
	buf = lsm6ds3trc_read_reg(&lsm6ds3trcdev, LSM6DS3TRC_STATUS_REG);
	return buf;
}

/* 寄存器初始配置 */
bool lsm6ds3trc_reg_init(void)
{
	if (lsm6ds3trc_getChipID() == false) {
		return false;
	} else {
		lsm6ds3trc_reset();
		lsm6ds3trc_setBDU(true);

		/* 加速度速率量程设置 */
		lsm6ds3trc_set_acc_rate(LSM6DS3TRC_ACC_RATE_833HZ);
		lsm6ds3trc_set_acc_scale(LSM6DS3TRC_ACC_FSXL_2G);

		/* 角速度速率量程设置 */
		lsm6ds3trc_set_gyr_rate(LSM6DS3TRC_GYR_RATE_833HZ);
		lsm6ds3trc_set_gyr_scale(LSM6DS3TRC_GYR_FSG_245);

		/* 加速度带宽设置 */
		lsm6ds3trc_set_acc_bw(LSM6DS3TRC_ACC_BW0XL_400HZ, LSM6DS3TRC_ACC_LOW_PASS_ODR_100);

		/* 角速度带宽设置 */
		lsm6ds3trc_set_reg7(LSM6DS3TRC_CTRL7_G_HP_EN_ENABLE | LSM6DS3TRC_CTRL7_G_HPM_260MHZ);
		lsm6ds3trc_set_reg6(LSM6DS3TRC_CTRL6_C_FTYPE_1);
		lsm6ds3trc_set_reg4(LSM6DS3TRC_CTRL4_LPF1_SELG_ENABLE);
		return true;
	}
}

/* 获取加速度原始数据 */
void lsm6ds3trc_get_acc_data(struct lsm6ds3trc_dev *dev, u8 *acc_data)
{

	u8 buf[6];

	lsm6ds3trc_read_regs(dev, LSM6DS3TRC_OUTX_L_XL, &buf, 6);
	acc_data[0] = buf[0];	/* X LS */
	acc_data[1] = buf[1];	/* X MS */
	acc_data[2] = buf[2];	/* Y LS */
	acc_data[3] = buf[3];	/* Y MS */
	acc_data[4] = buf[4];	/* Z LS */
	acc_data[5] = buf[5];	/* Z MS */

	printk("acc_data[0] = %d, acc_data[1] = %d, acc_data[2] = %d\n", buf[0], buf[1], buf[2]);

#if 0
	switch (scale)
	{
	case LSM6DS3TRC_ACC_FSXL_2G:
		acc_float[0] = ((float)acc[0] * 0.061f);
		acc_float[1] = ((float)acc[1] * 0.061f);
		acc_float[2] = ((float)acc[2] * 0.061f);
		break;

	case LSM6DS3TRC_ACC_FSXL_16G:
		acc_float[0] = ((float)acc[0] * 0.488f);
		acc_float[1] = ((float)acc[1] * 0.488f);
		acc_float[2] = ((float)acc[2] * 0.488f);
		break;

	case LSM6DS3TRC_ACC_FSXL_4G:
		acc_float[0] = ((float)acc[0] * 0.122f);
		acc_float[1] = ((float)acc[1] * 0.122f);
		acc_float[2] = ((float)acc[2] * 0.122f);
		break;

	case LSM6DS3TRC_ACC_FSXL_8G:
		acc_float[0] = ((float)acc[0] * 0.244f);
		acc_float[1] = ((float)acc[1] * 0.244f);
		acc_float[2] = ((float)acc[2] * 0.244f);
		break;
	}
#endif
}

void lsm6ds3trc_get_gyr_data(struct lsm6ds3trc_dev *dev, u8 *gyr_data)
{
	u8 buf[6];

	lsm6ds3trc_read_regs(dev, LSM6DS3TRC_OUTX_L_G, &buf, 6);
	gyr_data[0] = buf[0];	/* X LS */
	gyr_data[1] = buf[1];	/* X MS */
	gyr_data[2] = buf[2];	/* Y LS */
	gyr_data[3] = buf[3];	/* Y MS */
	gyr_data[4] = buf[4];	/* Z LS */
	gyr_data[5] = buf[5];	/* Z MS */

	printk("gyr_data[0] = %d, gyr_data[1] = %d, gyr_data[2] = %d\n", buf[0], buf[1], buf[2]);

#if 0
	switch (scale)
	{
		case LSM6DS3TRC_GYR_FSG_245:
			gyr_data[0] = (float)gyr[0] * 8.750f;
			gyr_data[1] = (float)gyr[1] * 8.750f;
			gyr_data[2] = (float)gyr[2] * 8.750f;
			break;
		
		case LSM6DS3TRC_GYR_FSG_500:
			gyr_data[0] = (float)gyr[0] * 17.50f;
			gyr_data[1] = (float)gyr[1] * 17.50f;
			gyr_data[2] = (float)gyr[2] * 17.50f;
			break;

		case LSM6DS3TRC_GYR_FSG_1000:
			gyr_data[0] = (float)gyr[0] * 35.00f;
			gyr_data[1] = (float)gyr[1] * 35.00f;
			gyr_data[2] = (float)gyr[2] * 35.00f;
			break;

		case LSM6DS3TRC_GYR_FSG_2000:
			gyr_data[0] = (float)gyr[0] * 70.00f;
			gyr_data[1] = (float)gyr[1] * 70.00f;
			gyr_data[2] = (float)gyr[2] * 70.00f;
			break;
	}
#endif
}

void lsm6ds3trc_get_temp_data(struct lsm6ds3trc_dev *dev,u8 *temp_data)
{
	u8 buf[2];

	lsm6ds3trc_read_regs(dev, LSM6DS3TRC_OUT_TEMP_L, &buf, 2);
	temp_data[0] = buf[0];
	temp_data[1] = buf[1];
}

/* 
 * @function  : 读取lsm6ds3tr的数据
 * @return    : 无
 */
void lsm6ds3trc_readdata(struct lsm6ds3trc_dev *dev)
{
	u8 status;
	status = lsm6ds3trc_get_status();

	/* 加速度数据更新 */
	if (status & LSM6DS3TRC_STATUS_ACCELEROMETER)
		lsm6ds3trc_get_acc_data(dev, dev->acc_data);
	if (status & LSM6DS3TRC_STATUS_GYROSCOPE)
		lsm6ds3trc_get_gyr_data(dev, dev->gyr_data);
	if (status & LSM6DS3TRC_STATUS_TEMPERATURE)
		lsm6ds3trc_get_temp_data(dev, dev->temp_data);

}

/* 寄存器初始设置 */
static int lsm6ds3trc_open(struct inode *inode, struct file *filp)
{
	bool val;
	filp->private_data = &lsm6ds3trcdev;

	val = lsm6ds3trc_reg_init();
	if (val == false) 
		return val;

    return 0;
}

static ssize_t lsm6ds3trc_read(struct file *filp, char __user *buf, size_t cnt, loff_t *off)
{
	struct lsm6ds3trc_dev *dev = (struct lsm6ds3trc_dev *)filp->private_data;

	u8 data[14];
	long err = 0;

	lsm6ds3trc_readdata(dev);
	
	/* 加速度数据 */
	data[0] = dev->acc_data[0];
	data[1] = dev->acc_data[1];
	data[2] = dev->acc_data[2];
	data[3] = dev->acc_data[3];
	data[4] = dev->acc_data[4];
	data[5] = dev->acc_data[5];

	/* 角速度数据 */
	data[6] = dev->gyr_data[0];
	data[7] = dev->gyr_data[1];
	data[8] = dev->gyr_data[2];
	data[9] = dev->gyr_data[3];
	data[10] = dev->gyr_data[4];
	data[11] = dev->gyr_data[5];

	/* 温度数据 */
	data[12] = dev->temp_data[0];
	data[13] = dev->temp_data[1];


	//printk("acc_data[0] = %d, acc_data[1] = %d, acc_data[2] = %d\n", data[0], data[1], data[2]);

#if 0 
	data[3] = dev->gyr_data[0];
	data[4] = dev->gyr_data[1];
	data[5] = dev->gyr_data[2];

	data[6] = dev->temp_data;
#endif
	err = copy_to_user(buf, data, sizeof(data));

    return 0;
}

static int lsm6ds3trc_release(struct inode *inode, struct file *filp)
{
	return 0;
}

/* lsm6ds3tr设备操作函数 */
static const struct file_operations lsm6ds3trc_fops = {
	.owner = THIS_MODULE,
	.open = lsm6ds3trc_open,
	.read = lsm6ds3trc_read,
	.release = lsm6ds3trc_release,
};

/* lsm6ds3tr设备注册 */
static int lsm6ds3trc_dev_init(void)
{
	/* 1、构建设备号 */
	if (lsm6ds3trcdev.major) {
		lsm6ds3trcdev.devid = MKDEV(lsm6ds3trcdev.major, 0);
		register_chrdev_region(lsm6ds3trcdev.devid, LSM6DS3TRC_CNT, LSM6DS3TRC_NAME);
	} else {
		alloc_chrdev_region(&lsm6ds3trcdev.devid, 0, LSM6DS3TRC_CNT, LSM6DS3TRC_NAME);
		lsm6ds3trcdev.major = MAJOR(lsm6ds3trcdev.devid);
	}

	/* 2、注册设备 */
	cdev_init(&lsm6ds3trcdev.cdev, &lsm6ds3trc_fops);
	cdev_add(&lsm6ds3trcdev.cdev, lsm6ds3trcdev.devid, LSM6DS3TRC_CNT);

	/* 3、创建类 */
	lsm6ds3trcdev.class = class_create(THIS_MODULE, LSM6DS3TRC_NAME);
	if (IS_ERR(lsm6ds3trcdev.class)) {
		return PTR_ERR(lsm6ds3trcdev.class);
	}

	/* 4、创建设备 */
	lsm6ds3trcdev.device = device_create(lsm6ds3trcdev.class, NULL, lsm6ds3trcdev.devid, NULL, LSM6DS3TRC_NAME);
	if (IS_ERR(lsm6ds3trcdev.device)) {
		return PTR_ERR(lsm6ds3trcdev.device);
	}

	return 0;
}

/* i2c驱动probe函数，设备匹配后执行 */
static int lsm6ds3trc_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int ret = 0;

	/* 设备初始化 */
    ret = lsm6ds3trc_dev_init();
	if (ret < 0) {
		printk("lsm6ds3trc_dev_init failed!\n");
		return ret;
	}

	lsm6ds3trcdev.private_data = client;

	return 0;
}

/* 移除i2c驱动执行 */
static int lsm6ds3trc_remove(struct i2c_client *client)
{
    /* 删除设备 */
	cdev_del(&lsm6ds3trcdev.cdev);
	unregister_chrdev_region(lsm6ds3trcdev.devid, LSM6DS3TRC_CNT);

	/* 注销掉类和设备 */
	device_destroy(lsm6ds3trcdev.class, lsm6ds3trcdev.devid);
	class_destroy(lsm6ds3trcdev.class);
	return 0;
}

/* 传统匹配方式ID列表 */
static const struct i2c_device_id lsm6ds3trc_id[] = {
	{"lsm6ds3tr-c", 0},  
	{}
};

/* 设备树匹配列表 */
static const struct of_device_id lsm6ds3trc_of_match[] = {
	{ .compatible = "st,lsm6ds3tr-c" },
	{}
};

/* i2c驱动结构体 */
static struct i2c_driver lsm6ds3trc_driver = {
	.probe = lsm6ds3trc_probe,
	.remove = lsm6ds3trc_remove,
	.driver = {
			.owner = THIS_MODULE,
		   	.name = "lsm6ds3tr-c",
		   	.of_match_table = lsm6ds3trc_of_match, 
		   },
	.id_table = lsm6ds3trc_id,
};

static int __init lsm6ds3trc_init(void)
{
    int ret = 0;

	ret = i2c_add_driver(&lsm6ds3trc_driver);
	return ret;
}

static void __exit lsm6ds3trc_exit(void)
{
    i2c_del_driver(&lsm6ds3trc_driver);
}

module_init(lsm6ds3trc_init);
module_exit(lsm6ds3trc_exit);
MODULE_LICENSE("GPL");
MODULE_AUTHOR("qutao");