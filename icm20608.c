#include <linux/module.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/io.h>
#include <linux/device.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/types.h>
#include <linux/ide.h>
#include <linux/irq.h>
#include <linux/of_irq.h>
#include <linux/platform_device.h>
#include <linux/miscdevice.h>
#include <linux/input.h>
#include <linux/delay.h>
#include <linux/spi/spi.h>
#include "icm20608_reg.h"

#define DEVICE_CNT      1
#define DEVICE_NAME     "icm20608"

struct icm20608_device {
        int major;
        int minor;
        dev_t devid;
        struct cdev cdev;
        struct class *class;
        struct device *device;
        struct device_node *nd;
        void *private_data;
        int cs_gpio;            /* 片选所使用的GPIO编号 */
        signed int gyro_x_adc;  /* 陀螺仪X轴原始值 */
        signed int gyro_y_adc;  /* 陀螺仪Y轴原始值 */
        signed int gyro_z_adc;  /* 陀螺仪Z轴原始值 */
        signed int accel_x_adc; /* 加速度计X轴原始值 */
        signed int accel_y_adc; /* 加速度计Y轴原始值 */
        signed int accel_z_adc; /* 加速度计Z轴原始值 */
        signed int temp_adc;    /* 温度原始值 */
};
static struct icm20608_device icm20608_dev;

static void icm20608_readdata(struct icm20608_device *dev);
static int icm20608_open(struct inode *inode, struct file *filp);
static ssize_t icm20608_read(struct file *filp, char __user *buf, size_t cnt, loff_t *off);
static int icm20608_release(struct inode *inode, struct file *filp);

static struct file_operations ops = {
        .owner = THIS_MODULE,
        .open = icm20608_open,
        .read = icm20608_read,
        .release = icm20608_release,
};

static int icm20608_open(struct inode *inode, struct file *filp)
{
        filp->private_data = &icm20608_dev;
        return 0;
}

static ssize_t icm20608_read(struct file *filp, char __user *buf, size_t cnt, loff_t *off)
{
        int data[7];
	long err = 0;
	struct icm20608_device *dev = (struct icm20608_device *)filp->private_data;

	icm20608_readdata(dev);
	data[0] = dev->gyro_x_adc;
	data[1] = dev->gyro_y_adc;
	data[2] = dev->gyro_z_adc;
	data[3] = dev->accel_x_adc;
	data[4] = dev->accel_y_adc;
	data[5] = dev->accel_z_adc;
	data[6] = dev->temp_adc;
	err = copy_to_user(buf, data, sizeof(data));
	return 0;
}

static int icm20608_release(struct inode *inode, struct file *filp)
{
        filp->private_data = NULL;
        return 0;
}

#if 0
static int icm20608_read_regs(struct icm20608_device *dev,
                              unsigned char reg,
                              void *buf,
                              int len)
{
        int ret = 0;
        unsigned char txdata[len];
        struct spi_message m;
        struct spi_transfer *t;
        struct spi_device *spi = (struct spi_device *)dev->private_data;

        t = kzalloc(sizeof(struct spi_transfer), GFP_KERNEL);

        /* 片选拉低 */
        gpio_set_value(dev->cs_gpio, 0);

        /* send reg addr */
        txdata[0] = reg | 0x80;
        t->tx_buf = txdata;
        t->len = 1;
        spi_message_init(&m);
        spi_message_add_tail(t, &m);
        ret = spi_sync(spi, &m);
        if (ret != 0) {
                printk("icm20608 read regs spi_sync error: %d\n", ret);
                goto error;
        }

        /* read reg data */
        txdata[0] = 0xff;  /* unvaild */
        t->tx_buf = txdata;
        t->rx_buf = buf;
        t->len = len;
        spi_message_init(&m);
        spi_message_add_tail(t, &m);
        ret = spi_sync(spi, &m);
        if (ret != 0) {
                printk("icm20608 read regs spi_sync error: %d\n", ret);
                goto error;
        }

error:
        /* 取消片选 */
        gpio_set_value(dev->cs_gpio, 1);
        /* 释放内存 */
        kfree(t);

        return ret;
}

static int icm20608_write_regs(struct icm20608_device *dev,
                               unsigned char reg,
                               unsigned char *buf,
                               unsigned char len)
{
        int ret = 0;
        unsigned char txdata[len];
        struct spi_message m;
        struct spi_transfer *t;
        struct spi_device *spi = (struct spi_device *)dev->private_data;

        t = kzalloc(sizeof(struct spi_transfer), GFP_KERNEL);

        /* 片选拉低 */
        gpio_set_value(dev->cs_gpio, 0);

        /* send reg addr */
        txdata[0] = reg & ~0x80;
        t->tx_buf = txdata;
        t->len = 1;
        spi_message_init(&m);
        spi_message_add_tail(t, &m);
        ret = spi_sync(spi, &m);
        if (ret != 0) {
                printk("icm20608 read regs spi_sync error: %d\n", ret);
                goto error;
        }

        /* read reg data */
        t->tx_buf = buf;
        t->len = len;
        spi_message_init(&m);
        spi_message_add_tail(t, &m);
        ret = spi_sync(spi, &m);
        if (ret != 0) {
                printk("icm20608 read regs spi_sync error: %d\n", ret);
                goto error;
        }

error:
        /* 取消片选 */
        gpio_set_value(dev->cs_gpio, 1);
        /* 释放内存 */
        kfree(t);

        return ret;
}
#else

static int icm20608_read_regs(struct icm20608_device *dev,
                              unsigned char reg,
                              void *buf,
                              int len)
{
        struct spi_device *spi = (struct spi_device *)dev->private_data;
        unsigned char data = 0;
        int ret = 0;

        /* 片选拉低 */
        gpio_set_value(dev->cs_gpio, 0);

        data = reg | 0x80;
        ret = spi_write(spi, &data, 1);
        if (ret < 0) {
                printk("read regs spi write error\n");
                goto error;
        }

        ret = spi_read(spi, buf, len);
        if (ret < 0) {
                printk("read regs spi write error\n");
                goto error;
        }

error:
        gpio_set_value(dev->cs_gpio, 1);
        return ret;
}

static int icm20608_write_regs(struct icm20608_device *dev,
                               unsigned char reg,
                               unsigned char *buf,
                               unsigned char len)
{
        struct spi_device *spi = (struct spi_device *)dev->private_data;
        unsigned char data = 0;
        int ret = 0;

        /* 片选拉低 */
        gpio_set_value(dev->cs_gpio, 0);

        data = reg & ~0x80;
        ret = spi_write(spi, &data, 1);
        if (ret < 0) {
                printk("read regs spi write error\n");
                goto error;
        }

        ret = spi_write(spi, buf, len);
        if (ret < 0) {
                printk("read regs spi write error\n");
                goto error;
        }

error:
        gpio_set_value(dev->cs_gpio, 1);
        return ret;
}

#endif

static unsigned char icm20608_read_onebyte(struct icm20608_device *dev,
                                           unsigned char reg)
{
        unsigned char value = 0;
        unsigned char ret = 0;
        ret = icm20608_read_regs(dev, reg, &value, 1);
        if (ret == 0)
                ret = value;
        return ret;
}

static int icm20608_write_onebyte(struct icm20608_device *dev,
                                  unsigned char reg,
                                  unsigned char data)
{
        int ret = 0;
        ret = icm20608_write_regs(dev, reg, &data, 1);
        return ret;
}

static void icm20608_readdata(struct icm20608_device *dev)
{
	unsigned char data[14];
	icm20608_read_regs(dev, ICM20_ACCEL_XOUT_H, data, 14);

	dev->accel_x_adc = (signed short)((data[0] << 8) | data[1]); 
	dev->accel_y_adc = (signed short)((data[2] << 8) | data[3]); 
	dev->accel_z_adc = (signed short)((data[4] << 8) | data[5]); 
	dev->temp_adc    = (signed short)((data[6] << 8) | data[7]); 
	dev->gyro_x_adc  = (signed short)((data[8] << 8) | data[9]); 
	dev->gyro_y_adc  = (signed short)((data[10] << 8) | data[11]);
	dev->gyro_z_adc  = (signed short)((data[12] << 8) | data[13]);
}

static int icm20608_reg_init(struct icm20608_device *dev)
{
        unsigned char value = 0;
	
	icm20608_write_onebyte(dev, ICM20_PWR_MGMT_1, 0x80);
	mdelay(50);
	icm20608_write_onebyte(dev, ICM20_PWR_MGMT_1, 0x01);
	mdelay(50);

	value = icm20608_read_onebyte(dev, ICM20_WHO_AM_I);
	printk("ICM20608 ID = %#X\r\n", value);

	icm20608_write_onebyte(dev, ICM20_SMPLRT_DIV, 0x00);   /* 输出速率是内部采样率 */
	icm20608_write_onebyte(dev, ICM20_GYRO_CONFIG, 0x18);  /* 陀螺仪±2000dps量程 */
	icm20608_write_onebyte(dev, ICM20_ACCEL_CONFIG, 0x18); /* 加速度计±16G量程 */
	icm20608_write_onebyte(dev, ICM20_CONFIG, 0x04);       /* 陀螺仪低通滤波BW=20Hz */
	icm20608_write_onebyte(dev, ICM20_ACCEL_CONFIG2, 0x04);/* 加速度计低通滤波BW=21.2Hz */
	icm20608_write_onebyte(dev, ICM20_PWR_MGMT_2, 0x00);   /* 打开加速度计和陀螺仪所有轴 */
	icm20608_write_onebyte(dev, ICM20_LP_MODE_CFG, 0x00);  /* 关闭低功耗 */
	icm20608_write_onebyte(dev, ICM20_FIFO_EN, 0x00);      /* 关闭FIFO */
        return 0;
}

static int icm20608_probe(struct spi_device *spi)
{
        int ret = 0;
        if (icm20608_dev.major) {
                icm20608_dev.devid = MKDEV(icm20608_dev.major, icm20608_dev.minor);
                ret = register_chrdev_region(icm20608_dev.devid, DEVICE_CNT, DEVICE_NAME);
        } else {
                ret = alloc_chrdev_region(&icm20608_dev.devid, 0, DEVICE_CNT, DEVICE_NAME);
        }
        if (ret < 0) {
                printk("chrdev region error!\n");
                goto fail_chrdev_region;
        }
        icm20608_dev.major = MAJOR(icm20608_dev.devid);
        icm20608_dev.minor = MINOR(icm20608_dev.devid);
        printk("major:%d, minor:%d\n", icm20608_dev.major, icm20608_dev.minor);

        cdev_init(&icm20608_dev.cdev, &ops);
        ret = cdev_add(&icm20608_dev.cdev, icm20608_dev.devid, DEVICE_CNT);
        if (ret < 0) {
                printk("cdev add error!\n");
                goto fail_cdev_add;
        }
        icm20608_dev.class = class_create(THIS_MODULE, DEVICE_NAME);
        if (IS_ERR(icm20608_dev.class)) {
                printk("class create error!\n");
                ret = -EINVAL;
                goto fail_class_create;
        }
        icm20608_dev.device = device_create(icm20608_dev.class, NULL,
                                             icm20608_dev.devid, NULL, DEVICE_NAME);
        if (IS_ERR(icm20608_dev.device)) {
                printk("device create error!\n");
                ret = -EINVAL;
                goto fail_device_create;
        }

#if 0
        icm20608_dev.nd = of_find_node_by_path("/soc/aips-bus@02000000/spba-bus@02000000/ecspi@02010000");
#endif
        icm20608_dev.nd = of_get_parent(spi->dev.of_node);
        if (icm20608_dev.nd == NULL) {
                printk("ecspi node not found!\n");
                goto fail_find_node;
        }
        icm20608_dev.cs_gpio = of_get_named_gpio(icm20608_dev.nd, "cs-gpio", 0); 
        if (icm20608_dev.cs_gpio < 0) {
                printk("can't get cs-gpio!\n");
                goto fail_get_gpio;
        }
        ret = gpio_request(icm20608_dev.cs_gpio, "cs_gpio");
        if (ret < 0) {
                printk("cs_gpio requeset fail!\n");
                goto fail_request;
        }
        ret = gpio_direction_output(icm20608_dev.cs_gpio, 1);
        if (ret < 0) {
                printk("can't set gpio!\n");
                goto fail_set_dir;
        }

        /* spi mode configure */
        spi->mode = SPI_MODE_0;         /* MODE0, CPOL=0, CPHA=0 */
        spi_setup(spi);
        icm20608_dev.private_data = spi;

        icm20608_reg_init(&icm20608_dev);

        goto success;

fail_set_dir:
        gpio_free(icm20608_dev.cs_gpio);
fail_request:
fail_get_gpio:
fail_find_node:
        device_destroy(icm20608_dev.class, icm20608_dev.devid);
fail_device_create:
        class_destroy(icm20608_dev.class);
fail_class_create:
        cdev_del(&icm20608_dev.cdev);
fail_cdev_add:
        unregister_chrdev_region(icm20608_dev.devid, DEVICE_CNT);
fail_chrdev_region:
success:
        return 0;
}

static int icm20608_remove(struct spi_device *spi)
{
        gpio_free(icm20608_dev.cs_gpio);
        device_destroy(icm20608_dev.class, icm20608_dev.devid);
        class_destroy(icm20608_dev.class);
        cdev_del(&icm20608_dev.cdev);
        unregister_chrdev_region(icm20608_dev.devid, DEVICE_CNT);
        icm20608_dev.private_data = NULL;
        return 0;
}

const struct spi_device_id icm20608_id_table[] = {
        {"icm20608", 0},
        {}
};

const struct of_device_id icm20608_match_table[] = {
        { .compatible = "alientek, icm20608" },
        {},
};

static struct spi_driver icm20608_driver = {
        .probe = icm20608_probe,
        .remove = icm20608_remove,
        .id_table = icm20608_id_table,
        .driver = {
                .of_match_table = icm20608_match_table,
                .owner = THIS_MODULE,
                .name = "icm20608",
        },
};

static int __init icm20608_init(void)
{
        spi_register_driver(&icm20608_driver);
        return 0;
}

static void __exit icm20608_exit(void)
{
        spi_unregister_driver(&icm20608_driver);
}

module_init(icm20608_init);
module_exit(icm20608_exit);
MODULE_AUTHOR("wanglei");
MODULE_LICENSE("GPL");
