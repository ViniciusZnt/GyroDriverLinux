#include <linux/init.h>
#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/gpio.h>

MODULE_AUTHOR("Vinicius Zanatta");
MODULE_DESCRIPTION("Gyroscope and GPIO Linux Driver");
MODULE_LICENSE("GPL");

MODULE_DEVICE_TABLE(i2c, gyro_id);
MODULE_DEVICE_TABLE(of, gyro_of_match);

#define GYROSCOPE_I2C_ADDR 0x68
#define GYROSCOPE_REG_XYZ 0x43
#define D20_SA0_HIGH_ADDRESS      0b1101011 // also applies to D20H
#define D20_SA0_LOW_ADDRESS       0b1101010 // also applies to D20H

#define D20H_WHO_ID     0xD7
#define D20_WHO_ID      0xD4
#define L3G4200D_WHO_ID 0xD3

#define GPIO_SDA 21
#define GPIO_SCL 22

struct i2c_client *gyro_client;

typedef enum {
    device_auto,
    device_D20H,
    device_D20,
    device_4200D
} deviceType;

typedef struct {
    deviceType device;
    int address;
    int16_t last_status;
    unsigned int io_timeout;
    bool did_timeout;
} L3G;

L3G init_L3G() {
    L3G gyro;
    gyro.device = device_D20;
    gyro.address = (D20_SA0_HIGH_ADDRESS);
    gyro.io_timeout = 0;
    gyro.did_timeout = false;
    return gyro;
}

static int gyro_read_data(struct i2c_client *client, u8 reg, s16 *data)
{
    struct i2c_msg msgs[2];
    int ret;

    msgs[0].addr = client->addr;
    msgs[0].flags = 0;
    msgs[0].buf = &reg;
    msgs[0].len = sizeof(reg);

    msgs[1].addr = client->addr;
    msgs[1].flags = I2C_M_RD;
    msgs[1].buf = (char *)data;
    msgs[1].len = sizeof(*data);

    ret = i2c_transfer(client->adapter, msgs, 2);
    if (ret < 0) {
        dev_err(&client->dev, "Failed to read data from the gyroscope\n");
        return ret;
    }

    return 0;
}

static ssize_t gyro_read(struct file *file, char __user *buf, size_t count, loff_t *pos)
{
    s16 gyro_data;

    if (gyro_read_data(gyro_client, GYROSCOPE_REG_XYZ, &gyro_data)) {
        return -EIO;
    }

    if (copy_to_user(buf, &gyro_data, sizeof(gyro_data))) {
        return -EFAULT;
    }

    return sizeof(gyro_data);
}

static const struct file_operations gyro_fops = {
    .owner = THIS_MODULE,
    .read = gyro_read,
};

static struct cdev gyro_cdev;

static int gyro_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
    int ret;
    dev_t devno;

    ret = alloc_chrdev_region(&devno, 0, 1, "gyroscope");
    if (ret < 0) {
        dev_err(&client->dev, "Failed to allocate char device region\n");
        return ret;
    }

    cdev_init(&gyro_cdev, &gyro_fops);
    gyro_cdev.owner = THIS_MODULE;

    ret = cdev_add(&gyro_cdev, devno, 1);
    if (ret < 0) {
        unregister_chrdev_region(devno, 1);
        dev_err(&client->dev, "Failed to add char device\n");
        return ret;
    }

    // Configurar GPIOs para SDA e SCL
    ret = gpio_request_one(GPIO_SDA, GPIOF_OUT_INIT_HIGH, "gyro_sda");
    if (ret) {
        dev_err(&client->dev, "Failed to request GPIO SDA\n");
        return ret;
    }

    ret = gpio_request_one(GPIO_SCL, GPIOF_OUT_INIT_HIGH, "gyro_scl");
    if (ret) {
        gpio_free(GPIO_SDA);
        dev_err(&client->dev, "Failed to request GPIO SCL\n");
        return ret;
    }

    gyro_client = client;

    dev_info(&client->dev, "Gyroscope driver loaded\n");

    return 0;
}

static int gyro_remove(struct i2c_client *client)
{
    cdev_del(&gyro_cdev);
    unregister_chrdev_region(gyro_cdev.dev, 1);

    gpio_free(GPIO_SDA);
    gpio_free(GPIO_SCL);

    dev_info(&client->dev, "Gyroscope driver unloaded\n");

    return 0;
}

static const struct i2c_device_id gyro_id[] = {
    { "gyroscope", 0 },
    { }
};

static const struct of_device_id gyro_of_match[] = {
    { .compatible = "gyroscope" },
    { },
};

static struct i2c_driver gyro_driver = {
    .driver = {
        .name = "gyroscope",
        .of_match_table = of_match_ptr(gyro_of_match),
    },
    .probe = gyro_probe,
    .remove = gyro_remove,
    .id_table = gyro_id,
};

module_i2c_driver(gyro_driver);

/**
 * GPIO Driver Part
 */

static dev_t my_device_nr;
static struct class *my_class;
static struct cdev my_device;

#define DRIVER_NAME "my_gpio_driver"
#define DRIVER_CLASS "MyModuleClass"

static ssize_t driver_read(struct file *File, char *user_buffer, size_t count, loff_t *offs) {
    int to_copy, not_copied, delta;
    char tmp[3] = " \n";

    to_copy = min(count, sizeof(tmp));
    tmp[0] = gpio_get_value(17) + '0';

    not_copied = copy_to_user(user_buffer, &tmp, to_copy);
    delta = to_copy - not_copied;

    return delta;
}

static ssize_t driver_write(struct file *File, const char *user_buffer, size_t count, loff_t *offs) {
    int to_copy, not_copied, delta;
    char value;

    to_copy = min(count, sizeof(value));
    not_copied = copy_from_user(&value, user_buffer, to_copy);

    switch (value) {
        case '0':
            gpio_set_value(4, 0);
            break;
        case '1':
            gpio_set_value(4, 1);
            break;
        default:
            printk("Invalid Input!\n");
            break;
    }

    delta = to_copy - not_copied;
    return delta;
}

static int driver_open(struct inode *device_file, struct file *instance) {
    printk("dev_nr - open was called!\n");
    return 0;
}

static int driver_close(struct inode *device_file, struct file *instance) {
    printk("dev_nr - close was called!\n");
    return 0;
}

static struct file_operations fops = {
    .owner = THIS_MODULE,
    .open = driver_open,
    .release = driver_close,
    .read = driver_read,
    .write = driver_write
};

static int __init ModuleInit(void) {
    printk("Hello, Kernel!\n");

    if (alloc_chrdev_region(&my_device_nr, 0, 1, DRIVER_NAME) < 0) {
        printk("Device Nr. could not be allocated!\n");
        return -1;
    }
    printk("read_write - Device Nr. Major: %d, Minor: %d was registered!\n", my_device_nr >> 20, my_device_nr && 0xfffff);

    if ((my_class = class_create(THIS_MODULE, DRIVER_CLASS)) == NULL) {
        printk("Device class can not be created!\n");
        goto ClassError;
    }

    if (device_create(my_class, NULL, my_device_nr, NULL, DRIVER_NAME) == NULL) {
        printk("Can not create device file!\n");
        goto FileError;
    }

    cdev_init(&my_device, &fops);

    if (cdev_add(&my_device, my_device_nr, 1) == -1) {
        printk("Registering of device to kernel failed!\n");
        goto AddError;
    }

    if (gpio_request(4, "rpi-gpio-4")) {
        printk("Can not allocate GPIO 4\n");
        goto AddError;
    }

    if (gpio_direction_output(4, 0)) {
        printk("Can not set GPIO 4 to output!\n");
        goto Gpio4Error;
    }

    if (gpio_request(17, "rpi-gpio-17")) {
        printk("Can not allocate GPIO 17\n");
        goto Gpio4Error;
    }

    if (gpio_direction_input(17)) {
        printk("Can not set GPIO 17 to input!\n");
        goto Gpio17Error;
    }

    return 0;

Gpio17Error:
    gpio_free(17);
Gpio4Error:
    gpio_free(4);
AddError:
    device_destroy(my_class, my_device_nr);
FileError:
    class_destroy(my_class);
ClassError:
    unregister_chrdev_region(my_device_nr, 1);
    return -1;
}

static void __exit ModuleExit(void) {
    gpio_set_value(4, 0);
    gpio_free(17);
    gpio_free(4);
    cdev_del(&my_device);
    device_destroy(my_class, my_device_nr);
    class_destroy(my_class);
    unregister_chrdev_region(my_device_nr, 1);
    printk("Goodbye, Kernel\n");
}

module_init(ModuleInit);
module_exit(ModuleExit);
