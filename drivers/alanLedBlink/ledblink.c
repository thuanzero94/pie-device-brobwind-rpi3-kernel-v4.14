/*
 * ledblink.c
 *
 *  Created on: Sep 21, 2019
 *      Author: Alan
 */
#include <linux/init.h>
#include <linux/module.h>
#include <asm/io.h>
#include <linux/timer.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/mm.h>
#include <linux/cdev.h>
#include <linux/fs.h>
#include <linux/uaccess.h>
#include <linux/kernel.h>

//Tested with kernel 4.14
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Alan, 2019");
MODULE_DESCRIPTION("Control and Blink Led via GPIO 18/Pin 12");

#define PERIPH_BASE 0x3f000000				// Start Physical Address of Rasp Pi 3B/3B+ base BCM2837 chipset (BCM2835 is 0x7e000000)
#define GPIO_BASE (PERIPH_BASE + 0x200000) 	// GPIO register address start at 0x3f200000

static struct class *gpio_class;
static struct cdev *gpio_cdev;
static dev_t gpio_dev_t = 0;
static struct device *gpio_device;

static char message[256] = { 0 }; ///< Memory for the string that is passed from userspace
static size_t size_of_message; ///< Used to remember the size of the string stored

static const int LedGpioPin = 18;

static struct timer_list s_BlinkTimer;
static int s_BlinkPeriod = 1000;
static int blink_status = 0;

struct GpioRegisters {
	uint32_t GPFSEL[6];
	uint32_t Reserved1;
	uint32_t GPSET[2];
	uint32_t Reserved2;
	uint32_t GPCLR[2];
};

struct GpioRegisters *s_pGpioRegisters;

static void SetGPIOFunction(int, int);
static void SetGPIOOutputValue(int, bool);
static void BlinkTimerHandler(unsigned long);
static ssize_t set_period_callback(struct device*, struct device_attribute*,
		const char*, size_t);
static ssize_t get_period_callback(struct device*, struct device_attribute*,
		char*);
static int gpio_cdev_open(struct inode*, struct file*);
static int gpio_cdev_release(struct inode*, struct file*);
static ssize_t gpio_cdev_read(struct file*, char __user*, size_t, loff_t*);
static ssize_t gpio_cdev_write(struct file*, const char __user*, size_t,
		loff_t*);

static struct file_operations gpio_cdev_fops = { .owner = THIS_MODULE, .open =
		gpio_cdev_open, .release = gpio_cdev_release, .read = gpio_cdev_read,
		.write = gpio_cdev_write, };

static DEVICE_ATTR(period, S_IRWXU | S_IRWXG, get_period_callback, set_period_callback);

static int __init LedBlinkModule_init(void) {

	int result;

	s_pGpioRegisters = (struct GpioRegisters*) ioremap(GPIO_BASE,
			sizeof(struct GpioRegisters));

	SetGPIOFunction(LedGpioPin, 0b001);  	//Configure the pin as output

	setup_timer(&s_BlinkTimer, BlinkTimerHandler, 0);
	result = mod_timer(&s_BlinkTimer,
			jiffies + msecs_to_jiffies(s_BlinkPeriod));
	if (result < 0) {
		printk(KERN_ERR "failed to mod_timer\n");
		goto fail_mod_timer;
	}

	printk(KERN_DEBUG "sample char device init\n");

	result = alloc_chrdev_region(&gpio_dev_t, 0, 1, "gpio-cdev");
	if (result < 0) {
		printk(KERN_ERR "failed to alloc chrdev region\n");
		goto fail_alloc_chrdev_region;
	}

	gpio_cdev = cdev_alloc();
	if (!gpio_cdev) {
		result = -ENOMEM;
		printk(KERN_ERR "failed to alloc cdev\n");
		goto fail_alloc_cdev;
	}

	cdev_init(gpio_cdev, &gpio_cdev_fops);
	result = cdev_add(gpio_cdev, gpio_dev_t, 1);
	if (result < 0) {
		printk(KERN_ERR "failed to add cdev\n");
		goto fail_add_cdev;
	}

	gpio_class = class_create(THIS_MODULE, "rasp_gpio");
	if (!gpio_class) {
		result = -EEXIST;
		printk(KERN_ERR "failed to create class\n");
		goto fail_create_class;
	}

	gpio_device = device_create(gpio_class, NULL, gpio_dev_t, NULL,
			"gpio%d_cdev%d", LedGpioPin, MINOR(gpio_dev_t));
	if (!gpio_device) {
		result = -EINVAL;
		printk(KERN_ERR "failed to create device\n");
		goto fail_create_device;
	}

	result = device_create_file(gpio_device, &dev_attr_period);
	if (result < 0) {
		printk(KERN_ERR "failed to create device\n");
		goto fail_device_create_file;
	}

	return 0;

	fail_device_create_file: device_destroy(gpio_class, gpio_dev_t);
	fail_create_device: class_destroy(gpio_class);
	fail_create_class: cdev_del(gpio_cdev);
	fail_add_cdev: fail_alloc_cdev: unregister_chrdev_region(gpio_dev_t, 1);
	fail_alloc_chrdev_region: del_timer(&s_BlinkTimer);
//    return result;
	fail_mod_timer: return result;
}

static void __exit LedBlinkModule_exit(void)
{
	printk(KERN_INFO "sample char device exit\n");
	SetGPIOFunction(LedGpioPin, 0);  	//Configure the pin as input
	iounmap(s_pGpioRegisters);
	device_remove_file(gpio_device, &dev_attr_period);
	del_timer(&s_BlinkTimer);
	device_destroy(gpio_class, gpio_dev_t);
	class_destroy(gpio_class);
	cdev_del(gpio_cdev);
	unregister_chrdev_region(gpio_dev_t, 1);
}

static void SetGPIOFunction(int GPIO, int functionCode) {
	int registerIndex = GPIO / 10;
	int bit = (GPIO % 10) * 3;
	unsigned newValue = 0;
	unsigned oldValue = s_pGpioRegisters->GPFSEL[registerIndex];
	unsigned mask = 0b111 << bit;
	printk(KERN_INFO "Changing function of GPIO%d from %x to %x\n", GPIO,
			(oldValue >> bit) & 0b111, functionCode);
	s_pGpioRegisters->GPFSEL[registerIndex] = (oldValue & ~mask)
			| ((functionCode << bit) & mask);
	newValue = s_pGpioRegisters->GPFSEL[registerIndex];
	printk(
			KERN_INFO "oldValue: %x\n mask: %x\n ~mask: %x\n oldValue & ~mask: %x\n(functionCode << bit) & mask: %x\nnewValue: %x",
			oldValue, mask, ~mask, oldValue & ~mask,
			(functionCode << bit) & mask, newValue);
}

static void SetGPIOOutputValue(int GPIO, bool outputValue) {
	if (outputValue)
		s_pGpioRegisters->GPSET[GPIO / 32] = (1 << (GPIO % 32));
	else
		s_pGpioRegisters->GPCLR[GPIO / 32] = (1 << (GPIO % 32));
}

static void BlinkTimerHandler(unsigned long unused) {
	static bool on = false;
	if (blink_status) {
		on = !on;
		SetGPIOOutputValue(LedGpioPin, on);
	}
	mod_timer(&s_BlinkTimer, jiffies + msecs_to_jiffies(s_BlinkPeriod));
}

static ssize_t set_period_callback(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count) {
	long period_value = 0;
	if (kstrtol(buf, 10, &period_value) < 0)
		return -EINVAL;
	if (period_value < 10)	//Safety check
		return - EINVAL;

	s_BlinkPeriod = period_value;
	return count;
}

static ssize_t get_period_callback(struct device *dev,
		struct device_attribute *attr, char *buf) {
	return sprintf(buf, "%d\n", s_BlinkPeriod);;
}

static int gpio_cdev_open(struct inode *inode, struct file *file) {
//    printk(KERN_INFO "open sample char device\n");
	return 0;
}

static int gpio_cdev_release(struct inode *inode, struct file *file) {
//    printk(KERN_INFO "release sample char device\n");
	return 0;
}

static ssize_t gpio_cdev_read(struct file *file, char __user *buf, size_t count,
		loff_t *offset) {
	ssize_t len = min(size_of_message - (size_t)*offset, size_of_message);

//    printk(KERN_INFO "Read sample char device\n");

	if (len <= 0)
		return 0;

	if (copy_to_user(buf, message + *offset, len)) {
		return -EFAULT;
	}

	*offset += len;

	return len;
}

static ssize_t gpio_cdev_write(struct file *file, const char __user *buf,
		size_t count, loff_t *offset) {
	size_t maxdatalen = 30, ncopied;
	uint8_t databuf[maxdatalen];

//    printk(KERN_INFO "write sample char device\n data:");

	if (count < maxdatalen) {
		maxdatalen = count;
	}

	ncopied = copy_from_user(databuf, buf, maxdatalen);

	if (ncopied == 0) {
		printk(KERN_INFO "Copied %zd bytes from the user\n", maxdatalen);
		sprintf(message, "%s", databuf);
		size_of_message = maxdatalen;
	} else {
		printk(KERN_INFO "Could't copy %zd bytes from the user\n", ncopied);
	}

	databuf[maxdatalen] = 0;
	printk(KERN_INFO "Data from the user: %s\n", databuf);
	// Check value for - Turn on/off GPIO18
	if (message[0] == '1') {
		printk(KERN_INFO "Turn On Led\n");
		blink_status = 0;
		SetGPIOOutputValue(LedGpioPin, 1);
	} else if (message[0] == '0') {
		printk(KERN_INFO "Turn off led\n");
		blink_status = 0;
		SetGPIOOutputValue(LedGpioPin, 0);
	} else if (!strncmp("blink", message, 5)) {
		printk(KERN_INFO "Enable Blink Led on GPIO%d - Period: %d\n", LedGpioPin, s_BlinkPeriod);
		blink_status = 1;
	} else {
		printk(KERN_INFO "Turn off led and Blink\n");
		blink_status = 0;
		SetGPIOOutputValue(LedGpioPin, 0);
	}

	return count;
}

module_init(LedBlinkModule_init);
module_exit(LedBlinkModule_exit);

