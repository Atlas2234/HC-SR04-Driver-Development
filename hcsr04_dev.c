#include <linux/module.h>
#include <linux/init.h>
#include <linux/kernel.h>

#include <linux/types.h>
#include <linux/kdev_t.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>

#include <linux/gpio/consumer.h>
#include <linux/gpio.h>

#include <linux/interrupt.h>

#define GPIO_OUT 20
#define GPIO_IN 21

#define GPIO_OFFSET 512

static dev_t gpio_dev;
static struct cdev gpio_cdev;

static int gpio_lock;
static volatile char gpio_in_value;

static struct gpio_desc *gpiod_out;
static struct gpio_desc *gpiod_in;

// For automatically adding device node
static struct class *gpio_class;
static struct device *gpio_device;

static irqreturn_t InterruptHandler(int irq, void *dev_id) {
	gpio_in_value = gpiod_get_value_cansleep(gpiod_in);

	pr_info("gpio_dev: %s got GPIO_IN with value %c\n", __func__, gpio_in_value + '0');

	return IRQ_HANDLED;
}

static char *gpio_devnode(const struct device *dev, umode_t *mode) {
	if (mode)
		*mode = 0666; //rw for everyone
	return NULL;
}

static int gpio_open(struct inode *inode, struct file *file) {
	pr_info("gpio_dev: %s\n", __func__);
	if(gpio_lock > 0)
		return -EBUSY;
	gpio_lock++;
	return 0;
}

static int gpio_close(struct inode *inode, struct file *file) {
	pr_info("gpio_dev: %s\n", __func__);
	gpio_lock = 0;
	return 0;
}

static ssize_t gpio_read(struct file *filp, char __user *buf, size_t count, loff_t *f_pos) {
	char kbuf[2];

	pr_info("gpio read (count=%d, offset=%d)\n", (int)count, (int)*f_pos);

	kbuf[0] = gpio_in_value;
	kbuf[1] = 0;

	if (copy_to_user(buf, kbuf, 1))
		return -EFAULT;

	return 1;
}

static ssize_t gpio_write(struct file *filp, const char __user *ubuf, size_t length, loff_t *offset) {
	char ch;
	size_t n = 0;

	while (length) {
		if (copy_from_user(&ch, ubuf, 1))
			return -EFAULT;

		if (ch == '0' || ch == '1') {
			int v = (ch == '1') ? 1 : 0;

			gpiod_set_value_cansleep(gpiod_out, v);
			pr_info("gpio_dev: %s wrote %c to GPIO_OUT\n", __func__, ch);
		}

		ubuf++;
		length--;
		n++;
	}

	return n;
}

static const struct file_operations gpio_fops = {
	.owner = THIS_MODULE,
	.read = gpio_read,
	.write = gpio_write,
	.open = gpio_open,
	.release = gpio_close,
};

static int __init gpio_module_init(void) {
	char buffer[64];
	int ret = 0;
	int irq;

	pr_info("loading gpio_module\n");

	ret = alloc_chrdev_region(&gpio_dev, 0, 1, "gpio_dev");
	if (ret)
		return ret;

	pr_info("%s\n", format_dev_t(buffer, gpio_dev));

	cdev_init(&gpio_cdev, &gpio_fops);
	gpio_cdev.owner = THIS_MODULE;
	ret = cdev_add(&gpio_cdev, gpio_dev, 1);
	if (ret)
		goto err_chrdev;

	gpio_class = class_create("hcsr04");
	if (IS_ERR(gpio_class)) {
		pr_err("gpio_dev: failed to create class\n");
		ret = PTR_ERR(gpio_class);

		goto err_cdev;
	}

	gpio_class->devnode = gpio_devnode;

	gpio_device = device_create(
		gpio_class,
		NULL,
		gpio_dev,
		NULL,
		"hcsr04_dev"	
	);
	
	if (IS_ERR(gpio_device)) {
		pr_err("gpio_dev: failed to create device \n");
		ret = PTR_ERR(gpio_device);
		goto err_class;
	}

	gpiod_out = gpio_to_desc(GPIO_OUT + GPIO_OFFSET);
	if(!gpiod_out) {
		pr_err("gpio_dev: %s unable to get desc for GPIO_OUT=%d\n", __func__, GPIO_OUT);
		ret = -EINVAL;
		goto err_cdev;
	}

	gpiod_in = gpio_to_desc(GPIO_IN + GPIO_OFFSET);
	if (!gpiod_in) {
		pr_err("gpio_dev: %s unable to get desc for GPIO_IN=%d\n", __func__, GPIO_IN);
		ret = -EINVAL;
		goto err_cdev;
	}

	ret = gpiod_direction_output(gpiod_out, 0);
	if (ret) {
		pr_err("gpio_dev: %s unable to set GPIO_OUT as output\n", __func__);
		goto err_cdev;
	}

	ret = gpiod_direction_input(gpiod_in);
	if (ret) {
		pr_err("gpio_dev: %s unable to get GPIO_IN as input\n", __func__);
		goto err_cdev;
	}

	irq = gpiod_to_irq(gpiod_in);
	if (irq < 0) {
		pr_err("gpio_dev: %s unable to map GPIO-IN to IRQ\n", __func__);
		ret = irq;
		goto err_cdev;
	}

	ret = request_irq(irq, InterruptHandler, IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING, "gpio_dev", NULL);
	if(ret < 0) {
		pr_err("gpio_dev: %s unable to register IRQ for GPIO_IN\n", __func__);
		goto err_cdev;
	}

	return 0;

err_cdev:
	cdev_del(&gpio_cdev);
err_chrdev:
	unregister_chrdev_region(gpio_dev, 1);
err_class:
	class_destroy(gpio_class);

return ret;
}

static void __exit gpio_module_cleanup(void) {
	int irq;

	pr_info("Cleaning-up gpio_dev. \n");

	irq = gpiod_to_irq(gpiod_in);
	if (irq >= 0)
		free_irq(irq, NULL);

	gpio_lock = 0;
	
	device_destroy(gpio_class, gpio_dev);
	class_destroy(gpio_class);

	cdev_del(&gpio_cdev);
	unregister_chrdev_region(gpio_dev, 1);
}

module_init(gpio_module_init);
module_exit(gpio_module_cleanup);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Atlas");
MODULE_DESCRIPTION("A Linux kernel module to enable gpio pins 20 and 21");
