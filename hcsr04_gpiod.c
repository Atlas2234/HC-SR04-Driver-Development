// hcsr04_gpiod.c
#include <linux/module.h>
#include <linux/init.h>
#include <linux/kernel.h>

#include <linux/types.h>
#include <linux/kdev_t.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>

#include <linux/gpio/consumer.h>   // descriptor-based GPIO API
#include <linux/kobject.h>
#include <linux/sysfs.h>

#include <linux/ktime.h>
#include <linux/delay.h>

#define GPIO_OUT        20      /* Trigger (GPIO20) */
#define GPIO_IN         21      /* Echo    (GPIO21) */

/* Safety timeout so we don't spin forever waiting for echo (microseconds) */
#define ECHO_TIMEOUT_US 40000

static dev_t hcsr04_dev;
static struct cdev hcsr04_cdev;
static int hcsr04_lock;

static struct kobject *hcsr04_kobject;

static ktime_t rising, falling;

/* GPIO descriptors */
static struct gpio_desc *trig_gpiod;  /* output */
static struct gpio_desc *echo_gpiod;  /* input */

static int hcsr04_open(struct inode *inode, struct file *file)
{
    pr_info("hcsr04_dev: %s\n", __func__);

    if (hcsr04_lock > 0)
        return -EBUSY;

    hcsr04_lock++;
    return 0;
}

static int hcsr04_close(struct inode *inode, struct file *file)
{
    pr_info("hcsr04_dev: %s\n", __func__);
    hcsr04_lock = 0;
    return 0;
}

static ssize_t hcsr04_read(struct file *filp, char __user *buf,
                           size_t count, loff_t *f_pos)
{
    int pulse_us;
    int not_copied;

    pr_info("hcsr04_dev: %s\n", __func__);

    pulse_us = (int)ktime_to_us(ktime_sub(falling, rising));

    if (count < sizeof(pulse_us))
        return -EINVAL;

    not_copied = copy_to_user(buf, &pulse_us, sizeof(pulse_us));
    if (not_copied)
        return -EFAULT;

    return sizeof(pulse_us);
}

static ssize_t hcsr04_write(struct file *filp, const char __user *buffer,
                            size_t length, loff_t *offset)
{
    ktime_t t_start;

    pr_info("hcsr04_dev: %s\n", __func__);

    /* 10us trigger pulse */
    gpiod_set_value(trig_gpiod, 0);
    gpiod_set_value(trig_gpiod, 1);
    udelay(10);
    gpiod_set_value(trig_gpiod, 0);

    /* Wait for echo rising edge (with timeout) */
    t_start = ktime_get();
    while (!gpiod_get_value(echo_gpiod)) {
        if (ktime_to_us(ktime_sub(ktime_get(), t_start)) > ECHO_TIMEOUT_US)
            return -ETIMEDOUT;
        cpu_relax();
    }
    rising = ktime_get();

    /* Wait for echo falling edge (with timeout) */
    t_start = ktime_get();
    while (gpiod_get_value(echo_gpiod)) {
        if (ktime_to_us(ktime_sub(ktime_get(), t_start)) > ECHO_TIMEOUT_US)
            return -ETIMEDOUT;
        cpu_relax();
    }
    falling = ktime_get();

    /* Signal success by reporting we "consumed" the write */
    return length ? length : 1;
}

static const struct file_operations hcsr04_fops = {
    .owner   = THIS_MODULE,
    .read    = hcsr04_read,
    .write   = hcsr04_write,
    .open    = hcsr04_open,
    .release = hcsr04_close,
};

/* sysfs: /sys/kernel/hcsr04/hcsr04 */
static ssize_t hcsr04_show(struct kobject *kobj,
                           struct kobj_attribute *attr, char *buf)
{
    pr_info("hcsr04_dev: %s\n", __func__);
    return scnprintf(buf, PAGE_SIZE, "%d\n",
                     (int)ktime_to_us(ktime_sub(falling, rising)));
}

static ssize_t hcsr04_store(struct kobject *kobj,
                            struct kobj_attribute *attr,
                            const char *buf, size_t count)
{
    pr_info("hcsr04_dev: %s\n", __func__);
    /* Nothing to store in this example */
    return count;
}

static struct kobj_attribute hcsr04_attribute =
    __ATTR(hcsr04, 0660, hcsr04_show, hcsr04_store);

static int __init hcsr04_module_init(void)
{
    char buffer[64];
    int ret;

    pr_info("Loading hcsr04_module\n");

    ret = alloc_chrdev_region(&hcsr04_dev, 0, 1, "hcsr04_dev");
    if (ret)
        return ret;

    pr_info("%s\n", format_dev_t(buffer, hcsr04_dev));

    cdev_init(&hcsr04_cdev, &hcsr04_fops);
    hcsr04_cdev.owner = THIS_MODULE;

    ret = cdev_add(&hcsr04_cdev, hcsr04_dev, 1);
    if (ret)
        goto err_chrdev;

    /* Convert fixed GPIO numbers to descriptors */
    trig_gpiod = gpio_to_desc(GPIO_OUT);
    if (!trig_gpiod) {
        pr_err("hcsr04_dev: failed to get desc for GPIO_OUT %d\n", GPIO_OUT);
        ret = -EINVAL;
        goto err_cdev;
    }

    echo_gpiod = gpio_to_desc(GPIO_IN);
    if (!echo_gpiod) {
        pr_err("hcsr04_dev: failed to get desc for GPIO_IN %d\n", GPIO_IN);
        ret = -EINVAL;
        goto err_cdev;
    }

    /* Configure directions (no explicit request/free needed) */
    ret = gpiod_direction_output(trig_gpiod, 0);
    if (ret) {
        pr_err("hcsr04_dev: failed to set TRIG as output: %d\n", ret);
        goto err_cdev;
    }

    ret = gpiod_direction_input(echo_gpiod);
    if (ret) {
        pr_err("hcsr04_dev: failed to set ECHO as input: %d\n", ret);
        goto err_cdev;
    }

    /* Create sysfs node: /sys/kernel/hcsr04/hcsr04 */
    hcsr04_kobject = kobject_create_and_add("hcsr04", kernel_kobj);
    if (!hcsr04_kobject) {
        ret = -ENOMEM;
        goto err_cdev;
    }

    ret = sysfs_create_file(hcsr04_kobject, &hcsr04_attribute.attr);
    if (ret) {
        pr_err("hcsr04_dev: failed to create /sys/kernel/hcsr04/hcsr04\n");
        ret = -ENOMEM;
        goto err_kobj;
    }

    return 0;

err_kobj:
    kobject_put(hcsr04_kobject);
err_cdev:
    cdev_del(&hcsr04_cdev);
err_chrdev:
    unregister_chrdev_region(hcsr04_dev, 1);
    return ret;
}

static void __exit hcsr04_module_cleanup(void)
{
    pr_info("Cleaning-up hcsr04_dev.\n");

    if (hcsr04_kobject) {
        sysfs_remove_file(hcsr04_kobject, &hcsr04_attribute.attr);
        kobject_put(hcsr04_kobject);
    }

    hcsr04_lock = 0;

    cdev_del(&hcsr04_cdev);
    unregister_chrdev_region(hcsr04_dev, 1);
}

module_init(hcsr04_module_init);
module_exit(hcsr04_module_cleanup);

MODULE_AUTHOR("Your name");
MODULE_LICENSE("GPL");
