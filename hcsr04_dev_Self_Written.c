// hcsr04_dev.c
#include <linux/module.h>
#include <linux/init.h>
#include <linux/kernel.h>

#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/uaccess.h>

#include <linux/gpio.h>
#include <linux/gpio/consumer.h>
#include <linux/interrupt.h>

#include <linux/ktime.h>
#include <linux/wait.h>
#include <linux/delay.h>
#include <linux/version.h>

#define DEVICE_NAME "hcsr04_dev"
#define CLASS_NAME  "hcsr04"

/*
 * Raspberry Pi GPIO numbers
 * (BCM numbering + offset depends on your kernel;
 * this matches what you were already using)
 */
#define GPIO_TRIG   21
#define GPIO_ECHO   20
#define GPIO_OFFSET 512

static dev_t devt;
static struct cdev hcsr04_cdev;
static struct class *hcsr04_class;
static struct device *hcsr04_device;

/* GPIO descriptors */
static struct gpio_desc *gpiod_trig;
static struct gpio_desc *gpiod_echo;

/* IRQ */
static int echo_irq;

/* Timing + synchronization */
static ktime_t echo_start;
static ktime_t echo_end;
static wait_queue_head_t echo_wq;
static bool measurement_done;

/* -------------------------------------------------- */
/* Interrupt handler                                  */
/* -------------------------------------------------- */
static irqreturn_t hcsr04_irq_handler(int irq, void *dev_id)
{
    int value = gpiod_get_value(gpiod_echo);

    if (value) {
        /* Rising edge */
        echo_start = ktime_get();
    } else {
        /* Falling edge */
        echo_end = ktime_get();
        measurement_done = true;
        wake_up_interruptible(&echo_wq);
    }

    return IRQ_HANDLED;
}

/* -------------------------------------------------- */
/* Trigger pulse                                      */
/* -------------------------------------------------- */
static void hcsr04_trigger(void)
{
    gpiod_set_value(gpiod_trig, 1);
    udelay(10);                /* 10 µs pulse */
    gpiod_set_value(gpiod_trig, 0);
}

/* -------------------------------------------------- */
/* Character device read()                            */
/* -------------------------------------------------- */
static ssize_t hcsr04_read(
    struct file *file,
    char __user *buf,
    size_t count,
    loff_t *ppos
)
{
    s64 duration_ns;
    u32 distance_cm;

    if (count < sizeof(distance_cm))
        return -EINVAL;

    measurement_done = false;

    /* Start measurement */
    hcsr04_trigger();

    /* Wait for echo (timeout ~60 ms) */
    if (wait_event_interruptible_timeout(
            echo_wq,
            measurement_done,
            msecs_to_jiffies(60)) == 0) {
        return -ETIMEDOUT;
    }

    duration_ns = ktime_to_ns(ktime_sub(echo_end, echo_start));

    /*
     * HC-SR04 formula:
     * distance_cm = echo_time_us / 58
     */
    distance_cm = (u32)((duration_ns / 1000) / 58);

    if (copy_to_user(buf, &distance_cm, sizeof(distance_cm)))
        return -EFAULT;

    return sizeof(distance_cm);
}

/* -------------------------------------------------- */
/* File operations                                    */
/* -------------------------------------------------- */
static const struct file_operations hcsr04_fops = {
    .owner = THIS_MODULE,
    .read  = hcsr04_read,
};

/* -------------------------------------------------- */
/* Module init                                        */
/* -------------------------------------------------- */
static int __init hcsr04_init(void)
{
    int ret;

    pr_info("hcsr04: loading driver\n");

    /* Allocate device numbers */
    ret = alloc_chrdev_region(&devt, 0, 1, DEVICE_NAME);
    if (ret)
        return ret;

    /* Register cdev */
    cdev_init(&hcsr04_cdev, &hcsr04_fops);
    ret = cdev_add(&hcsr04_cdev, devt, 1);
    if (ret)
        goto err_chrdev;

    /* Create class (kernel-version safe) */
#if LINUX_VERSION_CODE >= KERNEL_VERSION(6,4,0)
    hcsr04_class = class_create(CLASS_NAME);
#else
    hcsr04_class = class_create(THIS_MODULE, CLASS_NAME);
#endif
    if (IS_ERR(hcsr04_class)) {
        ret = PTR_ERR(hcsr04_class);
        goto err_cdev;
    }

    /* Auto-create /dev/hcsr04_dev */
    hcsr04_device = device_create(
        hcsr04_class,
        NULL,
        devt,
        NULL,
        DEVICE_NAME
    );
    if (IS_ERR(hcsr04_device)) {
        ret = PTR_ERR(hcsr04_device);
        goto err_class;
    }

    /* GPIO setup */
    gpiod_trig = gpio_to_desc(GPIO_TRIG + GPIO_OFFSET);
    gpiod_echo = gpio_to_desc(GPIO_ECHO + GPIO_OFFSET);

    if (!gpiod_trig || !gpiod_echo) {
        ret = -EINVAL;
        goto err_device;
    }

    ret = gpiod_direction_output(gpiod_trig, 0);
    if (ret)
        goto err_device;

    ret = gpiod_direction_input(gpiod_echo);
    if (ret)
        goto err_device;

    /* IRQ */
    echo_irq = gpiod_to_irq(gpiod_echo);
    if (echo_irq < 0) {
        ret = echo_irq;
        goto err_device;
    }

    ret = request_irq(
        echo_irq,
        hcsr04_irq_handler,
        IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
        DEVICE_NAME,
        NULL
    );
    if (ret)
        goto err_device;

    init_waitqueue_head(&echo_wq);
    measurement_done = false;

    pr_info("hcsr04: driver loaded successfully\n");
    return 0;

err_device:
    device_destroy(hcsr04_class, devt);
err_class:
    class_destroy(hcsr04_class);
err_cdev:
    cdev_del(&hcsr04_cdev);
err_chrdev:
    unregister_chrdev_region(devt, 1);
    return ret;
}

/* -------------------------------------------------- */
/* Module exit                                        */
/* -------------------------------------------------- */
static void __exit hcsr04_exit(void)
{
    pr_info("hcsr04: unloading driver\n");

    free_irq(echo_irq, NULL);
    device_destroy(hcsr04_class, devt);
    class_destroy(hcsr04_class);
    cdev_del(&hcsr04_cdev);
    unregister_chrdev_region(devt, 1);
}

module_init(hcsr04_init);
module_exit(hcsr04_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Atlas");
MODULE_DESCRIPTION("HC-SR04 ultrasonic distance sensor driver");
