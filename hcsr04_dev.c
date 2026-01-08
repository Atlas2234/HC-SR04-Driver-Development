#include <linux/module.h>
#include <linux/init.h>
#include <linux/kernel.h>

#include <linux/types.h>
#include <linux/kdev_t.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <asm/uaccess.h>

#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/kobject.h>
#include <linux/sysfs.h>

#include <linux/time.h>
#include <linux/ktime.h>
#include <asm/delay.h>
#include <linux/delay.h>

#define GPIO_OUT        20                      // GPIO20 trigger pin
#define GPIO_IN         21                      // GPIO21 echo pin

static dev_t hcsr04_dev;

struct cdev hcsr04_cdev;

// Lock variable to prevent multiple access to device
static int hcsr04_lock = 0;

static struct kobject *hcsr04_kobject;

static ktime_t rising, falling;

// File operation callback functions for our character device
int hcsr04_open(struct inode *inode, struct file *file)
{
    int ret = 0;
    printk( KERN_INFO "hcsr04_dev: %s\n", __func__ );

    // Check if device is locked
    if( hcsr04_lock > 0 )
    {
        ret = -EBUSY;
    }
    // Lock device if not locked
    else
        hcsr04_lock++;

   return( ret );
}

int hcsr04_close(struct inode *inode, struct file *file)
{
    printk( KERN_INFO "hcsr04_dev: %s\n", __func__ );

    // Unlock device when closed
    hcsr04_lock = 0;

    return( 0 );
}

ssize_t hcsr04_read(struct file *filp, char __user *buf, size_t count, loff_t *f_pos)
{
    int ret;
    int pulse;

    printk( KERN_INFO "hcsr04_dev: %s\n", __func__ );

    // Calculate pulse width in microseconds
    pulse = (int)ktime_to_us( ktime_sub( falling, rising ) );

    // Copy pulse width to user space
    ret = copy_to_user( buf, &pulse, 4 );

    return 4;
}

ssize_t hcsr04_write(struct file *filp, const char *buffer, size_t length, loff_t * offset)
{
    printk( KERN_INFO "hcsr04_dev: %s\n", __func__ );

    // Send 10us pulse to trigger pin
    gpio_set_value( GPIO_OUT, 0 );
    gpio_set_value( GPIO_OUT, 1 );
    udelay( 10 );
    gpio_set_value( GPIO_OUT, 0 );

    // Wait for echo pin to go high and then low
    // Rising edge
    while( gpio_get_value( GPIO_IN ) == 0 );
    // Record time of rising edge
    rising = ktime_get();
    // Falling edge
    while( gpio_get_value( GPIO_IN ) == 1 );
    // Record time of falling edge
    falling = ktime_get();

    return( 1 );
}

// Define file operations structure
struct file_operations hcsr04_fops = {
    .owner = THIS_MODULE,
    .read = hcsr04_read,
    .write = hcsr04_write,
    .open = hcsr04_open,
    .release = hcsr04_close,
};

// Module initialization function
static int __init hcsr04_module_init(void)
{
    char buffer[64];
    int ret = 0;

    printk(KERN_INFO "Loading hcsr04_module\n");

    // Allocate character device region
    alloc_chrdev_region(&hcsr04_dev, 0, 1, "hcsr04_dev");
    printk(KERN_INFO "%s\n", format_dev_t(buffer, hcsr04_dev));
    // Initialize character device
    cdev_init(&hcsr04_cdev, &hcsr04_fops);
    // Set owner of the device
    hcsr04_cdev.owner = THIS_MODULE;
    // Add the character device to the system
    cdev_add(&hcsr04_cdev, hcsr04_dev, 1);

    // Request GPIO_OUT
    if( gpio_request( GPIO_OUT, "hcsr04_dev" ) )
    {
        printk( KERN_INFO "hcsr04_dev: %s unable to get GPIO_OUT\n", __func__ );
        ret = -EBUSY;
        goto Done;
    }
    // Request GPIO_IN
    if( gpio_request( GPIO_IN, "hcsr04_dev" ) )
    {
        printk( KERN_INFO "hcsr04_dev: %s unable to get GPIO_IN\n", __func__ );
        ret = -EBUSY;
        goto Done;
    }

    // Set GPIO output direction
    if( gpio_direction_output( GPIO_OUT, 0 ) < 0 )
    {
        printk( KERN_INFO "hcsr04_dev: %s unable to set GPIO_OUT as output\n", __func__ );
        ret = -EBUSY;
        goto Done;
    }

    // Set GPIO input direction
    if( gpio_direction_input( GPIO_IN ) < 0 )
    {
        printk( KERN_INFO "hcsr04_dev: %s unable to set GPIO_IN as input\n", __func__ );
        ret = -EBUSY;
        goto Done;
    }

Done:
    return ret;
}

// Module cleanup function
static void __exit hcsr04_module_cleanup(void)
{
    printk(KERN_INFO "Cleaning-up hcsr04_dev.\n");

    gpio_free( GPIO_OUT );
    gpio_free( GPIO_IN );

    hcsr04_lock = 0;

    // Remove character device and unregister device region
    cdev_del(&hcsr04_cdev);
    unregister_chrdev_region( hcsr04_dev, 1 );
}

// Specify module initialization and cleanup functions
module_init(hcsr04_module_init);
module_exit(hcsr04_module_cleanup);

// Module metadata
MODULE_LICENSE("GPL"); 
MODULE_AUTHOR("Atlas");
MODULE_DESCRIPTION("HC-SR04 Ultrasonic Sensor Driver");






// Test code for user space application to interact with the driver
Test:
#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>

int main(int argc, char **argv)
{   
    // Set application and character device names
    char *app_name = argv[0];
    char *dev_name = "/dev/hcsr04"; 
    int fd = -1;     
    char c;   
    int d;

    // Open character device
    if( (fd = open(dev_name, O_RDWR)) < 0 )
    {
        fprintf(stderr, "%s: unable to open %s: %s\n", app_name, dev_name, strerror(errno));           
        return( 1 ); 
    }

    c = 1;
    // Write to device to trigger measurement
    write( fd, &c, 1 );

    // Read measurement result from device
    read( fd, &d, 4 );

    // Print distance in cm (pulse width divided by 58)
    printf( "%d: %f\n", d, d/58.0 );

    // Close character device
    close( fd );

    return 0;
}
