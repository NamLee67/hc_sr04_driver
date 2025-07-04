/* 
 * hc_sr04.c - Linux character device driver for HC-SR04 ultrasonic distance sensor
 *
 * This driver interfaces with the HC-SR04 ultrasonic sensor using GPIO pins to
 * measure distance. It supports single-process access, interrupt-driven measurement,
 * and provides distance data (in mm) through a character device file.
 *
 * Author: [Le Van Nam]
 */

#include <linux/cdev.h>
#include <linux/delay.h>
#include <linux/fs.h>
#include <linux/gpio.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/kdev_t.h>
#include <linux/module.h>
#include <linux/semaphore.h>
#include <linux/timekeeping.h>
#include <linux/uaccess.h>
#include <linux/wait.h>
#include <asm/atomic.h>

/* GPIO pin definitions */
#define GPIO_TRIGGER 26  /* GPIO pin for triggering measurement */
#define GPIO_ECHO 6      /* GPIO pin for receiving echo signal */

/* Device and class names */
#define DEVICE_NAME "hc_sr04"        /* Name of the device file in /dev */
#define CLASS_NAME "hc_sr04_class"   /* Name of the device class in /sys/class */

/* Global variables */
static struct cdev *hc_sr04_cdev;           /* Character device structure */
static int drv_major;                       /* Major device number */
static int gpio_irq_number;                 /* Interrupt number for ECHO pin */
static struct class *hc_sr04_class;         /* Device class */
static struct device *hc_sr04_device;       /* Device instance */
static wait_queue_head_t wait_for_echo;     /* Wait queue for echo signal */
static volatile int condition_echo;          /* Condition for wait queue */
static volatile ktime_t ktime_start;         /* Start time of echo pulse */
static volatile ktime_t ktime_end;           /* End time of echo pulse */
static ktime_t ktime_last_measurement;      /* Time of last measurement */
static atomic_t opened = ATOMIC_INIT(-1);   /* Atomic counter for single-process access */
static DECLARE_SEMAPHORE(read_semaphore);   /* Semaphore for read synchronization */

/*
 * gpio_echo_irq_handler - Interrupt handler for rising and falling edges of ECHO pin
 * @irq: Interrupt number
 * @dev_id: Device identifier (unused in this driver)
 *
 * Handles rising edge to start timing and falling edge to end timing,
 * waking up the read function when the echo pulse is complete.
 *
 * Return: IRQ_HANDLED to indicate interrupt was handled
 */
static irqreturn_t gpio_echo_irq_handler(int irq, void *dev_id)
{
    int gpio_value = gpio_get_value(GPIO_ECHO);

    if (gpio_value == 1) {
        /* Rising edge: start measuring time */
        ktime_start = ktime_get();
    } else if (gpio_value == 0) {
        /* Falling edge: store end time and wake up read function */
        ktime_end = ktime_get();
        condition_echo = 1;
        wake_up_interruptible(&wait_for_echo);
    }

    return IRQ_HANDLED;
}

/*
 * hc_sr04_read - Read handler to measure distance and return it to user space
 * @filp: File structure
 * @buf: User space buffer to store distance
 * @count: Number of bytes requested
 * @f_pos: File position offset
 *
 * Triggers a measurement, waits for the echo pulse, calculates the distance
 * in millimeters, and copies it to user space.
 *
 * Return: Number of bytes read, or negative error code
 */
static ssize_t hc_sr04_read(struct file *filp, char __user *buf, size_t count, loff_t *f_pos)
{
    ktime_t elapsed_time;
    unsigned int range_mm;
    long remaining_delay;

    /* Return EOF if file position is non-zero */
    if (*f_pos > 0)
        return 0;

    /* Ensure only one thread performs a measurement */
    if (down_interruptible(&read_semaphore))
        return -ERESTARTSYS;

    /* Ensure at least 60ms between measurements */
    if (ktime_to_ms(ktime_sub(ktime_get(), ktime_last_measurement)) < 60) {
        pr_warn("[HC-SR04] Too soon since last measurement: %lld ms\n",
                ktime_to_ms(ktime_sub(ktime_get(), ktime_last_measurement)));
        up(&read_semaphore);
        return -EBUSY;
    }

    ktime_last_measurement = ktime_get();
    condition_echo = 0;

    /* Trigger measurement: 10us pulse on TRIGGER pin */
    gpio_set_value(GPIO_TRIGGER, 1);
    udelay(10);
    gpio_set_value(GPIO_TRIGGER, 0);

    /* Wait for echo pulse with 100ms timeout */
    remaining_delay = wait_event_interruptible_timeout(wait_for_echo, condition_echo, HZ / 10);
    if (remaining_delay == 0) {
        pr_warn("[HC-SR04] Measurement timeout after 100ms\n");
        up(&read_semaphore);
        return -EAGAIN;
    }

    /* Calculate distance: range_mm = (time_us * speed_of_sound / 2) / 1000 */
    elapsed_time = ktime_sub(ktime_end, ktime_start);
    range_mm = (unsigned int)(ktime_to_us(elapsed_time) * 340u / 2u / 1000u);

    /* Copy distance to user space */
    if (copy_to_user(buf, &range_mm, sizeof(range_mm))) {
        up(&read_semaphore);
        return -EFAULT;
    }

    /* Update file position */
    *f_pos += sizeof(range_mm);

    up(&read_semaphore);
    return sizeof(range_mm);
}

/*
 * hc_sr04_open - Open handler to allow exclusive access to the device
 * @inode: Inode structure
 * @filp: File structure
 *
 * Ensures only one process can open the device at a time using atomic operations.
 *
 * Return: 0 on success, -EBUSY if device is already open
 */
static int hc_sr04_open(struct inode *inode, struct file *filp)
{
    if (atomic_inc_and_test(&opened))
        return 0;
    return -EBUSY;
}

/*
 * hc_sr04_release - Release handler to free the device
 * @inode: Inode structure
 * @filp: File structure
 *
 * Resets the atomic counter to allow another process to open the device.
 *
 * Return: 0 on success
 */
static int hc_sr04_release(struct inode *inode, struct file *filp)
{
    atomic_set(&opened, -1);
    return 0;
}

/* File operations structure */
static struct file_operations hc_sr04_fops = {
    .owner = THIS_MODULE,
    .read = hc_sr04_read,
    .open = hc_sr04_open,
    .release = hc_sr04_release,
};

/*
 * hc_sr04_init - Module initialization function
 *
 * Initializes the character device, GPIO pins, interrupt, and wait queue.
 *
 * Return: 0 on success, negative error code on failure
 */
static int __init hc_sr04_init(void)
{
    int result;
    dev_t dev = MKDEV(drv_major, 0);

    pr_info("[HC-SR04] Initializing HC-SR04 driver\n");

    /* Allocate major number */
    result = alloc_chrdev_region(&dev, 0, 1, DEVICE_NAME);
    if (result < 0) {
        pr_err("[HC-SR04] Failed to allocate character device region\n");
        return result;
    }
    drv_major = MAJOR(dev);
    pr_info("[HC-SR04] Registered with major number %d\n", drv_major);

    /* Create device class */
    hc_sr04_class = class_create(THIS_MODULE, CLASS_NAME);
    if (IS_ERR(hc_sr04_class)) {
        pr_err("[HC-SR04] Failed to create device class\n");
        unregister_chrdev_region(dev, 1);
        return PTR_ERR(hc_sr04_class);
    }

    /* Create device node */
    hc_sr04_device = device_create(hc_sr04_class, NULL, dev, NULL, DEVICE_NAME);
    if (IS_ERR(hc_sr04_device)) {
        pr_err("[HC-SR04] Failed to create device node\n");
        class_destroy(hc_sr04_class);
        unregister_chrdev_region(dev, 1);
        return PTR_ERR(hc_sr04_device);
    }

    /* Initialize character device */
    hc_sr04_cdev = cdev_alloc();
    if (!hc_sr04_cdev) {
        pr_err("[HC-SR04] Failed to allocate cdev\n");
        device_destroy(hc_sr04_class, dev);
        class_destroy(hc_sr04_class);
        unregister_chrdev_region(dev, 1);
        return -ENOMEM;
    }
    hc_sr04_cdev->ops = &hc_sr04_fops;
    result = cdev_add(hc_sr04_cdev, dev, 1);
    if (result < 0) {
        pr_err("[HC-SR04] Failed to add cdev\n");
        cdev_del(hc_sr04_cdev);
        device_destroy(hc_sr04_class, dev);
        class_destroy(hc_sr04_class);
        unregister_chrdev_region(dev, 1);
        return result;
    }

    /* Configure TRIGGER GPIO */
    if (!gpio_is_valid(GPIO_TRIGGER)) {
        pr_err("[HC-SR04] Invalid GPIO %d for TRIGGER\n", GPIO_TRIGGER);
        goto err_cdev;
    }
    if (gpio_request(GPIO_TRIGGER, "GPIO_TRIGGER") < 0) {
        pr_err("[HC-SR04] Failed to request GPIO %d for TRIGGER\n", GPIO_TRIGGER);
        goto err_cdev;
    }
    gpio_direction_output(GPIO_TRIGGER, 0);
    gpio_set_value(GPIO_TRIGGER, 0);

    /* Configure ECHO GPIO */
    if (!gpio_is_valid(GPIO_ECHO)) {
        pr_err("[HC-SR04] Invalid GPIO %d for ECHO\n", GPIO_ECHO);
        goto err_trigger;
    }
    if (gpio_request(GPIO_ECHO, "GPIO_ECHO") < 0) {
        pr_err("[HC-SR04] Failed to request GPIO %d for ECHO\n", GPIO_ECHO);
        goto err_trigger;
    }
    gpio_direction_input(GPIO_ECHO);

    /* Request interrupt for ECHO pin */
    gpio_irq_number = gpio_to_irq(GPIO_ECHO);
    if (gpio_irq_number < 0) {
        pr_err("[HC-SR04] Failed to get IRQ for GPIO %d\n", GPIO_ECHO);
        goto err_echo;
    }
    result = request_irq(gpio_irq_number, gpio_echo_irq_handler,
                        IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
                        "hc-sr04", NULL);
    if (result) {
        pr_err("[HC-SR04] Failed to request IRQ %d\n", gpio_irq_number);
        goto err_echo;
    }

    /* Initialize wait queue and measurement time */
    init_waitqueue_head(&wait_for_echo);
    ktime_last_measurement = ktime_set(0, 0);

    return 0;

err_echo:
    gpio_free(GPIO_ECHO);
err_trigger:
    gpio_free(GPIO_TRIGGER);
err_cdev:
    cdev_del(hc_sr04_cdev);
    device_destroy(hc_sr04_class, dev);
    class_destroy(hc_sr04_class);
    unregister_chrdev_region(dev, 1);
    return result < 0 ? result : -1;
}

/*
 * hc_sr04_exit - Module cleanup function
 *
 * Frees all resources allocated during initialization.
 */
static void __exit hc_sr04_exit(void)
{
    dev_t dev = MKDEV(drv_major, 0);

    /* Remove character device */
    cdev_del(hc_sr04_cdev);

    /* Remove device node and class */
    device_destroy(hc_sr04_class, dev);
    class_destroy(hc_sr04_class);
    unregister_chrdev_region(dev, 1);

    /* Free GPIO and interrupt resources */
    gpio_set_value(GPIO_TRIGGER, 0);
    gpio_free(GPIO_TRIGGER);
    free_irq(gpio_irq_number, NULL);
    gpio_free(GPIO_ECHO);

    pr_info("[HC-SR04] Driver unloaded\n");
}

module_init(hc_sr04_init);
module_exit(hc_sr04_exit);

MODULE_DESCRIPTION("Linux device driver for HC-SR04 ultrasonic distance sensor");
MODULE_AUTHOR("[Le Van Nam]");
MODULE_LICENSE("GPL");