#ifndef ULTRASONIC_SENSOR_H
#define ULTRASONIC_SENSOR_H

#include <linux/types.h>
#include <linux/kfifo.h>
#include <linux/spinlock.h>
#include <linux/wait.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/workqueue.h>

#define EL_PRINT_E(string, arg...) printk(KERN_ERR "[ultrasonic] : (%s) : " string "\n", __FUNCTION__, ##arg)
#define EL_PRINT_W(string, arg...) printk(KERN_WARNING "[ultrasonic] : (%s) : " string "\n", __FUNCTION__, ##arg)
#define EL_PRINT_I(string, arg...) printk(KERN_INFO "[ultrasonic] : (%s) : " string "\n", __FUNCTION__, ##arg)
#define EL_PRINT_D(string, arg...) printk(KERN_DEBUG "[ultrasonic] : (%s) : " string "\n", __FUNCTION__, ##arg)


#define ULTRASONIC_DEVICENAME "ultrasonic"
#define ULTRASONIC_NUM_DEVICES 2

#define IOCTL_ULTRASONIC_APP    197
#define MIRROR_TAG      0x3D0A4842

#define IOCTL_ULTRASONIC_DATA_IO_CANCEL _IO(IOCTL_ULTRASONIC_APP, 2)
#define IOCTL_ULTRASONIC_ACTIVATE_ENGINE _IOW(IOCTL_ULTRASONIC_APP, 3, int)
#define IOCTL_ULTRASONIC_SET_RAMP_DOWN _IO(IOCTL_ULTRASONIC_APP, 4)
#define IOCTL_ULTRASONIC_SYSTEM_CONFIGURATION _IOW(IOCTL_ULTRASONIC_APP, 5, int)
#define IOCTL_ULTRASONIC_DATA_IO_MIRROR _IOW(IOCTL_ULTRASONIC_APP, 117, unsigned char *)


#define ULTRASONIC_DATA_IO_AP_TO_DSP 0
#define ULTRASONIC_DATA_IO_DSP_TO_AP 1

#define ULTRASONIC_DATA_IO_READ_OK 0
#define ULTRASONIC_DATA_IO_READ_BUSY 1
#define ULTRASONIC_DATA_IO_READ_CANCEL 2

#define ULTRASONIC_MSG_BUF_SIZE 512

/* wake source timeout in ms*/
#define ULTRASONIC_WAKEUP_TIMEOUT 250

#define ULTRASONIC_DATA_FIFO_SIZE (PAGE_SIZE)


struct ultrasonic_data {
    unsigned int wakeup_timeout; /* wake lock timeout */

    /* members for top half interrupt handling */
    struct kfifo fifo_isr;
    spinlock_t fifo_isr_spinlock;

    /* buffer to swap data from isr fifo to userspace fifo */
    uint8_t isr_swap_buffer[ULTRASONIC_MSG_BUF_SIZE];

    /* members for bottom half handling */
    struct kfifo fifo_userspace;
    struct mutex fifo_usp_lock;
    wait_queue_head_t fifo_usp_not_empty;

    atomic_t abort_io;
    struct work_struct work;
    struct workqueue_struct *wq;

    /* debug counters, reset between open/close */
    uint32_t isr_fifo_flush_count;
    uint32_t userspace_fifo_flush_count;

    /* debug counters, persistent */
    uint32_t isr_fifo_flush_count_total;
    uint32_t userspace_fifo_flush_count_total;
    uint32_t userspace_read_total;
    uint32_t isr_write_total;

};

struct ultrasonic_device {
    int opened;
    struct cdev cdev;
    struct semaphore sem;
    struct ultrasonic_data el_data;
};


#endif
