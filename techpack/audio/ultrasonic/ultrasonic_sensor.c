/**
* Copyright Nubia
*
*/
#define DEBUG

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/version.h>
/*  includes the file structure, that is, file open read close */
#include <linux/fs.h>


#include <dsp/msm-ultrasound.h>
#include <dsp/apr_audio-v2.h>
/* include the character device, makes cdev avilable */
#include <linux/cdev.h>
#include <linux/semaphore.h>

/* includes copy_user vice versa */
#include <linux/uaccess.h>
#include <linux/spinlock.h>
#include <linux/slab.h>
#include <linux/stat.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/types.h>
#include <linux/kdev_t.h>
#include <linux/device.h>

#include <linux/pm_wakeup.h>
#include <linux/kfifo.h>
#include <linux/poll.h>
#include <linux/kobject.h>

#include "ultrasonic_sensor.h"

static struct ultrasonic_device *ultrasonic_devices;

/* Global variable for the device class*/
struct class *ultrasonic_class;


/* Major number provided by the kernel*/
static dev_t ultrasonic_major;

static struct wakeup_source *wake_source;


#define ULTRASONIC_SYSFS_ENGINE_FOLDER "engine"
#define ULTRASONIC_SYSFS_ROOT_FOLDER "ultrasonic"
#define ULTRASONIC_SYSFS_CALIBRATION_FILENAME "calibration"
#define ULTRASONIC_SYSFS_VERSION_FILENAME "version"


static ssize_t calibration_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    struct nubia_system_configuration_parameter param;
    uint32_t param_id = NUBIA_ULTRASOUND_SET_PARAMS;
    u8 cmd = simple_strtoul(buf, NULL, 10 );

    param.type = ESCPT_CALIBRATION_STATE;
    param.calibration_state = cmd;

    EL_PRINT_D("set ultrasonic calibration state: %d\n", cmd);

    return ultrasound_apr_set(NUBIA_PORT_ID, &param_id, (u8 *)&param, sizeof(param));
}

static ssize_t calibration_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    ssize_t result = 0;
    uint8_t *caldata;
    
    struct nubia_shared_data_block *calibration_obj = nubia_get_shared_obj(NUBIA_OBJ_ID_CALIBRATION_DATA);

    if (NULL == calibration_obj) {
        EL_PRINT_E("calibration_obj is NULL");
        return -EINVAL;
    }

    if (calibration_obj->size > PAGE_SIZE) {
        EL_PRINT_E("calibration_obj->size > PAGE_SIZE");
        return -EINVAL;
    }

    caldata = (uint8_t *)calibration_obj->buffer;

    result = snprintf(buf, NUBIA_CALIBRATION_DATA_SIZE+1, caldata);
    
    EL_PRINT_D("calibration data size:%ld\n",calibration_obj->size);
    
    return result;
}


static ssize_t version_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    ssize_t result;
    struct nubia_engine_version_info *version_info;
    int length;

    struct nubia_shared_data_block *version_obj = nubia_get_shared_obj(NUBIA_OBJ_ID_VERSION_INFO);

    if (NULL == version_obj) {
        EL_PRINT_E("version_obj is NULL");
        return -EINVAL;
    }

    if (version_obj->size > PAGE_SIZE) {
        EL_PRINT_E("version_obj->size > PAGE_SIZE");
        return -EINVAL;
    }

    version_info = (struct nubia_engine_version_info *)version_obj->buffer;
    length = snprintf(buf, PAGE_SIZE, "%d.%d.%d.%d\n", version_info->major, version_info->minor, version_info->build, version_info->revision);
    result = (ssize_t)length;
    
    return result;
}


static struct device_attribute calibration_attr = __ATTR_RW(calibration);
static struct device_attribute version_attr = __ATTR_RO(version);

static struct attribute *ultrasonic_attrs[] = {
    &calibration_attr.attr,
    &version_attr.attr,
    NULL,
};

static struct attribute_group ultrasonic_attr_group = {
    .name = ULTRASONIC_SYSFS_ENGINE_FOLDER,
    .attrs = ultrasonic_attrs,
};

static struct kobject *ultrasonic_sysfs_kobj;

int ultrasonic_initialize_sysfs(void)
{
    int err;

    ultrasonic_sysfs_kobj = kobject_create_and_add(ULTRASONIC_SYSFS_ROOT_FOLDER, kernel_kobj->parent);

    if (!ultrasonic_sysfs_kobj) {
        EL_PRINT_E("failed to create kobj");
        return -ENOMEM;
    }

    err = sysfs_create_group(ultrasonic_sysfs_kobj, &ultrasonic_attr_group);

    if (err) {
        EL_PRINT_E("failed to create sysfs group");
        kobject_put(ultrasonic_sysfs_kobj);
        return -ENOMEM;
    }

    return 0;
}

void ultrasonic_cleanup_sysfs(void)
{
    kobject_put(ultrasonic_sysfs_kobj);
}

void ultrasonic_data_reset_debug_counters(struct ultrasonic_data *ultrasonic_data)
{
    ultrasonic_data->isr_fifo_flush_count = 0;
    ultrasonic_data->userspace_fifo_flush_count = 0;
}

void ultrasonic_data_print_debug_counters(struct ultrasonic_data *ultrasonic_data)
{
    if (ultrasonic_data->isr_fifo_flush_count > 0) {
        EL_PRINT_E("isr fifo flushed %u times", ultrasonic_data->isr_fifo_flush_count);
    }

    if (ultrasonic_data->userspace_fifo_flush_count > 0) {
        EL_PRINT_E("userspace fifo flushed %u times", ultrasonic_data->userspace_fifo_flush_count);
    }

    if (ultrasonic_data->userspace_read_total != ultrasonic_data->isr_write_total) {
        EL_PRINT_I("user space reads / isr writes : %u / %u", ultrasonic_data->userspace_read_total, ultrasonic_data->isr_write_total);
    }

    EL_PRINT_I("total isr fifo flushed count : %u, total userspace fifo flushed count : %u\n",
    ultrasonic_data->isr_fifo_flush_count_total, ultrasonic_data->userspace_fifo_flush_count_total);
}

void ultrasonic_data_update_debug_counters(struct ultrasonic_data *ultrasonic_data)
{
    ultrasonic_data->isr_fifo_flush_count_total += ultrasonic_data->isr_fifo_flush_count;
    ultrasonic_data->userspace_fifo_flush_count_total += ultrasonic_data->userspace_fifo_flush_count;
}


/* spin lock for isr must be held prior to calling */
static void ultrasonic_data_flush_isr_fifo(struct ultrasonic_data *ultrasonic_data)
{
    kfifo_reset(&ultrasonic_data->fifo_isr);
}

/* mutex lock for user space copy must be held prior to calling */
static void ultrasonic_data_flush_userspace_fifo(struct ultrasonic_data *ultrasonic_data)
{
    kfifo_reset(&ultrasonic_data->fifo_userspace);
}

/* inode refers to the actual file on disk */
static int device_open(struct inode *inode, struct file *filp)
{
    unsigned int major;
    unsigned int minor;
    struct ultrasonic_device *dev;
    struct ultrasonic_data *ultrasonic_data;

    major = imajor(inode);
    minor = iminor(inode);

    if (major != ultrasonic_major || minor < 0 || minor >= ULTRASONIC_NUM_DEVICES) {
        EL_PRINT_W("no device found with minor=%d and major=%d", major, minor);
        return -ENODEV; /* No such device */
    }

    dev = NULL;
    dev = &ultrasonic_devices[minor];
    filp->private_data = dev;

    if (inode->i_cdev != &dev->cdev) {
        EL_PRINT_W("dev pointer mismatch");
        return -ENODEV; /* No such device */
    }

    if (down_interruptible(&dev->sem) != 0) {
        EL_PRINT_E("the device has been opened by some other device, unable to open lock");
        return -EINVAL;
    }

    ultrasonic_data = &dev->el_data;
    spin_lock(&ultrasonic_data->fifo_isr_spinlock);
    ultrasonic_data_flush_isr_fifo(ultrasonic_data);
    spin_unlock(&ultrasonic_data->fifo_isr_spinlock);

    mutex_lock(&ultrasonic_data->fifo_usp_lock);
    ultrasonic_data_flush_userspace_fifo(ultrasonic_data);
    mutex_unlock(&ultrasonic_data->fifo_usp_lock);

    ultrasonic_data_reset_debug_counters(ultrasonic_data);

    EL_PRINT_I("Opened device ultrasonic%u", minor);
    dev->opened = 1;
    return 0;
}

static void ultrasonic_data_work_handler(struct work_struct *ws)
{
    struct ultrasonic_data *ultrasonic_data;
    unsigned long flags;

    unsigned int fifo_result = 0;
    size_t available_space = 0;
    ultrasonic_data = container_of(ws, struct ultrasonic_data, work);

    if (kfifo_is_empty(&ultrasonic_data->fifo_isr)) {
        EL_PRINT_W("work handler called when isr fifo is empty");
        return;
    }

    mutex_lock(&ultrasonic_data->fifo_usp_lock);

    spin_lock_irqsave(&ultrasonic_data->fifo_isr_spinlock, flags);

    fifo_result = kfifo_out(&ultrasonic_data->fifo_isr, ultrasonic_data->isr_swap_buffer, ULTRASONIC_MSG_BUF_SIZE);
    if (fifo_result == 0) {
        EL_PRINT_E("failed to copy from fifo isr to swap buffer %u", fifo_result);
        goto fail;
    }

    available_space = kfifo_avail(&ultrasonic_data->fifo_userspace);

    if (ULTRASONIC_MSG_BUF_SIZE > available_space) {
        EL_PRINT_E("user space fifo available_space %lu not enough to push entry_size %lu. Flushing user space fifo", available_space, (size_t)ULTRASONIC_MSG_BUF_SIZE);
        ++ultrasonic_data->userspace_fifo_flush_count;
        ultrasonic_data_flush_userspace_fifo(ultrasonic_data);
        goto fail;
    }

    fifo_result = kfifo_in(&ultrasonic_data->fifo_userspace, ultrasonic_data->isr_swap_buffer, ULTRASONIC_MSG_BUF_SIZE);
    if (fifo_result == 0) {
        EL_PRINT_E("failed to copy from swap to fifo user space: %u", fifo_result);
        goto fail;
    }

    spin_unlock_irqrestore(&ultrasonic_data->fifo_isr_spinlock, flags);
    mutex_unlock(&ultrasonic_data->fifo_usp_lock);
    wake_up_interruptible(&ultrasonic_data->fifo_usp_not_empty);
    __pm_wakeup_event(wake_source, ultrasonic_data->wakeup_timeout);
    return;

    fail:
    spin_unlock_irqrestore(&ultrasonic_data->fifo_isr_spinlock, flags);
    mutex_unlock(&ultrasonic_data->fifo_usp_lock);

}

#define WORK_QUEUE_HANDLER_NAME_LENGTH 64
int ultrasonic_data_initialize(struct ultrasonic_data *ultrasonic_data, size_t queue_size, unsigned int wakeup_timeout, int id)
{
    int is_power_of_two;

    char name[WORK_QUEUE_HANDLER_NAME_LENGTH] = {0};
    is_power_of_two = (queue_size != 0) && !(queue_size & (queue_size - 1));

    if (is_power_of_two != 1) {
        EL_PRINT_E("data initialize with non power of 2 fifo size");
        return -EINVAL;
    }

    if (kfifo_alloc(&ultrasonic_data->fifo_isr, queue_size, GFP_KERNEL) != 0) {
        EL_PRINT_E("data initialize failed to allocate fifo isr");
        return -EINVAL;
    }

    if (kfifo_alloc(&ultrasonic_data->fifo_userspace, queue_size, GFP_KERNEL) != 0) {
        EL_PRINT_E("data initialize failed to allocate fifo user space");
        return -EINVAL;
    }


    atomic_set(&ultrasonic_data->abort_io, 0);
    spin_lock_init(&ultrasonic_data->fifo_isr_spinlock);

    INIT_WORK(&ultrasonic_data->work, ultrasonic_data_work_handler);
    mutex_init(&ultrasonic_data->fifo_usp_lock);
    init_waitqueue_head(&ultrasonic_data->fifo_usp_not_empty);

    snprintf(name, WORK_QUEUE_HANDLER_NAME_LENGTH, "%s_%d", "ULTRASONIC_DATA_WORK_HANDLER", id);
    ultrasonic_data->wq = create_singlethread_workqueue(name);

    return 0;
}

int ultrasonic_data_cleanup(struct ultrasonic_data *ultrasonic_data)
{
    spin_unlock(&ultrasonic_data->fifo_isr_spinlock);
    kfifo_free(&ultrasonic_data->fifo_isr);
    return 0;
}

size_t ultrasonic_data_pop(struct ultrasonic_data *ultrasonic_data, char __user *buffer, size_t buffer_size)
{
    int result;
    unsigned int num_copied;

    if (buffer_size < ULTRASONIC_MSG_BUF_SIZE) {
        EL_PRINT_E("buffer_size : %lu smaller than ULTRASONIC_MSG_BUF_SIZE : %lu", buffer_size, (size_t)ULTRASONIC_MSG_BUF_SIZE);
        return 0;
    }

    result = wait_event_interruptible(ultrasonic_data->fifo_usp_not_empty,
                                (kfifo_is_empty(&ultrasonic_data->fifo_userspace) == 0) || (atomic_read(&ultrasonic_data->abort_io) == 1));

    if (atomic_read(&ultrasonic_data->abort_io) == 1) {
        atomic_set(&ultrasonic_data->abort_io, 0);
        EL_PRINT_D("pop cancelled");
        return 0;
    }


    if (result == 0) {
        mutex_lock(&ultrasonic_data->fifo_usp_lock);

        num_copied = 0;
        result = kfifo_to_user(&ultrasonic_data->fifo_userspace, buffer, ULTRASONIC_MSG_BUF_SIZE, &num_copied);

        if (result == -EFAULT) {
            EL_PRINT_E("failed kfifo_to_user");
            mutex_unlock(&ultrasonic_data->fifo_usp_lock);
            return 0;
        }

        mutex_unlock(&ultrasonic_data->fifo_usp_lock);
        ++ultrasonic_data->userspace_read_total;

        if ((size_t)num_copied != ULTRASONIC_MSG_BUF_SIZE) {
            EL_PRINT_E("fifo_copied_to_user less than entry size : %u", num_copied);
            return (size_t)num_copied;
        }
    } else {
        if (-ERESTARTSYS == result)
            EL_PRINT_D("wait interrupted");
        else
            EL_PRINT_E("wait error = %d", result);
    }

    return (size_t)ULTRASONIC_MSG_BUF_SIZE;
}

int ultrasonic_data_push(const char *buffer, size_t buffer_size)
{
    size_t available_space;
    size_t space_required;
    size_t zeros_to_pad;
    int err;
    int i;
    unsigned long flags;
    struct ultrasonic_device *device;
    struct ultrasonic_data *ultrasonic_data;
    unsigned int fifo_result;
    static uint8_t zero_pad_buffer[ULTRASONIC_MSG_BUF_SIZE];

    err = 0;
    fifo_result = 0;

    if (buffer_size > ULTRASONIC_MSG_BUF_SIZE) {
        EL_PRINT_E("buffer size %lu is larger than max buffer size %lu", buffer_size, (size_t)ULTRASONIC_MSG_BUF_SIZE);
        return -EINVAL;
    }

    zeros_to_pad = ULTRASONIC_MSG_BUF_SIZE - buffer_size;

    for (i = 0; i < ULTRASONIC_NUM_DEVICES; ++i) {
        device = &ultrasonic_devices[i];
        ultrasonic_data = &device->el_data;

        if ((!device->opened))
            continue;

        available_space = kfifo_avail(&ultrasonic_data->fifo_isr);
        space_required = ULTRASONIC_MSG_BUF_SIZE;

        spin_lock_irqsave(&ultrasonic_data->fifo_isr_spinlock, flags);

        if (available_space < space_required) {
            EL_PRINT_W("not enough available space: %lu, flushing fifo contents", available_space);

            ++ultrasonic_data->isr_fifo_flush_count;
            ultrasonic_data_flush_isr_fifo(ultrasonic_data);
        }

        fifo_result = kfifo_in(&ultrasonic_data->fifo_isr, buffer, buffer_size);
        if (fifo_result == 0) {
            EL_PRINT_W("failed to push buffer to fifo");
            spin_unlock_irqrestore(&ultrasonic_data->fifo_isr_spinlock, flags);
            continue;
        }

        if (zeros_to_pad > 0) {
            fifo_result = kfifo_in(&ultrasonic_data->fifo_isr, zero_pad_buffer, zeros_to_pad);
            if (fifo_result == 0) {
                EL_PRINT_W("failed to push zero pad to fifo, flushing fifo");
                spin_unlock_irqrestore(&ultrasonic_data->fifo_isr_spinlock, flags);

                ++ultrasonic_data->isr_fifo_flush_count;
                ultrasonic_data_flush_isr_fifo(ultrasonic_data);
                continue;
            }
        }

        ++ultrasonic_data->isr_write_total;
        spin_unlock_irqrestore(&ultrasonic_data->fifo_isr_spinlock, flags);

        queue_work(ultrasonic_data->wq, &ultrasonic_data->work);
    }

    return err;
}

int32_t ultrasonic_data_io_write(uint32_t message_id, const char *data, size_t data_size)
{
    uint32_t port_id;

    port_id = SLIMBUS_1_TX;
    return ultrasound_apr_set(port_id, &message_id, (u8 *)data, (int32_t)data_size);
}

/**
*
* @return Number of bytes read.
*/
static ssize_t device_read(struct file *fp, char __user *buff, size_t length, loff_t *ppos)
{
    ssize_t bytes_read = 0;
    struct ultrasonic_device *ultrasonic_device;
    struct ultrasonic_data *ultrasonic_data;

    ultrasonic_device = (struct ultrasonic_device *)fp->private_data;
    ultrasonic_data = (struct ultrasonic_data *)&ultrasonic_device->el_data;

    bytes_read = ultrasonic_data_pop(ultrasonic_data, buff, length);

    return bytes_read;
}

/**
*
* @return number of bytes actually written
*/
static ssize_t device_write(struct file *fp, const char *buff, size_t length, loff_t *ppos)
{
    ssize_t ret_val;

    ret_val = 0;
    if ((buff != NULL) && (length != 0))
        ret_val = ultrasonic_data_io_write(NUBIA_ULTRASOUND_SET_PARAMS, buff, length);

    return ret_val >= 0 ? (ssize_t)length : 0;
}


static long device_ioctl(struct file *fp, unsigned int number, unsigned long param)
{
#if 0
    struct ultrasonic_device *device;
    struct ultrasonic_data *ultrasonic_data;
    int err;
    unsigned int mirror_tag, mirror_payload_size;
    unsigned char *data_ptr;

    device = (struct ultrasonic_device *)(fp->private_data);
    ultrasonic_data = &device->el_data;

    switch (number) {
    case IOCTL_ULTRASONIC_DATA_IO_CANCEL:
        EL_PRINT_D("IOCTL_ULTRASONIC_CANCEL_READ %ld", param);
        ultrasonic_data_io_cancel(ultrasonic_data);
        break;

    case IOCTL_ULTRASONIC_DATA_IO_MIRROR:
        data_ptr = (unsigned char *) param;
        mirror_tag = *(unsigned int *) data_ptr;
        mirror_payload_size = *((unsigned int *) data_ptr + 1);

        if ((mirror_tag == MIRROR_TAG) &&
            (mirror_payload_size != 0) &&
            (mirror_payload_size <= (NUBIA_SET_PARAMS_SIZE * 4))) {
            pr_debug(" ultrasonic : () IOCTL_ULTRASONIC_DATA_IO_MIRROR Tag=%x, len=%d\n", mirror_tag, mirror_payload_size);
            err = ultrasonic_data_io_write(NUBIA_ULTRASOUND_SET_PARAMS, (data_ptr + 8), mirror_payload_size);

            if (err != 0) {
                EL_PRINT_E("ultrasonic_data_io_write failed");
                return err;
            }

            /* ultrasonic_data_io(ULTRASONIC_ULTRASOUND_SET_PARAMS, ULTRASONIC_PORT_ID, (data_ptr + 8), mirror_payload_size);*/
        } else {
            pr_debug(" ultrasonic : () IOCTL_ULTRASONIC_DATA_IO_MIRROR, TAG or Length is not valid\n");
        }

        break;

    default:
        pr_debug(" ultrasonic : () UNKNOWN IOCTL number=%d\n", number);
        break;
    }
#endif
//wangweiping delete for test
    return 0;
}


static unsigned int device_poll(struct file *file, struct poll_table_struct *poll_table)
{
        unsigned int mask;

        struct ultrasonic_device *device;
        struct ultrasonic_data *ultrasonic_data;

        mask = 0;
        device = (struct ultrasonic_device *)file->private_data;
        ultrasonic_data = (struct ultrasonic_data *)&device->el_data;

        poll_wait(file, &ultrasonic_data->fifo_usp_not_empty, poll_table);

        if (!kfifo_is_empty(&ultrasonic_data->fifo_userspace))
            mask = POLLIN | POLLRDNORM;

        return mask;
}


static int device_close(struct inode *inode, struct file *filp)
{
    struct ultrasonic_device *device;
    struct ultrasonic_data *ultrasonic_data;
    unsigned int minor;

    device = filp->private_data;
    ultrasonic_data = &device->el_data;
    minor = iminor(inode);
    if (device == NULL) {
        EL_PRINT_E("device not found");
        return -ENODEV;
    }

    device->opened = 0;
    ultrasonic_data_update_debug_counters(ultrasonic_data);
    ultrasonic_data_print_debug_counters(ultrasonic_data);

    up(&device->sem);

    EL_PRINT_I("Closed device ultrasonic%u", minor);
    return 0;
}

/* defines the file operations provided by the driver */
static const struct file_operations ultrasonic_fops = {
    .owner = THIS_MODULE, /* prevents unloading when operations are in use*/
    .open = device_open,  /*to open the device*/
    .write = device_write, /*to write to the device*/
    .read = device_read, /*to read the device*/
    .poll = device_poll,
    .unlocked_ioctl = device_ioctl, /* IOCTL calls */
    .release = device_close, /*to close the device*/
};


static int ultrasonic_device_initialize(struct ultrasonic_device *ultrasonic_device, int minor, struct class *class)
{
    int err;
    dev_t device_number;
    struct device *device;

    BUG_ON(ultrasonic_device == NULL || class == NULL);

    EL_PRINT_D(" enter\n");

    err = 0;
    device = NULL;
    device_number = MKDEV(ultrasonic_major, minor);
    /* Memory is to be allocated when the device is opened the first time */
    sema_init(&ultrasonic_device->sem, 1);
    cdev_init(&ultrasonic_device->cdev, &ultrasonic_fops);
    ultrasonic_device->cdev.owner = THIS_MODULE;

    err = cdev_add(&ultrasonic_device->cdev, device_number, 1);

    if (err) {
        EL_PRINT_E("error %d while trying to add %s%d", err, ULTRASONIC_DEVICENAME, minor);
        return err;
    }

    device = device_create(class, NULL, device_number, NULL, ULTRASONIC_DEVICENAME "%d", minor);

    if (IS_ERR(device)) {
        err = PTR_ERR(device);
        EL_PRINT_E("error %d while trying to create %s%d", err, ULTRASONIC_DEVICENAME, minor);
        cdev_del(&ultrasonic_device->cdev);
        return err;
    }

    if (err) {
        EL_PRINT_E("failed device initialize");
        return err;
    }

    return 0;
}

static void ultrasonic_device_cleanup(struct ultrasonic_device *dev, int minor, struct class *class)
{
    BUG_ON(dev == NULL || class == NULL);
    device_destroy(class, MKDEV(ultrasonic_major, minor));
    cdev_del(&dev->cdev);
    up(&dev->sem);
}

static void ultrasonic_driver_cleanup(int devices_to_destroy)
{
    int i;

    if (ultrasonic_devices) {
        //ultrasonic_data_io_cleanup();  //wangweiping delete for test

        for (i = 0; i < devices_to_destroy; ++i) {
            ultrasonic_data_cleanup(&ultrasonic_devices[i].el_data);
            ultrasonic_device_cleanup(&ultrasonic_devices[i], i, ultrasonic_class);
        }

        kfree(ultrasonic_devices);
    }

    if (ultrasonic_class)
        class_destroy(ultrasonic_class);

    unregister_chrdev_region(MKDEV(ultrasonic_major, 0), ULTRASONIC_NUM_DEVICES);
}
static int __init ultrasonic_driver_init(void)
{
    /* we will get the major number dynamically this is recommended please read ldd3*/
    int err;
    int i;
    int devices_to_destroy = 0;
    dev_t device_number;
    EL_PRINT_D("enter\n");

    err = alloc_chrdev_region(&device_number, 0, ULTRASONIC_NUM_DEVICES, ULTRASONIC_DEVICENAME);

    if (err < 0) {
        EL_PRINT_E("Failed to allocate cdev region");
        return err;
    }

    ultrasonic_major = MAJOR(device_number);
    ultrasonic_class = class_create(THIS_MODULE, "chardev");

    if (ultrasonic_class == NULL) {
        EL_PRINT_E("Class creation failed");
        goto fail;
    }
    
    err = ultrasonic_initialize_sysfs();

    if (err)
        goto fail;

//wangweiping delete for test
    ultrasonic_devices = (struct ultrasonic_device *)kzalloc(sizeof(struct ultrasonic_device) * ULTRASONIC_NUM_DEVICES, GFP_KERNEL);

    if (ultrasonic_devices == NULL) {
        err = -ENOMEM;
        goto fail;
    }
/*
    if (ultrasonic_data_io_initialize())
    goto fail;

*/
//wangweiping delete for test
    devices_to_destroy = 0;

    for (i = 0; i < ULTRASONIC_NUM_DEVICES; ++i) {
        if (ultrasonic_device_initialize(&ultrasonic_devices[i], i, ultrasonic_class)) {
            devices_to_destroy = i;
            goto fail;
        }

        if (ultrasonic_data_initialize(&ultrasonic_devices[i].el_data, ULTRASONIC_DATA_FIFO_SIZE, ULTRASONIC_WAKEUP_TIMEOUT, i)) {
            goto fail;
        }
    }

    wake_source = kmalloc(sizeof(struct wakeup_source), GFP_KERNEL);

    if (!wake_source) {
        EL_PRINT_E("failed to allocate wake source");
        return -ENOMEM;
    }

    wakeup_source_init(wake_source, "ultrasonic_wake_source");
    EL_PRINT_D("exit \n");
    return 0;

fail:
    ultrasonic_driver_cleanup(devices_to_destroy);
    return err;
}

static void ultrasonic_driver_exit(void)
{
    if (wake_source) {
        wakeup_source_trash(wake_source);
        kfree(wake_source);
    }

    ultrasonic_cleanup_sysfs();
    ultrasonic_driver_cleanup(ULTRASONIC_NUM_DEVICES);
}

MODULE_AUTHOR("Nubia Audio");
MODULE_DESCRIPTION("Providing Interface to UPS data");
MODULE_LICENSE("GPL");

module_init(ultrasonic_driver_init);
module_exit(ultrasonic_driver_exit);

