/*
 * IMPORTANT: !!!!!
 * I based this driver off the one on web site:
 * "https://xdecroc.wordpress.com/2016/03/08/writing-my-first-character-device-driver-lkm/"
 *
 */
#include <linux/mutex.h> // mutex locks to provide concurrency protection
#include <linux/version.h>
#include <linux/kernel.h> 
#include <linux/init.h>
#include <linux/module.h>
#include <linux/proc_fs.h>
#include <linux/delay.h>
#include <linux/fs.h>
#include <linux/slab.h>
#include <linux/pci.h>
#include <linux/interrupt.h>
#include <linux/tty.h>
#include <linux/termios.h>
#include <linux/serial.h>
#include <linux/pci.h>
#include <linux/pagemap.h>

#include <linux/poll.h>
#include <linux/mm.h>

#include "sample.h"

/*======================================================================
 * Kernel module interface
 *====================================================================*/

MODULE_LICENSE("Dual BSD/GPL");
MODULE_AUTHOR("Roger Tim Ricker");
MODULE_DESCRIPTION("Sample Character Driver");

//#define DEVICE_NAME "char_drv" /// The device will appear at /dev/chardrv using this value
#define CLASS_NAME "mod" /// The device class -- this is a character device

//module_param(device_major, int, 0);
//static int device_major = 0;
static int majorNumber; /// Stores the device number -- determined automatically
static char message[256] = {0}; /// Memory for the string that is passed from userspace
static short size_of_message; /// Used to remember the size of the string stored
//static int numberOpens = 0; /// Counts the number of times the device is opened
static struct class* chardrvClass = NULL; // The device-driver class struct pointer
static struct device* chardrvDevice = NULL; // The device-driver device struct pointer
static DEFINE_MUTEX(char_mutex); // declare mutex lock

#if 0
static int __device_close(device_ch_t *ch)
{
    printk (KERN_INFO "__device_close\n");
//    PDEBUG("__device_close(%d, %d, %d) ends here\n", ch->dev->card->id, ch->dev->id, ch->id);

    return 0;
}       // __device_close
#endif
/*======================================================================
 * NAME:  device_open()
 *
 * DESCRIPTION:
 *    This routine initializes device when file is opened.
 *
 * ARGUMENTS:
 *    inode - file inode
 *    filp  - opened file
 *
 * RETURN VALUE:
 *    int - success >= 0 or 0 < error.
 *
 *====================================================================*/

static int device_open( struct inode *inode, struct file  *filp)
{
    device_ch_t* ch = NULL;

    printk (KERN_INFO "device_open(0x%p, 0x%p)\n", inode, filp);

    filp->private_data = ch = container_of(inode->i_cdev, device_ch_t, cdev);

    return 0;

//error:
//    __device_close(ch);
//
//    //up(&ch->sem);
//
    return ch->open_result;
}       // device_open

/*======================================================================
 * NAME:  device_write()
 *
 * DESCRIPTION:
 *    This routine copies interrupt queueue copy to user buffer.
 *
 * ARGUMENTS:
 *    filp  - opened file
 *    buff  - user buffer
 *    count - buffer buff size in bytes
 *    f_pos - not used
 *
 * RETURN VALUE:
 *    ssize_t - number of bytes read >= 0 or 0 < error.
 *
 *====================================================================*/
#if 0 
static ssize_t device_write(struct file *filp, const char __user *buff, size_t count, loff_t *f_pos)
{
    device_ch_t* ch = NULL;

    //printk(KERN_INFO "device_write(%d, %d, %d) buff %p, count %d\n",ch->dev->card->id, ch->dev->id, ch->id, buff, (int)count);

    ch->f_flags = filp->f_flags;
    ch->write_result = 0;
//    if (buff && (count > 0) && ((ch->write_result = hss_dltx_write(ch, buff, count)) > 0)) {
//        *f_pos += ch->write_result;
//    }

    //printk(KERN_INFO "write(%d, %d, %d) ends here, ch->write_result: %d\n", ch->dev->card->id, ch->dev->id, ch->id, ch->write_result);

    //up(&ch->w_sem);
    return ch->write_result;
} /* device_write */
#else
/** @brief This function is called whenever the device is being written to from user space i.e.
 * data is sent to the device from the user. The data is copied to the message[] array in this
 * LKM using the sprintf() function along with the length of the string.
 * @param filep A pointer to a file object
 * @param buffer The buffer to that contains the string to write to the device
 * @param len The length of the array of data that is being passed in the const char buffer
 * @param offset The offset if required
 */
static ssize_t device_write(struct file *filep, const char *buffer, size_t len, loff_t *offset){
    sprintf(message, "%s(%ld letters)", buffer, len); // appending received string with its length
    size_of_message = strlen(message); // store the length of the stored message
    printk(KERN_INFO "Chardrv: Received %ld characters from the user\n", len);
    return len;
} /* device_write */
#endif

/*======================================================================
 * NAME:  device_read()
 * DESCRIPTION:
 *    This routine copies interrupt queue copy to user buffer.
 *
 * ARGUMENTS:
 *    filp  - opened file
 *    buff  - user buffer
 *    count - buffer buff size in bytes
 *    f_pos - not used
 *
 * RETURN VALUE:
 *    ssize_t - number of bytes read >= 0 or 0 < error.
 *
 *====================================================================*/
#if 0
static ssize_t device_read( struct file *filp, char __user *buff, size_t  count, loff_t  *f_pos)
{
    device_ch_t *ch = NULL;
    printk(KERN_INFO "device_read");

    if( (ch = (device_ch_t*)filp->private_data) == NULL ) {
        printk(KERN_ALERT "device_read() no device.\n");
        return -EINVAL;
    }
 
    //printk("device_read(%d, %d, %d) starts here\n", ch->dev->card->id, ch->dev->id, ch->id);

    if( down_interruptible(&ch->r_sem) ) {
        return -ERESTARTSYS;
    }

    printk ("read(%d), ch->f_flags: 0x%08x\n",ch->id,ch->f_flags);
    ch->read_result = 0;
    ch->f_flags = filp->f_flags;

//    up(&ch->r_sem);
    return ch->read_result; /* success */
} /* device_read */

#else
/** @brief This function is called whenever device is being read from user space i.e. data is
 * being sent from the device to the user. In this case is uses the copy_to_user() function to
 * send the buffer string to the user and captures any errors.
 * @param filep A pointer to a file object (defined in linux/fs.h)
 * @param buffer The pointer to the buffer to which this function writes the data
 * @param len The length of the b
 * @param offset The offset if required
 */
static ssize_t device_read(struct file *filep, char *buffer, size_t len, loff_t *offset){
    printk(KERN_INFO "device_read\n");
    int error_count = 0;
    // copy_to_user has the format ( * to, *from, size) and returns 0 on success
    error_count = copy_to_user(buffer, message, size_of_message);

    if (error_count==0){ // if true then have success
        printk(KERN_INFO "Chardrv: Sent %d characters to the user\n", size_of_message);
        return (size_of_message=0); // clear the position to the start and return 0
    } else {
        printk(KERN_INFO "Chardrv: Failed to send %d characters to the user\n", error_count);
        return -EFAULT; // Failed -- return a bad address message (i.e. -14)
    }
} /* device_read */
#endif

/*======================================================================
 * NAME:  device_release()
 *
 * DESCRIPTION:
 *    This routine clears device when file is closed.
 *
 * ARGUMENTS:
 *    inode - file inode
 *    filp  - opened file
 *
 * RETURN VALUE:
 *    int - success >= 0 or 0< error.
 *
 *====================================================================*/
#if 0
static int device_release( struct inode *inode, struct file *filp)
{
    device_ch_t *ch = NULL;
    // Remember... device_release gets called at file close! NOT driver unload.
    printk("device_release\n");

    if( !(ch = (device_ch_t*)filp->private_data) ) {
        printk(KERN_ALERT "device_release() no device.\n");
        return -EINVAL;
    }

//    fasync_helper(-1, filp, 0, &ch->async_queue);

//    up(&ch->sem);

    return 0;       // success
} /* device_release */
#else
/** @brief The device release function that is called whenever the device is closed/released by
 * the userspace program
 * @param inodep A pointer to an inode object (defined in linux/fs.h)
 * @param filep A pointer to a file object (defined in linux/fs.h)
 */
static int device_release(struct inode *inodep, struct file *filep){
    printk(KERN_INFO "device_release\n");
    mutex_unlock(&char_mutex);
    //printk(KERN_INFO "Chardrv: Device successfully closed\n");
    return 0;
} /* device_release */
#endif

/*======================================================================
 * Module implementation
 *====================================================================*/

/*======================================================================
 * NAME:  device_cleanup_module()
 *
 * DESCRIPTION:
 *    This routine cleans after module is undloaded.
 *
 * ARGUMENTS:
 *  none 
 *
 * RETURN VALUE:
 *    int - success >= 0 or 0< error.
 *
 *====================================================================*/

//static int registered_driver;
#if 0
static void device_cleanup_module(void)
{
    char name[30];

    printk("device_cleanup_module\n");

    if (!registered_driver) {
        return;
    }
    //pci_unregister_driver(&device_pci_driver);

    sprintf (name, "driver/%s",DEVICE_NAME);
    remove_proc_entry(name, NULL);

    unregister_chrdev_region(MKDEV(device_major, 0), 1);

    return;
} /* device_cleanup */
#else
/** @brief The LKM cleanup function
 * Similar to the initialization function, it is static. The __exit macro notifies that if this
 * code is used for a built-in driver (not a LKM) that this function is not required.
 */
static void __exit device_exit(void){
    printk(KERN_INFO "device_exit\n");
    mutex_destroy(&char_mutex); // remove mutex lock
    device_destroy(chardrvClass, MKDEV(majorNumber, 0)); // remove the device
    class_unregister(chardrvClass); // unregister the device class
    class_destroy(chardrvClass); // remove the device class
    unregister_chrdev(majorNumber, DEVICE_NAME); // unregister the major number
    printk(KERN_INFO "Chardrv: Goodbye from the LKM!\n");
}
#endif

/*======================================================================*/
struct file_operations Fops = {
    .owner =    THIS_MODULE,
    .write =    device_write,
    .read =     device_read,
    .open =     device_open,
    .release =  device_release,
}; /* Fops */
/*======================================================================*/

/*======================================================================
 * NAME:  device_init()
 *
 * DESCRIPTION:
 *    This routine initializes module.
 *
 * ARGUMENTS:
 *  none
 *
 * RETURN VALUE:
 *    int - success >= 0 or 0< error.
 *
 *====================================================================*/

static int __init device_init(void)
{
    printk(KERN_INFO "device_init\n");
#if 0
    dev_t devno;
    int res;
   if( device_major ) {
        devno = MKDEV(device_major, 0);
        res = register_chrdev_region(devno, 1, DEVNAME);
    } else {
        res = alloc_chrdev_region(&devno, 0, 1, DEVNAME);
        device_major = MAJOR(devno);
    }

    printk("major assigned: %d\n", device_major);
    registered_driver = 0;
#else
    printk(KERN_INFO "Chardrv: Initializing the Char LKM\n");
 
    // Try to dynamically allocate a major number for the device -- more difficult but worth it
    majorNumber = register_chrdev(0, DEVICE_NAME, &Fops);
    if (majorNumber<0){
        printk(KERN_ALERT "%s failed to register a major number\n", DEVICE_NAME);
        return majorNumber;
    }
    printk(KERN_INFO "Chardrv: registered correctly with major number %d\n", majorNumber);
 
    // Register the device class
    chardrvClass = class_create(THIS_MODULE, CLASS_NAME);
    if (IS_ERR(chardrvClass)){ // Check for error and clean up if there is
        unregister_chrdev(majorNumber, DEVICE_NAME);
        printk(KERN_ALERT "Failed to register device class\n");
        return PTR_ERR(chardrvClass); // Correct way to return an error on a pointer
    }
    printk(KERN_INFO "Chardrv: device class registered correctly: %s\n", DEVICE_NAME);
 
    // Register the device driver
    chardrvDevice = device_create(chardrvClass, NULL, MKDEV(majorNumber, 0), NULL, DEVICE_NAME);
    if (IS_ERR(chardrvDevice)){ // Clean up if there is an error
        class_destroy(chardrvClass); // Repeated code but the alternative is goto statements
        unregister_chrdev(majorNumber, DEVICE_NAME);
        printk(KERN_ALERT "Failed to create the device\n");
        return PTR_ERR(chardrvDevice);
    }

    printk(KERN_INFO "Chardrv: device class created correctly\n"); // Made it! device was initialized
    mutex_init(&char_mutex); // initialize mutex lock
#endif

    return 0;
 
//error:
//    printk("device_init_module FAILED\n");
//    device_exit();
//
//    return res;
 
} /* device_init_module */

/*======================================================================
 * NAME:  device_setup_cdev()
 *
 * DESCRIPTION:
 *    This routine initializes character device structure.
 *
 * ARGUMENTS:
 *  dev   - device
 *  index - device index  
 *
 * RETURN VALUE:
 *    int - success >= 0 or 0< error.
 *
 *====================================================================*/
#if 0
static int device_setup_cdev( device_ch_t *ch, int index)
{
    int err, devno = MKDEV(device_major, index);

    cdev_init(&ch->cdev, &Fops);
    ch->cdev.owner = THIS_MODULE;
    ch->cdev.ops = &Fops;
    err = cdev_add (&ch->cdev, devno, 1);

    /* Fail gracefully if need be */
    if( (err = cdev_add (&ch->cdev, devno, 1))) {
        printk(KERN_ALERT "Error %d adding DEVNAME%d", err, index);
        return err;
    }

    ch->cdev_init = 1;

    return 0;
} /* device_setup_cdev */
/*====================================================================*/
#endif

/*====================================================================*/

module_init(device_init);
module_exit(device_exit);

/*====================================================================*/



