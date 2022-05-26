
/*
 *  chardev.c - Create an input/output character device
 */
#if 0
#include <linux/kernel.h>	/* We're doing kernel work */
#include <linux/module.h>	/* Specifically, a module */
#include <linux/fs.h>
#include <asm/uaccess.h>	/* for get_user and put_user */
#else

#include <linux/mutex.h> // mutex locks to provide concurrency protection
#include <linux/kernel.h>       /* We're doing kernel work */
#include <linux/module.h>       /* Specifically, a module */
#include <linux/version.h>
#include <linux/init.h>
#include <linux/fs.h>
//#include <linux/aio.h>
//#include <linux/vmalloc.h>
//#include <asm/uaccess.h>        /* for get_user and put_user */
 
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/raw.h>
#include <linux/utsname.h>
#include <generated/utsrelease.h>
//#include <asm/current.h>
#include <linux/sched.h>
#include <linux/delay.h> 
 
#include "chardev.h"

#endif

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Roger Tim Ricker");
MODULE_DESCRIPTION("A sample character driver");

#define SUCCESS 0
#define BUF_LEN 80

#define CLASS_NAME "mod" /// The device class -- this is a character device
static int majorNumber; /// Stores the device number -- determined automatically
//static char message[256] = {0}; /// Memory for the string that is passed from userspace
//static short size_of_message; /// Used to remember the size of the string stored
static struct class* chardrvClass = NULL; // The device-driver class struct pointer
static struct device* chardrvDevice = NULL; // The device-driver device struct pointer
static DEFINE_MUTEX(char_mutex); // declare mutex lock
static char Message[BUF_LEN];
static char *Message_Ptr;
/*  Prototypes - this would normally go in a .h file */ 
static int device_open(struct inode *, struct file *); 
static int device_release(struct inode *, struct file *); 
static ssize_t device_read(struct file *, char __user *, size_t, loff_t *); 
static ssize_t device_write(struct file *, const char __user *, size_t, loff_t *); 


static int device_open( struct inode *inode, struct file  *filp)
{
#ifdef DEBUG
	printk(KERN_INFO "device_open(%p)\n", file);
#endif
	device_ch_t* ch = NULL;

    printk (KERN_INFO "device_open(0x%p, 0x%p)\n", inode, filp);

    filp->private_data = ch = container_of(inode->i_cdev, device_ch_t, cdev);

    return SUCCESS;
}

/** @brief The device release function that is called whenever the device is closed/released by
 * the userspace program
 * @param inodep A pointer to an inode object (defined in linux/fs.h)
 * @param filep A pointer to a file object (defined in linux/fs.h)
 */
static int device_release(struct inode *inodep, struct file *filep){
    printk(KERN_INFO "device_release\n");
    mutex_unlock(&char_mutex);
    //printk(KERN_INFO "Chardrv: Device successfully closed\n");
    return SUCCESS;
} /* device_release */


/** @brief This function is called whenever device is being read from user space i.e. data is
 * being sent from the device to the user. In this case is uses the copy_to_user() function to
 * send the buffer string to the user and captures any errors.
 * @param filep A pointer to a file object (defined in linux/fs.h)
 * @param buffer The pointer to the buffer to which this function writes the data
 * @param len The length of the b
 * @param offset The offset if required
 */
static ssize_t 
device_read(struct file *filep, char *buffer, size_t len, loff_t *offset)
{
    char * tmpPtr = buffer;
    int tmpLen = 0;
    //int error_count = 0;
    int bytes_read = 0;
    //int idx = 0;
    printk(KERN_INFO "device_read\n");
    // copy_to_user has the format ( * to, *from, size) and returns 0 on success
    //error_count = copy_to_user(buffer, Message, len);
    //bytes_read = __copy_to_user(buffer, Message, strlen(Message));
	//printk(KERN_INFO "device_read(%p,%s,%d)", filep, buffer, bytes_read);
	//printk(KERN_INFO "device_read(%p,%s,%s,%d)", filep, buffer, message, error_count);

    //if (error_count==0){ // if true then have success
    //    printk(KERN_INFO "Chardrv: Sent %d characters to the user\n", size_of_message);
    //    return (size_of_message=0); // clear the position to the start and return 0
    //} else {
    //    printk(KERN_INFO "Chardrv: Failed to send %d characters to the user\n", error_count);
    //    return -EFAULT; // Failed -- return a bad address message (i.e. -14)
    //}
#if 1
    /* 
     * Actually put the data into the buffer 
     */
    bytes_read = 0;
//    tmpPtr = buffer;
    tmpLen = len;
    while (len && *Message_Ptr) {

        /* 
         * Because the buffer is in the user data segment,
         * not the kernel data segment, assignment wouldn't
         * work. Instead, we have to use put_user which
         * copies data from the kernel data segment to the
         * user data segment. 
         */
        put_user(*Message_Ptr++, buffer++);
        len--;
        bytes_read++;
    }
	printk(KERN_INFO "device_read(%p,%s,%d)", filep, tmpPtr, bytes_read);
#endif

    return bytes_read;
} /* device_read */


/* 
 * This function is called when somebody tries to
 * write into our device file. 
 */
static ssize_t
device_write(struct file *file,
	     const char __user * buffer, 
		 size_t length, 
		 loff_t * offset)
{
	int i;

//#ifdef DEBUG
	printk(KERN_INFO "device_write(%p,%s,%ld)", file, buffer, length);
//#endif

	for (i = 0; i < length && i < BUF_LEN; i++)
		get_user(Message[i], buffer + i);

	Message_Ptr = Message;

	/* 
	 * Again, return the number of input characters used 
	 */
	return i;
}


/* Extra driver entry points. */

loff_t device_llseek (struct file * file, 
                    loff_t loff, int val)
{
  printk(KERN_INFO "device_llseek(%p)\n", file);
  return 0;
}

ssize_t device_read_iter (struct kiocb * kiocb, 
                          struct iov_iter * lov_iter)
{
  printk(KERN_INFO "device_read_iter()\n");
  return 0;
}

ssize_t device_write_iter (struct kiocb * kiocb, 
                          struct iov_iter * iov_iter)
{
  printk(KERN_INFO "device_write_iter()\n");
  return 0;
}

int device_iopoll (struct kiocb * kiocb, 
                   bool spin)
{
  printk(KERN_INFO "device_iopoll()\n");
  return 0;
}

int device_iterate (struct file * file, 
                    struct dir_context * dir_context)
{
  printk(KERN_INFO "device_iterate()\n");
  return 0;
}

int device_iterate_shared (struct file * file, 
                    struct dir_context * dir_context)
{
  printk(KERN_INFO "device_iterate_shared()\n");
  return 0;
}

int device_fsync (struct file * file, loff_t begin_loff, loff_t end_loff, int datasync)
{
  printk(KERN_INFO "device_fsync()\n");
  return 0;
}

__poll_t device_poll (struct file * file, 
                struct poll_table_struct * poll_table)
{
  printk(KERN_INFO "device_poll(%p)\n", file);
  return 0;
}

int device_mmap (struct file * file, 
                struct vm_area_struct * vm_area)
{
  printk(KERN_INFO "device_mmap(%p)\n", file);
  return 0;
}

ssize_t device_aio_read (struct kiocb * kiocb, 
                        const struct iovec * iovec, 
                        unsigned long val, 
                        loff_t loff)
{
  printk(KERN_INFO "device_aio_read()\n");
  return 0;
}
 
ssize_t device_aio_write (struct kiocb * kiocb, 
                          const struct iovec * iovec, 
                          unsigned long val, 
                          loff_t loff)
{
  printk(KERN_INFO "device_aio_write\n");
  return 0;
}

int device_flush (struct file * file, fl_owner_t id)
{
  printk(KERN_INFO "device_flush(%p)\n", file);
  return 0;
}

int device_fasync (int val, struct file * file, int val1)
{
  printk(KERN_INFO "device_fasync()\n");
  return 0;
}

/* 
 * This function is called whenever a process tries to do an ioctl on our
 * device file. We get two extra parameters (additional to the inode and file
 * structures, which all device functions get): the number of the ioctl called
 * and the parameter given to the ioctl function.
 *
 * If the ioctl is write or read/write (meaning output is returned to the
 * calling process), the ioctl call returns the output of this function.
 *
 */

long device_unlocked_ioctl (struct file * file,
                         unsigned int ioctl_num,
                         unsigned long ioctl_param)

{
	//int i;
	//char *temp;
	//char ch;
#if 0
	/* 
	 * Switch according to the ioctl called 
	 */
//printk (KERN_INFO "switch num (%d) param (%ld)\n", ioctl_num, ioctl_param);
	switch (ioctl_num) {
	case IOCTL_SET_MSG:
		/* 
		 * Receive a pointer to a message (in user space) and set that
		 * to be the device's message.  Get the parameter given to 
		 * ioctl by the process. 
		 */
		temp = (char *)ioctl_param;

		/* 
		 * Find the length of the message 
		 */
		get_user(ch, temp);
		for (i = 0; ch && i < BUF_LEN; i++, temp++) {
			get_user(ch, temp);
         }

		device_write(file, (char *)ioctl_param, i, 0);
		break;

	case IOCTL_GET_MSG:
		/* 
		 * Give the current message to the calling process - 
		 * the parameter we got is a pointer, fill it. 
		 */
		i = device_read(file, (char *)ioctl_param, 99, 0);

		/* 
		 * Put a zero at the end of the buffer, so it will be 
		 * properly terminated 
		 */
		put_user('\0', (char *)ioctl_param + i);
		break;

	case IOCTL_GET_NTH_BYTE:
//printk (KERN_INFO "IOCTL_GET_NTH_BYTE\n");
//printk (KERN_INFO "%ld\n", (unsigned long int)ioctl_param);
		/* 
		 * This ioctl is both input (ioctl_param) and 
		 * output (the return value of this function) 
		 */
		return Message[ioctl_param];
		break;
	}
#endif
	return SUCCESS;
}

/* Module Declarations */

/* 
 * This structure will hold the functions to be called
 * when a process does something to the device we
 * created. Since a pointer to this structure is kept in
 * the devices table, it can't be local to
 * init_module. NULL is for unimplemented functions. 
 */
struct file_operations Fops = {
	.owner = THIS_MODULE,
	.open = device_open,
	.read = device_read,
	.write = device_write,
	.llseek = device_llseek,
	.release = device_release,	/* a.k.a. close */

	.read_iter = device_read_iter,
	.write_iter = device_write_iter,
	.unlocked_ioctl = device_unlocked_ioctl,
	.flush = device_flush,
	.iopoll = device_iopoll,
	.iterate = device_iterate,
	.iterate_shared = device_iterate_shared,
	.poll = device_poll, 
	.mmap = device_mmap,
	.fsync = device_fsync,
	.fasync = device_fasync,
};

/* 
 * Initialize the module - Register the character device 
 */

int device_init(void)
{

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

    return SUCCESS; 
}

/*
 * Cleanup - unregister the appropriate file from /proc 
 */

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



/*====================================================================*/

module_init(device_init);
module_exit(device_exit);

/*====================================================================*/
