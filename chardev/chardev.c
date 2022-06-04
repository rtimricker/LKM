
/*
 *  chardev.c - Create an input/output character device
 */
#include <linux/mutex.h> // mutex locks to provide concurrency protection
#include <linux/kernel.h>       /* We're doing kernel work */
#include <linux/module.h>       /* Specifically, a module */
#include <linux/moduleparam.h>
#include <linux/version.h>
#include <linux/init.h>
#include <linux/fs.h>
#include <asm/uaccess.h>        /* for get_user and put_user */
#include <asm/types.h>
#include <linux/mm.h>
 
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/raw.h>
#include <linux/utsname.h>
#include <generated/utsrelease.h>
#include <linux/sched.h>
#include <linux/delay.h> 

#include <linux/slab.h>     /* kmalloc */
#include <linux/dma-mapping.h>
#include <linux/moduleparam.h>

#include <linux/proc_fs.h>

#include <linux/pci.h>

#include "chardev.h"

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Roger Tim Ricker");
MODULE_DESCRIPTION("A sample character driver");

#define SUCCESS 0
#define BUF_LEN 80

#define CLASS_NAME "mod" // The device class -- this is a character device
static int majorNumber;  // Stores the device number -- determined automatically
// static char message[256] = {0}; /// Memory for the string that is passed from userspace
// static short size_of_message; /// Used to remember the size of the string stored
static struct class *chardrvClass = NULL;   // The device-driver class struct pointer
static struct device *chardrvDevice = NULL; // The device-driver device struct pointer
static DEFINE_MUTEX(char_mutex);            // declare mutex lock
static char Message[BUF_LEN];
static char *Message_Ptr;

/*  Prototypes - this would normally go in a .h file */
static int device_open(struct inode *, struct file *);
static int device_release(struct inode *, struct file *);
static ssize_t device_read(struct file *, char __user *, size_t, loff_t *);
static ssize_t device_write(struct file *, const char __user *, size_t, loff_t *);

static int device_open(struct inode *inode, struct file *filp)
{
#ifdef DEBUG
  printk(KERN_INFO "Chardrv: device_open(%p)\n", file);
#endif
  // device_ch_t* ch = NULL;

  printk(KERN_INFO "Chardrv: device_open(0x%p, 0x%p)\n", inode, filp);

  // filp->private_data = ch = container_of(inode->i_cdev, device_ch_t, cdev);

  return SUCCESS;
}

/** @brief The device release function that is called whenever the device is closed/released by
 * the userspace program
 * @param inodep A pointer to an inode object (defined in linux/fs.h)
 * @param filep A pointer to a file object (defined in linux/fs.h)
 */
static int device_release(struct inode *inodep, struct file *filep)
{
  printk(KERN_INFO "Chardrv: device_release\n");
  mutex_unlock(&char_mutex);
  printk(KERN_INFO "Chardrv: Device successfully closed\n");
  return SUCCESS;
} /* device_release */


/*======================================================================
 * NAME:  device_remove()
 *
 * DESCRIPTION:
 *    This routine uninstalls a PCI device
 *
 * ARGUMENTS:
 *    pcidev - kernel PCI device
 *
 * RETURN VALUE:
 *    none.
 *
 *====================================================================*/

static void device_remove(
                struct pci_dev *pcidev)
{
        //dscc4_dev_t *dev = (dscc4_dev_t*)pci_get_drvdata(pcidev);
        //int idx;

        //PDEBUG ("device_remove() dev->id: %d\n",dev->id);
        PDEBUG ("device_remove()\n");

        //if( !(dev = (device_dev_t*)pci_get_drvdata(pcidev)) ) {
        //        return;
        //}

        /*
        ** Request IRQ
        ** Although device can still be used without interrupts
        ** there is no code to handle them when hardware interrupts
        ** are triggered. Fail open request in this case.
        */
        //if( ( request_irq(dev->pcidev->irq,
        //        (irqreturn_t (*)(int, void*, struct pt_regs*))device_isr,
        //        IRQF_SHARED,
        //        DEVNAME,        //                dev->name,
        //        dev)) ){
        //                printk(KERN_ALERT "request_irq failed\n");
        //}

        //device_uninitialize(dev);
        //free_irq(dev->pcidev->irq,dev);

        //PDEBUG ("list_del\n");
        //list_del(&dev->entry);  // Remove device from the list.

        //for( idx = 0; idx < device_CHANNEL_MAX; idx++ ) {
        //        if( dev->ch[idx].cdev_init ) {
        //                cdev_del(&dev->ch[idx].cdev);
        //        }
        //}
        // Clear all.

        //flush_workqueue(dev->wq);
        //destroy_workqueue(dev->wq);

        //if( dev->mem ){
        //        device_iounmap(dev->mem);
        //}

        //memset(dev, 0, sizeof(dscc4_dev_t));

        //kfree(dev);

        PDEBUG("remove ends here\n");
} /* device_remove */

static ssize_t
device_proc_read(struct file *filep, char *buffer, size_t len, loff_t *offset)
{
//  char *tmpPtr = buffer;
//  int tmpLen = 0;
//  int bytes_read = 0;
  printk(KERN_INFO "Chardrv: device_read_proc\n");

  return SUCCESS;
}

static ssize_t
device_proc_write(struct file *file, const char __user *buffer, size_t length, loff_t *offset)
{
//  char *tmpPtr = buffer;
//  int tmpLen = 0;
//  int bytes_read = 0;
  printk(KERN_INFO "Chardrv: device_write_proc\n");

  return SUCCESS;
}

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
  char *tmpPtr = buffer;
  int tmpLen = 0;
  int bytes_read = 0;
  printk(KERN_INFO "Chardrv: device_read\n");

  /*
   * Actually put the data into the buffer
   */
  bytes_read = 0;
  tmpLen = len;
  while (len && *Message_Ptr)
  {
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
  printk(KERN_INFO "Chardrv: device_read(%p,%s,%d)", filep, tmpPtr, bytes_read);

  return bytes_read;
} /* device_read */

/*
 * This function is called when somebody tries to
 * write into our device file.
 */
static ssize_t
device_write(struct file *file, const char __user *buffer, size_t length, loff_t *offset)
{
  int i;

  //#ifdef DEBUG
  printk(KERN_INFO "Chardrv: device_write(%p,%s,%ld)", file, buffer, length);
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

loff_t device_llseek(struct file *file,
                     loff_t loff, int val)
{
  printk(KERN_INFO "Chardrv: device_llseek(%p)\n", file);
  return 0;
}

ssize_t device_read_iter(struct kiocb *kiocb,
                         struct iov_iter *lov_iter)
{
  printk(KERN_INFO "Chardrv: device_read_iter()\n");
  return 0;
}

ssize_t device_write_iter(struct kiocb *kiocb,
                          struct iov_iter *iov_iter)
{
  printk(KERN_INFO "Chardrv: device_write_iter()\n");
  return 0;
}

int device_iopoll(struct kiocb *kiocb,
                  bool spin)
{
  printk(KERN_INFO "Chardrv: device_iopoll()\n");
  return 0;
}

int device_iterate(struct file *file,
                   struct dir_context *dir_context)
{
  printk(KERN_INFO "Chardrv: device_iterate()\n");
  return 0;
}

int device_iterate_shared(struct file *file,
                          struct dir_context *dir_context)
{
  printk(KERN_INFO "Chardrv: device_iterate_shared()\n");
  return 0;
}

int device_fsync(struct file *file, loff_t begin_loff, loff_t end_loff, int datasync)
{
  printk(KERN_INFO "Chardrv: device_fsync()\n");
  return 0;
}

__poll_t device_poll(struct file *file,
                     struct poll_table_struct *poll_table)
{
  printk(KERN_INFO "Chardrv: device_poll(%p)\n", file);
  return 0;
}

int device_mmap(struct file *file,
                struct vm_area_struct *vm_area)
{
  printk(KERN_INFO "Chardrv: device_mmap(%p)\n", file);
  return 0;
}

ssize_t device_aio_read(struct kiocb *kiocb,
                        const struct iovec *iovec,
                        unsigned long val,
                        loff_t loff)
{
  printk(KERN_INFO "Chardrv: device_aio_read()\n");
  return 0;
}

ssize_t device_aio_write(struct kiocb *kiocb,
                         const struct iovec *iovec,
                         unsigned long val,
                         loff_t loff)
{
  printk(KERN_INFO "Chardrv: device_aio_write\n");
  return 0;
}

int device_flush(struct file *file, fl_owner_t id)
{
  printk(KERN_INFO "Chardrv: device_flush(%p)\n", file);
  return 0;
}

int device_fasync(int val, struct file *file, int val1)
{
  printk(KERN_INFO "Chardrv: device_fasync()\n");
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

long device_unlocked_ioctl(struct file *file,
                           unsigned int ioctl_num,
                           unsigned long ioctl_param)

{
  // int i;
  // char *temp;
  // char ch;
#if 0
	/* 
	 * Switch according to the ioctl called 
	 */
//printk (KERN_INFO "Chardrv: switch num (%d) param (%ld)\n", ioctl_num, ioctl_param);
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
//printk (KERN_INFO "Chardrv: IOCTL_GET_NTH_BYTE\n");
//printk (KERN_INFO "Chardrv: %ld\n", (unsigned long int)ioctl_param);
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
static struct file_operations Fops = {
    .owner = THIS_MODULE,
    .open = device_open,
    .read = device_read,
    .write = device_write,
    .llseek = device_llseek,
    .release = device_release, /* a.k.a. close */

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

//*****
// * struct pci_driver {
// *   struct list_head node;
// *   const char *name;
// *   const struct pci_device_id *id_table; /* Must be non-NULL for probe to be called */
// *   int (*probe)(struct pci_dev *dev, const struct pci_device_id *id); /* New device inserted */
// *   void (*remove)(struct pci_dev *dev); /* Device removed (NULL if not a hot-plug capable driver) */
// *   int (*suspend)(struct pci_dev *dev, pm_message_t state); /* Device suspended */
// *   int (*resume)(struct pci_dev *dev); /* Device woken up */
// *   void (*shutdown)(struct pci_dev *dev);
// *   int (*sriov_configure)(struct pci_dev *dev, int num_vfs); /* On PF */
// *   const struct pci_error_handlers *err_handler;
// *   const struct attribute_group **groups;
// *   struct device_driver driver;
// *   struct pci_dynids dynids;
// * };
// *****/

/*======================================================================
 * NAME:  device_probe()
 *
 * DESCRIPTION:
 *    This routine installs a PCI device
 *
 * ARGUMENTS:
 *    pcidev - kernel PCI device
 *    id     - PCI hardware ID table 
 *
 * RETURN VALUE:
 *    int - success >= 0, 0 > error.
 *
 *====================================================================*/

static int device_probe(
                struct pci_dev * pcidev,
                const struct pci_device_id * id)
{
        int res;
        //device_dev_t *dev = NULL;
        //device_card_t *brd = NULL;
        //int i;
        //int dev_number;
        //int bus_number;
  //      int ii;
        //char workqueue_name[25];
        //u8 info;

        //device_setup_scc_t default_scc_setup;
        //get_default_setup(&default_scc_setup);

        PDEBUG("device_probe\n");

        if( (res = pci_enable_device(pcidev)) ) {
                PDEBUG("probe: pci_enable_device failed\n");
                return res;
        }
        return 0;
//error:
        device_remove(pcidev);

        return res;

} /* device_probe */


                  
static struct pci_driver device_pci_driver = {
  .name = DEVICE_NAME,
  .probe = device_probe

  /*
      .name = DEVICE_NAME,
      .id_table = device_ids,
      .probe =    device_probe,
      .remove =   device_remove,
    */
};     /* device_pci_driver */

/*
 * The proc_ops structure. This is the glue layer which associates the
 * proc entry to the read and write operations.
 */
struct proc_ops _proc_fops = {
    .proc_read = device_proc_read,
    .proc_write = device_proc_write,
};

struct k_list {
  struct list_head test_list;
  int temp;
};
//struct list_head test_head;
LIST_HEAD(test_head);
struct k_list *one,*two,*three;
struct k_list *entry;
struct list_head test_head;
struct list_head *ptr;

#if 0
/*======================================================================
 * NAME:  device_read_procmem()
 *
 * DESCRIPTION:
 *    This routine implements /proc file read interface.
 *
 * ARGUMENTS:
 *  buf    - bufer to store data
 *  start  - where data is written in the buffer
 *  offset - start offset within the buffer
 *  count  - size of the buffer
 *  eof    - end of file flag
 *  data   - user specific data
 *
 * RETURN VALUE:
 *    int - success >= 0 or 0< error.
 *
 *====================================================================*/
static int device_read_procmem(
                  char  *buf,
                  char  **start,
                  off_t offset,
                  int   count,
                  int   *eof,
                  void  *data)
{
        int len = 0;
//        struct list_head *pos;
        //device_dev_t *dev;
//        int i;
 
        PDEBUG ("device_read_procmem()\n");
//        list_for_each(pos, &device_dev_list) {
//                if ( pos && (dev = list_entry(pos, device_dev_t, entry))) {
//                        for( i = 0; i < dev->ch_num; i++ ) {
//                                len += sprintf(buf + len, "%s %d ", dev->ch[i].name, dev->ch[i].minor );
//                        }
//                }
//        }

        *eof = 1;

        return len;

} /* device_read_procmem */
#endif


struct proc_dir_entry *proc;
//int len;
char *msg = NULL;
char name[30];

void create_new_proc_entry(void) 
{
//  char *DATA = "Hello People"
  sprintf("%s","Hello People");
  PDEBUG ("CALLING create_proc read/write entry()\n");
  sprintf (name, "driver/%s", DEVICE_NAME);

  msg = "Hello World ";
  proc_create_data("hello",0,NULL,&_proc_fops,msg);
  //proc_create("hello",0,NULL,&_proc_fops);

  //len = strlen(msg);
  //temp = len;
}

/*
 * Initialize the module - Register the character device
 */

static int __init device_init(void)
{
 /* sample */
  int i;  
  int res = 0;
//  char msg[30];
 
  one=kmalloc(sizeof(struct k_list),GFP_KERNEL);
  printk(KERN_INFO " kmalloc: %p\n", one);
  two=kmalloc(sizeof(struct k_list),GFP_KERNEL);
  printk(KERN_INFO " kmalloc: %p\n", two);
  three=kmalloc(sizeof(struct k_list),GFP_KERNEL);
  printk(KERN_INFO " kmalloc: %p\n", three);
  //
  one->temp=10;
  two->temp=20;
  three->temp=30;
  //
  list_add(&one->test_list, &test_head);
  list_add(&two->test_list, &test_head);
  list_add(&three->test_list, &test_head);
  //
  i = 0;  
  ptr=&(one->test_list);
  list_for_each(ptr,&test_head) {
      entry=list_entry(ptr,struct k_list,test_list);
      i=i+1;
      //printk(KERN_INFO "\n Entry %d %d  \n \n\n ", i,entry->temp);
      printk(KERN_INFO "Entry %d %d\n", i,entry->temp);
  }

  if (three) {
    printk(KERN_INFO " Deleting one entry\n");
    list_del(&(three->test_list));
    kfree(three);
    three = NULL;
  }
  if (two) {
    printk(KERN_INFO " Deleting one entry\n");
    list_del(&(two->test_list));
    kfree(two);
    two = NULL;
  }
  if(one) {
    printk(KERN_INFO " Deleting one entry\n");
    list_del(&(one->test_list));
    one = NULL;
  }

  printk(KERN_INFO "Done....\n ");
  //
  /* end of sample */

  printk(KERN_INFO "Chardrv: Initializing the Char LKM\n");  

  // Try to dynamically allocate a major number for the device -- more difficult but worth it
  majorNumber = register_chrdev(0, DEVICE_NAME, &Fops);
  if (majorNumber < 0) {
    printk(KERN_ALERT "Chardrv: %s failed to register a major number\n", DEVICE_NAME);
    return majorNumber;
  }
  printk(KERN_INFO "Chardrv: registered correctly with major number %d\n", majorNumber);

#if 0
//  if( (res = pci_register_driver(&device_pci_driver)) < 0 ) {
//    PDEBUG ("pci_register_driver() FAILED\n");
//    unregister_chrdev(majorNumber, DEVICE_NAME);         // unregister the major number
//    //goto error;
//    return -1;
//  }
//  PDEBUG ("pci_register_driver() SUCCESS\n");
#endif

  // Register the device class
  chardrvClass = class_create(THIS_MODULE, CLASS_NAME);
  if (IS_ERR(chardrvClass)) { // Check for error and clean up if there is
    unregister_chrdev(majorNumber, DEVICE_NAME);
    printk(KERN_ALERT "Chardrv: Failed to register device class\n");
    return PTR_ERR(chardrvClass); // Correct way to return an error on a pointer
  }
  printk(KERN_INFO "Chardrv: device class registered correctly: %s\n", DEVICE_NAME);

  //create_new_proc_entry();

  // Register the device driver
  chardrvDevice = device_create(chardrvClass, NULL, MKDEV(majorNumber, 0), NULL, DEVICE_NAME);
  if (IS_ERR(chardrvDevice)) {                              // Clean up if there is an error
    class_destroy(chardrvClass); // Repeated code but the alternative is goto statements
    unregister_chrdev(majorNumber, DEVICE_NAME);
    printk(KERN_ALERT "Chardrv: Failed to create the device\n");
    return PTR_ERR(chardrvDevice);
  }

  printk(KERN_INFO "Chardrv: device class created correctly\n"); // Made it! device was initialized
  mutex_init(&char_mutex);                                       // initialize mutex lock

  return SUCCESS;
}

/*
 * Cleanup - unregister the appropriate file from /proc
 */

/** @brief The LKM cleanup function
 * Similar to the initialization function, it is static. The __exit macro notifies that if this
 * code is used for a built-in driver (not a LKM) that this function is not required.
 */
static void __exit device_exit(void)
{
  char name[30];

  printk(KERN_INFO "Chardrv: device_exit\n");
  mutex_destroy(&char_mutex);                          // remove mutex lock
  device_destroy(chardrvClass, MKDEV(majorNumber, 0)); // remove the device
  class_unregister(chardrvClass);                      // unregister the device class

  sprintf (name, "driver/%s", DEVICE_NAME);
  //remove_proc_entry(name, NULL);

  class_destroy(chardrvClass);                         // remove the device class
//  pci_unregister_driver(device_pci_driver);
  unregister_chrdev(majorNumber, DEVICE_NAME);         // unregister the major number
  printk(KERN_INFO "Chardrv: Goodbye from the LKM!\n");
}

/*====================================================================*/

module_init(device_init);
module_exit(device_exit);

/*====================================================================*/
