
/*
 *  chardev.h - the header file with the ioctl definitions.
 *
 *  The declarations here have to be in a header file, because
 *  they need to be known both to the kernel module
 *  (in chardev.c) and the process calling ioctl (ioctl.c)
 */

#ifndef CHARDEV_H
#define CHARDEV_H

/* 
 * Set the message of the device driver 
 */
//#define IOCTL_SET_MSG _IOR(MAJOR_NUM, 0, char *)
/*
 * _IOR means that we're creating an ioctl command 
 * number for passing information from a user process
 * to the kernel module. 
 *
 * The first arguments, MAJOR_NUM, is the major device 
 * number we're using.
 *
 * The second argument is the number of the command 
 * (there could be several with different meanings).
 *
 * The third argument is the type we want to get from 
 * the process to the kernel.
 */

/* 
 * Get the message of the device driver 
 */
//#define IOCTL_GET_MSG _IOR(MAJOR_NUM, 1, char *)
/* 
 * This IOCTL is used for output, to get the message 
 * of the device driver. However, we still need the 
 * buffer to place the message in to be input, 
 * as it is allocated by the process.
 */

/* 
 * Get the n'th byte of the message 
 */
//#define IOCTL_GET_NTH_BYTE _IOWR(MAJOR_NUM, 2, int)
/* 
 * The IOCTL is used for both input and output. It 
 * receives from the user a number, n, and returns 
 * Message[n]. 
 */

/* 
 * The name of the device file 
 */
#define DEVICE_NAME "char_dev"
#define PDEBUG(fmt, args...) printk( KERN_ALERT DEVICE_NAME": " fmt, ## args)

#include <linux/termios.h>

#if 1
//struct _cdev {
//	struct kobject kobj;
//	struct module *owner;
//	const struct file_operations *ops;
//	struct list_head list;
//	dev_t dev;
//	unsigned int count;
//} __randomize_layout;

typedef struct termios termios_t;
typedef struct __DEVICE_ISR_BH_ARG_TYPE device_isr_bh_arg_t;
typedef struct __device_dev_t device_dev_t;
typedef struct __device_ch_t device_ch_t;
typedef struct __device_card_t device_card_t;
typedef struct cdev cdev_t;

#define DEVICE_CHANNEL_MAX 4
#define NUM_BH_MAX 8

typedef struct __DEVICE_ISR_BH_ARG_TYPE dscc4_isr_bh_arg_t;

 typedef enum __DEVICE_SERIAL_PORT_MODE_ENUM_TYPE{
    ProtocolHDLC = 0,
    ProtocolASYNC = 1,
    ProtocolBISYNC = 2
} device_sm_t;

struct __DEVICE_ISR_BH_ARG_TYPE {
    struct work_struct *work;        // work entry 
    int idx;
    int id;
    device_dev_t *dev;
    device_ch_t *ch;
 };
 
struct __device_ch_t {
    struct cdev             *cdev;                   // char device structure
    device_dev_t            *dev;
    int                     cdev_init;              // CDEV initialized
    int                     id;                     // channel ID
    int                     minor;                  // channel minor number
    char                    name[32];               // channel name
    int                     ref_count; 
};
struct __device_dev_t {
    struct              list_head * entry;        // device list entry
    struct workqueue_struct *wq;  /* work queue */
    device_dev_t            *dev;
    int                     id;                     // device ID
    device_ch_t             ch[DEVICE_CHANNEL_MAX];
    char                    name[32];               // device name 
    struct pci_dev          *pcidev;                // kernel PCI deviceG
    device_card_t*          card;
    int                     ch_num;                 // number of channels 
    device_isr_bh_arg_t     bh[NUM_BH_MAX];
};
struct __device_card_t {
    device_dev_t*            dev[3];                         // number of controllers per card
    int                     irq;                            // irq per card;
    int                     id;                             // card id/number;
    //device_dev_t            *dev;
    int                     ref_count;
    int                     bus_number;
    int                     controller_count;
    int                     board_number;                   // used for device name in INIT
    int                     previous_board_type;
    int                     do12_set[12];
    device_isr_bh_arg_t     bh[NUM_BH_MAX];
};


#endif

#if 0
/*======================================================================
 * Definition of device channel
 *====================================================================*/

struct __device_ch_t {
//    struct cdev             cdev;                   // char device structure
    device_dev_t            *dev;
    int                     cdev_init;              // CDEV initialized
    int                     id;                     // channel ID
    int                     minor;                  // channel minor number
    char                    name[32];               // channel name
    int                     ref_count;              // reference count
//    struct semaphore        sem;                    // mutual exclusion semaphore
//    struct semaphore        w_sem;                  // write mutual exclusion semaphore
//    struct semaphore        r_sem;                  // read mutual exclusion semaphore

//    wait_queue_head_t       tx_event;               // hss_tx and hss_drv
//    spinlock_t              lock;                   // sync spinlock 
    int                     tx_reset;               // hss_tx and hss_drv
    int                     tx_initialized;         // hss_tx and hss_drv
    int                     write_result;
    int                     read_result;
    int                     open_result;
    int                     fasync_result;
    int                     drv_ioctl_result;

    termios_t               t;

    struct fasync_struct    *async_queue;           // for fasync
    unsigned long           rx_isr_flags;
//    ssize_t                 flen;                   // receive frame length
    int                     toie;                   // timeout indication enabled automatic processing

    // _tx
    int                     tx_result;
    int                     write_count;
    unsigned long           tx_flags;

    // _ioctl
    int                     hdplx;                  // duplex mode, 0-full, 1-half
    int                     ioctl_result;
    unsigned long           ioctl_flags;
    int                     ioctl_val;              // used in __update_setup_from_termios
    // used in setup routine
    unsigned long           ioctl_iq_flags;
    unsigned long           ioctl_iqp_flags;
    unsigned long           ioctl_rx_flags;
    unsigned long           ioctl_tx_flags;
    unsigned long           setup_flags;
    int                     setup_shift;
    int                     setup_ch_num;

    int                     prog12_idx;
    int                     cmd_idx;
    //u32                     cmd_ccr0_bak;
    //u32                     cmd_brr_bak;
    int                     cmd_to;
    unsigned long           cmd_flags;

    int                     rx_buffer_num;
    int                     f_flags;
    unsigned long           poll_flags;
    unsigned int            retmask;

}; /* struct __device_ch_t */
#endif

#if 0
/*======================================================================
 * Definition of device device
 *====================================================================*/
struct __device_dev_t {
    struct              list_head * entry;        // device list entry
    int                     id;                     // device ID
    //spinlock_t              lock;                   // HW access spinlock
    char                    name[32];               // device name
    int                     ref_count;              // reference count
    struct pci_dev          *pcidev;                // kernel PCI device
    unsigned long           mem_addr;               // On-board memory base address
    unsigned long           mem_len;                // On-board memory length
    void                    *mem;                   // On-board memory virtual base address
    int                     ch_num;                 // number of channels
    int                     arack;
    //wait_queue_head_t       iqcfg_q;
    //struct semaphore        sem;                    // sync semaphore

    int                     set12_idx;
    int                     dev_controller_count;

    //struct workqueue_struct *wq;  /* work queue */
    int                     drv_isr_idx;
    unsigned long           isr_flags;
    int                     gcmd_result;
    unsigned long           gcmd_flags;
}; /* struct __device_dev_t */
#endif

#endif
