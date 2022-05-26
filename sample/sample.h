
#define DEVICE_NAME "char_dev"
#define PDEBUG(fmt, args...) printk( KERN_ALERT DEVICE_NAME": " fmt, ## args)
#define NUM_BH_MAX 8

#include <linux/termios.h>
typedef struct termios termios_t;
typedef struct __device_dev_t device_dev_t;
typedef struct __device_ch_t device_ch_t;
typedef struct __device_ISR_BH_ARG_TYPE device_isr_bh_arg_t;

//static struct pci_device_id dscc4_ids[] =
//    {
//    {
//    .vendor = SBS_DSCC4_VENDOR_ID,
//    .device = SBS_DSCC4_DEVICE_ID
//    },
//    { 0, }
//};
//
//MODULE_DEVICE_TABLE(pci, dscc4_ids);

#if 0
static struct pci_driver device_pci_driver = {
    .name = DEVICE_NAME,
    //.id_table = device_ids,
    .probe =    device_probe,
    .remove =   device_remove,
};      /* device_pci_driver */
#endif


/*======================================================================
 * Definition of device channel
 *====================================================================*/

// #define MAX_RX_FIFO 10

struct __device_ch_t {
    struct cdev             cdev;                   // char device structure
    device_dev_t            *dev;
    int                     cdev_init;              // CDEV initialized
    int                     id;                     // channel ID
    int                     minor;                  // channel minor number
    char                    name[32];               // channel name
    int                     ref_count;              // reference count
    struct semaphore        sem;                    // mutual exclusion semaphore
    struct semaphore        w_sem;                  // write mutual exclusion semaphore
    struct semaphore        r_sem;                  // read mutual exclusion semaphore

    wait_queue_head_t       tx_event;               // hss_tx and hss_drv
    //cmdr_t                  cmdr;
    //ccr0_t                  drv_ccr0;
    spinlock_t              lock;                   // sync spinlock 
    //_iq_t              iqscctx;
    //_iq_t              iqsccrx;
    //_sm_t              sm;                     // Serial Port Mode. 
    //_sm_t              prot;
    int                     tx_reset;               // hss_tx and hss_drv
    int                     tx_initialized;         // hss_tx and hss_drv
    int                     write_result;
    int                     read_result;
    int                     open_result;
    int                     fasync_result;
    int                     drv_ioctl_result;

//    device_setup_scc_t         setup;
    //device_phi_t               phy_interface;
    termios_t               t;

    //u32                     scc_copy[SCC_REG_MAX];  // copy as per errata

    struct fasync_struct    *async_queue;           // for fasync
    // hss_isr
    //dscc4_iv_t              rx_iv;
    //dscc4_iv_t              tx_iv;
    unsigned long           rx_isr_flags;

    // _isr and _ioctl
    ssize_t                 flen;                   // receive frame length
    int                     toie;                   // timeout indication enabled automatic processing

    // _tx
    //dscc4_dl_t              dltx;                   // transmit descriptor list
    int                     tx_result;
    int                     write_count;
    unsigned long           tx_flags;

    // _ioctl
    int                     hdplx;                  // duplex mode, 0-full, 1-half
    int                     ioctl_result;
    unsigned long           ioctl_flags;
    //gcmdr_t                 ioctl_gcmdr;
    int                     ioctl_val;              // used in __update_setup_from_termios
    //iqlenr0_t               ioctl_iqlenr0;          // used in __hss_set_iqsccrx routine
    //iqlenr1_t               ioctl_iqlenr1;          // used in __hss_set_iqcfg_size
    // used in setup routine
    unsigned long           ioctl_iq_flags;
    unsigned long           ioctl_iqp_flags;
    unsigned long           ioctl_rx_flags;
    unsigned long           ioctl_tx_flags;
    unsigned long           setup_flags;
    //fifocr1_t               setup_fifocr1;
    //fifocr2_t               setup_fifocr2;
    //fifocr3_t               setup_fifocr3;
    //fifocr4_t               setup_fifocr4;
    //isr_t                   setup_imr;              //    = {0x0005F3FF};
    //ccr0_t                  setup_ccr0;


    //ccr1_t                  setup_ccr1;
    //ccr2_t                  setup_ccr2;
    //xnxf_t                  setup_xnxf;
    //tcr_t                   setup_tcr;
    //timr_t                  setup_timr;
    //brr_t                   setup_brr;
    //syncr_t                 setup_syncr;
    //star_t                  setup_star;
    int                     setup_shift;
    int                     setup_ch_num;
    // used in device_set_phinterface
    //ccr0_t                  set4_ccr0;
    unsigned char           set4_tmp;
    //gpdata_t                set4_gpdata;
    // used in device_set_phinterface
    //ccr0_t                  set8_ccr0;
    unsigned char           set8_tmp;
    //gpdata_t                set8_gpdata;
    int                     set8_shift;

    // used in set_phinterface
    //gpdata_t                set12_gpdata;

    //gpdir_t                 set12_gpdir;
    //gpim_t                  set12_gpim;
    //ccr0_t                  set12_ccr0;
    int                     set12_ch_num;
    int                     set12_idx;
    int                     set12_shift;
    int                     set12_result;
    unsigned char           set12_tmp;
    unsigned char           set12_clk_hdplx;        // = 0; receive clock on BI_CLK
    unsigned char           set12_bi_clk_term;      // = 0; A/C termination on BI_CLK
    unsigned char           set12_rx_clk_term;      // = 0; no A/C termination on RX_CLK
    unsigned char           set12_rx_data_term;     // = 0; no A/C termination on RX_DATA
    unsigned char           set12_rs485;            // = 0; RS232 Mode
    unsigned char           set12_tx_clk_en;        // = 0; disable TX clock
    unsigned char           set12_fast;             // = 1; Fast rates
    unsigned char           set12_tx_data_en;       // = 1; Data enable
    unsigned char           set12_failsafe_rx_term; // = 1;
// used in hss12_program_do
    int                     prog12_idx;
    //gpdata_t                prog12_gpdata;

    //ccr1_t                  rts_ccr1;
    //star_t                  cts_star;
    //star_t                  cd_star;

    // hss_cmd
    //star_t                  cmd_star;
    //brr_t                   cmd_brr;
    int                     cmd_idx;
    //ccr0_t                  cmd_ccr0;
    u32                     cmd_ccr0_bak;
    u32                     cmd_brr_bak;
    int                     cmd_to;
    unsigned long           cmd_flags;

    // in routine dscc4_poll in hss_drv
    int                     rx_buffer_num;
    int                     f_flags;
    unsigned long           poll_flags;
    unsigned int            retmask;

}; /* struct __device_ch_t */

/*======================================================================
 * Definition of device device
 *====================================================================*/
struct __device_dev_t {
    struct                  list_head entry;        // device list entry
    int                     id;                     // device ID
    spinlock_t              lock;                   // HW access spinlock
    char                    name[32];               // device name
    int                     ref_count;              // reference count
    struct pci_dev          *pcidev;                // kernel PCI device
    unsigned long           mem_addr;               // On-board memory base address
    unsigned long           mem_len;                // On-board memory length
    void                    *mem;                   // On-board memory virtual base address
    int                     ch_num;                 // number of channels
    int                     arack;
    wait_queue_head_t       iqcfg_q;
    struct semaphore        sem;                    // sync semaphore

    int                     set12_idx;
    int                     dev_controller_count;

    //struct workqueue_struct *wq;  /* work queue */
    int                     drv_isr_idx;
    unsigned long           isr_flags;
    int                     gcmd_result;
    unsigned long           gcmd_flags;
}; /* struct __device_dev_t */

