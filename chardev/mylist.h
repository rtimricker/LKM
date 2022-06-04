

#ifndef MYLIST_H 
#define MYLIST_H 

#include <linux/input.h> 
#include <linux/spinlock.h> 


struct io_device; 
 
struct _io { 
    long id; 
    long devno; 
    long last_entry; 
    spinlock_t lock; 
 
    void *priv_data; 
 
    struct device dev; 
    struct list_head list; 
    struct input_dev *input; 
    struct io_device *type; 
}; 

struct io_ops { 
    int (*init)(struct _io *); 
    int (*kill)(struct _io *); 
    int (*send)(struct _io *, char *, int); 
    int (*read)(struct _io *, char *, int); 
}; 

struct io_device { 
    char name[16]; 
    struct list_head list; 
    struct io_ops *ops; 
}; 

#endif