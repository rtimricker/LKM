/*Code to show usage of list_del . Tested in kernel 5.3*/

#include<linux/kernel.h>
#include<linux/module.h>
#include<linux/list.h>
#include<linux/slab.h>

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Roger Tim Ricker");
MODULE_DESCRIPTION("A sample character driver");

struct k_list {
    struct list_head test_list;
    int temp;
};


int create_list_init(void) {

    struct k_list *one,*two,*three,*entry;
    struct list_head test_head;
    struct list_head *ptr;
    int i=0;
    one=kmalloc(sizeof(struct k_list),GFP_KERNEL);
    two=kmalloc(sizeof(struct k_list),GFP_KERNEL);
    three=kmalloc(sizeof(struct k_list),GFP_KERNEL);

    one->temp=10;
    two->temp=20;
    three->temp=30;

    INIT_LIST_HEAD(&test_head);
    list_add_tail(&one->test_list,&test_head);
    list_add_tail(&two->test_list,&test_head);
    list_add_tail(&three->test_list,&test_head);
    ptr=&(one->test_list);
    list_for_each(ptr,&test_head) {
        entry=list_entry(ptr,struct k_list,test_list);
        i=i+1;
        printk(KERN_INFO "\n Entry %d %d  \n \n\n ", i,entry->temp);
    }
    i=0;
    printk(KERN_INFO " Deleting one entry\n \n\n ");
    list_del(&(one->test_list));

    list_for_each(ptr,&test_head) {
        entry=list_entry(ptr,struct k_list,test_list);
        i=i+1;
        printk(KERN_INFO "\n Entry %d %d  \n \n\n ", i,entry->temp);
    }

    printk(KERN_INFO "\n Done....  \n \n\n ");
    return 0;
}

void create_list_exit(void) {
    //return 0 ;
}

module_init(create_list_init);
module_exit(create_list_exit);

