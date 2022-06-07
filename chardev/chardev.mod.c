#include <linux/module.h>
#define INCLUDE_VERMAGIC
#include <linux/build-salt.h>
#include <linux/elfnote-lto.h>
#include <linux/vermagic.h>
#include <linux/compiler.h>

BUILD_SALT;
BUILD_LTO_INFO;

MODULE_INFO(vermagic, VERMAGIC_STRING);
MODULE_INFO(name, KBUILD_MODNAME);

__visible struct module __this_module
__section(".gnu.linkonce.this_module") = {
	.name = KBUILD_MODNAME,
	.init = init_module,
#ifdef CONFIG_MODULE_UNLOAD
	.exit = cleanup_module,
#endif
	.arch = MODULE_ARCH_INIT,
};

#ifdef CONFIG_RETPOLINE
MODULE_INFO(retpoline, "Y");
#endif

static const struct modversion_info ____versions[]
__used __section("__versions") = {
	{ 0xd9726f80, "module_layout" },
	{ 0xc959d152, "__stack_chk_fail" },
	{ 0xe9412c50, "proc_remove" },
	{ 0xc5645b17, "class_unregister" },
	{ 0xe340d421, "device_destroy" },
	{ 0x977f511b, "__mutex_init" },
	{ 0x999e8297, "vfree" },
	{ 0xbe8b05f8, "proc_create" },
	{ 0xd6ee688f, "vmalloc" },
	{ 0xe914e41e, "strcpy" },
	{ 0x64b60eb0, "class_destroy" },
	{ 0x9f4f34bc, "device_create" },
	{ 0x6bc3fbc0, "__unregister_chrdev" },
	{ 0xa946dcde, "__class_create" },
	{ 0x4102c7cb, "__register_chrdev" },
	{ 0x409bcb62, "mutex_unlock" },
	{ 0xc3aaf0a9, "__put_user_1" },
	{ 0x167e7f9d, "__get_user_1" },
	{ 0xc5850110, "printk" },
	{ 0xbdfb6dbb, "__fentry__" },
};

MODULE_INFO(depends, "");


MODULE_INFO(srcversion, "383B24ED4006727BC965E0F");
