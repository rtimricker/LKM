cmd_/home/timr/LKM/chardev/modules.order := {   echo /home/timr/LKM/chardev/chardev.ko; :; } | awk '!x[$$0]++' - > /home/timr/LKM/chardev/modules.order
