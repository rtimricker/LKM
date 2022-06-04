cmd_/home/timr/LKM/link_list/modules.order := {   echo /home/timr/LKM/link_list/link_list_del.ko; :; } | awk '!x[$$0]++' - > /home/timr/LKM/link_list/modules.order
