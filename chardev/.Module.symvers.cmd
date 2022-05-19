cmd_/home/timr/LKM/chardev/Module.symvers := sed 's/\.ko$$/\.o/' /home/timr/LKM/chardev/modules.order | scripts/mod/modpost -m -a  -o /home/timr/LKM/chardev/Module.symvers -e -i Module.symvers   -T -
