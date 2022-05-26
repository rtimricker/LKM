
/*
 *  ioctl.c - the process to use ioctl's to control the kernel module
 *
 *  Until now we could have used cat for input and output.  But now
 *  we need to do ioctl's, which require writing our own process.
 */

/* 
 * device specifics, such as ioctl numbers and the
 * major device file. 
 */
//#include "chardev.h"

#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>		/* open */
#include <unistd.h>		/* exit */
#//include <sys/ioctl.h>		/* ioctl */
#include <errno.h>
#include <string.h>
#if 0
/* 
 * Functions for the ioctl calls 
 */

void ioctl_set_msg(int file_desc, char *message)
{
	int ret_val;

	ret_val = ioctl(file_desc, IOCTL_SET_MSG, message);

	if (ret_val < 0) {
		printf("ioctl_set_msg failed:%d\n", ret_val);
		exit(-1);
	}
}

void ioctl_get_msg(int file_desc)
{
	int ret_val;
	char message[100];

	/* 
	 * Warning - this is dangerous because we don't tell
	 * the kernel how far it's allowed to write, so it
	 * might overflow the buffer. In a real production
	 * program, we would have used two ioctls - one to tell
	 * the kernel the buffer length and another to give
	 * it the buffer to fill
	 */
	ret_val = ioctl(file_desc, IOCTL_GET_MSG, message);

	if (ret_val < 0) {
		printf("ioctl_get_msg failed:%d\n", ret_val);
		exit(-1);
	}

	printf("get_msg message:%s\n", message);
}

void ioctl_get_nth_byte(int file_desc)
{
	int i;
	char c;

	//printf("get_nth_byte message:\n");

	i = 0;
	do {
		c = ioctl(file_desc, IOCTL_GET_NTH_BYTE, i++);

		if (c < 0) {
			printf
			    ("ioctl_get_nth_byte failed at the %d'th byte:\n",
			     i);
			exit(-1);
		}

		putchar(c);
	} while (c != 0);
	putchar('\n');
}
#endif
/* 
 * Main - Call the ioctl functions 
 */

#define BUFFER_LENGTH 256 // The buffer length (crude but fine)
static char receive[BUFFER_LENGTH]; // The receive buffer from the LKM

int main()
{
	char receive[100]; // The receive buffer from the LKM
	char stringToSend[100] = "something";
    int bytes_written = 0;
    int bytes_read = 0;
	int file_desc, ret_val;
	//char msg[] = "--> Message passed by ioctl\n";
    char fileName[] = "/dev/char_dev";
    printf("Starting device test code example... [%s]\n", fileName);
    file_desc = open(fileName, O_RDWR); // Open the device with read/write access
    if (file_desc < 0){
        perror("Failed to open the device...");
        return errno;
    }

	printf("Type in a short string to send to the kernel module:\n");
 	scanf("%[^\n]%*c", stringToSend); // Read in a string (with spaces)
    printf("Writing message to the device [%s].\n", stringToSend);

	int bytes = write(file_desc, stringToSend, strlen(stringToSend)); // Send the string to the LKM
    printf ("bytes_written: %d\n", bytes);

    printf("Press ENTER to read back from the device...\n");
    getchar();

    printf("Reading from the device...\n");
    bytes_read = read(file_desc, receive, BUFFER_LENGTH); // Read the response from the LKM
    if (bytes_read < 0){
        perror("Failed to read the message from the device.");
        return errno;
    }

    printf ("bytes_read: %d\n", bytes_read);
    printf("The received message is: [%s]\n", receive);
    printf("End of the program\n");
#if 0
	ioctl_get_nth_byte(file_desc);
	ioctl_get_msg(file_desc);
	ioctl_set_msg(file_desc, msg);
#endif
	close(file_desc);
    return 0;
}
