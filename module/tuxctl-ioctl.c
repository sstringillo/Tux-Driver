/* tuxctl-ioctl.c
 *
 * Driver (skeleton) for the mp2 tuxcontrollers for ECE391 at UIUC.
 *
 * Mark Murphy 2006
 * Andrew Ofisher 2007
 * Steve Lumetta 12-13 Sep 2009
 * Puskar Naha 2013
 */

#include <asm/current.h>
#include <asm/uaccess.h>

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/sched.h>
#include <linux/file.h>
#include <linux/miscdevice.h>
#include <linux/kdev_t.h>
#include <linux/tty.h>
#include <linux/spinlock.h>

#include "tuxctl-ld.h"
#include "tuxctl-ioctl.h"
#include "mtcp.h"

#define debug(str, ...) \
	printk(KERN_DEBUG "%s: " str, __FUNCTION__, ## __VA_ARGS__)

/************************ Protocol Implementation *************************/

/* tuxctl_handle_packet()
 * IMPORTANT : Read the header for tuxctl_ldisc_data_callback() in 
 * tuxctl-ld.c. It calls this function, so all warnings there apply 
 * here as well.
 */
void tuxctl_handle_packet (struct tty_struct* tty, unsigned char* packet)
{
    unsigned a, b, c;

    a = packet[0]; /* Avoid printk() sign extending the 8-bit */
    b = packet[1]; /* values when printing them. */
    c = packet[2];
	
    printk("packet : %x %x %x\n", a, b, c); 
}

/******** IMPORTANT NOTE: READ THIS BEFORE IMPLEMENTING THE IOCTLS ************
 *                                                                            *
 * The ioctls should not spend any time waiting for responses to the commands *
 * they send to the controller. The data is sent over the serial line at      *
 * 9600 BAUD. At this rate, a byte takes approximately 1 millisecond to       *
 * transmit; this means that there will be about 9 milliseconds between       *
 * the time you request that the low-level serial driver send the             *
 * 6-byte SET_LEDS packet and the time the 3-byte ACK packet finishes         *
 * arriving. This is far too long a time for a system call to take. The       *
 * ioctls should return immediately with success if their parameters are      *
 * valid.                                                                     *
 *                                                                            *
 ******************************************************************************/
int 
tuxctl_ioctl (struct tty_struct* tty, struct file* file, 
	      unsigned cmd, unsigned long arg)
{
    switch (cmd) {
	case TUX_INIT:
		return tux_init_helper(tty);
	case TUX_BUTTONS:
	case TUX_SET_LED:
		return tux_set_leds(tty,arg);	
	case TUX_LED_ACK:
	case TUX_LED_REQUEST:
	case TUX_READ_LED:
	default:
	    return -EINVAL;
    }
}

int tux_init_helper(struct tty_struct* tty){
	unsigned char buf[2];
	buf[0]= MTCP_BIOC_ON;
	buf[1]= MTCP_LED_USR;
	return tuxctl_ldisc_put(tty,buf,2);
}


int tux_set_leds(struct tty_struct* tty, unsigned long arg){
	unsigned char displayvals[18] = {0xE7,0x06,0xCB,0x8F,0x2E,0xAD,0xED,0x86,0xEF,0xAE,0xEE,0x6D,0xE1,0x4F,0xE9,0xE8,0x00,0x10};
	unsigned char buf[6];
	buf[0] = MTCP_LED_SET;
	buf[1]=0x0F; //which leds to display 
	int hexval,leds_on,dec_point;
	hexval = arg & 0xFFFF; //16-bit hex val to display
	leds_on = arg >>16; //IF LED IS 1 CONT LIKE NORMAL, IF LED IS 0 SET THAT CHECK TO BLANK
	leds_on = leds_on & 0x000F;
	dec_point = arg >>24;
	dec_point = dec_point & 0x000F; //which decimal points to display
	int i,check,dec_check,led_check;
	for(i=0;i<4;i++){
		check = hexval >> (4*i);
		check = check & 0x000F;
		led_check = leds_on;
		led_check = led_check >> i;
		led_check = led_check & 0x0001;
		dec_check = dec_point;
		dec_check = dec_check >> i;
		dec_check = dec_check & 0x0001;
		dec_check = dec_check << 4;
		if(led_check==0){
			if(dec_check==0){
				check = 1000;
				buf[i+2]= displayvals[16];
			}
			else{
				check=1000;
				buf[i+2]= displayvals[17];
			}
		}
		switch(check){
			case 0x0:
				check=displayvals[0];
				buf[i+2] = check | dec_check; 
				break;
			case 0x1:
				check=displayvals[1];
				buf[i+2] = check | dec_check; 
				break;
			case 0x2:
				check=displayvals[2];
				buf[i+2] = check | dec_check; 
				break;
			case 0x3:
				check=displayvals[3];
				buf[i+2] = check | dec_check; 
				break;
			case 0x4:
				check=displayvals[4];
				buf[i+2] = check | dec_check; 
				break;
			case 0x5:
				check=displayvals[5];
				buf[i+2] = check | dec_check; 
				break;
			case 0x6:
				check=displayvals[6];
				buf[i+2] = check | dec_check; 
				break;
			case 0x7:
				check=displayvals[7];
				buf[i+2] = check | dec_check; 
				break;
			case 0x8:
				check=displayvals[8];
				buf[i+2] = check | dec_check; 
				break;
			case 0x9:
				check=displayvals[9];
				buf[i+2] = check | dec_check; 
				break;
			case 0xA:
				check=displayvals[10];
				buf[i+2] = check | dec_check; 
				break;
			case 0xB:
				check=displayvals[11];
				buf[i+2] = check | dec_check; 
				break;
			case 0xC:
				check=displayvals[12];
				buf[i+2] = check | dec_check; 
				break;
			case 0xD:
				check=displayvals[13];
				buf[i+2] = check | dec_check; 
				break;
			case 0xE:
				check=displayvals[14];
				buf[i+2] = check | dec_check; 
				break;
			case 0xF:
				check=displayvals[15];
				buf[i+2] = check | dec_check; 
				break;
			default:
				check=0;
				break; 
		}

	}
	return tuxctl_ldisc_put(tty,buf,6);
}


