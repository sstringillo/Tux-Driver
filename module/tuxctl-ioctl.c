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


unsigned long buttons;
unsigned char globbuffer[6];
int ack; //if 0 we can do stuff, if 1 we are "locked"
static spinlock_t myspinlock;
/************************ Protocol Implementation *************************/

/* tuxctl_handle_packet()
 * IMPORTANT : Read the header for tuxctl_ldisc_data_callback() in 
 * tuxctl-ld.c. It calls this function, so all warnings there apply 
 * here as well.
 */
void tuxctl_handle_packet (struct tty_struct* tty, unsigned char* packet)
{
    unsigned a, b, c;
	//unsigned up,left,down,right,start,tux_a,tux_b,tux_c;
    a = packet[0]; /* Avoid printk() sign extending the 8-bit */
    b = packet[1]; /* values when printing them. */
    c = packet[2];
	unsigned buttoncheck;
	
	switch (a){
		case MTCP_ACK:
			ack = 0;
			break;
		case MTCP_RESET:
			tux_init_helper(tty);
			tuxctl_ldisc_put(tty,globbuffer,6);
			break;
		case MTCP_BIOC_EVENT:
				if(ack==1){
					return -1;
				}
				unsigned long flags;
				spin_lock_irqsave(&myspinlock,flags);
				ack=1;
				buttoncheck = b & 0x0F;
				if(buttoncheck==0x0E){ //start
					buttons = 0xFE;
				}
				if(buttoncheck==0x0D){ //a
					buttons = 0xFD;
				}
				if(buttoncheck==0x0B){ //b
					buttons = 0xFB;
				}
				if(buttoncheck==0x07){ //c
					buttons = 0xF7;
				}

				buttoncheck = c & 0x0F;
				if(buttoncheck==0x0E){ //up
					buttons = 0xEF;
				}
				if(buttoncheck==0x0D){ //left
					buttons = 0xBF;
				}
				if(buttoncheck==0x0B){ //down
					buttons = 0xDF;
				}
				if(buttoncheck==0x07){ //right
					buttons = 0x7F;
				}
				ack=0;
				spin_unlock_irqrestore(&myspinlock, flags);
				printk("buttons1 %x", buttons);
				break; 

	}
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
		return tux_buttons_helper(tty, (unsigned long*)arg);
	case TUX_SET_LED:
		return tux_set_leds(tty,arg);	
	case TUX_LED_ACK: //no
	case TUX_LED_REQUEST: //nah
	case TUX_READ_LED: //dont need to do
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
	if(ack==1){
		return -1;
	}
	unsigned long flags;
	spin_lock_irqsave(&myspinlock,flags);
	ack=1;
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
			globbuffer[0]= MTCP_LED_SET;
			globbuffer[1] = 0xFF;
			globbuffer[2] = buf[2];
			globbuffer[3] = buf[3];
			globbuffer[4] = buf[4];
			globbuffer[5] = buf[5];
			
	}
	ack=0;
	spin_unlock_irqrestore(&myspinlock, flags);
	return tuxctl_ldisc_put(tty,buf,6);
}

int tux_buttons_helper(struct tty_struct* tty, unsigned long arg){
	if((unsigned long*)arg==NULL){
		return -EINVAL;
	}
	unsigned long flags;
	spin_lock_irqsave(&myspinlock,flags);
	//ack=1;
	int copycheck;
	copycheck = copy_to_user(arg,buttons,sizeof(buttons));
	if(copycheck!=0){
		return -EINVAL;
	}

	//ack=0;
	spin_unlock_irqrestore(&myspinlock, flags);
	return 0; 
	
	

}


