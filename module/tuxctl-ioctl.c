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
	//printk(KERN_DEBUG "%s: " str, __FUNCTION__, ## __VA_ARGS__)

#define BUFFERSIZE 6 
#define BUTTONCHECK 0x0F
#define BUTTONLVL1 0x0E
#define BUTTONLVL2 0x0D
#define BUTTONLVL3 0x0B
#define BUTTONLVL4 0x07
#define START 0xFE
#define A 0xFD
#define B 0xFB
#define C 0xF7
#define UP 0xEF
#define LEFT 0xBF
#define DOWN 0xDF
#define RIGHT 0x7F


unsigned int buttons;
unsigned char globbuffer[BUFFERSIZE];
int ack; //if 0 we can do stuff, if 1 we are "locked"
static spinlock_t myspinlock;
unsigned long flags;
/************************ Protocol Implementation *************************/

/* tuxctl_handle_packet()
 * IMPORTANT : Read the header for tuxctl_ldisc_data_callback() in 
 * tuxctl-ld.c. It calls this function, so all warnings there apply 
 * here as well.
 */

/*
 * tuxctl_handle_packet()
 *   DESCRIPTION: Takes a 3 byte packet and based on signal in the first packet
 * 				  responds accordingly	  
 *   INPUTS: tty - line discipline, used to recieve commands 
 *           packet - 3 btye packed contain opcode, and values of tux buttons
 *   OUTPUTS: none
 *   RETURN VALUE: none
 *   SIDE EFFECTS: based on opcode we either reset, recieve and ack, set LEDs, etc... 
 */
void tuxctl_handle_packet (struct tty_struct* tty, unsigned char* packet)
{
    unsigned a, b, c;
	unsigned buttoncheck;
    a = packet[0]; /* Avoid printk() sign extending the 8-bit */
    b = packet[1]; /* values when printing them. */
    c = packet[2];
	
	switch (a){
		case MTCP_ACK:
			ack = 0;
			break;
		case MTCP_RESET:
			//ack and then init, reset leds
			//ack = 0;
			tux_init_helper(tty);
			tuxctl_ldisc_put(tty,globbuffer,BUFFERSIZE);
			ack = 1;
			break;
		case MTCP_BIOC_EVENT:
				buttons = 0x00;
				//if(ack==1){
				//	return -1;
				//}
				//unsigned long flags;
				//spin_lock_irqsave(&myspinlock,flags);
				//ack=1;
				buttoncheck = b & BUTTONCHECK;
				if(buttoncheck==BUTTONLVL1){ //start
					buttons = START;
				}
				if(buttoncheck==BUTTONLVL2){ //a
					buttons = A;
				}
				if(buttoncheck==BUTTONLVL3){ //b
					buttons = B;
				}
				if(buttoncheck==BUTTONLVL4){ //c
					buttons = C;
				}

				buttoncheck = c & BUTTONCHECK;
				if(buttoncheck==BUTTONLVL1){ //up
					buttons = UP;
				}
				if(buttoncheck==BUTTONLVL2){ //left
					buttons = LEFT;
				}
				if(buttoncheck==BUTTONLVL3){ //down
					buttons = DOWN;
				}
				if(buttoncheck==BUTTONLVL4){ //right
					buttons = RIGHT;
				}
				//ack=0;
				//spin_unlock_irqrestore(&myspinlock, flags);
				break; 

	}
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

/*
 * tuxctl_ioctl()
 *   DESCRIPTION: Based on our given cmd, we have our tux do a list of things such as init,
 * 				  get button values, set the LEDs, etc...	  
 *   INPUTS: tty - line discipline, used to recieve commands 
 *           file - 
 * 			 cmd - commands for us to execute
 * 			 arg - value passed through for us to use, such as led values or button pressed values
 *   OUTPUTS: none
 *   RETURN VALUE: acknowledgement of our functions completion
 *   SIDE EFFECTS: based on opcode we either reset, recieve and ack, set LEDs, etc... 
 */
int 
tuxctl_ioctl (struct tty_struct* tty, struct file* file, 
	      unsigned cmd, unsigned long arg)
{
    switch (cmd) {
	case TUX_INIT:
		return tux_init_helper(tty);
	case TUX_BUTTONS:
		return tux_buttons_helper(tty, arg);
	case TUX_SET_LED:
		return tux_set_leds(tty,arg);	
	case TUX_LED_ACK: //unused
	case TUX_LED_REQUEST: //unused
	case TUX_READ_LED: //unused
	default:
	    return -EINVAL;
    }
}

#define SBUFFERSIZE 2 

/*
 * tux_init_helper()
 *   DESCRIPTION: initilizes our tux with the given opcodes to prepare it for use	  
 *   INPUTS: tty - line discipline, used to recieve commands 
 *   OUTPUTS: none
 *   RETURN VALUE: acknowledgement
 *   SIDE EFFECTS: initilizes our tux for use 
 */
int tux_init_helper(struct tty_struct* tty){
	//init by sending tux these 2 signals
	unsigned char buf[SBUFFERSIZE];
	ack = 1;
	buf[0]= MTCP_BIOC_ON;
	buf[1]= MTCP_LED_USR;
	return tuxctl_ldisc_put(tty,buf,SBUFFERSIZE);
}

#define LEDBUFSET 0x0F
#define HEXMASK 0xFFFF
#define LEDMASK 0x000F
#define LSHIFT 16
#define BSHIFT 24
#define ANDMASK 0x0001
#define RSHIFT 4
#define IMPOSVAL 1000
#define BOFFSET 2
#define LOOPER0 0
#define LOOPER1 1
#define LOOPER2 2
#define LOOPER3 3
#define LOOPER4 4
#define LOOPER5 5
#define LOOPER6 6
#define LOOPER7 7
#define LOOPER8 8
#define LOOPER9 9
#define LOOPER10 10
#define LOOPER11 11
#define LOOPER12 12
#define LOOPER13 13
#define LOOPER14 14
#define LOOPER15 15
#define LOOPER16 16
#define LOOPER17 17
#define MAXARR 18
#define LEDSETTER0 0xE7
#define LEDSETTER1 0x06
#define LEDSETTER2 0xCB
#define LEDSETTER3 0x8F
#define LEDSETTER4 0x2E
#define LEDSETTER5 0xAD
#define LEDSETTER6 0xED
#define LEDSETTER7 0x86
#define LEDSETTER8 0xEF
#define LEDSETTER9 0xAE
#define LEDSETTER10 0xEE
#define LEDSETTER11 0x6D
#define LEDSETTER12 0xE1
#define LEDSETTER13 0x4F
#define LEDSETTER14 0xE9
#define LEDSETTER15 0xE8
#define LEDSETTER16 0x00
#define LEDSETTER17 0x10

/*
 * tux_init_helper()
 *   DESCRIPTION: sets the LEDs on the tux  
 *   INPUTS: tty - line discipline, used to recieve commands
 *           arg - value to display on the LEDs 
 *   OUTPUTS: none
 *   RETURN VALUE: acknowledgement
 *   SIDE EFFECTS: tux LEDs light up with correct value
 */
int tux_set_leds(struct tty_struct* tty, unsigned long arg){
	int hexval,leds_on,dec_point; //values of LEDs
	int i,check,dec_check,led_check; //variables to check input 
	unsigned char displayvals[MAXARR];
	unsigned char buf[BUFFERSIZE];
	if(ack==1){
		return -1;
	}
	//unsigned long flags;
	//spin_lock_irqsave(&myspinlock,flags);
	ack=1;
	//unsigned char displayvals[MAXARR] = {0xE7,0x06,0xCB,0x8F,0x2E,0xAD,0xED,0x86,0xEF,0xAE,0xEE,0x6D,0xE1,0x4F,0xE9,0xE8,0x00,0x10};

	//stores how leds should look for specific number/letter
	displayvals[LOOPER0] = LEDSETTER0;
	displayvals[LOOPER1] = LEDSETTER1;
	displayvals[LOOPER2] = LEDSETTER2;
	displayvals[LOOPER3] = LEDSETTER3;
	displayvals[LOOPER4] = LEDSETTER4;
	displayvals[LOOPER5] = LEDSETTER5;
	displayvals[LOOPER6] = LEDSETTER6;
	displayvals[LOOPER7] = LEDSETTER7;
	displayvals[LOOPER8] = LEDSETTER8;
	displayvals[LOOPER9] = LEDSETTER9;
	displayvals[LOOPER10] = LEDSETTER10;
	displayvals[LOOPER11] = LEDSETTER11;
	displayvals[LOOPER12] = LEDSETTER12;
	displayvals[LOOPER13] = LEDSETTER13;
	displayvals[LOOPER14] = LEDSETTER14;
	displayvals[LOOPER15] = LEDSETTER15;
	displayvals[LOOPER16] = LEDSETTER16;
	displayvals[LOOPER17] = LEDSETTER17;

	 
	//we always want buf[0], buf[1] these vals
	buf[LOOPER0] = MTCP_LED_SET;
	buf[LOOPER1]= LEDBUFSET;
	//LED on and decimal point on checking 
	hexval = arg & HEXMASK; 
	leds_on = arg >>LSHIFT; 
	leds_on = leds_on & LEDMASK;
	dec_point = arg >> BSHIFT;
	dec_point = dec_point & LEDMASK;
	//puts decimal into correct spot in 8 bit value 
	for(i=0;i<RSHIFT;i++){
		check = hexval >> (RSHIFT*i);
		check = check & LEDMASK;
		led_check = leds_on;
		led_check = led_check >> i;
		led_check = led_check & ANDMASK;
		dec_check = dec_point;
		dec_check = dec_check >> i;
		dec_check = dec_check & ANDMASK;
		dec_check = dec_check << RSHIFT;
		//No led on/ only dec point on checks
		if(led_check==0){
			if(dec_check==0){
				check = IMPOSVAL;
				buf[i+BOFFSET]= displayvals[LOOPER16];
			}
			else{
				check=IMPOSVAL;
				buf[i+BOFFSET]= displayvals[LOOPER17];
			}
		}
		//check what number we have and put in correct format for LEDs
		switch(check){
			case LOOPER0:
				check=displayvals[LOOPER0];
				buf[i+BOFFSET] = check | dec_check; 
				break;
			case LOOPER1:
				check=displayvals[LOOPER1];
				buf[i+BOFFSET] = check | dec_check; 
				break;
			case LOOPER2:
				check=displayvals[LOOPER2];
				buf[i+BOFFSET] = check | dec_check; 
				break;
			case LOOPER3:
				check=displayvals[LOOPER3];
				buf[i+BOFFSET] = check | dec_check; 
				break;
			case LOOPER4:
				check=displayvals[LOOPER4];
				buf[i+BOFFSET] = check | dec_check; 
				break;
			case LOOPER5:
				check=displayvals[LOOPER5];
				buf[i+BOFFSET] = check | dec_check; 
				break;
			case LOOPER6:
				check=displayvals[LOOPER6];
				buf[i+BOFFSET] = check | dec_check; 
				break;
			case LOOPER7:
				check=displayvals[LOOPER7];
				buf[i+BOFFSET] = check | dec_check; 
				break;
			case LOOPER8:
				check=displayvals[LOOPER8];
				buf[i+BOFFSET] = check | dec_check; 
				break;
			case LOOPER9:
				check=displayvals[LOOPER9];
				buf[i+BOFFSET] = check | dec_check; 
				break;
			case LOOPER10:
				check=displayvals[LOOPER10];
				buf[i+BOFFSET] = check | dec_check; 
				break;
			case LOOPER11:
				check=displayvals[LOOPER11];
				buf[i+BOFFSET] = check | dec_check; 
				break;
			case LOOPER12:
				check=displayvals[LOOPER12];
				buf[i+BOFFSET] = check | dec_check; 
				break;
			case LOOPER13:
				check=displayvals[LOOPER13];
				buf[i+BOFFSET] = check | dec_check; 
				break;
			case LOOPER14:
				check=displayvals[LOOPER14];
				buf[i+BOFFSET] = check | dec_check; 
				break;
			case LOOPER15:
				check=displayvals[LOOPER15];
				buf[i+BOFFSET] = check | dec_check; 
				break;
			default:
				check=0;
				break; 
		}
			//copy values to global buffer for reset
			globbuffer[LOOPER0]= MTCP_LED_SET;
			globbuffer[LOOPER1] = LEDBUFSET;
			globbuffer[LOOPER2] = buf[LOOPER2];
			globbuffer[LOOPER3] = buf[LOOPER3];
			globbuffer[LOOPER4] = buf[LOOPER4];
			globbuffer[LOOPER5] = buf[LOOPER5];
			
	}
	//spin_unlock_irqrestore(&myspinlock, flags);
	return tuxctl_ldisc_put(tty,buf,BUFFERSIZE);
}

/*
 * tux_buttons_helper()
 *   DESCRIPTION: allows us to communicate with user level code to get values for tux  
 *   INPUTS: tty - line discipline, used to recieve commands
 *           arg - value of pressed button on tux
 *   OUTPUTS: none
 *   RETURN VALUE: acknowledgement
 *   SIDE EFFECTS: gets value of pressed button on tux (active low)
 */
int tux_buttons_helper(struct tty_struct* tty, int arg){
	int copycheck;
	//check for invalid pointer
	spin_lock_irqsave(&myspinlock,flags);
	if((int*)arg==NULL){
		return -EINVAL;
	}
	spin_unlock_irqrestore(&myspinlock,flags);

	//copy to user to get button presses
	spin_lock_irqsave(&myspinlock,flags);
	copycheck = copy_to_user((int*)arg,&buttons,4);
	if(copycheck!=0){
		return -EINVAL;
	}
	spin_unlock_irqrestore(&myspinlock,flags);
	
	return 0;

}


