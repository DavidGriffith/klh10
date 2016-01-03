/* DVLITES.H - Driver for Spare Time Gizmos Panda Display lights
*/

/*  Copyright 2005 Mark Crispin
**  All Rights Reserved
**
**  Modified in 2016 by David Griffith
**  to support the new USB-based Panda Display
**
**
**  This file is part of the KLH10 Distribution.  Use, modification, and
**  re-distribution is permitted subject to the terms in the file
**  named "LICENSE", which contains the full text of the legal notices
**  and should always accompany this Distribution.
**
**  This program is provide "AS IS" with NO WARRANTY OF ANY KIND.
**
**  This notice (including the copyright and warranty disclaimer)
**  must be included in all copies or derivations of this software.
*/

#if KLH10_DEV_LITES		/* Moby conditional */


#define LITEBUF_SIZE	6
#define STRINGBUF	256

/*
 * This is a vendor/device ID pair belonging to Objective Development
 * and is offered for free for users of their V-USB package.  V-USB is a
 * library for use with AVR microcontrollers that provides USB
 * connectivity for chips that lack dedicated USB hardware.  Because we
 * can conceivably have a device attached that has these ID numbers, but
 * is not a Panda Display, we must check the device name to be sure.
 *
 */
#define VENDOR_ID	0x16c0	/* Van Ooijen Technische Informatica */
#define PRODUCT_ID	0x05df  /* HID device except mice, keyboards, and joysticks */
#define PRODNAME	"Panda Display"
#define PRODNAME_LEN	13

/* General routines for all displays */

				/* one-time initialize and set port */
int lites_init (void);
int lites_shutdown(void);

				/* write data LEDs on current unit */
void lites_data (unsigned char data[5]);

				/* write status LEDs on current unit */
void lites_status (unsigned char data);

#define STATUS_LED1 0x80	/* status LED1 */
#define STATUS_LED2 0x40	/* status LED2 */
#define STATUS_LED3 0x20	/* status LED3 */
#define STATUS_LED4 0x10	/* status LED4 */
#define STATUS_LEDS (STATUS_LED1 | STATUS_LED2 | STATUS_LED3 | STATUS_LED4)


/* Specific routines for display 0 */

				/* write program status lights */
void lights_pgmlites (unsigned long lh,unsigned long rh);
				/* write program auxillary lights */
void lights_pgmaux (unsigned char aux);
				/* system heartbeat */
void lights_status (char cpu,char disk,char tape,char net,char push);
#else	/* KLH10_DEV_LITES */
#define lights_pgmlites(lh,rh)
#define lights_pgmaux(aux)
#define lights_status(cpu,disk,tape,net,push)
#endif /* KLH10_DEV_LITES */
