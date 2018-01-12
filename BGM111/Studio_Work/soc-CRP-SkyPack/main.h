/*
 * main.h
 *
 *  Created on: Oct 13, 2017
 *      Author: rprui
 */

#ifndef MAIN_H_
#define MAIN_H_

#define STR_LEN			25		// Length of String
#define BUFFSIZE		256		// Transmit Buffer Size
#define	UART_EP			2		// Selected UART endpoint. 2 = UART1, 5 = UART0
#define	LED0_BANK		5		// LED0 is mapped to pin F6
#define LED0_PIN_MASK	0x40
#define	LED1_BANK		5		// LED1 is mapped to pin F7
#define LED1_PIN_MASK	0x80

#define VERSION_NUM     "1.0"                 // Monitor Revision
#define REL_DATE        "Jan 12, 2018"
#define LEGACY_BANNER   "1.0 1/12/18"        // Needed to allow Legacy Design to work

#endif /* MAIN_H_ */
