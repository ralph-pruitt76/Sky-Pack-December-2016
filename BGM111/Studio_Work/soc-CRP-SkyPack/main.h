/*
 * main.h
 *
 *  Created on: Oct 13, 2017
 *      Author: rprui
 */

#ifndef MAIN_H_
#define MAIN_H_

#define STR_LEN				25		// Length of String
#define BUFFSIZE			1536	// Transmit and Receive Buffer Size
#define MAX_BBFRWRT_ERRS	3		// Only Allow three of these errors.
#define	UART_EP				2		// Selected UART endpoint. 2 = UART1, 5 = UART0
#define	LED0_BANK			5		// LED0 is mapped to pin F6
#define LED0_PIN_MASK		0x40
#define	LED1_BANK			5		// LED1 is mapped to pin F7
#define LED1_PIN_MASK		0x80
#define BUFFER_LNGTH		80		// Length of Generic Buffer used in Main.

#define VERSION_NUM     "1.2"                 // Monitor Revision
#define REL_DATE        "Jan 24, 2018"
#define LEGACY_BANNER   "1.3 1/24/18"        // Needed to allow Legacy Design to work

#endif /* MAIN_H_ */
