/* Name: main.c
 * Project: HID-Test
 * Author: Christian Starkjohann
 * Creation Date: 2006-02-02
 * Tabsize: 4
 * Copyright: (c) 2006 by OBJECTIVE DEVELOPMENT Software GmbH
 * License: GNU GPL v2 (see License.txt) or proprietary (CommercialLicense.txt)
 * This Revision: $Id: main.c 299 2007-03-29 17:07:19Z cs $
 */

#define F_CPU   12000000L    /* evaluation board runs on 4MHz */

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <avr/wdt.h>

// AVR-Libc stdlib.h
#include <stdlib.h>

#include "usbdrv.h"
#include "oddebug.h"

/* ----------------------- hardware I/O abstraction ------------------------ */

/* pin assignments:
PB0	(not used)
PB1	(not used)
PB2	(not used)
PB3	(not used)
PB4	(not used)
PB5 (not used)

PC0	(not used)
PC1	(not used)
PC2	(not used)
PC3	Key 3
PC4	Key 2
PC5	Key 1

PD0	USB-
PD1	debug tx
PD2	USB+ (int0)
PD3	(not used)
PD4	(not used)
PD5	(not used)
PD6	(not used)
PD7	(not used)
*/

static void hardwareInit(void)
{
uchar	i, j;

    PORTB = 0xff;   /* activate all pull-ups */
    DDRB = 0;       /* all pins input */
    PORTC = 0xff;   /* activate all pull-ups */
    DDRC = 0;       /* all pins input */
    PORTD = 0xfa;   /* 1111 1010 bin: activate pull-ups except on USB lines */
    DDRD = 0x07;    /* 0000 0111 bin: all pins input except USB (-> USB reset) */
	j = 0;
	while(--j){     /* USB Reset by device only required on Watchdog Reset */
		i = 0;
		while(--i)
			asm(""); /* delay >10ms for USB reset */
	}
    DDRD = 0x02;    /* 0000 0010 bin: remove USB reset condition */
    /* configure timer 0 for a rate of 12M/(1024 * 256) = 45.78 Hz (~22ms) */
    TCCR0 = 5;      /* timer 0 prescaler: 1024 */
}

/* ------------------------------------------------------------------------- */

#define NUM_KEYS    17

/* The following function returns an index for the first key pressed. It
 * returns 0 if no key is pressed.
 */
static uchar    keyPressed(void)
{
uchar   i, mask, x;

    x = PINB;
    mask = 1;
    for(i=0;i<6;i++){
        if((x & mask) == 0)
            return i + 1;
        mask <<= 1;
    }
    x = PINC;
    mask = 1;
    for(i=0;i<6;i++){
        if((x & mask) == 0)
            return i + 7;
        mask <<= 1;
    }
    x = PIND;
    mask = 1 << 3;
    for(i=0;i<5;i++){
        if((x & mask) == 0)
            return i + 13;
        mask <<= 1;
    }
    return 0;
}

/* ------------------------------------------------------------------------- */
/* ----------------------------- USB interface ----------------------------- */
/* ------------------------------------------------------------------------- */

static uchar    reportBuffer[2];    /* buffer for HID reports */
static uchar    idleRate;           /* in 4 ms units */

PROGMEM char usbHidReportDescriptor[35] = { /* USB report descriptor */
    0x05, 0x01,                    // USAGE_PAGE (Generic Desktop)
    0x09, 0x06,                    // USAGE (Keyboard)
    0xa1, 0x01,                    // COLLECTION (Application)
    0x05, 0x07,                    //   USAGE_PAGE (Keyboard)
    0x19, 0xe0,                    //   USAGE_MINIMUM (Keyboard LeftControl)
    0x29, 0xe7,                    //   USAGE_MAXIMUM (Keyboard Right GUI)
    0x15, 0x00,                    //   LOGICAL_MINIMUM (0)
    0x25, 0x01,                    //   LOGICAL_MAXIMUM (1)
    0x75, 0x01,                    //   REPORT_SIZE (1)
    0x95, 0x08,                    //   REPORT_COUNT (8)
    0x81, 0x02,                    //   INPUT (Data,Var,Abs)
    0x95, 0x01,                    //   REPORT_COUNT (1)
    0x75, 0x08,                    //   REPORT_SIZE (8)
    0x25, 0x65,                    //   LOGICAL_MAXIMUM (101)
    0x19, 0x00,                    //   USAGE_MINIMUM (Reserved (no event indicated))
    0x29, 0x65,                    //   USAGE_MAXIMUM (Keyboard Application)
    0x81, 0x00,                    //   INPUT (Data,Ary,Abs)
    0xc0                           // END_COLLECTION
};
/* We use a simplifed keyboard report descriptor which does not support the
 * boot protocol. We don't allow setting status LEDs and we only allow one
 * simultaneous key press (except modifiers). We can therefore use short
 * 2 byte input reports.
 * The report descriptor has been created with usb.org's "HID Descriptor Tool"
 * which can be downloaded from http://www.usb.org/developers/hidpage/.
 * Redundant entries (such as LOGICAL_MINIMUM and USAGE_PAGE) have been omitted
 * for the second INPUT item.
 */

/* Keyboard usage values, see usb.org's HID-usage-tables document, chapter
 * 10 Keyboard/Keypad Page for more codes.
 */
#define MOD_CONTROL_LEFT    (1<<0)
#define MOD_SHIFT_LEFT      (1<<1)
#define MOD_ALT_LEFT        (1<<2)
#define MOD_GUI_LEFT        (1<<3)
#define MOD_CONTROL_RIGHT   (1<<4)
#define MOD_SHIFT_RIGHT     (1<<5)
#define MOD_ALT_RIGHT       (1<<6)
#define MOD_GUI_RIGHT       (1<<7)

#define KEY_A          4
#define KEY_B          5
#define KEY_C          6
#define KEY_D          7
#define KEY_E          8
#define KEY_F          9
#define KEY_G          10
#define KEY_H          11
#define KEY_I          12
#define KEY_J          13
#define KEY_K          14
#define KEY_L          15
#define KEY_M          16
#define KEY_N          17
#define KEY_O          18
#define KEY_P          19
#define KEY_Q          20
#define KEY_R          21
#define KEY_S          22
#define KEY_T          23
#define KEY_U          24
#define KEY_V          25
#define KEY_W          26
#define KEY_X          27
#define KEY_Y          28
#define KEY_Z          29
#define KEY_1          30
#define KEY_2          31
#define KEY_3          32
#define KEY_4          33
#define KEY_5          34
#define KEY_6          35
#define KEY_7          36
#define KEY_8          37
#define KEY_9          38
#define KEY_0          39
#define KEY_ENTER      40
#define KEY_SPACE      44
#define KEY_MINUS      45
#define KEY_EQUAL      46
#define KEY_SEMICOLON  51
#define KEY_COMMA      54
#define KEY_PERIOD     55

#define KEY_F1         58
#define KEY_F2         59
#define KEY_F3         60
#define KEY_F4         61
#define KEY_F5         62
#define KEY_F6         63
#define KEY_F7         64
#define KEY_F8         65
#define KEY_F9         66
#define KEY_F10        67
#define KEY_F11        68
#define KEY_F12        69

static void buildReport(uchar key)
{
	if (key >= '0' && key <= '9') {
		reportBuffer[0] = 0;
		reportBuffer[1] = KEY_0 + key - '0';
	}
	else if (key >= 'a' && key <= 'z') {
		reportBuffer[0] = 0;
		reportBuffer[1] = KEY_A + key - 'a';
	}
	else if (key >= 'A' && key <= 'Z') {
		reportBuffer[0] = MOD_SHIFT_LEFT;
		reportBuffer[1] = KEY_A + key - 'A';
	}
	else {
		switch (key) {
			case '\n':
				reportBuffer[0] = 0;
				reportBuffer[1] = KEY_ENTER;
				break;

			case ' ':
				reportBuffer[0] = 0;
				reportBuffer[1] = KEY_SPACE;
				break;

			case '-':
				reportBuffer[0] = 0;
				reportBuffer[1] = KEY_MINUS;
				break;
			case '_':
				reportBuffer[0] = MOD_SHIFT_LEFT;
				reportBuffer[1] = KEY_MINUS;
				break;

			case '=':
				reportBuffer[0] = 0;
				reportBuffer[1] = KEY_EQUAL;
				break;
			case '+':
				reportBuffer[0] = MOD_SHIFT_LEFT;
				reportBuffer[1] = KEY_EQUAL;
				break;

			case ';':
				reportBuffer[0] = 0;
				reportBuffer[1] = KEY_SEMICOLON;
				break;
			case ':':
				reportBuffer[0] = MOD_SHIFT_LEFT;
				reportBuffer[1] = KEY_SEMICOLON;
				break;

			case ',':
				reportBuffer[0] = 0;
				reportBuffer[1] = KEY_COMMA;
				break;
			case '<':
				reportBuffer[0] = MOD_SHIFT_LEFT;
				reportBuffer[1] = KEY_COMMA;
				break;

			case '.':
				reportBuffer[0] = 0;
				reportBuffer[1] = KEY_PERIOD;
				break;
			case '>':
				reportBuffer[0] = MOD_SHIFT_LEFT;
				reportBuffer[1] = KEY_PERIOD;
				break;

			default:
				reportBuffer[0] = 0;
				reportBuffer[1] = 0;
		}
	}
}

uchar	usbFunctionSetup(uchar data[8])
{
usbRequest_t    *rq = (void *)data;

    usbMsgPtr = reportBuffer;
    if((rq->bmRequestType & USBRQ_TYPE_MASK) == USBRQ_TYPE_CLASS){    /* class request type */
        if(rq->bRequest == USBRQ_HID_GET_REPORT){  /* wValue: ReportType (highbyte), ReportID (lowbyte) */
            /* we only have one report type, so don't look at wValue */
            buildReport(keyPressed());
			// Achei que isto fosse necessário, mas não é
            // usbMsgPtr = reportBuffer;
            return sizeof(reportBuffer);
        }else if(rq->bRequest == USBRQ_HID_GET_IDLE){
            usbMsgPtr = &idleRate;
            return 1;
        }else if(rq->bRequest == USBRQ_HID_SET_IDLE){
			// Set/Get IDLE defines how long the device should keep "quiet" if
			// the state has not changed.
			// Recommended default value for keyboard is 500ms, and infinity
			// for joystick and mice.
			// See pages 52 and 53 from HID1_11.pdf
            idleRate = rq->wValue.bytes[1];
        }
    }else{
        /* no vendor specific requests implemented */
    }
	return 0;
}

/* ------------------------------------------------------------------------- */

int	main(void)
{
uchar   key, lastKey = 0, keyDidChange = 0;
uchar   idleCounter = 0;

	wdt_enable(WDTO_2S);
    hardwareInit();
	odDebugInit();
	usbInit();
	sei();
    DBG1(0x00, 0, 0);
	for(;;){	/* main event loop */
		wdt_reset();
		usbPoll();
        key = keyPressed();
        if(lastKey != key){
            lastKey = key;
            keyDidChange = 1;
        }
        if(TIFR & (1<<TOV0)){   /* 22 ms timer */
            TIFR = 1<<TOV0;
            if(idleRate != 0){
                if(idleCounter > 4){
                    idleCounter -= 5;   /* 22 ms in units of 4 ms */
                }else{
                    idleCounter = idleRate;
                    keyDidChange = 1;
                }
            }
        }
        if(keyDidChange && usbInterruptIsReady()){
            keyDidChange = 0;
            /* use last key and not current key status in order to avoid lost
               changes in key status. */
            buildReport(lastKey);
            usbSetInterrupt(reportBuffer, sizeof(reportBuffer));
        }
	}
	return 0;
}

/* ------------------------------------------------------------------------- */
