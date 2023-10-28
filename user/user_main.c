//Copyright 2015 <>< Charles Lohr, see LICENSE file.

#include "mem.h"
#include "c_types.h"
#include "user_interface.h"
#include "ets_sys.h"
#include "uart.h"
#include "osapi.h"
#include "espconn.h"
#include "commonservices.h"
#include <mdns.h>
#include <esp82xxutil.h>
#include <gpio.h>
#include <common.h>
#include <usb.h>

#define PORT 7777

#define procTaskPrio        0
#define procTaskQueueLen    1

static volatile os_timer_t some_timer;


//int ICACHE_FLASH_ATTR StartMDNS();

void user_rf_pre_init(void)
{
	//nothing.
}

char * strcat( char * dest, char * src )
{
	return strcat(dest, src );
}

//Tasks that happen all the time.

os_event_t    procTaskQueue[procTaskQueueLen];


//Awkward example with use of control messages to get data to/from device.
uint8_t user_control[144]; //Enough for FW######## ### [128 bytes of data] [null]
int     user_control_length_acc; //From host to us.
int     user_control_length_ret; //From us to host.


void usb_handle_custom_control( int bmRequestType, int bRequest, int wLength, struct usb_internal_state_struct * ist )
{
	struct usb_endpoint * e = ist->ce;

	if( bmRequestType == 0x80 )
	{
		if( bRequest == 0xa7) //US TO HOST "in"
		{
			if( user_control_length_ret )
			{
				e->ptr_in = user_control;
				e->size_in = user_control_length_ret;
				if( wLength < e->size_in ) e->size_in = wLength;
				user_control_length_ret = 0;
			}
		}
	}

	if( bmRequestType == 0x00 )
	{
		if( bRequest == 0xa6 && user_control_length_acc == 0 ) //HOST TO US "out"
		{
			e->ptr_out = user_control;
			e->max_size_out = sizeof( user_control );
			if( e->max_size_out > wLength ) e->max_size_out = wLength;
			e->got_size_out = 0;
			e->transfer_done_ptr = &user_control_length_acc;
		}

	}

}

uint8_t my_ep1[4];
uint8_t my_ep2[8];
extern uint32_t usb_ramtable[31];

extern int keybt;
extern int keymod;
extern int keypress;


uint8_t masks = 0;
bool rel = false;

uint8_t codes[5][4] = {{0x60, 0x5a, 0x5c, 0x5e}, {0x1a, 0x16, 0x04, 0x07}, {0x52, 0x51, 0x50, 0x4f}, {0x0c, 0x0e, 0x0d, 0x0f}, {0x17, 0x0a, 0x09, 0x0b}};

uint8_t profile = 0;

static void ICACHE_FLASH_ATTR procTask(os_event_t *events)
{
	struct usb_internal_state_struct * uis = &usb_internal_state;
	struct usb_endpoint * e2 = &uis->eps[2];
	if(!GPIO_INPUT_GET( GPIO_ID_PIN(12)))
		masks |= (1<<0);
	if(!GPIO_INPUT_GET( GPIO_ID_PIN(13)))
		masks |= (1<<1);
	if(!GPIO_INPUT_GET( GPIO_ID_PIN(14)))
		masks |= (1<<2);
	if(!GPIO_INPUT_GET( GPIO_ID_PIN(3)))
		masks |= (1<<3);
	e2->ptr_in = my_ep2;
	e2->place_in = 0;
	e2->size_in = sizeof( my_ep2 );
	if( (masks || !rel) && e2->send == 0 )
	{
		my_ep2[0] = 0;
		my_ep2[2] = masks & (1<<0)?codes[profile][0]:0;
		my_ep2[3] = masks & (1<<1)?codes[profile][1]:0;
		my_ep2[4] = masks & (1<<2)?codes[profile][2]:0;
		my_ep2[5] = masks & (1<<3)?codes[profile][3]:0;
		e2->send = 1;
		rel = !masks;
		masks = 0;
		
	}
	if( user_control_length_acc )
	{
		//printf( "\nGot: %s\n", usb_internal_state.user_control );
		int r = issue_command( user_control, sizeof( user_control )-1, user_control, user_control_length_acc );
		user_control_length_acc = 0;
		//printf( "%d/%s/%d\n", usb_internal_state.user_control_length_acc, usb_internal_state.user_control, r );
		if( r >= 0 )
			user_control_length_ret = r;
	}

	system_os_post(procTaskPrio, 0, 0 );
}


//Timer event.
static void ICACHE_FLASH_ATTR myTimer(void *arg)
{
		if(!GPIO_INPUT_GET( GPIO_ID_PIN(12)))
			profile = 1;
		else if(!GPIO_INPUT_GET( GPIO_ID_PIN(13)))
			profile = 2;
		else if(!GPIO_INPUT_GET( GPIO_ID_PIN(14)))
			profile = 3;
		else if(!GPIO_INPUT_GET( GPIO_ID_PIN(3)))
			profile = 4;
}


void ICACHE_FLASH_ATTR charrx( uint8_t c )
{
	//Called from UART.
}


volatile uint32_t my_table[] = { 0, (uint32_t)&PIN_IN, (uint32_t)&PIN_OUT_SET, (uint32_t)&PIN_OUT_CLEAR, 0xffff0000, 0x0000ffff };



#ifdef PROFILE
int time_ccount(void) 
{
        unsigned r;

/*	volatile unsigned a = 0xabcdef01;
        asm volatile ("testp:");
	a &= ~(1<<10);
*/

        asm volatile ("\
	\n\
\
intrs: \
	call0 my_func\n\
	j end\n\
\n\
end:\n\
\
	\n\
	sub %[out], a11, a9\n\
	" : [out] "=r"(r) : : "a9", "a10", "a11" );

        return r; //rsr a9, ccount //rsr a11, ccount
//	addi %[out], %[out], -1\n\
}
#endif

void user_rf_cal_sector_set()
{
}

void ICACHE_FLASH_ATTR user_init(void)
{
	uart_init(BIT_RATE_115200, BIT_RATE_115200);

	uart0_sendStr("\r\n\033c" ); //Clear screen
	uart0_sendStr("esp8266 test usb driver\r\n");
	system_update_cpu_freq( 80 );
//#define PROFILE
#ifdef PROFILE
	uint32_t k = 0x89abcdef;
	uint8_t * g  = (uint8_t*)&k;
 system_update_cpu_freq(160);
	my_table[0] = 5;
	printf( "%02x %02x %02x %02x\n", g[0], g[1], g[2], g[3] );
	uint32_t rr = time_ccount();
	printf( ":::::%d / %02x / %d\n", rr, rr, my_table[0] );
	system_restart();
	while(1);
#endif

	//Print reboot cause
	
	struct rst_info * r = system_get_rst_info();
	printf( "Reason: %p\n", r->reason );
	printf( "Exec  : %p\n", r->exccause );
	printf( "epc1  : %p\n", r->epc1 );
	printf( "epc2  : %p\n", r->epc2 );
	printf( "epc3  : %p\n", r->epc3 );
	printf( "excvaddr:%p\n", r->excvaddr );
	printf( "depc: %p\n", r->depc );



//Uncomment this to force a system restore.
//	system_restore();

	//Set GPIO16 for INput
	WRITE_PERI_REG(PAD_XPD_DCDC_CONF,
		(READ_PERI_REG(PAD_XPD_DCDC_CONF) & 0xffffffbc) | (uint32)0x1);     // mux configuration for XPD_DCDC and rtc_gpio0 connection

	WRITE_PERI_REG(RTC_GPIO_CONF,
		(READ_PERI_REG(RTC_GPIO_CONF) & (uint32)0xfffffffe) | (uint32)0x0); //mux configuration for out enable

	WRITE_PERI_REG(RTC_GPIO_ENABLE,
		READ_PERI_REG(RTC_GPIO_ENABLE) & (uint32)0xfffffffe);       //out disable

	PIN_FUNC_SELECT(PERIPHS_IO_MUX_MTDI_U, FUNC_GPIO12);
	PIN_FUNC_SELECT(PERIPHS_IO_MUX_MTCK_U, FUNC_GPIO13);
	PIN_FUNC_SELECT(PERIPHS_IO_MUX_MTMS_U, FUNC_GPIO14);
	PIN_FUNC_SELECT(PERIPHS_IO_MUX_U0RXD_U, FUNC_GPIO3);

	GPIO_DIS_OUTPUT(GPIO_ID_PIN(12));
	GPIO_DIS_OUTPUT(GPIO_ID_PIN(13));
	GPIO_DIS_OUTPUT(GPIO_ID_PIN(14));
	GPIO_DIS_OUTPUT(GPIO_ID_PIN(3));

	SET_PERI_REG_MASK(PERIPHS_IO_MUX_MTDI_U, PERIPHS_IO_MUX_PULLUP);
	SET_PERI_REG_MASK(PERIPHS_IO_MUX_MTCK_U, PERIPHS_IO_MUX_PULLUP);
	SET_PERI_REG_MASK(PERIPHS_IO_MUX_MTMS_U, PERIPHS_IO_MUX_PULLUP);
	SET_PERI_REG_MASK(PERIPHS_IO_MUX_U0RXD_U, PERIPHS_IO_MUX_PULLUP);
	

	//Add a process
	system_os_task(procTask, procTaskPrio, procTaskQueue, procTaskQueueLen);

	//Timer example
	os_timer_disarm(&some_timer);
	os_timer_setfn(&some_timer, (os_timer_func_t *)myTimer, NULL);
	os_timer_arm(&some_timer, 100, 0);

	printf( "Boot Ok.\n" );

	usb_init();
	
	
	wifi_set_opmode(NULL_MODE);
	//wifi_init();
	//wifi_set_sleep_type(LIGHT_SLEEP_T);
	//wifi_fpm_set_sleep_type(LIGHT_SLEEP_T);

	system_os_post(procTaskPrio, 0, 0 );
}


//There is no code in this project that will cause reboots if interrupts are disabled.
void EnterCritical()
{
}

void ExitCritical()
{
}


