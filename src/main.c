/*
	kuroBox / naniBox
	Copyright (c) 2013
	david morris-oliveros // naniBox.com

    This file is part of kuroBox / naniBox.

    kuroBox / naniBox is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 3 of the License, or
    (at your option) any later version.

    kuroBox / naniBox is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.

*/

#include <string.h>
#include <ch.h>
#include <hal.h>
#include <chprintf.h>
#include <memstreams.h>
#include <chrtclib.h>
#include <shell.h>
#include <stdlib.h>
#include <time.h>
#include <math.h>
#include "spiEEPROM.h"
#include "ST7565.h"
#include "kb_buttons.h"
#include "kb_gpio.h"

//-----------------------------------------------------------------------------
#define FACTORY_CONFIG_MAGIC				0x426b426e	// nBkB ;)
#define FACTORY_CONFIG_VERSION				0x01010101
typedef struct factory_config_t factory_config_t;
struct __PACKED__ factory_config_t
{
	uint32_t preamble;
	uint32_t version;
	uint16_t checksum;
	uint32_t serial_number;
	uint32_t tcxo_compensation;
	uint32_t rtc_compensation;
	uint8_t __pad[SPIEEPROM_PAGE_SIZE - (4+4+4+2+4+4)];

};
STATIC_ASSERT(sizeof(factory_config_t)==SPIEEPROM_PAGE_SIZE, SPIEEPROM_PAGE_SIZE);
factory_config_t factory_config;

//-----------------------------------------------------------------------------
// forward declarations
static void gptcb(GPTDriver *gptp);
static void gps_timepulse(EXTDriver *extp, expchannel_t channel);

//-----------------------------------------------------------------------------
static SerialConfig serial1_cfg = {
	SERIAL_DEFAULT_BITRATE,
	0,
	USART_CR2_STOP1_BITS | USART_CR2_LINEN,
	0
};

//-----------------------------------------------------------------------------
static SerialConfig serial2_cfg = {
	230400,
	0,
	USART_CR2_STOP1_BITS | USART_CR2_LINEN,
	0
};

//-----------------------------------------------------------------------------
static const spiEepromConfig eeprom_cfg =
{
	{	// SPIConfig struct
		NULL,			// callback
		GPIOE,
		GPIOE_EEPROM_NSS,
		SPI_CR1_BR_2 | SPI_CR1_CPOL | SPI_CR1_CPHA
	}
};

//-----------------------------------------------------------------------------
static const ST7565Config lcd_cfg = 
{
	{	// SPIConfig struct
		NULL,
		GPIOE,
		GPIOE_LCD_NSS,
		SPI_CR1_CPOL | SPI_CR1_CPHA
	},
	{ GPIOE, GPIOE_LCD_A0 },		// A0 pin
	{ GPIOE, GPIOE_LCD_RST }		// RST pin
};

//-----------------------------------------------------------------------------
static const EXTConfig extcfg = {
  {
	{EXT_CH_MODE_BOTH_EDGES | EXT_CH_MODE_AUTOSTART | EXT_MODE_GPIOA, btn_0_exti_cb},	// 0
	{EXT_CH_MODE_BOTH_EDGES | EXT_CH_MODE_AUTOSTART | EXT_MODE_GPIOA, btn_1_exti_cb},	// 1

    {EXT_CH_MODE_DISABLED, NULL},	// 2
	{EXT_CH_MODE_FALLING_EDGE | EXT_CH_MODE_AUTOSTART | EXT_MODE_GPIOC, gps_timepulse},	// 3
    {EXT_CH_MODE_DISABLED, NULL},	// 4
    {EXT_CH_MODE_DISABLED, NULL},	// 5
    {EXT_CH_MODE_DISABLED, NULL},	// 6
    {EXT_CH_MODE_DISABLED, NULL},	// 7
    {EXT_CH_MODE_DISABLED, NULL},	// 8
    {EXT_CH_MODE_DISABLED, NULL},	// 9
    {EXT_CH_MODE_DISABLED, NULL},	// 10
    {EXT_CH_MODE_DISABLED, NULL},	// 11
    {EXT_CH_MODE_DISABLED, NULL},	// 12
    {EXT_CH_MODE_DISABLED, NULL},	// 13
    {EXT_CH_MODE_DISABLED, NULL},	// 14
    {EXT_CH_MODE_DISABLED, NULL},	// 15
    {EXT_CH_MODE_DISABLED, NULL},	// 16
    {EXT_CH_MODE_DISABLED, NULL},	// 17
    {EXT_CH_MODE_DISABLED, NULL},	// 18
    {EXT_CH_MODE_DISABLED, NULL},	// 19
    {EXT_CH_MODE_DISABLED, NULL},	// 20
    {EXT_CH_MODE_DISABLED, NULL},	// 21
    {EXT_CH_MODE_DISABLED, NULL}	// 22
  }
};

//-----------------------------------------------------------------------------
static const GPTConfig gptcfg = {
	84000000,
	gptcb,
	0
};

//-----------------------------------------------------------------------------
// static data
static uint8_t lcd_buffer[ST7565_BUFFER_SIZE];
volatile bool_t pps_changed;
volatile uint32_t pps_diff;
volatile uint32_t pps_time;
volatile uint32_t pps;

static char charbuf[128];
static MemoryStream msb;
#define INIT_CBUF() \
	memset(charbuf,0,sizeof(charbuf));\
	msObjectInit(&msb, (uint8_t*)charbuf, 128, 0);
#define bss ((BaseSequentialStream *)&msb)
#define prnt ((BaseSequentialStream *)&SD2)
#define PASSES 3600*6	// 6 hours
//#define PASSES 3600	// 1 hour
//#define PASSES 10

// we store the times so we can extract meaningful data at the end
uint32_t times[PASSES];


//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------
static void gptcb(GPTDriver *gptp)
{
	(void)gptp;
}

//-----------------------------------------------------------------------------
static void
gps_timepulse(EXTDriver *extp, expchannel_t channel)
{
	(void)extp;
	(void)channel;

	uint32_t b = halGetCounterValue();
	pps_diff = b-pps_time;
	pps_time = b;
	pps_changed = TRUE;
	pps++;
	kbg_toggleLED2();
}

//-----------------------------------------------------------------------------
int
kuroBoxInit(void)
{
	kbg_setLED1(1);
	kbg_setLED2(1);
	kbg_setLED3(1);

	// Serial
	sdStart(&SD1, &serial1_cfg);
	sdStart(&SD2, &serial2_cfg);

	// start the SPI bus, just use the LCD's SPI config since
	// it's shared with the eeprom
	spiStart(&SPID1, &lcd_cfg.spicfg);
	memset(lcd_buffer, 0, sizeof(lcd_buffer));
	st7565Start(&ST7565D1, &lcd_cfg, &SPID1, lcd_buffer);

	// EEPROM
	spiEepromStart(&spiEepromD1, &eeprom_cfg, &SPID1);

	// this turns on Layer 1 power, this turns on the mosfets controlling the
	// VCC rail. After this, we can start GPS, VectorNav and altimeter
	kbg_setL1PowerOn();

	// wait for it to stabilise, poweron devices, and let them start before continuing
	chThdSleepMilliseconds(500);

	// set initial button state.
	kuroBoxButtonsInit();

	gptStart(&GPTD2, &gptcfg);
	gptStartContinuous(&GPTD2, 84000000);

	// indicate we're ready
	chThdSleepMilliseconds(100);
	kbg_setLED1(0);
	kbg_setLED2(0);
	kbg_setLED3(0);

	// all external interrupts, all the system should now be ready for it
	extStart(&EXTD1, &extcfg);

	return KB_OK; 
}

//-----------------------------------------------------------------------------
static void
cmd_print(BaseSequentialStream * chp, int argc, char * argv[])
{
	(void)argc;
	(void)argv;

	chprintf(chp, "=====  %s  =====\n", BOARD_NAME);
	chprintf(chp, "Config:\n");
	chprintf(chp, "  Preamble  : 0x%.8X\n", factory_config.preamble);
	chprintf(chp, "  Version   : 0x%.8X\n", factory_config.version);
	chprintf(chp, "  SerialNum : %d\n", 	factory_config.serial_number);
	chprintf(chp, "  Checksum  : 0x%.4X\n", factory_config.checksum);
	chprintf(chp, "  TCXO      : %d\n",     factory_config.tcxo_compensation);
	chprintf(chp, "  RTC       : %d\n",     factory_config.rtc_compensation);
	chprintf(chp, "\n");
}

//-----------------------------------------------------------------------------
static void
cmd_read(BaseSequentialStream  *chp, int argc, char * argv[])
{
	(void)argc;
	(void)argv;

	uint8_t * eeprombuf = (uint8_t*) &factory_config;
	while( spiEepromWIP(&spiEepromD1) )
	{}
	spiEepromReadPage(&spiEepromD1, 0, eeprombuf);

	chprintf(chp, "Config read\n");
}

//-----------------------------------------------------------------------------
static void
cmd_write(BaseSequentialStream * chp, int argc, char * argv[])
{
	(void)argc;
	(void)argv;

	factory_config.preamble = FACTORY_CONFIG_MAGIC;
	factory_config.version = FACTORY_CONFIG_VERSION;
	factory_config.checksum = calc_checksum_16((uint8_t*)&factory_config, sizeof(factory_config));

	uint8_t * eeprombuf = (uint8_t*) &factory_config;
	spiEepromEnableWrite(&spiEepromD1);
	spiEepromWritePage(&spiEepromD1, 0, eeprombuf);

	volatile uint32_t count = 0;
	while( spiEepromWIP(&spiEepromD1) ) {count++;}
	spiEepromDisableWrite(&spiEepromD1);

	chprintf(chp, "Config written\n");
}

//-----------------------------------------------------------------------------
static void
cmd_serialnum(BaseSequentialStream * chp, int argc, char * argv[])
{
	if ( argc == 0 )
		return;

	factory_config.serial_number = atoi(argv[0]);
	chprintf(chp, "Setting serial number to: %d\n", factory_config.serial_number);
}

//-----------------------------------------------------------------------------
static void
cmd_rtc(BaseSequentialStream * chp, int argc, char * argv[])
{
	(void)chp;
	(void)argc;
	(void)argv;
}

//-----------------------------------------------------------------------------
static void
cmd_tcxo(BaseSequentialStream * chp, int argc, char * argv[])
{
	uint32_t passes = 0;
	if ( argc == 0 )
	{
		passes = PASSES;
	}
	else
	{
		passes = atoi(argv[0]);
		if ( passes > PASSES )
		{
			chprintf(chp, "WARNING: you asked for too many passes (%d), doing just (%d)\n",
					passes, PASSES);
			passes = PASSES;
		}
	}

	// wait X seconds to make sure the GPS has started up...
	pps = 0;
	while (pps < 5)
	{
		st7565_clear(&ST7565D1);
		st7565_drawstring(&ST7565D1, 0, 0, "TCXO vs GPS TEST");
		INIT_CBUF();
		chprintf(bss, "Waiting on GPS: %d", 5-pps);
		st7565_drawstring(&ST7565D1, 0, 1, charbuf);
		st7565_display(&ST7565D1);
		chThdSleepMilliseconds(50);
	}

	kbg_setLED2(0);

	//--------------------------------------------------------------------------
	// doing the XTAL run first

	st7565_clear(&ST7565D1);
	chprintf(chp, "pass,PASSES,pps_diff,diff_from_parity\n");

	for ( uint32_t pass = 0 ; pass < passes ; )
	{
		chThdSleepMilliseconds(50);
		if ( pps_changed )
		{
			kbg_setLED1(1);
			times[pass] = pps_diff;
			// this may be negative!
			int32_t diff_from_parity = 168000000 - pps_diff;
			chprintf(chp, "%6d,%6d,%14d,%14d\n", pass, passes, pps_diff, diff_from_parity);
			pps_changed = FALSE;
			pass++;

			st7565_drawstring(&ST7565D1, 0, 0, "TCXO vs GPS TEST");

			INIT_CBUF();
			chprintf(bss,"Doing %d passes", passes);
			st7565_drawstring(&ST7565D1, 0, 1, charbuf);

			INIT_CBUF();
			chprintf(bss,"Pass: %d", pass);
			st7565_drawstring(&ST7565D1, 0, 2, charbuf);

			INIT_CBUF();
			chprintf(bss,"Time diff: %d", diff_from_parity);
			st7565_drawstring(&ST7565D1, 0, 3, charbuf);

			st7565_display(&ST7565D1);
			st7565_clear(&ST7565D1);

			chThdSleepMilliseconds(10);
			kbg_setLED1(0);
		}
	}

	int32_t max = -2147483647;
	int32_t min = 2147483647;
	int32_t avg = 0;
	for ( uint32_t pass = 0 ; pass < passes ; pass++ )
	{
		int32_t t = (168000000 - times[pass]);
		if ( max < t ) max = t;
		if ( min > t ) min = t;
		avg += 168000000 - times[pass];
	}
	avg /= passes;

	uint32_t std_dev = 0;
	for ( uint32_t pass = 0 ; pass < passes ; pass++ )
	{
		int32_t diff = avg - (168000000 - times[pass]);
		std_dev += diff*diff;
	}
	std_dev = sqrt(std_dev / passes);

	chprintf(chp, "Time differences\n");
	chprintf(chp, "\tAverage = %d (TCXO is %s)\n", avg, avg<0?"slower":"faster");
	chprintf(chp, "\tMin     = %d\n", min);
	chprintf(chp, "\tMax     = %d\n", max);
	chprintf(chp, "\tStdDev  = %d\n", std_dev);

	//--------------------------------------------------------------------------

	st7565_drawstring(&ST7565D1, 0, 0, "FINISHED TCXO vs GPS");

	INIT_CBUF();
	chprintf(bss,"Passes: %d", passes);
	st7565_drawstring(&ST7565D1, 0, 1, charbuf);

	INIT_CBUF();
	chprintf(bss,"Average: %d (%s)", avg, avg<0?"slower":"faster");
	st7565_drawstring(&ST7565D1, 0, 2, charbuf);

	INIT_CBUF();
	chprintf(bss,"-+S: %d, %d, %d", min, max, std_dev);
	st7565_drawstring(&ST7565D1, 0, 3, charbuf);

	st7565_display(&ST7565D1);
	st7565_clear(&ST7565D1);

	factory_config.tcxo_compensation = avg;}

//-----------------------------------------------------------------------------
static const ShellCommand commands[] =
{
	{"tcxo", 		cmd_tcxo},
	{"rtc", 		cmd_rtc},
	{"serialnum",	cmd_serialnum},
	{"print", 		cmd_print},
	{"read", 		cmd_read},
	{"write", 		cmd_write},
	{NULL, NULL}
};

//-----------------------------------------------------------------------------
static const ShellConfig shell_cfg =
{
	(BaseSequentialStream *)&SD2,
	commands
};

#define SHELL_WA_SIZE   THD_WA_SIZE(2048)

//-----------------------------------------------------------------------------
int main(void)
{
	halInit();
	chSysInit();
	kuroBoxInit();
	chprintf(prnt, "Starting... [%s] \n", BOARD_NAME);
	kbg_setLCDBacklight(1);

	memset(&factory_config, 0, sizeof(factory_config));

	cmd_read(prnt, 0, NULL);
	cmd_print(prnt, 0, NULL);

	Thread * shelltp = NULL;
	while ( 1 )
	{
		if (!shelltp)
		{
			shelltp = shellCreate(&shell_cfg, SHELL_WA_SIZE, NORMALPRIO);
		}
		else if (chThdTerminated(shelltp))
		{
			chThdRelease(shelltp);
			shelltp = NULL;
		}
	}

	//--------------------------------------------------------------------------
	// endless loop to say we've finished
	while(1)
	{
		kbg_setLED1(0);
		kbg_setLED2(0);
		kbg_setLED3(0);
		chThdSleepMilliseconds(200);
		kbg_setLED1(1);
		kbg_setLED2(1);
		kbg_setLED3(1);
		chThdSleepMilliseconds(200);
	}
}

