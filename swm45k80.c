/* Computer Graphics Dot Primitives for a non-raster display */

/*
 *
 *  E0.01 LED 7*5*2 MATRIX DISPLAY, CTMU touch driver
 *  E0.02 Fix X/Y swap bug
 *  E0.03 XC8 rewrite
 * 
 *  ***		background I/O using timer0/timer2  adc, usart2 TX,RX interrupts
 *  Timer3 counter/buffer used for ATOMIC 16bit reads and writes of touch data
 *  INPUTS		AN0-3 touch input
 *  MATRIX X/Y	X-PORTC 0-7, Y-PORTB 0-7, X-PORTA 6-7 DIGITAL OUTPUTS 25mA source or sink
 *  VCC VAOM-A20571G 2.0" dot-matrix display
 *
 *
 * This application is designed for the
 * pic18F45K80  device with CTMU module.
 *
 */

#include "matrix.X/mcc_generated_files/mcc.h"
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>

//#define ROMS // more than one display object

#define	PDELAY	0xA8

#define GRID_S          8
#define PIXEL_NUM       255	// max number of pixels in display ram
#define OBJ_NUM		64	// max nuber of pixels in one object
#define	ROT_SPEED_UP	1	// The highest speed is 1, 35 for demo speed
#define	ROT_SPEED_P	1	// The highest speed is 1, 35 for demo speed
#define ROTATION	1.0	// degree per step
#define DIAG_BITS	PIXEL_NUM-8

#define FALSE	false
#define TRUE	true
#define	ON	0
#define ALLON	0x00
#define	OFF	1
#define ALLOFF	0xff
#define CMARK	0x1957

//	CTMU section
uint16_t touch_base_calc(uint8_t);
void touch_channel(uint8_t);
uint16_t ctmu_touch(uint8_t, uint8_t);
int16_t ctmu_setup(uint8_t, uint8_t);

#define	TIMERCHARGE_BASE_X10            65400		// 5.5 uA time, large plate ~150us
#define	TIMERCHARGE_BASE_1		64000		// .55 uA time, large plate max sens ~700us
#define	TIMERCHARGE_BASE_2		61543		// .55 uA time, large plate low sens ~1000us
#define	TIMERCHARGE_BASE_3		65000		// .55 uA time, small plate max sens ~200us
#define	TIMERCHARGE_BASE_4		62543		// .55 uA time, small plate low sens ~750us
#define	TIMERDISCHARGE			51000		// discharge and max touch data update period 1.8ms

#define TRIP 64  //Difference between pressed
//and un-pressed switch
#define HYST 16 //amount to change
//from pressed to un-pressed
#define PRESSED 1
#define UNPRESSED 0
#define	CHOP_BITS	1               // remove this many bits to reduce noise from the touch sensor
#define MAX_CHAN	1		//	0..1 ADC channels

/* used to hold 16-bit timer value */
union Timers {
	unsigned int lt;
	char bt[2];
};

typedef struct pixel_t {
	int8_t x, y; // display bit x,y and v for pixel value 0=off
	uint8_t v; // pixel dot value
	int8_t m_link, n_link; // pixel links m_ id for each pixel, n_ pixel group id for object
} volatile pixel_t; // -1 in the m_link and n_link means end of display data

/* store the pixel data in rom then copy it to the ram buffer as needed. */
const struct pixel_t pixel_rom[] = {
	-1, -3, 1, 0, 0,
	0, -2, 0, 1, 0,
	1, -1, 1, 2, 0,
	2, 0, 1, 3, 0,
	-1, 0, 1, 4, 0,
	1, 1, 1, 5, 0,
	0, 2, 1, 6, 0,
	-1, 3, 1, 7, 0,
	-2, 0, 1, 8, 0,
	-2, -2, 1, 9, 9,
	-1, -1, 1, 10, 9,
	1, 1, 1, 11, 9,
	2, 2, 1, 12, 9,
	0, 0, 1, 13, 9,
	2, 0, 1, 14, 14,
	0, 2, 1, 15, 14,
	-2, 0, 1, 16, 14,
	0, -2, 1, 17, 14,
	2, 2, 1, 18, 14,
	-2, -2, 1, 19, 14,
	0, 3, 1, 20, 20,
	0, 2, 1, 21, 20,
	0, 1, 1, 22, 20,
	2, 2, 1, 23, 20,
	3, 0, 1, 24, 20,
	2, -2, 1, 25, 20,
	0, -3, 1, 26, 20,
	-2, -2, 1, 27, 20,
	-3, 0, 1, 28, 20,
	-2, 2, 1, 29, 20,
	0, 0, 0, 30, 20,
	0, 0, 0, -1, -1,
	0, 0, 0, -1, -1
};

/*
 * Display file point mode data for line drawing display
 */

/* default data for ram buffer */
volatile struct pixel_t pixel[PIXEL_NUM] = {
	0, 0, 0, -1, -1
},
pixel_temp = {0};

char prog_name[] = "\r\n Matrix Led \r\n";

volatile uint8_t ctmu_button, list_numd;
volatile uint16_t switchState, xd, yd;

uint8_t PEAK_READS = 1;
volatile uint8_t CTMU_ADC_UPDATED = FALSE, TIME_CHARGE = FALSE, CTMU_WORKING = FALSE, SEND_PACKET = FALSE,
	isr_channel = 0;
volatile uint16_t touch_base[16], charge_time[16]; //storage for reading parameters

void high_handler_tmr0(void);
void high_handler_adc(void);
void low_handler_tmr2(void); // DOT MATRIX updater

void d_scan_on(void);
void d_scan_off(void);
void display_init(void); // setup display data structure.

void pixel_init(void); // init the RAM pixel array with all of the ROM array.
void pixel_set(uint8_t, uint8_t); // pixel, value
void pixel_trans(uint8_t, int8_t, int8_t); // pixel,x,y
void pixel_rotate(uint8_t, float); // pixel,degree
void pixel_scale(uint8_t, float, float); // pixel,scale x,y

uint8_t obj_init(uint8_t, uint8_t); // returns the ram object ID of the object copied from the ROM array
void object_set(uint8_t, uint8_t); // object ID, value
void object_trans(uint8_t, int8_t, int8_t); // object ID,x,y
void object_rotate(uint8_t, float); // object ID, degrees
void object_scale(uint8_t, float, float); // object ID,x,y

/* This is a simple scan converter to a random access display */

/* Timer 2 */
void low_handler_tmr2(void)
{
	debug_low0_SetHigh();
	debug_low0_SetLow();
	debug_low0_SetHigh();
	TMR2_WriteTimer(PDELAY);
	LATB = 0xff; // blank the display
	LATC = 0x00;
	while (!pixel[list_numd].v) { // quickly skip pixels that are off
		if ((pixel[list_numd].m_link == -1) || (++list_numd >= PIXEL_NUM)) {
			list_numd = 0;
			break;
		}
	}
	// We move up the display list data array and display a DOT on the matrix display as needed
	if ((pixel[list_numd].x >= 0) && (pixel[list_numd].y >= 0)) { // clip display space to +x and +y
		xd = 1; // load a bit at origin x0
		yd = 1; // load a bit at origin y0
		xd = xd << pixel[list_numd].x; // move the cross bar to the correct location
		yd = yd << pixel[list_numd].y;
		if (pixel[list_numd].v) {
			LATB = (uint8_t) ~yd; // set to low for dot on, load the crossbar into the chip outputs
			LATC = (uint8_t) xd; // set to high for dot on
		} else { // no dot
			LATB = 0xff;
			LATC = 0x00;
		}
	}
	if ((pixel[list_numd].m_link == -1) || (++list_numd >= PIXEL_NUM)) {
		list_numd = 0; // start over again from next line
	}
	debug_low0_SetLow();
}

void high_handler_tmr0(void)
{
	if (!CTMUCONHbits.IDISSEN) { // charge cycle timer0 int, because not shorting the CTMU voltage.
		LATEbits.LATE0 = 1; // flash external led
		CTMUCONLbits.EDG1STAT = 0; // Stop charging touch circuit
		TIME_CHARGE = FALSE; // clear charging flag
		CTMU_WORKING = TRUE; // set working flag, doing touch ADC conversion
		LATEbits.LATE1 = 1; // flash external led
		// configure ADC for next reading
		ADCON0bits.CHS = isr_channel; // Select ADC
		ADCON0bits.ADON = 1; // Turn on ADC
		ADCON0bits.GO = 1; // and begin A/D conv, will set adc int flag when done.
	} else { // discharge cycle timer0 int, because CTMU voltage is shorted
		LATEbits.LATE0 = 0; // flash external led
		CTMUCONHbits.IDISSEN = 0; // end drain of touch circuit
		TIME_CHARGE = TRUE; // set charging flag
		CTMU_WORKING = TRUE; // set working flag, doing
		TMR0_WriteTimer(charge_time[isr_channel]); // set timer to charge rate time
		CTMUCONLbits.EDG1STAT = 1; // Begin charging the touch circuit
	}
}

void high_handler_adc(void)
{
	union Timers timer;

	LATEbits.LATE1 = 0; // flash external led
	timer.lt = ADRES;
	timer.lt = timer.lt >> CHOP_BITS; // toss lower bit noise
	if ((timer.lt) < (touch_base[isr_channel] - TRIP)) { // see if we have a pressed button
		if (isr_channel == 0) {
			switchState = PRESSED;
		}
		if (isr_channel == 1) {
			switchState = UNPRESSED;
		}
		LATEbits.LATE2 = 1; // flash external led
	} else if ((timer.lt) > (touch_base[isr_channel] - TRIP + HYST)) {
		LATEbits.LATE2 = 0; // flash external led
	}
	TMR3H = timer.bt[1];
	TMR3L = timer.bt[0]; // copy low byte and write to timer counter
	CTMU_ADC_UPDATED = TRUE; // New data is in timer3 counter, set to FALSE in main program flow
	CTMU_WORKING = FALSE; // clear working flag, ok to read timer3 counter.
	// config CTMU for next reading
	CTMUCONHbits.CTMUEN = 1; // Enable the CTMU
	CTMUCONLbits.EDG1STAT = 0; // Set Edge status bits to zero
	CTMUCONLbits.EDG2STAT = 0;
	CTMUCONHbits.IDISSEN = 1; // drain charge on the circuit
	TMR0_WriteTimer(TIMERDISCHARGE);
}

uint16_t touch_base_calc(uint8_t channel)
{
	static uint32_t t_avg = 0;
	static int16_t i;

	t_avg = 0;
	touch_channel(channel);
	CTMU_ADC_UPDATED = FALSE;
	while (!CTMU_ADC_UPDATED) ClrWdt(); // wait for touch update cycle
	for (i = 0; i < 8; i++) {
		CTMU_ADC_UPDATED = FALSE;
		while (!CTMU_ADC_UPDATED) ClrWdt(); // wait for touch update cycle
		t_avg += (uint32_t) ctmu_touch(channel, FALSE);
	}
	touch_base[channel] = (uint16_t) (t_avg / 8L);
	if (touch_base[channel] < TRIP) touch_base[channel] = TRIP + HYST;
	return touch_base[channel];
}

void touch_channel(uint8_t channel)
{
	if (channel > MAX_CHAN) return;
	while (CTMU_WORKING) ClrWdt(); // wait for CTMU idle
	INTCONbits.GIEH = 0;
	isr_channel = channel;
	CTMU_ADC_UPDATED = FALSE;
	INTCONbits.GIEH = 1;
	while (!CTMU_ADC_UPDATED) ClrWdt(); // wait for touch update cycle
}

int16_t ctmu_setup(uint8_t current, uint8_t channel)
{
	//CTMUCONH/1 - CTMU Control registers
	CTMUCONH = 0x00; //make sure CTMU is disabled
	CTMUCONL = 0x90;
	//CTMU continues to run when emulator is stopped,CTMU continues
	//to run in idle mode,Time Generation mode disabled, Edges are blocked
	//No edge sequence order, Analog current source not grounded, trigger
	//output disabled, Edge2 polarity = positive level, Edge2 source =
	//source 0, Edge1 polarity = positive level, Edge1 source = source 0,
	//CTMUICON - CTMU Current Control Register
	CTMUICON = 0x01; //.55uA, Nominal - No Adjustment default

	switch (current) {
	case 11:
		charge_time[channel] = TIMERCHARGE_BASE_1;
		break;
	case 12:
		charge_time[channel] = TIMERCHARGE_BASE_2;
		break;
	case 13:
		charge_time[channel] = TIMERCHARGE_BASE_3;
		break;
	case 14:
		charge_time[channel] = TIMERCHARGE_BASE_4;
		break;
	default:
		charge_time[channel] = TIMERCHARGE_BASE_X10; // faster
		CTMUICON = 0b01111101; //.55uA, Adjustment

		break;
	}
	if (current == 0x02) {
		CTMUICON = 0x02; //5.5uA, Nominal - No Adjustment
		charge_time[channel] = TIMERCHARGE_BASE_X10; // faster
	}
	/**************************************************************************/
	//Set up AD converter;
	/**************************************************************************/

	// Use AN0 as an analog channel

	// timer3 register used for atomic data transfer
	T3CONbits.TMR3ON = 0; // Timer is off
	T3CONbits.RD16 = 1; // enable 16 bit reads/writes
	TMR3H = 0;
	TMR3L = 0;
	return 0;
}

uint16_t ctmu_touch(uint8_t channel, uint8_t NULL0)
{
	static uint16_t ctmu_change = 0, last = 0, null = 0;
	static union Timers timer;

	if (CTMU_ADC_UPDATED) {
		timer.bt[0] = TMR3L; // read low byte and read 16bits from timer counter into TMR3 16bit buffer
		timer.bt[1] = TMR3H; // read high byte
		timer.lt = timer.lt & 0x003f;

		if (NULL0 == FALSE) {
			return(timer.lt);
		}
		if (timer.lt < touch_base[channel]) {
			ctmu_change = touch_base[channel] - timer.lt; // read diff
			ctmu_change = ctmu_change & 0x001f;
		}

		if ((null == 0) && NULL0) null = ctmu_change;
		last = ctmu_change;
		return(uint16_t) ctmu_change;
	} else {
		return(uint16_t) last;
	}
}

/* display memory */
void display_init(void)
{
	// what's needed to display data
	list_numd = 0;
}

/* copy the entire ROM to RAM display memory */
void pixel_init(void)
{
	memcpy((void *) pixel, (const void *) pixel_rom, sizeof(pixel_rom));
}

//FIXME we have a ram index bug here

/* move the pixel object from the ROM array to display RAM memory link point, if clear is TRUE reset RAM index back to zero */

/* returns a 8-bit object ID */
uint8_t obj_init(uint8_t rom_link, uint8_t clear)
{
	size_t pixel_size;
	uint8_t ram_link_start = 0;
	static uint8_t ram_link = 0;

	if (clear) {
		ram_link = 0;
		pixel[ram_link].m_link = -1;
		pixel[ram_link].n_link = -1;
		return 0;
	}

	pixel_size = sizeof(pixel_t); // size in bytes of one pixel data structure
	do {
		memcpy((void *) &pixel[ram_link + ram_link_start].x, (const void *) &pixel_rom[rom_link + ram_link_start].x, pixel_size);
		pixel[ram_link + ram_link_start].m_link = (int8_t) (ram_link + ram_link_start); // make a RAM ID for each pixel
		pixel[ram_link + ram_link_start].n_link = (int8_t) ram_link; // link RAM ID to object
		++ram_link_start;
	} while (pixel_rom[ram_link_start + rom_link].n_link == rom_link);

	ram_link += ram_link_start;
	pixel[ram_link].m_link = -1;
	pixel[ram_link].n_link = -1;
	return ram_link - ram_link_start;
}

void pixel_set(uint8_t list_num, uint8_t value)
{
	if (list_num >= PIXEL_NUM) return;
	pixel[list_num].v = value;
}

void pixel_rotate(uint8_t list_num, float degree) // pixel,degree rotation
{
	// remember old rotations
	static float to_rad, float_x, float_y, sine, cosine, old_degree = 1957.7;

	if (degree != old_degree) {
		to_rad = (float) 0.0175 * degree;
		cosine = (float) cos(to_rad);
		sine = (float) sin(to_rad);
		old_degree = degree;
	}

	float_x = (float) pixel[list_num].x;
	float_y = (float) pixel[list_num].y;

	pixel[list_num].x = (int8_t) (float_x * cosine - float_y * sine);
	pixel[list_num].y = (int8_t) (float_x * sine + float_y * cosine);

}

void pixel_trans(uint8_t list_num, int8_t x_new, int8_t y_new)
{
	pixel[list_num].x += x_new;
	pixel[list_num].y += y_new;
}

void pixel_scale(uint8_t list_num, float x_scale, float y_scale)
{
	float float_x, float_y;

	float_x = (float) pixel[list_num].x;
	float_y = (float) pixel[list_num].y;
	pixel[list_num].x = (int8_t) (float_x * x_scale);
	pixel[list_num].y = (int8_t) (float_y * y_scale);
}

void object_rotate(uint8_t list_num, float degree)
{
	uint8_t i;

	if (list_num >= PIXEL_NUM) return; // check for valid range

	for (i = 0; i < OBJ_NUM; i++) {
		if (pixel[list_num + i].n_link != list_num) return; // invalid current object id
		pixel_rotate(list_num + i, degree);
	}
}

void object_trans(uint8_t list_num, int8_t x_new, int8_t y_new)
{
	uint8_t i;

	if (list_num >= PIXEL_NUM) return; // check for valid range

	for (i = 0; i < OBJ_NUM; i++) {
		if (pixel[list_num + i].n_link != list_num) return; // invalid current object id
		pixel_trans(list_num + i, x_new, y_new);
	}
}

void object_scale(uint8_t list_num, float x_scale, float y_scale)
{
	uint8_t i;

	if (list_num >= PIXEL_NUM) return; // check for valid range

	for (i = 0; i < OBJ_NUM; i++) {
		if (pixel[list_num + i].n_link != list_num) return; // invalid current object id
		pixel_scale(list_num + i, x_scale, y_scale);
	}
}

void object_set(uint8_t list_num, uint8_t value)
{
	uint8_t i;

	if (list_num >= PIXEL_NUM) return; // check for valid range

	for (i = 0; i < OBJ_NUM; i++) {
		if (pixel[list_num + i].n_link != list_num) return; // invalid current object id
		pixel_set(list_num + i, value);
	}
}

void d_scan_on(void)
{
	list_numd = 0;
	LATDbits.LATD1 = !LATDbits.LATD1;
	INTCONbits.GIEL = 1; // restart display scanner
}

void d_scan_off(void)
{
	LATDbits.LATD1 = !LATDbits.LATD1;
	INTCONbits.GIEL = 0; // suspend list processing during matrix operations
	LATB = 0xff;
	LATC = 0x00;
}

void main_init(void)
{
	uint16_t touch_zero = 0;
	uint8_t x = 1, y = 1, t, i, romid = 9;
	uint32_t move = 0, times = ROT_SPEED_UP;
	uint8_t obj1;
	int8_t x_p = 0, y_p = 0, x_o = 1, y_o = 1;
	float rotation = 0.0, scaling = 3.0;

	display_init(); // Setup the pixel display data MUST BE CALLED FIRST
	switchState = UNPRESSED;
	LATA = 0b00000000;
	LATB = 0xff;
	LATC = 0xff;
	LATD = 0x00;
	LATE = 0x00;

	SLRCON = 0x00; // set slew rate to max

	TMR0_WriteTimer(TIMERDISCHARGE);
	TMR2_WriteTimer(PDELAY);
	TMR2_SetInterruptHandler(low_handler_tmr2);
	TMR0_SetInterruptHandler(high_handler_tmr0);
	ADC_SetInterruptHandler(high_handler_adc);

	/* Enable interrupt priority */
	RCONbits.IPEN = 1;
	/* Enable all high priority interrupts */
	INTCONbits.GIEH = 1;
	/* Enable all low priority interrupts */
	INTCONbits.GIEL = 1;

	//		CTMU setups
	ctmu_button = 0; // select start touch input
	ctmu_setup(13, 0); // config the CTMU for touch response
	ctmu_setup(13, 1);
	ctmu_setup(13, 2);
	ctmu_setup(13, 3);
	touch_zero = touch_base_calc(0);
	touch_zero = touch_base_calc(1);
	touch_zero = touch_base_calc(2);
	touch_zero = touch_base_calc(3);

	printf("%s", prog_name);

	/* Loop forever */

	while (TRUE) {
		for (ctmu_button = 0; ctmu_button <= MAX_CHAN; ctmu_button++) {
			touch_channel(ctmu_button);
			if (ctmu_button == 0) {
				t = (uint8_t) ctmu_touch(ctmu_button, FALSE); // display channel  0 only
			} else {
				ctmu_touch(ctmu_button, FALSE);
			}
			/* CTMU testing
					       // set the lower 8 bits into the pixel array for display
					       for (i = 0; i < 8; i++) {
						       if (((t >> i)& 0x01) == 0) {
							       pixel_set(i, 0);
						       } else {
							       pixel_set(i, 1);
						       }
					       }
			 */
			ClrWdt(); // reset the WDT timer

			/* transformation testing */
			if (++move >= times) {
				d_scan_off(); // suspend list processing during matrix operations
				if (false && switchState == UNPRESSED) {
					x_o = -x_o;
					times = ROT_SPEED_UP;
					obj_init(0, TRUE); // clear ram display memory
					obj1 = obj_init(romid, FALSE); // return ID for rom object into ram id
					//object_scale(obj1, scaling, scaling); // big to small
					object_rotate(obj1, rotation); // CW
					//object_trans(obj1, x_p, y_p); // move to near center
					object_trans(obj1, 4, 3); // move to near center
				} else {
					times = ROT_SPEED_P;
					obj_init(0, TRUE); // clear ram display memory
					obj1 = obj_init(romid, FALSE); // return ID for rom object into ram id
					//object_scale(obj1, (float) 2.0 - scaling, (float) 2.0 - scaling); // small to big
					object_rotate(obj1, (float) 360.0 - rotation); // CCW
					//object_trans(obj1, x_p, y_p);
					object_trans(obj1, 4, 3); // move to near center
				}

				d_scan_on();
				rotation += ROTATION;
				if (rotation >= 360.00) { // spin and grow or shrink
					rotation = 0.0;
					scaling -= 0.2;
					if (scaling < -0.01) {
						scaling = 3.0;
						if (x_p > 8) x_o = -1;
						if (x_p < -1) x_o = 1;
						x_p += x_o;

						if (y_p > 8) y_o = -1;
						if (y_p < -1) y_o = 1;
						y_p += y_o;

#ifdef ROMS
						switch (romid) {
						case 0:
							romid = 9;
							break;
						case 9:
							romid = 14;
							break;
						case 14:
							romid = 20;
							break;
						case 20:
							romid = 0;
							break;
						default:
							romid = 0;
							break;
						}
#endif
					}
				}
				move = 0;
			}
		}
	}
}
