/* Computer Graphics Dot Primitives for a non-raster display */

// PIC18F45K80 Configuration Bit Settings

#include <p18f45k80.h>

// CONFIG1L
#pragma config RETEN = OFF      // VREG Sleep Enable bit (Ultra low-power regulator is Disabled (Controlled by REGSLP bit))
#pragma config INTOSCSEL = HIGH // LF-INTOSC Low-power Enable bit (LF-INTOSC in High-power mode during Sleep)
#pragma config SOSCSEL = DIG    // SOSC Power Selection and mode Configuration bits (Digital (SCLKI) mode)
#pragma config XINST = OFF      // Extended Instruction Set (Disabled)

// CONFIG1H
#pragma config FOSC = INTIO2    // Oscillator (Internal RC oscillator)
#pragma config PLLCFG = ON      // PLL x4 Enable bit (Enabled)
#pragma config FCMEN = OFF      // Fail-Safe Clock Monitor (Disabled)
#pragma config IESO = OFF       // Internal External Oscillator Switch Over Mode (Disabled)

// CONFIG2L
#pragma config PWRTEN = OFF     // Power Up Timer (Disabled)
#pragma config BOREN = SBORDIS  // Brown Out Detect (Enabled in hardware, SBOREN disabled)
#pragma config BORV = 3         // Brown-out Reset Voltage bits (1.8V)
#pragma config BORPWR = ZPBORMV // BORMV Power level (ZPBORMV instead of BORMV is selected)

// CONFIG2H
#pragma config WDTEN = OFF      // Watchdog Timer (WDT disabled in hardware; SWDTEN bit disabled)
#pragma config WDTPS = 1024     // Watchdog Postscaler (1:1024)

// CONFIG3H
#pragma config CANMX = PORTC    // ECAN Mux bit (ECAN TX and RX pins are located on RC6 and RC7, respectively)
#pragma config MSSPMSK = MSK7   // MSSP address masking (7 Bit address masking mode)
#pragma config MCLRE = OFF      // Master Clear Enable (MCLR Disabled, RG5 Enabled)

// CONFIG4L
#pragma config STVREN = ON      // Stack Overflow Reset (Enabled)
#pragma config BBSIZ = BB2K     // Boot Block Size (2K word Boot Block size)

// CONFIG5L
#pragma config CP0 = OFF        // Code Protect 00800-01FFF (Disabled)
#pragma config CP1 = OFF        // Code Protect 02000-03FFF (Disabled)
#pragma config CP2 = OFF        // Code Protect 04000-05FFF (Disabled)
#pragma config CP3 = OFF        // Code Protect 06000-07FFF (Disabled)

// CONFIG5H
#pragma config CPB = OFF        // Code Protect Boot (Disabled)
#pragma config CPD = OFF        // Data EE Read Protect (Disabled)

// CONFIG6L
#pragma config WRT0 = OFF       // Table Write Protect 00800-03FFF (Disabled)
#pragma config WRT1 = OFF       // Table Write Protect 04000-07FFF (Disabled)
#pragma config WRT2 = OFF       // Table Write Protect 08000-0BFFF (Disabled)
#pragma config WRT3 = OFF       // Table Write Protect 0C000-0FFFF (Disabled)

// CONFIG6H
#pragma config WRTC = OFF       // Config. Write Protect (Disabled)
#pragma config WRTB = OFF       // Table Write Protect Boot (Disabled)
#pragma config WRTD = OFF       // Data EE Write Protect (Disabled)

// CONFIG7L
#pragma config EBTR0 = OFF      // Table Read Protect 00800-03FFF (Disabled)
#pragma config EBTR1 = OFF      // Table Read Protect 04000-07FFF (Disabled)
#pragma config EBTR2 = OFF      // Table Read Protect 08000-0BFFF (Disabled)
#pragma config EBTR3 = OFF      // Table Read Protect 0C000-0FFFF (Disabled)

// CONFIG7H
#pragma config EBTRB = OFF      // Table Read Protect Boot (Disabled)


/*
 *
 *  E0.01 LED 7*5*2 MATRIX DISPLAY, CTMU touch driver
 *  ***		background I/O using timer0/timer2  adc, usart2 interrupts
 *  Timer3 counter/buffer used for ATOMIC 16bit reads and writes of touch data
 *  INPUTS		AN0-3 touch input
 *  MATRIX X/Y	PORTB 0-7, PORTC 0-7, PORTA 6-7 DIGITAL OUTPUTS 25mA source or sink
 *  VCC VAOM-A20571G 2.0" dot-matrix display
 *
 *
 * This application is designed for the
 * pic18F45K80  device with CTMU module.
 *
 */

#include <string.h>
#include <stdlib.h>
#include <EEP.h>
#include <timers.h>
#include <adc.h>
#include <ctmu.h>
#include <usart.h>
#include <math.h>


#ifdef INTTYPES
#include <stdint.h>
#else
#define INTTYPES
// unsigned types
typedef unsigned char uint8_t;
typedef unsigned int uint16_t;
typedef unsigned long uint32_t;
typedef unsigned long long uint64_t;
// signed types
typedef signed char int8_t;
typedef signed int int16_t;
typedef signed long int32_t;
typedef signed long long int64_t;
#endif

#define	PDELAY	0xA8

#define GRID_S          8
#define PIXEL_NUM       128
#define OBJ_NUM		24
#define	ROT_SPEED	35
#define ROTATION	12.0
#define DIAG_BITS	PIXEL_NUM-8

#define FALSE	0
#define TRUE	1
#define	ON		0
#define ALLON	0x00
#define	OFF		1
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

#define TRIP 32  //Difference between pressed
//and un-pressed switch
#define HYST 8 //amount to change
//from pressed to un-pressed
#define PRESSED 1
#define UNPRESSED 0
#define	CHOP_BITS	4               // remove this many bits to reduce noise from the touch sensor
#define MAX_CHAN	3		//	0..3 ADC channels

typedef struct pixel_t {
	int8_t x, y, v; // display bit x,y and v for pixel value 0=off
	int8_t m_link, n_link; // pixel links m_ id for each pixel, n_ pixel group id
} volatile pixel_t; // -1 in the m_link and n_link means end of display data

/* store the pixel data in rom then copy it to the ram buffer as needed. */
const rom struct pixel_t pixel_rom[PIXEL_NUM] = {
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
	0, -3, 1, 13, 13,
	3, 0, 1, 14, 13,
	0, 3, 1, 15, 13,
	-3, 0, 1, 16, 13,
	0, 0, 0, 17, 13,
	3, 3, 1, 18, 13,
	0, 0, 0, 19, 13,
	0, 0, 0, -1, -1,
	0, 0, 0, -1, -1
};

#pragma idata bigdata

/*
 * Display file point mode data for line drawing display
 */

/* default data for ram buffer */
volatile struct pixel_t pixel[PIXEL_NUM] = {
	0, 0, 0, -1, -1
},
pixel_temp = {0};

uint8_t prog_name[] = "nsaspook";
#pragma idata

#pragma	idata

uint8_t ctmu_button, PEAK_READS = 1;
volatile uint8_t CTMU_ADC_UPDATED = FALSE, TIME_CHARGE = FALSE, CTMU_WORKING = FALSE, SEND_PACKET = FALSE,
	isr_channel = 0;
volatile uint16_t touch_base[16], switchState = UNPRESSED, charge_time[16]; //storage for reading parameters

void high_handler(void); //reads the CTMU voltage using a ADC channel, interrupt driven
void low_handler(void); // MATRIX updater

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

#pragma code high_interrupt = 0x8

void high_int(void)
{
	_asm goto high_handler _endasm
}
#pragma code

#pragma code low_interrupt = 0x18

void low_int(void)
{
	_asm goto low_handler _endasm
}
#pragma code

#pragma interrupt low_handler

/* This is a simple scan converter to a random access display */
void low_handler(void)
{
	static uint8_t list_num = 0;
	static uint16_t x, y;

	if (PIR1bits.TMR2IF) {
		PIR1bits.TMR2IF = 0; // clear TMR2 int flag
		WriteTimer2(PDELAY);
		LATB = 0xff; // blank the display
		LATC = 0x00;
		while (!pixel[list_num].v) { // quickly skip pixels that are off
			if ((pixel[list_num].m_link == -1) || (++list_num >= PIXEL_NUM)) {
				list_num = 0;
				break;
			}
		}
		// We move up the display list data array and display a DOT on the matrix display as needed
		if ((pixel[list_num].x >= 0) && (pixel[list_num].y >= 0)) { // clip display space to +x and +y
			x = 1; // load a bit at origin x0
			y = 1; // load a bit at origin y0
			x = x << pixel[list_num].x; // move the cross bar to the correct location
			y = y << pixel[list_num].y;
			if (pixel[list_num].v) {
				LATB = ~x; // set to low for dot on, load the crossbar into the chip outputs
				LATC = y; // set to high for dot on
			} else { // no dot
				LATB = 0xff;
				LATC = 0x00;
			}
		}
		if ((pixel[list_num].m_link == -1) || (++list_num >= PIXEL_NUM)) list_num = 0; // start over again from next line
	}
}

#pragma interrupt high_handler

void high_handler(void)
{
	static union Timers timer;
	static uint8_t i = 0, host_c, *data_ptr = prog_name;
	static int16_t data_pos = 0, data_len = 0;

	/* start with data_ptr pointed to address of data, data_len to length of data in bytes, data_pos to 0 to start at the beginning of data block */
	/* then enable the interrupt and wait for the interrupt enable flag to clear
	/* send buffer and count xmit data bytes for host link */
	if (PIE3bits.TX2IE && PIR3bits.TX2IF) { // send data TX2
		if (data_pos >= data_len) { // buffer has been sent
			if (TXSTA2bits.TRMT) { // last bit has been shifted out
				PIE3bits.TX2IE = 0; // stop data xmit
				SEND_PACKET = FALSE;
			}
		} else {
			TXREG2 = *data_ptr; // send data and clear PIR3bits.TX2IF

			data_pos++; // move the data pointer
			data_ptr++; // move the buffer pointer position
		}
	}

	if (PIR3bits.RC2IF) { // receive data  RX2
		if (RCSTA2bits.OERR) {
			RCSTA2bits.CREN = 0; //	clear overrun
			RCSTA2bits.CREN = 1; // re-enable
		}

		host_c = RCREG2;
		prog_name[0] = host_c;
		data_ptr = prog_name;
		data_pos = 0;
		data_len = 1;
		PIE3bits.TX2IE = 1; // start data xmit
	}

	if (INTCONbits.TMR0IF) { // check timer0 irq
		// clr  TMR0 int flag
		INTCONbits.TMR0IF = 0; //clear interrupt flag
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
			WriteTimer0(charge_time[isr_channel]); // set timer to charge rate time
			CTMUCONLbits.EDG1STAT = 1; // Begin charging the touch circuit
		}
	}
	if (PIR1bits.ADIF) { // check ADC irq
		PIR1bits.ADIF = 0; // clear ADC int flag
		LATEbits.LATE1 = 0; // flash external led
		timer.lt = ADRES;
		timer.lt = timer.lt >> CHOP_BITS; // toss lower bit noise
		if ((timer.lt) < (touch_base[isr_channel] - TRIP)) { // see if we have a pressed button
			if (isr_channel == 0) switchState = PRESSED;
			if (isr_channel == 1) switchState = UNPRESSED;
			LATEbits.LATE2 = 1; // flash external led
			//			pixel[DIAG_BITS + isr_channel].v = 1;
		} else if ((timer.lt) > (touch_base[isr_channel] - TRIP + HYST)) {
			//			switchState = UNPRESSED;
			LATEbits.LATE2 = 0; // flash external led
			//			pixel[DIAG_BITS + isr_channel].v = 0;
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
		WriteTimer0(TIMERDISCHARGE); // set timer to discharge rate
	}
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
		t_avg += ctmu_touch(channel, FALSE);
	}
	touch_base[channel] = (uint16_t) t_avg / 8L;
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

	// Configure AN0 as an analog channel
	ANCON0bits.ANSEL0 = 1;
	TRISAbits.TRISA0 = 1;

	// ADCON2
	ADCON2bits.ADFM = 1; // Results format 1= Right justified
	ADCON2bits.ACQT = 7; // Acquition time 7 = 20TAD 2 = 4TAD 1=2TAD
	ADCON2bits.ADCS = 6; // Clock conversion bits 6= FOSC/64 2=FOSC/32
	// ADCON1
	ADCON1bits.VCFG = 3; // Vref+ = 4.096
	ADCON1bits.VNCFG = 0; // Vref- = AVss
	ADCON1bits.CHSN = 0; // single ended
	// ADCON0
	ADCON0bits.CHS = 0; // Select ADC channel
	ADCON0bits.ADON = 1; // Turn on ADC
	PIE1bits.ADIE = 1; // enable ADC int

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

/* copy the entire ROM to RAM display memory */
void pixel_init(void)
{
	static int16_t i;

	memcpypgm2ram((void *) pixel, (const rom void *) pixel_rom, sizeof(pixel));

}

/* move the pixel object from the ROM array to display RAM memeory, if clear is TRUE reset RAM index back to zero */
uint8_t obj_init(uint8_t rom_link, uint8_t clear)
{
	static int16_t i, pixel_size;
	static uint8_t ram_link = 0, ram_link_start = 0;

	if (clear) {
		ram_link = 0;
		pixel[ram_link].m_link = -1;
		pixel[ram_link].n_link = -1;
		return 0;
	}

	ram_link_start = 0;
	pixel_size = sizeof(pixel_temp);
	do {
		memcpypgm2ram((void *) &pixel[ram_link + ram_link_start].x, (const rom void *) &pixel_rom[rom_link + ram_link_start].x, pixel_size);
		++ram_link_start;
	} while (pixel_rom[ram_link_start + rom_link].n_link == rom_link);

	for (i = ram_link; i < (ram_link + ram_link_start); i++) {
		pixel[i].m_link = ram_link + i; // make a RAM ID for each pixel
		pixel[i].n_link = ram_link; // link RAM ID to object
	}
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
	static float to_rad, float_x, float_y, sine, cosine, old_degree = 1957.7;

	if (degree != old_degree) {
		to_rad = 0.0175 * degree;
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
	static float float_x, float_y;

	float_x = (float) pixel[list_num].x;
	float_y = (float) pixel[list_num].y;
	pixel[list_num].x = (int8_t) float_x * x_scale;
	pixel[list_num].y = (int8_t) float_y * y_scale;
}

void object_rotate(uint8_t list_num, float degree)
{
	static uint8_t i;

	if (list_num >= PIXEL_NUM) return; // check for valid range

	for (i = 0; i < OBJ_NUM; i++) {
		if (pixel[list_num + i].n_link != list_num) return; // invalid current object id
		pixel_rotate(list_num + i, degree);
	}
}

void object_trans(uint8_t list_num, int8_t x_new, int8_t y_new)
{
	static uint8_t i;

	if (list_num >= PIXEL_NUM) return; // check for valid range

	for (i = 0; i < OBJ_NUM; i++) {
		if (pixel[list_num + i].n_link != list_num) return; // invalid current object id
		pixel_trans(list_num + i, x_new, y_new);
	}
}

void object_scale(uint8_t list_num, float x_scale, float y_scale)
{
	static uint8_t i;

	if (list_num >= PIXEL_NUM) return; // check for valid range

	for (i = 0; i < OBJ_NUM; i++) {
		if (pixel[list_num + i].n_link != list_num) return; // invalid current object id
		pixel_scale(list_num + i, x_scale, y_scale);
	}
}

void object_set(uint8_t list_num, uint8_t value)
{
	static uint8_t i;

	if (list_num >= PIXEL_NUM) return; // check for valid range

	for (i = 0; i < OBJ_NUM; i++) {
		if (pixel[list_num + i].n_link != list_num) return; // invalid current object id
		pixel_set(list_num + i, value);
	}
}

void scan_on(void)
{
	INTCONbits.GIEL = 1; // restart display scanner
}

void scan_off(void)
{
	INTCONbits.GIEL = 0; // suspend list processing during matrix operations
	LATB = 0xff;
	LATC = 0x00;
}

void main(void)
{
	uint16_t touch_zero = 0;
	uint8_t x = 1, y = 1, t, i, romid = 9;
	uint32_t move = 0, times = ROT_SPEED;
	uint8_t obj1;
	float rotation = 0.0, scaling = 2.0;

	pixel_init(); // Setup the pixel display data MUST BE CALLED FIRST

	TRISA = 0b00001111; //	0..3 inputs 4..7 outputs
	LATA = 0b00000000;
	ANCON0 = 0b00001111; // analog inputs 0-3
	ANCON1 = 0b00000000;
	TRISB = 0x00; //	outputs
	LATB = 0xff;
	TRISC = 0x00; //	outputs
	LATC = 0xff;
	TRISD = 0x00; //        outputs
	LATD = 0x00;
	TRISE = 0x00; //        outputs
	LATE = 0x00;

	OSCCON = 0x70; // internal osc
	OSCTUNE = 0xC0;
	SLRCON = 0x00; // set slew rate to max

	OpenTimer0(TIMER_INT_ON & T0_16BIT & T0_SOURCE_INT & T0_PS_1_1); // CTMU timer
	WriteTimer0(TIMERDISCHARGE); //	start timer0

	OpenTimer2(TIMER_INT_ON & T2_PS_1_4 & T2_POST_1_16); // PWN isr timer
	IPR1bits.TMR2IP = 0; // set timer2 low pri interrupt
	WriteTimer2(PDELAY);

	/* HOST */
	Open2USART(USART_TX_INT_ON & //FIXME need to check for correct speed
		USART_RX_INT_ON &
		USART_ASYNCH_MODE &
		USART_EIGHT_BIT &
		USART_CONT_RX &
		USART_BRGH_LOW, 103); // 64mhz 9600 baud

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

	/* Loop forever */

	while (TRUE) {
		for (ctmu_button = 0; ctmu_button <= MAX_CHAN; ctmu_button++) {


			touch_channel(ctmu_button);
			if (ctmu_button == 0) {
				t = ctmu_touch(ctmu_button, FALSE); // display channel  0 only
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

				INTCONbits.GIEL = 0;
				scan_off; // suspend list processing during matrix operations
				if (switchState == UNPRESSED) {
					times = ROT_SPEED;
					//pixel_init();
					obj_init(0, TRUE); // clear memory to only selected objects
					obj1 = obj_init(romid, FALSE); // return ID for rom object into ram id
					object_scale(obj1, scaling, scaling);
					object_rotate(obj1, rotation);
					object_trans(obj1, 3, 3);
				} else {
					times = ROT_SPEED;
					obj_init(0, TRUE); // clear memory to only selected objects
					obj1 = obj_init(romid, FALSE); // return ID for rom object into ram id
					object_scale(obj1, 2.0 - scaling, 2.0 - scaling);
					object_rotate(obj1, 360.0 - rotation);
					object_trans(obj1, 3, 3);
				}
				INTCONbits.GIEL = 1;
				scan_on();
				rotation += ROTATION;
				if (rotation > 360.00) {
					rotation = 0.0;
					scaling -= 0.1;
					if (scaling < -0.01) {
						scaling = 2.0;
						if (romid == 9) {
							romid = 13;
						} else {
							romid = 9;
						}
					}
				}
				move = 0;
			}
		}
	}
}
