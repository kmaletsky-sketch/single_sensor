/*
 * Control routine for single_sensor board
 *  - Based on the analog level of the PA3 pin play a track on DFPlayer Mini when
 *    that voltages crosses a threshold. The direction of crossing is controlled by PA2
 *  - Flash an LED for various functions and as a heartbeat
 *
 * Bit-banged UART to talk to the DFPlayer Mini
 *
 * This is bare-metal code for ATtiny402 using avr-gcc headers and is not based on the Arduino IDE
 */

#include <avr/io.h>
#include <util/delay.h>
#include <stdint.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>

// Misc declarations
static void flash_led(uint8_t count_flash);

/*
 * ADC Related items
 */
// Set VCC_MV to match your board supply (5000 for 5V, 3300 for 3.3V).
#ifndef VCC_MV
#define VCC_MV 5000UL
#endif

// Misc ADC constants
#define ADC_MAX 1023UL

// The threshold between light and dark. For the 5516 photocell
//  with typical room lighting (1-2K light and 5-10K dark) 
//  and the value of the resistors in the divider (3K to Vcc)
//
// For other photocells, resistors and/or other lighting conditions this voltage will differ
#define THRESH_MV 2750UL		// The voltage between light and dark

// The threshold in terms of counts (AVR has a 10 bit A/D
#define	THRESHOLD	 ((uint16_t) ((THRESH_MV * ADC_MAX) / VCC_MV))

// Initialize PA3 as an ADC input. In birthday mode, PA3 is considered digital
static void init_adc(void)
{
  // Ensure PA3 is input (analog)
  PORTA.DIRCLR = PIN3_bm;
  
  // Disable digital input buffer
  //  This also disables the pullup which would be enabled wth the PORT_PULLUPEN_bm bit
  PORTA.PIN3CTRL = PORT_ISC_INPUT_DISABLE_gc;

  // Select PA3 as ADC input
  ADC0.MUXPOS = ADC_MUXPOS_AIN3_gc;
  // Enable ADC, set for 10 bit resolution
  ADC0.CTRLA = ADC_ENABLE_bm | ADC_RESSEL_10BIT_gc;
  // Set for single sample mode at this time
  ADC0.CTRLB = ADC_SAMPNUM_ACC1_gc;
  // Set Reference to VDD
  // Set Clock Prescaler to divide by 16
  //   Per the datasheet the ADC requires an input clock frequency between
  //   50 kHz and 1.5 MHz for maximum resolution
  //   Dividing 10 MHz by 16 yields 625 KHz
  // It takes ~15 cycles for a conversion so that is 24us, several orders of magnitude
  //   faster than it needs to be with an input RC time constant of ~10ms
  ADC0.CTRLC = ADC_PRESC_DIV16_gc | ADC_REFSEL_VDDREF_gc;
}

// Perform a conversion on the ADC, here always PA3 and compare it to the threshold
//   value we've chosen. Return true if above that voltage
static bool read_adc(void)
{
	// Start ADC conversion on PA3
	ADC0.COMMAND = ADC_STCONV_bm;
	
	// Wait for result ready
	while (!(ADC0.INTFLAGS & ADC_RESRDY_bm))
	    ;
	uint16_t sample = ADC0.RES;			// Make sure all the bits are read
	
	ADC0.INTFLAGS = ADC_RESRDY_bm;	// clear the ready bit, may not be necessary here
	
	// true if the current analog value is above the threshold
	return(sample > THRESHOLD);
}

// read_adc() returns false (0) on light hitting the sensor
#define	ADC_LIGHT	false
#define	ADC_DARK	true

// Check to see if read_adc() returns the target value 20 times in a row over a second
//  No early out on failure, always take 1 second
static bool check_constant_adc(bool target)
{
	uint8_t	i;
	bool		retval = true;
	
	for(i=0; i<20; i++)
	{
		_delay_ms(50);
		if (read_adc() != target)	retval = false;
	}
	
	return(retval);
}

/*
 * PA1 is an AVR_controlled LED which is used for heartbeat and
 *  diagnostic functions.
 */
static void led_on(void) {
    PORTA.OUTCLR = PIN1_bm;
}
static void led_off(void) {
    PORTA.OUTSET = PIN1_bm;
}

#define	HALF_CYCLE	250
static void flash_led(uint8_t count_flash) {
	uint8_t i;
	
	for(i=0; i<count_flash; ++i)
	{
    led_on();
    _delay_ms(HALF_CYCLE);
		led_off();
		_delay_ms(HALF_CYCLE);		
	}	
}


/*
 * Serial interface
 *
 * This version of the software uses a bit-bang UART instead
 *  of the hardware in the AVR. Maybe a future version of the code
 *  will use that hardware.
 */

// Pin definitions (PA6 = TX, PA7 = RX)  use device-pack macros
#define TX_PIN_bm PIN6_bm
#define RX_PIN_bm PIN7_bm

// Serial parameters
#define BAUD 9600
// bit time in microseconds (rounded)
#define BIT_US (1000000UL / BAUD)

// Simple blocking bit-banged UART TX
static void uart_init_pins(void) {
	
    // TX output, drive high (idle)
    PORTA_DIRSET = TX_PIN_bm;
    PORTA_OUTSET = TX_PIN_bm;

    // RX input with pull-up enabled (clear DIR, set OUT for pull-up)
    PORTA_DIRCLR = RX_PIN_bm;
    PORTA_OUTSET = RX_PIN_bm;
}

static void uart_tx_byte(uint8_t b) {
    uint8_t i;
    
    // start bit (low)
    PORTA_OUTCLR = TX_PIN_bm;
    _delay_us(BIT_US);

    // data bits LSB first
    for (i = 0; i < 8; ++i) {
        if (b & (1 << i)) PORTA_OUTSET = TX_PIN_bm;
        else PORTA_OUTCLR = TX_PIN_bm;
        _delay_us(BIT_US);
    }

    // stop bit (high)
    PORTA_OUTSET = TX_PIN_bm;
    _delay_us(BIT_US);
}

// Blocking receive: waits for start bit, samples in middle of bit
// Returns 0 on timeout (if timeout_us==0, waits indefinitely)
static uint8_t uart_rx_byte(uint32_t timeout_us) {
    uint32_t waited = 0;

    // wait for start bit (line goes low)
    while (PORTA_IN & RX_PIN_bm) {
        if (timeout_us && (waited >= timeout_us)) return 0;
        _delay_us(10);
        waited += 10;
    }

    // found falling edge, wait half bit to sample center
    _delay_us(BIT_US / 2);

    uint8_t b = 0;
    for (uint8_t i = 0; i < 8; ++i) {
        _delay_us(BIT_US);
        if (PORTA_IN & RX_PIN_bm) b |= (1 << i);
    }

    // wait stop bit time
    _delay_us(BIT_US);
    return b;
}

/*
 * The audio module for this board is the DFPlayer Mini,
 *  originally created by DFRobot and still sold by them but
 *  also available as a wide range of clones, some of which are better
 *  than others!
 */
#define	PLAY_TRACK_CMD		0x03		// Param is track number
#define	REPEAT_TRK_CMD		0x19		// Repeat the currently playing track (0=on, 1=off)
#define	VOLUME_UP 				0x04
#define	VOLUME_DOWN				0x05
#define	VOLUME_CMD				0x06		// Param is volume 0-30
#define	REPEAT_TRACK_CMD	0x08		// Param is track number
#define	PAUSE_CMD					0x0E
#define	STOP_CMD					0x16
#define	QUERY_VOLUME_CMD	0x43		// Returns the current volume 0-30
#define	QUERY_STATUS_CMD	0x42 		// Returns one of the values below

#define	QUERY_RESP_BYTE	  6		    // Index 6 (7th byte) is the useful query response

#define	QUERY_STOPPED			0
#define	QUERY_PLAYING			1
#define	QUERY_BUSY  			4			// such as initialization

#define	MAX_VOLUME				30

#define	RESP_SIZE					10		// All responses should be this long

// Not exactly sure just what the feedback byte is for. Every example I have seen
//  sets this byte to 0 so that is fixed in this software
#define FEEDBACK          1    // feedback requested
#define NO_FEEDBACK       0    // no feedback requested
/*
 * Send a frame (cmd) to the DFPlayer Mini
 *
 * frame: 0x7E 0xFF 0x06 cmd feedback param1 param2 checksum_hi checksum_lo 0xEF
 */
static void dfplayer_send_cmd(uint8_t cmd, uint8_t p1, uint8_t p2) {
    uint16_t sum = 0;
    uint8_t frame[10];
    
    frame[0] = 0x7E;
    frame[1] = 0xFF;
    frame[2] = 0x06;
    frame[3] = cmd;
    frame[4] = NO_FEEDBACK;
    frame[5] = p1;
    frame[6] = p2;

    sum = frame[1] + frame[2] + frame[3] + frame[4] + frame[5] + frame[6];
    uint16_t checksum = 0xFFFF - sum + 1;
    frame[7] = (uint8_t)(checksum >> 8);
    frame[8] = (uint8_t)(checksum & 0xFF);
    frame[9] = 0xEF;

    for (uint8_t i = 0; i < 10; ++i) uart_tx_byte(frame[i]);
}

// Read DFPlayer response into provided buffer (max_len). Returns number of bytes read.
// Waits up to timeout_ms for first byte; after first byte, waits 50ms between bytes.
static uint8_t dfplayer_read_resp(uint8_t *buf, uint8_t max_len, uint32_t timeout_ms) {
    uint8_t idx = 0;
    uint32_t waited = 0;

    // wait for first byte
    while ((PORTA_IN & RX_PIN_bm) && (waited < (timeout_ms * 1000UL))) {
        _delay_us(100);
        waited += 100;
    }
    if (waited >= (timeout_ms * 1000UL)) return 0;

    // read until end byte 0xEF or buffer full
    while (idx < max_len) {
        uint8_t v = uart_rx_byte(200000); // 200ms per byte timeout
        //if (v == 0) break;
        buf[idx++] = v;
        if (v == 0xEF) break;
    }
    return idx;
}

/* 
 * Get DFPlayer status (send the Query_Status command)
 *   In general, we expect to wait between queries but in some
 *   situations might not want the wait
 */
static uint8_t dfplayer_send_query(uint8_t command, uint8_t wait_tenths) {
		uint8_t resp_buf[16];		// should only ever be 10 bytes...
		uint8_t	i;

    // Query status to see when it's done playing or initializeing
    while(1) {
			// _delay_ms() expects a compile time delay
			for (i=wait_tenths; i > 0; --i)	_delay_ms(100);
			
	    dfplayer_send_cmd(QUERY_STATUS_CMD, 0x00, 0x00);		// status query
	    
	    // read response
	    uint8_t n = dfplayer_read_resp(resp_buf, sizeof(resp_buf), 500);
	   	if (n != RESP_SIZE)
	   		{ flash_led(2); continue;	} // try again on a failure
	   		
	   	return(resp_buf[QUERY_RESP_BYTE]);	// We only care about one byte
	  }
}

// Reduce the volume quickly instead of an abrupt stop
#define	FAST_REDUCE_STEPS	4
#define	FAST_REDUCE_TIME	1			// seconds for entire reduction loop
#define	FAST_REDUCE_DELAY	((FAST_REDUCE_TIME * 1000) / FAST_REDUCE_STEPS)
static void reduce_volume_fast(void)
{
	uint8_t	i;

	uint8_t	cur_volume = dfplayer_send_query(QUERY_VOLUME_CMD, 0);	// Get current volume without waiting
	uint8_t	vol_delta = cur_volume / FAST_REDUCE_STEPS;
	
	// Reduce the volume in 4 increments
	for(i=FAST_REDUCE_STEPS; i>0; i--)
	{
		// Depending on the values chosen and the way the division result get rounded
		//  we don't want cur_volume to go (essentially) negative
		if (cur_volume >= vol_delta) cur_volume -= vol_delta;
		else												 cur_volume = 0;
			
		dfplayer_send_cmd(VOLUME_CMD, 0x00, cur_volume);
		_delay_ms(FAST_REDUCE_DELAY);
	}
}

/*
 * Initialize the pins, AVR, DFPlayer Mini
 */
 
// The default clock rate for the Tiny402 is 20MHz but nothing in this
//  application really requires that speed. At least initially, anything
//  less than 10 MHz causes the UART code to fail
// Make sure that ADC clock rate is consistent with the setting here
#define	PDIV_2	(0)																				// 10 MHz
#define	PDIV_4	(CLKCTRL_PDIV_0_bm)												// 5 MHz
#define	PDIV_16	(CLKCTRL_PDIV_0_bm | CLKCTRL_PDIV_1_bm)		// 1.25 MHz
static void init_avr(void)
{
    uint8_t	timeout;
    
    // set clock rate to 10Mhz (assuming fuse 0x02 is set to 2)
    //  -->> This must match the compile-time variable
	  _PROTECTED_WRITE(CLKCTRL.MCLKCTRLB, CLKCTRL_PEN_bm | PDIV_2); 
	
		// Configure PA1 (LED) as output and drive it high initially
    PORTA.DIRSET = PIN1_bm;
    led_off();

    // Configure PA2 as input with internal pull-up - the state of this pin
    //  determines the way in which music is played
    PORTA.DIRCLR = PIN2_bm;
    PORTA.PIN2CTRL = PORT_PULLUPEN_bm;
    
    // Initialize the UART pins
    uart_init_pins();
    
    flash_led(1);	// Startup signal

		// let DFPlayer power up. Could take from 1-5 seconds
		for(timeout=0; dfplayer_send_query(QUERY_STATUS_CMD, 50) == QUERY_BUSY; timeout++)
		{
			// This should be replaced with a watchdog timer function to reset
			//  the AVR as it does not have a reset instruction per se
			if (timeout > 10)
			{
				while(1)	flash_led(1);
			}
		}
}

/*
 * There are two modes of operation:
 *	Train Mode: A model train going around a circular track with the sensor located on the track
 *   before a crossing. When the light level at the sensor decreases (analog level goes high) 
 *   the music starts playing. All music transitions end with a volume reduction
 *   - If the sensor either stays dark or switches between light and dark (space between train cars)
 *       the music keeps playing.
 *   - If the music ends while the sensor is still dark then wait until it's steady light
 *       before looking for the next dark transition to start the music again
 *   - If it's steady light (dark transitions end) before the music ends then the music
 *       gets cut off.
 
 *	Birthday Mode: The sound board is in a box with a battery. When the box is opened it starts
 *   playing a song - presumably "Happy Birthday". When the light level at the sensor increases
 *	 (analog level goes low) the music starts playing on repeat. All music is at full volume.
 *   - If the sensor goes dark while the music is still playing, the song goes on pause and is
 *       resumed when the sensor sees light again.
 *
 * Train mode is selected by leaving PA2 open or jumpering it high. Birthday mode is selected
 *  by jumpering PA2 to ground. PA2 is read only once on power-on.
 */
 
/*
 * Train mode routine
 *
 *  Typical HO train speeds are 1-3 feet/second (corresponding to 20-60mph IRL). Figure a typical car is
 *  1/3 to 1/2 a foot. So the bursts of light between cars should be spaced 2-9 times per second.
 *  If the sensor is lit for an entire second we consider the train is gone 
 */
#define	TRAIN_INITIAL_WAIT	3			// After starting the track, wait this long before reducing volume
#define	TRAIN_WAIT_END			3			// Seconds after transitions end before cutting music off
#define	VOL_REDUCE_INTERVAL	2			// Wait this many seconds between volume reductions
#define	IDLE_READ_ADC				100		// Read the ADC 10 times/second when idle

static void train_mode(void)
{
	uint8_t	i;
		
	// Initialize the ADC which we use for precision comparison of the analog input (photocell resistor divider)
	init_adc();

	while(1) {		
		// Nothing is happening and sensor shows light. Wait a bit and try again
		if (read_adc() == ADC_LIGHT)	{
			_delay_ms(IDLE_READ_ADC);
			continue;
		}
		
		// No sensor noise suppression on starting to play track
		dfplayer_send_cmd(VOLUME_CMD, 0x00, MAX_VOLUME);
    dfplayer_send_cmd(PLAY_TRACK_CMD, 0x00, 0x01);
    flash_led(1);
    
    // Always wait this long before looking to reduce volume or shut track off
		_delay_ms(TRAIN_INITIAL_WAIT * 1000);
   	flash_led(1);
   	uint8_t vol_reduce_cnt = 0;

		// Slowly reduce the volume as the train moves beyond the crossing. For slow/long trains and/or short sound tracks the
		/// sound might go away too soon. And for short trains there might be no actual volume reduction
 		while(1) {
			// See if the ADC is detecting light for an entire second. This routine *always* takes a second to execute
			if (check_constant_adc(ADC_LIGHT)) {
				
				// We always wait a little for the train to clear the crossing since the sensor is expected to
				//   be *BEFORE* the crossing. We already waited one second in check_constant_light()
				for(i=0; i < (TRAIN_WAIT_END - 1); ++i, _delay_ms(1000))
					dfplayer_send_cmd(VOLUME_DOWN, 0x00, 0x00);

				// Now fade the track out quickly - it might have already ended but that's OK
				reduce_volume_fast();
				
				flash_led(3);
				
				// Back to main loop waiting for new train
				break;	
			}
			
			// See if we should reduce the volume. It's reduced by only one unit out of 30, so would take
			//   an entire minute to reduce it to zero at a 2 second interval
			if (vol_reduce_cnt >= VOL_REDUCE_INTERVAL) {
				dfplayer_send_cmd(VOLUME_DOWN, 0x00, 0x00);
				vol_reduce_cnt = 0;
				flash_led(1);
			}
			else
				vol_reduce_cnt++;
    }	
	}
}

/*
 * Birthday mode routines
 *
 * In this mode, we wait for the PA3 input pin to go low and then we start playing the song.
 *
 * The DFplayer mini has a standby mode but most commenters suggest that at best it reduces the current from 20mA
 *   to 15mA. Waking out of standby mode is said to be unreliable. The 2575 switching regulator adds another 5-10mA
 *   of current so the best we could hope for is something in the range of 30mA
 * 9V battery has a power capacity of around 500mA-Hr so can't expect it to last more than about 15 hours.
 *
 * Nonetheless - as an exercise - we will put the AVR to sleep and look for a digital low level on PA3. The assumption for 
 *   this mode is that the two light levels are 'very bright' and 'very dark' so that the actual voltage levels on the pin
 *   are either close to 0V or close to 5V. We might need to increase the size of the upper resistor on the photocell
 *   divider to get to a logic low level input.
 *
 *   PA2 & PA6 are the designated asynchronous wake pins on the tiny402. The other pins work but only in
 *     BOTHEDGES or LEVEL mode (and not RISING or FALLING). The board only has PA3 as an input. 
 */
#define	PLAY_REPEATEDLY 1							// Comment this line out to play only once
 
// Other than clearing the interrupt flag the ISR really has no function beyond waking the chip up
ISR(PORTA_PORT_vect)
{
    if (PORTA.INTFLAGS & PIN3_bm) {
        PORTA.INTFLAGS = PIN3_bm;   // clear interrupt flag
    }
}

// One time setup of PA3
static void pa3_wake_init(void)
{
    cli();

    // Configure as an input
    PORTA.DIRCLR = PIN3_bm;

    // Disable ADC to save power, though it is normally disabled by default
    ADC0.CTRLA &= ~ADC_ENABLE_bm;

    // Clear any pending port interrupt flag
    PORTA.INTFLAGS = PIN3_bm;

    // Enable port interrupt on low level. No need for a pullup as the input is a resistor divider on the board
    PORTA.PIN3CTRL = PORT_ISC_LEVEL_gc;

    sei();
}

static void go_to_sleep(void)
{
    set_sleep_mode(SLEEP_MODE_PWR_DOWN);		// It may not be necessary to call this routine each time
    sleep_enable();
    sleep_cpu();
    
    // On wake, we get to here after executing the ISR
    sleep_disable();
}

static void birthday_mode(void)
{
    
    pa3_wake_init();																	// Initialize PA3 to be wake on low level
    dfplayer_send_cmd(VOLUME_CMD, 0x00, MAX_VOLUME);	// Not strictly necessary as this is the default

    while (1) {
    	// Enable interrupts now that the pin is high then sleep until the analog input/pin goes low
    	sei();
      go_to_sleep();

	    // We really wanted an edge sensitive interrupt but that's not possible with PA3 and anyway
	    //  there might be some noise near the edge on the sensor signal which we'd like to ignore
	    // So turn off interrupts and we'll re-enable them when the pin goes high
	    cli();

			// Start playing the track. Playing endlessly to incentivize the user to go to
			//   dark mode and save a little battery life. But it could be very annoying
	    dfplayer_send_cmd(PLAY_TRACK_CMD, 0x00, 0x01);
#ifdef PLAY_REPEATEDLY
	    dfplayer_send_cmd(REPEAT_TRK_CMD, 0x00, 0x00);		// Param of 0 turns this mode on
#endif // PLAY_REPEATEDLY

	    flash_led(1);
	   
	   // As long as there is light, we'll stay in this loop
	    while(1) {
	    	// Once sensor goes dark we can go back to sleep
	    	if ((PORTA.IN & PIN3_bm) != 0)
	    		break;
	    	
	    	// Wait only half a second to check again as we want the music to stop right away
	    	_delay_ms(500);
	    	
	    	flash_led(1);
	    }
	    
	    // If the sensor goes dark when we are still playing, pause the track. To ensure that we are in
	    //   sync, force a stop if query doesn't say that we are playing. We'll always be playing if in
	    //   PLAY_REPEATEDLY mode so nothing left to do but stop
#ifndef PLAY_REPEATEDLY
	    if (dfplayer_send_query(QUERY_STATUS_CMD, 50) == QUERY_PLAYING)
	    	dfplayer_send_cmd(PAUSE_CMD, 0x00, 0x00);
	    else
#endif // PLAY_REPEATEDLY

	    	dfplayer_send_cmd(STOP_CMD, 0x00, 0x00);
    }
}
	

int main(void)
{
	// All the initialization functions other than the ADC are gathered in here
	init_avr();
	
	// Read PA2 level. This isn't intended to be dynamic, rather a jumper or left open - so we read it once only
	// Neither of these "_mode()" routines return
 	if ((PORTA.IN & PIN2_bm) == 0)
 		birthday_mode();
  else
 		train_mode();												

	// Should never get here. The AVR seems to just start back at the beginning of main if we do?
  return 0;
}
