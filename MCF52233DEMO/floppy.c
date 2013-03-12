#include <assert.h>
#include "floppy.h"
#include "support_common.h"
#include "exceptions.h"
#include "uart_support.h"


#if (CONSOLE_IO_SUPPORT || ENABLE_UART_SUPPORT) 
#include <stdio.h>
#endif

///////////////////
// NOTE HANDLING //
///////////////////

const uint16 NOTE_PERIOD[] =
{
	0,
    6116, 5772, 5448, 5143, 4854, 4582, 4324, 4082, 3853, 3636, 3432, 3240,
    3058, 2886, 2724, 2571, 2427, 2291, 2162, 2041, 1926, 1818, 1716, 1620,
    1529, 1443, 1362, 1286, 1213, 1145, 1081, 1020, 963, 909, 858, 810,
    764, 722, 681, 643, 607, 573, 541, 510, 482, 455, 429, 405,
    382, 361, 341, 321, 303, 286, 270, 255, 241, 227, 215, 202,
    191, 180, 170, 161, 152, 143, 135, 128, 120, 114, 107, 101,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
};

// Hand initialize these given NOTE_PERIOD data.
// Used in getNearestNoteIndex() and setFloppyPeriod().
const uint16 MIN_NOTE_INDEX = 1;
const uint16 MAX_NOTE_INDEX = 72;	// Post pitchbend
const uint16 MIN_NOTE_PERIOD = 101;	// Post pitchbend

const int CHROMATIC[] = 
{
	37, 38, 39, 40, 41, 42, 43, 44, 45, 46, 47, 48,
	61,
};

const int IONIAN[] =
{
	37, 39, 41, 42, 44, 46, 48,
	49,
};

const int DORIAN[] = 
{
	37, 39, 40, 42, 44, 46, 47,
	49,
};

const int PHRYGIAN[] = 
{
	37, 38, 40, 42, 44, 45, 47,
	49,
};

const int LYDIAN[] = 
{
	37, 39, 41, 43, 44, 46, 48,
	49,
};

const int MIXOLYDIAN[] = 
{
	37, 39, 41, 42, 44, 46, 47,
	49,
};

const int AEOLIAN[] = 
{
	37, 39, 40, 42, 44, 45, 47,
	49,
};

const int LOCRIAN[] = 
{
	37, 38, 40, 42, 43, 45, 47,
	49,
};

const int BLUES[] = 
{
	37, 40, 42, 43, 44, 47,
	49,
};

const int* SCALE[] =
{
	CHROMATIC, IONIAN, DORIAN, PHRYGIAN, LYDIAN,
	MIXOLYDIAN, AEOLIAN, LOCRIAN, BLUES,
};

const int SCALE_SIZE[] =
{
	13, 8, 8, 8, 8, 8, 8, 8, 7,
};

const int NUM_SCALES = 9;

uint8 current_scale = 0;


//////////////////////////////
// DISTANCE SENSOR HANDLING //
//////////////////////////////

const uint8 NUM_SAMPLES = 8;
const uint8 CM_PER_NOTE = 6;
const uint8 MIN_DISTANCE = 10;
uint8 max_distance;	// Initialized in instrumentModeLoop()


/////////////////////
// FLOPPY HANDLING //
/////////////////////

// The GPIO port for each floppy (see floppy.h).
vuint8* floppy_port[NUM_FLOPPIES] =
{
	FDD0, FDD1, FDD2, FDD3, FDD4, FDD5, //FDD6, FDD7, 
};

// The value to OR into a floppy's GPIO port
// in order to output high on the step pin.
const uint8 FLOPPY_STEP_BIT[NUM_FLOPPIES] = 
{
	FDD0_STEP, FDD1_STEP, FDD2_STEP, FDD3_STEP, 
	FDD4_STEP, FDD5_STEP, //FDD6_STEP, FDD7_STEP, 
};

// The value to OR into a floppy's GPIO port
// in order to output high on the direction pin.
const uint8 FLOPPY_DIRECTION_BIT[NUM_FLOPPIES] = 
{
	FDD0_DIRECTION, FDD1_DIRECTION, FDD2_DIRECTION, FDD3_DIRECTION, 
	FDD4_DIRECTION, FDD5_DIRECTION, //FDD6_DIRECTION, FDD7_DIRECTION, 
};

// Number of interrupts between pulsing stepper motor
// while resetting floppy drive heads
const uint16 RESET_STEP_PERIOD = (int) (INTERRUPT_FREQUENCY / 240.0 + .5);

// The MIDI channel associated with each FDD.
uint8 floppy_to_channel[NUM_FLOPPIES] =
{
    0,1,2,3,2,3,//6,7,
};

// Counts down from NUM_TRACKS - 1 to 0 while drive
// heads are being reset in interrupt routine.
// While reset_steps_until_finished != 0,
// all drives will be pulsed every reset_step_period
// interrupts in timer_handler(). When
// reset_steps_until_finished == 0, drives will respond
// to normal requests.
uint16 reset_steps_until_finished = 0;

// Number of interrupts left before pulsing stepper motor
// while resetting floppy drive heads
uint16 reset_count_until_step;

// Number of interrupts between pulsing stepper motor,
// given a floppy drive's current note frequency.
// Value of 0 indicates floppy is not playing a note.
uint16 step_period[NUM_FLOPPIES];

// Number of interrupts left before pulsing stepper motor
// Might need to be initialized to positive number...
uint16 count_until_step[NUM_FLOPPIES];

// Number of steps until head direction must be reversed
uint16 count_until_reversal[NUM_FLOPPIES];


////////////////////////////////
// SEVEN SEGMENT LED HANDLING //
////////////////////////////////

const uint8 SSEG_BLANK = 0;
const uint8 SSEG_A = 0b01110111;
const uint8 SSEG_B = 0b01111111;
const uint8 SSEG_C = 0b01001110;
const uint8 SSEG_D = 0b00111101;
const uint8 SSEG_E = 0b01001111;
const uint8 SSEG_F = 0b01000111;
const uint8 SSEG_G = 0b01111011;
const uint8 SSEG_FLAT = 0b00011111;

const uint8 SSEG_DIGIT0_SYMBOLS[] =
{
	SSEG_C, SSEG_D, SSEG_D,
	SSEG_E, SSEG_E, SSEG_F,
	SSEG_G, SSEG_G, SSEG_A,
	SSEG_A, SSEG_B, SSEG_B
};

const uint8 SSEG_DIGIT1_SYMBOLS[] =
{
	SSEG_BLANK, SSEG_FLAT, SSEG_BLANK,
	SSEG_FLAT, SSEG_BLANK, SSEG_BLANK,
	SSEG_FLAT, SSEG_BLANK, SSEG_FLAT,
	SSEG_BLANK, SSEG_FLAT, SSEG_BLANK
};

const uint8 SSEG0_FLOPPY = 0;

uint8 sseg0_digit0 = 0;
uint8 sseg0_digit1 = 0;
uint8 current_sseg_digit = 0;


//////////
// CODE //
//////////

int main(void)
{
	initializeFloppies();
	initializeGPIO();
	initializeInterrupts();
	initializePIT();
	initializeADC();
	uart_init(0, SYSTEM_CLOCK_KHZ, kBaud19200);
	
	resetFloppies(); // Enables interrupts only during function
	
	// Enable PIT0 timer
	MCF_PIT0_PCSR |= MCF_PIT_PCSR_EN;
	
	midiModeLoop();
	//instrumentModeLoop();
}

void midiModeLoop() 
{
	unsigned char channel;
	uint8 period_high_byte;
	uint8 period_low_byte;
	uint16 period;
	uint8 nearest_note_index;
	uint8 nearest_note_mod12;
	uint8 i;
	
	for (;;)
	{
		channel = uart_getchar(0);
		
		if (channel == RESET_FLOPPIES_MESSAGE) 
		{
			sseg0_digit0 = SSEG_BLANK;
			sseg0_digit1 = SSEG_BLANK;
			
			resetFloppies();
		}
		else 
		{
			period_high_byte = uart_getchar(0);
			period_low_byte = uart_getchar(0);
			period = (uint16) (((period_high_byte & 0xFF) << 8) |
				(period_low_byte & 0xFF));

			for (i = 0; i < NUM_FLOPPIES; i++)
			{
				if (channel == floppy_to_channel[i]) 
				{
					setFloppyPeriod(i, period);
				}
			}

			if (period == 0) 
			{
				if (channel == floppy_to_channel[SSEG0_FLOPPY]) 
				{
					sseg0_digit0 = SSEG_BLANK;
					sseg0_digit1 = SSEG_BLANK;
				}
			}
			else
			{
				nearest_note_index = getNearestNoteIndex(period);
				nearest_note_mod12 = (uint8) ((nearest_note_index - 1) % 12);
				
				if (channel == floppy_to_channel[SSEG0_FLOPPY]) 
				{
					sseg0_digit0 = SSEG_DIGIT0_SYMBOLS[nearest_note_mod12];
					sseg0_digit1 = SSEG_DIGIT1_SYMBOLS[nearest_note_mod12];
				}
			}
		}
	}
}

void instrumentModeLoop() 
{
	uint8 counter = 0;
	uint16 ds0_adc_value;
	uint16 ds1_adc_value;
	uint8 cm;
	uint8 note_index;
	uint8 i;
	uint32 j;
	uint8 nearest_note_mod12;
	
	// Must do this once here since this compiler
	// doesn't like it outside of function scope.
	max_distance = (uint8) (MIN_DISTANCE + CM_PER_NOTE *
		SCALE_SIZE[current_scale]);
	
	for (;;)
	{
		ds0_adc_value = 0;
		ds1_adc_value = 0;
		
		// Take a few samples and average them
		for (i = 0; i < NUM_SAMPLES; i++) 
		{
	        // Start ADC conversion
	        MCF_ADC_CTRL1 = MCF_ADC_CTRL1_START0;
	        
	        // Delay before reading result
			for (j = 0; j < 0x0ffff; j++)
	        {
	        	// Prevent loop from being optimized away.
	        	// Might not work with full optimizations on.
				counter++;
	        }
	        
		    // Read result
	        ds0_adc_value += MCF_ADC_ADRSLT1 >> 3;
	        ds1_adc_value += MCF_ADC_ADRSLT0 >> 3;
		}
		
		ds0_adc_value /= NUM_SAMPLES;
		ds1_adc_value /= NUM_SAMPLES;
		
		cm = 1.0 / ((ds0_adc_value - 2900) / 27000.0 + .1);
			
		if (cm >= MIN_DISTANCE && cm < max_distance) 
		{
			note_index = (uint8) ((cm - MIN_DISTANCE) / CM_PER_NOTE);

			setFloppyPeriod(0, NOTE_PERIOD[SCALE[current_scale][note_index]]);
			setFloppyPeriod(1, NOTE_PERIOD[SCALE[current_scale][note_index]]);
			setFloppyPeriod(2, NOTE_PERIOD[SCALE[current_scale][note_index]]);

			nearest_note_mod12 = (uint8) ((SCALE[current_scale][note_index] - 1) % 12);
			
			sseg0_digit0 = SSEG_DIGIT0_SYMBOLS[nearest_note_mod12];
			sseg0_digit1 = SSEG_DIGIT1_SYMBOLS[nearest_note_mod12];
		}
		else 
    	{
    		setFloppyPeriod(0, 0);
    		setFloppyPeriod(1, 0);
    		setFloppyPeriod(2, 0);
    		
			sseg0_digit0 = SSEG_BLANK;
			sseg0_digit1 = SSEG_BLANK;
    	}

		cm = 1.0 / ((ds1_adc_value - 2900) / 27000.0 + .1);

		if (cm >= MIN_DISTANCE && cm < max_distance) 
		{
			note_index = (uint8) ((cm - MIN_DISTANCE) / CM_PER_NOTE);

			setFloppyPeriod(3, NOTE_PERIOD[SCALE[current_scale][note_index]]);
			setFloppyPeriod(4, NOTE_PERIOD[SCALE[current_scale][note_index]]);
			setFloppyPeriod(5, NOTE_PERIOD[SCALE[current_scale][note_index]]);
		}
		else 
    	{
    		setFloppyPeriod(3, 0);
    		setFloppyPeriod(4, 0);
    		setFloppyPeriod(5, 0);
       	}
    }
}

__declspec(interrupt:0) void timerHandler(void) 
{
	int i;
	
	// Writing 1 to PCSR[PIF] clears interrupt
	MCF_PIT0_PCSR |= MCF_PIT_PCSR_PIF;
	
	// If still resetting floppy drives...
	if (reset_steps_until_finished != 0) 
	{
		// If it's time to step the motors...
		if (--reset_count_until_step == 0) 
		{
			// Step each floppy's motor
			for (i = 0; i < NUM_FLOPPIES; i++) 
			{
				*floppy_port[i] |= FLOPPY_STEP_BIT[i];
				*floppy_port[i] &= ~FLOPPY_STEP_BIT[i];
			}
			
			// Set up counter for next motor step
			reset_count_until_step = RESET_STEP_PERIOD;
			reset_steps_until_finished--;
		}
	}
	else 
	{
		for (i = 0; i < NUM_FLOPPIES; i++) 
		{
			// If this floppy is currently playing a note,
			// and if it is time to step the motor...
			// (Be aware of short circuiting in conditional...)
			if (step_period[i] != 0 && --count_until_step[i] == 0) 
			{
				// Step the motor
				*floppy_port[i] |= FLOPPY_STEP_BIT[i];
				*floppy_port[i] &= ~FLOPPY_STEP_BIT[i];
				
				// Reset counter
				count_until_step[i] = step_period[i];
				
				if (--count_until_reversal[i] == 0) 
				{
					// Change direction
					*floppy_port[i] ^= FLOPPY_DIRECTION_BIT[i];
					count_until_reversal[i] = NUM_TRACKS - 1;
				}
			}
		}
	}
	
	// Change currently displayed digit of SSEGs
	if (current_sseg_digit == 0)
	{
		// Time to display digit 1...
		
		// Turn off digit 0
		SSEGOff(0);
		
		// Set symbol to display on digit 1
		setSSEG(sseg0_digit1);
		
		// Turn on digit 1
		SSEGOn(1);
		current_sseg_digit = 1;
	}
	else
	{
		// Time to display digit 0...
		
		// Turn off digit 1
		SSEGOff(1);

		// Set symbol to display on digit 0
		setSSEG(sseg0_digit0);
		
		// Turn on digit 0
		SSEGOn(0);
		current_sseg_digit = 0;
	}
}

__declspec(interrupt:0) void sw1Handler(void) 
{
	// Clear interrupt
	MCF_EPORT0_EPFR = MCF_EPORT_EPFR_EPF7;
	
	// Change scale
	current_scale = (uint8) ((current_scale + 1) % NUM_SCALES);
	max_distance = (uint8) (MIN_DISTANCE + CM_PER_NOTE *
		SCALE_SIZE[current_scale]);
}

// Pulls floppy read heads to back of drive
inline void resetFloppies()
{
	int i;
	// Save PIT0_PCSR state
	uint16 pcsr = MCF_PIT0_PCSR;
	
	// Disable PIT0
	MCF_PIT0_PCSR &= ~MCF_PIT_PCSR_EN;
	
	for (i = 0; i < NUM_FLOPPIES; i++) 
	{
		// Reset floppy drive state
		step_period[i] = 0;
		count_until_reversal[i] = NUM_TRACKS - 1;
		
		// Set direction to backward
		*floppy_port[i] |= FLOPPY_DIRECTION_BIT[i];
		
		// Initialize reset variables for interrupt routine
		reset_steps_until_finished = NUM_TRACKS;
		reset_count_until_step = RESET_STEP_PERIOD;
	}
	
	// Enable PIT0 timer
	MCF_PIT0_PCSR |= MCF_PIT_PCSR_EN;
	
	// Wait until reset...if the load of reset_steps_until_finished
	// is a word load, then there is no shared data problem
	while (reset_steps_until_finished > 0) 
	{
		;
	}
	
	// Disable PIT0 to set floppy directions safely
	MCF_PIT0_PCSR &= ~MCF_PIT_PCSR_EN;
	for (i = 0; i < NUM_FLOPPIES; i++) 
	{
		// Set direction to forward
		*floppy_port[i] &= ~FLOPPY_DIRECTION_BIT[i];
	}
	
	// Restore PIT0_PCSR state
	MCF_PIT0_PCSR = pcsr;
}

inline void initializeFloppies() 
{
	uint8 i;
	
	// Associate channels with floppies and set state variables.
	for (i = 0; i < NUM_FLOPPIES; i++) 
	{
		step_period[i] = 0;
		count_until_step[i] = 1;
		count_until_reversal[i] = NUM_TRACKS - 1;
	}
}

inline void initializeGPIO() 
{
	int i;
	
	// Enable GPIO on port TC for FDD0 and FDD1
	MCF_GPIO_PTCPAR = 0
		| MCF_GPIO_PTCPAR_DTIN0_GPIO	// FDD1 direction pin
		| MCF_GPIO_PTCPAR_DTIN1_GPIO	// FDD1 step pin
		| MCF_GPIO_PTCPAR_DTIN2_GPIO	// FDD0 direction pin
		| MCF_GPIO_PTCPAR_DTIN3_GPIO;	// FDD0 step pin
		
	// Set GPIO on TC to output mode
	MCF_GPIO_DDRTC = 0
		| MCF_GPIO_DDRTC_DDRTC0
		| MCF_GPIO_DDRTC_DDRTC1
		| MCF_GPIO_DDRTC_DDRTC2
		| MCF_GPIO_DDRTC_DDRTC3;
	
	
	// Enable GPIO on port TA for FDD2 and SSEG0
	MCF_GPIO_PTAPAR = 0
		| MCF_GPIO_PTAPAR_GPT0_GPIO		// SSEG0 C
		| MCF_GPIO_PTAPAR_GPT1_GPIO		// SSEG0 B
		| MCF_GPIO_PTAPAR_GPT2_GPIO		// FDD2 direction pin
		| MCF_GPIO_PTAPAR_GPT3_GPIO;	// FDD2 step pin
		
	// Set GPIO on TA to output mode
	MCF_GPIO_DDRTA = 0
		| MCF_GPIO_DDRTA_DDRTA0
		| MCF_GPIO_DDRTA_DDRTA1
		| MCF_GPIO_DDRTA_DDRTA2
		| MCF_GPIO_DDRTA_DDRTA3;
	
	
	// Enable GPIO on port UB for SSEG0
	MCF_GPIO_PUBPAR = 0
		| MCF_GPIO_PUBPAR_UTXD1_GPIO	// SSEG0 G
		| MCF_GPIO_PUBPAR_URXD1_GPIO	// SSEG0 F
		| MCF_GPIO_PUBPAR_URTS1_GPIO	// SSEG0 E
		| MCF_GPIO_PUBPAR_UCTS1_GPIO;	// SSEG0 D
		
	// Set GPIO on UB to output mode
	MCF_GPIO_DDRUB = 0
		| MCF_GPIO_DDRUB_DDRUB0
		| MCF_GPIO_DDRUB_DDRUB1
		| MCF_GPIO_DDRUB_DDRUB2
		| MCF_GPIO_DDRUB_DDRUB3;
	
	
	// Enable GPIO on port QS for SSEG0 and LED
	MCF_GPIO_PQSPAR = 0
		| MCF_GPIO_PQSPAR_QSPI_DOUT_GPIO// SSEG0 A
		| MCF_GPIO_PQSPAR_QSPI_DIN_GPIO	// SSEG0 DIG1
		| MCF_GPIO_PQSPAR_QSPI_CLK_GPIO // SSEG0 DIG0
		| MCF_GPIO_PQSPAR_QSPI_CS0_GPIO;// FDD4 LED
	// Set GPIO on QS to output mode
	MCF_GPIO_DDRQS = 0
		| MCF_GPIO_DDRQS_DDRQS0
		| MCF_GPIO_DDRQS_DDRQS1
		| MCF_GPIO_DDRQS_DDRQS2;


	// Enable GPIO on port AS for FDD3
	MCF_GPIO_PASPAR = 0
		| MCF_GPIO_PASPAR_SCL_GPIO		// FDD3 direction pin
		| MCF_GPIO_PASPAR_SDA_GPIO		// FDD3 step pin
		| MCF_GPIO_PASPAR_SYNCA_GPIO	// FDD7 direction pin
		| MCF_GPIO_PASPAR_SYNCB_GPIO;	// FDD7 step pin

	// Set GPIO on AS to output mode
	MCF_GPIO_DDRAS = 0
		| MCF_GPIO_DDRAS_DDRAS0
		| MCF_GPIO_DDRAS_DDRAS1
		| MCF_GPIO_DDRAS_DDRAS2
		| MCF_GPIO_DDRAS_DDRAS3;
		

	// Enable ADC on port AN for DS0 and DS1, and
	// enable GPIO on port AN for FDD4, FDD5, and FDD6.
	MCF_GPIO_PANPAR = 0
		| MCF_GPIO_PANPAR_AN0_AN0		// DS1 input pin
		| MCF_GPIO_PANPAR_AN1_AN1		// DS0 input pin
		| MCF_GPIO_PANPAR_AN2_GPIO		// FDD6 direction pin
		| MCF_GPIO_PANPAR_AN3_GPIO		// FDD6 step pin
		| MCF_GPIO_PANPAR_AN4_GPIO		// FDD5 direction pin
		| MCF_GPIO_PANPAR_AN5_GPIO		// FDD5 step pin
		| MCF_GPIO_PANPAR_AN6_GPIO		// FDD4 direction pin
		| MCF_GPIO_PANPAR_AN7_GPIO;		// FDD4 step pin

	// Set GPIO on AN to output mode
	MCF_GPIO_DDRAN = 0
		| MCF_GPIO_DDRAN_DDRAN2
		| MCF_GPIO_DDRAN_DDRAN3
		| MCF_GPIO_DDRAN_DDRAN4
		| MCF_GPIO_DDRAN_DDRAN5
		| MCF_GPIO_DDRAN_DDRAN6
		| MCF_GPIO_DDRAN_DDRAN7;


	// Enable RX on port UA for serial communication,
	// and enable GPIO on port UA for LEDs
    MCF_GPIO_PUAPAR = 0
    	| MCF_GPIO_PUAPAR_UTXD0_GPIO	// FDD3	LED
        | MCF_GPIO_PUAPAR_URXD0_URXD0
        | MCF_GPIO_PUAPAR_URTS0_GPIO	// FDD2 LED
        | MCF_GPIO_PUAPAR_UCTS0_GPIO;	// FDD1 LED
	
	// Set GPIO on UA to output mode
	MCF_GPIO_DDRUA = 0
		| MCF_GPIO_DDRUA_DDRUA0
		| MCF_GPIO_DDRUA_DDRUA2
		| MCF_GPIO_DDRUA_DDRUA3;
	
	// Enable IRQ signals on EPORT pin 7 (SW1), and
	// enable GPIO on port NQ for LEDs
	MCF_GPIO_PNQPAR = 0
	  | MCF_GPIO_PNQPAR_IRQ1_GPIO		// FDD0 LED
	  | MCF_GPIO_PNQPAR_IRQ4_GPIO		// FDD5 LED
	  | MCF_GPIO_PNQPAR_IRQ7_IRQ7;
	
	// Set GPIO on NQ to output mode
	MCF_GPIO_DDRNQ = 0
	  | MCF_GPIO_DDRNQ_DDRNQ1
	  | MCF_GPIO_DDRNQ_DDRNQ4;
	  

	// Set FDDn output pins to low
	for (i = 0; i < NUM_FLOPPIES; i++) 
	{
		*floppy_port[i] &= ~FLOPPY_DIRECTION_BIT[i];
		*floppy_port[i] &= ~FLOPPY_STEP_BIT[i];
	}
	
	
	// Turn off SSEGs and set state to blank
	SSEGOff(0);
	SSEGOff(1);
	setSSEG(0);
}

inline void initializeInterrupts() 
{
	// Enable interrupts of all priorities
	asm 
	{
		move.w 	SR, d0
		and		#0xfffff8ff, d0
		move.w	d0, SR
	}
	
	// Set interrupt level and priority of interrupt
	// source 55 (PIT0)
	MCF_INTC0_ICR55 = (0
		| MCF_INTC_ICR_IL(4)
		| MCF_INTC_ICR_IP(4));
		
	// Enable interrupts on source 55
	MCF_INTC0_IMRH &= ~MCF_INTC_IMRH_INT_MASK55;
	 
	
	// Enable interrupts from EPORT pin 7 (SW1)
	MCF_EPORT0_EPIER = 0
	  | MCF_EPORT_EPIER_EPIE7;

	// Set EPORT to look for rising edges
	MCF_EPORT0_EPPAR = 0
	  | MCF_EPORT_EPPAR_EPPA7_RISING;
	
	// Clear any existing interrupt flags from EPORT pin 7
 	MCF_EPORT0_EPFR = MCF_EPORT_EPFR_EPF7;
	 
	// Unmask interrupt in the interrupt controller
	MCF_INTC0_IMRL &= ~(0
	  | MCF_INTC_IMRL_INT_MASK7
	  | MCF_INTC_IMRL_MASKALL);
}

inline void initializePIT() 
{
	// Set prescaler
	// Enable overwrite of PIT counter on writing to PMR
	// Enable interrupts
	// Enable automatic reload of PIT counter when counter reaches 0
	MCF_PIT0_PCSR |= (0
		| MCF_PIT_PCSR_PRE(PIT0_PRESCALER)
		| MCF_PIT_PCSR_OVW
		| MCF_PIT_PCSR_PIE
		| MCF_PIT_PCSR_RLD);
	
	// Set interrupt frequency to INTERRUPT_FREQUENCY
	MCF_PIT0_PMR = getModulus(PIT0_PRESCALER, INTERRUPT_FREQUENCY);
}

inline void initializeADC() 
{
	// Clear stop bit, sync bit, interrupt bits,
	// and set once-sequential scanning mode
	// Scanning two samples in this mode takes
	// 1.7 + 1.2 = 3.9 microseconds.
	// (1st sample + 2nd sample)
	MCF_ADC_CTRL1 = 0;
	
	// Only scan the first two pins
	MCF_ADC_ADSDIS = MCF_ADC_ADSDIS_DS2;
	
	// Power up ADCs with a 0x0d ADC clock cycle delay.
	// Disable auto standby and auto power-down modes.
	// Keep power down control for voltage reference circuit.
    MCF_ADC_POWER = 0
    	| MCF_ADC_POWER_PD2
    	| MCF_ADC_POWER_PUDELAY(0x0d);
    
    // Wait for ADCs to power up.
    while (MCF_ADC_POWER & MCF_ADC_POWER_PSTS0)
    	;
    while (MCF_ADC_POWER & MCF_ADC_POWER_PSTS1)
    	;
}

// Returns the value to use in Pit Modulus Register
// given the prescaler value (0 - 7) and the desired
// interrupt frequency.
inline uint16 getModulus(uint16 prescaler, uint32 frequency) 
{
	const uint32 modulus = (30000000.0 / (frequency * (1 << prescaler)) + 0.5);
	assert(modulus > 0 && modulus < 65536);
	return (uint16) modulus;
}

inline void setFloppyPeriod(uint8 floppy, uint16 period) 
{
	if (floppy < NUM_FLOPPIES &&
		(period == 0 || period >= MIN_NOTE_PERIOD))
	{
		// Disable PIT0 interrupts, update step_period, and
		// restore interrupt mask register to previous state
		const uint32 mask = MCF_INTC0_IMRH;
		MCF_INTC0_IMRH |= MCF_INTC_IMRH_INT_MASK55;
		step_period[floppy] = period;
		MCF_INTC0_IMRH = mask;
		
		
		if (period == 0) 
		{
			floppyLEDOff(floppy);
		}
		else if
		{
			floppyLEDOn(floppy);
		}
	}
}

// If PIT is enabled, client code should use globals sseg0digit0
// and sseg0digit1 to set SSEG state. If PIT is disabled, client
// can use setSSEG, SSEGOn, and SSEGOff instead, but be aware
// that setSSEG sets state for both digits.

// state: xabcdefg
// x: nothing
// abcdefg: 0 = segment reset, 1 = segment set
inline void setSSEG(uint8 state) 
{
	// SSEG0 A
	// Clear old state for A
	MCF_GPIO_PORTQS &= ~MCF_GPIO_PORTQS_PORTQS0;
	
	// Set new state for A
	MCF_GPIO_PORTQS |= (0x40 & state) >> 6;
	
	// SSEG0 BC
	// Clear old state for BC
	MCF_GPIO_PORTTA &= ~(0
		| MCF_GPIO_PORTTA_PORTTA0
		| MCF_GPIO_PORTTA_PORTTA1);
		
	// Set new state for BC
	MCF_GPIO_PORTTA |= (0x30 & state) >> 4;
	
	// SSEG0 DEFG
	// Set new state for DEFG
	MCF_GPIO_PORTUB = (uint8) (0x0F & state);
}

inline void SSEGOn(uint8 digit) 
{
	switch (digit)
	{
	case 0:
		MCF_GPIO_PORTQS |= MCF_GPIO_PORTQS_PORTQS2;
		break;
	case 1:
		MCF_GPIO_PORTQS |= MCF_GPIO_PORTQS_PORTQS1;
		break;
	}
}

inline void SSEGOff(uint8 digit)
{
	switch (digit)
	{
	case 0:
		MCF_GPIO_PORTQS &= ~MCF_GPIO_PORTQS_PORTQS2;
		break;
	case 1:
		MCF_GPIO_PORTQS &= ~MCF_GPIO_PORTQS_PORTQS1;
		break;
	}
}

inline void floppyLEDOn(uint8 floppy)
{
	switch (floppy) 
	{
	case 0:
		MCF_GPIO_PORTNQ |= 0x02;
		break;
	case 1:
		MCF_GPIO_PORTUA |= 0x8;
		break;
	case 2:
		MCF_GPIO_PORTUA |= 0x2;
		break;
	case 3:
		MCF_GPIO_PORTUA |= 0x1;
		break;
	case 4:
		MCF_GPIO_PORTQS |= 0x8;
		break;
	case 5:
		MCF_GPIO_PORTNQ |= 0x10;
		break;
	default:
		break;
	}
}

inline void floppyLEDOff(uint8 floppy)
{
	switch (floppy) 
	{
	case 0:
		MCF_GPIO_PORTNQ &= ~0x02;
		break;
	case 1:
		MCF_GPIO_PORTUA &= ~0x8;
		break;
	case 2:
		MCF_GPIO_PORTUA &= ~0x2;
		break;
	case 3:
		MCF_GPIO_PORTUA &= ~0x1;
		break;
	case 4:
		MCF_GPIO_PORTQS &= ~0x8;
		break;
	case 5:
		MCF_GPIO_PORTNQ &= ~0x10;
		break;
	default:
		break;
	}
}

// Returns MIN_NOTE_INDEX for pitches lower than
// NOTE_PERIOD[MIN_NOTE_INDEX] and MAX_NOTE_INDEX
// for pitches higher than NOTE_PERIOD[MAX_NOTE_INDEX].
uint8 getNearestNoteIndex(uint16 period) 
{
	uint8 low = MIN_NOTE_INDEX;
	uint8 high = MAX_NOTE_INDEX;
	uint8 middle;
	
	if (period == 0) 
	{
		return 0;
	}
	
	while (low < high) 
	{
		middle = (uint8) ((low + high) / 2);
		
		if (period < NOTE_PERIOD[middle]) 
		{
			low = (uint8) (middle + 1);
		}
		else if (period > NOTE_PERIOD[middle]) 
		{
			high = (uint8) (middle - 1);
		}
		else if (period == NOTE_PERIOD[middle]) 
		{
			return middle;
		}
	}
	
	// Find out which note is closest, roughly,
	// since note frequency doesn't increase
	// linearly with perceived pitch.
	
	if (period < NOTE_PERIOD[low]) 
	{
		if (low == MAX_NOTE_INDEX) 
		{
			return MAX_NOTE_INDEX;
		}
		
		if (NOTE_PERIOD[low] - period < period - NOTE_PERIOD[low + 1]) 
		{
			return low;
		}
		
		return (uint8) (low + 1);
	}
	else
	{
		if (low == MIN_NOTE_INDEX) 
		{
			return MIN_NOTE_INDEX;
		}
		
		if (period - NOTE_PERIOD[low] < NOTE_PERIOD[low - 1] - period) 
		{
			return low;
		}

		return (uint8) (low - 1);
	}
}