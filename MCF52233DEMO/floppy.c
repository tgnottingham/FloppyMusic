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

const float NOTE_FREQUENCY[] =
{
	0,
	8.1758, 8.66196, 9.17702, 9.72272, 10.3009, 10.9134, 11.5623, 12.2499, 12.9783, 13.75, 14.5676, 15.4339,
	16.3516, 17.3239, 18.354, 19.4454, 20.6017, 21.8268, 23.1247, 24.4997, 25.9565, 27.5, 29.1352, 30.8677,
	32.7032, 34.6478, 36.7081, 38.8909, 41.2034, 43.6535, 46.2493, 48.9994, 51.9131, 55, 58.2705, 61.7354,
	65.4064, 69.2957, 73.4162, 77.7817, 82.4069, 87.3071, 92.4986, 97.9989, 103.826, 110, 116.541, 123.471,
	130.813, 138.591, 146.832, 155.563, 164.814, 174.614, 184.997, 195.998, 207.652, 220, 233.082, 246.942,
	261.626, 277.183, 293.665, 311.127, 329.628, 349.228, 369.994, 391.995, 415.305, 440, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0,
};

const int CHROMATIC[] = 
{
	0,
	37, 38, 39, 40, 41, 42, 43, 44, 45, 46, 47, 48,
	49, 50, 51, 52, 53, 54, 55, 56, 57, 58, 59, 60,
	61,
};

const int IONIAN[] =
{
	0,
	37, 39, 41, 42, 44, 46, 48,
	49, 51, 53, 54, 56, 58, 60,
	61,
};

const int DORIAN[] = 
{
	0,
	37, 39, 40, 42, 44, 46, 47,
	49, 51, 52, 54, 56, 58, 59,
	61,
};

const int PHRYGIAN[] = 
{
	0,
	37, 38, 40, 42, 44, 45, 47,
	49, 50, 52, 54, 56, 57, 59,
	61,
};

const int LYDIAN[] = 
{
	0,
	37, 39, 41, 43, 44, 46, 48,
	49, 51, 53, 55, 56, 58, 60,
	61,
};

const int MIXOLYDIAN[] = 
{
	0,
	37, 39, 41, 42, 44, 46, 47,
	49, 51, 53, 54, 56, 58, 59,
	61,
};

const int AEOLIAN[] = 
{
	0,
	37, 39, 40, 42, 44, 45, 47,
	49, 51, 52, 54, 56, 57, 59,
	61,
};

const int LOCRIAN[] = 
{
	0,
	37, 38, 40, 42, 43, 45, 47,
	49, 50, 52, 54, 55, 57, 59,
	61,
};

const int BLUES[] = 
{
	0,
	37, 40, 42, 43, 44, 47,
	49, 52, 54, 55, 56, 59,
	61,
};

const int* SCALE[] =
{
	CHROMATIC, IONIAN, DORIAN, PHRYGIAN, LYDIAN,
	MIXOLYDIAN, AEOLIAN, LOCRIAN, BLUES,
};

const int SCALE_SIZE[] =
{
	26, 16, 16, 16, 16, 16, 16, 16, 14,
};


/////////////////////
// FLOPPY HANDLING //
/////////////////////

// The GPIO port for each floppy (see floppy.h).
vuint8* floppy_port[NUM_FLOPPIES] =
{
	FDD0, FDD1, FDD2, 
};

// The value to OR into a floppy's GPIO port
// in order to output high on the step pin.
const uint8 FLOPPY_STEP_BIT[NUM_FLOPPIES] = 
{
	FDD0_STEP, FDD1_STEP, FDD2_STEP, 
};

// The value to OR into a floppy's GPIO port
// in order to output high on the direction pin.
const uint8 FLOPPY_DIRECTION_BIT[NUM_FLOPPIES] = 
{
	FDD0_DIRECTION, FDD1_DIRECTION, FDD2_DIRECTION, 
};

// Number of interrupts between pulsing stepper motor
// while resetting floppy drive heads
const uint16 RESET_STEP_PERIOD = (int) (INTERRUPT_FREQUENCY / 240.0 + .5);

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


//////////
// CODE //
//////////

int main(void)
{
	int counter = 0;
	
	initializeFloppies();
	initializeGPIO();
	initializeInterrupts();
	initializePIT();
	initializeADC();
	//uart_init(0, SYSTEM_CLOCK_KHZ, kBaud19200);
	
	resetFloppies();
	
	// Enable PIT0 timer
	MCF_PIT0_PCSR |= MCF_PIT_PCSR_EN;
	
	//midiLoop();

    // start initial ADC conversion
    MCF_ADC_CTRL2 = MCF_ADC_CTRL2_START1;

	for (;;)
	{
        if ((counter++ & 0x01fffff) == 0)
        {
        	int adcValue;
        	int minNoteIndex = 44;
			int minDistance = 10;
			int maxDistance = 60;
			int cmPerNote = 10;
        	int cm;
        	
        	// DS0 -> Pin 18 -> AN4 -> ADRSLT0
            adcValue = MCF_ADC_ADRSLT7 >> 3;
			cm = 1.0 / ((adcValue - 2900) / 30000.0 + .1);
			//printf("ds0: %i\n", adcValue);
			
			if (cm >= minDistance && cm < maxDistance) 
			{
				int index = minNoteIndex +
					(cm - minDistance) / cmPerNote;
				//printf("note: %i\n", index);
				setFloppyFrequency(0, NOTE_FREQUENCY[index]);
				//setFloppyFrequency(1, NOTE_FREQUENCY[index]);
			}
			else 
        	{
        		setFloppyPeriod(0, 0);
        		//setFloppyPeriod(1, 0);
        	}
        	
        	// DS1 -> Pin 16 -> AN3 -> ADRSLT1
            adcValue = MCF_ADC_ADRSLT5 >> 3;
			cm = 1.0 / ((adcValue - 2900) / 30000.0 + .1);
        	//printf("ds1: %i\n", adcValue);
        	
			if (cm >= minDistance && cm < maxDistance) 
			{
				int index = minNoteIndex +
					(cm - minDistance) / cmPerNote;
					
				setFloppyFrequency(1, NOTE_FREQUENCY[index]);
			}
			else 
        	{
        		setFloppyPeriod(1, 0);
        	}
        	
            // start next ADC conversion
            MCF_ADC_CTRL2 = MCF_ADC_CTRL2_START1;
        }
	}
}

void midiLoop() 
{
	for (;;) 
	{
		unsigned char channel;
		channel = uart_getchar(0);
		
		if (channel == RESET_FLOPPIES_MESSAGE) 
		{
			resetFloppies();
		}
		else 
		{
			uint8 periodHighByte;
			uint8 periodLowByte;
			uint16 period;
			
			periodHighByte = uart_getchar(0);
			periodLowByte = uart_getchar(0);
			period = (uint16) (((periodHighByte & 0xFF) << 8) | (periodLowByte & 0xFF));

			setFloppyPeriod(channel, period);
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
	int i;
	for (i = 0; i < NUM_FLOPPIES; i++) 
	{
		step_period[i] = 0;
		count_until_step[i] = 0;
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
	
	
	// Enable GPIO on port TA for FDD2
	MCF_GPIO_PTAPAR = 0
		| MCF_GPIO_PTAPAR_GPT2_GPIO		// FDD2 direction pin
		| MCF_GPIO_PTAPAR_GPT3_GPIO;	// FDD2 step pin
		
	// Set GPIO on TA to output mode
	MCF_GPIO_DDRTA = 0
		| MCF_GPIO_DDRTA_DDRTA2
		| MCF_GPIO_DDRTA_DDRTA3;
	
	
	// TODO: enable GPIO for SSEGs
	
	
	// Enable RX on UART0
    MCF_GPIO_PUAPAR = 0
        | MCF_GPIO_PUAPAR_URXD0_URXD0;;
	
	
	// Enable ADC on port AN for DS0 and DS1
	MCF_GPIO_PANPAR = 0
		| MCF_GPIO_PANPAR_AN5_AN5		// DS1 input pin
		| MCF_GPIO_PANPAR_AN7_AN7;		// DS0 input pin


	// Set FDDn output pins to low.
	// TODO: set SSEGn output pins to low
	for (i = 0; i < NUM_FLOPPIES; i++) 
	{
		*floppy_port[i] = 0;
	}
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
	MCF_ADC_CTRL1 = 1;
	MCF_ADC_CTRL2 = 0x0002;
	
	// Enable scanning of distance sensor 0 (DS0) on
	// pin AN4 and distance sensor 1 (DS1) on pin AN3
	// RSLT0 will be scan result for DS0
	// RSLT1 will be scan result for DS1
	//MCF_ADC_ADLST1 = 0
	//	| MCF_ADC_ADLST1_SAMPLE0(0);
		
	MCF_ADC_ADLST2 = 0
		| MCF_ADC_ADLST2_SAMPLE5(5)
		| MCF_ADC_ADLST2_SAMPLE7(7);
	
	// Disable scanning for all other samples
	//MCF_ADC_ADSDIS = MCF_ADC_ADSDIS_DS2;
	
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

inline void setFloppyFrequency(uint16 floppy, float frequency)
{
	if (floppy < NUM_FLOPPIES) 
	{
		if (frequency != 0) 
		{
			// Disable PIT0 interrupts, update step_period, and
			// restore interrupt mask register to previous state
			const uint32 mask = MCF_INTC0_IMRH;
			MCF_INTC0_IMRH |= MCF_INTC_IMRH_INT_MASK55;
			step_period[floppy] = (uint16) (INTERRUPT_FREQUENCY / frequency + .5);
			count_until_step[floppy] = step_period[floppy];
			MCF_INTC0_IMRH = mask;
		}
		else 
		{
			const uint32 mask = MCF_INTC0_IMRH;
			MCF_INTC0_IMRH |= MCF_INTC_IMRH_INT_MASK55;
			step_period[floppy] = 0;
			count_until_step[floppy] = 0;
			MCF_INTC0_IMRH = mask;
		}
	}
}

inline void setFloppyPeriod(uint16 floppy, uint16 period) 
{
	if (floppy < NUM_FLOPPIES) 
	{
		const uint32 mask = MCF_INTC0_IMRH;
		MCF_INTC0_IMRH |= MCF_INTC_IMRH_INT_MASK55;
		step_period[floppy] = period;
		//count_until_step[floppy] = step_period[floppy];
		MCF_INTC0_IMRH = mask;
	}
}
