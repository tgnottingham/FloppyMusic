#ifndef _FLOPPY_H
#define _FLOPPY_H

#include "support_common.h"

#define PIT0_PRESCALER 0
// (50000 interrupts / sec. = 20 microsecond resolution)
#define INTERRUPT_FREQUENCY 50000
#define NUM_FLOPPIES 3
#define NUM_TRACKS 80
#define RESET_FLOPPIES_MESSAGE 128

// FDD0 = PORTTC
#define FDD0 ((vuint8*) (0x4010000F))
#define FDD0_DIRECTION (0x4)
#define FDD0_STEP (0x8)

// FDD1 = PORTTC
#define FDD1 ((vuint8*) (0x4010000F))
#define FDD1_DIRECTION (0x1)
#define FDD1_STEP (0x2)

// FDD2 = PORTTA
#define FDD2 ((vuint8*) (0x4010000E))
#define FDD2_DIRECTION (0x4)
#define FDD2_STEP (0x8)

void midiModeLoop();
void instrumentModeLoop();
__declspec(interrupt:0) void timerHandler(void);
void resetFloppies();
void initializeFloppies();
void initializeGPIO();
void initializeInterrupts();
void initializePIT();
void initializeADC();
uint16 getModulus(uint16 prescaler, uint32 frequency);
void setFloppyPeriod(uint16 floppy, uint16 period);
void setSSEG(uint8 state);

#endif
