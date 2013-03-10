#ifndef _FLOPPY_H
#define _FLOPPY_H

#include "support_common.h"

#define PIT0_PRESCALER 0
// (50000 interrupts / sec. = 20 microsecond resolution)
#define INTERRUPT_FREQUENCY 50000
#define NUM_CHANNELS 16
#define NUM_FLOPPIES 8
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

// FDD3 = PORTAS
#define FDD3 ((vuint8*) (0x4010000B))
#define FDD3_DIRECTION (0x1)
#define FDD3_STEP (0x2)

// FDD4 = PORTAN
#define FDD4 ((vuint8*) (0x4010000A))
#define FDD4_DIRECTION (0x40)
#define FDD4_STEP (0x80)

// FDD5 = PORTAN
#define FDD5 ((vuint8*) (0x4010000A))
#define FDD5_DIRECTION (0x10)
#define FDD5_STEP (0x20)

// FDD6 = PORTAN
#define FDD6 ((vuint8*) (0x4010000A))
#define FDD6_DIRECTION (0x04)
#define FDD6_STEP (0x08)

// FDD7 = PORTAS
#define FDD7 ((vuint8*) (0x4010000B))
#define FDD7_DIRECTION (0x8)
#define FDD7_STEP (0x4)

void midiModeLoop();
void instrumentModeLoop();
__declspec(interrupt:0) void timerHandler(void);
__declspec(interrupt:0) void sw2Handler(void);
void resetFloppies();
void initializeFloppies();
void initializeGPIO();
void initializeInterrupts();
void initializePIT();
void initializeADC();
uint16 getModulus(uint16 prescaler, uint32 frequency);
void setFloppyPeriod(uint8 floppy, uint16 period);
void setSSEG(uint8 state);
void SSEGOn(uint8 digit);
void SSEGOff(uint8 digit);
uint8 getNearestNoteIndex(uint16 period);

#endif
