#include <iostream>
#include "windows.h"
#include "jdkmidi/world.h"
#include "jdkmidi/track.h"
#include "jdkmidi/multitrack.h"
#include "jdkmidi/filereadmultitrack.h"
#include "jdkmidi/fileread.h"
#include "jdkmidi/fileshow.h"
#include "jdkmidi/sequencer.h"

using namespace jdkmidi;

const int MICROSECONDS_PER_MILLISECOND = 1000;
const int NUM_MIDI_CHANNELS = 16;
const int NUM_MIDI_NOTES = 128;

const int INTERRUPT_FREQUENCY = 50000;
const byte RESET_MESSAGE[] = { 128 };
const unsigned char MIN_NOTE = 0;
const unsigned char MAX_NOTE = 57;

// Number of timer interrupts between stepping FDD for a given
// MIDI note, given the interrupt frequency of the target platform
UINT16 notePeriods[NUM_MIDI_NOTES] = { 0 };

// The period of the note on a given channel, ignoring any
// active pitch bends.
UINT16 currentBasePeriod[NUM_MIDI_CHANNELS] = { 0 };

// The signed pitch bend value for a given channel (-8192 to 8191)
INT16 currentPitchBend[NUM_MIDI_CHANNELS] = { 0 };

BOOL WINAPI exitHandler(_In_ DWORD dwCtrlType);
void initializeNotePeriods(const unsigned char minNote,
        const unsigned char maxNote, const unsigned int interruptFrequency);
void initializeSerialPort(HANDLE& serial);
void initializeSleepResolution();
void delayTenthsOfMicroSeconds(__int64 tenthsOfMicroSeconds);
void sendMidiMessage(HANDLE& serial, MIDITimedBigMessage& msg);
void send(HANDLE& serial, const byte buffer[], unsigned int n);
unsigned char getNoteInAllowedRange(const unsigned char minNote,
        const unsigned char maxNote, const unsigned char note);

HANDLE serial;
bool serialOpened = false;
bool sleepResolutionInitialized = false;
bool programClosing = false;

int main(int argc, char **argv)
{
    if (argc > 1) {
        SetConsoleCtrlHandler(exitHandler, true);
        initializeNotePeriods(MIN_NOTE, MAX_NOTE, INTERRUPT_FREQUENCY);
        initializeSerialPort(serial);
        initializeSleepResolution();

        MIDIFileReadStreamFile rs(argv[1]);
        MIDIMultiTrack tracks(64);
        MIDIFileReadMultiTrack track_loader(&tracks);
        MIDIFileRead reader(&rs, &track_loader);
        MIDISequencer seq(&tracks);

        reader.Parse();

        std::cout << "Tracks: " << reader.GetNumberTracks() << std::endl;

        seq.GoToZero();

        float prevEventTimeMs = 0;
        float nextEventTimeMs;

        while (seq.GetNextEventTimeMs(&nextEventTimeMs) && !programClosing) {
            const float delayMs = nextEventTimeMs - prevEventTimeMs;

            if (delayMs > 0) {
                delayTenthsOfMicroSeconds((__int64) (10 * MICROSECONDS_PER_MILLISECOND * delayMs));
            }

            int tracknum;
            MIDITimedBigMessage msg;
            seq.GetNextEvent(&tracknum, &msg);

            sendMidiMessage(serial, msg);

            prevEventTimeMs = nextEventTimeMs;
        }
        
        // Restore Windows' timer state
        if (sleepResolutionInitialized) {
            TIMECAPS timerResolution;
            timeGetDevCaps(&timerResolution, sizeof(timerResolution));
            timeEndPeriod(timerResolution.wPeriodMin);
        }

        if (serialOpened) {
            // The program has been closed. Tell the microcontroller
            // to reset the floppy drives.

            // If song ended naturally, let a couple seconds
            // pass before resetting drives
            if (!programClosing) {
                Sleep(3000);
            }

            send(serial, RESET_MESSAGE, 1);
            Sleep(250);
            CloseHandle(serial);
        }
    }

    return 0;
}

BOOL WINAPI exitHandler(_In_ DWORD dwCtrlType) {
    programClosing = true;
    // Buy some time to let main thread close down safely.
    // May need to change delayTenthsOfMicroseconds() to
    // check programClosing every so often in order to break
    // out before program closes without clean up.
    Sleep(1000);
    return true;
}

void initializeNotePeriods(const unsigned char minNote,
        const unsigned char maxNote, const unsigned int interruptFrequency) {
    for (int note = minNote; note <= maxNote; note++) {
        double frequency = std::pow(2, (note - 69) / 12.0) * 440;
        notePeriods[note] = (int) ((interruptFrequency / frequency) + .5);
    }
}

void initializeSleepResolution() {
    TIMECAPS timerResolution;
    timeGetDevCaps(&timerResolution, sizeof(timerResolution));
    timeBeginPeriod(timerResolution.wPeriodMin);
    sleepResolutionInitialized = true;
}

void initializeSerialPort(HANDLE& serial) {
    serial = CreateFile("COM5", GENERIC_WRITE, 0, 0, OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, 0);

    if (serial == INVALID_HANDLE_VALUE) {
        std::cerr << "Couldn't open COM5. Exiting program.\n" <<
            "Press any key to continue.\n";
        getchar();
        exit(1);
    }

    DCB serialParameters = {0};
    serialParameters.DCBlength = sizeof(serialParameters);

    if (!GetCommState(serial, &serialParameters)) {
        std::cerr << "Error getting serial port parameters. " << 
            "Exiting program.\nPress any key to continue.\n";
        getchar();
        exit(1);
    }

    serialParameters.BaudRate = CBR_19200;
    serialParameters.ByteSize = 8;
    serialParameters.StopBits = ONESTOPBIT;
    serialParameters.Parity = NOPARITY;

    if (!SetCommState(serial, &serialParameters)) {
        std::cerr << "Error setting serial port parameters. " << 
            "Exiting program.\nPress any key to continue.\n";
        getchar();
        exit(1);
    }

    COMMTIMEOUTS timeOuts = {0};

    if (!SetCommTimeouts(serial, &timeOuts)) {
        std::cerr << "Error setting time out parameters. " << 
            "Exiting program.\nPress any key to continue.\n";
        getchar();
        exit(1);
    }

    serialOpened = true;
}

void delayTenthsOfMicroSeconds(__int64 tenthsOfMicroSeconds) {
    FILETIME startTime;
    GetSystemTimeAsFileTime((LPFILETIME) &startTime);

    __int64 nextEventTime =
        ((((__int64) startTime.dwHighDateTime) << 32) | startTime.dwLowDateTime) + 
        tenthsOfMicroSeconds;

    FILETIME currentTime;
    __int64 currentTimeForCompare;

    // Sleep for all but 1 millisecond in order to prevent some busy waiting.
    DWORD sleepTime = tenthsOfMicroSeconds / (10 * MICROSECONDS_PER_MILLISECOND) - 1;

    if (sleepTime > 0) {
        Sleep(sleepTime);
    }

    do {
        GetSystemTimeAsFileTime((LPFILETIME) &currentTime);
        currentTimeForCompare =
            (((__int64) currentTime.dwHighDateTime) << 32) | currentTime.dwLowDateTime;
    } while (currentTimeForCompare < nextEventTime);
}

void sendMidiMessage(HANDLE& serial, MIDITimedBigMessage& msg) {
    byte buffer[3];

    if (msg.IsNoteOff()) {
        buffer[0] = msg.GetChannel();
        buffer[2] = buffer[1] = 0;
        currentBasePeriod[buffer[0]] = 0;
        send(serial, buffer, 3);
    } else if (msg.IsNoteOn()) {
        buffer[0] = msg.GetChannel();
        const unsigned char note = getNoteInAllowedRange(MIN_NOTE, MAX_NOTE, msg.GetNote());

        if (msg.GetVelocity() == 0) {
            buffer[2] = buffer[1] = 0;
            currentBasePeriod[buffer[0]] = 0;
        } else {
            UINT16 period = notePeriods[note];

            // If there is a pitch bend active, apply it
            if (currentPitchBend[buffer[0]] != 0) {
                period = (UINT16) (period / (std::pow(2.0, (double) currentPitchBend[buffer[0]] / (4096 * 12))));
            }

            buffer[1] = (byte) ((period >> 8) & 0xFF);
            buffer[2] = (byte) (period & 0xFF);

            // currentBasePeriod is base period, not pitch-bent period
            currentBasePeriod[buffer[0]] = notePeriods[note];
        }

        send(serial, buffer, 3);
    } else if (msg.IsPitchBend()) {
        buffer[0] = msg.GetChannel();
        // http://dsp.stackexchange.com/questions/1645/converting-a-pitch-bend-midi-value-to-a-normal-pitch-value
        // msg.GetBenderValue() range is [-8192, 8191], not [0, 16384]

        // Save pitch bend value for current and future notes
        currentPitchBend[buffer[0]] = msg.GetBenderValue();

        // If currently playing a note, apply the pitch bend now
        if (currentBasePeriod[buffer[0]] != 0) {
            UINT16 period = (UINT16) (currentBasePeriod[buffer[0]] /
                (std::pow(2.0, (double) currentPitchBend[buffer[0]] / (4096 * 12))));

            buffer[1] = (byte) ((period >> 8) & 0xFF);
            buffer[2] = (byte) (period & 0xFF);
            send(serial, buffer, 3);
        }
    } else if (msg.IsAllNotesOff()) {
        for (int i = 0; i < NUM_MIDI_CHANNELS; i++) {
            buffer[0] = i;
            buffer[2] = buffer[1] = 0;
            currentBasePeriod[i] = 0;
            send(serial, buffer, 3);
        }
    }
}

void send(HANDLE& serial, const byte buffer[], unsigned int n) {
    DWORD bytesWritten;

    if (!WriteFile(serial, buffer, n, &bytesWritten, NULL)) {
        std::cerr << "Error writing to serial port.\n";
    }
}

unsigned char getNoteInAllowedRange(const unsigned char minNote,
        const unsigned char maxNote, const unsigned char note) {
    unsigned char newNote = note;

    if (note < minNote) {
        newNote += (int) std::ceil((minNote - note) / 12.0) * 12;
    } else if (note > maxNote) {
        newNote -= (int) std::ceil((note - maxNote) / 12.0) * 12;
    }

    if (newNote < minNote || maxNote < newNote) {
        newNote = note;
    }

    return newNote;
}