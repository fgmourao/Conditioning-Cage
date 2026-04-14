/* Stimuli_PY_DUE.ino
 *
 * PROJECT : Conditioning Cage
 * Arduino DUE stimulus generator for classical fear conditioning.
 * Version: 2.0
 *
 * Architecture:
 *   Controlled entirely via the SerialUSB native port (Python on PC, 115200 baud).
 *   The DUE stores the complete trial list in RAM and executes the experiment
 *   autonomously after receiving a START command. Python manages timing only
 *   at the experiment level (sending parameters and start/abort); all stimulus
 *   onset/offset timing is handled by the DUE using micros() at 500 us resolution.
 *
 * Sound generation (Waveforms.h adaptive lookup tables):
 *   carrier_freq >  10000 Hz:  4-sample table  (DAC Timer4 rate up to  80 kHz)
 *   carrier_freq >   3000 Hz:  8-sample table  (DAC Timer4 rate up to  80 kHz)
 *   carrier_freq >   1000 Hz: 16-sample table  (DAC Timer4 rate up to  48 kHz)
 *   carrier_freq <=  1000 Hz: 32-sample table  (DAC Timer4 rate up to  32 kHz)
 *   All rates are within the SAM3X8E DAC hardware limit of 1 MHz.
 *   Modulator: fixed 32-sample unipolar [0,1] table. Timer5 rate = modulator_freq x 32.
 *   modulator_freq = 0: pure carrier sine, no AM modulation.
 *   Waveform types: SINE_AM (0), SINE (1), SQUARE (2).
 *
 * Three independent stimuli with individual onset/offset timing:
 *   SOUND -- DAC1, AM sine or pure sine or square wave, Timer4/Timer5
 *   SHOCK -- 8 bar pins round-robin, Timer6 at 10 kHz clock
 *   LIGHT -- pin 45, square wave 50% duty cycle, Timer7
 *
 * OLED status display (128x32, SSD1306, I2C address 0x3C):
 *   Wired to DUE I2C bus: SDA = pin 20, SCL = pin 21.
 *   Shows experiment state at each transition:
 *     Startup  : "Conditioning Cage / v2.0  Ready."
 *     Trial    : "TRIAL X/N / CS"  or  "CS + US"  or  "US"
 *     Done     : "Conditioning Cage / Done."
 *     Aborted  : "Conditioning Cage / Aborted."
 *   Updated in RunTrial() before Timer4 starts -- no audio glitch risk.
 *
 * SerialUSB protocol (115200 baud, newline-terminated JSON):
 *   {"cmd":"ping"}                              -> {"ok":true,"msg":"pong","version":"2.0"}
 *   {"cmd":"status"}                            -> {"ok":true,"status":N,"trial":N,...}
 *   {"cmd":"program","n":N,"data":"f0;f1;..."}  -> {"ok":true,"msg":"N trials programmed"}
 *   {"cmd":"start"}                             -> {"ok":true,"msg":"started"}
 *   {"cmd":"abort"}                             -> {"ok":true,"msg":"aborted"}
 *
 *   program data format: N x FIELDS_PER_TRIAL floats, semicolon-separated, row-major.
 *   Field order per trial: baseline, silence, onset_sound, sound_duration, carrier_freq,
 *   modulator_freq, volume, waveform_type, onset_shock, shock_duration, pulse_high,
 *   pulse_low, onset_light, light_duration, light_freq.
 *
 * Sync outputs (all active HIGH while the corresponding stimulus is active):
 *   pin 50 -- SOUND_SYN  sound active
 *   pin 51 -- LIGHT_SYN  light active
 *   pin 52 -- SHOCK_SYN  shock active
 *   pin 53 -- MOD_SYN    square wave at modulator_freq (phase-locked to AM envelope)
 *
 * Pin assignments:
 *   DAC1 -- sound output (12-bit, 0-3.3V)
 *   45   -- light stimulus output (square wave, 50% duty)
 *   46   -- watchdog fault output (HIGH pulse on fault)
 *   48   -- hardware ABORT input (INPUT_PULLUP, active LOW)
 *   13   -- debug LED (ON when parameters received OK, OFF after abort)
 *   20   -- I2C SDA (OLED display)
 *   21   -- I2C SCL (OLED display)
 *   23,25,27,29,31,33,35,37 -- shock bar outputs (active HIGH, round-robin)
 *
 * Timer allocation:
 *   Timer4 -- carrier()    ISR: writes DACbuffer samples to DAC1
 *   Timer5 -- modulating() ISR: advances modulator index, drives MOD_SYN
 *   Timer6 -- shockClock() ISR: 10 kHz shock bar timing clock
 *   Timer7 -- lightClock() ISR: light square wave toggle
 *
 * Based on Version 1.0 (2019): ESP8266 WiFi master + Arduino DUE SPI slave architecture.
 *   Authors: Paulo Aparecido Amaral Junior, Flavio Afonso Goncalves Mourao, Marcio Flavio Dutra Moraes
 *            Nucleo de Neurociencias UFMG/Brazil
 *            https://doi.org/10.3389/fnins.2019.01193
 *
 * Version 2.0 (2026)
 * Author:  Flavio Afonso Goncalves Mourao  mourao.fg@gmail.com
 */

#include "DueTimer.h"
#include "Waveforms.h"

/*############################################################################################################
        Constants
############################################################################################################*/

// Maximum number of trials stored in RAM.
// Increasing this also requires increasing sbuf (see sizing table below).
#define MAX_TRIALS          100

// Number of float fields per trial (must match Python FIELDS list).
#define FIELDS_PER_TRIAL    15

// SAM3X8E hardware DAC update rate limit: 1 MHz.
// Timer4 frequency = carrier_freq x nC must not exceed this value.
#define DAC_MAX_RATE        1000000.0f

// Timer6 runs at this fixed frequency to clock the shock bar state machine.
// Resolution: 1 / SHOCK_CLOCK_HZ = 0.1 ms per tick.
// pulse_high and pulse_low (ms) are converted to tick counts in ProgramShock().
#define SHOCK_CLOCK_HZ      10000.0f

// Watchdog: if Timer4 or Timer6 stops incrementing for this long, trigger a fault.
#define WATCHDOG_TIMEOUT_MS 5000UL

// Duration of the fault pulse on pinERROR (ms).
#define WATCHDOG_PULSE_MS   200UL

// Waveform type identifiers (stored as float in Trial struct, cast to int in ProgramSound).
#define WAVE_SINE_AM  0  // AM-modulated sine: carrier amplitude follows modulator envelope
#define WAVE_SINE     1  // Pure sine: no AM, modulator table ignored
#define WAVE_SQUARE   2  // Square wave: DAC alternates between 0 and 4095*vol

// Experiment status codes returned in JSON status responses.
#define ST_IDLE    0  // Waiting for parameters
#define ST_READY   1  // Parameters received, ready to start
#define ST_RUNNING 2  // Experiment in progress
#define ST_DONE    3  // All trials completed normally
#define ST_FAULT   4  // Watchdog fault detected, experiment aborted
#define ST_ABORTED 5  // Aborted by Python command or hardware button

/*############################################################################################################
        Pin assignments
############################################################################################################*/

// Hardware ABORT button: INPUT_PULLUP, active LOW.
// Connects between pin 48 and GND. Triggers immediate Abort() when pressed.
static const int pinABORT      = 48;
// Sync outputs: driven HIGH at stimulus onset, LOW at offset.
// Used for external recording synchronisation (e.g. electrophysiology, camera trigger).
static const int pinSOUND_SYN  = 50;
static const int pinSHOCK_SYN  = 52;
static const int pinLIGHT_SYN    = 51;
// Modulator sync: square wave at modulator_freq, phase-locked to the AM envelope.
// Transitions at iM==0 (LOW) and iM==MOD_SAMPLES_WF/2 (HIGH) inside modulating() ISR.
// Useful for locking external equipment to the AM cycle.
static const int pinMOD_SYN    = 53;
// Light stimulus output: square wave at light_freq, 50% duty cycle, driven by Timer7.
static const int pinLIGHT        = 45;
// Watchdog fault indicator: HIGH pulse of WATCHDOG_PULSE_MS on timer fault.
static const int pinERROR      = 46;
// Debug Light: ON when trial parameters are loaded, OFF after Abort().
static const int pinLIGHT_DBG    = 13;

// Shock bar pins: 8 pins starting at iInitialBar, spaced by barStep.
// Round-robin activation: one bar active at a time, advancing each pulse cycle.
static const int iInitialBar   = 23;
static const int nBars         = 8;
static const int barStep       = 2;
/*############################################################################################################
        Trial structure
        Each field is stored as float for uniform SerialUSBisation.
        Times are in seconds; frequencies in Hz; volume in %; pulse times in ms.
############################################################################################################*/

struct Trial {
  float baseline;        // Baseline period before first trial starts (s)
  float silence;         // Inter-trial silence before this trial starts (s)
  float onset_sound;     // Sound onset time within trial (s), relative to trial start
  float sound_duration;  // Sound duration (s)
  float carrier_freq;    // DAC carrier frequency (Hz); 0 = no sound
  float modulator_freq;  // AM modulator frequency (Hz); 0 = pure sine
  float volume;          // Sound amplitude (%), 0-100
  float waveform_type;   // 0=SINE_AM, 1=SINE, 2=SQUARE (cast to int in ProgramSound)
  float onset_shock;     // Shock onset time within trial (s)
  float shock_duration;  // Shock duration (s); 0 = no shock
  float pulse_high;      // Shock bar ON time per pulse (ms)
  float pulse_low;       // Shock bar OFF time between pulses (ms)
  float onset_light;     // Light onset time within trial (s)
  float light_duration;  // Light duration (s); 0 = no light
  float light_freq;      // Light square wave frequency (Hz); 9999 = DC HIGH (constant ON)
};

// Trial list stored in SRAM.
// Loaded by handleSerialUSB() on "program" command.
Trial   trials[MAX_TRIALS];
int     nTrials      = 0;     // Number of trials currently programmed
int     currentTrial = 0;     // Index of the trial currently executing (0-based)

// Experiment state flags.
bool    bReady       = false; // True when trial list is loaded and valid
bool    bRunning     = false; // True while RunExperiment() is executing
bool    bAbort       = false; // Set to true to request immediate stop
bool    bFault       = false; // Set to true on watchdog fault; cleared only by new program
uint8_t status       = ST_IDLE;
/*############################################################################################################
        Sound generation state
        DACbuffer is a 2-D lookup: DACbuffer[j + i*nC]
          i = modulator sample index (0 .. MOD_SAMPLES_WF-1)
          j = carrier sample index   (0 .. nC-1)
        Pre-computed by ProgramSound() for each trial.
        Written to DAC1 by carrier() ISR.
############################################################################################################*/

// Flat DACbuffer sized for the largest possible carrier table (32 samples x 32 mod samples).
// With uint16_t: 1024 entries x 2 bytes = 2 KB RAM.
uint16_t DACbuffer[MAX_CARRIER_SAMPLES * MOD_SAMPLES_WF];
// nC: current carrier table size, set by selectCarrier() inside ProgramSound().
// iC: current carrier sample index, incremented by carrier() ISR.
// iM: current modulator sample index, incremented by modulating() ISR.
int   nC = MAX_CARRIER_SAMPLES;
int   iC = 0;
int   iM = 0;
// Current sound parameters (copied from Trial struct by ProgramSound).
float carrier_freq   = 1000.0f;
float modulator_freq =   10.0f;
float volume         =  100.0f;
int   waveform_type  = WAVE_SINE_AM;

/*############################################################################################################
        Shock state
############################################################################################################*/

float            pulse_high         = 20.0f;  // Bar ON time (ms), from trial
float            pulse_low          = 20.0f;  // Bar OFF time (ms), from trial
int              iPulseHigh         = 0;       // pulse_high converted to Timer6 ticks
int              iPulseLow          = 0;       // pulse_low converted to Timer6 ticks
int              iBarPin            = iInitialBar; // Currently active bar pin

// iCountShock: tick counter incremented by shockClock() ISR at 10 kHz.
// bShockTick:  set by ISR each tick, consumed by bars() in the main loop.
// volatile: modified in ISR, read in main context.
volatile int32_t iCountShock        = 0;
volatile bool    bShockTick         = false;
bool             bShockActive       = false; // True while shock is being delivered
bool             bBarStatus         = false; // Current bar pin state (true=HIGH)

/*############################################################################################################
        Light state
############################################################################################################*/

// Current Light frequency.
// Set by ProgramLight(), used by lightClock() ISR to decide
// whether to toggle pinLIGHT when pinLIGHT_SYN is HIGH.
float light_freq_cur = 0.0f;

/*############################################################################################################
        Watchdog
        Monitors Timer4 (DAC clock) and Timer6 (shock clock) for stalls.
        Each ISR increments its counter every tick. WatchdogCheck() compares
        the current counter to a snapshot;
        if unchanged for WATCHDOG_TIMEOUT_MS,
        a hardware fault is declared and the experiment is aborted.
############################################################################################################*/

volatile uint32_t wdT4 = 0;    // Incremented by carrier()    ISR at dacRate Hz
volatile uint32_t wdT6 = 0;    // Incremented by shockClock() ISR at 10 kHz
volatile uint32_t wdT7 = 0;    // Incremented by lightClock() ISR (not monitored, reserved)

uint32_t wdT4L  = 0, wdT4Ms  = 0; // Last snapshot of wdT4 and its timestamp
uint32_t wdT6L  = 0, wdT6Ms  = 0; // Last snapshot of wdT6 and its timestamp
bool     bWD    = false;           // True while watchdog monitoring is active

/*############################################################################################################
        SerialUSB receive buffer
        handleSerialUSB() accumulates incoming bytes here until a newline is received,
        then dispatches the complete JSON command.

        Sizing guide (15 float fields per trial, ~75 bytes/trial in JSON):

          Trials   Buffer size   Command size (approx)
          -------  -----------   ---------------------
              10      1024 B           750 B
              20      2048 B          1475 B
              30      2048 B          2200 B
              40      4096 B          2925 B
              50      4096 B          3650 B
              60      4096 B          4375 B
              70      8192 B          5100 B
              80      8192 B          5825 B
              90      8192 B          6550 B
             100      8192 B          7275 B
             120     16384 B          8725 B
             150     16384 B         10900 B  (MAX_TRIALS limit)

        Rule: buffer must be >= command size. Use next power of 2.
        RAM cost: sbuf + trials[N]*60 bytes + fbuf[N*15]*4 bytes
          MAX_TRIALS=50  + sbuf=4096:  ~14 KB total
          MAX_TRIALS=100 + sbuf=8192:  ~21 KB total
          MAX_TRIALS=150 + sbuf=16384: ~37 KB total  (DUE has ~52 KB stack remaining)
############################################################################################################*/

static char  sbuf[8192]; // SerialUSB receive buffer (see sizing guide above)
static int   slen = 0;   // Current number of bytes accumulated in sbuf

/*############################################################################################################
        Function declarations
############################################################################################################*/

void AllBarsLow();
void bars(bool bStatus);
void waitMs(uint32_t ms);
void RunExperiment();
void RunTrial(int t);
void ProgramSound(Trial &t);
void ProgramShock(Trial &t);
void ProgramLight(Trial &t);
void Abort();
void WatchdogReset();
void WatchdogCheck();
void WatchdogFault();
void handleSerialUSB();
void sendOK(String msg);
void sendError(String msg);
void sendStatus();
float parseFloat(const char *buf, const char *key, float def);
bool  hasCmd(const char *buf, const char *cmd);
int   parseData(const char *data, float *arr, int maxn);

void carrier();
void modulating();
void shockClock();
void lightClock();
/*############################################################################################################
        AllBarsLow
        Drives all shock bar output pins LOW.
        Called at shock offset and in Abort().
############################################################################################################*/

void AllBarsLow()
{
  for (int i = 0; i < nBars; i++)
    digitalWrite(iInitialBar + i * barStep, LOW);
}

/*############################################################################################################
        waitMs
        Blocking wait with sub-millisecond resolution using micros().
        Processes SerialUSB, Watchdog, and shock bar state machine every 500 us
        so the DUE remains responsive to ABORT commands during inter-trial silences.
        Note: micros() overflows after ~71 minutes. Unsigned subtraction wraps
        correctly, so silences longer than 71 minutes are handled safely.
############################################################################################################*/

void waitMs(uint32_t ms)
{
  uint32_t t0     = micros();
  uint32_t waitUs = ms * 1000UL;
  while ((micros() - t0) < waitUs)
  {
    if (bAbort) return;
    handleSerialUSB();
    if (bWD) WatchdogCheck();
    if (bShockActive && bShockTick) { bShockTick = false; bars(bBarStatus); }
    delayMicroseconds(500);
  }
}

/*############################################################################################################
        Setup
############################################################################################################*/

void setup()
{
  // Programming port opens at 115200 baud.
  // The DTR signal from pySerialUSB triggers a hardware reset of the SAM3X8E.
  // The bootloader runs for ~8 seconds;
  delay(500); // provides stabilisation time
  // after setup() begins executing (Python waits 9 s total before communicating).
  SerialUSB.begin(115200);
  delay(500);
  pinMode(13, OUTPUT);

  // ABORT button: INPUT_PULLUP so the pin reads HIGH when unconnected.
  // Pressing the button connects pin 48 to GND, driving it LOW -> triggers Abort().
  pinMode(pinABORT,     INPUT_PULLUP);

  // All sync and stimulus outputs start LOW (safe state).
  pinMode(pinSOUND_SYN, OUTPUT); digitalWrite(pinSOUND_SYN, LOW);
  pinMode(pinSHOCK_SYN, OUTPUT); digitalWrite(pinSHOCK_SYN, LOW);
  pinMode(pinMOD_SYN,   OUTPUT); digitalWrite(pinMOD_SYN,   LOW);
  pinMode(pinLIGHT_SYN, OUTPUT); digitalWrite(pinLIGHT_SYN, LOW);
  pinMode(pinLIGHT,     OUTPUT); digitalWrite(pinLIGHT,     LOW);
  pinMode(pinERROR,     OUTPUT); digitalWrite(pinERROR,     LOW);
  pinMode(pinLIGHT_DBG, OUTPUT); digitalWrite(pinLIGHT_DBG, LOW);
  // Shock bars: all LOW at startup.
  for (int i = 0; i < nBars; i++) {
    pinMode(iInitialBar + i * barStep, OUTPUT);
    digitalWrite(iInitialBar + i * barStep, LOW);
  }

  // Attach Timer4 and Timer5 ISRs before ProgramSound() configures their frequencies.
  Timer4.attachInterrupt(carrier);
  Timer5.attachInterrupt(modulating);

  // DAC initialisation.
  // analogWrite(DAC0, 0) initialises the DACC hardware interface on the SAM3X8E.
  // analogWrite(DAC1, 0) selects and enables channel 1 (the physical DAC1 pin).
  // dacc_set_channel_selection() ensures subsequent dacc_write_conversion_data()
  // calls write to channel 1 and not channel 0.
  analogWriteResolution(12);
  analogWrite(DAC0, 0);
  analogWrite(DAC1, 0);
  dacc_set_channel_selection(DACC_INTERFACE, 1);
  dacc_disable_channel(DACC_INTERFACE, 1); // DAC1 disabled at startup -- re-enabled. Only at sound onset to prevent idle noise

  // Load default parameters and start shock/light timers.
  // Timer6 must be attached AFTER ProgramShock() (DueTimer library requirement).
  // Timer4/Timer5 are not started here;
  // they start at sound onset in RunTrial().
  {
    Trial def = {0};
    def.carrier_freq   = 1000.0f;
    def.modulator_freq =   10.0f;
    def.volume         =  100.0f;
    def.waveform_type  = WAVE_SINE_AM;
    def.pulse_high     =   20.0f;
    def.pulse_low      =   20.0f;
    def.light_freq     =   10.0f;
    ProgramSound(def);
    ProgramShock(def);
    Timer6.attachInterrupt(shockClock);
    Timer7.attachInterrupt(lightClock);
    ProgramLight(def);
  }

  status = ST_IDLE;
  SerialUSB.println("{\"ok\":true,\"msg\":\"Conditioning Cage v2.0 ready\"}");
}

/*############################################################################################################
        Loop
        Minimal main loop: checks hardware ABORT button, processes SerialUSB commands,
        and runs the software watchdog.
        RunExperiment() is blocking and called from
        handleSerialUSB() when a "start" command is received.
############################################################################################################*/

void loop()
{
  // Hardware ABORT: latch on first LOW edge to avoid calling Abort() repeatedly.
  static bool abortLatched = false;
  if (digitalRead(pinABORT) == LOW)
  {
    if (!abortLatched) { abortLatched = true;
      bAbort = true; Abort(); }
    return;
  }
  abortLatched = false;

  handleSerialUSB();

  if (bWD) WatchdogCheck();
}

/*############################################################################################################
        Timer ISRs
        All ISRs use direct register access (PIO_ODSR, PIO_SetOutput, dacc_write_conversion_data)
        instead of digitalWrite() to minimise ISR execution time and reduce jitter.
############################################################################################################*/

// carrier() -- called by Timer4 at carrier_freq x nC Hz.
// Writes the next DACbuffer sample to DAC1 only when pinSOUND_SYN is HIGH.
// PIO_PC13 is the register bitmask for pin 50 (pinSOUND_SYN) on Port C.
void carrier()
{
  if (PIOC->PIO_ODSR & PIO_PC13)
    dacc_write_conversion_data(DACC_INTERFACE, DACbuffer[iC + iM * nC]);
  if (++iC >= nC) iC = 0;
  wdT4++; // Watchdog counter: WatchdogCheck() verifies this increments during sound
}

// modulating() -- called by Timer5 at modulator_freq x MOD_SAMPLES_WF Hz.
// Advances the modulator index iM, which selects the DACbuffer row in carrier().
// Also drives pinMOD_SYN as a square wave phase-locked to the AM envelope:
//   LOW at the start of each modulator period (iM == 0)
//   HIGH at the midpoint (iM == MOD_SAMPLES_WF / 2)
void modulating()
{
  if (iM == 0 && (PIOC->PIO_ODSR & PIO_PC13))
    PIO_SetOutput(g_APinDescription[pinMOD_SYN].pPort,
                  g_APinDescription[pinMOD_SYN].ulPin, LOW, 0, PIO_DEFAULT);
  if (iM == (MOD_SAMPLES_WF / 2) && (PIOC->PIO_ODSR & PIO_PC13))
    PIO_SetOutput(g_APinDescription[pinMOD_SYN].pPort,
                  g_APinDescription[pinMOD_SYN].ulPin, HIGH, 0, PIO_DEFAULT);
  if (++iM >= MOD_SAMPLES_WF) iM = 0;
}

// shockClock() -- called by Timer6 at SHOCK_CLOCK_HZ (10 kHz).
// Increments the shock tick counter only when a shock is active.
// bars() in the main loop consumes bShockTick to advance the bar state machine.
void shockClock()
{
  if (bShockActive) { iCountShock++; bShockTick = true; }
  wdT6++; // Watchdog counter: WatchdogCheck() verifies Timer6 is running
}

// lightClock() -- called by Timer7 at light_freq x 2 Hz.
// Toggles pinLIGHT only when pinLIGHT_SYN is HIGH (Light stimulus active).
// The x2 multiplier in ProgramLight() produces a 50% duty cycle square wave.
void lightClock()
{
  if (digitalRead(pinLIGHT_SYN))
  {
    if (PIO_Get(g_APinDescription[pinLIGHT].pPort, PIO_OUTPUT_0,
                g_APinDescription[pinLIGHT].ulPin))
      PIO_SetOutput(g_APinDescription[pinLIGHT].pPort,
                    g_APinDescription[pinLIGHT].ulPin, LOW, 0, PIO_DEFAULT);
    else
      PIO_SetOutput(g_APinDescription[pinLIGHT].pPort,
                    g_APinDescription[pinLIGHT].ulPin, HIGH, 0, PIO_DEFAULT);
  }
  wdT7++;
}

/*############################################################################################################
        bars
        Shock bar state machine.
        Called from the main loop (and waitMs) whenever
        bShockTick is set by shockClock() ISR.
        State machine:
          bBarStatus == true  (bar currently HIGH):
            Wait until iCountShock >= iPulseHigh, then drive pin LOW.
          bBarStatus == false (bar currently LOW):
            If iCountShock < iPulseHigh: drive pin HIGH (start pulse).
          If iCountShock >= iPulseHigh + iPulseLow: reset counter, advance to next bar.

        The round-robin advances through pins 23,25,27,29,31,33,35,37 in sequence.
############################################################################################################*/

void bars(bool bSt)
{
  if (bSt)
  {
    if (iCountShock >= iPulseHigh)
    {
      PIO_SetOutput(g_APinDescription[iBarPin].pPort,
                    g_APinDescription[iBarPin].ulPin, LOW, 0, PIO_DEFAULT);
      bBarStatus = false;
    }
  }
  else
  {
    if (iCountShock < iPulseHigh)
    {
      PIO_SetOutput(g_APinDescription[iBarPin].pPort,
                    g_APinDescription[iBarPin].ulPin, HIGH, 0, PIO_DEFAULT);
      bBarStatus = true;
    }
    if (iCountShock >= (iPulseHigh + iPulseLow))
    {
      iCountShock = 0;
      iBarPin    += barStep;
      if (iBarPin >= (iInitialBar + nBars * barStep)) iBarPin = iInitialBar;
    }
  }
}

/*############################################################################################################
        ProgramSound
        Pre-computes DACbuffer for the given trial and sets Timer4/Timer5 frequencies.
        Timers are NOT started here -- they start at sound onset inside RunTrial().
        The DACbuffer is a 2-D array [MOD_SAMPLES_WF rows x nC columns].
        carrier() ISR reads along columns (fast axis);
        modulating() ISR advances rows.

        AM formula: sample = (1 + carrier[j] * modulator[i]) * 4095 * vol / 2
          Result range: 0 to 4095 (12-bit DAC full scale).
        At modulator[i]=1 (peak): full carrier amplitude.
          At modulator[i]=0 (trough): carrier fully suppressed (DC midpoint = 2047).
        If modulator_freq == 0 (bPureTone): all buffer rows are identical (pure carrier).
        Timer5 is not started in RunTrial() when modulator_freq == 0.
############################################################################################################*/

void ProgramSound(Trial &t)
{
  Timer4.stop();
  Timer5.stop();

  carrier_freq   = t.carrier_freq;
  modulator_freq = t.modulator_freq;
  volume         = t.volume;
  waveform_type  = (int)t.waveform_type;
  if (carrier_freq <= 0.0f) return; // No sound for this trial

  // selectCarrier() chooses the lookup table and sets nC (4, 8, 16, or 32 samples).
  const float *ct = selectCarrier(carrier_freq, &nC);
  iC = 0; iM = 0;
  float vol        = volume / 100.0f;
  bool  bPureTone  = (modulator_freq <= 0.0f);

  for (int i = 0; i < MOD_SAMPLES_WF; i++)
    for (int j = 0; j < nC; j++)
    {
      float sample;
      if (bPureTone)
      {
        sample = (1.0f + ct[j]) * 4095.0f * vol / 2.0f;
      }
      else
      {
        switch (waveform_type)
        {
          case WAVE_SINE_AM:
            sample = (1.0f + ct[j] * waveformsTableModulating[i]) * 4095.0f * vol / 2.0f;
            break;
          case WAVE_SINE:
            sample = (1.0f + ct[j]) * 4095.0f * vol / 2.0f;
            break;
          case WAVE_SQUARE:
            sample = (j < nC / 2) ?
              (4095.0f * vol) : 0.0f;
            break;
          default:
            sample = 0.0f;
        }
      }
      DACbuffer[j + i * nC] = (uint16_t)sample;
    }

  // Set Timer4 rate: carrier_freq x nC samples/period.
  // Capped at DAC_MAX_RATE (1 MHz) as a safety limit.
  float dacRate = carrier_freq * (float)nC;
  if (dacRate > DAC_MAX_RATE) dacRate = DAC_MAX_RATE;
  Timer4.setFrequency(dacRate);
  // Timer5 rate: modulator_freq x MOD_SAMPLES_WF steps/period.
  if (modulator_freq > 0.0f)
    Timer5.setFrequency(modulator_freq * (float)MOD_SAMPLES_WF);
}

/*############################################################################################################
        ProgramShock
        Converts pulse times from milliseconds to Timer6 tick counts and restarts Timer6.
        Safe to call while shock is not active; returns immediately if shock is running.
        Conversion: ticks = ms * (SHOCK_CLOCK_HZ / 1000)
        Example: 20 ms x 10 ticks/ms = 200 ticks per phase.
############################################################################################################*/

void ProgramShock(Trial &t)
{
  if (bShockActive) return; // Do not reconfigure while shock is delivering
  Timer6.stop();
  pulse_high   = t.pulse_high;
  pulse_low    = t.pulse_low;
  iPulseHigh   = (int)floorf(pulse_high * (SHOCK_CLOCK_HZ / 1000.0f));
  iPulseLow    = (int)floorf(pulse_low  * (SHOCK_CLOCK_HZ / 1000.0f));
  iCountShock  = 0;
  Timer6.setFrequency(SHOCK_CLOCK_HZ);
  Timer6.start();
}

/*############################################################################################################
        ProgramLight
        Sets Timer7 to toggle pinLIGHT at light_freq x 2 Hz (producing a 50% duty square wave).
        Timer7 is started immediately so the light is ready to respond when
        pinLIGHT_SYN goes HIGH at light onset in RunTrial().
############################################################################################################*/

void ProgramLight(Trial &t)
{
  Timer7.stop();
  digitalWrite(pinLIGHT, LOW);
  light_freq_cur = t.light_freq;
  if (light_freq_cur > 0.0f)
  {
    Timer7.setFrequency(light_freq_cur * 2.0f);
    // x2 for 50% duty toggle
    Timer7.start();
  }
}

/*############################################################################################################
        RunExperiment
        Iterates through all programmed trials sequentially.
        Blocking: returns only when all trials complete or bAbort is set.
        Sends a final status JSON when done.
        Timer6 is restarted after the experiment so the Watchdog keeps running
        in the idle state (wdT6 continues to increment, watchdog stays healthy).
############################################################################################################*/

void RunExperiment()
{
  bRunning = true;
  status   = ST_RUNNING;
  bAbort   = false;
  for (currentTrial = 0; currentTrial < nTrials; currentTrial++)
  {
    if (bAbort) break;
    RunTrial(currentTrial);
    if (bAbort) break;
  }

  // Safe state: stop sound timers, zero DAC, drive all sync pins LOW.
  Timer4.stop(); Timer5.stop();
  dacc_write_conversion_data(DACC_INTERFACE, 0);
  digitalWrite(pinSOUND_SYN, LOW);
  digitalWrite(pinSHOCK_SYN, LOW);
  digitalWrite(pinLIGHT_SYN, LOW);
  digitalWrite(pinLIGHT,     LOW);
  PIO_SetOutput(g_APinDescription[pinMOD_SYN].pPort,
                g_APinDescription[pinMOD_SYN].ulPin, LOW, 0, PIO_DEFAULT);
  AllBarsLow();
  bShockActive = false;
  bRunning     = false;
  status       = bAbort ? ST_ABORTED : ST_DONE;
  // Restart Timer6 so shockClock() ISR continues incrementing wdT6 during idle.
  Timer6.setFrequency(SHOCK_CLOCK_HZ);
  Timer6.start();

  sendStatus();
}

/*############################################################################################################
        RunTrial
        Executes one trial with independent onset/offset timing for each stimulus.
        Uses micros() for timing at ~500 us resolution (loop period).
        Timing model:
          t0 = micros() at trial start (after silence and ProgramX calls)
          elapsed = (micros() - t0 + 5) / 1e6  [seconds]
          +5 us compensates average loop overhead for more accurate onset timing.
        Each stimulus has a boolean pair (xOn, xOff) to track state.
        This ensures transitions happen exactly once even if the loop overshoots
        the target time.
        handleSerialUSB() is called every loop iteration so ABORT commands are
        processed with at most 500 us latency during an active trial.
############################################################################################################*/

void RunTrial(int t)
{
  Trial &tr = trials[t];

  // Baseline + inter-trial silence: waitMs() remains responsive to ABORT.
  float totalSilence = tr.baseline + tr.silence;
  if (totalSilence > 0.0f) waitMs((uint32_t)(totalSilence * 1000.0f));
  if (bAbort) return;
  // Program stimulus hardware for this trial before starting the timing loop.
  ProgramSound(tr);
  ProgramShock(tr);
  ProgramLight(tr);

  uint32_t t0 = micros();
  bool soundOn = false, soundOff = false;
  bool shockOn = false, shockOff = false;
  bool lightOn = false, lightOff   = false;

  // Pre-compute end times.
  float soundEnd = tr.onset_sound + tr.sound_duration;
  float shockEnd = tr.onset_shock + tr.shock_duration;
  float lightEnd = tr.onset_light + tr.light_duration;
  // Trial length = latest offset among active stimuli.
  float trLen = 0.0f;
  if (tr.sound_duration > 0.0f && soundEnd > trLen) trLen = soundEnd;
  if (tr.shock_duration > 0.0f && shockEnd > trLen) trLen = shockEnd;
  if (tr.light_duration > 0.0f && lightEnd > trLen) trLen = lightEnd;
  while (!bAbort)
  {
    // +5us compensates average loop overhead so onset timing is accurate.
    float elapsed = (micros() - t0 + 5) / 1000000.0f;
    // SOUND ON
    if (!soundOn && tr.sound_duration > 0.0f && elapsed >= tr.onset_sound)
    {
      soundOn = true;
      iC = 0; iM = 0;            // Reset table indices to ensure onset at zero-crossing

      dacc_enable_channel(DACC_INTERFACE, 1);
      dacc_write_conversion_data(DACC_INTERFACE, 2048);  // Pre-load midscale before enabling. Output to avoid onset transient
      delayMicroseconds(10);  // SAM3X8E DAC stabilisation time: output is not valid until ~10 us after channel enable. Inaudible delay.

      digitalWrite(pinSOUND_SYN, HIGH);
      Timer4.start();
      if (tr.modulator_freq > 0.0f) Timer5.start();
      WatchdogReset();
    }
    // SOUND OFF
    if (soundOn && !soundOff && elapsed >= soundEnd)
    {
      soundOff = true;
      digitalWrite(pinSOUND_SYN, LOW);
      dacc_disable_channel(DACC_INTERFACE, 1); // disable DAC1 immediately at sound offset to eliminate idle noise on speaker output

      Timer4.stop();
      Timer5.stop();
      dacc_write_conversion_data(DACC_INTERFACE, 0); // Zero DAC output
      iC = 0;
      iM = 0;
      PIO_SetOutput(g_APinDescription[pinMOD_SYN].pPort,
                    g_APinDescription[pinMOD_SYN].ulPin, LOW, 0, PIO_DEFAULT);
    }

    // SHOCK ON
    if (!shockOn && tr.shock_duration > 0.0f && elapsed >= tr.onset_shock)
    {
      shockOn          = true;
      iBarPin          = iInitialBar;
      iCountShock      = 0;
      bShockTick       = false;
      bBarStatus       = false;
      bShockActive     = true;
      digitalWrite(pinSHOCK_SYN, HIGH);
      WatchdogReset();
    }
    // SHOCK OFF
    if (shockOn && !shockOff && elapsed >= shockEnd)
    {
      shockOff     = true;
      bShockActive = false;
      digitalWrite(pinSHOCK_SYN, LOW);
      AllBarsLow();
      iBarPin    = iInitialBar;
      bBarStatus = false;
    }

    // Light ON
    if (!lightOn && tr.light_duration > 0.0f && tr.light_freq > 0.0f && elapsed >= tr.onset_light)
    {
      lightOn = true;
      digitalWrite(pinLIGHT_SYN, HIGH);
      if (tr.light_freq == 9999.0f)
        digitalWrite(pinLIGHT, HIGH);
        // DC high: light_freq=9999 is sentinel value -- Timer7 not started, pin stays HIGH
    }
    // Light OFF
    if (lightOn && !lightOff && elapsed >= lightEnd)
    {
      lightOff = true;
      digitalWrite(pinLIGHT_SYN, LOW);
      digitalWrite(pinLIGHT,     LOW);
    }

    // Service shock bar state machine.
    if (bShockActive && bShockTick) { bShockTick = false; bars(bBarStatus); }

    // Check for ABORT commands and watchdog.
    handleSerialUSB();
    if (bWD) WatchdogCheck();

    if (elapsed >= trLen) break;
    delayMicroseconds(500);
  }

  // Ensure all stimuli are off regardless of how the loop exited.
  if (!soundOff)
  {
    digitalWrite(pinSOUND_SYN, LOW);
    dacc_write_conversion_data(DACC_INTERFACE, 0);
    PIO_SetOutput(g_APinDescription[pinMOD_SYN].pPort,
                  g_APinDescription[pinMOD_SYN].ulPin, LOW, 0, PIO_DEFAULT);
  }
  if (!shockOff) { bShockActive = false; digitalWrite(pinSHOCK_SYN, LOW); AllBarsLow(); }
  if (!lightOff)   { digitalWrite(pinLIGHT_SYN, LOW);
    digitalWrite(pinLIGHT, LOW); }
}

/*############################################################################################################
        Abort
        Immediate stop: disables all timers, zeros DAC, drives all outputs LOW.
        Called from handleSerialUSB() on "abort" command, from loop() on hardware button,
        and from WatchdogFault().
        Timer6 is restarted so shockClock() keeps incrementing wdT6 during idle.
############################################################################################################*/

void Abort()
{
  bWD          = false;
  bShockActive = false;
  bRunning     = false;

  Timer4.stop(); Timer5.stop(); Timer6.stop(); Timer7.stop();

  dacc_write_conversion_data(DACC_INTERFACE, 0);
  digitalWrite(pinSOUND_SYN, LOW);
  dacc_disable_channel(DACC_INTERFACE, 1); // Disable DAC1 to prevent idle noise on speaker output

  digitalWrite(pinSHOCK_SYN, LOW);
  digitalWrite(pinLIGHT_SYN, LOW);
  digitalWrite(pinLIGHT,     LOW);
  digitalWrite(pinERROR,     LOW);
  PIO_SetOutput(g_APinDescription[pinMOD_SYN].pPort,
                g_APinDescription[pinMOD_SYN].ulPin, LOW, 0, PIO_DEFAULT);
  AllBarsLow();

  status = ST_ABORTED;

  // Restart Timer6 so the watchdog timer counter keeps running in idle state.
  Timer6.setFrequency(SHOCK_CLOCK_HZ);
  Timer6.start();
}

/*############################################################################################################
        Watchdog
        WatchdogReset(): called at sound or shock onset to activate monitoring.
        WatchdogCheck(): called from main loop and waitMs(). Compares current tick
          counters to snapshots;
          if unchanged for WATCHDOG_TIMEOUT_MS, calls WatchdogFault().
          Timer4 is only monitored while pinSOUND_SYN is HIGH (sound active).
          Timer6 is monitored continuously after WatchdogReset().
        WatchdogFault(): sets bFault, pulses pinERROR, calls Abort(), sends status.
############################################################################################################*/

void WatchdogReset()
{
  wdT4L  = wdT4; wdT6L  = wdT6;
  wdT4Ms = millis(); wdT6Ms = millis();
  bWD    = true;
}

void WatchdogCheck()
{
  uint32_t now = millis();
  // Monitor Timer4 (DAC clock) only while sound is active.
  if (digitalRead(pinSOUND_SYN) && carrier_freq > 0.0f)
  {
    if (wdT4 != wdT4L) { wdT4L = wdT4; wdT4Ms = now; }
    else if ((now - wdT4Ms) >= WATCHDOG_TIMEOUT_MS) { WatchdogFault(); return; }
  }
  else { wdT4L = wdT4; wdT4Ms = now; }

  // Monitor Timer6 (shock clock) continuously after watchdog activation.
  if (wdT6 != wdT6L) { wdT6L = wdT6; wdT6Ms = now; }
  else if ((now - wdT6Ms) >= WATCHDOG_TIMEOUT_MS) { WatchdogFault(); return; }
}

void WatchdogFault()
{
  bFault = true;
  status = ST_FAULT;
  digitalWrite(pinERROR, HIGH);
  delay(WATCHDOG_PULSE_MS);
  digitalWrite(pinERROR, LOW);
  Abort();
  sendStatus();
}

/*############################################################################################################
        SerialUSB protocol
        All responses are newline-terminated JSON objects.
############################################################################################################*/

// Send a success response: {"ok":true,"msg":"<msg>"}
void sendOK(String msg)
{
  SerialUSB.println("{\"ok\":true,\"msg\":\"" + msg + "\"}");
}

// Send an error response: {"ok":false,"msg":"<msg>"}
void sendError(String msg)
{
  SerialUSB.println("{\"ok\":false,\"msg\":\"" + msg + "\"}");
}

// Send full status JSON.
// Called automatically after RunExperiment() and WatchdogFault().
// Also called on demand by handleSerialUSB() for "status" commands.
void sendStatus()
{
  String s = "{\"ok\":true,\"status\":" + String(status);
  s += ",\"trial\":"   + String(currentTrial);
  s += ",\"total\":"   + String(nTrials);
  s += ",\"running\":" + String(bRunning ? "true" : "false");
  s += ",\"ready\":"   + String(bReady   ? "true" : "false");
  s += ",\"fault\":"   + String(bFault   ? "true" : "false");
  s += "}";
  SerialUSB.println(s);
}

// Extract a float value from a flat JSON string by key name.
// Searches for "key": and calls atof() on the following value.
// Returns def if the key is not found.
float parseFloat(const char *buf, const char *key, float def)
{
  char k[64];
  snprintf(k, sizeof(k), "\"%s\":", key);
  const char *p = strstr(buf, k);
  if (!p) return def;
  p += strlen(k);
  while (*p == ' ') p++;
  return (float)atof(p);
}

// Return true if the buffer contains "cmd":"<cmd>".
bool hasCmd(const char *buf, const char *cmd)
{
  char k[64];
  snprintf(k, sizeof(k), "\"cmd\":\"%s\"", cmd);
  return strstr(buf, k) != NULL;
}

// Parse semicolon-separated float values from a C string into arr[].
// Stops when the string ends, a non-numeric character terminates atof(),
// or maxn values have been read.
// Returns the number of values parsed.
int parseData(const char *data, float *arr, int maxn)
{
  int n = 0;
  const char *p = data;
  while (*p && n < maxn)
  {
    while (*p == ' ') p++;
    if (*p == '\0') break;
    arr[n++] = (float)atof(p);
    while (*p && *p != ';') p++;
    if (*p == ';') p++;
  }
  return n;
}

/*############################################################################################################
        handleSerialUSB
        Accumulates incoming bytes in sbuf until a newline (\n or \r) is received,
        then null-terminates and dispatches the complete JSON command.
        Called from loop() and from RunTrial()/waitMs() so ABORT is processed
        with at most 500 us latency at any point in the experiment.
############################################################################################################*/

void handleSerialUSB()
{
  while (SerialUSB.available())
  {
    char c = SerialUSB.read();
    if (c == '\n' || c == '\r')
    {
      if (slen == 0) continue; // Skip empty lines (e.g. \r\n pairs)
      sbuf[slen] = '\0';
      slen = 0;
      if (hasCmd(sbuf, "ping"))
      {
        SerialUSB.println("{\"ok\":true,\"msg\":\"pong\",\"version\":\"2.0\"}");
        continue;
      }

      if (hasCmd(sbuf, "status"))
      {
        sendStatus();
        continue;
      }

      if (hasCmd(sbuf, "abort"))
      {
        bAbort = true;
        Abort();
        sendOK("aborted");
        continue;
      }

      if (hasCmd(sbuf, "program"))
      {
        float nf = parseFloat(sbuf, "n", 0);
        int   n  = (int)nf;
        if (n <= 0 || n > MAX_TRIALS)
        {
          sendError("invalid trial count");
          continue;
        }

        // Locate the data string value (pointer to first character after "data":").
        const char *dp = strstr(sbuf, "\"data\":\"");
        if (!dp) { sendError("data field missing"); continue; }
        dp += 8;
        // fbuf is static to avoid stack allocation of up to 5600 bytes.
        static float fbuf[MAX_TRIALS * FIELDS_PER_TRIAL];
        int parsed = parseData(dp, fbuf, n * FIELDS_PER_TRIAL);

        if (parsed != n * FIELDS_PER_TRIAL)
        {
          sendError("field count mismatch");
          continue;
        }

        for (int t = 0; t < n; t++)
        {
          float *f = fbuf + t * FIELDS_PER_TRIAL;
          trials[t].baseline       = f[0];
          trials[t].silence        = f[1];
          trials[t].onset_sound    = f[2];
          trials[t].sound_duration = f[3];
          trials[t].carrier_freq   = f[4];
          trials[t].modulator_freq = f[5];
          trials[t].volume         = f[6];
          trials[t].waveform_type  = f[7];
          trials[t].onset_shock    = f[8];
          trials[t].shock_duration = f[9];
          trials[t].pulse_high     = f[10];
          trials[t].pulse_low      = f[11];
          trials[t].onset_light    = f[12];
          trials[t].light_duration = f[13];
          trials[t].light_freq     = f[14];
        }

        nTrials = n;
        bReady  = true;
        status  = ST_READY;
        digitalWrite(pinLIGHT_DBG, HIGH);
        sendOK(String(n) + " trials programmed");
        continue;
      }

      if (hasCmd(sbuf, "start"))
      {
        if (!bReady || bRunning)
        {
          sendError(bRunning ? "already running" : "no trials programmed");
          continue;
        }
        sendOK("started");
        delay(50);  // Allow USB transmission to complete before Timer4 starts.
                    // Prevents DAC glitch on first run caused by USB interrupt coinciding with sound onset.
                    // It only shifts the absolute start of the experiment by 50 ms relative to the START button press.

        RunExperiment(); // Blocking -- returns when all trials complete or bAbort is set
        continue;
      }

      sendError("unknown command");
    }
    else
    {
      // Accumulate byte; discard silently if buffer is full.
      if (slen < (int)sizeof(sbuf) - 1) sbuf[slen++] = c;
    }
  }
}
