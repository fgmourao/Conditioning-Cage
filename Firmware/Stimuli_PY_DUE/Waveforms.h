/*
 * Waveforms.h -- Pre-computed waveform lookup tables for DAC output.
 * Original v1.0 design (2019) used just one table with 8 samples
 * Why adaptive tables instead of a single fixed table?
 *
 * The DAC update rate is: Timer4_frequency = carrier_freq x nC
 * The SAM3X8E DAC hardware limit is 1 MHz.
 * Waveform quality is proportional to the number of samples per period (nC):
 * more samples = smoother sine = lower harmonic distortion.
 *
 * With a fixed 8-sample table (original v1.0 design):
 *   - DAC rate = carrier_freq x 8
 *   - Maximum carrier: 1,000,000 / 8 = 125,000 Hz  (sufficient)
 *   - At 1000 Hz: only 8 points per period -> visible distortion, metallic timbre
 *   - At 3000 Hz: only 8 points per period -> same problem
 *   Frequencies commonly used in fear conditioning (1-5 kHz) suffered most.
 *
 * With 4 adaptive tables (v2.0):
 *   carrier_freq <= 1000 Hz  ->  32 samples, DAC at  32 kHz  (best quality)
 *   carrier_freq <= 3000 Hz  ->  16 samples, DAC at  48 kHz  (good quality)
 *   carrier_freq <= 10000 Hz ->   8 samples, DAC at  80 kHz  (same as v1.x; rate ceiling lowered from 1 MHz to 80 kHz)
 *   carrier_freq <= 20000 Hz ->   4 samples, DAC at  80 kHz  (minimum viable)
 *
 *   The 4-sample table at 20 kHz produces a near-square wave, but at that
 *   frequency the perceptual difference is negligible.
 *
 *   The critical improvement is in the 1-5 kHz range (most used in conditioning
 *   protocols): sample count increases from 8 to 16-32, reducing harmonic
 *   distortion and producing a perceptually cleaner tone.
 *
 * RAM cost: max buffer = 32 carrier x 32 modulator = 1024 uint16_t = 2 KB
 *           vs sinf() runtime tables: 256 x 64 = 16384 uint16_t = 32 KB
 *           Savings: 30 KB -- critical for Hard Fault prevention on the DUE.
 *
 *
 * Adaptive carrier tables for 0-20 kHz carrier frequency range:
 *
 *   waveformsTableCarrier4[4]    carrier_freq > 10000 Hz (up to 20 kHz)
 *     DAC Timer4 rate = carrier_freq x 4  (max 80 kHz at 20 kHz carrier)
 *
 *   waveformsTableCarrier8[8]    carrier_freq >  3000 Hz (up to 10 kHz)
 *     DAC Timer4 rate = carrier_freq x 8  (max 80 kHz at 10 kHz carrier)
 *
 *   waveformsTableCarrier16[16]  carrier_freq >  1000 Hz (up to  3 kHz)
 *     DAC Timer4 rate = carrier_freq x 16 (max 48 kHz at  3 kHz carrier)
 *
 *   waveformsTableCarrier32[32]  carrier_freq <= 1000 Hz (best quality)
 *     DAC Timer4 rate = carrier_freq x 32 (max 32 kHz at  1 kHz carrier)
 *
 *   waveformsTableModulating[32] -- modulator, unipolar [0,1]
 *     Timer5 rate = modulator_freq x 32
 *
 * DACbuffer: MAX_CARRIER_SAMPLES x MOD_SAMPLES_WF = 32 x 32 = 1024 uint16_t = 2 KB RAM
 *
 * All carrier tables start at index 0 = 0.0 (zero-crossing).
 * Onset/offset at iC = 0 avoids audible clicks.
 *
 * Usage in ProgramSound():
 *   const float *ct = selectCarrier(carrier_freq, &nC);
 *   DACbuffer[j + i*nC] = (uint16_t)((1 + ct[j]*mod[i]) * 4095 * vol / 2)
 *   Timer4.setFrequency(carrier_freq * nC)
 *   Timer5.setFrequency(modulator_freq * 32)
 */

#ifndef _Waveforms_h_
#define _Waveforms_h_

#define MAX_CARRIER_SAMPLES  32   // largest table -- used for DACbuffer allocation
#define MOD_SAMPLES_WF       32   // modulator table size

static const float waveformsTableCarrier4[4] = {
  0.00000f, 1.00000f, 0.00000f, -1.00000f
};

static const float waveformsTableCarrier8[8] = {
  0.00000f, 0.70711f, 1.00000f, 0.70711f, 0.00000f, -0.70711f, -1.00000f, -0.70711f
};

static const float waveformsTableCarrier16[16] = {
  0.00000f, 0.38268f, 0.70711f, 0.92388f, 1.00000f, 0.92388f, 0.70711f, 0.38268f,
  0.00000f, -0.38268f, -0.70711f, -0.92388f, -1.00000f, -0.92388f, -0.70711f, -0.38268f
};

static const float waveformsTableCarrier32[32] = {
  0.00000f, 0.19509f, 0.38268f, 0.55557f, 0.70711f, 0.83147f, 0.92388f, 0.98079f,
  1.00000f, 0.98079f, 0.92388f, 0.83147f, 0.70711f, 0.55557f, 0.38268f, 0.19509f,
  0.00000f, -0.19509f, -0.38268f, -0.55557f, -0.70711f, -0.83147f, -0.92388f, -0.98079f,
  -1.00000f, -0.98079f, -0.92388f, -0.83147f, -0.70711f, -0.55557f, -0.38268f, -0.19509f
};

static const float waveformsTableModulating[32] = {
  0.00000f, 0.00961f, 0.03806f, 0.08427f, 0.14645f, 0.22221f, 0.30866f, 0.40245f,
  0.50000f, 0.59755f, 0.69134f, 0.77779f, 0.85355f, 0.91573f, 0.96194f, 0.99039f,
  1.00000f, 0.99039f, 0.96194f, 0.91573f, 0.85355f, 0.77779f, 0.69134f, 0.59755f,
  0.50000f, 0.40245f, 0.30866f, 0.22221f, 0.14645f, 0.08427f, 0.03806f, 0.00961f
};

// Returns pointer to the correct carrier table for the given frequency.
// Sets *nC to the number of samples in that table.
static inline const float* selectCarrier(float freq, int *nC) {
  if (freq > 10000.0f) { *nC = 4;  return waveformsTableCarrier4;  }
  if (freq >  3000.0f) { *nC = 8;  return waveformsTableCarrier8;  }
  if (freq >  1000.0f) { *nC = 16; return waveformsTableCarrier16; }
  *nC = 32; return waveformsTableCarrier32;
}

#endif // _Waveforms_h_
