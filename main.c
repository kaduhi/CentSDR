#include "ch.h"
#include "hal.h"
#include "usbcfg.h"

#include <chprintf.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <shell.h>

#include "nanosdr.h"
#include "si5351.h"

#include <stm32f303xc.h>

stat_t stat;

static void calc_stat(void);
static void measure_power_dbm(void);
static void measure_adc(void);

static THD_WORKING_AREA(waThread1, 128);
static __attribute__((noreturn)) THD_FUNCTION(Thread1, arg)
{
    (void)arg;
    int count;
    chRegSetThreadName("blink");
    while (1) {
      systime_t time = 100;
      chThdSleepMilliseconds(time);

      calc_stat();      
      measure_power_dbm();
      disp_update_power();

      if (++count == 10) {
        stat.fps = stat.fps_count;
        stat.fps_count = 0;
        stat.overflow = stat.overflow_count;
        stat.overflow_count = 0;
        count = 0;

        measure_adc();
        disp_update();
      }

#ifdef SI5351_GEN_QUADRATURE_LO_BELOW_3500KHZ
      si5351_adjust_rdiv_if_necessary();
#endif
    }
}

static void cmd_reset(BaseSequentialStream *chp, int argc, char *argv[])
{
    (void)argc;
    (void)argv;

    chprintf(chp, "Performing reset\r\n");

    //rccEnableWWDG(FALSE);

    WWDG->CFR = 0x60;
    WWDG->CR = 0xff;

    while (1)
	;
}

static const I2CConfig i2ccfg = {
  /*  31:28 PRESC     0000
      27:24 reserved  0000
      23:20 SCLDEL    1001
      19:16 SDADEL    0000
      15: 8 SCLH      00100000
       7: 0 SCLL      00100101
  */
  0x00902025, //voodoo magic
  //0x00420F13,  // 100kHz @ 72MHz
  0,
  0
};

static void cmd_freq(BaseSequentialStream *chp, int argc, char *argv[])
{
    int freq;
    if (argc != 1) {
        chprintf(chp, "usage: freq {frequency(Hz)}\r\n");
        return;
    }
    freq = atoi(argv[0]);
    si5351_set_frequency(freq);
}

static void cmd_tune(BaseSequentialStream *chp, int argc, char *argv[])
{
    int freq;
    if (argc != 1) {
        chprintf(chp, "usage: tune {frequency(Hz)}\r\n");
        return;
    }
    freq = atoi(argv[0]);
    set_tune(freq);

    uistat.freq = freq;
    uistat.mode = FREQ;
    disp_update();
}


int16_t rx_buffer[AUDIO_BUFFER_LEN * 2];
int16_t tx_buffer[AUDIO_BUFFER_LEN * 2];

#ifdef PORT_uSDX_TO_CentSDR
int16_t tx_amp_ph[AUDIO_BUFFER_LEN / 10][2];
#endif

const buffer_ref_t buffers_table[BUFFERS_MAX] = {
  { BT_C_INTERLEAVE, AUDIO_BUFFER_LEN, rx_buffer,  NULL },
  { BT_IQ,           AUDIO_BUFFER_LEN, buffer[0],  buffer[1] },
  { BT_IQ,           AUDIO_BUFFER_LEN, buffer2[0], buffer2[1] },
  { BT_R_INTERLEAVE, AUDIO_BUFFER_LEN, tx_buffer,  NULL }
};

const char *agcmode_table[] = {
  "manual", "slow", "mid", "fast"
};

signal_process_func_t signal_process = am_demod;
int16_t mode_freq_offset = AM_FREQ_OFFSET;
int16_t mode_freqoffset_phasestep;
int16_t cw_tone_phasestep = PHASESTEP(800);
int32_t center_frequency;

// restored from/into flash memory
config_t config = {
  .magic = CONFIG_MAGIC,
  .dac_value = 1080,
  .agc = {
    .target_level = 6,
    .maximum_gain = 127
  },
  .uistat = {
    .mode = CHANNEL,
	.channel = 0,
    .freq = 567000,
	.digit = 3,
	.modulation = MOD_AM,
	.volume = 0,
	.rfgain = 40, // 0 ~ 95
	//.agcmode = AGC_MANUAL,
    .agcmode = AGC_MID,
    .cw_tone_freq = 800
  },
  .channels = {
    /*    freq, modulation */
    {   567000, MOD_AM },
    {   747000, MOD_AM },
    {  1287000, MOD_AM },
    {  1440000, MOD_AM },
    {  7100000, MOD_LSB },
    { 14100000, MOD_USB },
    { 21100000, MOD_USB },
    { 26800200, MOD_FM_STEREO },
    { 27500200, MOD_FM_STEREO },
    { 28400200, MOD_FM_STEREO },
    {  2932000, MOD_USB },
    {  5628000, MOD_USB },
    {  6655000, MOD_USB },
    {  8951000, MOD_USB },
    { 10048000, MOD_USB },
    { 11330000, MOD_USB },
    { 13273000, MOD_USB },
    { 17904000, MOD_USB }
  },
  .button_polarity = 0x01,
  .freq_inverse = -1,
  .lcd_rotation = 0,
  .rotary_encoder_direction = -1
};

struct {
  signal_process_func_t demod_func;
  int16_t freq_offset;
  int16_t fs;
  const char *name;
} mod_table[] = {
  { cw_demod, AM_FREQ_OFFSET,  48, "cw" },
  { lsb_demod,             0,  48, "lsb" },
  { usb_demod,             0,  48, "usb" },
  { am_demod, AM_FREQ_OFFSET,  48, "am" },
  { fm_demod,              0, 192, "fm" },
  { fm_demod_stereo,       0, 192, "fms" },
  { iq_demod,              0,  48, "i/q" },
};

void set_modulation(modulation_t mod)
{
  if (mod >= MOD_MAX)
    return;

  uistat.fs = mod_table[mod].fs;
  set_fs(mod_table[mod].fs);
  signal_process = mod_table[mod].demod_func;

  mode_freq_offset = mod_table[mod].freq_offset;
  mode_freqoffset_phasestep = PHASESTEP(mode_freq_offset);
  cw_tone_phasestep = PHASESTEP(uistat.cw_tone_freq);
  
  uistat.modulation = mod;
  disp_update();
}

void
set_tune(int hz)
{
  center_frequency = hz - mode_freq_offset;
#ifdef SI5351_GEN_QUADRATURE_LO
  si5351_set_frequency(center_frequency);
#else
  si5351_set_frequency(center_frequency * 4);
#endif
}

static int current_fs = 48;

void
set_fs(int fs)
{
  if (fs != 48 && fs != 96 && fs != 192)
    return;

  if (fs != current_fs) {
    current_fs = fs;
    // stop WCLK,BCLK
    tlv320aic3204_stop();

    // then stop I2S
    i2sStopExchange(&I2SD2);
    // wait a second (not enough in 20ms)
    chThdSleepMilliseconds(40);
    // re-prepare I2S
    i2sStartExchange(&I2SD2);

    // enable WCLK,BCLK
    tlv320aic3204_set_fs(fs);
  }
}

void
update_cwtone(void)
{
    cw_tone_phasestep = PHASESTEP(uistat.cw_tone_freq);
}

void
update_iqbal(void)
{
    double value = config.freq_inverse - (double)uistat.iqbal / 10000.0;
    tlv320aic3204_config_adc_filter2(value);
}

void
update_agc(void)
{
  set_agc_mode(uistat.agcmode);
}

void
save_config_current_channel(void)
{
  int channel = uistat.channel;
  config.channels[channel].freq = uistat.freq;
  config.channels[channel].modulation = uistat.modulation;
  
  config.uistat = uistat;
  config_save();
}

#ifdef PORT_uSDX_TO_CentSDR

#define USE_QCX_SSB_R1_02J                1     // official release QCX-SSB-R1.02j https://github.com/threeme3/QCX-SSB/archive/R1.02j.zip
#define USE_FEATURE_RX_IMPROVED_BRANCH    0     // QCX-SSB/feature-rx-improved branch @ f0cc3d8f4af16aeeb94ba6293065a4c3aa290e3a


#if USE_QCX_SSB_R1_02J

// code from official release QCX-SSB-R1.02j https://github.com/threeme3/QCX-SSB/archive/R1.02j.zip

//  QCX-SSB.ino - https://github.com/threeme3/QCX-SSB
//
//  Copyright 2019, 2020   Guido PE1NNZ <pe1nnz@amsat.org>
//
//  Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions: The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software. THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

#ifdef PORT_uSDX_TO_CentSDR
enum mode_t { USDX_LSB, USDX_USB, USDX_CW, USDX_AM, USDX_FM };
volatile int8_t mode = USDX_LSB;
#else
enum mode_t { LSB, USB, CW, AM, FM };
volatile int8_t mode = USB;
#endif

static bool tx = false;
volatile bool vox = false;

inline void _vox(uint8_t trigger)
{
#ifndef PORT_uSDX_TO_CentSDR
  if(trigger){
    //if(!tx){ /* TX can be enabled here */ }
    tx = (vox) ? 255 : 1; // hangtime = 255 / 4402 = 58ms (the time that TX at least stays on when not triggered again)
  } else {
    if(tx){
      tx--;
      //if(!tx){ /* RX can be enabled here */ }
    }
  }
#endif
}

//#define F_SAMP_TX 4402
#ifdef PORT_uSDX_TO_CentSDR
#define F_SAMP_TX 4800        //4810 // ADC sample-rate; is best a multiple of _UA and fits exactly in OCR0A = ((F_CPU / 64) / F_SAMP_TX) - 1 , should not exceed CPU utilization
#else
#define F_SAMP_TX 4810        //4810 // ADC sample-rate; is best a multiple of _UA and fits exactly in OCR0A = ((F_CPU / 64) / F_SAMP_TX) - 1 , should not exceed CPU utilization
#endif
#define _UA  (F_SAMP_TX)      //360  // unit angle; integer representation of one full circle turn or 2pi radials or 360 degrees, should be a integer divider of F_SAMP_TX and maximized to have higest precision
//#define MAX_DP  (_UA/1)  //(_UA/2) // the occupied SSB bandwidth can be further reduced by restricting the maximum phase change (set MAX_DP to _UA/2).
//#define CONSTANT_AMP  1 // enable this in case there is no circuitry for controlling envelope (key shaping circuit)
//#define CARRIER_COMPLETELY_OFF_ON_LOW  1    // disable oscillator on no-envelope transitions, to prevent potential unwanted biasing/leakage through PA circuit
#define MULTI_ADC  1  // multiple ADC conversions for more sensitive (+12dB) microphone input

inline int16_t arctan3(int16_t q, int16_t i)  // error ~ 0.8 degree
{ // source: [1] http://www-labs.iro.umontreal.ca/~mignotte/IFT2425/Documents/EfficientApproximationArctgFunction.pdf
#define _atan2(z)  (_UA/8 - _UA/22 * z + _UA/22) * z  //derived from (5) [1]
  //#define _atan2(z)  (_UA/8 - _UA/24 * z + _UA/24) * z  //derived from (7) [1]
  int16_t r;
  if(abs(q) > abs(i))
    r = _UA / 4 - _atan2(abs(i) / abs(q));        // arctan(z) = 90-arctan(1/z)
  else
    r = (i == 0) ? 0 : _atan2(abs(q) / abs(i));   // arctan(z)
  r = (i < 0) ? _UA / 2 - r : r;                  // arctan(-z) = -arctan(z)
  return (q < 0) ? -r : r;                        // arctan(-z) = -arctan(z)
}

#define magn(i, q) (abs(i) > abs(q) ? abs(i) + abs(q) / 4 : abs(q) + abs(i) / 4) // approximation of: magnitude = sqrt(i*i + q*q); error 0.95dB

uint8_t lut[256];
volatile uint8_t amp;
volatile uint8_t vox_thresh = (1 << 2);
volatile uint8_t drive = 2;   // hmm.. drive>2 impacts cpu load..why?

inline int16_t ssb(int16_t in)
{
  static int16_t dc;

  int16_t i, q;
  uint8_t j;
  static int16_t v[16];

  for(j = 0; j != 15; j++) v[j] = v[j + 1];

  dc += (in - dc) / 2;
#ifdef PORT_uSDX_TO_CentSDR
  v[15] = in;          // no DC decoupling
#else
  v[15] = in - dc;     // DC decoupling
#endif
  //dc = in;  // this is actually creating a high-pass (emphasis) filter

  i = v[7];
  q = ((v[0] - v[14]) * 2 + (v[2] - v[12]) * 8 + (v[4] - v[10]) * 21 + (v[6] - v[8]) * 15) / 128 + (v[6] - v[8]) / 2; // Hilbert transform, 40dB side-band rejection in 400..1900Hz (@4kSPS) when used in image-rejection scenario; (Hilbert transform require 5 additional bits)

  uint16_t _amp = magn(i, q);
  if(vox) _vox(_amp > vox_thresh);
  //_amp = (_amp > vox_thresh) ? _amp : 0;   // vox_thresh = 1 is a good setting

  _amp = _amp << (drive);
#ifdef CONSTANT_AMP
  if(_amp < 4 ){ amp = 0; return 0; } //hack: for constant amplitude cases, set drive=1 for good results
  //digitalWrite(RX, (_amp < 4)); // fast on-off switching for constant amplitude case
#endif
  _amp = ((_amp > 255) || (drive == 8)) ? 255 : _amp; // clip or when drive=8 use max output
  amp = (tx) ? lut[_amp] : 0;

  static int16_t prev_phase;
  int16_t phase = arctan3(q, i);

  int16_t dp = phase - prev_phase;  // phase difference and restriction
  //dp = (amp) ? dp : 0;  // dp = 0 when amp = 0
  prev_phase = phase;

  if(dp < 0) dp = dp + _UA; // make negative phase shifts positive: prevents negative frequencies and will reduce spurs on other sideband
#ifdef MAX_DP
  if(dp > MAX_DP){ // dp should be less than half unit-angle in order to keep frequencies below F_SAMP_TX/2
    prev_phase = phase - (dp - MAX_DP);  // substract restdp
    dp = MAX_DP;
  }
#endif
#ifdef PORT_uSDX_TO_CentSDR
  if(mode == USDX_USB)
#else
  if(mode == USB)
#endif
    return dp * ( F_SAMP_TX / _UA); // calculate frequency-difference based on phase-difference
  else
    return dp * (-F_SAMP_TX / _UA);
}

#ifdef PORT_uSDX_TO_CentSDR
static uint8_t pwm_min = 0;    // PWM value for which PA reaches its minimum: 29 when C31 installed;   0 when C31 removed;   0 for biasing BS170 directly
static uint8_t pwm_max = 70;  // PWM value for which PA reaches its maximum: 96 when C31 installed; 255 when C31 removed; 220 for biasing BS170 directly
#else
static uint8_t pwm_min = 0;    // PWM value for which PA reaches its minimum: 29 when C31 installed;   0 when C31 removed;   0 for biasing BS170 directly
static uint8_t pwm_max = 220;  // PWM value for which PA reaches its maximum: 96 when C31 installed; 255 when C31 removed; 220 for biasing BS170 directly
#endif

void build_lut()
{
  // this code is from setup()
  for(uint16_t i = 0; i != 256; i++)    // refresh LUT based on pwm_min, pwm_max
    lut[i] = (float)i / ((float)255 / ((float)pwm_max - (float)pwm_min)) + pwm_min;
}

#elif USE_FEATURE_RX_IMPROVED_BRANCH

// code from QCX-SSB/feature-rx-improved branch @ f0cc3d8f4af16aeeb94ba6293065a4c3aa290e3a

//  QCX-SSB.ino - https://github.com/threeme3/QCX-SSB
//
//  Copyright 2019, 2020   Guido PE1NNZ <pe1nnz@amsat.org>
//
//  Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions: The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software. THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

#ifdef PORT_uSDX_TO_CentSDR
enum mode_t { USDX_LSB, USDX_USB, USDX_CW, USDX_FM, USDX_AM };
volatile uint8_t mode = USDX_LSB;
#else
enum mode_t { LSB, USB, CW, FM, AM };
volatile uint8_t mode = USB;
#endif

static bool tx = false;
volatile uint8_t vox = 0;

inline void _vox(uint8_t trigger)
{
#ifndef PORT_uSDX_TO_CentSDR
  if(trigger){
    tx = (tx) ? 254 : 255; // hangtime = 255 / 4402 = 58ms (the time that TX at least stays on when not triggered again). tx == 255 when triggered first, 254 follows for subsequent triggers, until tx is off.
  } else {
    if(tx) tx--;
  }
#endif
}

//#define F_SAMP_TX 4402
#ifdef PORT_uSDX_TO_CentSDR
#define F_SAMP_TX 4800        //4810 // ADC sample-rate; is best a multiple of _UA and fits exactly in OCR2A = ((F_CPU / 64) / F_SAMP_TX) - 1 , should not exceed CPU utilization
#else
#define F_SAMP_TX 4810        //4810 // ADC sample-rate; is best a multiple of _UA and fits exactly in OCR2A = ((F_CPU / 64) / F_SAMP_TX) - 1 , should not exceed CPU utilization
#endif
//#define MAX_DP  (_UA/4)  //(_UA/2) // the occupied SSB bandwidth can be further reduced by restricting the maximum phase change (set MAX_DP to _UA/2).
#define _UA  (F_SAMP_TX)      //360  // unit angle; integer representation of one full circle turn or 2pi radials or 360 degrees, should be a integer divider of F_SAMP_TX and maximized to have higest precision
#define CARRIER_COMPLETELY_OFF_ON_LOW  1    // disable oscillator on low amplitudes, to prevent potential unwanted biasing/leakage through PA circuit
#define MULTI_ADC  1  // multiple ADC conversions for more sensitive (+12dB) microphone input
//#define TX_CLK0_CLK1  1   // use CLK0, CLK1 for TX (instead of CLK2), you may enable and use NTX pin for enabling the TX path (this is like RX pin, except that RX may also be used as attenuator)
//#define QUAD  1       // invert TX signal for phase changes > 180

inline int16_t arctan3(int16_t q, int16_t i)  // error ~ 0.8 degree
{ // source: [1] http://www-labs.iro.umontreal.ca/~mignotte/IFT2425/Documents/EfficientApproximationArctgFunction.pdf
//#define _atan2(z)  (_UA/8  + _UA/22) * z  // very much of a simplification...not accurate at all, but fast
#define _atan2(z)  (_UA/8 - _UA/22 * z + _UA/22) * z  //derived from (5) [1]
  //#define _atan2(z)  (_UA/8 - _UA/24 * z + _UA/24) * z  //derived from (7) [1]
  int16_t r;
  if(abs(q) > abs(i))
    r = _UA / 4 - _atan2(abs(i) / abs(q));        // arctan(z) = 90-arctan(1/z)
  else
    r = (i == 0) ? 0 : _atan2(abs(q) / abs(i));   // arctan(z)
  r = (i < 0) ? _UA / 2 - r : r;                  // arctan(-z) = -arctan(z)
  return (q < 0) ? -r : r;                        // arctan(-z) = -arctan(z)
}

#define magn(i, q) (abs(i) > abs(q) ? abs(i) + abs(q) / 4 : abs(q) + abs(i) / 4) // approximation of: magnitude = sqrt(i*i + q*q); error 0.95dB

uint8_t lut[256];
volatile uint8_t amp;
volatile uint8_t vox_thresh = (1 << 0); //(1 << 2);
volatile uint8_t drive = 2;   // hmm.. drive>2 impacts cpu load..why?

volatile uint8_t quad = 0;

inline int16_t ssb(int16_t in)
{
  static int16_t dc, z1;

  int16_t i, q;
  uint8_t j;
  static int16_t v[16];

  for(j = 0; j != 15; j++) v[j] = v[j + 1];

  //dc += (in - dc) / 2;       // fast moving average
  dc = (in + dc) / 2;        // average
#ifdef PORT_uSDX_TO_CentSDR
  int16_t ac = (in - dc);    // DC decoupling
#else
  uint16_t ac = (in - dc);   // DC decoupling
#endif
#ifdef PORT_uSDX_TO_CentSDR
  v[15] = in;                // without DC decoupling
#else
  v[15] = ac;// - z1;        // high-pass (emphasis) filter
  //z1 = ac;
#endif

  i = v[7];
  q = ((v[0] - v[14]) * 2 + (v[2] - v[12]) * 8 + (v[4] - v[10]) * 21 + (v[6] - v[8]) * 15) / 128 + (v[6] - v[8]) / 2; // Hilbert transform, 40dB side-band rejection in 400..1900Hz (@4kSPS) when used in image-rejection scenario; (Hilbert transform require 5 additional bits)

  uint16_t _amp = magn(i, q);
#ifdef CARRIER_COMPLETELY_OFF_ON_LOW
  _vox(_amp > vox_thresh);
#else
  if(vox) _vox(_amp > vox_thresh);
#endif
  //_amp = (_amp > vox_thresh) ? _amp : 0;   // vox_thresh = 4 is a good setting
  //if(!(_amp > vox_thresh)) return 0;

  _amp = _amp << (drive);
  _amp = ((_amp > 255) || (drive == 8)) ? 255 : _amp; // clip or when drive=8 use max output
  amp = (tx) ? lut[_amp] : 0;

  static int16_t prev_phase;
  int16_t phase = arctan3(q, i);

  int16_t dp = phase - prev_phase;  // phase difference and restriction
  //dp = (amp) ? dp : 0;  // dp = 0 when amp = 0
  prev_phase = phase;

  if(dp < 0) dp = dp + _UA; // make negative phase shifts positive: prevents negative frequencies and will reduce spurs on other sideband
#ifdef QUAD
  if(dp >= (_UA/2)){ dp = dp - _UA/2; quad = !quad; }
#endif

#ifdef MAX_DP
  if(dp > MAX_DP){ // dp should be less than half unit-angle in order to keep frequencies below F_SAMP_TX/2
    prev_phase = phase - (dp - MAX_DP);  // substract restdp
    dp = MAX_DP;
  }
#endif
#ifdef PORT_uSDX_TO_CentSDR
  if(mode == USDX_USB)
#else
  if(mode == USB)
#endif
    return dp * ( F_SAMP_TX / _UA); // calculate frequency-difference based on phase-difference
  else
    return dp * (-F_SAMP_TX / _UA);
}

#ifdef PORT_uSDX_TO_CentSDR
static uint8_t pwm_min = 110;    // PWM value for which PA reaches its minimum: 29 when C31 installed;   0 when C31 removed;   0 for biasing BS170 directly
static uint8_t pwm_max = 175;  // PWM value for which PA reaches its maximum: 96 when C31 installed; 255 when C31 removed;
#else
static uint8_t pwm_min = 0;    // PWM value for which PA reaches its minimum: 29 when C31 installed;   0 when C31 removed;   0 for biasing BS170 directly
#ifdef QCX
static uint8_t pwm_max = 255;  // PWM value for which PA reaches its maximum: 96 when C31 installed; 255 when C31 removed;
#else
static uint8_t pwm_max = 128;  // PWM value for which PA reaches its maximum:                                              128 for biasing BS170 directly
#endif
#endif

//refresh LUT based on pwm_min, pwm_max
void build_lut()
{
  for(uint16_t i = 0; i != 256; i++)    // refresh LUT based on pwm_min, pwm_max
    lut[i] = (i * (pwm_max - pwm_min)) / 255 + pwm_min;
    //lut[i] = min(pwm_max, (float)106*log(i) + pwm_min);  // compressed microphone output: drive=0, pwm_min=115, pwm_max=220
}

#endif


static uint16_t s_tx_amp_ph_w_idx = 0;
static uint16_t s_tx_amp_ph_r_idx = 0;

#define SINE_WAVE_BUFFER_SIZE 360
/*
 * DAC test buffer (sine wave).
 */
static const int16_t k_sine_wave_buffer[SINE_WAVE_BUFFER_SIZE] = {
  2047, 2082, 2118, 2154, 2189, 2225, 2260, 2296, 2331, 2367, 2402, 2437,
  2472, 2507, 2542, 2576, 2611, 2645, 2679, 2713, 2747, 2780, 2813, 2846,
  2879, 2912, 2944, 2976, 3008, 3039, 3070, 3101, 3131, 3161, 3191, 3221,
  3250, 3278, 3307, 3335, 3362, 3389, 3416, 3443, 3468, 3494, 3519, 3544,
  3568, 3591, 3615, 3637, 3660, 3681, 3703, 3723, 3744, 3763, 3782, 3801,
  3819, 3837, 3854, 3870, 3886, 3902, 3917, 3931, 3944, 3958, 3970, 3982,
  3993, 4004, 4014, 4024, 4033, 4041, 4049, 4056, 4062, 4068, 4074, 4078,
  4082, 4086, 4089, 4091, 4092, 4093, 4094, 4093, 4092, 4091, 4089, 4086,
  4082, 4078, 4074, 4068, 4062, 4056, 4049, 4041, 4033, 4024, 4014, 4004,
  3993, 3982, 3970, 3958, 3944, 3931, 3917, 3902, 3886, 3870, 3854, 3837,
  3819, 3801, 3782, 3763, 3744, 3723, 3703, 3681, 3660, 3637, 3615, 3591,
  3568, 3544, 3519, 3494, 3468, 3443, 3416, 3389, 3362, 3335, 3307, 3278,
  3250, 3221, 3191, 3161, 3131, 3101, 3070, 3039, 3008, 2976, 2944, 2912,
  2879, 2846, 2813, 2780, 2747, 2713, 2679, 2645, 2611, 2576, 2542, 2507,
  2472, 2437, 2402, 2367, 2331, 2296, 2260, 2225, 2189, 2154, 2118, 2082,
  2047, 2012, 1976, 1940, 1905, 1869, 1834, 1798, 1763, 1727, 1692, 1657,
  1622, 1587, 1552, 1518, 1483, 1449, 1415, 1381, 1347, 1314, 1281, 1248,
  1215, 1182, 1150, 1118, 1086, 1055, 1024,  993,  963,  933,  903,  873,
   844,  816,  787,  759,  732,  705,  678,  651,  626,  600,  575,  550,
   526,  503,  479,  457,  434,  413,  391,  371,  350,  331,  312,  293,
   275,  257,  240,  224,  208,  192,  177,  163,  150,  136,  124,  112,
   101,   90,   80,   70,   61,   53,   45,   38,   32,   26,   20,   16,
    12,    8,    5,    3,    2,    1,    0,    1,    2,    3,    5,    8,
    12,   16,   20,   26,   32,   38,   45,   53,   61,   70,   80,   90,
   101,  112,  124,  136,  150,  163,  177,  192,  208,  224,  240,  257,
   275,  293,  312,  331,  350,  371,  391,  413,  434,  457,  479,  503,
   526,  550,  575,  600,  626,  651,  678,  705,  732,  759,  787,  816,
   844,  873,  903,  933,  963,  993, 1024, 1055, 1086, 1118, 1150, 1182,
  1215, 1248, 1281, 1314, 1347, 1381, 1415, 1449, 1483, 1518, 1552, 1587,
  1622, 1657, 1692, 1727, 1763, 1798, 1834, 1869, 1905, 1940, 1976, 2012
};

static uint16_t _sine_wave_delta = 6720; //6720:700Hz   360 / (4800 / freq) = 360 * freq / 4800, 7.5(100Hz) ~ 157.5(2100Hz), <<7 = 960 ~ 20160

static void _lowpass_1900hz_1(int16_t *in, int16_t *out, size_t n)
{
  static int16_t x1, x2, y1, y2;
  int16_t x0;
  int32_t accum;
  n /= 2;
  while (n--) {
    in++;
    x0 = *in++;
    accum  = (int32_t)x0 * 185;   // b0
    accum += (int32_t)x1 * 370;   // b1
    accum += (int32_t)x2 * 185;   // b2
    x2 = x1;
    x1 = x0;
    accum += (int32_t)y1 * 25875;   // a1
    accum += (int32_t)y2 * -10313;  // a2
    y2 = y1;
    y1 = accum >> 14;
    *out++ = y1;
    *out++ = y1;
  }
}

static void _lowpass_1900hz_2(int16_t *in, int16_t *out, size_t n)
{
  static int16_t x1, x2, y1, y2;
  int16_t x0;
  int32_t accum;
  n /= 2;
  while (n--) {
    in++;
    x0 = *in++;
    accum  = (int32_t)x0 * 128;
    accum += (int32_t)x1 * 256;
    accum += (int32_t)x2 * 128;
    x2 = x1;
    x1 = x0;
    accum += (int32_t)y1 * 29026;
    accum += (int32_t)y2 * -13563;
    y2 = y1;
    y1 = accum >> 14;
    *out++ = y1;
    *out++ = y1;
  }
}

static void usdx_tx_test(int16_t *rx_data, size_t n)
{
  // data format: | L-ch | R-ch | L-ch | ... |
  // R-ch is from Mic Input

  static uint8_t cnt10 = 0;
  static int32_t sum10 = 0;
  static int16_t ave10 = 0;

  uint16_t prev_w_idx = s_tx_amp_ph_w_idx;

  int16_t *p = rx_data;
  for (int i = 0; i < n; i += 2) {
    sum10 += p[1];
    if (++cnt10 >= 10) {
      cnt10 = 0;
      ave10 = sum10 / 10;
      sum10 = 0;

      ave10 = p[1];   // no averaging

      // int16_t in = ave10 >> 6;  // /= 64, in: -512 ~ +511
      int16_t in = ave10 >> 3;
      if (in < -512) in = -512;
      if (in > 511) in = 511;

#if 0   //KZT for testing with sine wave
      static uint16_t _sine_wave_index = 0;
      in = (k_sine_wave_buffer[(_sine_wave_index >> 7)] - 2047) >> 2; // -511 ~ +511
      // in >>= 1; // -255 ~ +255
      _sine_wave_index += _sine_wave_delta;
      if ((_sine_wave_index >> 7) >= SINE_WAVE_BUFFER_SIZE)
        _sine_wave_index -= SINE_WAVE_BUFFER_SIZE << 7;
      // static uint8_t _delta2 = 0;
      // if (++_delta2 >= 4) {
      //   _delta2 = 0;
      //   _sine_wave_delta++;
      //   if (_sine_wave_delta > 20160)
      //     _sine_wave_delta = 960;
      // }
#endif

      tx_amp_ph[s_tx_amp_ph_w_idx][1] = ssb(in);

      tx_amp_ph[s_tx_amp_ph_w_idx][0] = amp;
      //tx_amp_ph[s_tx_amp_ph_w_idx][0] = 255;    // always 100% amp

      if (++s_tx_amp_ph_w_idx >= (AUDIO_BUFFER_LEN / 10))
        s_tx_amp_ph_w_idx = 0;
    }

    //p[0] = p[1] = ave10;
    p += 2;
  }

  s_tx_amp_ph_r_idx = prev_w_idx;
}

#endif

void i2s_end_callback(I2SDriver *i2sp, size_t offset, size_t n)
{
  int32_t cnt_s = port_rt_get_counter_value();
  int32_t cnt_e;
  int16_t *p = &rx_buffer[offset];
  int16_t *q = &tx_buffer[offset];
  (void)i2sp;
  palSetPad(GPIOC, GPIOC_LED);

#ifdef PORT_uSDX_TO_CentSDR
  if (tx) {
    _lowpass_1900hz_1(p, q, n);
    _lowpass_1900hz_2(q, p, n);
    usdx_tx_test(p, n);
  }
#endif

  (*signal_process)(p, q, n);

  cnt_e = port_rt_get_counter_value();
  stat.interval_cycles = cnt_s - stat.last_counter_value;
  stat.busy_cycles = cnt_e - cnt_s;
  stat.last_counter_value = cnt_s;

  stat.callback_count++;
  palClearPad(GPIOC, GPIOC_LED);
}

static const I2SConfig i2sconfig = {
  tx_buffer, // TX Buffer
  rx_buffer, // RX Buffer
  AUDIO_BUFFER_LEN * 2,
  i2s_end_callback, // tx callback
  NULL, // rx callback
  0, // i2scfgr
  2 // i2spr
};

static void tone_generate(int freq)
{
    int i;
    for (i = 0; i < AUDIO_BUFFER_LEN; i++) {
      int16_t x = (int16_t)(sin(2*M_PI * i * freq / FS) * 10000);
      tx_buffer[i*2  ] = x;
      tx_buffer[i*2+1] = x;
    }
}

static void cmd_tone(BaseSequentialStream *chp, int argc, char *argv[])
{
    int freq = 440;
    if (argc > 1) {
        chprintf(chp, "usage: tone {audio frequency(Hz)}\r\n");
        return;
    } else if (argc == 1) {
      freq = atoi(argv[0]);
    }
    
    I2SD2.spi->I2SCFGR &= ~SPI_I2SCFGR_I2SE;
    //I2SD2.spi->CR2 = 0;
    tone_generate(freq);
    I2SD2.spi->I2SCFGR |= SPI_I2SCFGR_I2SE;
    //I2SD2.spi->CR2 = SPI_CR2_TXDMAEN;
}

static void cmd_data(BaseSequentialStream *chp, int argc, char *argv[])
{
  int i, j;
  (void)argc;
  (void)argv;
  int16_t *buf = rx_buffer;
  
  if (argc > 0) {
    switch (atoi(argv[0])) {
    case 0:
      break;
    case 1:
      buf = tx_buffer;
      break;
    case 2:
      buf = buffer[0];
      break;
    case 3:
      buf = buffer2[0];
      break;
    default:
      chprintf(chp, "unknown source\r\n");
      return;
    }
  }

  //i2sStopExchange(&I2SD2);
  for (i = 0; i < AUDIO_BUFFER_LEN; ) {
    for (j = 0; j < 16; j++, i++) {
      chprintf(chp, "%04x ", 0xffff & (int)buf[i]);
    }
    chprintf(chp, "\r\n");
  }
  //i2sStartExchange(&I2SD2);
}

static void
calc_stat(void)
{
  int16_t *p = &rx_buffer[0];
  int16_t min0 = 0, min1 = 0;
  int16_t max0 = 0, max1 = 0;
  int32_t count = AUDIO_BUFFER_LEN;
  int i;
  float accx0 = 0, accx1 = 0;
  for (i = 0; i < AUDIO_BUFFER_LEN*2; i += 2) {
    if (min0 > p[i]) min0 = p[i];
    if (min1 > p[i+1]) min1 = p[i+1];
    if (max0 < p[i]) max0 = p[i];
    if (max1 < p[i+1]) max1 = p[i+1];
    float x0 = p[i];
    float x1 = p[i+1];
    accx0 += x0 * x0;
    accx1 += x1 * x1;
  }
  stat.rms[0] = sqrtf(accx0 / count);
  stat.rms[1] = sqrtf(accx1 / count);
  stat.min[0] = min0;
  stat.min[1] = min1;
  stat.max[0] = max0;
  stat.max[1] = max1;
}  

int16_t measured_power_dbm;

static void
measure_power_dbm(void)
{
  extern int log2_q31(int32_t x);
  int agcgain = uistat.rfgain;
  if (uistat.agcmode != AGC_MANUAL)
    agcgain = tlv320aic3204_get_left_agc_gain();
  
  int dbm =                    // fixed point 8.8 format
    6 * log2_q31(stat.rms[0])  // 6dB/bit
    - (agcgain << 7);          // 0.5dB/agcgain
  dbm -= 116 << 8;
  measured_power_dbm = dbm;
}

uint16_t adc_single_read(ADC_TypeDef *adc, uint32_t chsel)
{
  /* ADC setup */
  adc->ISR    = adc->ISR;
  adc->IER    = 0;
  adc->SMPR1  = ADC_SMPR1_SMP0_2; // 19.5 cycle
  adc->CFGR   = 0; // 12bit
  adc->SQR1   = chsel << 6;

  /* ADC conversion start.*/
  adc->CR |= ADC_CR_ADSTART;
  while (adc->CR & ADC_CR_ADSTART)
    ;

  return adc->DR;
}

#define ADC1_CHANNEL_TEMP 16
#define ADC1_CHANNEL_BAT  17
#define ADC1_CHANNEL_VREF 18

static void measure_adc(void)
{
  stat.temperature = adc_single_read(ADC1, ADC1_CHANNEL_TEMP);
  stat.battery = adc_single_read(ADC1, ADC1_CHANNEL_BAT);
  stat.vref = adc_single_read(ADC1, ADC1_CHANNEL_VREF);
}

static void cmd_stat(BaseSequentialStream *chp, int argc, char *argv[])
{
  (void)argc;
  (void)argv;

  chprintf(chp, "average: %d %d\r\n", stat.ave[0], stat.ave[1]);
  chprintf(chp, "rms: %d %d\r\n", stat.rms[0], stat.rms[1]);
  chprintf(chp, "min: %d %d\r\n", stat.min[0], stat.min[1]);
  chprintf(chp, "max: %d %d\r\n", stat.max[0], stat.max[1]);
  chprintf(chp, "callback count: %d\r\n", stat.callback_count);
  chprintf(chp, "load: %d%% (%d/%d)\r\n", stat.busy_cycles * 100 / stat.interval_cycles, stat.busy_cycles, stat.interval_cycles);
  chprintf(chp, "fps: %d\r\n", stat.fps);
  chprintf(chp, "overflow: %d\r\n", stat.overflow);
  int gain0 = tlv320aic3204_get_left_agc_gain();
  int gain1 = tlv320aic3204_get_right_agc_gain();
  chprintf(chp, "agc gain: %d %d\r\n", gain0, gain1);

  chprintf(chp, "fm stereo: %d %d\r\n", stereo_separate_state.sdi, stereo_separate_state.sdq);
  chprintf(chp, "  corr: %d %d %d\r\n", stereo_separate_state.corr, stereo_separate_state.corr_ave, stereo_separate_state.corr_std);
  chprintf(chp, "  int: %d\r\n", stereo_separate_state.integrator);

  chprintf(chp, "temp: %d\r\n", adc_single_read(ADC1, ADC1_CHANNEL_TEMP));
  chprintf(chp, "bat: %d\r\n", adc_single_read(ADC1, ADC1_CHANNEL_BAT));
  chprintf(chp, "vref: %d\r\n", adc_single_read(ADC1, ADC1_CHANNEL_VREF));
  
#if 0
  p = &tx_buffer[0];
  acc0 = acc1 = 0;
  for (i = 0; i < AUDIO_BUFFER_LEN*2; i += 2) {
    acc0 += p[i];
    acc1 += p[i+1];
  }
  ave0 = acc0 / count;
  ave1 = acc1 / count;
  chprintf(chp, "audio average: %d %d\r\n", ave0, ave1);
#endif
}

static void cmd_power(BaseSequentialStream *chp, int argc, char *argv[])
{
  (void)argc;
  (void)argv;
  chprintf(chp, "power: %d.%01ddBm\r\n", measured_power_dbm >> 8,
           ((measured_power_dbm&0xff) * 10) >> 8);
}

static void cmd_impedance(BaseSequentialStream *chp, int argc, char *argv[])
{
    int imp;
    if (argc != 1) {
        chprintf(chp, "usage: imp {gain(1-3)}\r\n");
        return;
    }

    imp = atoi(argv[0]);
    tlv320aic3204_set_impedance(imp);
}

static void cmd_gain(BaseSequentialStream *chp, int argc, char *argv[])
{
    int gain;
    if (argc != 1 && argc != 2 && argc != 3) {
        chprintf(chp, "usage: gain {pga gain(0-95)} [digital gain(-24-40)] [adjust]\r\n");
        return;
    }

    gain = atoi(argv[0]);
    tlv320aic3204_set_gain(gain, gain);
    uistat.rfgain = gain;
    
    if (argc >= 2) {
      int adjust = 0;
      gain = atoi(argv[1]);
      if (argc == 3) {
        adjust = atoi(argv[2]);
      }
      tlv320aic3204_set_digital_gain(gain, gain + adjust);
      uistat.rfgain += gain;
    }

    disp_update();
}

static void cmd_phase(BaseSequentialStream *chp, int argc, char *argv[])
{
    int value;
    if (argc != 1) {
        chprintf(chp, "usage: phase {adjust value(-128-127)}\r\n");
        return;
    }

    value = atoi(argv[0]);
    tlv320aic3204_set_adc_phase_adjust(value);
}

static void cmd_finegain(BaseSequentialStream *chp, int argc, char *argv[])
{
    int g1 = 0, g2 = 0;
    if (argc != 1 && argc != 2) {
        chprintf(chp, "usage: gainadjust {gain1 gain2} (0 - -4)\r\n");
        return;
    }
    g1 = atoi(argv[0]);
    if (argc == 2) {
      g2 = atoi(argv[1]);
    }
    tlv320aic3204_set_adc_fine_gain_adjust(g1, g2);
}

static void cmd_iqbal(BaseSequentialStream *chp, int argc, char *argv[])
{
    if (argc != 1) {
        chprintf(chp, "usage: iqbal {coeff}\r\n");
        return;
    }
    uistat.iqbal = atoi(argv[0]);
    update_iqbal();
}

static void cmd_volume(BaseSequentialStream *chp, int argc, char *argv[])
{
    int gain;
    if (argc != 1) {
        chprintf(chp, "usage: volume {gain(-7-29)}\r\n");
        return;
    }

    gain = atoi(argv[0]);
    tlv320aic3204_set_volume(gain);
    uistat.volume = gain;
    disp_update();
}

static void cmd_dcreject(BaseSequentialStream *chp, int argc, char *argv[])
{
    int value;
    if (argc != 1) {
        chprintf(chp, "usage: dcreject {0|1}\r\n");
        return;
    }
    value = atoi(argv[0]);
    tlv320aic3204_config_adc_filter(value);
}

static void cmd_dac(BaseSequentialStream *chp, int argc, char *argv[])
{
    int value;
    if (argc != 1) {
        chprintf(chp, "usage: dac {value(0-4095)}\r\n");
        chprintf(chp, "current value: %d\r\n", config.dac_value);
        return;
    }
    value = atoi(argv[0]);
    config.dac_value = value;
    dacPutChannelX(&DACD1, 0, value);
}

static void cmd_agc(BaseSequentialStream *chp, int argc, char *argv[])
{
    const char *cmd;
    if (argc == 0) {
      chprintf(chp, "usage: agc {cmd} [args...]\r\n");
      chprintf(chp, "\tmanual/slow/mid/fast\r\n");
      chprintf(chp, "\tenable/disable\r\n");
      chprintf(chp, "\tlevel {0-7}\r\n");
      chprintf(chp, "\thysteresis {0-3}\r\n");
      chprintf(chp, "\tattack {0-31} [scale:0-7]\r\n");
      chprintf(chp, "\tdecay {0-31} [scale:0-7]\r\n");
      chprintf(chp, "\tmaxgain {0-116}\r\n");
      return;
    }

    cmd = argv[0];
    if (strncmp(cmd, "manual", 3) == 0) {
      set_agc_mode(AGC_MANUAL);
    } else if (strncmp(cmd, "slow", 2) == 0) {
      set_agc_mode(AGC_SLOW);
    } else if (strncmp(cmd, "mid", 2) == 0) {
      set_agc_mode(AGC_MID);
    } else if (strncmp(cmd, "fast", 2) == 0) {
      set_agc_mode(AGC_FAST);
    } else if (strncmp(cmd, "di", 2) == 0) {
      tlv320aic3204_agc_config(NULL);
    } else if (strncmp(cmd, "enable", 2) == 0) {
      tlv320aic3204_agc_config(&config.agc);
    } else if (strncmp(cmd, "le", 2) == 0 && argc == 2) {
      config.agc.target_level = atoi(argv[1]);
      tlv320aic3204_agc_config(&config.agc);
    } else if (strncmp(cmd, "hy", 2) == 0 && argc == 2) {
      config.agc.gain_hysteresis = atoi(argv[1]);
      tlv320aic3204_agc_config(&config.agc);
    } else if (strncmp(cmd, "at", 2) == 0 && argc >= 2) {
      config.agc.attack = atoi(argv[1]);
      if (argc >= 3)
        config.agc.attack_scale = atoi(argv[2]);
      tlv320aic3204_agc_config(&config.agc);
    } else if (strncmp(cmd, "de", 2) == 0 && argc >= 2) {
      config.agc.decay = atoi(argv[1]);
      if (argc >= 3)
        config.agc.decay_scale = atoi(argv[2]);
      tlv320aic3204_agc_config(&config.agc);
    } else if (strncmp(cmd, "max", 3) == 0 && argc >= 2) {
      config.agc.maximum_gain = atoi(argv[1]);
      tlv320aic3204_agc_config(&config.agc);
    }
}

void set_agc_mode(int mode)
{
  if (mode == AGC_MANUAL) {
    tlv320aic3204_agc_config(NULL);
    uistat.agcmode = mode;
    disp_update();
    return;
  }
  switch (mode) {
  case AGC_FAST:
    config.agc.decay = 0;
    config.agc.decay_scale = 0;
    break;
  case AGC_MID:
    config.agc.decay = 7;
    config.agc.decay_scale = 0;
    break;
  case AGC_SLOW:
    config.agc.decay = 31;
    config.agc.decay_scale = 4;
    break;
  }
  tlv320aic3204_agc_config(&config.agc);
  uistat.agcmode = mode;
  disp_update();
}

static void cmd_mode(BaseSequentialStream *chp, int argc, char *argv[])
{
    const char *cmd;
    if (argc == 0) {
      chprintf(chp, "usage: mode {lsb|usb|am|fm|fms}\r\n");
      return;
    }

    cmd = argv[0];
    if (strncmp(cmd, "am", 1) == 0) {
      set_modulation(MOD_AM);
    } else if (strncmp(cmd, "lsb", 1) == 0) {
      set_modulation(MOD_LSB);
    } else if (strncmp(cmd, "usb", 1) == 0) {
      set_modulation(MOD_USB);
    } else if (strncmp(cmd, "cw", 1) == 0) {
      set_modulation(MOD_CW);
    } else if (strncmp(cmd, "fms", 3) == 0) {
      set_modulation(MOD_FM_STEREO);
    } else if (strncmp(cmd, "fm", 1) == 0) {
      set_modulation(MOD_FM);
    }
}

static void cmd_cwtone(BaseSequentialStream *chp, int argc, char *argv[])
{
    int freq = 0;
    if (argc == 0) {
        chprintf(chp, "%d\r\n", uistat.cw_tone_freq);
        return;
    }

    if (argc == 1)
        freq = atoi(argv[0]);

    if (freq == 0) {
        chprintf(chp, "usage: cwtone {audio frequency(Hz)}\r\n");
        return;
    }
    uistat.cw_tone_freq = freq;
    update_cwtone();
}

static void cmd_fs(BaseSequentialStream *chp, int argc, char *argv[])
{
  int fs = 0;
  
  if (argc == 1) {
    fs = atoi(argv[0]);
  }

  if (fs == 48 || fs == 96 || fs == 192) {
    set_fs(fs);
    uistat.fs = fs;
  } else {
    chprintf(chp, "usage: fs {48|96|192}\r\n");
  }
}

static void cmd_winfunc(BaseSequentialStream *chp, int argc, char *argv[])
{
    int type;
    if (argc == 0) {
      chprintf(chp, "usage: winfunc {0|1|2}\r\n");
      return;
    }

    type = atoi(argv[0]);
    set_window_function(type);
}

static void cmd_show(BaseSequentialStream *chp, int argc, char *argv[])
{
  if (argc == 0 || strcmp(argv[0], "all") == 0) {
    chprintf(chp, "tune: %d\r\n", uistat.freq);
    chprintf(chp, "volume: %d\r\n", uistat.volume);
    chprintf(chp, "mode: %s\r\n", mod_table[uistat.modulation].name);
    chprintf(chp, "gain: %d\r\n", uistat.rfgain);
    chprintf(chp, "channel: %d\r\n", uistat.channel);
    chprintf(chp, "agc: %s\r\n", agcmode_table[uistat.agcmode]);
  } else if (strcmp(argv[0], "tune") == 0) {
    chprintf(chp, "%d\r\n", uistat.freq);
  } else if (strcmp(argv[0], "volume") == 0) {
    chprintf(chp, "%d\r\n", uistat.volume);
  } else if (strcmp(argv[0], "mode") == 0) {
    chprintf(chp, "%s\r\n", mod_table[uistat.modulation].name);
  } else if (strcmp(argv[0], "gain") == 0) {
    chprintf(chp, "%d\r\n", uistat.rfgain);
  } else if (strcmp(argv[0], "channel") == 0) {
    chprintf(chp, "%d\r\n", uistat.channel);
  } else if (strcmp(argv[0], "agc") == 0) {
    chprintf(chp, "agc: %s\r\n", agcmode_table[uistat.agcmode]);
  } else {
    chprintf(chp, "usage: show [all|tune|volume|gain|agc]\r\n");
    return;
  }
}

static void cmd_channel(BaseSequentialStream *chp, int argc, char *argv[])
{
    if (argc == 0) {
      chprintf(chp, "usage: channel [save|del|list] [n(0-99)]\r\n");
      return;
    }

    int channel;
    if (strncmp(argv[0], "save", 1) == 0) {
      channel = uistat.channel;
      if (argc >= 2) {
        channel = atoi(argv[1]);
        if (channel < 0 || channel >= CHANNEL_MAX) {
          chprintf(chp, "specified channel is out of range\r\n");
          return;
        }
      } else {
        chprintf(chp, "channel saved on %d\r\n", channel);
      }
      config.channels[channel].freq = uistat.freq;
      config.channels[channel].modulation = uistat.modulation;
    } else if (strncmp(argv[0], "del", 1) == 0) {
      channel = uistat.channel;
      if (argc >= 2) {
        channel = atoi(argv[1]);
        if (channel < 0 || channel >= CHANNEL_MAX) {
          chprintf(chp, "specified channel is out of range\r\n");
          return;
        }
      } else {
        chprintf(chp, "channel %d deleted\r\n", channel);
      }
      config.channels[channel].freq = 0;
    } else if (strncmp(argv[0], "list", 1) == 0) {
      for (channel = 0; channel < CHANNEL_MAX; channel++) {
        if (config.channels[channel].freq) {
          chprintf(chp, "%d %d %s\r\n", channel,
                   config.channels[channel].freq,
                   mod_table[config.channels[channel].modulation].name);
        }
      }
    } else {
      channel = atoi(argv[0]);
      if (channel < 0 || channel >= CHANNEL_MAX) {
        chprintf(chp, "specified channel is out of range\r\n");
        return;
      }
      recall_channel(channel);
      uistat.mode = CHANNEL;
      uistat.channel = channel;
      disp_update();
    }
}

static void cmd_copy_channels(BaseSequentialStream *chp, int argc, char *argv[])
{
  if (argc < 3) {
    chprintf(chp, "usage: copych [src_start_ch] [src_end_ch] [dst_ch]\r\n");
    return;
  }

  int src_ch = atoi(argv[0]);
  int channels = atoi(argv[1]) - src_ch + 1;
  int dst_ch = atoi(argv[2]);

  if (src_ch < 0 || CHANNEL_MAX <= src_ch ||
      dst_ch < 0 || CHANNEL_MAX <= dst_ch ||
    channels < 1
  ) {
    chprintf(chp, "error: parameter out of range\r\n");
    return;
  }

  if (CHANNEL_MAX <= (src_ch + channels)) {
    channels = CHANNEL_MAX - src_ch;
  }
  if (CHANNEL_MAX <= (dst_ch + channels)) {
    channels = CHANNEL_MAX - dst_ch;
  }

  channel_t *src = &config.channels[src_ch];
  channel_t *dst = &config.channels[dst_ch];
  memmove(dst, src, channels * sizeof(channel_t));

  int channel = uistat.channel;
  recall_channel(channel);
  uistat.mode = CHANNEL;
  disp_update();
}

static void cmd_revision(BaseSequentialStream *chp, int argc, char *argv[])
{
  if (argc == 0) {
    chprintf(chp, "usage: revision {rev}\r\n");
    return;
  }
  int rev = atoi(argv[0]);
  switch (rev) {
  case 0:
    config.freq_inverse = 1;
    config.button_polarity = 0x00;
    config.rotary_encoder_direction = 1;
    break;
  case 1:
    config.freq_inverse = -1;
    config.button_polarity = 0x01;
    config.rotary_encoder_direction = 1;
    break;
  case 2:
    config.freq_inverse = -1;
    config.button_polarity = 0x01;
    config.rotary_encoder_direction = -1;
    break;
  default:
    chprintf(chp, "unknown revision\r\n");
    break;
  }
}

static void cmd_save(BaseSequentialStream *chp, int argc, char *argv[])
{
  (void)argc;
  (void)argv;

  config.uistat = uistat;
  config_save();
  
  chprintf(chp, "Config saved.\r\n");
}

static void cmd_clearconfig(BaseSequentialStream *chp, int argc, char *argv[])
{
  if (argc != 1) {
    chprintf(chp, "usage: clearconfig {protection key}\r\n");
    return;
  }

  if (strcmp(argv[0], "1234") != 0) {
    chprintf(chp, "Key unmatched.\r\n");
    return;
  }

  clear_all_config_prop_data();
  chprintf(chp, "Config and all cal data cleared.\r\n");
}

static void cmd_uitest(BaseSequentialStream *chp, int argc, char *argv[])
{
  (void)argc;
  (void)argv;
  int i;
  for (i = 0; i < 100; i++) {
    //extern int btn_check(void);
    //int n = btn_check();
    //extern int read_buttons(void);
    //int n = read_buttons();
    extern int enc_count;
    chprintf(chp, "%d\r\n", enc_count);
    chThdSleepMilliseconds(100);
  }
}

static void cmd_lcd(BaseSequentialStream *chp, int argc, char *argv[])
{
  if (argc != 1) {
    chprintf(chp, "usage: lcd {rotate 180}\r\n");
    return;
  }
  int rot = atoi(argv[0]);
  ili9341_set_direction(rot);
  config.lcd_rotation = rot;
  disp_init(); // refresh all
}

static void _sample_portb10_11(BaseSequentialStream *chp)
{
  uint16_t buf[5];
  uint16_t buf2[32];
  chThdSleepMilliseconds(100);

  chSysLock();
  {
    register uint16_t *p = buf;
    register volatile uint16_t *portb = (uint16_t *)0x48000410;
    register uint16_t val1;
    register uint16_t val2 = 0;
    while ((val1 = (*portb & 0x0C00)) == val2) { }
    *p++ = val1;
    while ((val2 = (*portb & 0x0C00)) == val1) { }
    *p++ = val2;
    while ((val1 = (*portb & 0x0C00)) == val2) { }
    *p++ = val1;
    while ((val2 = (*portb & 0x0C00)) == val1) { }
    *p++ = val2;
    while ((val1 = (*portb & 0x0C00)) == val2) { }
    *p   = val1;
  }
  {
    register uint16_t *p = buf2;
    register volatile uint16_t *portb = (uint16_t *)0x48000410;
    *p++ = *portb;
    *p++ = *portb;
    *p++ = *portb;
    *p++ = *portb;
    *p++ = *portb;
    *p++ = *portb;
    *p++ = *portb;
    *p++ = *portb;
    *p++ = *portb;
    *p++ = *portb;
    *p++ = *portb;
    *p++ = *portb;
    *p++ = *portb;
    *p++ = *portb;
    *p++ = *portb;
    *p++ = *portb;
    *p++ = *portb;
    *p++ = *portb;
    *p++ = *portb;
    *p++ = *portb;
    *p++ = *portb;
    *p++ = *portb;
    *p++ = *portb;
    *p++ = *portb;
    *p++ = *portb;
    *p++ = *portb;
    *p++ = *portb;
    *p++ = *portb;
    *p++ = *portb;
    *p++ = *portb;
    *p++ = *portb;
    *p   = *portb;
  }
  chSysUnlock();

  for (int i = 0; i < 5; i++) {
    uint16_t b = buf[i];
    chprintf(chp, "%c %c\r\n", ((b & 0x0800) ? '1' : '0'), ((b & 0x0400) ? '1' : '0'));
  }
  chprintf(chp, "\r\n");
  for (int i = 0; i < 32; i++) {
    uint16_t b = buf2[i];
    chprintf(chp, "%c %c\r\n", ((b & 0x0800) ? '1' : '0'), ((b & 0x0400) ? '1' : '0'));
  }
}

#ifdef PORT_uSDX_TO_CentSDR
static int16_t _amp_test_level = -1;
#endif

static void cmd_test(BaseSequentialStream *chp, int argc, char *argv[])
{
  (void)argc;
  (void)argv;

  // _sample_portb10_11(chp);

#ifdef PORT_uSDX_TO_CentSDR
  // static uint8_t selected_in = 0;
  // if (++selected_in & 1)
  //   tlv320aic3204_select_in1();
  // else
  //   tlv320aic3204_select_in3();

  _amp_test_level = atoi(argv[0]);

  //_sine_wave_delta = (uint16_t)(((uint32_t)atoi(argv[0]) * 360 / 4800) << 7);
  //  interesting findings:
  //  - sudden frequency jump between 1093Hz and 1094Hz
  //  - strong harmonic starts 2094Hz (2093Hz is fine)
#endif
}

static void cmd_audio_codec_read_register(BaseSequentialStream *chp, int argc, char *argv[])
{
  if (argc < 2) {
    chprintf(chp, "acr [page] [register]\r\n");
  }

  uint8_t page = atoi(argv[0]);
  uint8_t reg = atoi(argv[1]);
  uint8_t r = tlv320aic3204_read_register(page, reg);
  chprintf(chp, "%02X\r\n", r);
}

static void cmd_audio_codec_write_register(BaseSequentialStream *chp, int argc, char *argv[])
{
  if (argc < 3) {
    chprintf(chp, "acw [page] [register] [value]\r\n");
  }

  uint8_t page = atoi(argv[0]);
  uint8_t reg = atoi(argv[1]);
  uint8_t val = atoi(argv[2]);

  uint8_t r = tlv320aic3204_read_register(page, reg);
  chprintf(chp, "before: %02X\r\n", r);

  tlv320aic3204_write_register(page, reg, val);

  r = tlv320aic3204_read_register(page, reg);
  chprintf(chp, "after: %02X\r\n", r);
}

#ifdef PORT_uSDX_TO_CentSDR

static void _set_tx_rx(bool next_tx)
{
  if (next_tx) {
    tlv320aic3204_select_in1();
    chThdSleepMilliseconds(50);
  }

  tx = next_tx;
  chThdSleepMilliseconds(50);

  if (!tx) {
    PWMD3.tim->CCR[2] = PWM_PERCENTAGE_TO_WIDTH(&PWMD3, 0); // turn off PWM
    si5351_set_frequency(center_frequency);                 // back to receive frequency
    tlv320aic3204_select_in3();
  }
}

#endif

static const ShellCommand commands[] =
{
    { "reset", cmd_reset },
    { "freq", cmd_freq },
    { "tune", cmd_tune },
    { "dac", cmd_dac },
    { "uitest", cmd_uitest },
    { "tone", cmd_tone },
    { "cwtone", cmd_cwtone },
    { "data", cmd_data },
    { "stat", cmd_stat },
    { "gain", cmd_gain },
    { "volume", cmd_volume },
    { "agc", cmd_agc },
    { "iqbal", cmd_iqbal },
    { "dcreject", cmd_dcreject },
    { "imp", cmd_impedance },
    { "mode", cmd_mode },
    { "fs", cmd_fs },
    { "winfunc", cmd_winfunc },
    { "show", cmd_show },
    { "power", cmd_power },
    { "channel", cmd_channel },
    { "ch", cmd_channel },
    { "revision", cmd_revision },
    { "save", cmd_save },
    { "clearconfig", cmd_clearconfig },
    { "phase", cmd_phase },
    { "finegain", cmd_finegain },
    { "lcd", cmd_lcd },
    { "test", cmd_test },
    { "copych", cmd_copy_channels },
    { "acr", cmd_audio_codec_read_register },
    { "acw", cmd_audio_codec_write_register },
    { NULL, NULL }
};

static THD_WORKING_AREA(waThread2, 512);
static __attribute__((noreturn)) THD_FUNCTION(Thread2, arg)
{
    (void)arg;
    chRegSetThreadName("button");
    while (1)
    {
      disp_process();
      ui_process();
      chThdSleepMilliseconds(10);
      stat.fps_count++;

#ifdef PORT_uSDX_TO_CentSDR
      // read PTT
      bool next_tx = (palReadLine(PAL_LINE(GPIOA, 1U)) == 0) ? false : true;
      if (next_tx != tx) {
        _set_tx_rx(next_tx);
      }

      if (!tx)
#endif
      {
        int flag = tlv320aic3204_get_sticky_flag_register();
        if (flag & AIC3204_STICKY_ADC_OVERFLOW)
          stat.overflow_count++;
      }
    }
}


static DACConfig dac1cfg1 = {
  //init:         2047U,
  init:         1080U,
  datamode:     DAC_DHRM_12BIT_RIGHT
};


#define SHELL_WA_SIZE THD_WORKING_AREA_SIZE(2048)

static const ShellConfig shell_cfg1 =
{
    (BaseSequentialStream *)&SDU1,
    commands
};

#ifdef PORT_uSDX_TO_CentSDR

static inline void set_tx_freq_delta(int delta)
{
  si5351_set_frequency(center_frequency + delta);
  // PWMD3.tim->CCR[2] = PWM_PERCENTAGE_TO_WIDTH(&PWMD3, ((uint32_t)delta * 10000 + 128) / 4800);   // test: observe the freq changes as PWM width
}

static inline void set_tx_amplitude(uint8_t amp)
{
  //pwmEnableChannel(&PWMD3, 2, PWM_PERCENTAGE_TO_WIDTH(&PWMD3, ((uint32_t)amp * 10000 + 128) / 256));  // 0 ~ 9961   // this causes the thread hung!!
  PWMD3.tim->CCR[2] = PWM_PERCENTAGE_TO_WIDTH(&PWMD3, ((uint32_t)amp * 10000 + 128) / 256);
  // PWMD3.tim->CCR[2] = PWM_PERCENTAGE_TO_WIDTH(&PWMD3, 10000);   // test: always 100%
}

static void gpt6_callback(GPTDriver *gptp)
{
  (void)gptp;

  if (tx) {
    int16_t amp = tx_amp_ph[s_tx_amp_ph_r_idx][0];   // amp: 0 ~ 255
    int16_t phase = tx_amp_ph[s_tx_amp_ph_r_idx][1]; // phase: 0 ~ 2400Hz
    if (++s_tx_amp_ph_r_idx >= (AUDIO_BUFFER_LEN / 10))
      s_tx_amp_ph_r_idx = 0;

    if (_amp_test_level >= 0 && _amp_test_level <= 255)
      amp = _amp_test_level;

    set_tx_freq_delta(phase);
    set_tx_amplitude((uint8_t)amp);
  }
  else {
    set_tx_amplitude(0);
  }
}

#endif


/*
 * Application entry point.
 */
int __attribute__((noreturn)) main(void)
{
  /*
   * System initializations.
   * - HAL initialization, this also initializes the configured device drivers
   *   and performs the board-specific initializations.
   * - Kernel initialization, the main() function becomes a thread and the
   *   RTOS is active.
   */
  halInit();
  chSysInit();

#ifdef PORT_uSDX_TO_CentSDR
  // pull-down the TX signal as soon as possible to avoid any unintentional TX
  palSetPadMode(GPIOA, 1, PAL_MODE_INPUT_PULLDOWN);
#endif

  /* restore config */
  config_recall();

  if (config.button_polarity != 0) {
    // pullup for revision 1 board
    palSetGroupMode(GPIOA, 1, 0, PAL_MODE_INPUT_PULLUP);
    palSetGroupMode(GPIOB, 6, 0, PAL_MODE_INPUT_PULLUP);
  }
  
  // copy uistat from uistat
  uistat = config.uistat;

  /*
   * Starting DAC1 driver, setting up the output pin as analog as suggested
   * by the Reference Manual.
   */
  dac1cfg1.init = config.dac_value;
  dacStart(&DACD1, &dac1cfg1);

  /*
   * Activates the ADC1 driver and the temperature sensor.
   */
  adcStart(&ADCD1, NULL);
  adcSTM32EnableTS(&ADCD1);
  adcSTM32EnableVBAT(&ADCD1);
  adcSTM32EnableVREF(&ADCD1);

  
  i2cStart(&I2CD1, &i2ccfg);
  /*
   * Initializes a serial-over-USB CDC driver.
   */
  sduObjectInit(&SDU1);
  sduStart(&SDU1, &serusbcfg);

  /*
   * Activates the USB driver and then the USB bus pull-up on D+.
   * Note, a delay is inserted in order to not have to disconnect the cable
   * after a reset.
   */
  usbDisconnectBus(serusbcfg.usbp);
  //chThdSleepMilliseconds(200);
  usbStart(serusbcfg.usbp, &usbcfg);
  usbConnectBus(serusbcfg.usbp);

#if 1
  /*
   * I2S Initialize
   */
  tlv320aic3204_init();

  i2sInit();
  i2sObjectInit(&I2SD2);
  i2sStart(&I2SD2, &i2sconfig);
  i2sStartExchange(&I2SD2);
#endif
  
#ifdef SI5351_GEN_QUADRATURE_LO_BELOW_3500KHZ
  palSetPadMode(GPIOB, 10, PAL_MODE_INPUT);
  palSetPadMode(GPIOB, 11, PAL_MODE_INPUT);
#endif

  dsp_init();
  
  /*
   * SPI LCD Initialize
   */
  ili9341_init();
  //ili9341_test(4);
  //ili9341_test(3);

#if 1
  /*
   * Initialize display
   */
  disp_init();
  ili9341_set_direction(config.lcd_rotation);
#endif
  /*
   * Shell manager initialization.
   */
  shellInit();

  /*
   * Creates the blinker thread.
   */
  chThdCreateStatic(waThread1, sizeof(waThread1), NORMALPRIO, Thread1, NULL);

#if 1
  ui_init();
#endif

  update_iqbal();
  update_agc();
  //tlv320aic3204_config_adc_filter2(config.freq_inverse /* + 0.129 */); // enable DC reject
  //tlv320aic3204_config_adc_filter(1); // enable DC reject

  /*
   * Creates the button thread.
   */
  chThdCreateStatic(waThread2, sizeof(waThread2), NORMALPRIO, Thread2, NULL);

#ifdef PORT_uSDX_TO_CentSDR
  static PWMConfig pwmcfg = {
    36000000,                               /* (STM32_HCLK / 2) = 36MHz PWM clock frequency */
    459,                                    /* Initial PWM period 78.431KHz */
    NULL,
    {
    {PWM_OUTPUT_DISABLED, NULL},
    {PWM_OUTPUT_DISABLED, NULL},
    {PWM_OUTPUT_ACTIVE_LOW, NULL},          // you can change to PWM_OUTPUT_ACTIVE_HIGH to flip the logic of PWM signal
    {PWM_OUTPUT_DISABLED, NULL},
    {PWM_OUTPUT_DISABLED, NULL},
    {PWM_OUTPUT_DISABLED, NULL}
    },
    0,
    0
  };
  palSetPadMode(GPIOB, 0, PAL_MODE_ALTERNATE(2));
  pwmStart(&PWMD3, &pwmcfg);
  pwmEnableChannel(&PWMD3, 2, PWM_PERCENTAGE_TO_WIDTH(&PWMD3, 0));   // initial duty cycle between 0 (0%) to 10000 (100%)

  /*
  * GPT6 configuration.
  */
  static const GPTConfig gpt6cfg1 = {
    .frequency    = 9600U,
    .callback     = gpt6_callback,
    .cr2          = TIM_CR2_MMS_1,    /* MMS = 010 = TRGO on Update Event.    */
    .dier         = 0U
  };
  gptStart(&GPTD6, &gpt6cfg1);
  gptStartContinuous(&GPTD6, 2U);   // 9600 / 2 = 4800Hz

  build_lut();
#endif


  /*
   * Normal main() thread activity, spawning shells.
   */
  while (true) {
    if (SDU1.config->usbp->state == USB_ACTIVE) {
      thread_t *shelltp = chThdCreateFromHeap(NULL, SHELL_WA_SIZE,
                                              "shell", NORMALPRIO + 1,
                                              shellThread, (void *)&shell_cfg1);
      chThdWait(shelltp);               /* Waiting termination.             */
    }
    chThdSleepMilliseconds(1000);
  }
}
