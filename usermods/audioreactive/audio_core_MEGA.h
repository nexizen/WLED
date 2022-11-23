#pragma once

// "MEGA" audio processing core:
// -- 22Khz
// -- overlaping (double-silde) FFT with 1024 samples
// -- 32 GEQ channels
// -- (to be added) "beatroot" beat detection based on FFT
// -- for ESP32, ESP32-S3
// -- not recommended for ESP32-S2, ESSP32-C3 or slower

#ifdef HAVE_AUDIO_CORE
#error please include only one audio core when compiling
#endif

#define HAVE_AUDIO_CORE

// WORK IN PROGRESS !!


// audio source parameters and constant
constexpr SRate_t SAMPLE_RATE = 22050;        // Base sample rate in Hz - 22Khz is a standard rate. Physical sample time -> 23ms
//constexpr SRate_t SAMPLE_RATE = 16000;        // 16kHz - use if FFTtask takes more than 20ms. Physical sample time -> 32ms
//constexpr SRate_t SAMPLE_RATE = 20480;        // Base sample rate in Hz - 20Khz is experimental.    Physical sample time -> 25ms
//constexpr SRate_t SAMPLE_RATE = 10240;        // Base sample rate in Hz - previous default.         Physical sample time -> 50ms
#define FFT_MIN_CYCLE 21                      // minimum time before FFT task is repeated. Use with 22Khz sampling
//#define FFT_MIN_CYCLE 30                      // Use with 16Khz sampling
//#define FFT_MIN_CYCLE 23                      // minimum time before FFT task is repeated. Use with 20Khz sampling
//#define FFT_MIN_CYCLE 46                      // minimum time before FFT task is repeated. Use with 10Khz sampling

// FFT Constants
constexpr uint16_t samplesFFT = 1024;            // Samples in an FFT batch - This value MUST ALWAYS be a power of 2
constexpr uint16_t samplesFFT_2 = 512;          // meaningfull part of FFT results - only the "lower half" contains useful information.
// the following are observed values, supported by a bit of "educated guessing"
//#define FFT_DOWNSCALE 0.65f                             // 20kHz - downscaling factor for FFT results - "Flat-Top" window @20Khz, old freq channels 
#define FFT_DOWNSCALE 0.46f                             // downscaling factor for FFT results - for "Flat-Top" window @22Khz, new freq channels
#define LOG_256  5.54517744                             // log2(256)

// These are the input and output vectors.  Input vectors receive computed results from FFT.
static float vReal[samplesFFT] = {0.0f};       // FFT sample inputs / freq output -  these are our raw result bins
static float vImag[samplesFFT] = {0.0f};       // imaginary parts
#ifdef UM_AUDIOREACTIVE_USE_NEW_FFT
static float windowWeighingFactors[samplesFFT] = {0.0f};
#endif

// Create FFT object
#ifdef UM_AUDIOREACTIVE_USE_NEW_FFT
// lib_deps += https://github.com/kosme/arduinoFFT#develop @ 1.9.2
#define FFT_SPEED_OVER_PRECISION     // enables use of reciprocals (1/x etc), and an a few other speedups
#define FFT_SQRT_APPROXIMATION       // enables "quake3" style inverse sqrt
#define sqrt(x) sqrtf(x)             // little hack that reduces FFT time by 50% on ESP32 (as alternative to FFT_SQRT_APPROXIMATION)
#else
// lib_deps += https://github.com/blazoncek/arduinoFFT.git
#endif
#include <arduinoFFT.h>
#ifdef UM_AUDIOREACTIVE_USE_NEW_FFT
static ArduinoFFT<float> FFT = ArduinoFFT<float>( vReal, vImag, samplesFFT, SAMPLE_RATE, windowWeighingFactors);
#else
static arduinoFFT FFT = arduinoFFT(vReal, vImag, samplesFFT, SAMPLE_RATE);
#endif

//
// FFT main task
//
void FFTcode(void * parameter)
{
  DEBUGSR_PRINT("FFT started on core: "); DEBUGSR_PRINTLN(xPortGetCoreID());

  // see https://www.freertos.org/vtaskdelayuntil.html
  const TickType_t xFrequency = FFT_MIN_CYCLE * portTICK_PERIOD_MS;  

  TickType_t xLastWakeTime = xTaskGetTickCount();
  for(;;) {
    delay(1);           // DO NOT DELETE THIS LINE! It is needed to give the IDLE(0) task enough time and to keep the watchdog happy.
                        // taskYIELD(), yield(), vTaskDelay() and esp_task_wdt_feed() didn't seem to work.

    // Don't run FFT computing code if we're in Receive mode or in realtime mode
    if (disableSoundProcessing || (audioSyncEnabled & 0x02)) {
      vTaskDelayUntil( &xLastWakeTime, xFrequency);        // release CPU, and let I2S fill its buffers
      continue;
    }

#if defined(WLED_DEBUG) || defined(SR_DEBUG)|| defined(SR_STATS)
    uint64_t start = esp_timer_get_time();
    bool haveDoneFFT = false; // indicates if second measurement (FFT time) is valid
#endif

    // get a fresh batch of samples from I2S
    if (audioSource) audioSource->getSamples(vReal, samplesFFT);

#if defined(WLED_DEBUG) || defined(SR_DEBUG)|| defined(SR_STATS)
    if (start < esp_timer_get_time()) { // filter out overflows
      uint64_t sampleTimeInMillis = (esp_timer_get_time() - start +5ULL) / 10ULL; // "+5" to ensure proper rounding
      sampleTime = (sampleTimeInMillis*3 + sampleTime*7)/10; // smooth
    }
    start = esp_timer_get_time(); // start measuring FFT time
#endif

    xLastWakeTime = xTaskGetTickCount();       // update "last unblocked time" for vTaskDelay

    // band pass filter - can reduce noise floor by a factor of 50
    // downside: frequencies below 100Hz will be ignored
    if (useBandPassFilter) runMicFilter(samplesFFT, vReal);

    // find highest sample in the batch
    float maxSample = 0.0f;                         // max sample from FFT batch
    for (int i=0; i < samplesFFT; i++) {
	    // set imaginary parts to 0
      vImag[i] = 0;
	    // pick our  our current mic sample - we take the max value from all samples that go into FFT
	    if ((vReal[i] <= (INT16_MAX - 1024)) && (vReal[i] >= (INT16_MIN + 1024)))  //skip extreme values - normally these are artefacts
        if (fabsf((float)vReal[i]) > maxSample) maxSample = fabsf((float)vReal[i]);
    }
    // release highest sample to volume reactive effects early - not strictly necessary here - could also be done at the end of the function
    // early release allows the filters (getSample() and agcAvg()) to work with fresh values - we will have matching gain and noise gate values when we want to process the FFT results.
    micDataReal = maxSample;

    // run FFT (takes 3-5ms on ESP32)
    //if (fabsf(sampleAvg) > 0.25f) { // noise gate open
    if (fabsf(volumeSmth) > 0.25f) { // noise gate open

      // run FFT (takes 3-5ms on ESP32, ~12ms on ESP32-S2)
#ifdef UM_AUDIOREACTIVE_USE_NEW_FFT
      FFT.dcRemoval();                                            // remove DC offset
      #if !defined(FFT_PREFER_EXACT_PEAKS)
        FFT.windowing( FFTWindow::Flat_top, FFTDirection::Forward);        // Weigh data using "Flat Top" function - better amplitude accuracy
      #else
        FFT.windowing(FFTWindow::Blackman_Harris, FFTDirection::Forward);  // Weigh data using "Blackman- Harris" window - sharp peaks due to excellent sideband rejection
      #endif
      FFT.compute( FFTDirection::Forward );                       // Compute FFT
      FFT.complexToMagnitude();                                   // Compute magnitudes
#else
      FFT.DCRemoval(); // let FFT lib remove DC component, so we don't need to care about this in getSamples()

      //FFT.Windowing( FFT_WIN_TYP_HAMMING, FFT_FORWARD );        // Weigh data - standard Hamming window
      //FFT.Windowing( FFT_WIN_TYP_BLACKMAN, FFT_FORWARD );       // Blackman window - better side freq rejection
      #if !defined(FFT_PREFER_EXACT_PEAKS)
        FFT.Windowing( FFT_WIN_TYP_FLT_TOP, FFT_FORWARD );        // Flat Top Window - better amplitude accuracy
      #else
        FFT.Windowing( FFT_WIN_TYP_BLACKMAN_HARRIS, FFT_FORWARD );// Blackman-Harris - excellent sideband rejection
      #endif
      FFT.Compute( FFT_FORWARD );                             // Compute FFT
      FFT.ComplexToMagnitude();                               // Compute magnitudes
#endif

#ifdef UM_AUDIOREACTIVE_USE_NEW_FFT
      FFT.majorPeak(FFT_MajorPeak, FFT_Magnitude);                // let the effects know which freq was most dominant
#else
      FFT.MajorPeak(&FFT_MajorPeak, &FFT_Magnitude);              // let the effects know which freq was most dominant
#endif
      FFT_MajorPeak = constrain(FFT_MajorPeak, 1.0f, 11025.0f);   // restrict value to range expected by effects

#if defined(WLED_DEBUG) || defined(SR_DEBUG) || defined(SR_STATS)
      haveDoneFFT = true;
#endif

    } else { // noise gate closed - only clear results as FFT was skipped. MIC samples are still valid when we do this.
      memset(vReal, 0, sizeof(vReal));
      FFT_MajorPeak = 1;
      FFT_Magnitude = 0.001;
    }

    for (int i = 0; i < samplesFFT; i++) {
      float t = fabsf(vReal[i]);                      // just to be sure - values in fft bins should be positive any way
      vReal[i] = t / 16.0f;                           // Reduce magnitude. Want end result to be scaled linear and ~4096 max.
    } // for()

    // mapping of FFT result bins to frequency channels
    //if (fabsf(sampleAvg) > 0.25f) { // noise gate open
    if (fabsf(volumeSmth) > 0.25f) { // noise gate open
#if 0
    /* This FFT post processing is a DIY endeavour. What we really need is someone with sound engineering expertise to do a great job here AND most importantly, that the animations look GREAT as a result.
    *
    * Andrew's updated mapping of 256 bins down to the 16 result bins with Sample Freq = 10240, samplesFFT = 512 and some overlap.
    * Based on testing, the lowest/Start frequency is 60 Hz (with bin 3) and a highest/End frequency of 5120 Hz in bin 255.
    * Now, Take the 60Hz and multiply by 1.320367784 to get the next frequency and so on until the end. Then detetermine the bins.
    * End frequency = Start frequency * multiplier ^ 16
    * Multiplier = (End frequency/ Start frequency) ^ 1/16
    * Multiplier = 1.320367784
    */                                    //  Range
      fftCalc[ 0] = fftAddAvg(2,4);       // 60 - 100
      fftCalc[ 1] = fftAddAvg(4,5);       // 80 - 120
      fftCalc[ 2] = fftAddAvg(5,7);       // 100 - 160
      fftCalc[ 3] = fftAddAvg(7,9);       // 140 - 200
      fftCalc[ 4] = fftAddAvg(9,12);      // 180 - 260
      fftCalc[ 5] = fftAddAvg(12,16);     // 240 - 340
      fftCalc[ 6] = fftAddAvg(16,21);     // 320 - 440
      fftCalc[ 7] = fftAddAvg(21,29);     // 420 - 600
      fftCalc[ 8] = fftAddAvg(29,37);     // 580 - 760
      fftCalc[ 9] = fftAddAvg(37,48);     // 740 - 980
      fftCalc[10] = fftAddAvg(48,64);     // 960 - 1300
      fftCalc[11] = fftAddAvg(64,84);     // 1280 - 1700
      fftCalc[12] = fftAddAvg(84,111);    // 1680 - 2240
      fftCalc[13] = fftAddAvg(111,147);   // 2220 - 2960
      fftCalc[14] = fftAddAvg(147,194);   // 2940 - 3900
      fftCalc[15] = fftAddAvg(194,250);   // 3880 - 5000 // avoid the last 5 bins, which are usually inaccurate
#else
      /* new mapping, optimized for 22050 Hz by softhack007 */
                                                    // bins frequency  range
      if (useBandPassFilter) {
        // skip frequencies below 100hz
        fftCalc[ 0] = 0.8f * fftAddAvg(3,4);
        fftCalc[ 1] = 0.9f * fftAddAvg(4,5);
        fftCalc[ 2] = fftAddAvg(5,6);
        fftCalc[ 3] = fftAddAvg(6,7);
        // don't use the last bins from 206 to 255. 
        fftCalc[15] = fftAddAvg(165,205) * 0.75f;   // 40 7106 - 8828 high             -- with some damping
      } else {
        fftCalc[ 0] = fftAddAvg(1,2);               // 1    43 - 86   sub-bass
        fftCalc[ 1] = fftAddAvg(2,3);               // 1    86 - 129  bass
        fftCalc[ 2] = fftAddAvg(3,5);               // 2   129 - 216  bass
        fftCalc[ 3] = fftAddAvg(5,7);               // 2   216 - 301  bass + midrange
        // don't use the last bins from 216 to 255. They are usually contaminated by aliasing (aka noise) 
        fftCalc[15] = fftAddAvg(165,215) * 0.70f;   // 50 7106 - 9259 high             -- with some damping
      }
      fftCalc[ 4] = fftAddAvg(7,10);                // 3   301 - 430  midrange
      fftCalc[ 5] = fftAddAvg(10,13);               // 3   430 - 560  midrange
      fftCalc[ 6] = fftAddAvg(13,19);               // 5   560 - 818  midrange
      fftCalc[ 7] = fftAddAvg(19,26);               // 7   818 - 1120 midrange -- 1Khz should always be the center !
      fftCalc[ 8] = fftAddAvg(26,33);               // 7  1120 - 1421 midrange
      fftCalc[ 9] = fftAddAvg(33,44);               // 9  1421 - 1895 midrange
      fftCalc[10] = fftAddAvg(44,56);               // 12 1895 - 2412 midrange + high mid
      fftCalc[11] = fftAddAvg(56,70);               // 14 2412 - 3015 high mid
      fftCalc[12] = fftAddAvg(70,86);               // 16 3015 - 3704 high mid
      fftCalc[13] = fftAddAvg(86,104);              // 18 3704 - 4479 high mid
      fftCalc[14] = fftAddAvg(104,165) * 0.88f;     // 61 4479 - 7106 high mid + high  -- with slight damping
#endif
    } else {  // noise gate closed - just decay old values
      for (int i=0; i < NUM_GEQ_CHANNELS; i++) {
        fftCalc[i] *= 0.85f;  // decay to zero
        if (fftCalc[i] < 4.0f) fftCalc[i] = 0.0f;
      }
    }

    // post-processing of frequency channels (pink noise adjustment, AGC, smooting, scaling)
    if (pinkIndex > MAX_PINK) pinkIndex = MAX_PINK;
    //postProcessFFTResults((fabsf(sampleAvg) > 0.25f)? true : false , NUM_GEQ_CHANNELS);
    postProcessFFTResults((fabsf(volumeSmth)>0.25f)? true : false , NUM_GEQ_CHANNELS);

#if defined(WLED_DEBUG) || defined(SR_DEBUG)|| defined(SR_STATS)
    if (haveDoneFFT && (start < esp_timer_get_time())) { // filter out overflows
      uint64_t fftTimeInMillis = ((esp_timer_get_time() - start) +5ULL) / 10ULL; // "+5" to ensure proper rounding
      fftTime  = (fftTimeInMillis*3 + fftTime*7)/10; // smooth
    }
#endif
    // run peak detection
    autoResetPeak();
    detectSamplePeak();
    
    #if !defined(I2S_GRAB_ADC1_COMPLETELY)    
    if ((audioSource == nullptr) || (audioSource->getType() != AudioSource::Type_I2SAdc))  // the "delay trick" does not help for analog ADC
    #endif
      vTaskDelayUntil( &xLastWakeTime, xFrequency);        // release CPU, and let I2S fill its buffers

  } // for(;;)ever
} // FFTcode() task end


////////////////////
// Peak detection //
////////////////////

// peak detection is called from FFT task when vReal[] contains valid FFT results
static void detectSamplePeak(void) {
  bool havePeak = false;

  // Poor man's beat detection by seeing if sample > Average + some value.
  // This goes through ALL of the 255 bins - but ignores stupid settings
  // Then we got a peak, else we don't. The peak has to time out on its own in order to support UDP sound sync.
  if ((sampleAvg > 1) && (maxVol > 0) && (binNum > 1) && (vReal[binNum] > maxVol) && ((millis() - timeOfPeak) > 100)) {
    havePeak = true;
  }

#if 0
  // alternate detection, based on FFT_MajorPeak and FFT_Magnitude. Not much better...
  if ((binNum > 1) && (binNum < 10) && (sampleAgc > 127) && 
      (FFT_MajorPeak > 50) && (FFT_MajorPeak < 250) && (FFT_Magnitude > (16.0f * (maxVol+42.0)) /*my_magnitude > 136.0f*16.0f*/) && 
      (millis() - timeOfPeak > 80)) {
    havePeak = true;
  }
#endif

  if (havePeak) {
    samplePeak    = true;
    timeOfPeak    = millis();
    udpSamplePeak = true;
  }

}
