/* This code computes time delays of arrival (TDOAs) for acoustic positioning.
 * It reads an analog signal provided by an electret microphone and processed by a front-end circuitry.
 * 
 * The acoustic signal has a length of T_cycle and is emitted by N emitters consecutively.
 * A precisely timed loop (realized via interrupts) runs with T_cycle.
 * A cross-correlation of two consecutive signals is computed by R_xy = IFFT[ conj( FFT( x ) ) * FFT( y ) ].
 * 1024 points xcorr is computed in ~4500 us.
 * 
 * Author: Eugen Solowjow
 */

// DEBUGGING PREPROCESSOR MACRO DEFINITION
#if DEBUG == 1
#define dprint(expression) Serial.print("# "); Serial.print( #expression ); Serial.print( ": " ); Serial.println( expression )
#define dshow(expression) Serial.println( expression )
#else
#define dprint(expression)
#define dshow(expression)
#endif
//***************************************

#include "ADC.h"
#include "RingBufferDMA.h"
#include "arm_math.h"

// INITIALIZATIONS
// Timing 
const uint32_t T_cycle = 75000;      // [us]. Period for highly precise loop. Has to coincide with the duration of the acoustic signal. Consists of recording time T_recording and the processing time.
const uint32_t T_recording = 50000;  // [us]. Recording time. Starts immidiately after loop is entered.
volatile boolean isrFlag = 0;        // flag for ISR to enter main loop every T_recording.
IntervalTimer isrTimer;              // interrupt to call recording loop at a precise timing interval

// ADC Conversion
const int readPin = A2;             // read pin
ADC *adc = new ADC();               // create ADC object
const int buffer_size = 4096;       // define the array that holds the conversions here. Must be power of two. TODO: Bug: buffer_size has to be 2 * signal_raw right now.
DMAMEM static volatile int16_t __attribute__((aligned(buffer_size+0))) buffer[buffer_size]; //buffer is stored with the correct alignment in the DMAMEM section, the +0 in the aligned attribute is necessary b/c of a bug in gcc.

// Direct Memory Access
RingBufferDMA *dmaBuffer = new RingBufferDMA(buffer, buffer_size, ADC_0);  // use dma with ADC0

//Initialize FFT objects and FFT arrays
const int16_t FFT_SIZE = 1024;
int32_t xCorrResult[2 * FFT_SIZE];
arm_cfft_radix4_instance_q15 fft_inst; // create arm fft object, q15 data corresponds to int16_t
arm_cfft_radix4_instance_q31 ifft_inst;  // create arm ifft object, q15 data corresponds to int32_t

// Containers for Xcorr preprocessing
const int16_t SIGNAL_RAW_SIZE = 2048;
int16_t  signal_raw[SIGNAL_RAW_SIZE];           // raw signal recorded by ADC.
int16_t signal_0[FFT_SIZE] = {0}, signal_1[FFT_SIZE] = {0}, signal_2[FFT_SIZE] = {0}, signal_3[FFT_SIZE] = {0};  //Xcorr inputs. Buffer acoustic signal aquired by ADC. Each T_cycle period one array is updated. The most recent array is signal_0.

// Function Prototypes
void setIsrTimingFlag( void );
void adc0_isr( void );
void fast_xcorr( int16_t signal_A[], int16_t signal_B[], int32_t xCorrResult[], uint16_t FFT_SIZE );
void peak_detection( int16_t *peak_offset, int32_t *peak_value, int32_t xCorrResult[], uint16_t FFT_SIZE );


// Setup
void setup() {

  Serial.begin(9600);

// Timing
  isrTimer.priority(220);
  isrTimer.begin( setIsrTimingFlag, T_cycle );

// ADC Conversion, scroll down for details
  adc -> setAveraging( 8 ); // set number of averages
  adc -> setResolution( 16 ); // set bits of resolution
  adc -> setConversionSpeed( ADC_VERY_HIGH_SPEED ); // change the conversion speed
  adc -> setSamplingSpeed( ADC_VERY_HIGH_SPEED ); // change the sampling speed

// FFT
  arm_cfft_radix4_init_q15( &fft_inst, FFT_SIZE, 0, 1 );   // initialize arm fft 
  arm_cfft_radix4_init_q31( &ifft_inst, FFT_SIZE, 1, 1 );  // initialize arm ifft 
  
// Enable DMA and Interrupts
  adc -> enableDMA( ADC_0 );
  adc -> enableInterrupts( ADC_0 );
  dmaBuffer -> start();     // start DMA buffer
}

// Main Loop
void loop() {
  
  uint16_t count = 0;     // count elements written to raw data array
    
  if ( isrFlag ) {      // enters loop every T_cycle us

    while ( count < SIGNAL_RAW_SIZE ) {
      adc -> analogRead( readPin, ADC_0 );    // ADC conversion, result is saved to dmaBuffer
      count ++;
    }
       
    isrFlag = false;    // sets timing isr flag to false, so that a new loop can be entered

    for ( int i = 0; i < SIGNAL_RAW_SIZE; i++ ) {  //loop through dmaBuffer and copy content to signal_raw array
        signal_raw[i] = (dmaBuffer -> p_elems[i]) - 12000;  //TODO: Remove DC gain automatically
      }

    // Process recorded data
    elapsedMicros sinceProcStart;
    
    int16_t peak_offset = 0;     // initialize peak offset and peak value of xcorr result
    int32_t peak_value = 0;

    for ( unsigned it = 0; it < 1; it++ ) {

        for ( unsigned i = 0; i < FFT_SIZE; i++ ) {
          signal_0[i] = signal_raw[i + it * 256];       //Copy FFT_SIZE elements of latest signal_raw to signal_0
        }

        // Perform xcorr    
        fast_xcorr( signal_0, signal_1, xCorrResult, FFT_SIZE );  // The output data is in every second element of xCorrResult

        // Find peak and associated offset
        peak_detection( &peak_offset, &peak_value, xCorrResult, FFT_SIZE);    // updates peak_offset and peak_value for provided xCorrResult array
        peak_offset -= it * 256;                                              // adjusts the peak_offset

    }

      Serial.println( -peak_offset ); 
      Serial.print(" ");

      memcpy( signal_3 , signal_2, 2*FFT_SIZE );
      memcpy( signal_2 , signal_1, 2*FFT_SIZE );
      memcpy( signal_1 , signal_0, 2*FFT_SIZE );
      
      int32_t DataProcessing = sinceProcStart;
      dprint( DataProcessing );

   }   // timing loop
}      // main

/* Set isr flag to true when called.
 */
void setIsrTimingFlag( void ) {
  isrFlag = true;
}

/* Clear ADC interrupt
 */
void adc0_isr( void ) {
    ADC0_RA; // clear interrupt
}

/* Compute cross-correlation of signal_A and signal_B of size FFT_SIZE.
 * R_xy = IFFT[ conj( FFT( x ) ) * FFT( y ) ]
 * The result is written to xCorrResult array.
 * Three FFT operations are required. However, two FFTs can be computed for the price of one, if the signals are real valued.
 */
void fast_xcorr( int16_t signal_A[], int16_t signal_B[], int32_t xCorrResult[], uint16_t FFT_SIZE ) {       
  
       int16_t FFT_buffer[2 * FFT_SIZE];   // buffers FFT results
       int32_t iFFT_buffer[2 * FFT_SIZE];  // buffers iFFT results

        // FFT_buffer = signal_A + j * signal_B. See http://www.ti.com/lit/an/spra291/spra291.pdf , Sec. 3.1
        for ( int i = 0; i < FFT_SIZE; i++ ){
          ((int32_t *)FFT_buffer)[i] = (signal_A[i] << 16) | signal_B[i];
        }
        
        // Perform FFT
        arm_cfft_radix4_q15( &fft_inst, FFT_buffer );

        // Untangle fft of both signals and calculate conj(fft(signal_A)) * fft(ssignal_B)
        int16_t sigA_r, sigA_i, sigB_r, sigB_i;
        // DC point
        
        sigA_r = FFT_buffer[0]; // sigA_i = 0
        sigB_r = FFT_buffer[0]; // sigB_i = 0
        iFFT_buffer[0] = sigA_r * sigB_r;
        iFFT_buffer[1] = 0;
        
        // other points
        int16_t *forward = FFT_buffer + 2;
        int16_t *reverse = FFT_buffer + 2 * (FFT_SIZE - 1);
        for ( size_t i = 2; i < FFT_SIZE; i += 2, forward += 2, reverse -= 2 ) {
          sigA_r = (forward[0] + reverse[0]) >> 1;
          sigA_i = (forward[1] - reverse[1]) >> 1;
          sigB_r = (reverse[1] + forward[1]) >> 1;
          sigB_i = (reverse[0] - forward[0]) >> 1;
          
          // calculate conj(fft(signal_A)) * fft(signal_B)
          int32_t cs_sigB_r = (sigA_r * sigB_r + sigA_i * sigB_i);
          int32_t cs_sigB_i = (sigA_r * sigB_i - sigA_i * sigB_r);
          iFFT_buffer[i] = iFFT_buffer[2 * FFT_SIZE - i] = cs_sigB_r;
          iFFT_buffer[i + 1] = cs_sigB_i;
          iFFT_buffer[2 * FFT_SIZE - i + 1] = -cs_sigB_i;
        }
        
        //  Nyquist point
        sigA_r = forward[0]; // sigA_i = 0
        sigB_r = forward[1]; // sigB_i = 0
        iFFT_buffer[FFT_SIZE] = sigA_r * sigB_r;
        iFFT_buffer[FFT_SIZE + 1] = 0;

        // Compute inverse FFT
        arm_cfft_radix4_q31( &ifft_inst, iFFT_buffer );

        memcpy( xCorrResult ,iFFT_buffer, 4*2*FFT_SIZE );
}

/* Detect peak offset where xCorrResult is max
 */
void peak_detection( int16_t *peak_offset, int32_t *peak_value, int32_t xCorrResult[], uint16_t FFT_SIZE ) {
        int16_t peak_offset_local = 0;
        int32_t peak_value_local = xCorrResult[0];

        for ( int16_t i = 1, j = 2; i < FFT_SIZE; i++, j += 2 ) {
          if (xCorrResult[j] > peak_value_local) {
            peak_value_local = xCorrResult[j];
            peak_offset_local = i;
          }
          if ( peak_offset_local > FFT_SIZE / 2 )
            peak_offset_local -= FFT_SIZE;
        }

        if ( peak_value_local > *peak_value ) {
            *peak_value = peak_value_local;
            *peak_offset = peak_offset_local;
        }
}

/* ADC ReadMe
 *  
 *  Always call the compare functions after changing the resolution.
 *  adc->enableCompare(1.0/3.3*adc->getMaxValue(ADC_0), 0, ADC_0); // measurement will be ready if value < 1.0V
 *  adc->enableCompareRange(1.0*adc->getMaxValue(ADC_1)/3.3, 2.0*adc->getMaxValue(ADC_1)/3.3, 0, 1, ADC_1); // ready if value lies out of [1.0,2.0] V
 *  
 */
// COPIED FROM ADC-LIB
// RESULTS OF THE TEST Teensy 3.x
// Measure continuously a voltage divider.
// Measurement pin A9 (23). Clock speed 96 Mhz, bus speed 48 MHz.

//
//  Using ADC_LOW_SPEED (same as ADC_VERY_LOW_SPEED) for sampling and conversion speeds
// ADC resolution     Measurement frequency                 Num. averages
//     16  bits            64 kHz                               1
//     13  bits            71 kHz                               1
//     11  bits            71 kHz                               1
//      9  bits            77 kHz                               1

//     16  bits             2.0 kHz                              32
//     13  bits             2.2 kHz                              32
//     11  bits             2.2 kHz                              32
//      9  bits             2.4 kHz                              32

//
//  Using ADC_MED_SPEED for sampling and conversion speeds
// ADC resolution     Measurement frequency                 Num. averages
//     16  bits           150 kHz                               1
//     13  bits           167 kHz                               1
//     11  bits           167 kHz                               1
//      9  bits           182 kHz                               1

//     11  bits            42 kHz                               4 (default settings) corresponds to about 17.24 us

//
//  Using ADC_HIGH_SPEED (same as ADC_HIGH_SPEED_16BITS) for sampling and conversion speeds
// ADC resolution     Measurement frequency                 Num. averages
//     16  bits           316 kHz                               1
//     13  bits           353 kHz                               1
//     11  bits           353 kHz                               1
//      9  bits           387 kHz                               1
//
//      9  bits           245 kHz                               1 ADC_VERY_LOW_SPEED sampling
//      9  bits           293 kHz                               1 ADC_LOW_SPEED sampling
//      9  bits           343 kHz                               1 ADC_MED_SPEED sampling
//      9  bits           414 kHz                               1 ADC_VERY_HIGH_SPEED sampling

//
//  Using ADC_VERY_HIGH_SPEED for sampling and conversion speeds
//  This conversion speed is over the limit of the specs! (speed=24MHz, limit = 18 MHz for res<16 and 12 for res=16)
// ADC resolution     Measurement frequency                 Num. averages
//     16  bits           666 kHz                               1
//     13  bits           749 kHz                               1
//     11  bits           749 kHz                               1
//      9  bits           827 kHz                               1
// At 96 Mhz (bus at 48 MHz), 414 KHz is the fastest we can do within the specs, and only if the sample's impedance is low enough.

