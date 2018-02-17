/*************************************************************************************
			       DEPARTMENT OF ELECTRICAL AND ELECTRONIC ENGINEERING
					   		     IMPERIAL COLLEGE LONDON 

 				      EE 3.19: Real Time Digital Signal Processing
					       Dr Paul Mitcheson and Daniel Harvey

				        		 PROJECT: Frame Processing

 				            ********* ENHANCE. C **********
							 Shell for speech enhancement 

  		Demonstrates overlap-add frame processing (interrupt driven) on the DSK. 

 *************************************************************************************
 				             By Danny Harvey: 21 July 2006
							 Updated for use on CCS v4 Sept 2010
 ************************************************************************************/
/*
 *	You should modify the code so that a speech enhancement project is built 
 *  on top of this template.
 */
/**************************** Pre-processor statements ******************************/
//  library required when using calloc
#include <stdlib.h>
//  Included so program can make use of DSP/BIOS configuration tool.  
#include "dsp_bios_cfg.h"

/* The file dsk6713.h must be included in every program that uses the BSL.  This 
   example also includes dsk6713_aic23.h because it uses the 
   AIC23 codec module (audio interface). */
#include "dsk6713.h"
#include "dsk6713_aic23.h"

// math library (trig functions)
#include <math.h>

/* Some functions to help with Complex algebra and FFT. */
#include "cmplx.h"      
#include "fft_functions.h"  

// Some functions to help with writing/reading the audio ports when using interrupts.
#include <helper_functions_ISR.h>

#define WINCONST 0.85185			/* 0.46/0.54 for Hamming window */
#define FSAMP 8000.0				/* sample frequency, ensure this matches Config for AIC */
#define FFTLEN 256					/* fft length = frame length 256/8000 = 32 ms*/
#define NFREQ (1+FFTLEN/2)			/* number of frequency bins from a real FFT */
#define OVERSAMP 4					/* oversampling ratio (2 or 4) */  
#define FRAMEINC (FFTLEN/OVERSAMP)	/* Frame increment */
#define CIRCBUF (FFTLEN+FRAMEINC)	/* length of I/O buffers */

#define OUTGAIN 16000.0				/* Output gain for DAC */
#define INGAIN  (1.0/16000.0)		/* Input gain for ADC  */

#define PI 3.141592653589793
#define TFRAME FRAMEINC/FSAMP       /* time between calculation of each frame */

/******************************* Global declarations ********************************/

/* Audio port configuration settings: these values set registers in the AIC23 audio 
   interface to configure it. See TI doc SLWS106D 3-3 to 3-10 for more info. */
DSK6713_AIC23_Config Config = { \
			 /**********************************************************************/
			 /*   REGISTER	            FUNCTION			      SETTINGS         */ 
			 /**********************************************************************/\
    0x0017,  /* 0 LEFTINVOL  Left line input channel volume  0dB                   */\
    0x0017,  /* 1 RIGHTINVOL Right line input channel volume 0dB                   */\
    0x01f9,  /* 2 LEFTHPVOL  Left channel headphone volume   0dB                   */\
    0x01f9,  /* 3 RIGHTHPVOL Right channel headphone volume  0dB                   */\
    0x0011,  /* 4 ANAPATH    Analog audio path control       DAC on, Mic boost 20dB*/\
    0x0000,  /* 5 DIGPATH    Digital audio path control      All Filters off       */\
    0x0000,  /* 6 DPOWERDOWN Power down control              All Hardware on       */\
    0x0043,  /* 7 DIGIF      Digital audio interface format  16 bit                */\
    0x008d,  /* 8 SAMPLERATE Sample rate control        8 KHZ-ensure matches FSAMP */\
    0x0001   /* 9 DIGACT     Digital interface activation    On                    */\
			 /**********************************************************************/
};

// Codec handle:- a variable used to identify audio interface  
DSK6713_AIC23_CodecHandle H_Codec;

float *inbuffer, *outbuffer;   		/* Input/output circular buffers */
float *inframe, *outframe;          /* Input and output frames */
float *inwin, *outwin;              /* Input and output windows */

/************* Array Declarations ************/
float *X_magnitude;					/* Magnitude spectrum */
float *Pt, *Pt_prev;				/* Low pass filtered input */
float *low_pass_noise; 				/* Low pass noise estimate array */
float *low_pass_noise_prev; 		/* Low pass noise estimate array */
float *noise_estimate; 				/* Noise estimate array */
float *M1, *M2, *M3, *M4, *tmp_M4; 	/* 2.5 sec buffers to find minimum noise amp */
complex *intermediate; 	            /* Complex array to perform FFT/IFFT calculations */ 

/**********************************************/
float ingain, outgain;				/* ADC and DAC gains */ 
float cpufrac; 						/* Fraction of CPU time used */

/*********************************************/
volatile int io_ptr=0;              /* Input/ouput pointer for circular buffers */
volatile int frame_ptr=0;           /* Frame pointer */
volatile int f_index=-1;			/* Counts until 312 corresponding to 2.5 secs*/

/*********************************************/
float alpha1 = 2;					/* Correction factor for noise estimation */
float alpha_basic = 20; 			/* Correction factor for noise estimation */
float lambda = 0.1; 				/* Lambda value */
float g; 							/* Frequency dependent gain factor */
float tau = 0.025; 					/* Time constant tau=25ms */
float tau_4 = 0.06;					/* Time constant for enhancement 4 */
float noise_tau3 = 0.08; 			/* Time constant for enhancement 3 */
float kappa; 						/* Kappa variable for enhancements 1,2 */
float kappa_noise; 					/* Kappa variable for enhancement 3 */
float kappa_4; 						/* Kappa variable for enhancement 4 */
float global_min;					/* Variable to hold the min out of all frames */
float ROTATE_TIME = 2;				/* Parameter to control rotation of frames */

/*********** Enhancement 8 Declarations **********/
int frame;			
float Y_min;		
float residualth =0.6;
float *Y_8;						
complex *Y_minus1;
complex *Y_minus2;
complex *Y_current;
complex *f_switch;

/* Choose enhancements */
int mode_4 = 2;						
int mode_5 = 2;		
int choose_enhancement = 4;

 /******************************* Function prototypes *******************************/
void init_hardware(void);    	/* Initialize codec */ 
void init_HWI(void);            /* Initialize hardware interrupts */
void ISR_AIC(void);             /* Interrupt service routine for codec */
void process_frame(void);       /* Frame processing routine */
void basic(void); 
void enhancement1(void); 
void enhancement2(void);
void enhancement3(void);
void enhancement4(void);
void enhancement5(void);
void enhancement6(void); 
void enhancement8(void);
void enhancement10(void); 
float find_min(float a, float b);
float find_max(float a, float b);
           
/********************************** Main routine ************************************/
void main()
{      
  	int k; // used in various for loops
  
	/*  Initialize and zero fill arrays */  

	inbuffer				= (float *) 	calloc(CIRCBUF, sizeof(float));		/* Input array */
    outbuffer				= (float *) 	calloc(CIRCBUF, sizeof(float));		/* Output array */
	inframe					= (float *) 	calloc(FFTLEN, 	sizeof(float));		/* Array for processing*/
    outframe				= (float *) 	calloc(FFTLEN, 	sizeof(float));		/* Array for processing*/
    inwin					= (float *) 	calloc(FFTLEN, 	sizeof(float));		/* Input window */
    outwin					= (float *) 	calloc(FFTLEN, 	sizeof(float));		/* Output window */
    M1						= (float *) 	calloc(NFREQ, 	sizeof(float));		/* M1 */
    M2						= (float *) 	calloc(NFREQ, 	sizeof(float));		/* M2 */
    M3						= (float *) 	calloc(NFREQ, 	sizeof(float));		/* M3 */
    M4						= (float *) 	calloc(NFREQ, 	sizeof(float));		/* M4 */
    Pt						= (float *) 	calloc(NFREQ, 	sizeof(float));		/* P(t) */
	Pt_prev					= (float *) 	calloc(NFREQ, 	sizeof(float));		/* P(t-1) */
	X_magnitude				= (float *) 	calloc(NFREQ, 	sizeof(float));		/* Magnitude spectrum */
	low_pass_noise 			= (float *) 	calloc(NFREQ, 	sizeof(float));		/* Low pass noise estimate */
	low_pass_noise_prev 	= (float *) 	calloc(NFREQ,	sizeof(float));		/* Previous low pass noise estimate */
	noise_estimate   		= (float *) 	calloc(NFREQ, 	sizeof(float)); 	/* Noise estimate array */
	intermediate			= (complex *) 	calloc(FFTLEN, 	sizeof(complex));	/* Complex array */
	
	/* Enhancement 8 */
	Y_8 					= (float *) 	calloc(NFREQ, 	sizeof(float));
 	Y_minus1 				= (complex *) 	calloc(FFTLEN, 	sizeof(complex)); 	/* Array holding previous Y(n-1) frame output*/
	Y_minus2 				= (complex *) 	calloc(FFTLEN, 	sizeof(complex)); 	/* Array holding Y(n-2) frame output */
 	f_switch 				= (complex * ) 	calloc(FFTLEN, 	sizeof(complex)) ; 	/* Array to facilitate the pointer switch */
	Y_current 				= (complex * ) 	calloc(FFTLEN, 	sizeof(complex)) ; 	/* Array holding current output */
	
	/* initialize board and the audio port */
  	init_hardware();
  
  	/* initialize hardware interrupts */
  	init_HWI();    
  
	/* initialize algorithm constants */  
	kappa 		= exp(-TFRAME/tau); /* Kappa variable for enhancements 1,2 */
	kappa_noise = exp(-TFRAME/noise_tau3); /* Kappa variable for enhancement 3 */
	kappa_4 	= exp(-TFRAME/tau_4);
                       
  	for (k=0;k<FFTLEN;k++)
	{                           
	inwin[k]  = sqrt((1.0-WINCONST*cos(PI*(2*k+1)/FFTLEN))/OVERSAMP);
	outwin[k] = inwin[k]; 
	} 
  	ingain=INGAIN;
  	outgain=OUTGAIN;        
 							
  	/* main loop, wait for interrupt */  
  	while(1) 	process_frame();
}
    
/********************************** init_hardware() *********************************/  
void init_hardware()
{
    /* Initialize the board support library, must be called first */
    DSK6713_init();
    
    // Start the AIC23 codec using the settings defined above in config 
    H_Codec = DSK6713_AIC23_openCodec(0, &Config);

	/* Function below sets the number of bits in word used by MSBSP (serial port) for 
	receives from AIC23 (audio port). We are using a 32 bit packet containing two 
	16 bit numbers hence 32BIT is set for  receive */
	MCBSP_FSETS(RCR1, RWDLEN1, 32BIT);	

	/* Configures interrupt to activate on each consecutive available 32 bits 
	from Audio port hence an interrupt is generated for each L & R sample pair */	
	MCBSP_FSETS(SPCR1, RINTM, FRM);

	/* These commands do the same thing as above but applied to data transfers to the 
	audio port */
	MCBSP_FSETS(XCR1, XWDLEN1, 32BIT);	
	MCBSP_FSETS(SPCR1, XINTM, FRM);	
	

}
/********************************** init_HWI() **************************************/ 
void init_HWI(void)
{
	IRQ_globalDisable();			// Globally disables interrupts
	IRQ_nmiEnable();				// Enables the NMI interrupt (used by the debugger)
	IRQ_map(IRQ_EVT_RINT1,4);		// Maps an event to a physical interrupt
	IRQ_enable(IRQ_EVT_RINT1);		// Enables the event
	IRQ_globalEnable();				// Globally enables interrupts

}
        
/******************************** process_frame() ***********************************/  
void process_frame(void)
{
	int k, m; 
	int io_ptr0; 

	/* work out fraction of available CPU time used by algorithm */    
	cpufrac = ((float) (io_ptr & (FRAMEINC - 1)))/FRAMEINC;  
		
	/* wait until io_ptr is at the start of the current frame */ 	
	while((io_ptr/FRAMEINC) != frame_ptr); 
	
	/* then increment the framecount (wrapping if required) */ 
	if (++frame_ptr >= (CIRCBUF/FRAMEINC)) frame_ptr=0;
	
	/*Check for time condition and rotate buffers when necessary */
	if (++f_index > (ROTATE_TIME/FRAMEINC*FSAMP)){ 
		 	
 		f_index = 0;
 		tmp_M4 = M4;
 		M4 = M3;
 		M3 = M2;
 		M2 = M1;
 		M1 = tmp_M4;	
 	}
 	
 	/* save a pointer to the position in the I/O buffers (inbuffer/outbuffer) where the 
 	data should be read (inbuffer) and saved (outbuffer) for the purpose of processing */
 	io_ptr0=frame_ptr * FRAMEINC;
	
	/* copy input data from inbuffer into inframe (starting from the pointer position) */ 
	 
	m=io_ptr0;
    for (k=0;k<FFTLEN;k++)
	{                           
		inframe[k] = inbuffer[m] * inwin[k]; 
		intermediate[k] = cmplx(inframe[k],0); //copy samples from inframe to intermediate array
		if (++m >= CIRCBUF) m=0; /* wrap if required */
	} 
	
	/************************* DO PROCESSING OF FRAME  HERE **************************/
	 
	/* Perform FFT calculation over the intermediate array */
	fft(FFTLEN,intermediate); 
	
	/* Calculate the magnitude spectrum of the input signal |X(w)| */
	/* loop over the number of frequency bins */	
	for (k=0;k<NFREQ;k++) 
	{
		X_magnitude[k] = cabs(intermediate[k]); 
	}
	
	/* Implement software switches */
	switch(choose_enhancement)
	{
		case(0):
		basic();
		break; 
		
		case(1):
		enhancement1();
		break;
		
		case(2):
		enhancement2();
		break; 
		
		case(3):
		enhancement3();
		break;
		
		case(4):
		enhancement4(); 
		break; 
		
		case(5):
		enhancement5();
		break; 
		
		case(6):
		enhancement6(); 
		break; 		
		
		case(10): 
		enhancement10();
		break; 
		
	}						      									
							      									
	/* Perform inverse FFT */
	if(choose_enhancement != 8){
		ifft(FFTLEN,intermediate); 
		/* Write to the output frame */
		for (k=0;k<FFTLEN;k++)
		{
			outframe[k] = intermediate[k].r; 
		}
	}
	 
	/********************************************************************************/
	
    /* multiply outframe by output window and overlap-add into output buffer */  
                           
	m=io_ptr0;
    
    for (k=0;k<(FFTLEN-FRAMEINC);k++) 
	{    										/* this loop adds into outbuffer */                       
	  	outbuffer[m] = outbuffer[m]+outframe[k]*outwin[k];   
		if (++m >= CIRCBUF) m=0; /* wrap if required */
	}         
    for (;k<FFTLEN;k++) 
	{                           
		outbuffer[m] = outframe[k]*outwin[k];   /* this loop over-writes outbuffer */        
	    m++;
	}	                                   
}        
/*************************** INTERRUPT SERVICE ROUTINE  *****************************/

// Map this to the appropriate interrupt in the CDB file
   
void ISR_AIC(void)
{       
	short sample;
	/* Read and write the ADC and DAC using inbuffer and outbuffer */
	
	sample = mono_read_16Bit();
	inbuffer[io_ptr] = ((float)sample)*ingain;
		/* write new output data */
	mono_write_16Bit((int)(outbuffer[io_ptr]*outgain)); 
	
	/* update io_ptr and check for buffer wraparound */    
	
	if (++io_ptr >= CIRCBUF) io_ptr=0;
}

/************************************************************************************/
void basic(void)
{
	int k;
	for(k=0;k<NFREQ;k++)
	{
		/* Select the minimum */
		if (f_index == 0){
			M1[k] = X_magnitude[k]; /* For the first frame set the minimum as the current sample */
		}
		else{
			M1[k] = find_min(M1[k],X_magnitude[k]); /* For any other frame compare the current sample with the current minimum */
		}
		
		/* Find the minimum out of all Mi */
		global_min = find_min( find_min(M1[k],M2[k]), find_min(M3[k],M4[k]) ); 

		/* Calculate the noise estimate considering the global minimum magnitude and
		estimate the noise spectrum using the correction factror alpha */
		noise_estimate[k] = alpha_basic*global_min; 
	}
	
	/* Subtract the noise spectrum */
	for (k=0;k<NFREQ;k++) 
	{
		/* Calculate frequency dependent gain factor */
		g = find_max((lambda*1.0), (1-(noise_estimate[k]/X_magnitude[k]))); 
		
		/* Zero-phase filtering */
		/* Use symmetry in FFT to fill the values [FFTLEN-k] */
		intermediate[k] = rmul(g, intermediate[k]);
		intermediate[FFTLEN - k].r = intermediate[k].r; 
		intermediate[FFTLEN - k].i =- intermediate[k].i;
	}
	
}
	/********************************************************************************/
	
void enhancement1(void)
{
	/* Usimg low pass filter version of |X(w)| */
	int k; 
	
	for(k=0;k<NFREQ;k++)
	{
		Pt[k] = (1 - kappa)*X_magnitude[k] + kappa*Pt[k]; 
		
		if (f_index == 0){
			M1[k] = Pt[k]; 
		}
		else{
			M1[k] = find_min(M1[k],Pt[k]); 
		}
		
		global_min = find_min( find_min(M1[k],M2[k]), find_min(M3[k],M4[k]) ); 
		noise_estimate[k] = alpha1*global_min; 
	}
	
	/* Subtract noise spectrum */
	for (k=0;k<NFREQ;k++) 
	{	
		g = find_max((lambda*1.0), (1-(noise_estimate[k]/Pt[k]))); 
		
		/* Zero-phase filtering */
		intermediate[k] = rmul(g, intermediate[k]);
		intermediate[FFTLEN - k].r = intermediate[k].r; 
		intermediate[FFTLEN - k].i =- intermediate[k].i;
	}
}

void enhancement2(void)
{
	/* Usimg low pass filter version of |X(w)| */
	int k; 
	
	for(k=0;k<NFREQ;k++)
	{
		Pt[k] = sqrt(((1 - kappa)*X_magnitude[k]*X_magnitude[k] + kappa*Pt_prev[k]*Pt_prev[k]));
	
		if (f_index == 0){
			M1[k] = Pt[k]; 
		}
		else{
			M1[k] = find_min(M1[k],Pt[k]); 
		}
		
		global_min = find_min( find_min(M1[k],M2[k]), find_min(M3[k],M4[k]) ); 
		noise_estimate[k] = alpha1*global_min; 
	}

	/* Subtract noise spectrum */
	for (k=0;k<NFREQ;k++) 
	{		
		g = find_max((lambda*1.0), (1-(noise_estimate[k]/Pt[k]))); 
		
		/* Zero-phase filtering */
		intermediate[k] = rmul(g, intermediate[k]);
		intermediate[FFTLEN - k].r = intermediate[k].r; 
		intermediate[FFTLEN - k].i =- intermediate[k].i;
	}
	Pt_prev = Pt;
}

void enhancement3(void)
{
	/* Usimg low pass filter version of |X(w)| */
	int k; 
	float alpha1;  
	
	for(k=0;k<NFREQ;k++)
	{
		if (f_index == 0){
			M1[k] = X_magnitude[k]; 
		}
		else{
			M1[k] = find_min(M1[k],X_magnitude[k]); 
		}
		
		global_min = find_min( find_min(M1[k],M2[k]), find_min(M3[k],M4[k]) ); 
		noise_estimate[k] = alpha1*global_min; 
		
		/* Low pass filter the noise to avoid discontinuities when the minimization buffers rotate */
		low_pass_noise[k] = (1-kappa_noise)*noise_estimate[k] + kappa_noise*low_pass_noise_prev[k];
	}
	
	/* Subtract noise spectrum */
	for (k=0;k<NFREQ;k++) 
	{	
		g = find_max((lambda*1.0), (1-(low_pass_noise[k]/X_magnitude[k]))); 
		
		/* Zero-phase filtering */
		intermediate[k] = rmul(g, intermediate[k]);
		intermediate[FFTLEN - k].r = intermediate[k].r; 
		intermediate[FFTLEN - k].i =- intermediate[k].i;
	}
	low_pass_noise_prev = low_pass_noise;
}

void enhancement4(void){
	int k; 
	
	for(k=0;k<NFREQ;k++)
	{
		Pt[k] = sqrt(((1-kappa_4)*(X_magnitude[k]*X_magnitude[k]) + kappa_4*Pt_prev[k]*Pt_prev[k]));

		if (f_index == 0){
			M1[k] = Pt[k]; 
		}
		else{
			M1[k] = find_min(M1[k],Pt[k]); 
		}
		global_min = find_min( find_min(M1[k],M2[k]), find_min(M3[k],M4[k]) ); 
		noise_estimate[k] = alpha1*global_min; 
	}
	for(k=0; k<NFREQ; k++)
	{
		switch(mode_4)
			{
			case 1:
					g = find_max(lambda*(noise_estimate[k]/X_magnitude[k]), (1-(noise_estimate[k]/X_magnitude[k]))); 
			break; 
		
			case 2:
					g = find_max(lambda*(Pt[k]/X_magnitude[k]), (1-(noise_estimate[k]/X_magnitude[k]))); 
			break;
		
			case 3:
					g = find_max((lambda*(noise_estimate[k]/Pt[k])), (1-(noise_estimate[k]/Pt[k]))); 
			break; 
		
			case 4:
					g = find_max(lambda, (1-(noise_estimate[k]/Pt[k]))); 
			break;
		}
		intermediate[k] = rmul(g, intermediate[k]);
		intermediate[FFTLEN - k].r = intermediate[k].r; 
		intermediate[FFTLEN - k].i =- intermediate[k].i;
	}		
	Pt_prev = Pt;
}

void enhancement5(void){
	int k; 
	float temporary;
	
	for(k=0;k<NFREQ;k++)
	{
		Pt[k] = sqrt(((1-kappa_4)*(X_magnitude[k]*X_magnitude[k]) + kappa_4*Pt_prev[k]*Pt_prev[k])); 

		if (f_index == 0){
			M1[k] = Pt[k]; 
		}
		else{
			M1[k] = find_min(M1[k],Pt[k]); 
		}
		global_min = find_min( find_min(M1[k],M2[k]), find_min(M3[k],M4[k]) ); 
		noise_estimate[k] = alpha1*global_min; 
	}

	for(k=0; k<NFREQ; k++)
	{
		switch(mode_5){
			case 1:
					temporary= sqrt(1 - (noise_estimate[k]*noise_estimate[k])/(X_magnitude[k]*X_magnitude[k]));
					g = find_max((lambda*(noise_estimate[k]/X_magnitude[k])), temporary); 
			break; 
		
			case 2:
					temporary = sqrt(1 - (noise_estimate[k]*noise_estimate[k])/(X_magnitude[k]*X_magnitude[k]));
					g = find_max((lambda*(Pt[k]/X_magnitude[k])), temporary); 
			break;
		
			case 3:	
					temporary = sqrt(1 - (noise_estimate[k]*noise_estimate[k])/(Pt[k]*Pt[k]));
					g = find_max((lambda*(noise_estimate[k]/Pt[k])), temporary); 
			break; 
		
			case 4:
					temporary = sqrt(1-(noise_estimate[k]*noise_estimate[k])/(Pt[k]*Pt[k]));
					g = find_max(lambda*1.0, temporary); 
			break;
		}	
		intermediate[k] = rmul(g, intermediate[k]);
		intermediate[FFTLEN - k].r = intermediate[k].r; 
		intermediate[FFTLEN - k].i =- intermediate[k].i;
	}

	Pt_prev = Pt;
}

void enhancement6(void)
{
	int k;
	float SNR, alpha6; 
	
	for(k=0;k<NFREQ;k++)
	{
		/* Select the minimum */
		if (f_index == 0){
			M1[k] = X_magnitude[k]; 
		}
		else{
			M1[k] = find_min(M1[k],X_magnitude[k]);
		}
		
		global_min = find_min( find_min(M1[k],M2[k]), find_min(M3[k],M4[k]) ); 

		SNR = 20*log10f((X_magnitude[k]/global_min)); 
		if(SNR < -5) 		alpha6 = 4.75; 
		else if(SNR > 20)	alpha6 = 1.0; 
		else				alpha6 = (4.0 - 0.15*SNR); 
		
		noise_estimate[k] = alpha6*global_min; 
	}
	
	/* Subtract noise spectrum */
	for (k=0;k<NFREQ;k++) 
	{	
		g = find_max((lambda*1.0), (1-(noise_estimate[k]/X_magnitude[k]))); 
		
		/* Zero-phase filtering */
		intermediate[k] = rmul(g, intermediate[k]);
		intermediate[FFTLEN - k].r = intermediate[k].r; 
		intermediate[FFTLEN - k].i =- intermediate[k].i;
	}
}

void enhancement8(void){
	int k; 
		for(k=0;k<NFREQ;k++)
	{
		/* Select the minimum */
		if (f_index == 0){
			M1[k] = X_magnitude[k]; /* For the first frame set the minimum as the current sample */
		}
		else{
			M1[k] = find_min(M1[k],X_magnitude[k]); /* For any other frame compare the current sample with the current minimum */
		}
		
		/* Find the minimum out of all Mi */
		global_min = find_min( find_min(M1[k],M2[k]), find_min(M3[k],M4[k]) ); 
		noise_estimate[k] = alpha_basic*global_min; 
	}
	
	/* Subtract noise spectrum */
	for (k=0;k<NFREQ;k++) 
	{	
		g = find_max((lambda*1.0), (1-(noise_estimate[k]/X_magnitude[k]))); 
		/* Zero-phase filtering */
		intermediate[k].r 		*= g; 
		intermediate[k].i       *= g;
		intermediate[FFTLEN - k].r = intermediate[k].r; 
		intermediate[FFTLEN - k].i = -intermediate[k].r; 
		
		Y_8[k] = g*X_magnitude[k];
		Y_current[k]= intermediate[k];
		Y_current[FFTLEN-k] = intermediate[FFTLEN - k];
		
		if (noise_estimate[k]/X_magnitude[k] >  residualth) {
			Y_min = Y_8[k];
			frame = 0;
			if (cabs(Y_minus1[k]) <  Y_min) {
				Y_min = cabs(Y_minus1[k]);
				frame = -1;
			}
			if (cabs(Y_minus2[k]) <  Y_min) {
				Y_min = cabs(Y_minus2[k]);
				frame = -2;
			}
			if(frame!=0){
				switch(frame){
					case(-1):
						Y_current[k].r = Y_minus1[k].r;
						Y_current[k].i = Y_minus1[k].i;
						Y_current[FFTLEN-k].r = Y_current[k].r;
						Y_current[FFTLEN-k].i = -Y_current[k].i;
						break;
					case(-2):
						Y_current[k].r = Y_minus2[k].r;
						Y_current[k].i = Y_minus2[k].i;
						Y_current[FFTLEN-k].r = Y_current[k].r;
						Y_current[FFTLEN-k].i = -Y_current[k].i;
				break;
				}
			}
		}
	}
	/* Swap pointers using auxiliary pointer */
	f_switch=Y_minus2 ; 
 	Y_minus2=Y_minus1 ;
 	Y_minus1=intermediate ;
 	intermediate=f_switch ;
 	
 	/* Computes the IFFT */
 	ifft (FFTLEN, Y_current ) ; 
 	for ( k=0;k<FFTLEN; k++)
 	{
 		outframe[k] = Y_current[k].r ; /* copy input straight into output */
	 }
}

void enhancement10(void)
{
	int k; 
	float SNR, alpha10; 

	for(k=0;k<NFREQ;k++)
	{
		/* Select the minimum */
		if (f_index == 0){
			M1[k] = X_magnitude[k]; /* For the first frame set the minimum as the current sample */
		}
		else{
			M1[k] = find_min(M1[k],X_magnitude[k]); /* For any other frame compare the current sample with the current minimum */
		}
		
		/* Find the minimum out of all Mi */
		global_min = find_min( find_min(M1[k],M2[k]), find_min(M3[k],M4[k]) ); 
		
		SNR = (X_magnitude[k]/global_min); 
		if(SNR < -0.5) 		alpha10 = 4.75; 
		else if(SNR > 10)	alpha10 = 1.0; 
		else				alpha10 = (3.5 + 2.5*SNR); 
								
		noise_estimate[k] = alpha10*global_min;  	 				  
	}
	
	/* Subtract noise spectrum */
	for (k=0;k<NFREQ;k++) 
	{	
		g = find_max((lambda*1.0), (1-(noise_estimate[k]/X_magnitude[k]))); 
		
		/* Zero-phase filtering */
		intermediate[k] = rmul(g, intermediate[k]);
		intermediate[FFTLEN - k].r = intermediate[k].r; 
		intermediate[FFTLEN - k].i =- intermediate[k].i;
	}		
}

/* Function to calculate the minimum between the two input arguments */
float find_min(float a, float b)
{
	if (a <= b)
	{
		return a;
	}
	else
	{
		return b; 
	}
}

/* Function to calculate the maximum between the two input arguments */
float find_max(float a, float b)
{
	if (a >= b)
	{
		return a;
	}
	else
	{
		return b; 
	}
}