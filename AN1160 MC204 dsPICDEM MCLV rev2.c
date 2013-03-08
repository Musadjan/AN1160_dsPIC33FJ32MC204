/******************************************************************************/
//
//							 Software License Agreement
//
// CODE OWNERSHIP AND DISCLAIMER OF LIABILITY

// Microchip Technology Incorporated ("Microchip") retains 
// all ownership and intellectual property rights in the code 
// accompanying this message and in all derivatives hereto.  
// You may use this code, and any derivatives created by any 
// person or entity by or on your behalf, exclusively with 
// Microchip’s proprietary products.  Your acceptance and/or 
// use of this code constitutes agreement to the terms and 
// conditions of this notice.

// CODE ACCOMPANYING THIS MESSAGE IS SUPPLIED BY MICROCHIP "AS IS".  
// NO WARRANTIES, WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING, 
// BUT NOT LIMITED TO, IMPLIED WARRANTIES OF NON-INFRINGEMENT, 
// MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE APPLY TO 
// THIS CODE, ITS INTERACTION WITH MICROCHIP’S PRODUCTS, COMBINATION 
// WITH ANY OTHER PRODUCTS, OR USE IN ANY APPLICATION. 

// YOU ACKNOWLEDGE AND AGREE THAT, IN NO EVENT, SHALL MICROCHIP BE 
// LIABLE, WHETHER IN CONTRACT, WARRANTY, TORT (INCLUDING NEGLIGENCE
// OR BREACH OF STATUTORY DUTY), STRICT LIABILITY, INDEMNITY, CONTRIBUTION,
// OR OTHERWISE, FOR ANY INDIRECT, SPECIAL, PUNITIVE, EXEMPLARY, INCIDENTAL 
// OR CONSEQUENTIAL LOSS, DAMAGE, FOR COST OR EXPENSE OF ANY KIND WHATSOEVER
// RELATED TO THE CODE, HOWSOEVER CAUSED, EVEN IF MICROCHIP HAS BEEN ADVISED
// OF THE POSSIBILITY OR THE DAMAGES ARE FORESEEABLE.  TO THE FULLEST EXTENT
// ALLOWABLE BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL CLAIMS IN ANY WAY 
// RELATED TO THIS CODE, SHALL NOT EXCEED THE PRICE YOU PAID DIRECTLY TO 
// MICROCHIP SPECIFICALLY TO HAVE THIS CODE DEVELOPED.

// You agree that you are solely responsible for testing the code and
// determining its suitability.  Microchip has no obligation to modify,
// test, certify, or support the code.

/*****************************************************************************/
//	File:		AN1160 MC204 dsPICDEM MCLV.c
//
// This program commutates a sensor less BLDC motor using closed PI loop.
// The circuit and operation is as depicted in AN1160. 
//
//	Written By:		Daniel Torres, 
//				Microchip Technology Inc
//						
// 
// The following files should be included in the MPLAB project:
//
//		AN1160 MC204 dsPICDEM MCLV.c	-- Main source code file
//      RTDMUSER.h                      -- RTDM user definitions header file
//      RTDM.h                          -- RTDM header file
//      RTDM.c                          -- RTDM source code file
//		p33FJ12MC204.h		            -- MPLAB-C30  dsPIC33FJ12MC202 processor header
//		p33FJ12MC204.gld		        -- Linker script file
//				
//
/*****************************************************************************/
//
// Revision History
//
// 12/3/07  -- First Version Close Loop with BEMF sensing using the ADC
// 12/10/07 -- Adding comments and modifying global variables names
// 2/22/08 -- Spell Checking, Dead Variable Clean up, Ramp up bug
// 5/5/08   -- DMCI with real-time data monitor added
// 6/04/08 -- SW migrated to the dsPICDEM MCLV board
// 6/17/08 -- Commens addded, PI coefficients recalculated,
/*****************************************************************************/

#include "p33FJ32MC204.h"
#include "dsp.h"

/********************Setting Configuration Bits *********************************/
/* Internal FRC Oscillator */
_FOSCSEL(FNOSC_FRC);

/*  Clock Switching is enabled and 
**  Fail Safe Clock Monitor is disabled
**	OSC2 Pin Function: OSC2 is Clock Output
**	Primary Oscillator Mode: Disabled */
_FOSC(FCKSM_CSECMD & OSCIOFNC_ON  & POSCMD_NONE); 

/*Watchdog Timer Enabled/disabled by user software*/
_FWDT(FWDTEN_OFF);  

/* PWM Output Pin Reset: Controlled by Port Register
** PWM High Side Polarity: Active High
** PWM Low Side Polarity: Active High */
_FPOR(PWMPIN_ON & HPOL_ON & LPOL_ON);

/* Background Debug Enable Bit: Device will Reset in user mode
** Debugger/Emulator Enable Bit: Reset in operational mode
** JTAG Enable Bit: JTAG is disabled
** ICD communication channel select bits: communicate on PGC3/EMUC3 and PGD3/EMUD3 **/
//_FICD(BKBUG_ON & COE_ON & JTAGEN_OFF & ICS_PGD3);
	
/*****************************SYSTEM DEFINES***********************************/
/*FREQUENCY SYSTEM DEFINES*/
#define FCY  29491200			//FRC w/PLL x16
#define MILLISEC FCY/29491		//1 mSec delay constant
#define FPWM 20000				//20KHz PWM Freq
#define S3	 !PORTBbits.RB4		//Defines for the Push Buttons status
#define S2       !PORTAbits.RA8
/******************************************************************************
* OPEN LOOP DEFINITION
* If defined then the compiler will add the PID controller for speed
*******************************************************************************/
#define CLOSELOOPMODE

/********************* Motor Control Definitions *********************************/
/*START-UP SEQUENCE PARAMETERS*/	// Maximum PWM pulses applied to motor 						
#define MAX_PWM_TICKS 1230 		//for detecting initial rotor position
#define RAM_UP_DELAY  0    	 	// Delay for the ramp up sequence, expressed in millisecond
#define MAX_DUTY_CYCLE 1400  	// 100% duty cycle P1DC1=P1DC2=P1DC3 = 1469
#define MIN_DUTY_CYCLE 40	 	// 2.7% duty cycle P1DC1=P1DC2=P1DC3 = 72
#define PHASE_ADVANCE_DEGREES 25 //Phase advance angles to get the best motor performance
#define BLANKING_COUNT 5		//Blanking count expressed in PWM periods used to avoid false zero-crossing detection after commutating motor
#define POLEPAIRS 12			 	// Number of pole pairs of the motor
/*PI CONTROLLER PARAMETER, CONVERSION SPEED FACTOR */
// SPEEDMULT = (TIMER2 TIMEBASE/PWM TIMEBASE) * MAX_DUTY_CYLE*2	
#define SPEEDMULT	94016			// Factor used to calculated speed PWM TIMEBASE = FCY/2, TIMER2 TIMEBASE / FCY/64
#define INDEX		1				// Commutation base index
#define DEBOUNCE_DELAY 	20			// Push button debounce delay, expressed in millisecond


/* Six-Step Commutation States*/
/*PHASE A is MI,  PHASE B is M2, PHASE C is M3 in the dsPICDEM MCLV board*/
/* State 0x2001 Connect the PHASE A to -Vbus & PHASE C to +Vbus */
/* State 0x2004 Connect the PHASE B to -Vbus & PHASE C to +Vbus */
/* State 0x0204 Connect the PHASE B to -Vbus & PHASE A to +Vbus */
/* State 0x0210 Connect the PHASE C to -Vbus & PHASE A to +Vbus */
/* State 0x0810 Connect the PHASE C to -Vbus & PHASE B to +Vbus */
/* State 0x0801 Connect the PHASE A to -Vbus & PHASE B to +Vbus */
const unsigned int PWM_STATE[]	=	{0x0000,0x2001,0x2004,0x0204,0x0210,0x0810,0x0801,0x0000};

/*AND & OR operators for masking the active BEMF signal*/
const unsigned int ADC_MASK[8]	=   {0x0000,0x0002,0x0001,0x0004,0x0002,0x0001,0x0004,0x0000};
const unsigned int ADC_XOR[8]	=   {0x0000,0x0000,0xFFFF,0x0000,0xFFFF,0x0000,0xFFFF,0x0000};

/*BEMF Majority Function Filter values*/
const unsigned char ADC_BEMF_FILTER[64]=
									{0x00,0x02,0x04,0x06,0x08,0x0A,0x0C,0x0E,0x10,0x12,0x14,0x16,0x18,0x1A,0x1C,0x1E,
									 0x20,0x22,0x24,0x26,0x28,0x2A,0x2C,0x2E,0x01,0x01,0x01,0x36,0x01,0x3A,0x3C,0x3E,
									 0x00,0x02,0x04,0x06,0x08,0x0A,0x0C,0x0E,0x01,0x01,0x01,0x16,0x01,0x1A,0x1C,0x1E,
									 0x01,0x01,0x01,0x26,0x01,0x2A,0x2C,0x2E,0x01,0x01,0x01,0x36,0x01,0x3A,0x3C,0x3E};
/*Application Flags to indicate the motor status*/
struct {
			unsigned RunMotor : 	  1;
                        unsigned RotorAlignment : 1;
			unsigned unused :		  14;     
		}Flags;        

/*Boolean variables used to save the comparison between the phases and the neutral point*/
struct {
		
   			unsigned PhaseAOutput : 	1;
			unsigned PhaseBOutput : 	1;
			unsigned PhaseCOutput : 	1;
			unsigned unused 	  :		13;

		}Comparator;

/********************* Motor Control Varaibles *********************************/		
unsigned int  PWMticks;
unsigned char CommState;
unsigned char ADCCommState;
unsigned char adcBackEMFFilter;
unsigned int PhaseAdvance = 0;
unsigned char BlankingCounter;

unsigned int MotorNeutralVoltage;
unsigned int MotorPhaseA;
unsigned int MotorPhaseB;
unsigned int MotorPhaseC;
unsigned int ComparatorOutputs;
unsigned int CommutationStatus;
unsigned int DesiredPWMDutyCycle;
unsigned int CurrentPWMDutyCycle;
unsigned char RampUpCommState;

unsigned int Timer2Value;
unsigned int Timer2Average;
unsigned int Timer1Value;


/********************* PID Varibles  *********************************/
#ifdef CLOSELOOPMODE
unsigned int ReferenceSpeed;
int DesiredSpeed, CurrentSpeed;
unsigned int SpeedControl_P = 2500;		// The P term for the PI speed control loop
unsigned int SpeedControl_I = 256;		// The I term for the PI speed control loop
unsigned int SpeedControl_D = 0;		// The I term for the PI speed control loop

fractional PIDGainCoefficients[3];	//Control gains for the speed control loop
fractional abcCoefficients[3] __attribute__ ((space(xmemory),__aligned__(4)));
fractional controlHistory[3] __attribute__ ((space(ymemory),__aligned__(4)));
tPID PIDStructure;					// PID Structure			
#endif

/********************* Function Definitions *********************************/
void InitMCPWM(void);
void InitADC10(void);
void InitTMR2(void);
void InitTMR1(void);
void PreCommutationState(void);
void SpeedPILoopController(void);
void OpenLoopController(void);
void DelayNmSec(unsigned int N);

//-----------------------------Musadjans-------------------------------//
 int Sub_timer;

/******************************************************************************
* Function:     main(void)
*
* Output:		None
*
* Overview:		Main function used to init the ADC, PWM and TIMER2 modules. 
*				It also inits the global variables used in the interrupts and 
*				PI controller.
*				The main task executed here is to start and stop the motor 
*				as well as setting the ramp-up initial parameters to 
*				spin the motor
*
* Note:			None
*******************************************************************************/
int main(void)
{

	// Configure Oscillator to operate the device at 30Mhz
	// Fosc= Fin*M/(N1*N2), Fcy=Fosc/2
	// Fosc= 7.37*32/(2*2)= 58.96Mhz for 7.37 input clock
	/****************** Clock definitions *********************************/
	PLLFBD=30;					// M=32
	CLKDIVbits.PLLPOST=0;		// N1=2
	CLKDIVbits.PLLPRE=0;		// N2=2
	OSCTUN=0;					// Tune FRC oscillator, if FRC is used

	// Disable Watch Dog Timer
	RCONbits.SWDTEN=0;

	// Clock switch to incorporate PLL
	__builtin_write_OSCCONH(0x01);		// Initiate Clock Switch to
										// FRC with PLL (NOSC=0b001)
	__builtin_write_OSCCONL(0x01);		// Start clock switching
	while (OSCCONbits.COSC != 0b001);	// Wait for Clock switch to occur

	// Wait for PLL to lock
	while(OSCCONbits.LOCK!=1) {};
		   
	/****************** PID init **************************************/	
#ifdef CLOSELOOPMODE
	ReferenceSpeed = (MIN_DUTY_CYCLE*2)/3;								
	
	// load the PID gain coeffecients into an array;
	PIDGainCoefficients[0] = SpeedControl_P;
	PIDGainCoefficients[1] = SpeedControl_I;
	PIDGainCoefficients[2] = SpeedControl_D;				

	// Initialize the PIDStructure variable before calculation the K1, K2, and K3 terms	
	PIDStructure.abcCoefficients = abcCoefficients;	
	PIDStructure.controlHistory = controlHistory;
	PIDCoeffCalc(PIDGainCoefficients, &PIDStructure);
	// initialize control history
	PIDInit(&PIDStructure);
	PIDStructure.controlOutput = MIN_DUTY_CYCLE;
	/****************************************************************/							
#endif	
	
	/****************** I/O port Init *********************************/
	/* Configuring Digital PORTS multiplexed with PWMs as outputs*/
	LATBbits.LATB10 = 0;	//PWM1H3
	TRISBbits.TRISB10 = 0;
	LATBbits.LATB11 = 0;	//PWM1L3
	TRISBbits.TRISB11 = 0;
	LATBbits.LATB12 = 0;	//PWM1H2
	TRISBbits.TRISB12 = 0;
	LATBbits.LATB13 = 0;	//PWM1L2
	TRISBbits.TRISB13 = 0;	
	LATBbits.LATB14 = 0;	//PWM1H1
	TRISBbits.TRISB14 = 0;	
	LATBbits.LATB15 = 0;	//PWM1L1
	TRISBbits.TRISB15 = 0;	

	/*Push Buttons ports*/
	LATBbits.LATB4 = 0;
	TRISBbits.TRISB4 = 1;
	LATAbits.LATA8 = 0;
	TRISAbits.TRISA8 = 1;
	
	/****************** Functions init *********************************/
	INTCON1bits.NSTDIS = 0;		    // Enabling nested interrupts	
	InitMCPWM();					//Configuring MC PWM module
	InitADC10();					//Configuring ADC
	InitTMR2();						//Configuring TIMER 3, used to measure speed
	InitTMR1();						//Configuring TIMER 1, used for the commutation delay

	Flags.RotorAlignment = 0;		// TURN OFF RAMP UP
	Flags.RunMotor = 0;				// indication the run motor condition
	
	/****************** Infinite Loop *********************************/
	for(;;)
		{	

		while (!S3);					// wait for S3 button to be hit
		while (S3)						// wait till button is released
			DelayNmSec(DEBOUNCE_DELAY);
		
		T2CONbits.TON = 1;				// Start TIMER , enabling speed measurement
		PWM1CON1 = 0x0777;				// enable PWM outputs
			
		/*ROTOR ALIGNMENT SEQUENCE*/
		Flags.RotorAlignment = 1;		// TURN ON rotor alignment sequence
		Flags.RunMotor = 1;				// Indicating that motor is running
		CurrentPWMDutyCycle = MIN_DUTY_CYCLE;	//Init PWM values
		DesiredPWMDutyCycle = MIN_DUTY_CYCLE;	//Init PWM values
		PWMticks = 0;					//Init Rotor aligment counter
		
		/************* Rotor alignment sequence and ramp-up sequence ************/
		for(RampUpCommState=1;RampUpCommState<7;RampUpCommState++)
			{
            
			while(++PWMticks<MAX_PWM_TICKS)
				P1OVDCON=PWM_STATE[RampUpCommState];
			PWMticks = 0;				
			}   
		Flags.RotorAlignment = 0;		// TURN OFF rotor alignment sequence
		PWMticks = MAX_PWM_TICKS+1;		// RAMP UP for breaking the motor IDLE state
		DelayNmSec(RAM_UP_DELAY);		// RAMP UP DELAY
                Sub_timer=Sub_timer++;
		/****************** Motor is running *********************************/
		while(Flags.RunMotor)			// while motor is running
			{
 
		/****************** Stop Motor *********************************/
                    


			//if (S3)							// if S3 is pressed
				//{
                    Sub_timer=0;
                    while ( Sub_timer=Sub_timer++<=600)
				//while (S3)					// wait for key release
					DelayNmSec(DEBOUNCE_DELAY);		
		
				PWM1CON1 = 0x0700;			// disable PWM outputs
  			    P1OVDCON = 0x0000;			// override PWM low.
				Flags.RotorAlignment = 0;	// turn on RAMP UP
				Flags.RunMotor = 0;			// reset run flag
				CurrentPWMDutyCycle = 1;	// Set PWM to the min value
				
				//Initi speed measurement variables & timer
				T2CONbits.TON = 0;	// Stop TIMER2
				TMR2 = 0;			//Clear TIMER2 register
				Timer2Value = 0;
				Timer2Average = 0;

#ifdef CLOSELOOPMODE					
				//Init PI controller variables
				DesiredSpeed = (int)((ReferenceSpeed*3)/2);
				CurrentSpeed = 0;

				// load the PID gain coeffecients into an array;
				PIDGainCoefficients[0] = SpeedControl_P;
				PIDGainCoefficients[1] = SpeedControl_I;
				PIDGainCoefficients[2] = SpeedControl_D;				

				// Initialize the PIDStructure variable before calculation the K1, K2, and K3 terms	
				PIDStructure.abcCoefficients = abcCoefficients;	
				PIDStructure.controlHistory = controlHistory;
				PIDCoeffCalc(PIDGainCoefficients, &PIDStructure);
				// initialize control history
				PIDInit(&PIDStructure);
				PIDStructure.controlOutput = MIN_DUTY_CYCLE;
#endif
								
               // }
			}//end of motor running loop
		}//end of infinite loop

 return 0;
}//end of main function



/******************************************************************************
* Function:     _ADCInterrupt(void) 


*
* Output:		None
*
* Overview:		ADC interrupt used to measure the BEMF signals, reconstruct
* 				the Motor Virtual Neutral Point and compare the BEMF signals
*				against the neutral point reference to detect the zero-crossing 
*				event
*
* Note:			None
*******************************************************************************/
void __attribute__((__interrupt__,auto_psv)) _ADC1Interrupt(void)
{
	ReferenceSpeed = ADC1BUF0;	//ADC CH0 holds the POT value
	if(ReferenceSpeed < (MIN_DUTY_CYCLE*2)/3)
		ReferenceSpeed = (MIN_DUTY_CYCLE*2)/3;
		
	MotorPhaseA = ADC1BUF1;		//ADC CH1 holds the Phase A value
	MotorPhaseB = ADC1BUF2;		//ADC CH2 holds the Phase B value
	MotorPhaseC = ADC1BUF3;		//ADC CH3 holds the Phase C value
	//Reconstrucs Voltage at the  Motor Neutral Point
	MotorNeutralVoltage = (MotorPhaseA + MotorPhaseB + MotorPhaseC)/3; 

	/********************* ADC SAMPLING & BMEF signals comparison ****************/
	if(BlankingCounter > BLANKING_COUNT){
		ComparatorOutputs = 0;						// Precondition all comparator bits as zeros
		if(MotorPhaseA > MotorNeutralVoltage)
			ComparatorOutputs += 1;					// Set bit 0 when Phase C is higher than Neutural
		if(MotorPhaseB > MotorNeutralVoltage)
			ComparatorOutputs += 2;					// Set bit 1 when Phase C is higher than Neutural
		if(MotorPhaseC > MotorNeutralVoltage)
			ComparatorOutputs += 4;					// Set bit 2 when Phase C is higher than Neutral
	}
	AD1CON1bits.DONE = 0;
	IFS0bits.AD1IF = 0;
}


/******************************************************************************
* Function:     _PWMInterrupt(void)
*
* Output:		None
*
* Overview:		PWM reload interrupt used to filter the BEMF signals using the
*				Majority detection filter to detect a valid zero-crossing event
*				if a valid zero-crossing event was detected then PreCommutationState.
*				This function also includes the start-up sequence for detecting
*				the initial rotor position
*
* Note:			None
*******************************************************************************/
void __attribute__((__interrupt__,auto_psv)) _MPWM1Interrupt(void)
{
	//Sets the ADC sampling point according to the PWM duty cycle
	if(CurrentPWMDutyCycle>160)
		P1SECMPbits.SEVTCMP = CurrentPWMDutyCycle>>1;
	else if(CurrentPWMDutyCycle>76)
		P1SECMPbits.SEVTCMP = CurrentPWMDutyCycle>>3;
	else
		P1SECMPbits.SEVTCMP = 0;

	//Ramp-up period to detect the rotor position
	if(Flags.RotorAlignment && Flags.RunMotor)
		{
    	P1DC1=P1DC2=P1DC3=CurrentPWMDutyCycle;
		++PWMticks;
		}
	//Check if the Ramp-up value is disabled, if so starts sensorless operation
	if (++PWMticks>MAX_PWM_TICKS)
		PreCommutationState();

	// Masking the BEMF signals according to the SECTOR in order to determine the ACTIVE BEMF signal
	// XOR operator helps to determine the direction of the upcoming zero-crossing slope
	BlankingCounter++;
	if(BlankingCounter > BLANKING_COUNT){
		if((ComparatorOutputs^ADC_XOR[ADCCommState])& ADC_MASK[ADCCommState])
			adcBackEMFFilter|=0x01;
		
		//Majority detection filter	
		adcBackEMFFilter = ADC_BEMF_FILTER[adcBackEMFFilter];
		if (adcBackEMFFilter&0b00000001)
			PreCommutationState();	
	}
	IFS3bits.PWM1IF = 0;
}


/******************************************************************************
* Function:      _T1Interrupt(void)
*
* Output:		None
*
* Overview:		Here is where the motor commutation occurs, 
*				Timer1 ISR is utilized as the commutation delay used to 
*               commutate the motor windings at the right time
*
* Note:			None
*******************************************************************************/
void __attribute__((__interrupt__,auto_psv)) _T1Interrupt(void)
{

	P1OVDCON=PWM_STATE[ADCCommState];
	
	BlankingCounter = 0;
		
	IFS0bits.T1IF = 0; 		// Clear Timer 1 Interrupt Flag
	T1CONbits.TON = 0;		// Stop TIMER1
	TMR1 = 0;

}


/******************************************************************************
* Function:     PreCommutationState(void)
*
* Output:		None
*
* Overview:		This function measures the 60 and 30 electrical degrees
*				using the TIMER2. The 60 electrical degrees is proportional 
*				to the elpased time between zero-crossing events. 
*				The zero-crossing events occur 30 electrical degrees in advace 
*				of the commutation point. Hence a delay proportional to the 30
*				electrical degrees is added using the TIMER1
*
* Note:			None
*******************************************************************************/
void PreCommutationState(void)
{
	// Calculate the time proportional to the 60 electrical degrees
	T2CONbits.TON = 0;	// Stop TIMER2
	Timer2Average = ((Timer2Average + Timer2Value + TMR2)/3);
	Timer2Value = TMR2;
	TMR2 = 0;
	T2CONbits.TON = 1;	// Start TIMER2
	
	//Calculate the delay in TIMER1 counts proportional to the Phase Adv angle
	PhaseAdvance = ((Timer2Average*PHASE_ADVANCE_DEGREES)/60);
	
	// Calculate the time proportional to the 30 electrical degrees
	// Load the TIMER1 with  the TIMER1 counts porportional to 30 deg	minus the PHASE ADV angle delay
	Timer1Value = (((Timer2Average)>>1)-PhaseAdvance);
	if(Timer1Value>1)
		PR1 = Timer1Value;	
	else
		PR1 = Timer1Value = 1;	
	
	// Start TIMER1	
	T1CONbits.TON = 1;	  	

	//disabling rotor alignment & ramp-up sequence
	PWMticks = 0;
	RampUpCommState = 7;

	//if Motor is runnining in sensorless mode and the PI button is ON on DMIC window 
	//then the PI controller is enabled if the PI button is OFF then motor runs in open loop
	if((!Flags.RotorAlignment) && Flags.RunMotor){

#ifdef CLOSELOOPMODE
		SpeedPILoopController();
#else	
		OpenLoopController();
#endif
	}
		
	// Change The Six-Step Commutation Sector
	adcBackEMFFilter=0;
	if (++ADCCommState>6)
		ADCCommState=1;
}

/******************************************************************************
* Function:     SpeedPILoopController(void)
*
* Output:		None
*
* Overview:		When the PI button is ON on the DMCI window 
*				the motor operates in close loop mode. The Kp and Ki 
*				parameters were determined using the HURST MOTOR shipped with 
*				the MCLV board. These values should be modified according to 
*				the motorand load characteristics.
*
* Note:			None
*******************************************************************************/
#ifdef CLOSELOOPMODE
void SpeedPILoopController(void)
{
	//For the HURST motor the Timer2Avg values
	//TIMER2 counts = 66 AT 100% dutycycle (P1DC1 = 1469)	
	//TIMER2 counts = 1057 AT 4.9% dutycycle (P1DC1 = 72)
	//ADC POT RANGE 0-1024, 1024*3/2 = MAXDUTYCYCLE
	DesiredSpeed = (int)((ReferenceSpeed*3)/2);
		
	// Normalizing TIMER2 counts to electrical RPS expressed in PWM counts	
	// Timer 2 Counts are converted to PWM counts 
	// and then multipied by the number of sector
	// required to complete 1 electrical RPS
	CurrentSpeed = (int)((long)SPEEDMULT/(long)Timer2Average);

	//PID controller
	PIDStructure.controlReference = (fractional)DesiredSpeed;
	PIDStructure.measuredOutput = (fractional)CurrentSpeed;
	PID(&PIDStructure);
	
	//Max and Min Limimts for the PID output
	if (PIDStructure.controlOutput < 0)
		PIDStructure.controlOutput = MIN_DUTY_CYCLE;
	if (PIDStructure.controlOutput > MAX_DUTY_CYCLE) 
		{
		P1DC1 = MAX_DUTY_CYCLE;
		PIDStructure.controlOutput = MAX_DUTY_CYCLE;
		}
	else 
	  	CurrentPWMDutyCycle = PIDStructure.controlOutput;
	P1DC1 =	CurrentPWMDutyCycle;
	P1DC2 = CurrentPWMDutyCycle;
	P1DC3 = CurrentPWMDutyCycle;
}

/******************************************************************************
* Function:     OpenLoopController(void)
*
* Output:		None
*
* Overview:		When the PI button is OFF on the DMCI window 
*				the motor operates in open loop mode. 
*
* Note:			None
*******************************************************************************/
#else
void OpenLoopController(void)
{
	//PWM duty cycle = pot value *3/2
	DesiredPWMDutyCycle = (ReferenceSpeed*3)/2;
	//Update the duty cycle according to the POT value, a POT follower is implemented here
	if(CurrentPWMDutyCycle != DesiredPWMDutyCycle)
		{
		if(CurrentPWMDutyCycle < DesiredPWMDutyCycle)
			CurrentPWMDutyCycle++;		
		if(CurrentPWMDutyCycle > DesiredPWMDutyCycle)
			CurrentPWMDutyCycle--;
		}
	// Max and Min PWM duty cycle limits
	if (CurrentPWMDutyCycle < MIN_DUTY_CYCLE)
		CurrentPWMDutyCycle = MIN_DUTY_CYCLE;
	if (CurrentPWMDutyCycle > MAX_DUTY_CYCLE)
		CurrentPWMDutyCycle = MAX_DUTY_CYCLE;
	//Assigning new duty cycles to the PWM channels
	P1DC1 = CurrentPWMDutyCycle;
	P1DC2 = CurrentPWMDutyCycle;
	P1DC3 = CurrentPWMDutyCycle;

} 
#endif

/******************************************************************************
* Function:     InitADC10(void)
*
* Output:		None
*
* Overview:		Initializes the ADC module to operate in simultaneous mode
*				sampling terminals AN0,AN1, AN2, AN3 using MUX A. The ADC channels are
*				assigned as follows in the MCLV board
*				CH0->AN8  (POT)
*				CH1->AN3 PHASE A
*				CH2->AN4 PHASE B
*				CH3->AN5  PHASE C
* 				ADC is sync with the PWM. ADC is conversion is triggered
*				every time a PWM reload event occurs. Tadc = 84.75 nSec.
*				ADC resulting samples are formatted as unsigned 10-bits 
*				Right-justified 
*
* Note:			None
*******************************************************************************/
void InitADC10(void)
{

	AD1PCFGL = 0xFE00;	//Port pin multiplexed with AN0-AN8 in Analog mode
	
	AD1CON1 = 0x006C;   //ADC is off
						//Continue module operation in Idle mode
						//10-bit, 4-channel ADC operation
						//Data Output Format bits Integer (0000 00dd dddd dddd)
						//011 = Motor Control PWM interval ends sampling and starts conversion
						//Samples CH0, CH1, CH2, CH3 simultaneously when CHPS<1:0> = 1x
						//Sampling begins immediately after last conversion SAMP bit is auto-set.

	AD1CHS123 = 0x0001; //MUX B CH1, CH2, CH3 negative input is VREF-
						//MUX B CH1 positive input is AN0, CH2 positive input is AN1, CH3 positive input is AN2						
						//MUX A CH1, CH2, CH3 negative input is VREF-
						//MUX A CH1 positive input is AN3, CH2 positive input is AN4, CH3 positive input is AN5

	AD1CHS0 = 0x0008;   //MUX B Channel 0 negative input is VREF-
						//MUX B Channel 0 positive input is AN0
						//MUX A Channel 0 negative input is VREF-
						//MUX A Channel 0 positive input is AN8

	AD1CSSL = 0x0000;	//Skip all ANx channels for input scan

	AD1CON3 = 0x0002;	//ADC Clock derived from system clock
						//Autosample time time bits = 0 TAD sinjce PWM is controlling sampling time
						//TAD = 3*TCY, TAD = 101.7 nSec

	AD1CON2 = 0x0300;	//ADREF+ = AVDD ADREF- = AVSS
						//Do not scan inputs
						//1x = Converts CH0, CH1, CH2 and CH3
						//A/D is currently filling buffer 0x0-0x7
						//Interrupts at the completion of conversion for each sample/convert sequence
						//Always starts filling buffer from the beginning
						//Always uses channel input selects for Sample A

	AD1CON1bits.DONE = 0;	//Making sure that there is any conversion in progress
	IPC3bits.AD1IP = 5;		//Assigning ADC ISR priority
	IFS0bits.AD1IF = 0;		//Clearing the ADC Interrupt Flag
	IEC0bits.AD1IE = 1;		//Enabling the ADC conversion complete interrupt
	AD1CON1bits.ADON = 1;	//Enabling the ADC module

}


/******************************************************************************
* Function:     InitMCPWM(void)
*
* Output:		None
*
* Overview:		Initializes the PWM module to operate in center-aligned mode
*				at 20KHz. PWM terminals are configured in independent mode.
*				PWM time base is 67.8 nSec.
*				PDCx value range is 0-1464 for 0%-100% duty cycle 
*				ADC reload time is variable according to the PWM duty cycle
*
*
* Note:			None
*******************************************************************************/
void InitMCPWM(void)
{
	P1TPER = ((FCY/FPWM)/2 - 1);
							//FCY  29491200...FRC w/PLL x16
							//FPWM 20KHz PWM Freq
							// MAX_DUTY_CYCLE = 1469
							// 50% duty cycle = 734

	
	P1TCONbits.PTSIDL = 1;  // PWM time base halted in CPU IDLE mode
	P1TCONbits.PTOPS = 0;   // PWM time base 1:1 postscale
	P1TCONbits.PTCKPS = 0;  // PWM time base 1:1 prescale
	P1TCONbits.PTMOD = 2;	// Center Aligned with single interrupt mode per PWM period

	PWM1CON1 = 0x0700;		// disable PWMs
	P1OVDCON = 0x0000;		// allow control using OVD

	P1SECMPbits.SEVTDIR = 0; 	// trigger ADC when PWM counter is in upwards dir
								//....Tad=84.77, Tpwm=67.816
	P1SECMPbits.SEVTCMP = 0;	// generates a trigger event for the ADC
								// when PWM time base is counting upwards
								// just before reaching the PTPER value
								// causing a sampling in the middle of the 
								// pulse

	//FLTACON = 0x008F;			// The PWM output pin is driven Inactive on 
							// an external fault input event
							// The Fault A input pin latches all control pins to 0
							// PWMxH4-1/PWMxL4-1 pin pair is controlled by 
							// Fault Input A


    PWM1CON2 = 0x0000;		// 1:1 postscale values
							// Updates to the active PxDCy registers 
							// are sync to the PWM time base
							// Output overrides via the PxOVDCON register occur
							// on the next TCY boundary
							// Updates from duty cycle and period buffer registers 
							// are enabled

	IPC14bits.PWM1IP = 4;	// PWM Interrupt Priority 4
	IFS3bits.PWM1IF=0;		// Clearing the PWM Interrupt Flag
	IEC3bits.PWM1IE=1;		// Enabling the PWM interrupt


	P1TCONbits.PTEN = 1;	// Enabling the MC PWM module
}

/******************************************************************************
* Function:     InitTMR2(void)
*
* Output:		None
*
* Overview:		Initializes the TIMER2 module to operate in free-running 
*				up counting mode. The TIMER2 time base is Tcy*64= 2.17uSec.
*				This timer is used to calculate the motor speed
*
* Note:			None
*******************************************************************************/
void InitTMR2(void)
{							// Tcy = 33.908 nSec
	TMR2 = 0;				// Resetting TIMER
	PR2 = 0xFFFF;			// Setting TIMER periond to the MAX value
	T2CON = 0x0030;			// internal Tcy*64 clock = 2.17uSec
}


/******************************************************************************
* Function:     InitTMR1(void)
*
* Output:		None
*
* Overview:		Initializes the TIMER1 module to operate in free-running 
*				up counting mode. The TIMER1 time base is Tcy*64 = 2.64uSec.
*				This timer is used to calculate the commutation delay
*
* Note:			None
*******************************************************************************/
void InitTMR1(void)
{							// Tcy = 33.908 nSec
	TMR1 = 0;				// Resentting TIMER
	PR1 = 10;				// Intial commucation delay value 43.4 uSeg
	T1CON = 0x0030;			// internal Tcy*64 clock = 2.17uSec

	IPC0bits.T1IP = 5; 		// Set Timer 1 Interrupt Priority Level
	IFS0bits.T1IF = 0; 		// Clear Timer 1 Interrupt Flag
	IEC0bits.T1IE = 1; 		// Enable Timer1 interrupt
	T1CONbits.TON = 1; 		// Enable Timer1
	
}

/******************************************************************************
* Function:     DelayNmSec(unsigned int N)
*
* Output:		None
*
* Overview:		Delay funtion used for push buttons debounce loop and for the
*				motor start-up sequence
*
* Note:			None
*******************************************************************************/
void DelayNmSec(unsigned int N)
{
unsigned int j;
while(N--)
 	for(j=0;j < MILLISEC;j++);
}

