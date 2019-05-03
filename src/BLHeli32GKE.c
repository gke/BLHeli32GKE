#include "UAVX.h"

//**** **** **** **** ****
//
// BLHeli program for controlling brushless motors in helicopters and multirotors
//
// Copyright 2011, 2012 Steffen Skaug
// This program is distributed under the terms of the GNU General Public License
//
// This file is part of BLHeli.
//
// BLHeli is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// BLHeli is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY// without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with BLHeli.  If not, see <http://www.gnu.org/licenses/>.
//
//**** **** **** **** ****
//
// The software was initially designed for use with Eflite mCP X, but is now adapted to copters/planes in general
//
// The software was inspired by and started from from Bernard Konze's BLMC: http://home.versanet.de/~bkonze/blc_6a/blc_6a.htm
// And also Simon Kirby's TGY: https://github.com/sim-/tgy
//
// This file is best viewed with tab width set to 5
//
// The input signal can be positive 1kHz, 2kHz, 4kHz, 8kHz or 12kHz PWM (e.g. taken from the "resistor tap" on mCPx)
// And the input signal can be PPM (1-2ms) or OneShot125 (125-250us) at rates up to several hundred Hz.
// The code adapts itself to the various input modes/frequencies
// The code ESC can also be to accept inverted input signal.
//
// The first lines of the software must be modified according to the chosen environment:
// Uncomment the selected ESC and main/tail/multi mode
// BESCNO "ESC"_"mode"
//
//**** **** **** **** ****
// Revision history:
// - Rev1.0: Initial revision based upon BLHeli for AVR controllers
// - Rev2.0: Changed "Eeprom" initialization, layout and defaults
//           Various changes and improvements to comparator reading. Now using timer1 for time from pwm on/off
//           Beeps are made louder
//           Added programmable low voltage limit
//           Added programmable damped tail mode (only for 1S ESCs)
//           Added programmable motor rotation direction
// - Rev2.1: (minor changes by 4712)
//		  Added Disable TX Programming by PC Setup Application
//		  therfore changed EEPROM_LAYOUT_REVISION = 8
//		  Added Vdd Monitor as reset source when writing to "EEProm"
//		  Changed for use of batch file to assemble, link and make hex files
// - Rev2.2: (minor changes by 4712)
//           Added Disable Throttle Re-Arming every motor start by PC Setup Application
// - Rev2.3: (minor changes by 4712)
//           Added bugfixed (2x CLR C before j(n)c operations)thx Steffen!
// - Rev2.4: Revisions 2.1 to 2.3 integrated
// - Rev3.0: Added PPM (1050us-1866us) as accepted input signal
//           Added startup rpm as a programming parameter
//           Added startup acceleration as a programming parameter
//           Added option for using voltage measurements to compensate motor power
//           Added governor target by setup as a governor mode option
//           Governor is kept active regardless of rpm
//           Smooth governor spoolup/down in arm and setup modes
//           Increased governor P and I gain programming ranges
//           Increased and changed low voltage limit programming range
//           Disabled tx programming entry for all but the first arming sequence after power on
//           Made it possible to skip parameters in tx programming by setting throttle midstick
//           Made it default not to rearm for every restart
// - Rev3.1: Fixed bug that prevented chosen parameter to be set in tx programming
// - Rev3.2: ...also updated the EEPROM revision parameter
// - Rev3.3: Fixed negative number bug in voltage compensation
//           Fixed bug in startup power calculation for non-default power
//           Prevented possibility for voltage compensation fighting low voltage limiting
//           Applied overall spoolup control to ensure soft spoolup in any mode
//           Added a delay of 3 seconds from initiation of main motor stop until new startup is allowed
//           Reduced beep power to reduce power consumption for very strong motors/ESCs
// - Rev3.4: Fixed bug that prevented full power in governor arm and setup modes
//           Increased NFETON_DELAY for XP_7A and XP_12A to allow for more powerful fets
//           Increased initial spoolup power, and linked to startup power
// - Rev4.0: Fixed bug that made tail tx program beeps very weak
//           Added thermal protection feature
//           Governor P and I gain ranges are extended up to 8.0x gain
//           Startup sequence is aborted upon zero throttle
//           Avoided voltage compensation function induced latency for tail when voltage compensation is not enabled
//           Improved input signal frequency detection robustness
// - Rev4.1: Increased thermal protection temperature limits
// - Rev5.0: Added multi(copter) operating mode. TAIL define changed to MODE with three modes: MAIN, TAIL and MULTI
//           Added programmable commutation timing
//           Added a damped light mode that has less damping, but that can be used with all escs
//           Added programmable damping force
//           Added thermal protection for startup too
//           Added wait beeps when waiting more than 30 sec for throttle above zero (after having been armed)
//           Modified tail idling to provide option for very low speeds
//           Changed PPM range to 1150-1830us
//           Arming sequence is dropped for PPM input, unless it is governor arm mode
//           Loss of input signal will immediately stop the motor for PPM input
//           Bug corrected in Turnigy Plush 6A voltage measurement setup
//           FET switching delays are set for original fets. Stronger/doubled/tripled etc fets may require faster pfet off switching
//           Miscellaneous other changes
// - Rev6.0: Reverted comparator reading routine to rev5.0 equivalent, in order to avoid tail motor stops
//           Added governor range programmability
//           Implemented startup retry sequence with varying startup power for multi mode
//           In damped light mode, damping is now applied to the active nfet phase for fully damped capable ESCs
// - Rev6.1: Added input signal qualification criteria for PPM, to avoid triggering on noise spikes (fix for plush hardware)
//           Changed main and multi mode stop criteria. Will now be in run mode, even if RC pulse input is zero
//           Fixed bug in commutation that caused rough running in damped light mode
//           Miscellaneous other changes
// - Rev7.0  Added direct startup mode programmability
//           Added throttle calibration. Min>=1000us and Max<=2000us. Difference must be >520us, otherwise max is shifted so that difference=520us
//           Added programmable throttle change rate
//           Added programmable beep strength, beacon strength and beacon delay
//           Reduced power step to full power significantly
//           Miscellaneous other changes
// - Rev8.0  Added a 2 second delay after power up, to wait for receiver initialization
//           Added a programming option for disabling low voltage limit, and made it default for MULTI
//           Added programable demag compensation, using the concept of SimonK
//           Improved robustness against noisy input signal
//           Refined direct startup
//           Removed voltage compensation
//           Miscellaneous other changes
// - Rev9.0  Increased programming range for startup power, and made its default ESC dependent
//           Made default startup method ESC dependent
//           Even more smooth and gentle spoolup for MAIN, to suit larger helis
//           Improved transition from stepped startup to run
//           Refined direct startup
// - Rev9.1  Fixed bug that changed FW revision after throttle calibration or TX programming
// - Rev9.2  Altered timing of throttle calibration in order to work with MultiWii calibration firmware
//           Reduced main spoolup time to around 5 seconds
//           Changed default beacon delay to 3 minutes
// - Rev9.3  Fixed bug in Plush 60/80A temperature reading, that caused failure in operation above 4S
//           Corrected temperature limit for HiModel cool 22/33/41A, RCTimer 6A, Skywalker 20/40A, Turnigy AE45A, Plush 40/60/80A. Limit was previously set too high
// - Rev9.4  Improved timing for increased maximum rpm limit
// - Rev10.0 Added closed loop mode for multi
//           Added high/low BEC voltage option (for the ESCs where HW supports it)
//           Added method of resetting all parameter values to defaults by TX programming
//           Added Turnigy K-force 40A and Turnigy K-force 120A HV ESCs
//           Enabled fully damped mode for several ESCs
//           Extended startup power range downwards to enable very smooth start for large heli main motors
//           Extended damping force with a highest setting
//           Corrected temperature limits for F310 chips (Plush 40A and AE 45A)
//           Implemented temperature reading average in order to avoid problems with ADC noise on Skywalkers
//           Increased switching delays for XP 7A fast, in order to avoid cross conduction of N and P fets
//           Miscellaneous other changes
// - Rev10.1 Relaxed RC signal jitter requirement during frequency measurement
//           Corrected bug that prevented using governor low
//           Enabled vdd monitor always, in order to reduce likelihood of accidental overwriting of adjustments
//           Fixed bug that caused stop for PPM input above 2048us, and moved upper accepted limit to 2160us
// - Rev10.2 Corrected temperature limit for AE20-30/XP7-25, where limit was too high
//           Corrected temperature limit for 120HV, where limit was too low
//           Fixed bug that caused AE20/25/30A not to run in reverse
// - Rev10.3 Removed vdd monitor for 1S capable ESCs, in order to avoid brownouts/resets
//           Made auto bailout spoolup for main more smooth
// - Rev10.4 Ensured that main spoolup and governor activation will always be smooth, regardless of throttle input
//           Added capability to operate on 12kHz input signal too
// - Rev11.0 Fixed bug of programming default values for governor in MULTI mode
//           Disabled interrupts explicitly some places, to avoid possibilities for unintentional fet switching
//           Changed interrupt disable strategy, to always allow pwm interrupts, to avoid noise when running at low rpms
//           Added governor middle range for MAIN mode
//           Added bidirectional mode for TAIL and MULTI mode with PPM input
//           Changed and improved demag compensation
//           Miscellaneous other changes
// - Rev11.1 Fixed bug of slow acceleration response for MAIN mode running without governor
//           Fixed bug with PWM input, where throttle remains high even when zeroing throttle (seen on V922 tail)
//           Fixed bug in bidirectional operation, where direction change could cause reset
//           Improved autorotation bailout for MAIN
//           Reduced min speed back to 1220 erpm
//           Misc code cleanups
// - Rev11.2 Fixed throttle calibration bug
//           Added high side driver precharge for all-nfet ESCs
//           Optimized timing in general and for demag compensation in particular
//           Auto bailout functionality modified
//           Governor is deactivated for throttle inputs below 10%
//           Increased beacon delay times
// - Rev12.0 Added programmable main spoolup time
//           Added programmable temperature protection enable
//           Bidirectional mode stop/start improved. Motor is now stopped before starting
//           Power is limited for very low rpms (when BEMF is low), in order to avoid sync loss
//           Damped light mode is made more smooth and quiet, particularly at low and high rpms
//           Comparator signal qualification scheme is changed
//           Demag compensation scheme is significantly changed
//           Increased jitter tolerance for PPM frequency measurement
//           Fully damped mode removed, and damped light only supported on damped capable ESCs
//           Default tail mode changed to damped light
//           Miscellaneous other changes
// - Rev12.1 Fixed bug in tail code
//           Improved startup for Atmel
//           Added support for multiple high BEC voltages
//           Added support for RPM output
// - Rev12.2 Improved running smoothness, particularly for damped light
//           Avoiding lockup at full throttle when input signal is noisy
//           Avoiding detection of 1-wire programming signal as valid throttle signal
// - Rev13.0 Removed stepped start
//           Removed throttle change rate and damping force parameters
//           Added support for OneShot125
//           Improved commutation timing accuracy
// - Rev13.1 Removed startup ramp for MULTI
//           Improved startup for some odd ESCs
// - Rev13.2 Still tweaking startup to make it more reliable and faster for all ESC/motor combos
//           Increased deadband for bidirectional operation
//           Relaxed signal detection criteria
//           Added support for running 50MHz capable SiLabs MCUs at 50MHz
//           Added bootlader to SiLabs code
//           Miscellaneous other changes
// - Refxxx  Partilal transliteration from assembler to C done for curiousity only
//

//
//**** **** **** **** ****
// Master clock is internal 24MHz oscillator
// Timer 0 (167/500ns counts) always counts up and is used for
// - PWM generation
// Timer 1 (167/500ns counts) always counts up and is used for
// - Time from pwm on/off event
// Timer 2 (500ns counts) always counts up and is used for
// - RC pulse timeout/skip counts and commutation times
// Timer 3 (500ns counts) always counts up and is used for
// - Commutation timeouts
// PCA0 (500ns counts) always counts up and is used for
// - RC pulse measurement
//
//**** **** **** **** ****
// Interrupt handling
// The F330/2 does not disable interrupts when entering an interrupt routine.
// Also some interrupt flags need to be cleared by software
// The code disables interrupts in interrupt routines, in order to avoid too nested interrupts
// - Interrupts are disabled during beeps, to avoid audible interference from interrupts
// - RC pulse interrupts are periodically disabled in order to reduce interference with pwm interrupts.
//
//**** **** **** **** ****
// Motor control:
// - Brushless motor control with 6 states for each electrical 360 degrees
// - An advance timing of 0deg has zero cross 30deg after one commutation and 30deg before the next
// - Timing advance in this implementation is set to 15deg nominally
// - "Damped" commutation schemes are available, where more than one pfet is on when pwm is off. This will absorb energy from bemf and make step settling more damped.
// Motor sequence starting from zero crossing:
// - Timer wait: Wt_Comm			15deg	// Time to wait from zero cross to actual commutation
// - Timer wait: Wt_Advance		15deg	// Time to wait for timing advance. Nominal commutation point is after this
// - Timer wait: Wt_Zc_Scan		7.5deg	// Time to wait before looking for zero cross
// - Scan for zero cross			22.5deg	, Nominal, with some motor variations
//
// Motor startup:
// Startup is the only phase, before normal bemf commutation run begins.
//
//**** **** **** **** ****
// List of enumerated supported ESCs and modes  (main, tail or multi)

#define MAIN_MODE 0
#define TAIL_MODE 1
#define MULTI_MODE 2

#define Turnigy_Plush_25A_Multi 			42
#define Turnigy_Plush_Nfet_30A_Multi 		63

//**** **** **** **** ****
// Select the ESC and mode to use (or unselect all for use with external batch compile file)

#define BESCNO Turnigy_Plush_30A_Multi

//**** **** **** **** ****
// ESC selection statements

#if (BESCNO==Turnigy_Plush_30A_Multi)
#define MODE MULTI_MODE				// Choose mode. Set to 2 for multirotor
//#include "escincludes/Turnigy_Plush_30A.inc"	// Select Turnigy Plush 30A pinout
#endif

//**** **** **** **** ****
// TX programming defaults
//
// Parameter dependencies:
// - Governor P gain, I gain and Range is only used if one of the three governor modes is selected
// - Governor setup target is only used if Setup governor mode is selected (or closed loop mode is on for multi)
//
// MAIN
#define DEFAULT_PGM_MAIN_P_GAIN 			7 	// 1=0.13		2=0.17		3=0.25		4=0.38 		5=0.50 	6=0.75 	7=1.00 8=1.5 9=2.0 10=3.0 11=4.0 12=6.0 13=8.0
#define DEFAULT_PGM_MAIN_I_GAIN 			7 	// 1=0.13		2=0.17		3=0.25		4=0.38 		5=0.50 	6=0.75 	7=1.00 8=1.5 9=2.0 10=3.0 11=4.0 12=6.0 13=8.0
#define DEFAULT_PGM_MAIN_GOVERNOR_MODE 		1 	// 1=Tx 		2=Arm 		3=Setup		4=Off
#define DEFAULT_PGM_MAIN_GOVERNOR_RANGE 	1 	// 1=High		2=Middle		3=Low
#define DEFAULT_PGM_MAIN_LOW_VOLTAGE_LIM	4 	// 1=Off		2=3.0V/c		3=3.1V/c		4=3.2V/c		5=3.3V/c	6=3.4V/c
#define DEFAULT_PGM_MAIN_COMM_TIMING		3 	// 1=Low 		2=MediumLow 	3=Medium 		4=MediumHigh 	5=High
#if (DAMPED_MODE_ENABLE==1)
#define DEFAULT_PGM_MAIN_PWM_FREQ 			2 	// 1=High 		2=Low		3=DampedLight
#else
#define DEFAULT_PGM_MAIN_PWM_FREQ 			2 	// 1=High 		2=Low
#endif
#define DEFAULT_PGM_MAIN_DEMAG_COMP 		1 	// 1=Disabled	2=Low		3=High
#define DEFAULT_PGM_MAIN_DIRECTION			1 	// 1=Normal 	2=Reversed
#define DEFAULT_PGM_MAIN_RCP_PWM_POL 		1 	// 1=Positive 	2=Negative
#define DEFAULT_PGM_MAIN_GOV_SETUP_TARGET	180	// Target for governor in setup mode. Corresponds to 70% throttle
#define DEFAULT_PGM_MAIN_REARM_START		0 	// 1=Enabled 	0=Disabled
#define DEFAULT_PGM_MAIN_BEEP_STRENGTH		120	// Beep strength
#define DEFAULT_PGM_MAIN_BEACON_STRENGTH	200	// Beacon strength
#define DEFAULT_PGM_MAIN_BEACON_DELAY		4 	// 1=1m		2=2m			3=5m			4=10m		5=Infinite
// TAIL
#define DEFAULT_PGM_TAIL_GAIN 				3 	// 1=0.75 		2=0.88 		3=1.00 		4=1.12 		5=1.25
#define DEFAULT_PGM_TAIL_IDLE_SPEED 		4 	// 1=Low 		2=MediumLow 	3=Medium 		4=MediumHigh 	5=High
#define DEFAULT_PGM_TAIL_COMM_TIMING		3 	// 1=Low 		2=MediumLow 	3=Medium 		4=MediumHigh 	5=High
#if (DAMPED_MODE_ENABLE==1)
#define DEFAULT_PGM_TAIL_PWM_FREQ	 		3 	// 1=High 		2=Low 		3=DampedLight
#else
#define DEFAULT_PGM_TAIL_PWM_FREQ	 		1 	// 1=High 		2=Low
#endif
#define DEFAULT_PGM_TAIL_DEMAG_COMP 		1 	// 1=Disabled	2=Low		3=High
#define DEFAULT_PGM_TAIL_DIRECTION			1 	// 1=Normal 	2=Reversed	3=Bidirectional
#define DEFAULT_PGM_TAIL_RCP_PWM_POL 		1 	// 1=Positive 	2=Negative
#define DEFAULT_PGM_TAIL_BEEP_STRENGTH		250	// Beep strength
#define DEFAULT_PGM_TAIL_BEACON_STRENGTH	250	// Beacon strength
#define DEFAULT_PGM_TAIL_BEACON_DELAY		4 	// 1=1m		2=2m			3=5m			4=10m		5=Infinite
// MULTI
#define DEFAULT_PGM_MULTI_P_GAIN 			9 	// 1=0.13		2=0.17		3=0.25		4=0.38 		5=0.50 	6=0.75 	7=1.00 8=1.5 9=2.0 10=3.0 11=4.0 12=6.0 13=8.0
#define DEFAULT_PGM_MULTI_I_GAIN 			9 	// 1=0.13		2=0.17		3=0.25		4=0.38 		5=0.50 	6=0.75 	7=1.00 8=1.5 9=2.0 10=3.0 11=4.0 12=6.0 13=8.0
#define DEFAULT_PGM_MULTI_GOVERNOR_MODE 	4 	// 1=HiRange	2=MidRange	3=LoRange		4=Off
#define DEFAULT_PGM_MULTI_GAIN 				3 	// 1=0.75 		2=0.88 		3=1.00 		4=1.12 		5=1.25
#define DEFAULT_PGM_MULTI_LOW_VOLTAGE_LIM	1 	// 1=Off		2=3.0V/c		3=3.1V/c		4=3.2V/c		5=3.3V/c	6=3.4V/c
#define DEFAULT_PGM_MULTI_COMM_TIMING		3 	// 1=Low 		2=MediumLow 	3=Medium 		4=MediumHigh 	5=High
#if (DAMPED_MODE_ENABLE==1)
#define DEFAULT_PGM_MULTI_PWM_FREQ	 		1 	// 1=High 		2=Low 		3=DampedLight
#else
#define DEFAULT_PGM_MULTI_PWM_FREQ	 		1 	// 1=High 		2=Low
#endif
#define DEFAULT_PGM_MULTI_DEMAG_COMP 		2 	// 1=Disabled	2=Low		3=High
#define DEFAULT_PGM_MULTI_DIRECTION			1 	// 1=Normal 	2=Reversed	3=Bidirectional
#define DEFAULT_PGM_MULTI_RCP_PWM_POL 		1 	// 1=Positive 	2=Negative
#define DEFAULT_PGM_MULTI_BEEP_STRENGTH		40	// Beep strength
#define DEFAULT_PGM_MULTI_BEACON_STRENGTH	80	// Beacon strength
#define DEFAULT_PGM_MULTI_BEACON_DELAY		4 	// 1=1m		2=2m			3=5m			4=10m		5=Infinite
// COMMON
#define DEFAULT_PGM_ENABLE_TX_PROGRAM 		1 	// 1=Enabled 	0=Disabled
#define DEFAULT_PGM_PPM_MIN_THROTTLE		37	// 4*37+1000=1148
#define DEFAULT_PGM_PPM_MAX_THROTTLE		208	// 4*208+1000=1832
#define DEFAULT_PGM_PPM_CENTER_THROTTLE		122	// 4*122+1000=1488 (used in bidirectional mode)
#define DEFAULT_PGM_BEC_VOLTAGE_HIGH		0	// 0=Low		1+= High or higher
#define DEFAULT_PGM_ENABLE_TEMP_PROT	 	1 	// 1=Enabled 	0=Disabled
//**** **** **** **** ****
// Constant definitions for main
#if (MODE==MAIN_MODE)
#define GOV_SPOOLRATE			2	// Number of steps for governor requested pwm per 32ms
#define RCP_TIMEOUT_PPM			10	// Number of timer2H overflows (about 32ms) before considering rc pulse lost
#define RCP_TIMEOUT				64	// Number of timer2L overflows (about 128us) before considering rc pulse lost
#define RCP_SKIP_RATE			32	// Number of timer2L overflows (about 128us) before reenabling rc pulse detection
#define RCP_MIN					0	// This is minimum RC pulse length
#define RCP_MAX					255	// This is maximum RC pulse length
#define RCP_VALIDATE			2	// Require minimum this pulse length to validate RC pulse
#define RCP_STOP				1	// Stop motor at or below this pulse length
#define RCP_STOP_LIMIT			250	// Stop motor if this many timer2H overflows (~32ms) are below stop limit
#define PWM_START				50 	// PWM used as max power during start
#define COMM_TIME_RED			1	// Fixed reduction (in us) for commutation wait (to account for fixed delays)
#define COMM_TIME_MIN			1	// Minimum time (in us) for commutation wait
#define TEMP_CHECK_RATE			8	// Number of adc conversions for each check of temperature (the other conversions are used for voltage)
#elif (MODE==TAIL_MODE) // Constant definitions for tail
#define GOV_SPOOLRATE			1	// Number of steps for governor requested pwm per 32ms
#define RCP_TIMEOUT_PPM			10	// Number of timer2H overflows (about 32ms) before considering rc pulse lost
#define RCP_TIMEOUT				24	// Number of timer2L overflows (about 128us) before considering rc pulse lost
#define RCP_SKIP_RATE			6	// Number of timer2L overflows (about 128us) before reenabling rc pulse detection
#define RCP_MIN					0	// This is minimum RC pulse length
#define RCP_MAX					255	// This is maximum RC pulse length
#define RCP_VALIDATE			2	// Require minimum this pulse length to validate RC pulse
#define RCP_STOP				1	// Stop motor at or below this pulse length
#define RCP_STOP_LIMIT			130	// Stop motor if this many timer2H overflows (~32ms) are below stop limit
#define PWM_START				50 	// PWM used as max power during start
#define COMM_TIME_RED			1	// Fixed reduction (in us) for commutation wait (to account for fixed delays)
#define COMM_TIME_MIN			1	// Minimum time (in us) for commutation wait
#define TEMP_CHECK_RATE 		8 // Number of adc conversions for each check of temperature (the other conversions are used for voltage)
#elif (MODE==MULTI_MODE) // Constant definitions for multi
#define GOV_SPOOLRATE			1	// Number of steps for governor requested pwm per 32ms
#define RCP_TIMEOUT_PPM			10	// Number of timer2H overflows (about 32ms) before considering rc pulse lost
#define RCP_TIMEOUT				24	// Number of timer2L overflows (about 128us) before considering rc pulse lost
#define RCP_SKIP_RATE			6	// Number of timer2L overflows (about 128us) before reenabling rc pulse detection
#define RCP_MIN					0	// This is minimum RC pulse length
#define RCP_MAX					255	// This is maximum RC pulse length
#define RCP_VALIDATE			2	// Require minimum this pulse length to validate RC pulse
#define RCP_STOP				1	// Stop motor at or below this pulse length
#define RCP_STOP_LIMIT			250	// Stop motor if this many timer2H overflows (~32ms) are below stop limit
#define PWM_START				50 	// PWM used as max power during start
#define COMM_TIME_RED			1	// Fixed reduction (in us) for commutation wait (to account for fixed delays)
#define COMM_TIME_MIN			1	// Minimum time (in us) for commutation wait
#define TEMP_CHECK_RATE 		8 // Number of adc conversions for each check of temperature (the other conversions are used for voltage)
#endif

//**** **** **** **** ****

typedef void (*FETFuncPtr)();
static FETFuncPtr DPTR = NULL;

int32 Temp1, Temp2, Temp3, Temp4, Temp5, Temp6, Temp7, Temp8;
int32 A, C, AE, TL1;

//**** **** **** **** ****
// Register definitions
//zzzDSEG AT 20h					// Variables segment

int32 Bit_Access; // MUST BE AT THIS ADDRESS. Variable at bit accessible address (for non interrupt routines)
int32 Bit_Access_Int; // Variable at bit accessible address (for interrupts)

int32 Requested_Pwm; // Requested pwm (from RC pulse value)
int32 Governor_Req_Pwm; // Governor requested pwm (sets governor target)
int32 Current_Pwm; // Current pwm
int32 Current_Pwm_Limited; // Current pwm that is limited (applied to the motor output)
int32 Rcp_Prev_Edge; // RC pulse previous edge timer3 timestamp (lo byte)
int32 Rcp_Outside_Range_Cnt; // RC pulse outside range counter (incrementing)
int32 Rcp_Timeout_Cnt; // RC pulse timeout counter (decrementing)
int32 Rcp_Skip_Cnt; // RC pulse skip counter (decrementing)
int32 _Spare_Reg; // Spare register

Flags F;

//******
// ESC specific externals

#define FET_DELAY(d) {int32 n=d; while(n-->0){}}

#define NFETON_DELAY 0
#define PFETON_DELAY 0

enum {
	run1, run2, run3, run4, run5, run6
};
int32 runState = run1;

void All_nFETs_on(void) {
}
;
void All_nFETs_off(void) {
}
;
void All_pFETs_on(void) {
}
;
void All_pFETs_off(void) {
}

void AnFET_on(void) {
}
;
void AnFET_off(void) {
}
;
void ApFET_on(void) {
}
;
void ApFET_off(void) {
}
;

void BnFET_on(void) {
}
;
void BnFET_off(void) {
}
;
void BpFET_on(void) {
}
;
void BpFET_off(void) {
}
;

void CnFET_on(void) {
}
;
void CnFET_off(void) {
}
;
void CpFET_on(void) {
}
;
void CpFET_off(void) {
}
;

void Set_Comp_Phase_A(void) {
}
;
void Set_Comp_Phase_B(void) {
}
;
void Set_Comp_Phase_C(void) {
}
;

//**** **** **** **** ****
// RAM definitions


boolean Initial_Arm; // Variable that is set during the first arm sequence after power on

int32 Power_On_Wait_Cnt; // Power on wait counter (lo byte)

int32 Startup_Rot_Cnt; // Startup phase rotations counter
int32 Startup_Ok_Cnt; // Startup phase ok comparator waits counter (incrementing)
int32 Demag_Detected_Metric; // Metric used to gauge demag event frequency
int32 Demag_Pwr_Off_Thresh; // Metric threshold above which power is cut
int32 Low_Rpm_Pwr_Slope; // Sets the slope of power increase for low rpms

int32 Prev_Comm; // Previous commutation timer3 timestamp (lo byte)
int32 Comm_Period4x; // Timer3 counts between the last 4 commutations (lo byte)
int32 Comm_Phase; // Current commutation phase
int32 Comparator_Read_Cnt; // Number of comparator reads done

int32 Gov_Target; // Governor target (lo byte)
int32 Gov_Integral; // Governor integral error (lo byte)
int32 Gov_Integral_X; // Governor integral error (ex byte)
int32 Gov_Proportional; // Governor proportional error
int32 Gov_Prop_Pwm; // Governor calculated new pwm based upon proportional error
int32 Gov_Arm_Target; // Governor arm target value
int32 Gov_Active; // Governor active (enabled when speed is above minimum)

int32 Wt_Advance; // Timer3 counts for commutation advance timing (lo byte)
int32 Wt_Zc_Scan; // Timer3 counts from commutation to zero cross scan (lo byte)
int32 Wt_Zc_Timeout; // Timer3 counts for zero cross scan timeout (lo byte)
int32 Wt_Comm; // Timer3 counts from zero cross to commutation
int32 Next_Wt; // Timer3 counts for next wait period

int32 Rcp_PrePrev_Edge; // RC pulse pre previous edge pca timestamp (lo byte)
int32 Rcp_Edge; // RC pulse edge pca timestamp (lo byte)
int32 Rcp_Prev_Period; // RC pulse previous period (lo byte)
int32 Rcp_Period_Diff_Accepted; // RC pulse period difference acceptable
int32 New_Rcp; // New RC pulse value in pca counts
int32 Prev_Rcp_Pwm_Freq; // Previous RC pulse pwm frequency (used during pwm frequency measurement)
int32 Curr_Rcp_Pwm_Freq; // Current RC pulse pwm frequency (used during pwm frequency measurement)
int32 Rcp_Stop_Cnt; // Counter for RC pulses below stop value
int32 Auto_Bailout_Armed; // Set when auto rotation bailout is armed

int32 Pwm_Limit; // Maximum allowed pwm
int32 Pwm_Limit_Spoolup; // Maximum allowed pwm during spoolup
int32 Pwm_Limit_Low_Rpm; // Maximum allowed pwm for low rpms
int32 Pwm_Spoolup_Beg; // Pwm to begin main spoolup with
int32 Pwm_Motor_Idle; // Motor idle speed pwm
int32 Pwm_On_Cnt; // Pwm on event counter (used to increase pwm off time for low pwm)

int32 Spoolup_Limit_Cnt; // Interrupt count for spoolup limit
int32 Spoolup_Limit_Skip; // Interrupt skips for spoolup limit increment (1=no skips, 2=skip one etc)
int32 Main_Spoolup_Time_3x; // Main spoolup time x3
int32 Main_Spoolup_Time_10x; // Main spoolup time x10
int32 Main_Spoolup_Time_15x; // Main spoolup time x15

int32 Lipo_Adc_Reference; // Voltage reference adc value (lo byte)
int32 Lipo_Adc_Limit; // Low voltage limit adc value (lo byte)
int32 Adc_Conversion_Cnt; // Adc conversion counter

int32 Current_Average_Temp; // Current average temperature (lo byte ADC reading, assuming hi byte is 1)

int32 Ppm_Throttle_Gain; // Gain to be applied to RCP value for PPM input

int32 Skip_T2_Int; // Set for 50MHz MCUs when timer 2 interrupt shall be ignored
int32 Skip_T2h_Int; // Set for 50MHz MCUs when timer 2 high interrupt shall be ignored
int32 Timer0_Overflow_Value; // Remaining timer 0 wait time used with 50MHz MCUs

// Indirect addressing data segment
//zzzISEG AT 0D0h
int32 Tag_Temporary_Storage[48]; // Temporary storage for tags when updating "Eeprom"


//**** **** **** **** ****

#define EEPROM_FW_MAIN_REVISION 13
#define EEPROM_FW_SUB_REVISION 2
#define EEPROM_LAYOUT_REVISION 19

#define DEFAULT_PGM_MULTI_STARTUP_PWR 0

struct Params {
	int32 FW_Main_Revision; // EEPROM firmware main revision number
	int32 FW_Sub_Revision; // EEPROM firmware sub revision number
	int32 Layout_Revision; // EEPROM layout revision number

	int32 Gov_I_Gain;
	int32 Gov_P_Gain;
	int32 Gov_Mode; // closed loop mode
	int32 Low_Voltage_Lim; // low voltage limit
	int32 Motor_Gain; // tail gain
	int32 Motor_Idle; // tail idle speed
	int32 Startup_Pwr; // startup power
	int32 Pwm_Freq; // pwm frequency
	int32 Direction; // rotation direction
	int32 Input_Pol; // input polarity
	int32 Initialized; // EEPROM initialized signature low byte
	int32 Enable_TX_Program; // EEPROM TX programming enable
	int32 Main_Rearm_Start;
	int32 Gov_Setup_Target;
	int32 Startup_Rpm;
	int32 Startup_Accel;
	int32 Volt_Comp;
	int32 Comm_Timing; // commutation timing
	int32 Damping_Force;
	int32 Gov_Range;
	int32 Startup_Method;
	int32 Ppm_Min_Throttle; // minimum throttle (final value is 4x+1000=1148)
	int32 Ppm_Max_Throttle; // minimum throttle (final value is 4x+1000=1832)
	int32 Beep_Strength; // beep strength
	int32 Beacon_Strength; // beacon strength
	int32 Beacon_Delay; // beacon delay
	int32 Throttle_Rate;
	int32 Demag_Comp; // demag compensation
	int32 BEC_Voltage_High; // BEC voltage
	int32 Ppm_Center_Throttle; // center throttle (final value is 4x+1000=1488)
	int32 Main_Spoolup_Time;
	int32 Temp_Prot_Enable; // temperature protection enable

	int32 Dummy; // EEPROM address for safety reason
	uint8 Name[16]; // Name tag (16 Bytes)
} P;

// Table definitions
int32 GOV_GAIN_TABLE[] = { 0x02, 0x03, 0x04, 0x06, 0x08, 0x0C, 0x10, 0x18,
		0x20, 0x30, 0x40, 0x60, 0x80 };
int32 STARTUP_POWER_TABLE[] = { 0x04, 0x06, 0x08, 0x0C, 0x10, 0x18, 0x20, 0x30,
		0x40, 0x60, 0x80, 0x0A0, 0x0C0 };
#if (MODE==MAIN_MODE)
#if (DAMPED_MODE_ENABLE==1)
int32 TX_PGM_PARAMS_MAIN[] = {13, 13, 4, 3, 6, 13, 5, 3, 3, 2, 2};
#elif (DAMPED_MODE_ENABLE==0)
int32 TX_PGM_PARAMS_MAIN[] = {13, 13, 4, 3, 6, 13, 5, 2, 3, 2, 2};
#endif
#elif (MODE==TAIL_MODE)
#if (DAMPED_MODE_ENABLE==1)
int32 TX_PGM_PARAMS_TAIL[] = {5, 5, 13, 5, 3, 3, 3, 2};
#elif (DAMPED_MODE_ENABLE==0)
int32 TX_PGM_PARAMS_TAIL[] = {5, 5, 13, 5, 2, 3, 3, 2};
#endif
#elif (MODE==MULTI_MODE)
#if (DAMPED_MODE_ENABLE==1)
int32 TX_PGM_PARAMS_MULTI[] = {13, 13, 4, 5, 6, 13, 5, 3, 3, 3, 2};
#elif (DAMPED_MODE_ENABLE==0)
int32 TX_PGM_PARAMS_MULTI[] = { 13, 13, 4, 5, 6, 13, 5, 2, 3, 3, 2 };
#endif
#endif

//___________________________________________________________________________
//
// Timer0 interrupt routine
//
// Assumptions: DPTR register must be set to desired pwm_nBnFET_off(); label
// Requirements: Temp variables can NOT be used since PSW.3 is not set
//
//___________________________________________________________________________

void t0_int_pwm_on_exit(void);
void t0_int_pwm_off_exit(void);

void t0_int(void) { // Used for pwm control


} //t0_int

void t0_int_start(void) {
	/*zz
	 //zz EA = 0;			// Disable all interrupts
	 push	PSW			// Preserve registers through interrupt
	 push	ACC
	 // Check if pwm is on
	 jb	F.PWM_ON, t0_int_pwm_off	// Is pwm on?

	 // Do not execute pwm when stopped
	 jb	F.MOTOR_SPINNING, ($+5)
	 ajmp	t0_int_pwm_on_exit
	 // Do not execute pwm on during demag recovery
	 jnb	F.DEMAG_CUT_POWER, ($+5)
	 ajmp	t0_int_pwm_on_exit_pfets_off
	 // Pwm on cycle.
	 #if (MODE==TAIL_MODE)				// Tail
	 jnb	Current_Pwm_Limited.7, t0_int_pwm_on_low_pwm	// Jump for low pwm (<50%)
	 #endif

	 t0_int_pwm_on_execute:
	 A=0;
	 jmp	@A+DPTR					// Jump to pwm on routines. DPTR should be set to one of the pwm_nBnFET_off(); labels

	 #if (MODE==TAIL_MODE)				// Tail
	 t0_int_pwm_on_low_pwm:
	 // Skip pwm on cycles for very low pwm
	 inc	Pwm_On_Cnt				// Increment event counter
	 C=0;
	 A = #5					// Only skip for very low pwm
	 subb	A, Current_Pwm_Limited		// Check skipping shall be done (for low pwm only)
	 jc	t0_int_pwm_on_execute

	 subb	A, Pwm_On_Cnt				// Check if on cycle is to be skipped
	 jc	t0_int_pwm_on_execute

	 jb	F.STARTUP_PHASE, t0_int_pwm_on_execute

	 TL0=0;
	 TH1=0;
	 A = Current_Pwm_Limited;
	 jnz	t0_int_pwm_on_low_pwm_done

	 TL0=0;
	 TH1=0;
	 setb	F.PWM_TIMER0_OVERFLOW
	 Timer0_Overflow_Value, #0

	 t0_int_pwm_on_low_pwm_done:
	 jmp	t0_int_pwm_on_exit_no_timer_update
	 #endif
	 */
} // t0_int_start

void t0_int_pwm_off(void) {
	/*zz
	 // Pwm off cycle

	 C=0;
	 A = Current_Pwm_Limited;
	 rlc	A
	 jc	t0_int_pwm_off_set_timer;

	 TL0=0;
	 F.PWM_TIMER0_OVERFLOW = true;
	 Timer0_Overflow_Value= A;
	 ajmp	t0_int_pwm_off_timer_set

	 t0_int_pwm_off_set_timer:
	 TL0= A;
	 t0_int_pwm_off_timer_set:

	 // Clear pwm on flag
	 F.PWM_ON = false;
	 // Set full PWM (on all the time) if current PWM near max. This will give full power, but at the cost of a small "jump" in power
	 A = Current_Pwm_Limited;		// Load current pwm
	 cpl	A						// Full pwm?
	 jnz	($+4)					// No - branch
	 ajmp	t0_int_pwm_off_fullpower_exit	// Yes - exit

	 // Do not execute pwm when stopped
	 jb	F.MOTOR_SPINNING, ($+5)
	 ajmp	t0_int_pwm_off_exit_nfets_off

	 #if (DAMPED_MODE_ENABLE==1)
	 // If damped operation, set pFETs on in pwm_off
	 jb	F.PGM_PWMOFF_DAMPED, t0_int_pwm_off_damped	// Damped operation?
	 #endif

	 // Separate exit commands here for minimum delay
	 TL1= 0;		// Reset timer1

	 pop	ACC			// Restore preserved registers
	 pop	PSW
	 All_nFETs_Off();
	 //zzEA = 1			// Enable all interrupts
	 //reti

	 */
} // t0_int_pwm_off

void t0_int_pwm_off_damped(void) {
	All_nFETs_off();
	FET_DELAY(PFETON_DELAY);
	A = Comm_Phase; // Turn on pfets according to commutation phase
	A--;
	//zz	jb	ACC.2, t0_int_pwm_off_comm_5_6
	//zz	jb	ACC.1, t0_int_pwm_off_comm_3_4

	CpFET_on(); // Comm phase 1 or 2 - turn on C
	t0_int_pwm_off_exit();
} // t0_int_pwm_off_damped

void t0_int_pwm_off_comm_3_4(void) {
	BpFET_on(); // Comm phase 3 or 4 - turn on B
	t0_int_pwm_off_exit();
} // t0_int_pwm_off_comm_3_4

void t0_int_pwm_off_comm_5_6(void) {
	ApFET_on(); // Comm phase 5 or 6 - turn on A
	t0_int_pwm_off_exit();
} // t0_int_pwm_off_comm_5_6

void t0_int_pwm_off_exit_nfets_off(void) { // Exit from pwm off cycle
	TL1 = 0; // Reset timer1
#if (MCU_50MHZ==1)
	TH1= 0;
#endif

	All_nFETs_off();
	//zz	//zzEA = 1			// Enable all interrupts
	//reti
}

void t0_int_pwm_off_exit(void) {
	TL1 = 0; // Reset timer1

	/* zzz
	 t0_int_pwm_off_fullpower_exit:
	 pop	ACC			// Restore preserved registers
	 pop	PSW
	 //zzEA = 1			// Enable all interrupts
	 //reti
	 */

} // t0_int_pwm_off_exit

//___________________________________________________________________________
//
// Switch power off routine
//
// No assumptions
// Switches all fets off
//___________________________________________________________________________

void switch_power_off(void) {
	//DPTR=&pwm_noBnFET_off	// Set DPTR register to pwm_noBnFET_off label
	All_nFETs_off(); // Turn off all nfets
	All_pFETs_off(); // Turn off all pfets
	//	clr	F.PWM_ON		// Set pwm cycle to pwm off
} // switch_power_off


void pwm_noBnFET_off(void) { // Dummy pwm on cycle
	t0_int_pwm_on_exit();
} // pwm_noBnFET_off

void pwm_aBnFET_off(void) { // Pwm on cycle afet on (bfet off)
	AnFET_on();
	BnFET_off();
	t0_int_pwm_on_exit();
} // pwm_aBnFET_off

void pwm_bBnFET_off(void) { // Pwm on cycle bfet on (cfet off)
	BnFET_on();
	CnFET_off();
	t0_int_pwm_on_exit();
} // pwm_bBnFET_off

void pwm_cBnFET_off(void) { // Pwm on cycle cfet on (afet off)
	CnFET_on();
	AnFET_off();
	t0_int_pwm_on_exit();
} // pwm_cBnFET_off

void pwm_anfet_bpBnFET_off(void) { // Pwm on cycle anfet on (bnfet off) and bpfet on (used in damped state 6)
// Delay from pFETs are turned off (only in damped mode) until nFET is turned on (pFETs are slow)

	ApFET_off();
	CpFET_off();
	FET_DELAY(NFETON_DELAY);
	AnFET_on(); // Switch nFETs
	BnFET_off();
	t0_int_pwm_on_exit();
} // pwm_anfet_bpBnFET_off

void pwm_anfet_cpBnFET_off(void) { // Pwm on cycle anfet on (bnfet off) and cpfet on (used in damped state 5)
// Delay from pFETs are turned off (only in damped mode) until nFET is turned on (pFETs are slow)
	int32 d;

	ApFET_off();
	BpFET_off();
	FET_DELAY(NFETON_DELAY);
	AnFET_on(); // Switch nFETs
	BnFET_off();
	t0_int_pwm_on_exit();
} // pwm_anfet_cpBnFET_off

void pwm_bnfet_cpBnFET_off(void) { // Pwm on cycle bnfet on (cnfet off) and cpfet on (used in damped state 4)
// Delay from pFETs are turned off (only in damped mode) until nFET is turned on (pFETs are slow)
	BpFET_off();
	ApFET_off();
	FET_DELAY(NFETON_DELAY);
	BnFET_on(); // Switch nFETs
	CnFET_off();
	t0_int_pwm_on_exit();
} // pwm_bnfet_cpBnFET_off

void pwm_bnfet_apBnFET_off(void) { // Pwm on cycle bnfet on (cnfet off) and apfet on (used in damped state 3)
// Delay from pFETs are turned off (only in damped mode) until nFET is turned on (pFETs are slow)
	int32 d;

	BpFET_off();
	CpFET_off();
	FET_DELAY(NFETON_DELAY);
	BnFET_on(); // Switch nFETs
	CnFET_off();
	t0_int_pwm_on_exit();
} // pwm_bnfet_apBnFET_off

void pwm_cnfet_apBnFET_off(void) { // Pwm on cycle cnfet on (anfet off) and apfet on (used in damped state 2)
// Delay from pFETs are turned off (only in damped mode) until nFET is turned on (pFETs are slow)

	CpFET_off();
	BpFET_off();
	FET_DELAY(NFETON_DELAY);
	CnFET_on(); // Switch nFETs
	AnFET_off();
	t0_int_pwm_on_exit();
} // pwm_cnfet_apBnFET_off

void pwm_cnfet_bpBnFET_off(void) { // Pwm on cycle cnfet on (anfet off) and bpfet on (used in damped state 1)
// Delay from pFETs are turned off (only in damped mode) until nFET is turned on (pFETs are slow)

	CpFET_off();
	ApFET_off();
	FET_DELAY(NFETON_DELAY);
	CnFET_on(); // Switch nFETs
	AnFET_off();
	t0_int_pwm_on_exit();
} // pwm_cnfet_bpBnFET_off

void t0_int_pwm_on_exit_pfets_off(void) {
	//zz	jnb	F.PGM_PWMOFF_DAMPED, t0_int_pwm_on_exit	// If not damped operation - branch
	A = Comm_Phase; // Turn off pfets according to commutation phase
	//zz	jb	ACC.2, t0_int_pfets_off_comm_4_5_6
	//zz	jb	ACC.1, t0_int_pfets_off_comm_2_3
} // t0_int_pwm_on_exit_pfets_off

void t0_int_pfets_off_comm_1_6(void) {
	ApFET_off(); // Comm phase 1 and 6 - turn off A and C
	CpFET_off();
	t0_int_pwm_on_exit();
} // t0_int_pfets_off_comm_1_6

void t0_int_pfets_off_comm_4_5_6(void) {
	//zz	jb	ACC.1, t0_int_pfets_off_comm_1_6
	ApFET_off(); // Comm phase 4 and 5 - turn off A and B
	BpFET_off();
	t0_int_pwm_on_exit();
} // t0_int_pfets_off_comm_4_5_6

void t0_int_pfets_off_comm_2_3(void) {
	BpFET_off(); // Comm phase 2 and 3 - turn off B and C
	CpFET_off();
	t0_int_pwm_on_exit();
} // t0_int_pfets_off_comm_2_3

void t0_int_pwm_on_exit(void) {
	/*
	 // Set timer for coming on cycle length
	 A =Current_Pwm_Limited;		// Load current pwm
	 cpl	A						// cpl is 255-x
	 #if (MCU_50MHZ==0)
	 TL0= A;					// Write start point for timer
	 #else
	 C=0;
	 rlc	A
	 jc	t0_int_pwm_on_set_timer

	 TL0=0;
	 setb	F.PWM_TIMER0_OVERFLOW
	 Timer0_Overflow_Value= A;
	 ajmp	t0_int_pwm_on_timer_set

	 t0_int_pwm_on_set_timer:
	 TL0= A;
	 t0_int_pwm_on_timer_set:
	 #endif
	 // Set other variables
	 TL1=0;					// Reset timer1
	 #if (MCU_50MHZ==1)
	 TH1=0;
	 #endif
	 #if (MODE==TAIL_MODE)				// Tail
	 Pwm_On_Cnt=0;			// Reset pwm on event counter
	 #endif
	 setb	F.PWM_ON				// Set pwm on flag
	 t0_int_pwm_on_exit_no_timer_update:
	 // Exit interrupt
	 pop	ACC			// Restore preserved registers
	 pop	PSW
	 //zzEA = 1			// Enable all interrupts
	 */
} // t0_int_pwm_on_exit

//___________________________________________________________________________
//
// Timer2 interrupt routine
//
// No assumptions
//
//___________________________________________________________________________

void t2_int(void) { // Happens every 128us for low byte and every 32ms for high byte
	/*
	 EA=0;
	 clr	ET2			// Disable timer2 interrupts
	 anl	EIE1, #0EFh	// Disable PCA0 interrupts
	 push	PSW			// Preserve registers through interrupt
	 push	ACC
	 setb	PSW.3		// Select register bank 1 for interrupt routines
	 //zzEA = 1
	 #if (MCU_50MHZ==1
	 A = Clock_Set_At_50MHz;
	 jz 	t2_int_start

	 // Check skip variable
	 A = Skip_T2_Int;
	 jz	t2_int_start				// Execute this interrupt

	 Skip_T2_Int=0;
	 ajmp	t2_int_exit

	 t2_int_start:
	 Skip_T2_Int, #1			// Skip next interrupt
	 #endif
	 // Clear low byte interrupt flag
	 clr	TF2L						// Clear interrupt flag
	 // Check RC pulse timeout counter
	 A = Rcp_Timeout_Cnt			// RC pulse timeout count zero?
	 jz	t2_int_pulses_absent		// Yes - pulses are absent

	 // Decrement timeout counter (if PWM)
	 jb	F.RCP_PPM, t2_int_skip_start	// If flag is set (PPM) - branch

	 dec	Rcp_Timeout_Cnt			// No - decrement
	 ajmp	t2_int_skip_start

	 t2_int_pulses_absent:
	 // Timeout counter has reached zero, pulses are absent
	 Temp1 = #RCP_MIN			// RCP_MIN as default
	 Temp2 = #RCP_MIN
	 Read_Rcp_Int 					// Look at value of Rcp_In
	 jnb	ACC.Rcp_In, ($+5)			// Is it high?
	 Temp1 = #RCP_MAX			// Yes - set RCP_MAX
	 Rcp_Int_First 					// Set interrupt trig to first again
	 Rcp_Clear_Int_Flag 				// Clear interrupt flag
	 clr	F.RCP_EDGE_NO			// Set first edge flag
	 Read_Rcp_Int 					// Look once more at value of Rcp_In
	 jnb	ACC.Rcp_In, ($+5)			// Is it high?
	 Temp2 = #RCP_MAX			// Yes - set RCP_MAX
	 C=0;
	 A = Temp1
	 subb	A, Temp2					// Compare the two readings of Rcp_In
	 jnz 	t2_int_pulses_absent		// Go back if they are not equal

	 jnb	F.RCP_MEAS_PWM_FREQ, ($+6)	// Is measure RCP pwm frequency flag set?

	 Rcp_Timeout_Cnt, #RCP_TIMEOUT	// Yes - set timeout count to start value

	 jb	F.RCP_PPM, t2_int_ppm_timeout_set	// If flag is set (PPM) - branch

	 Rcp_Timeout_Cnt, #RCP_TIMEOUT	// For PWM, set timeout count to start value

	 t2_int_ppm_timeout_set:
	 New_Rcp, Temp1				// Store new pulse length
	 setb	F.RCP_UPDATED		 	// Set updated flag

	 t2_int_skip_start:
	 // Check RC pulse skip counter
	 A = Rcp_Skip_Cnt
	 jz 	t2_int_skip_end			// If RC pulse skip count is zero - end skipping RC pulse detection

	 // Decrement skip counter (only if edge counter is zero)
	 dec	Rcp_Skip_Cnt				// Decrement
	 ajmp	t2_int_setpoint_update_start

	 t2_int_skip_end:
	 jb	F.RCP_PPM, t2_int_setpoint_update_start	// If flag is set (PPM) - branch

	 // Skip counter has reached zero, start looking for RC pulses again
	 Rcp_Int_Enable 				// Enable RC pulse interrupt
	 Rcp_Clear_Int_Flag 				// Clear interrupt flag

	 t2_int_setpoint_update_start:
	 // Process updated RC pulse
	 jb	F.RCP_UPDATED, ($+5)	// Is there an updated RC pulse available?
	 ajmp	t2_int_current_pwm_done		// No - update pwm limits and exit

	 Temp1 = New_Rcp				// Load new pulse value
	 jb	F.RCP_MEAS_PWM_FREQ, ($+5)	// If measure RCP pwm frequency flag set - do not clear flag

	 clr	F.RCP_UPDATED		 	// Flag that pulse has been evaluated

	 // Use a gain of 1.0625x for pwm input if not governor mode
	 jb	F.RCP_PPM, t2_int_pwm_min_run	// If flag is set (PPM) - branch

	 #if (MODE==MAIN_MODE	// Main - do not adjust gain
	 ajmp	t2_int_pwm_min_run
	 #else

	 #if (MODE==MULTI_MODE	// Multi
	 Temp2 = #P.Gov_Mode		// Closed loop mode?
	 cjne	@Temp2, #4, t2_int_pwm_min_run// Yes - branch
	 #endif

	 // Limit the maximum value to avoid wrap when scaled to pwm range
	 C=0;
	 A = Temp1
	 subb	A, #240			// 240 = (255/1.0625) Needs to be updated according to multiplication factor below
	 jc	t2_int_setpoint_update_mult

	 A = #240			// Set requested pwm to max
	 Temp1 = A

	 t2_int_setpoint_update_mult:
	 // Multiply by 1.0625 (optional adjustment gyro gain)
	 A = Temp1
	 swap	A			// After this "0.0625"
	 anl	A, #0Fh
	 add	A, Temp1
	 Temp1 = A
	 // Adjust tail gain
	 Temp2 = #P.Motor_Gain
	 cjne	@Temp2, #3, ($+5)			// Is gain 1?
	 ajmp	t2_int_pwm_min_run			// Yes - skip adjustment

	 C=0;
	 A/=2;			// After this "0.5"
	 C=0;
	 A/=2;			// After this "0.25"
	 Bit_Access_Int, @Temp2				// (Temp2 has #P.Motor_Gain)
	 jb	Bit_Access_Int.0, t2_int_setpoint_gain_corr	// Branch if bit 0 in gain is set

	 C=0;
	 A/=2;			// After this "0.125"

	 t2_int_setpoint_gain_corr:
	 jb	Bit_Access_Int.2, t2_int_setpoint_gain_pos	// Branch if bit 2 in gain is set

	 C=0;
	 xch	A, Temp1
	 subb	A, Temp1					// Apply negative correction
	 Temp1 = A
	 ajmp	t2_int_pwm_min_run

	 t2_int_setpoint_gain_pos:
	 add	A, Temp1					// Apply positive correction
	 Temp1 = A;
	 jnc	t2_int_pwm_min_run			// Above max?

	 A = #0xff					// Yes - limit
	 Temp1 = A
	 #endif

	 t2_int_pwm_min_run:
	 #if (MODE==TAIL_MODE	// Tail - limit minimum pwm
	 // Limit minimum pwm
	 C=0;
	 A = Temp1;
	 subb	A, Pwm_Motor_Idle			// Is requested pwm lower than minimum?
	 jnc	t2_int_pwm_update			// No - branch

	 A = Pwm_Motor_Idle			// Yes - limit pwm to Pwm_Motor_Idle
	 Temp1 = A
	 #endif

	 t2_int_pwm_update:
	 // Update requested_pwm
	 Requested_Pwm= Temp1;		// Set requested pwm
	 // Limit pwm during direct start
	 jnb	F.STARTUP_PHASE, t2_int_current_pwm_update

	 #if (MODE==MULTI_MODE)	// Multi
	 A = Requested_Pwm;
	 A+=8;					// Add an extra power boost during start
	 Requested_Pwm= A;

	 jnc	($+5)
	 Requested_Pwm, #0xff
	 #endif
	 C=0;
	 A = Requested_Pwm;			// Limit pwm during direct start
	 A -=Pwm_Limit;
	 jc	t2_int_current_pwm_update

	 Requested_Pwm=Pwm_Limit;

	 t2_int_current_pwm_update:
	 #if (MODE==MAIN_MODE) || (MODE==MULTI_MODE)	// Main or multi
	 Temp1 = P.Gov_Mode;		// Governor mode?
	 cjne	@Temp1, #4, t2_int_pwm_exit	// Yes - branch
	 #endif

	 Current_Pwm= Requested_Pwm;	// Set equal as default

	 t2_int_current_pwm_done:
	 #if (MODE >= 1)	// Tail or multi
	 // Set current_pwm_limited
	 Temp1 = Current_Pwm			// Default not limited
	 C=0;
	 A = Current_Pwm;				// Check against limit
	 A-=Pwm_Limit;
	 jc	($+4)					// If current pwm below limit - branch

	 Temp1 = Pwm_Limit;			// Limit pwm

	 #if (MODE==MULTI_MODE)	// Multi
	 // Limit pwm for low rpms
	 C=0;
	 A = Temp1;					// Check against limit
	 A-=Pwm_Limit_Low_Rpm;
	 jc	($+4)					// If current pwm below limit - branch

	 Temp1 = Pwm_Limit_Low_Rpm;		// Limit pwm

	 #endif
	 Current_Pwm_Limited, Temp1
	 #endif
	 t2_int_pwm_exit:
	 // Set demag enabled if pwm is above limit
	 C=0;
	 A = Current_Pwm_Limited;
	 A-=0x40;					// Set if above 25%
	 jc	($+4)

	 setb	F.DEMAG_ENABLED

	 t2_int_exit:
	 // Check if high byte flag is set
	 jb	TF2H, t2h_int
	 pop	ACC			// Restore preserved registers
	 pop	PSW
	 clr	PSW.3		// Select register bank 0 for main program routines
	 orl	EIE1, #10h	// Enable PCA0 interrupts
	 setb	ET2			// Enable timer2 interrupts
	 */
}

void t2h_int(void) {
	/*
	 #if (MCU_50MHZ==1)
	 A = Clock_Set_At_50MHz;
	 jz 	t2h_int_start

	 // Check skip variable
	 A = Skip_T2h_Int;
	 jz	t2h_int_start				// Execute this interrupt

	 Skip_T2h_Int = 0;
	 ajmp	t2h_int_exit

	 t2h_int_start:
	 Skip_T2h_Int, #1			// Skip next interrupt
	 #endif
	 // High byte interrupt (happens every 32ms)
	 TF2H = 0;					// Clear interrupt flag
	 Temp1 = GOV_SPOOLRATE;	// Load governor spool rate
	 // Check RC pulse timeout counter (used here for PPM only)
	 A = Rcp_Timeout_Cnt;			// RC pulse timeout count zero?
	 if (A==0) goto	t2h_int_setpoint_stop_check;		// Yes - do not decrement

	 // Decrement timeout counter (if PPM)
	 jnb	F.RCP_PPM, t2h_int_setpoint_stop_check	// If flag is not set (PWM) - branch

	 Rcp_Timeout_Cnt--;			// No flag set (PPM) - decrement

	 t2h_int_setpoint_stop_check:
	 // Check RC pulse against stop value
	 C=0;
	 A = New_Rcp;				// Load new pulse value
	 subb	A, #RCP_STOP				// Check if pulse is below stop value
	 jc	t2h_int_setpoint_stop

	 // RC pulse higher than stop value, reset stop counter
	 Rcp_Stop_Cnt=0;			// Reset rcp stop counter
	 ajmp	t2h_int_setpoint_gov_pwm

	 t2h_int_setpoint_stop:
	 // RC pulse less than stop value
	 Auto_Bailout_Armed = 0;		// Disarm bailout
	 Spoolup_Limit_Cnt = 0;
	 A = Rcp_Stop_Cn;			// Increment stop counter
	 A++;
	 Rcp_Stop_Cnt= A;
	 jnc	t2h_int_setpoint_gov_pwm			// Branch if counter has not wrapped

	 Rcp_Stop_Cnt=0xff;			// Set stop counter to max

	 t2h_int_setpoint_gov_pwm:
	 #if (MODE==MAIN_MODE)	// Main
	 // Update governor variables
	 Temp2 = #P.Gov_Mode;			// Governor target by arm mode?
	 cjne	@Temp2, #2, t2h_int_setpoint_gov_by_setup	// No - branch

	 A = Gov_Active;					// Is governor active?
	 jz	t2h_int_setpoint_gov_by_tx			// No - branch (this ensures soft spoolup by tx)

	 C=0;
	 A = Requested_Pwm
	 subb	A, #50						// Is requested pwm below 20%?
	 jc	t2h_int_setpoint_gov_by_tx			// Yes - branch (this enables a soft spooldown)

	 Requested_Pwm, Gov_Arm_Target		// Yes - load arm target

	 t2h_int_setpoint_gov_by_setup:
	 Temp2 = #P.Gov_Mode;			// Governor target by setup mode?
	 cjne	@Temp2, #3, t2h_int_setpoint_gov_by_tx		// No - branch

	 A = Gov_Active;					// Is governor active?
	 jz	t2h_int_setpoint_gov_by_tx			// No - branch (this ensures soft spoolup by tx)

	 C=0;
	 A = Requested_Pwm;
	 subb	A, #50						// Is requested pwm below 20%?
	 jc	t2h_int_setpoint_gov_by_tx			// Yes - branch (this enables a soft spooldown)

	 Temp2 = #P.Gov_Setup_Target;		// Gov by setup - load setup target
	 Requested_Pwm, @Temp2

	 t2h_int_setpoint_gov_by_tx:
	 C=0;
	 A = Governor_Req_Pwm;
	 subb	A, Requested_Pwm				// Is governor requested pwm equal to requested pwm?
	 jz	t2h_int_setpoint_gov_pwm_done			// Yes - branch

	 jc	t2h_int_setpoint_gov_pwm_inc			// No - if lower, then increment

	 Governor_Req_Pwm--;				// No - if higher, then decrement
	 ajmp	t2h_int_setpoint_gov_pwm_done

	 t2h_int_setpoint_gov_pwm_inc:
	 Governor_Req_Pwm++;				// Increment

	 t2h_int_setpoint_gov_pwm_done:
	 djnz	Temp1, t2h_int_setpoint_gov_pwm		// If not number of steps processed - go back

	 Spoolup_Limit_Cnt++;				// Increment spoolup count
	 A = Spoolup_Limit_Cnt;
	 jnz	($+4)						// Wrapped?

	 dec	Spoolup_Limit_Cnt				// Yes - decrement

	 djnz	Spoolup_Limit_Skip, t2h_int_exit	// Jump if skip count is not reached

	 Spoolup_Limit_Skip, #1			// Reset skip count. Default is fast spoolup
	 Temp1 = 5;						// Default fast increase

	 C=0;
	 A = Spoolup_Limit_Cnt;
	 subb	A, Main_Spoolup_Time_3x			// No spoolup until 3*N*32ms

	 jc	t2h_int_exit

	 C=0;
	 A = Spoolup_Limit_Cnt
	 subb	A, Main_Spoolup_Time_10x			// Slow spoolup until "100"*N*32ms
	 jnc	t2h_int_setpoint_limit_middle_ramp

	 Temp1 = #1						// Slow initial spoolup
	 Spoolup_Limit_Skip, #3
	 jmp	t2h_int_setpoint_set_limit

	 t2h_int_setpoint_limit_middle_ramp:
	 C=0;
	 A = Spoolup_Limit_Cnt
	 subb	A, Main_Spoolup_Time_15x			// Faster spoolup until "150"*N*32ms
	 jnc	t2h_int_setpoint_set_limit

	 Temp1 = #1						// Faster middle spoolup
	 Spoolup_Limit_Skip, #1

	 t2h_int_setpoint_set_limit:
	 // Do not increment spoolup limit if higher pwm is not requested, unless governor is active
	 C=0;
	 A = Pwm_Limit_Spoolup
	 subb	A, Current_Pwm
	 jc	t2h_int_setpoint_inc_limit			// If Current_Pwm is larger than Pwm_Limit_Spoolup - branch

	 Temp2 = #P.Gov_Mode			// Governor mode?
	 cjne	@Temp2, #4, ($+5)
	 ajmp	t2h_int_setpoint_bailout_arm			// No - branch

	 A = Gov_Active					// Is governor active?
	 jnz	t2h_int_setpoint_inc_limit			// Yes - branch

	 Pwm_Limit_Spoolup, Current_Pwm	// Set limit to what current pwm is
	 A = Spoolup_Limit_Cnt			// Check if spoolup limit count is 255. If it is, then this is a "bailout" ramp
	 inc	A
	 jz	($+5)

	 Spoolup_Limit_Cnt, Main_Spoolup_Time_3x	// Stay in an early part of the spoolup sequence (unless "bailout" ramp)
	 Spoolup_Limit_Skip, #1			// Set skip count
	 Governor_Req_Pwm, #60			// Set governor requested speed to ensure that it requests higher speed
	 // 20=Fail on jerk when governor activates
	 // 30=Ok
	 // 100=Fail on small governor settling overshoot on low headspeeds
	 // 200=Fail on governor settling overshoot
	 jmp	t2h_int_exit					// Exit

	 t2h_int_setpoint_inc_limit:
	 A = Pwm_Limit_Spoolup			// Increment spoolup pwm
	 add	A, Temp1
	 jnc	t2h_int_setpoint_no_limit			// If below 255 - branch

	 Pwm_Limit_Spoolup, #0xff
	 ajmp	t2h_int_setpoint_bailout_arm

	 t2h_int_setpoint_no_limit:
	 Pwm_Limit_Spoolup, A
	 t2h_int_setpoint_bailout_arm:
	 A = Pwm_Limit_Spoolup
	 inc	A
	 jnz	t2h_int_exit

	 Auto_Bailout_Armed=255;			// Arm bailout
	 Spoolup_Limit_Cnt=255

	 #endif
	 t2h_int_exit:
	 pop	ACC			// Restore preserved registers
	 pop	PSW
	 clr	PSW.3		// Select register bank 0 for main program routines
	 orl	EIE1, #10h	// Enable PCA0 interrupts
	 setb	ET2			// Enable timer2 interrupts

	 */
}

//___________________________________________________________________________
//
// Timer3 interrupt routine
//
// No assumptions
//
//___________________________________________________________________________
void t3_int(void) { // Used for commutation timing
/*
 push	PSW				// Preserve registers through interrupt
 push	ACC
 clr 	EA				// Disable all interrupts
 anl	TMR3CN, #7Fh		// Clear timer3 interrupt flag
 anl	EIE1, #7Fh		// Disable timer3 interrupts
 clr	F.T3_PENDING 	// Flag that timer has wrapped
 #if (MCU_50MHZ==1)
 C=0;
 A = Next_Wt;
 rlc	A
 Next_Wt_L= A;
 A = Next_Wt;
 rlc	A
 Next_Wt_H= A;
 #endif
 // Set up next wait
 TMR3CN, #00h		// Timer3 disabled
 C=0;
 A=0;
 subb	A, Next_Wt_L		// Set wait value
 TMR3L, A
 A=0;
 subb	A, Next_Wt_H
 TMR3H= A;
 TMR3CN= 0x04;		// Timer3 enabled
 pop	ACC				// Restore preserved registers
 pop	PSW
 //zzEA = 1				// Enable all interrupts
 */
}

//___________________________________________________________________________
//
// PCA interrupt routine
//
// No assumptions
//
//___________________________________________________________________________
/*
 void pca_int(void) {	// Used for RC pulse timing
 //zz EA = 0;
 anl	EIE1, #0EFh	// Disable PCA0 interrupts
 clr	ET2			// Disable timer2 interrupts
 push	PSW			// Preserve registers through interrupt
 push	ACC
 push	B
 setb	PSW.3		// Select register bank 1 for interrupt routines
 //zzEA = 1
 // Get the PCA counter values
 Get_Rcp_Capture_Values();
 // Clear interrupt flag
 Rcp_Clear_Int_Flag();

 jnb	F.RCP_EDGE_NO, ($+5)	// Is it a first edge trig?
 ajmp pca_int_second_meas_pwm_freq	// No - branch to second

 Rcp_Int_Second();					// Yes - set second edge trig
 setb	F.RCP_EDGE_NO			// Set second edge flag
 // Read RC signal level
 Read_Rcp_Int();
 // Test RC signal level
 jb	ACC.Rcp_In, ($+5)			// Is it high?
 ajmp	pca_int_fail_minimum		// No - jump to fail minimum

 // RC pulse was high, store RC pulse start timestamp
 Rcp_Prev_Edge= Temp1;

 pca_int_exit();				// Exit
 }

 void pca_int_fail_minimum(void) {
 // Prepare for next interrupt
 Rcp_Int_First();					// Set interrupt trig to first again
 Rcp_Clear_Int_Flag(); 				// Clear interrupt flag
 clr	F.RCP_EDGE_NO			// Set first edge flag
 jnb	F.RCP_PPM, ($+5)		// If flag is not set (PWM) - branch

 ajmp	pca_int_set_timeout			// If PPM - ignore trig as noise

 Temp1 = RCP_MIN;			// Set RC pulse value to minimum
 Read_Rcp_Int(); 					// Test RC signal level again
 jnb	ACC.Rcp_In, ($+5)			// Is it high?

 ajmp	pca_int_set_timeout			// Yes - set new timeout and exit

 New_Rcp = Temp1;				// Store new pulse length
 ajmp	pca_int_limited			// Set new RC pulse, new timeout and exit

 pca_int_second_meas_pwm_freq:
 // Prepare for next interrupt
 Rcp_Int_First(); 					// Set first edge trig
 clr	F.RCP_EDGE_NO			// Set first edge flag
 // Check if pwm frequency shall be measured
 jb	F.RCP_MEAS_PWM_FREQ, ($+5)	// Is measure RCP pwm frequency flag set?
 ajmp	pca_int_fall				// No - skip measurements

 // Set second edge trig only during pwm frequency measurement
 Rcp_Int_Second 				// Set second edge trig
 Rcp_Clear_Int_Flag 				// Clear interrupt flag
 setb	F.RCP_EDGE_NO			// Set second edge flag
 // Store edge data to RAM
 Rcp_Edge_L= Temp1;
 Rcp_Edge_H=Temp2;
 // Calculate pwm frequency
 C=0;
 A = Temp1;
 subb	A, Rcp_PrePrev_Edge_L
 Temp1 = A;
 A = Temp2
 subb	A, Rcp_PrePrev_Edge_H
 Temp2 = A;
 A=0;
 Temp4= A;
 Temp7= 2;					// Set default period tolerance requirement (MSB)
 Temp3  = 0;					// (LSB)
 // Check if pulse is too short
 C=0;
 A = Temp1;
 subb	A, #low(140)				// If pulse below 70us, not accepted
 A = Temp2;
 subb	A, #high(140)
 jnc	rcp_int_check_12kHz

 Rcp_Period_Diff_Accepted=0;	// Set not accepted
 ajmp	pca_int_store_data

 rcp_int_check_12kHz:
 // Check if pwm frequency is 12kHz
 C=0;
 A = Temp1;
 subb	A, #low(200)				// If below 100us, 12kHz pwm is assumed
 A = Temp2;
 subb	A, #high(200)
 jnc	pca_int_check_8kHz

 A=0;
 setb	ACC.RCP_PWM_FREQ_12KHZ
 Temp4= A;
 Temp3  = 10;				// Set period tolerance requirement (LSB)
 ajmp	pca_int_restore_edge_set_msb

 pca_int_check_8kHz:
 // Check if pwm frequency is 8kHz
 C=0;
 A = Temp1;
 subb	A, #low(360)				// If below 180us, 8kHz pwm is assumed
 A = Temp2;
 subb	A, #high(360)
 jnc	pca_int_check_4kHz

 A=0;
 setb	ACC.RCP_PWM_FREQ_8KHZ
 Temp4= A;
 Temp3  = 15;				// Set period tolerance requirement (LSB)
 ajmp	pca_int_restore_edge_set_msb

 pca_int_check_4kHz:
 // Check if pwm frequency is 4kHz
 C=0;
 A = Temp1;
 subb	A, #low(720)				// If below 360us, 4kHz pwm is assumed
 A = Temp2;
 subb	A, #high(720)
 jnc	pca_int_check_2kHz

 A=0;
 setb	ACC.RCP_PWM_FREQ_4KHZ
 Temp4= A;
 Temp3  =30;				// Set period tolerance requirement (LSB)
 ajmp	pca_int_restore_edge_set_msb

 pca_int_check_2kHz:
 // Check if pwm frequency is 2kHz
 C=0;
 A = Temp1;
 subb	A, #low(1440)				// If below 720us, 2kHz pwm is assumed
 A = Temp2;
 subb	A, #high(1440)
 jnc	pca_int_check_1kHz

 A=0;
 setb	ACC.RCP_PWM_FREQ_2KHZ
 Temp4= A;
 Temp3  = 60;				// Set period tolerance requirement (LSB)
 ajmp	pca_int_restore_edge_set_msb

 pca_int_check_1kHz:
 // Check if pwm frequency is 1kHz
 C=0;
 A = Temp1;
 subb	A, #low(2200)				// If below 1100us, 1kHz pwm is assumed
 A = Temp2;
 subb	A, #high(2200)
 jnc	pca_int_restore_edge

 A=0;
 setb	ACC.RCP_PWM_FREQ_1KHZ
 Temp4= A;
 Temp3  = 120;				// Set period tolerance requirement (LSB)

 pca_int_restore_edge_set_msb:
 Temp7 =0;					// Set period tolerance requirement (MSB)
 pca_int_restore_edge:
 // Calculate difference between this period and previous period
 C=0;
 A = Temp1
 subb	A, Rcp_Prev_Period_L
 Temp5, A
 A = Temp2
 subb	A, Rcp_Prev_Period_H
 Temp6, A
 // Make positive
 jnb	ACC.7, pca_int_check_diff
 A = Temp5
 cpl	A
 add	A, #1
 Temp5, A
 A = Temp6
 cpl	A
 Temp6, A

 pca_int_check_diff:
 // Check difference
 Rcp_Period_Diff_Accepted, #0		// Set not accepted as default
 C=0;
 A = Temp5
 subb	A, Temp3						// Check difference
 A = Temp6
 subb	A, Temp7
 jnc	pca_int_store_data

 Rcp_Period_Diff_Accepted, #1		// Set accepted

 pca_int_store_data:
 // Store previous period
 Rcp_Prev_Period_L, Temp1
 Rcp_Prev_Period_H, Temp2
 // Restore edge data from RAM
 Temp1 = Rcp_Edge_L
 Temp2 = Rcp_Edge_H
 // Store pre previous edge
 Rcp_PrePrev_Edge_L, Temp1
 Rcp_PrePrev_Edge_H, Temp2
 Temp1 = #RCP_VALIDATE
 ajmp	pca_int_limited

 pca_int_fall:
 // RC pulse edge was second, calculate new pulse length
 C=0;
 A = Temp1
 subb	A, Rcp_Prev_Edge_L
 Temp1 = A
 A = Temp2
 subb	A, Rcp_Prev_Edge_H
 Temp2 = A
 jnb	F.RCP_PWM_FREQ_12KHZ, ($+5)	// Is RC input pwm frequency 12kHz?
 ajmp	pca_int_pwm_divide_done			// Yes - branch forward

 jnb	F.RCP_PWM_FREQ_8KHZ, ($+5)	// Is RC input pwm frequency 8kHz?
 ajmp	pca_int_pwm_divide_done			// Yes - branch forward

 jnb	F.RCP_PWM_FREQ_4KHZ, ($+5)	// Is RC input pwm frequency 4kHz?
 ajmp	pca_int_pwm_divide				// Yes - branch forward

 jb	F.RCP_PPM_ONESHOT125, ($+5)
 ajmp	rcp_int_fall_not_oneshot

 A = Temp2						// Oneshot125 - move to I_Temp5/6
 Temp6, A
 A = Temp1
 Temp5, A
 ajmp	rcp_int_fall_check_range

 rcp_int_fall_not_oneshot:
 A = Temp2						// No - 2kHz. Divide by 2
 C=0;
 A/=2;
 Temp2 = A
 A = Temp1
 A/=2;
 Temp1 = A

 jnb	F.RCP_PWM_FREQ_2KHZ, ($+5)	// Is RC input pwm frequency 2kHz?
 ajmp	pca_int_pwm_divide				// Yes - branch forward

 A = Temp2						// No - 1kHz. Divide by 2 again
 C=0;
 A/=2;
 Temp2 = A
 A = Temp1
 A/=2;
 Temp1 = A

 jnb	F.RCP_PWM_FREQ_1KHZ, ($+5)	// Is RC input pwm frequency 1kHz?
 ajmp	pca_int_pwm_divide				// Yes - branch forward

 A = Temp2						// No - PPM. Divide by 2 (to bring range to 256) and move to Temp5/6
 C=0;
 A/=2;
 Temp6= A;
 A = Temp1;
 A/=2;
 Temp5=A;
 rcp_int_fall_check_range:
 // Skip range limitation if pwm frequency measurement
 jb	F.RCP_MEAS_PWM_FREQ, pca_int_ppm_check_full_range

 // Check if 2160us or above (in order to ignore false pulses)
 C=0;
 A = Temp5;						// Is pulse 2160us or higher?
 subb	A, #28
 A = Temp6;
 subb A, #2
 jc	($+4)						// No - proceed

 ajmp	pca_int_ppm_outside_range		// Yes - ignore pulse

 pca_int_ppm_below_full_range:
 // Check if below 800us (in order to ignore false pulses)
 A = Temp6;
 jnz	pca_int_ppm_check_full_range

 C=0;
 A = Temp5;						// Is pulse below 800us?
 A -= 200;
 jnc	pca_int_ppm_check_full_range		// No - proceed

 pca_int_ppm_outside_range:
 inc	Rcp_Outside_Range_Cnt
 C=0;
 A = Rcp_Outside_Range_Cnt;
 A-=10;						// Allow a given number of outside pulses
 jnc	($+4)
 ajmp	pca_int_set_timeout				// If below limit - ignore pulse

 New_Rcp=0;					// Set pulse length to zero
 setb	F.RCP_UPDATED		 		// Set updated flag
 ajmp	pca_int_set_timeout

 pca_int_ppm_check_full_range:
 A = Rcp_Outside_Range_Cnt;
 jz	($+4)

 Rcp_Outside_Range_Cnt--;

 // Calculate "1000us" plus throttle minimum
 A = 0;						// Set 1000us as default minimum
 jb	F.FULL_THROTTLE_RANGE, pca_int_ppm_calculate	// Check if full range is chosen

 #if (MODE >= 1)	// Tail or multi
 Temp1 = #P.Direction			// Check if bidirectional operation
 A = @Temp1;
 #endif
 Temp1 = #P.Ppm_Min_Throttle		// Min throttle value is in 4us units
 #if (MODE >= 1)	// Tail or multi
 cjne	A, #3, ($+5)

 Temp1 = #P.Ppm_Center_Throttle	// Center throttle value is in 4us units
 #endif
 A = @Temp1;

 pca_int_ppm_calculate:
 add	A, #250						// Add 1000us to minimum
 Temp7= A;
 A=0;
 addc	A, #0
 Temp8, A

 C=0;
 A = Temp5;						// Subtract minimum
 subb	A, Temp7
 Temp5, A
 A = Temp6
 subb	A, Temp8
 Temp6= A;
 #if (MODE >= 1)	// Tail or multi
 Bit_Access_Int.0, C
 Temp1 = #P.Direction			// Check if bidirectional operation
 A = @Temp1;
 cjne	A, #3, pca_int_ppm_bidir_dir_set	// No - branch

 C= Bit_Access_Int.0;
 jnc	pca_int_ppm_bidir_fwd			// If result is positive - branch

 pca_int_ppm_bidir_rev:
 jb	F.PGM_DIR_REV, pca_int_ppm_bidir_dir_set	// If same direction - branch

 EA=0;							// Direction change, turn off all fets
 setb	F.PGM_DIR_REV
 ajmp	pca_int_ppm_bidir_dir_change

 pca_int_ppm_bidir_fwd:
 jnb	F.PGM_DIR_REV, pca_int_ppm_bidir_dir_set	// If same direction - branch

 EA=0;							// Direction change, turn off all fets
 clr	F.PGM_DIR_REV

 pca_int_ppm_bidir_dir_change:
 All_nFETs_Off();
 All_pFETs_Off();
 jb	F.STARTUP_PHASE, ($+5)		// Do not brake when starting

 setb	F.DIR_CHANGE_BRAKE			// Set brake flag

 //zzEA = 1
 pca_int_ppm_bidir_dir_set:
 C=Bit_Access_Int.0;
 #endif
 jnc	pca_int_ppm_neg_checked			// If result is positive - branch

 #if (MODE >= 1)	// Tail or multi
 A = @Temp1;						// Check if bidirectional operation (Temp1 has P.Direction)
 cjne	A, #3, pca_int_ppm_unidir_neg 	// No - branch

 A = Temp5;						// Change sign
 cpl	A;
 add	A, #1
 Temp5=A;
 A = Temp6;
 cpl	A;
 addc	A, #0
 Temp6= A;
 jmp	pca_int_ppm_neg_checked

 pca_int_ppm_unidir_neg:
 #endif
 Temp1 = RCP_MIN;				// Yes - set to minimum
 Temp2 = 0;
 pca_int_pwm_divide_done();

 pca_int_ppm_neg_checked:
 #if (MODE >= 1	// Tail or multi
 Temp1 = #P.Direction			// Check if bidirectional operation
 A = @Temp1
 cjne	A, #3, pca_int_ppm_bidir_done		// No - branch

 A = Temp5						// Multiply value by 2
 rlc	A
 Temp5 A
 A = Temp6
 rlc	A
 Temp6 A
 C=0;							// Subtract deadband
 A = Temp5
 subb	A, #10
 Temp5, A
 A = Temp6
 subb	A, #0
 Temp6, A
 jnc	pca_int_ppm_bidir_done

 Temp5, #RCP_MIN
 Temp6, #0

 pca_int_ppm_bidir_done:
 #endif
 C=0;							// Check that RC pulse is within legal range (max 255)
 A = Temp5;
 subb	A, #RCP_MAX
 A = Temp6;
 subb	A, #0
 jc	pca_int_ppm_max_checked

 Temp1 = #RCP_MAX
 Temp2 = 0;
 pca_int_pwm_divide_done();
 }

 pca_int_ppm_max_checked:
 A = Temp5						// Multiply throttle value by gain
 B, Ppm_Throttle_Gain
 mul	AB
 xch	A, B
 C, B.7						// Multiply result by 2 (unity gain is 128)
 rlc	A
 Temp1 = A						// Transfer to Temp1/2
 Temp2 = #0
 jc	pca_int_ppm_limit_after_mult

 jmp	pca_int_limited

 pca_int_ppm_limit_after_mult:
 Temp1 = #RCP_MAX
 Temp2 = #0
 jmp	pca_int_limited

 }

 void pca_int_pwm_divide(void) {
 A = Temp2						// Divide by 2
 C=0;
 A/=2;
 Temp2 = A;
 A = Temp1
 A/=2;
 Temp1 = A

 pca_int_pwm_divide_done:
 jnb	F.RCP_PWM_FREQ_12KHZ, pca_int_check_legal_range	// Is RC input pwm frequency 12kHz?
 A = Temp2						// Yes - check that value is not more than 255
 jz	($+4)

 Temp1 = #RCP_MAX

 C=0;
 A = Temp1						// Multiply by 1.5
 A/=2;
 addc	A, Temp1
 Temp1 = A
 A=0;
 addc	A, #0
 Temp2 = A

 pca_int_check_legal_range:
 // Check that RC pulse is within legal range
 C=0;
 A = Temp1
 subb	A, #RCP_MAX
 A = Temp2
 subb	A, #0
 jc	pca_int_limited

 Temp1 = #RCP_MAX

 pca_int_limited:
 // RC pulse value accepted
 New_Rcp, Temp1				// Store new pulse length
 setb	F.RCP_UPDATED		 	// Set updated flag
 jb	F.RCP_MEAS_PWM_FREQ, ($+5)	// Is measure RCP pwm frequency flag set?

 ajmp	pca_int_set_timeout			// No - skip measurements

 A = #((1 SHL RCP_PWM_FREQ_1KHZ)+(1 SHL RCP_PWM_FREQ_2KHZ)+(1 SHL RCP_PWM_FREQ_4KHZ)+(1 SHL RCP_PWM_FREQ_8KHZ)+(1 SHL RCP_PWM_FREQ_12KHZ))
 cpl	A
 anl	A, Flags3					// Clear all pwm frequency flags
 orl	A, Temp4					// Store pwm frequency value in flags
 Flags3, A
 clr	F.RCP_PPM				// Default, flag is not set (PWM)
 A=0;
 add	A, Temp4					// Check if all flags are cleared
 jnz	pca_int_set_timeout

 setb	F.RCP_PPM				// Set flag (PPM)

 pca_int_set_timeout:
 Rcp_Timeout_Cnt, #RCP_TIMEOUT	// Set timeout count to start value
 jnb	F.RCP_PPM, pca_int_ppm_timeout_set	// If flag is not set (PWM) - branch

 Rcp_Timeout_Cnt, #RCP_TIMEOUT_PPM	// No flag set means PPM. Set timeout count

 pca_int_ppm_timeout_set:
 jnb	F.RCP_MEAS_PWM_FREQ, ($+5)	// Is measure RCP pwm frequency flag set?

 ajmp pca_int_exit				// Yes - exit

 jb	F.RCP_PPM, pca_int_exit	// If flag is set (PPM) - branch

 Rcp_Int_Disable 				// Disable RC pulse interrupt

 pca_int_exit:	// Exit interrupt routine
 Rcp_Skip_Cnt, #RCP_SKIP_RATE	// Load number of skips
 jnb	F.RCP_PPM, ($+6)		// If flag is not set (PWM) - branch

 Rcp_Skip_Cnt, #10			// Load number of skips

 pop	B			// Restore preserved registers
 pop	ACC
 pop	PSW
 clr	PSW.3		// Select register bank 0 for main program routines
 setb	ET2			// Enable timer2 interrupts
 orl	EIE1, #10h	// Enable PCA0 interrupts
 //reti

 }

 */
//___________________________________________________________________________
//
// Beeper routines (4 different entry points)
//
// No assumptions
//
//___________________________________________________________________________

void beep(void) { // Beep loop start
/*
 Temp5=Current_Pwm_Limited;	// Store value
 Current_Pwm_Limited=1;		// Set to a nonzero value
 Temp2 = 2;					// Must be an even number (or direction will change)
 beep_onoff:
 //zz	cpl	F.PGM_DIR_REV			// Toggle between using A fet and C fet
 A=0;
 BpFET_off();			// BpFET off
 //zz	djnz	ACC, $		// Allow some time after pfet is turned off
 BnFET_on();			// BnFET on (in order to charge the driver of the BpFET)
 //zz	djnz	ACC, $		// Let the nfet be turned on a while
 BnFET_off();			// BnFET off again
 //zz	djnz	ACC, $		// Allow some time after nfet is turned off
 BpFET_on();			// BpFET on
 //zz	djnz	ACC, $		// Allow some time after pfet is turned on
 // Turn on nfet
 AnFET_on();			// AnFET on
 A = Beep_Strength
 //zz	djnz	ACC, $
 // Turn off nfet
 AnFET_off();			// AnFET off
 A = 150;		// 25µs off
 //zz	djnz	ACC, $
 //zz	djnz	Temp2, beep_onoff
 // Copy variable
 A = Temp3;
 Temp1 = A;
 beep_off:		// Fets off loop
 //zz	djnz	ACC, $
 //zz	djnz	Temp1,	beep_off
 //zz	djnz	Temp4,	beep
 BpFET_off();			// BpFET off
 Current_Pwm_Limited =Temp5;	// Restore value
 */
}

void beep_f1(void) { // Entry point 1, load beeper frequency 1 settings
	Temp3 = 20; // Off wait loop length
	Temp4 = 120; // Number of beep pulses
	beep();
}

void beep_f2(void) { // Entry point 2, load beeper frequency 2 settings
	Temp3 = 16;
	Temp4 = 140;
	beep();
}

void beep_f3(void) { // Entry point 3, load beeper frequency 3 settings
	Temp3 = 13;
	Temp4 = 180;
	beep();
}

void beep_f4(void) { // Entry point 4, load beeper frequency 4 settings
	Temp3 = 11;
	Temp4 = 200;
	beep();
}

//___________________________________________________________________________
//
// Division 16bit unsigned by 16bit unsigned
//
// Dividend shall be in Temp2/Temp1, divisor in Temp4/Temp3
// Result will be in Temp2/Temp1
//
//___________________________________________________________________________
void div_u16_by_u16(void) {
	/*
	 C=0;
	 Temp5, #0
	 Temp6, #0
	 B, #0
	 div_u16_by_u16_div1:
	 inc	B      			// Increment counter for each left shift
	 A = Temp3   		// Shift left the divisor
	 rlc	A
	 Temp3  = A
	 A = Temp4
	 rlc	A
	 Temp4, A
	 jnc	div_u16_by_u16_div1	// Repeat until carry flag is set from high-byte
	 div_u16_by_u16_div2:
	 A = Temp4   		// Shift right the divisor
	 A/=2;
	 Temp4, A
	 A = Temp3
	 A/=2;
	 Temp3  = A
	 C=0;
	 A = Temp2  		// Make a safe copy of the dividend
	 Temp8, A
	 A = Temp1
	 Temp7, A
	 A = Temp1   		// Move low-byte of dividend into accumulator
	 subb	A, Temp3  		// Dividend - shifted divisor = result bit (no factor, only 0 or 1)
	 Temp1 = A   		// Save updated dividend
	 A = Temp2   		// Move high-byte of dividend into accumulator
	 subb	A, Temp4  		// Subtract high-byte of divisor (all together 16-bit substraction)
	 Temp2 = A   		// Save updated high-byte back in high-byte of divisor
	 jnc	div_u16_by_u16_div3	// If carry flag is NOT set, result is 1
	 A = Temp8  		// Otherwise result is 0, save copy of divisor to undo subtraction
	 Temp2 = A
	 A = Temp7
	 Temp1 = A
	 div_u16_by_u16_div3:
	 cpl	C      			// Invert carry, so it can be directly copied into result
	 A = Temp5
	 rlc	A      			// Shift carry flag into temporary result
	 Temp5, A
	 A = Temp6
	 rlc	A
	 Temp6,A
	 djnz	B, div_u16_by_u16_div2 	//Now count backwards and repeat until "B" is zero
	 A = Temp6  		// Move result to Temp2/Temp1
	 Temp2 = A
	 A = Temp5
	 Temp1 = A
	 */
}

//___________________________________________________________________________
//
// Multiplication 16bit signed by 8bit unsigned
//
// Multiplicand shall be in Temp2/Temp1, multiplicator in Temp3
// Result will be in Temp2/Temp1. Result will divided by 16
//
//___________________________________________________________________________
void mult_s16_by_u8_div_16(void) {
	/*

	 A = Temp1		// Read input to math registers
	 B, Temp2
	 Bit_Access, Temp3
	 setb	PSW.4		// Select register bank 2 for math routines
	 Temp1 = A		// Store in math registers
	 Temp2 = B
	 Temp4, #0		// Set sign in Temp4 and test sign
	 jnb	B.7, mult_s16_by_u8_positive

	 Temp4, #0xff
	 cpl	A
	 add	A, #1
	 Temp1 = A
	 A = Temp2
	 cpl	A
	 addc	A, #0
	 Temp2 = A
	 mult_s16_by_u8_positive:
	 A = Temp1		// Multiply LSB with multiplicator
	 B, Bit_Access
	 mul	AB
	 Temp6, B		// Place MSB in Temp6
	 Temp1 = A		// Place LSB in Temp1 (result)
	 A = Temp2		// Multiply MSB with multiplicator
	 B, Bit_Access
	 mul	AB
	 Temp8, B		// Place in Temp8/7
	 Temp7, A
	 A = Temp6		// Add up
	 add	A, Temp7
	 Temp2 = A
	 A = #0
	 addc	A, Temp8
	 Temp3  = A
	 Temp5, #4		// Set number of divisions
	 mult_s16_by_u8_div_loop:
	 C=0;			// Rotate right
	 A = Temp3
	 A/=2;
	 Temp3  = A
	 A = Temp2
	 A/=2;
	 Temp2 = A
	 A = Temp1
	 A/=2;
	 Temp1 = A
	 djnz	Temp5, mult_s16_by_u8_div_loop

	 B, Temp4		// Test sign
	 jnb	B.7, mult_s16_by_u8_exit

	 A = Temp1
	 cpl	A
	 add	A, #1
	 Temp1 = A
	 A = Temp2
	 cpl	A
	 addc	A, #0
	 Temp2 = A

	 mult_s16_by_u8_exit:
	 A = Temp1		// Store output
	 B, Temp2
	 clr	PSW.4		// Select normal register bank
	 Temp1 = A
	 Temp2 = B
	 */
}

//___________________________________________________________________________
//
// Calculate governor routines
//
// No assumptions
//
// Governs headspeed based upon the Comm_Period4x variable and pwm
// The governor task is split into several routines in order to distribute processing time
//
//___________________________________________________________________________
// First governor routine - calculate governor target

void calc_governor_int_error(void) {

} // calc_governor_int_error

#if (MODE==MAIN_MODE)	// Main
void calc_governor_target(void) {

	Temp1 = P.Gov_Mode; // Governor mode?
	cjne @Temp1, 4, governor_speed_check // Yes
	jmp calc_governor_target_exit // No

	governor_speed_check:
	// Stop governor for stop RC pulse

	if (New_Rcp < (RCP_MAX/10)) { // Yes - deactivate

		if (!(F.STARTUP_PHASE || F.INITIAL_RUN_PHASE)) {// Deactivate if any startup phase set

			// Skip speed check if governor is already active
			A = Gov_Active;
			jnz governor_target_calc

			// Check speed (do not run governor for low speeds)
			Temp1 =0x05; // Default high range activation limit value (~62500 eRPM)
			Temp2 = P.Gov_Range;
			A = @Temp2; // Check if high range (Temp2 has #P.Gov_Range)
			A--;
			jz governor_act_lim_set // If high range - branch

			Temp1 =0x0A; // Middle range activation limit value (~31250 eRPM)
			A--;
			jz governor_act_lim_set // If middle range - branch

			Temp1 = 0x12; // Low range activation limit value (~17400 eRPM)
			governor_act_lim_set:
			C=0;
			A = Comm_Period4x;
			A-= Temp1;
			jc governor_activate // If speed above min limit  - run governor
		}
	}

	governor_deactivate:
	A = Gov_Active;
	jz governor_first_deactivate_done// This code is executed continuously. Only execute the code below the first time

	Pwm_Limit_Spoolup = Pwm_Spoolup_Beg;
	Spoolup_Limit_Cnt=255;
	Spoolup_Limit_Skip=1;

	governor_first_deactivate_done:
	Current_Pwm, Requested_Pwm // Set current pwm to requested

	Gov_Integral= Gov_Integral_X= 0;
	Gov_Active= false;
	jmp calc_governor_target_exit

	governor_activate:
	Gov_Active=1;

	governor_target_calc:
	// Governor calculations
	Temp2 = P.Gov_Range;
	A = Temp2; // Check high, middle or low range
	A--;
	jnz calc_governor_target_middle

	A = Governor_Req_Pwm // Load governor requested pwm
	cpl A; // Calculate 255-pwm (invert pwm)
	// Calculate comm period target (1 + 2*((255-Requested_Pwm)/256) - 0.25)
	rlc A; // Msb to carry
	rlc A; // To bit0
	Temp2 = A; // Now 1 lsb is valid for H
	rrc A
	Temp1 = A; // Now 7 msbs are valid for L
	A = Temp2
	anl A,0x01; // Calculate H byte
	A++; // Add 1
	Temp2 = A;
	A = Temp1;
	anl A,0x0FE; // Calculate L byte
	jmp calc_governor_subtract_025

	calc_governor_target_middle:
	A = Temp2 // Check middle or low range (Temp2 has #P.Gov_Range)
	dec A
	dec A
	jnz calc_governor_target_low

	A = Governor_Req_Pwm // Load governor requested pwm
	cpl A // Calculate 255-pwm (invert pwm)
	// Calculate comm period target (1 + 4*((255-Requested_Pwm)/256))
	rlc A // Msb to carry
	rlc A // To bit0
	rlc A // To bit1
	Temp2 = A // Now 2 lsbs are valid for H
	rrc A
	Temp1 = A // Now 6 msbs are valid for L
	A = Temp2
	anl A, 0x03; // Calculate H byte
	inc A // Add 1
	Temp2 = A
	A = Temp1
	anl A,0xFC // Calculate L byte
	jmp calc_governor_store_target

	calc_governor_target_low:
	A = Governor_Req_Pwm // Load governor requested pwm
	cpl A // Calculate 255-pwm (invert pwm)
	// Calculate comm period target (2 + 8*((255-Requested_Pwm)/256) - 0.25)
	rlc A // Msb to carry
	rlc A // To bit0
	rlc A // To bit1
	rlc A // To bit2
	Temp2 = A // Now 3 lsbs are valid for H
	rrc A
	Temp1 = A // Now 5 msbs are valid for L
	A = Temp2
	anl A, 0x07; // Calculate H byte
	inc A // Add 1
	inc A // Add 1 more
	Temp2 = A;
	A = Temp1;
	anl A, 0xF8; // Calculate L byte
	calc_governor_subtract_025:
	C=0;
	subb A, 0x40; // Subtract 0.25
	Temp1 = A;
	A = Temp2;
	A-=0;

	Temp2 = A
	calc_governor_store_target:
	// Store governor target
	Gov_Target= Temp1;
	Gov_Target_H, Temp2;
	calc_governor_target_exit:
}

#elif (MODE==TAIL_MODE)	// Tail
void calc_governor_target() {}

#elif (MODE==MULTI_MODE)	// Multi
void governor_deactivate(void) {

	Current_Pwm = Requested_Pwm; // Set current pwm to requested

	Gov_Target = Gov_Integral = Gov_Integral_X = 0;
	Gov_Active = false;
	//zzcalc_governor_target_exit();

} // governor_deactivate

void governor_activate(void) {

	Gov_Active = P.Gov_Mode != 0;

	Governor_Req_Pwm = Requested_Pwm;
	Comm_Period4x = (51000L / Requested_Pwm) * 2;

} // governor_activate

void calc_governor_target(void) {

	if (P.Gov_Mode)
		governor_activate();
	else {
		if (New_Rcp < RCP_STOP) // Is pulse below stop value?
			governor_deactivate(); // Yes - deactivate
	}
} // calc_governor_target


#endif

// Second governor routine - calculate governor proportional error
void calc_governor_prop_error(void) {

	// Exit if governor is inactive
	if (Gov_Active) {

#if ((MODE==MAIN_MODE)|| (MODE==TAIL_MODE))	// Main or tail
		Gov_Proportional = (Comm_Period4x>>1) - Gov_Target;
#elif (MODE==MULTI_MODE)	// Multi
		Gov_Proportional = Governor_Req_Pwm - Gov_Target;
#endif

		Gov_Integral_X += Gov_Proportional;
		Gov_Integral_X = Limit1(Gov_Integral_X, 127); // 0x00f0??

		Current_Pwm = Limit1(Current_Pwm, Pwm_Limit);

	}

} // calc_governor_prop_error

// Fourth governor routine - calculate governor proportional correction
void calc_governor_prop_correction(void) {

	if (Gov_Active) {
		Gov_Proportional = (P.Gov_P_Gain * Gov_Proportional) / 16;
		Gov_Proportional = Limit1(Gov_Proportional, 127);
	}
}

// Fifth governor routine - calculate governor integral correction
void calc_governor_int_correction(void) {

	if (Gov_Active) {
		Gov_Integral = (P.Gov_I_Gain * Gov_Integral) / 16;
		Gov_Integral = Limit1(Gov_Integral, 127);
	}
} // calc_governor_int_correction

//___________________________________________________________________________
//
// Set pwm limit low rpm
//
// No assumptions
// Sets power limit for low rpms and disables demag for low rpms
//___________________________________________________________________________

void set_pwm_limit_low_rpm(void) {
	/*

	 // Set pwm limit and demag disable for low rpms
	 Temp1 = #0xff					// Default full power
	 F.DEMAG_ENABLED = false;			// Default disabled
	 if (F.STARTUP_PHASE || F.INITIAL_RUN_PHASE) set_pwm_limit_low_rpm_exit;		// Exit if any startup phase set

	 F.DEMAG_ENABLED	= true;		// Enable demag
	 C=0;
	 A = Comm_Period4x_H
	 subb	A, #0Ah						// ~31250 eRPM
	 jc	set_pwm_demag_done				// If speed above - branch

	 C=0;
	 A = Current_Pwm_Limited
	 subb	A, #40h						// Do not disable if pwm above 25%
	 jnc	set_pwm_demag_done

	 F.DEMAG_ENABLED = false;			// Disable demag

	 set_pwm_demag_done:
	 A = Comm_Period4x_H
	 jz	set_pwm_limit_low_rpm_exit		// Avoid divide by zero

	 A = #255						// Divide 255 by Comm_Period4x_H
	 B, Comm_Period4x_H
	 div	AB
	 B, Low_Rpm_Pwr_Slope			// Multiply by slope
	 mul	AB
	 Temp1 = A						// Set new limit
	 xch	A, B
	 jz	($+4)						// Limit to max

	 Temp1 = #0xff

	 C=0;
	 A = Temp1						// Limit to min
	 subb	A, Pwm_Spoolup_Beg
	 jnc	set_pwm_limit_low_rpm_exit

	 Temp1 = Pwm_Spoolup_Beg

	 set_pwm_limit_low_rpm_exit:
	 Pwm_Limit_Low_Rpm, Temp1
	 */
}

//___________________________________________________________________________
//
// Measure lipo cells
//
// No assumptions
// Measure voltage and calculate lipo cells
//___________________________________________________________________________

void measure_lipo_cells(void) {

#if (MODE!=TAIL_MODE)	// Tail
#endif

} // measure_lipo_cells

//___________________________________________________________________________
//
// Start ADC conversion
//
// No assumptions
// Start conversion used for measuring power supply voltage
//___________________________________________________________________________
void start_adc_conversion(void) {

	// Start adc dma burst then stop to prevent traffic interference

} // start_adc_conversion

//___________________________________________________________________________
//
// Check temperature, power supply voltage and limit power
//
// No assumptions
// Used to limit main motor power in order to maintain the required voltage
//___________________________________________________________________________

void check_temp_voltage_and_limit_power(void) {

	//is routine reduces pwmLimit as battery sags or temperature rises to high

} // check_temp_voltage_and_limit_power

void check_voltage_start(void) {

	//cck initial voltage and set pwmLimit accordingly


} // check_voltage_start

//___________________________________________________________________________
//
// Set startup PWM routine
//
// Either the SETTLE_PHASE or the STEPPER_PHASE flag must be set
// Used for pwm control during startup
//___________________________________________________________________________
void set_startup_pwm(void) {

	Requested_Pwm = P.Startup_Pwr * PWM_START;
	Requested_Pwm = Limit(Requested_Pwm, 0, Pwm_Limit);

	Current_Pwm = Current_Pwm_Limited = Pwm_Spoolup_Beg = Requested_Pwm;

} // set_startup_pwm

//___________________________________________________________________________
//
// Initialize all timings routine
//
// No assumptions
// Part of initialization before motor start
//___________________________________________________________________________

void initialize_all_timings(void) {
	Comm_Period4x = 0x7F00;// Set commutation period registers
}

//___________________________________________________________________________
//
// Calculate next commutation timing routine
//
// No assumptions

// Called immediately after each commutation
// Also sets up timer 3 to wait advance timing
// Two entry points are used
//___________________________________________________________________________

void calc_next_comm_timing(void) { // Entry point for run phase
/*

 // Read commutation time
 TMR2CN = 0x20;		// Timer2 disabled
 Temp1 = TMR2L;		// Load timer value
 Temp2 = TMR2H;
 TMR2CN = 0x24;		// Timer2 enabled
 #if (MCU_50MHZ==1)
 C=0;
 A = Temp2
 A/=2;
 Temp2 = A
 A = Temp1
 A/=2;
 Temp1 = A
 #endif
 // Calculate this commutation time
 Temp3  = Prev_Comm_L
 Temp4, Prev_Comm_H
 Prev_Comm_L, Temp1		// Store timestamp as previous commutation
 Prev_Comm_H, Temp2
 C=0;
 A = Temp1
 subb	A, Temp3				// Calculate the new commutation time
 Temp1 = A
 A = Temp2
 subb	A, Temp4
 #if (MCU_50MHZ==1
 anl	A, #7Fh
 #endif
 Temp2 = A
 // Calculate new commutation time
 Temp3  = Comm_Period4x;	// Comm_Period4x(-l-h) holds the time of 4 commutations

 Temp5= Comm_Period4x;	// Copy variables

 Temp7=4;				// Divide Comm_Period4x 4 times as default
 Temp8=2;				// Divide new commutation time 2 times as default
 C=0;
 A = Temp4;
 if (Temp4 < 0x0400)

 Temp7--;				// Reduce averaging time constant for low speeds
 Temp8--;

 } else if (Temp4 < 0x0800) {

 dec	Temp7				// Reduce averaging time constant more for even lower speeds
 dec	Temp8
 }

 calc_next_comm_avg_period_div:
 C=0;
 A = Temp6
 A/=2;					// Divide by 2
 Temp6, A
 A = Temp5
 A/=2;
 Temp5, A
 djnz	Temp7, calc_next_comm_avg_period_div

 C=0;
 A = Temp3
 subb	A, Temp5				// Subtract a fraction
 Temp3  = A
 A = Temp4
 subb	A, Temp6
 Temp4, A
 A = Temp8				// Divide new time
 jz	calc_next_comm_new_period_div_done

 calc_next_comm_new_period_div:
 C=0;
 A = Temp2
 A/=2;					// Divide by 2
 Temp2 = A;
 A = Temp1
 A/=2;
 Temp1 = A;
 djnz	Temp8, calc_next_comm_new_period_div

 calc_next_comm_new_period_div_done:
 A = Temp3;
 A+=Temp1;				// Add the divided new time
 Temp3  = A;
 A = Temp4
 addc	A, Temp2
 Temp4, A;
 Comm_Period4x_L, Temp3	// Store Comm_Period4x_X
 Comm_Period4x_H, Temp4
 jc	calc_next_comm_slow		// If period larger than 0xffff - go to slow case

 */
}

void calc_next_comm_slow(void) {
	Comm_Period4x = 0xffff; // Set commutation period registers to very slow timing (0xffff)
}

//___________________________________________________________________________
//
// Wait advance timing routine
//
// No assumptions
// Waits for the advance timing to elapse and sets up the next zero cross wait
//___________________________________________________________________________

void wait_advance_timing(void) {
	/*

	 jnb	F.T3_PENDING, ($+5)
	 ajmp	wait_advance_timing

	 // Setup next wait time
	 Next_Wt_L, Wt_ZC_Timeout_L
	 Next_Wt_H, Wt_ZC_Timeout_H
	 setb	F.T3_PENDING
	 orl	EIE1, #80h	// Enable timer3 interrupts
	 */
}

//___________________________________________________________________________
//
// Calculate new wait times routine
//
// No assumptions
//___________________________________________________________________________

void calc_new_wait_times(void) {
	/*

	 // Load commutation timing
	 Temp1 = #P.Comm_Timing	// Load timing setting
	 A = @Temp1
	 Temp8, A				// Store in Temp8
	 C=0;
	 A = Demag_Detected_Metric	// Check demag metric
	 subb	A, #130
	 jc	($+3)

	 inc	Temp8				// Increase timing

	 C=0;
	 A = Demag_Detected_Metric
	 subb	A, #160
	 jc	($+3)

	 inc	Temp8				// Increase timing again

	 C=0;
	 A = Temp8				// Limit timing to max
	 subb	A, #6
	 jc	($+4)

	 Temp8, #5				// Set timing to max

	 Temp7, #(COMM_TIME_RED SHL 1)
	 dec	Temp7
	 jnb	F.PGM_PWMOFF_DAMPED, ($+4)	// More reduction for damped

	 inc	Temp7				// Increase more

	 C=0;
	 A = Comm_Period4x_H		// More reduction for higher rpms
	 subb	A, #3				// 104k eRPM
	 jnc	calc_new_wait_per_low

	 inc	Temp7				// Increase
	 inc	Temp7

	 jnb	F.PGM_PWMOFF_DAMPED, calc_new_wait_per_low	// More reduction for damped

	 inc	Temp7				// Increase more

	 calc_new_wait_per_low:
	 C=0;
	 A = Comm_Period4x_H		// More reduction for higher rpms
	 subb	A, #2				// 156k eRPM
	 jnc	calc_new_wait_per_high

	 inc	Temp7				// Increase more
	 inc	Temp7

	 jnb	F.PGM_PWMOFF_DAMPED, calc_new_wait_per_high	// More reduction for damped

	 inc	Temp7				// Increase more

	 calc_new_wait_per_high:
	 // Load current commutation timing
	 Temp2 = Comm_Period4x_H	// Load Comm_Period4x
	 Temp1 = Comm_Period4x_L
	 Temp3  = #4				// Divide 4 times
	 divide_wait_times:
	 C=0;
	 A = Temp2
	 A/=2;					// Divide by 2
	 Temp2 = A
	 A = Temp1
	 A/=2;
	 Temp1 = A
	 djnz	Temp3, divide_wait_times

	 C=0;
	 A = Temp1
	 subb	A, Temp7
	 Temp1 = A
	 A = Temp2
	 subb	A, #0
	 Temp2 = A
	 jc	load_min_time			// Check that result is still positive

	 C=0;
	 A = Temp1
	 subb	A, #(COMM_TIME_MIN SHL 1)
	 A = Temp2
	 subb	A, #0
	 jnc	adjust_timing			// Check that result is still above minumum

	 load_min_time:
	 Temp1 = #(COMM_TIME_MIN SHL 1)
	 A=0;
	 Temp2 = A

	 adjust_timing:
	 A = Temp2				// Copy values
	 Temp4, A
	 A = Temp1
	 Temp3  = A
	 C=0;
	 A = Temp2
	 A/=2;					// Divide by 2
	 Temp6, A
	 A = Temp1
	 A/=2;
	 Temp5, A
	 Wt_Zc_Timeout_L, Temp1	// Set 15deg time for zero cross scan timeout
	 Wt_Zc_Timeout_H, Temp2
	 C=0;
	 A = Temp8				// (Temp8 has P.Comm_Timing)
	 subb	A, #3				// Is timing normal?
	 jz	store_times_decrease	// Yes - branch

	 A = Temp8
	 jb	ACC.0, adjust_timing_two_steps	// If an odd number - branch

	 A = Temp1				// Add 7.5deg and store in Temp1/2
	 add	A, Temp5
	 Temp1 = A
	 A = Temp2
	 addc	A, Temp6
	 Temp2 = A
	 A = Temp5				// Store 7.5deg in Temp3/4
	 Temp3  = A
	 A = Temp6
	 Temp4, A
	 jmp	store_times_up_or_down

	 adjust_timing_two_steps:
	 A = Temp1				// Add 15deg and store in Temp1/2
	 add	A, Temp1
	 Temp1 = A
	 A = Temp2
	 addc	A, Temp2
	 Temp2 = A
	 C=0;
	 A = Temp1
	 subb	A, #(COMM_TIME_MIN SHL 1)
	 Temp1 = A
	 A = Temp2
	 subb	A, #0
	 Temp2 = A
	 Temp3  = #(COMM_TIME_MIN SHL 1)	// Store minimum time in Temp3/4
	 A=0;
	 Temp4, A

	 store_times_up_or_down:
	 C=0;
	 A = Temp8
	 subb	A, #3				// Is timing higher than normal?
	 jc	store_times_decrease	// No - branch

	 store_times_increase:
	 Wt_Comm_L, Temp3		// Now commutation time (~60deg) divided by 4 (~15deg nominal)
	 Wt_Comm_H, Temp4
	 Wt_Advance_L, Temp1		// New commutation advance time (~15deg nominal)
	 Wt_Advance_H, Temp2
	 Wt_Zc_Scan_L, Temp5		// Use this value for zero cross scan delay (7.5deg)
	 Wt_Zc_Scan_H, Temp6
	 ret

	 store_times_decrease:
	 Wt_Comm_L, Temp1		// Now commutation time (~60deg) divided by 4 (~15deg nominal)
	 Wt_Comm_H, Temp2
	 Wt_Advance_L, Temp3		// New commutation advance time (~15deg nominal)
	 Wt_Advance_H, Temp4
	 Wt_Zc_Scan_L, Temp5		// Use this value for zero cross scan delay (7.5deg)
	 Wt_Zc_Scan_H, Temp6
	 */
}

//___________________________________________________________________________
//
// Wait before zero cross scan routine
//
// No assumptions
//
// Waits for the zero cross scan wait time to elapse
// Also sets up timer 3 for the zero cross scan timeout time
//
//___________________________________________________________________________
void wait_before_zc_scan(void) {
	/*

	 jnb	F.T3_PENDING, ($+5)
	 ajmp	wait_before_zc_scan

	 setb	F.T3_PENDING
	 orl	EIE1, #80h			// Enable timer3 interrupts
	 A = Flags1
	 anl	A, #((1 SHL STARTUP_PHASE)+(1 SHL INITIAL_RUN_PHASE))
	 jz	wait_before_zc_exit

	 Temp1 = Comm_Period4x_L	// Set long timeout when starting
	 Temp2 = Comm_Period4x_H
	 // Break deadlock cyclic patterns during startup
	 A = Startup_Ok_Cnt
	 subb	A, #8
	 jnc	wait_before_zc_random_done

	 A = Temp1
	 jb	ACC.0, wait_before_zc_random_done	// Use LSB as a random number

	 Temp2 = Wt_Zc_Timeout_H

	 wait_before_zc_random_done:
	 #if (MCU_50MHZ==1
	 C=0;
	 A = Temp1
	 rlc	A
	 Temp1 = A
	 A = Temp2
	 rlc	A
	 Temp2 = A
	 #endif
	 TMR3CN, #00h			// Timer3 disabled
	 C=0;
	 A=0;
	 subb	A, Temp1				// Set timeout
	 TMR3L, A
	 A=0;
	 subb	A, Temp2
	 TMR3H, A
	 TMR3CN, #04h			// Timer3 enabled
	 setb	F.T3_PENDING
	 anl	TMR3CN, #07Fh			// Clear interrupt flag
	 orl	EIE1, #80h			// Enable timer3 interrupts

	 wait_before_zc_exit:
	 */
}

//___________________________________________________________________________
//
// Wait for comparator to go low/high routines
//
// No assumptions
//
// Waits for the zero cross scan wait time to elapse
// Then scans for comparator going low/high
//
//___________________________________________________________________________
void wait_for_comp_out_low(void) {

	F.DEMAG_DETECTED = true; // Set demag detected flag as default
	Comparator_Read_Cnt = 0;

	//zzBit_Access 0x00h			// Desired comparator output
	//zzjmp wait_for_comp_out_start

} // wait_for_comp_out_low

void wait_for_comp_out_high(void) {

	F.DEMAG_DETECTED = true; // Set demag detected flag as default
	Comparator_Read_Cnt = 0;
	//zzBit_Access,#40h			// Desired comparator output

} // wait_for_comp_out_high

void wait_for_comp_out_start(void) {

	if (F.STARTUP_PHASE || F.INITIAL_RUN_PHASE) {
		F.DEMAG_DETECTED = false;
		//zzz EA=1;						// Enable interrupts
		//zzz if (F.T3_PENDING, wait_for_comp_out_not_timed_out// Has zero cross scan timeout elapsed?
		while (Comparator_Read_Cnt == 0) {
		};
	}
} // wait_for_comp_out_start


void comp_wait_on_comp_able_not_timed_out(void) {
	/*
	 //zzEA = 1							// Enable interrupts
	 nop								// Allocate only just enough time to capture interrupt
	 nop
	 EA=0;							// Disable interrupts
	 C=0;
	 A = Comm_Period4x_H				// Reduce required distance to pwm transition for higher speeds
	 Temp4, A
	 subb	A, #0Fh
	 jc	($+4)

	 Temp4, #0Fh

	 A = Temp4
	 inc	A
	 jnb	F.PGM_PWM_HIGH_FREQ, ($+4)	// More delay for high pwm frequency

	 rl	A

	 jb	F.PWM_ON, ($+4)			// More delay for pwm off

	 rl	A

	 Temp2 = A
	 jnb	F.STARTUP_PHASE, ($+5)		// Set a long delay from pwm on/off events during startup

	 Temp2 = #130

	 #if (MCU_50MHZ==0
	 A = TL1
	 ELSE
	 A = TH1
	 A/=2;
	 A = TL1
	 A/=2;
	 #endif
	 C=0;
	 subb	A, Temp2
	 jc	comp_wait_on_comp_able		// Re-evaluate pwm cycle

	 inc	Comparator_Read_Cnt			// Increment comparator read count
	 A = Temp3
	 Temp4, A
	 read_comp_loop:
	 Read_Comp_Out					// Read comparator output
	 anl	A, #40h
	 cjne	A, Bit_Access, comp_read_wrong
	 djnz	Temp4, read_comp_loop		// Decrement readings count
	 ajmp	comp_read_ok

	 comp_read_wrong:
	 jb	F.DEMAG_DETECTED, ($+5)
	 ajmp	wait_for_comp_out_start		// If comparator output is not correct, and timeout already extended - go back and restart

	 clr	F.DEMAG_DETECTED		// Clear demag detected flag
	 TMR3CN, #00h				// Timer3 disabled
	 Temp7, Comm_Period4x_L		// Set timeout to comm period 4x value
	 Temp8, Comm_Period4x_H
	 #if (MCU_50MHZ==1
	 C=0;
	 A = Temp7
	 rlc	A
	 Temp7, A
	 A = Temp8
	 rlc	A
	 Temp8, A
	 #endif
	 C=0;
	 A=0;
	 A-=Temp7;
	 TMR3L, A
	 A=0;

	 subb	A, Temp8
	 TMR3H, A
	 TMR3CN, #04h				// Timer3 enabled
	 setb	F.T3_PENDING
	 anl	TMR3CN, #07Fh				// Clear interrupt flag in case there are pending interrupts
	 orl	EIE1, #80h				// Enable timer3 interrupts
	 ajmp	wait_for_comp_out_start		// If comparator output is not correct - go back and restart

	 comp_read_ok:
	 jnb	F.DEMAG_DETECTED, ($+5)	// Do not accept correct comparator output if it is demag
	 ajmp	wait_for_comp_out_start

	 djnz	Temp1, comp_wait_on_comp_able	// Decrement readings counter - repeat comparator reading if not zero

	 //zzEA = 1						// Enable interrupts
	 */
}

void wait_for_comp_out_not_timed_out(void) {

	// Set number of comparator readings
	Temp1 = 1; // Number of OK readings required
	Temp3 = 2; // Number of fast consecutive readings

	// Set number of readings higher for lower speeds
	if (Comm_Period4x > 0x0500) {
		Temp1 = 2;
		if (Comm_Period4x > 0x0a00) {
			Temp1 = 3;
			if (Comm_Period4x > 0x0f00) {
				Temp3 = 3;
			}
		}
	} else {
		Temp1 = 30;
		Temp3 = 1;
	}

	while (F.T3_PENDING && (Comparator_Read_Cnt == 0))
		comp_wait_on_comp_able_not_timed_out(); // Has zero cross scan timeout elapsed?

	//zzEA=1;							// Enable interrupts

} // wait_for_comp_out_not_timed_out

//___________________________________________________________________________
//
// Evaluate comparator integrity
//
// No assumptions
//
// Checks comparator signal behaviour versus expected behaviour
//
//___________________________________________________________________________
void evaluate_comparator_integrity(void) {
	/*

	 jnb	F.STARTUP_PHASE, eval_comp_check_timeout

	 inc	Startup_Ok_Cnt					// Increment ok counter
	 jb	F.T3_PENDING, eval_comp_exit

	 Startup_Ok_Cnt, #0				// Reset ok counter
	 jmp	eval_comp_exit

	 eval_comp_check_timeout:
	 jb	F.T3_PENDING, eval_comp_exit	// Has timeout elapsed?
	 jb	F.DEMAG_DETECTED, eval_comp_exit	// Do not exit run mode if it is a demag situation
	 jb	F.DIR_CHANGE_BRAKE, eval_comp_exit	// Do not exit run mode if it is a direction change brake
	 dec	SP							// Routine exit without "ret" command
	 dec	SP
	 ljmp	run_to_wait_for_power_on			// Yes - exit run mode

	 eval_comp_exit:
	 */
}

//___________________________________________________________________________
//
// Setup commutation timing routine
//
// No assumptions
//
// Sets up and starts wait from commutation to zero cross
//
//___________________________________________________________________________
void setup_comm_wait(void) {

	Delay1uS(Wt_Comm);
	/*

	 Temp1 = Wt_Comm	// Set wait commutation value

	 #if (MCU_50MHZ==1
	 C=0;
	 A = Temp1
	 rlc	A
	 Temp1 = A
	 A = Temp2
	 rlc	A
	 Temp2 = A
	 #endif
	 TMR3CN, #00h		// Timer3 disabled
	 anl	TMR3CN, #07Fh		// Clear interrupt flag
	 C=0;
	 A=0;
	 subb	A, Temp1
	 TMR3L, A
	 A=0;
	 subb	A, Temp2
	 TMR3H, A
	 TMR3CN, #04h		// Timer3 enabled
	 // Setup next wait time
	 Next_Wt= Wt_Advance;

	 F.T3_PENDING = true;
	 orl	EIE1, #80h		// Enable timer3 interrupts
	 */
}

void Clear_RPM_Out(void) {

} // Clear_RPM_Out

void Set_RPM_Out(void) {

} // Set_RPM_Out

//___________________________________________________________________________
//
// Wait for commutation routine
//
// No assumptions
// Waits from zero cross to commutation
//___________________________________________________________________________
void wait_for_comm(void) {
	// Update demag metric
	/*
	 Temp1 = #0
	 jnb	F.DEMAG_ENABLED, ($+8)// If demag disabled - branch
	 jnb	F.DEMAG_DETECTED, ($+5)

	 Temp1 = #1

	 A = Demag_Detected_Metric	// Sliding average of 8, 256 when demag and 0 when not. Limited to minimum 120
	 B, #7
	 mul	AB					// Multiply by 7
	 Temp2 = A
	 A = B					// Add new value for current demag status
	 add	A, Temp1
	 B, A
	 A = Temp2
	 C, B.0				// Divide by 8
	 A/=2;
	 C, B.1
	 A/=2;
	 C, B.2
	 A/=2;
	 Demag_Detected_Metric = A;
	 C=0;
	 subb	A, #120				// Limit to minimum 120
	 jnc	($+5)

	 Demag_Detected_Metric=120;

	 C=0;
	 A = Demag_Detected_Metric;	// Check demag metric
	 subb	A, Demag_Pwr_Off_Thresh
	 jc	wait_for_comm_wait		// Cut power if many consecutive demags. This will help retain sync during hard accelerations

	 setb	F.DEMAG_CUT_POWER	// Set demag power cut flag
	 All_nFETs_off();
	 wait_for_comm_wait();
	 */
}

void wait_for_comm_wait(void) {

	while (F.T3_PENDING) {
	};

	Next_Wt = Wt_Zc_Scan; // Setup next wait time
	F.T3_PENDING = true;
	//zzorl	EIE1, 0x80;			// Enable timer3 interrupts

} // wait_for_comm_wait

//___________________________________________________________________________
//
// Commutation routines
//
// No assumptions
//
// Performs commutation switching
// Damped routines uses all pfets on when in pwm off to dampen the motor
//
//___________________________________________________________________________

void comm_exit(void) {

#if (MODE >= 1)	// Tail or multi
	int32 d;

	if (F.DIR_CHANGE_BRAKE) { // Is it a direction change?
		switch_power_off();
		FET_DELAY(NFETON_DELAY);
		FET_DELAY(NFETON_DELAY); // ??
		All_pFETs_on();
	}

#endif
	F.DEMAG_CUT_POWER = false; // Clear demag power cut flag
	//zzsetb EA // Enable all interrupts

} // comm_exit


void comm1comm2(void) {
	int32 d;

	Set_RPM_Out();
	//zz EA = 0;
	All_pFETs_off();
	if (!F.PGM_PWMOFF_DAMPED) {
		DPTR = pwm_cnfet_apBnFET_off;
		FET_DELAY(NFETON_DELAY);
	} else {
#if (HIGH_DRIVER_PRECHG_TIME != 0)	// Precharge high side gate driver
		if (Comm_Period4x > 8) {
			AnFET_on();
			FET_DELAY(HIGH_DRIVER_PRECHG_TIME);
			AnFET_off();
			FET_DELAY(PFETON_DELAY);
		}
#endif
	}

	ApFET_on();
	Set_Comp_Phase_B(); // Set comparator to phase B
	Comm_Phase = 2;

	comm_exit();
} // comm1comm2

void comm2comm3(void) {

	Clear_RPM_Out();
	//zz//zz EA = 0; // Disable all interrupts
	CnFET_off(); // Cn off
	if (!F.PGM_PWMOFF_DAMPED) {
		DPTR = pwm_bnfet_apBnFET_off;
		BpFET_off();
		CpFET_off();
		FET_DELAY(NFETON_DELAY);
	} else {
		DPTR = pwm_bBnFET_off;
	}

	if (!F.PWM_ON) // Is pwm on?
		BnFET_on(); // Yes - Bn on

	Set_Comp_Phase_C(); // Set comparator to phase C
	Comm_Phase = 3;

	comm_exit();
} // comm2comm3

void comm3comm4(void) {

	//zzEA=0;
	All_pFETs_off(); // All pfets off
	if (F.PGM_PWMOFF_DAMPED) {
		DPTR = pwm_bnfet_cpBnFET_off;
		FET_DELAY(NFETON_DELAY);
	} else {
#if (HIGH_DRIVER_PRECHG_TIME != 0)	// Precharge high side gate driver
		A = Comm_Period4x_H
		anl A,0x0F8h // Check if comm period is less than 8
		jz comm34_prech_done
		CnFET_on();
		A = HIGH_DRIVER_PRECHG_TIME
		djnz ACC, $
		CnFET_off();
		PFETON_DELAY();
		djnz ACC, $
#endif
	}

	CpFET_on();
	Set_Comp_Phase_A();
	Comm_Phase = 4;

	comm_exit();
} // comm3comm4

void comm4comm5(void) {

	//zzclr 	EA					// Disable all interrupts
	BnFET_off(); // Bn off
	if (F.PGM_PWMOFF_DAMPED) {
		DPTR = pwm_anfet_cpBnFET_off;
		ApFET_off();
		BpFET_off();
		FET_DELAY(NFETON_DELAY);
	} else {
		DPTR = pwm_aBnFET_off;
		if (F.PWM_ON)
			AnFET_on();
	}

	Set_Comp_Phase_B(); // Set comparator to phase B
	Comm_Phase = 5;

	comm_exit();
} // comm4comm5

void comm5comm6(void) {

	// clr 	EA					// Disable all interrupts
	All_pFETs_off(); // All pfets off
	if (F.PGM_PWMOFF_DAMPED) {
		DPTR = pwm_anfet_bpBnFET_off;
		FET_DELAY(NFETON_DELAY);
	} else {
#if (HIGH_DRIVER_PRECHG_TIME != 0)	// Precharge high side gate driver
		if (Comm_Period4x > 8) {
			BnFET_on();
			FET_DELAY(HIGH_DRIVER_PRECHG_TIME);
			BnFET_off();
			FET_DELAY(PFETON_DELAY);
		}
#endif
	}
	BpFET_on();
	Set_Comp_Phase_C();
	Comm_Phase = 6;

	comm_exit();
} // comm5comm6

void comm6comm1(void) {

	// clr 	EA					// Disable all interrupts
	AnFET_off(); // An off
	if (F.PGM_PWMOFF_DAMPED) {
		DPTR = pwm_cnfet_bpBnFET_off;
		ApFET_off();
		CpFET_off();
		FET_DELAY(NFETON_DELAY);
	} else {
		DPTR = pwm_cBnFET_off;
	}

	if (F.PWM_ON)
		CnFET_on();

	Set_Comp_Phase_A(); // Set comparator to phase A
	Comm_Phase = 1;

	comm_exit();
} // comm6comm1

//___________________________________________________________________________
//
// Set default parameters
//
// No assumptions
// Sets default programming parameters
//___________________________________________________________________________

void set_default_parameters(void) {

	P.FW_Main_Revision = EEPROM_FW_MAIN_REVISION; // EEPROM firmware main revision number
	P.FW_Sub_Revision = EEPROM_FW_SUB_REVISION; // EEPROM firmware sub revision number
	P.Layout_Revision = EEPROM_LAYOUT_REVISION; // EEPROM layout revision number

#if (MODE==MAIN_MODE)
	P.Gov_P_Gain = DEFAULT_PGM_MAIN_P_GAIN; // governor P gain
	P.Gov_I_Gain = DEFAULT_PGM_MAIN_I_GAIN // governor I gain
	P.Gov_Mode = DEFAULT_PGM_MAIN_GOVERNOR_MODE // governor mode
	P.Low_Voltage_Lim = DEFAULT_PGM_MAIN_LOW_VOLTAGE_LIM // low voltage limit
	P.Motor_Gain = 0xff
	P.Motor_Idle = 0xff
	P.Startup_Pwr = DEFAULT_PGM_MAIN_STARTUP_PWR // startup power
	P.Pwm_Freq = DEFAULT_PGM_MAIN_PWM_FREQ // pwm frequency
	P.Direction = DEFAULT_PGM_MAIN_DIRECTION // rotation direction
	P.Input_Pol = DEFAULT_PGM_MAIN_RCP_PWM_POL // input polarity
	P.Initialized = 0x5AA5; // EEPROM initialized signature
	P.Enable_TX_Program = DEFAULT_PGM_ENABLE_TX_PROGRAM; // EEPROM TX programming enable
	P.Main_Rearm_Start = DEFAULT_PGM_MAIN_REARM_START // EEPROM re-arming main enable
	P.Gov_Setup_Target = DEFAULT_PGM_MAIN_GOV_SETUP_TARGET // EEPROM main governor setup target
	P.Startup_Rpm = 0xff
	P.Startup_Accel = 0xff
	P.Volt_Comp = 0xff
	P.Comm_Timing = DEFAULT_PGM_MAIN_COMM_TIMING // commutation timing
	P.Damping_Force = 0xff
	P.Gov_Range = DEFAULT_PGM_MAIN_GOVERNOR_RANGE // governor range
	P.Startup_Method = 0xff
	P.Ppm_Min_Throttle = DEFAULT_PGM_PPM_MIN_THROTTLE // minimum throttle (final value is 4x+1000=1148)
	P.Ppm_Max_Throttle = DEFAULT_PGM_PPM_MAX_THROTTLE // minimum throttle (final value is 4x+1000=1832)
	P.Beep_Strength = DEFAULT_PGM_MAIN_BEEP_STRENGTH // beep strength
	P.Beacon_Strength = DEFAULT_PGM_MAIN_BEACON_STRENGTH // beacon strength
	P.Beacon_Delay = DEFAULT_PGM_MAIN_BEACON_DELAY // beacon delay
	P.Throttle_Rate = 0xff
	P.Demag_Comp = DEFAULT_PGM_MAIN_DEMAG_COMP // demag compensation
	P.BEC_Voltage_High = DEFAULT_PGM_BEC_VOLTAGE_HIGH // BEC voltage
	P.Ppm_Center_Throttle = 0xff // center throttle (final value is 4x+1000=1488)
	P.Main_Spoolup_Time = DEFAULT_PGM_MAIN_SPOOLUP_TIME // main spoolup time
	P.Temp_Prot_Enable = DEFAULT_PGM_ENABLE_TEMP_PROT // temperature protection enable

#elif (MODE==TAIL_MODE)
	P.Gov_P_Gain = 0xff;
	P.Gov_I_Gain = 0xff;
	P.Gov_Mode = 0xff;
	P.Low_Voltage_Lim = 0xff;
	P.Motor_Gain = DEFAULT_PGM_TAIL_GAIN; // tail gain
	P.Motor_Idle = DEFAULT_PGM_TAIL_IDLE_SPEED; // tail idle speed
	P.Startup_Pwr = DEFAULT_PGM_TAIL_STARTUP_PWR; // startup power
	P.Pwm_Freq = DEFAULT_PGM_TAIL_PWM_FREQ; // pwm frequency
	P.Direction = DEFAULT_PGM_TAIL_DIRECTION; // rotation direction
	P.Input_Pol = DEFAULT_PGM_TAIL_RCP_PWM_POL; // input polarity
	Initialized = 0xA55A; // EEPROM initialized signature low byte
	Enable_TX_Program = DEFAULT_PGM_ENABLE_TX_PROGRAM; // EEPROM TX programming enable
	_Main_Rearm_Start = 0xff;
	P.Gov_Setup_Target = 0xff;
	P.Startup_Rpm = 0xff;
	P.Startup_Accel = 0xff;
	P.Volt_Comp = 0xff;
	P.Comm_Timing = DEFAULT_PGM_TAIL_COMM_TIMING; // commutation timing
	P.Damping_Force = 0xff;
	P.Gov_Range = 0xff;
	P.Startup_Method = 0xff;
	P.Ppm_Min_Throttle = DEFAULT_PGM_PPM_MIN_THROTTLE; // minimum throttle (final value is 4x+1000=1148)
	P.Ppm_Max_Throttle = DEFAULT_PGM_PPM_MAX_THROTTLE; // minimum throttle (final value is 4x+1000=1832)
	P.Beep_Strength = DEFAULT_PGM_TAIL_BEEP_STRENGTH; // beep strength
	P.Beacon_Strength = DEFAULT_PGM_TAIL_BEACON_STRENGTH; // beacon strength
	P.Beacon_Delay = DEFAULT_PGM_TAIL_BEACON_DELAY; // beacon delay
	P.Throttle_Rate = 0xff;
	P.Demag_Comp = DEFAULT_PGM_TAIL_DEMAG_COMP; // demag compensation
	P.BEC_Voltage_High = DEFAULT_PGM_BEC_VOLTAGE_HIGH; // BEC voltage
	P.Ppm_Center_Throttle = DEFAULT_PGM_PPM_CENTER_THROTTLE; // center throttle (final value is 4x+1000=1488)
	P.Main_Spoolup_Time = 0xff;
	P.Temp_Prot_Enable = DEFAULT_PGM_ENABLE_TEMP_PROT; // temperature protection enable
#elif (MODE==MULTI_MODE)
	P.Gov_P_Gain = DEFAULT_PGM_MULTI_P_GAIN; // closed loop P gain
	P.Gov_I_Gain = DEFAULT_PGM_MULTI_I_GAIN; // closed loop I gain
	P.Gov_Mode = DEFAULT_PGM_MULTI_GOVERNOR_MODE; // closed loop mode
	P.Low_Voltage_Lim = DEFAULT_PGM_MULTI_LOW_VOLTAGE_LIM; // low voltage limit
	P.Motor_Gain = DEFAULT_PGM_MULTI_GAIN; // tail gain
	P.Motor_Idle = 0xff; // tail idle speed
	P.Startup_Pwr = DEFAULT_PGM_MULTI_STARTUP_PWR; // startup power
	P.Pwm_Freq = DEFAULT_PGM_MULTI_PWM_FREQ; // pwm frequency
	P.Direction = DEFAULT_PGM_MULTI_DIRECTION; // rotation direction
	P.Input_Pol = DEFAULT_PGM_MULTI_RCP_PWM_POL; // input polarity
	P.Initialized = 0xAA55; // EEPROM initialized signature low byte
	P.Enable_TX_Program = DEFAULT_PGM_ENABLE_TX_PROGRAM; // EEPROM TX programming enable
	P.Main_Rearm_Start = 0xff;
	P.Gov_Setup_Target = 0xff;
	P.Startup_Rpm = 0xff;
	P.Startup_Accel = 0xff;
	P.Volt_Comp = 0xff;
	P.Comm_Timing = DEFAULT_PGM_MULTI_COMM_TIMING; // commutation timing
	P.Damping_Force = 0xff;
	P.Gov_Range = 0xff;
	P.Startup_Method = 0xff;
	P.Ppm_Min_Throttle = DEFAULT_PGM_PPM_MIN_THROTTLE; // minimum throttle (final value is 4x+1000=1148)
	P.Ppm_Max_Throttle = DEFAULT_PGM_PPM_MAX_THROTTLE; // minimum throttle (final value is 4x+1000=1832)
	P.Beep_Strength = DEFAULT_PGM_MULTI_BEEP_STRENGTH; // beep strength
	P.Beacon_Strength = DEFAULT_PGM_MULTI_BEACON_STRENGTH; // beacon strength
	P.Beacon_Delay = DEFAULT_PGM_MULTI_BEACON_DELAY; // beacon delay
	P.Throttle_Rate = 0xff;
	P.Demag_Comp = DEFAULT_PGM_MULTI_DEMAG_COMP; // demag compensation
	P.BEC_Voltage_High = DEFAULT_PGM_BEC_VOLTAGE_HIGH; // BEC voltage
	P.Ppm_Center_Throttle = DEFAULT_PGM_PPM_CENTER_THROTTLE; // center throttle (final value is 4x+1000=1488)
	P.Main_Spoolup_Time = 0xff;
	P.Temp_Prot_Enable = DEFAULT_PGM_ENABLE_TEMP_PROT; // temperature protection enable

	P.Dummy = 0xffff; // EEPROM address for safety reason
	//P.Name[] = "                "; // Name tag (16 Bytes)
#endif
} // set_default_parameters


void read_all_eeprom_parameters(void) {

	ReadBlockArmFlash(0, sizeof(P), (uint8 *) (&P));

} // read_all_eeprom_parameters

void write_parameters_to_eeprom(void) {

	WriteBlockArmFlash(true, 0, 0, sizeof(P), (uint8 *) (&P));

} // write_parameters_to_eeprom

//___________________________________________________________________________
//
// Decode parameters
//
// No assumptions
// Decodes programming parameters
//___________________________________________________________________________

void decode_parameters(void) {

	// Load pwm frequency

	Temp8 = P.Pwm_Freq;
	F.PGM_PWMOFF_DAMPED = false;
#if (DAMPED_MODE_ENABLE==1)
	cjne Temp8, 3, ($+5)
	F.PGM_PWMOFF_DAMPED = true;
#endif
	// Load direction
#if (MODE >= 1)	// Tail or multi
	if (P.Direction == 3)
		goto decode_params_dir_set;
#endif

	F.PGM_DIR_REV = false;
	//A = @Temp1
	//jnb ACC.1, ($+5)
	F.PGM_DIR_REV = true;

	decode_params_dir_set:

	F.PGM_RCP_PWM_POL = false;

	F.PGM_RCP_PWM_POL = P.Input_Pol;

	if (P.Pwm_Freq = 3) {
		//zzCKCON,0x00h;		// Timer0 set for clk/12 (8kHz pwm)
		F.PGM_PWM_HIGH_FREQ = false;
	} else {
		//CKCON,0x01 // Timer0 set for clk/4 (22kHz pwm)
		F.PGM_PWM_HIGH_FREQ = true;
	}

} // decode_parameters

//___________________________________________________________________________
//
// Decode governor gain
//
// No assumptions
// Decodes governor gains
//___________________________________________________________________________
void decode_governor_gains(void) {
	/*
	 // Decode governor gains
	 Temp1 = #P.Gov_P_Gain	// Decode governor P gain
	 A = @Temp1
	 dec	A
	 DPTR, #GOV_GAIN_TABLE
	 movc A, @A+DPTR
	 Temp1 = #P.Gov_P_Gain_Decoded
	 @Temp1, A
	 Temp1 = #P.Gov_I_Gain	// Decode governor I gain
	 A = @Temp1
	 dec	A
	 DPTR, #GOV_GAIN_TABLE
	 movc A, @A+DPTR
	 Temp1 = #P.Gov_I_Gain_Decoded
	 @Temp1, A
	 switch_power_off();		// Reset DPTR
	 */
} // decode_governor_gains


//___________________________________________________________________________
//
// Decode startup power
//
// No assumptions
//___________________________________________________________________________

void decode_startup_power(void) {
	/*
	 // Decode startup power
	 Temp1 = #P.Startup_Pwr
	 A = @Temp1
	 A--;
	 DPTR = STARTUP_POWER_TABLE;
	 movc A, @A+DPTR
	 Temp1 = #P.Startup_Pwr_Decoded;
	 @Temp1 = A;
	 */
	switch_power_off(); // Reset DPTR
} // decode_startup_power


//___________________________________________________________________________
//
// Decode main spoolup time
//
// No assumptions
//___________________________________________________________________________
void decode_main_spoolup_time(void) {

#if (MODE==MAIN_MODE)
	Main_Spoolup_Time_3x = P.Main_Spoolup_Time *3;
	Main_Spoolup_Time_15x = P.Main_Spoolup_Time *15;
#endif
} // decode_main_spoolup_time


//___________________________________________________________________________
//
// Decode demag compensation
//
// No assumptions
// Decodes demag comp
//___________________________________________________________________________

void decode_demag_comp(void) {
	/*

	 // Decode demag compensation
	 Temp1 = #P.Demag_Comp;
	 A = @Temp1
	 Demag_Pwr_Off_Thresh, #255	// Set default
	 Low_Rpm_Pwr_Slope, #12		// Set default
	 cjne	A, #2, decode_demag_high

	 Demag_Pwr_Off_Thresh, #160	// Settings for demag comp low
	 Low_Rpm_Pwr_Slope, #10

	 decode_demag_high:
	 cjne	A, #3, decode_demag_done

	 Demag_Pwr_Off_Thresh, #130	// Settings for demag comp high
	 Low_Rpm_Pwr_Slope, #5

	 decode_demag_done:
	 */
} // decode_demag_comp

//___________________________________________________________________________
//
// Set BEC voltage
//
// No assumptions
// Sets the BEC output voltage low or high
//___________________________________________________________________________

void set_bec_voltage(void) {
	/*

	 // Set bec voltage
	 #if (HIGH_BEC_VOLTAGE==1)
	 Set_BEC_Lo			// Set default to low
	 Temp1 = #P.BEC_Voltage_High
	 A = @Temp1
	 jz	set_bec_voltage_exit

	 Set_BEC_Hi			// Set to high

	 set_bec_voltage_exit:
	 #endif
	 #if (HIGH_BEC_VOLTAGE==2
	 Set_BEC_0				// Set default to low
	 Temp1 = #P.BEC_Voltage_High
	 A = @Temp1
	 cjne	A, #1, set_bec_voltage_2

	 Set_BEC_1				// Set to level 1

	 set_bec_voltage_2:
	 cjne	A, #2, set_bec_voltage_exit

	 Set_BEC_2				// Set to level 2

	 set_bec_voltage_exit:
	 #endif
	 */
}

//___________________________________________________________________________
//
// Find throttle gain
//
// The difference between max and min throttle must be more than 520us
// (a P.Ppm_xxx_Throttle difference of 130)
//
// Finds throttle gain from throttle calibration values
//___________________________________________________________________________

void find_throttle_gain(void) {
	/*

	 // Load minimum and maximum throttle
	 Temp1 = #P.Ppm_Min_Throttle
	 A = @Temp1
	 Temp3  = A
	 Temp1 = #P.Ppm_Max_Throttle
	 A = @Temp1
	 Temp4, A
	 // Check if full range is chosen
	 jnb	F.FULL_THROTTLE_RANGE, find_throttle_gain_calculate

	 Temp3  = #0
	 Temp4, #255

	 find_throttle_gain_calculate:
	 // Calculate difference
	 C=0;
	 A = Temp4
	 subb	A, Temp3
	 Temp5, A
	 // Check that difference is minimum 130
	 C=0;
	 subb	A, #130
	 jnc	($+4)

	 Temp5, #130

	 // Find gain
	 Ppm_Throttle_Gain, #0
	 test_throttle_gain:
	 inc	Ppm_Throttle_Gain
	 A = Temp5
	 B, Ppm_Throttle_Gain	// A has difference, B has gain
	 mul	AB
	 C=0;
	 A = B
	 subb	A, #128
	 jc	test_throttle_gain
	 */
}

//___________________________________________________________________________
//
// Average throttle
//
// Outputs result in Temp3
//
// Averages throttle calibration readings
//
//___________________________________________________________________________

void average_throttle(void) {
	/*
	 setb	F.FULL_THROTTLE_RANGE	// Set range to 1000-2020us
	 find_throttle_gain();	// Set throttle gain
	 Delay1mS(30);
	 Temp3  = 0;
	 Temp4=0;
	 Temp5=16;		// Average 16 measurments

	 average_throttle_meas:
	 wait3ms			// Wait for new RC pulse value
	 A = New_Rcp;		// Get new RC pulse value
	 A += Temp3;
	 Temp3  = A
	 A = 0;
	 addc A, Temp4
	 Temp4=A;
	 djnz	Temp5, average_throttle_meas

	 Temp5=4;			// Shift 4 times
	 average_throttle_div:
	 C=0;
	 A = Temp4;   		// Shift right
	 A/=2;
	 Temp4= A;
	 A = Temp3;
	 A/=2;
	 Temp3  = A;
	 djnz	Temp5, average_throttle_div

	 Temp7= A;   		// Copy to Temp7
	 clr	F.FULL_THROTTLE_RANGE
	 find_throttle_gain();	// Set throttle gain
	 */
}

//___________________________________________________________________________
//
// Main program start
//___________________________________________________________________________


void fullreset(void) {
	/*
	 reset:
	 // Check flash lock byte
	 A = RSTSRC
	 jb ACC.6, ($+6) // Check if flash access error was reset source

	 Bit_Access=0;	// No - then this is the first try
	 inc Bit_Access
	 DPTR,
	 #LOCK_BYTE_ADDRESS_16K	// First try is for 16k flash size
	 A = Bit_Access
	 dec A
	 jz lock_byte_test

	 DPTR,
	 #LOCK_BYTE_ADDRESS_8K	// Second try is for 8k flash size
	 dec A
	 jz lock_byte_test

	 lock_byte_test:
	 movc A, @A+DPTR // Read lock byte
	 inc A
	 jz lock_byte_ok // If lock byte is 0xFF, then start code execution

	 #if (ONE_S_CAPABLE==0)
	 RSTSRC,
	 #12h		// Generate hardware reset and set VDD monitor
	 ELSE
	 RSTSRC,
	 #10h		// Generate hardware reset and disable VDD monitor
	 #endif

	 lock_byte_ok:
	 // Select register bank 0 for main program routines
	 clr PSW.3 // Select register bank 0 for main program routines
	 // Disable the WDT.
	 anl PCA0MD,
	 #NOT(40h)	// Clear watchdog enable bit
	 // Initialize stack
	 SP,
	 #0c0h			// Stack = 64 upper bytes of RAM
	 // Initialize VDD monitor
	 orl VDM0CN,
	 #080h    	// Enable the VDD monitor
	 Delay1uS(1000); // Wait at least 100us
	 #if (ONE_S_CAPABLE==0
	 RSTSRC,
	 #02h   	// Set VDD monitor as a reset source (PORSF) if not 1S capable
	 ELSE
	 RSTSRC,
	 #00h   	// Do not set VDD monitor as a reset source for 1S ESCSs, in order to avoid resets due to it
	 #endif
	 // Set clock frequency
	 orl OSCICN,
	 #03h		// Set clock divider to 1
	 A = OSCICL
	 add A,
	 #04h			// 24.5MHz to 25MHz (~0.5% per step)
	 jb ACC.7, reset_cal_done // Is carry (7bit) set? - skip next instruction

	 OSCICL, A

	 reset_cal_done:
	 // Switch power off
	 switch_power_off();
	 // Ports initialization
	 P0 = P0_INIT;
	 P0MDOUT = P0_PUSHPULL;
	 P0MDIN = P0_DIGITAL;
	 P0SKIP = P0_SKIP;
	 P1 = P1_INIT;
	 P1MDOUT = P1_PUSHPULL;
	 P1MDIN = P1_DIGITAL;
	 P1SKIP=P1_SKIP
	 #if (PORT3_EXIST==1)
	 P2=P2_INIT;
	 #endif
	 P2MDOUT=P2_PUSHPULL;
	 #if (PORT3_EXIST==1)
	 P2MDIN=P2_DIGITAL;
	 P2SKIP=P2_SKIP;
	 P3=P3_INIT;
	 P3MDOUT=P3_PUSHPULL;
	 P3MDIN=P3_DIGITAL;
	 #endif
	 */
	//zzInitialize_Xbar();

	set_default_parameters();
	read_all_eeprom_parameters();

	//zz EA = 0; // Disable interrupts explicitly
	Delay1mS(200);
	beep_f1();
	Delay1mS(30);
	beep_f2();
	Delay1mS(30);
	beep_f3();
	Delay1mS(30);
#if ((MODE==MAIN_MODE) || (MODE==TAIL_MODE))
	// Wait for receiver to initialize
	Delay1mS(501);
#endif

}

//___________________________________________________________________________
//
// No signal entry point
//___________________________________________________________________________

void init_no_signal(void) {

	//disable interrupts, clear RAM

	set_default_parameters();
	read_all_eeprom_parameters();
	decode_parameters();
	decode_governor_gains();
	decode_startup_power();
	decode_main_spoolup_time();
	decode_demag_comp();

	set_bec_voltage();
	find_throttle_gain();

	switch_power_off();

	/*

	 // Timer control
	 TCON, #50h		// Timer0 and timer1 enabled
	 // Timer mode
	 TMOD, #12h		// Timer0 as 8bit, timer1 as 16bit
	 // Timer2: clk/12 for 128us and 32ms interrupts
	 TMR2CN, #24h		// Timer2 enabled, low counter interrups enabled
	 // Timer3: clk/12 for commutation timing
	 TMR3CN, #04h		// Timer3 enabled
	 // PCA
	 PCA0CN, #40h		// PCA enabled
	 // Enable interrupts
	 IE, #22h			// Enable timer0 and timer2 interrupts
	 IP, #02h			// High priority to timer0 interrupts
	 EIE1, #90h		// Enable timer3 and PCA0 interrupts
	 // Initialize comparator
	 CPT0CN= 0x80;		// Comparator enabled, no hysteresis
	 CPT0MD= 0x00;		// Comparator response time 100ns
	 #if (COMP1_USED==1
	 CPT1CN=0x80;		// Comparator enabled, no hysteresis
	 CPT1MD= 0x00;		// Comparator response time 100ns
	 #endif
	 // Initialize ADC
	 Initialize_Adc();			// Initialize ADC operation
	 Delay1uS(1000);
	 //zzEA = 1				// Enable all interrupts
	 // Measure number of lipo cells
	 Measure_Lipo_Cells();			// Measure number of lipo cells
	 // Initialize rc pulse
	 Rcp_Int_Enable();		 			// Enable interrupt
	 Rcp_Clear_Int_Flag(); 				// Clear interrupt flag
	 F.RCP_EDGE_NO=false;			// Set first edge flag
	 Delay1mS(200);
	 // Set initial arm variable
	 Initial_Arm=1;

	 // Measure PWM frequency
	 measure_pwm_freq_init:
	 F.RCP_MEAS_PWM_FREQ=true; 		// Set measure pwm frequency flag
	 Temp4=3;						// Number of attempts before going back to detect input signal
	 measure_pwm_freq_start:
	 Temp3  = 12;					// Number of pulses to measure
	 measure_pwm_freq_loop:
	 // Check if period diff was accepted
	 A = Rcp_Period_Diff_Accepted
	 jnz	measure_pwm_freq_wait

	 Temp3  = #12					// Reset number of pulses to measure
	 djnz	Temp4, ($+4)					// If it is not zero - proceed
	 ajmp	init_no_signal					// Go back to detect input signal

	 measure_pwm_freq_wait:
	 Delay1mS(30);						// Wait 30ms for new pulse
	 jb	F.RCP_UPDATED, ($+5)		// Is there an updated RC pulse available - proceed
	 ajmp	init_no_signal					// Go back to detect input signal

	 F.RCP_UPDATED=false;		 		// Flag that pulse has been evaluated
	 A = New_Rcp;					// Load value
	 C=0;
	 subb	A, #RCP_VALIDATE				// Higher than validate level?
	 jc	measure_pwm_freq_start			// No - start over

	 A = Flags3						// Check pwm frequency flags
	 anl	A, #((1 SHL RCP_PWM_FREQ_1KHZ)+(1 SHL RCP_PWM_FREQ_2KHZ)+(1 SHL RCP_PWM_FREQ_4KHZ)+(1 SHL RCP_PWM_FREQ_8KHZ)+(1 SHL RCP_PWM_FREQ_12KHZ))
	 Prev_Rcp_Pwm_Freq=Curr_Rcp_Pwm_Freq;		// Store as previous flags for next pulse
	 Curr_Rcp_Pwm_Freq=A;					// Store current flags for next pulse
	 cjne	A, Prev_Rcp_Pwm_Freq, measure_pwm_freq_start	// Go back if new flags not same as previous

	 djnz	Temp3, measure_pwm_freq_loop				// Go back if not required number of pulses seen

	 // Clear measure pwm frequency flag
	 clr	F.RCP_MEAS_PWM_FREQ
	 // Set up RC pulse interrupts after pwm frequency measurement
	 Rcp_Int_First 						// Enable interrupt and set to first edge
	 Rcp_Clear_Int_Flag 					// Clear interrupt flag
	 clr	F.RCP_EDGE_NO				// Set first edge flag
	 // Test whether signal is OnShot125
	 clr	F.RCP_PPM_ONESHOT125		// Clear OneShot125 flag
	 Rcp_Outside_Range_Cnt, #0		// Reset out of range counter
	 Delay1mS(100);						// Wait for new RC pulse
	 jnb	F.RCP_PPM, validate_setpoint_start	// If flag is not set (PWM) - branch

	 C=0;
	 A = Rcp_Outside_Range_Cnt			// Check how many pulses were outside normal PPM range (800-2160us)
	 subb	A, #10
	 jc	validate_setpoint_start

	 setb	F.RCP_PPM_ONESHOT125		// Set OneShot125 flag

	 // Validate RC pulse
	 validate_setpoint_start:
	 Delay1uS(30000);						// Wait for next pulse (NB: Uses Temp1/2!)
	 Temp1 = #RCP_VALIDATE			// Set validate level as default
	 jnb	F.RCP_PPM, ($+5)			// If flag is not set (PWM) - branch

	 Temp1 = #0						// Set level to zero for PPM (any level will be accepted)

	 C=0;
	 A = New_Rcp					// Load value
	 subb	A, Temp1						// Higher than validate level?
	 jc	validate_setpoint_start				// No - start over

	 // Beep arm sequence start signal
	 clr 	EA							// Disable all interrupts
	 beep_f1();						// Signal that RC pulse is ready
	 beep_f1();
	 beep_f1();
	 //zzEA = 1							// Enable all interrupts
	 Delay1mS(200);

	 // Arming sequence start
	 Gov_Arm_Target, #0		// Clear governor arm target
	 arming_start:
	 #if (MODE >= 1)	// Tail or multi
	 Temp1 = P.Direction;	// Check if bidirectional operation
	 A = @Temp1
	 cjne	A, #3, ($+5)

	 ajmp	program_by_tx_checked	// Disable tx programming if bidirectional operation
	 #endif

	 wait3ms
	 Temp1 = #P.Enable_TX_Program// Start programming mode entry if enabled
	 A = @Temp1
	 C=0;
	 subb	A, #1				// Is TX programming enabled?
	 jnc 	arming_initial_arm_check	// Yes - proceed

	 jmp	program_by_tx_checked	// No - branch

	 arming_initial_arm_check:
	 A = Initial_Arm			// Yes - check if it is initial arm sequence
	 C=0;
	 subb	A, #1				// Is it the initial arm sequence?
	 jnc 	arming_ppm_check		// Yes - proceed

	 jmp 	program_by_tx_checked	// No - branch

	 arming_ppm_check:
	 jb	F.RCP_PPM, throttle_high_cal_start	// If flag is set (PPM) - branch

	 // PWM tx program entry
	 C=0;
	 A = New_Rcp			// Load new RC pulse value
	 subb	A, #RCP_MAX			// Is RC pulse max?
	 jnc	program_by_tx_entry_pwm	// Yes - proceed

	 jmp	program_by_tx_checked	// No - branch

	 program_by_tx_entry_pwm:
	 EA=0;					// Disable all interrupts
	 beep_f4();
	 //zzEA = 1					// Enable all interrupts
	 Delay1mS(100);
	 C=0;
	 A = New_Rcp			// Load new RC pulse value
	 subb	A, #RCP_STOP			// Below stop?
	 jnc	program_by_tx_entry_pwm	// No - start over

	 program_by_tx_entry_wait_pwm:
	 EA=0;					// Disable all interrupts
	 beep_f1();
	 Delay1mS(10);
	 beep_f1();
	 //zzEA = 1					// Enable all interrupts
	 Delay1mS(100);
	 C=0;
	 A = New_Rcp;			// Load new RC pulse value
	 subb	A, #RCP_MAX			// At or above max?
	 jc	program_by_tx_entry_wait_pwm	// No - start over

	 jmp	program_by_tx			// Yes - enter programming mode

	 // PPM throttle calibration and tx program entry
	 throttle_high_cal_start:
	 #if (MODE <= 1	// Main or tail
	 Temp8, #5				// Set 3 seconds wait time
	 #else
	 Temp8, #2				// Set 1 seconds wait time
	 #endif
	 throttle_high_cal:
	 F.FULL_THROTTLE_RANGE=true;	// Set range to 1000-2020us
	 find_throttle_gain		// Set throttle gain
	 wait100ms				// Wait for new throttle value
	 EA=0;					// Disable interrupts (freeze New_Rcp value)
	 F.FULL_THROTTLE_RANGE=false;	// Set range
	 find_throttle_gain		// Set throttle gain
	 Temp7, New_Rcp			// Store new RC pulse value
	 C=0;
	 A = New_Rcp			// Load new RC pulse value
	 subb	A, #(RCP_MAX/2)		// Is RC pulse above midstick?
	 //zzEA = 1					// Enable interrupts
	 jc	arm_target_updated		// No - branch

	 wait1ms
	 EA=0;					// Disable all interrupts
	 beep_f4();
	 //zzEA = 1					// Enable all interrupts
	 djnz	Temp8, throttle_high_cal	// Continue to wait

	 average_throttle();
	 C=0;
	 A = Temp7;				// Limit to max 250
	 subb	A, #5				// Subtract about 2% and ensure that it is 250 or lower
	 Temp1 = #P.Ppm_Max_Throttle	// Store
	 @Temp1, A
	 Delay1mS(200);
	 erase_and_store_all_in_eeprom
	 success_beep

	 throttle_low_cal_start:
	 Temp8, #10			// Set 3 seconds wait time
	 throttle_low_cal:
	 F.FULL_THROTTLE_RANGE = true;	// Set range to 1000-2020us
	 find_throttle_gain		// Set throttle gain
	 wait100ms
	 EA=0;					// Disable interrupts (freeze New_Rcp value)
	 F.FULL_THROTTLE_RANGE=false;	// Set range
	 find_throttle_gain		// Set throttle gain
	 Temp7, New_Rcp			// Store new RC pulse value
	 C=0;
	 A = New_Rcp;			// Load new RC pulse value
	 subb	A, #(RCP_MAX/2)		// Below midstick?
	 //zzEA = 1					// Enable interrupts
	 jnc	throttle_low_cal_start	// No - start over

	 Delay1mS(1);
	 EA=0;					// Disable all interrupts
	 beep_f1();
	 Delay1mS(10);
	 beep_f1();
	 //zzEA = 1					// Enable all interrupts
	 djnz	Temp8, throttle_low_cal	// Continue to wait

	 average_throttle
	 A = Temp7
	 add	A, #5				// Add about 2%
	 Temp1 = #P.Ppm_Min_Throttle	// Store
	 @Temp1, A
	 Delay1mS(200);
	 erase_and_store_all_in_eeprom();
	 success_beep_inverted();

	 program_by_tx_entry_wait_ppm:
	 Delay1mS(100);
	 find_throttle_gain();		// Set throttle gain
	 C=0;
	 A = New_Rcp;			// Load new RC pulse value
	 subb	A, #RCP_MAX			// At or above max?
	 jc	program_by_tx_entry_wait_ppm	// No - start over

	 goto	program_by_tx;			// Yes - enter programming mode

	 program_by_tx_checked:
	 C=0;
	 A = New_Rcp;			// Load new RC pulse value
	 subb	A, Gov_Arm_Target		// Is RC pulse larger than arm target?
	 jc	arm_target_updated		// No - do not update

	 Gov_Arm_Target, New_Rcp	// Yes - update arm target

	 arm_target_updated:
	 Delay1mS(100);				// Wait for new throttle value
	 Temp1 = RCP_STOP;		// Default stop value
	 Temp2 = P.Direction;	// Check if bidirectional operation
	 A = @Temp2
	 cjne	A, #3, ($+5)			// No - branch

	 Temp1 = #(RCP_STOP+4)	// Higher stop value for bidirectional

	 C=0;
	 A = New_Rcp;			// Load new RC pulse value
	 subb	A, Temp1;				// Below stop?
	 jc	arm_end_beep			// Yes - proceed

	 jmp	arming_start			// No - start over

	 arm_end_beep:
	 // Beep arm sequence end signal
	 EA=0;					// Disable all interrupts
	 beep_f4();				// Signal that rcpulse is ready
	 beep_f4();
	 beep_f4();
	 //zzEA = 1;					// Enable all interrupts
	 Delay1mS(200);

	 // Clear initial arm variable
	 Initial_Arm=false;

	 // Armed and waiting for power on
	 wait_for_power_on:
	 A=0;
	 Power_On_Wait_Cnt_L= A;	// Clear wait counter
	 Power_On_Wait_Cnt_H = A;
	 wait_for_power_on_loop:
	 Power_On_Wait_Cnt_L++;		// Increment low wait counter
	 A = Power_On_Wait_Cnt;
	 cpl	A
	 jnz	wait_for_power_on_no_beep// Counter wrapping (about 1 sec)?

	 Power_On_Wait_Cnt_H++;		// Increment high wait counter
	 Temp1 = #P.Beacon_Delay;
	 A = @Temp1
	 Temp1 = 25;		// Approximately 1 min
	 A--;
	 jz	beep_delay_set

	 Temp1 = 50;		// Approximately 2 min
	 A--;
	 jz	beep_delay_set

	 Temp1 = 125;		// Approximately 5 min
	 A--;
	 jz	beep_delay_set

	 Temp1 = 250;		// Approximately 10 min
	 A--;
	 jz	beep_delay_set

	 Power_On_Wait_Cnt_H=0;		// Reset counter for infinite delay

	 beep_delay_set:
	 C=0;
	 A = Power_On_Wait_Cnt_H
	 subb	A, Temp1				// Check against chosen delay
	 jc	wait_for_power_on_no_beep// Has delay elapsed?

	 Power_On_Wait_Cnt_H--;		// Decrement high wait counter
	 Power_On_Wait_Cnt_L= 180; // Set low wait counter
	 Temp1 = #P.Beacon_Strength;
	 Beep_Strength= @Temp1;
	 EA=0;					// Disable all interrupts
	 beep_f4();				// Signal that there is no signal
	 //zzEA = 1					// Enable all interrupts
	 Temp1 = P.Beep_Strength;
	 Beep_Strength= @Temp1;
	 Delay1mS(100);				// Wait for new RC pulse to be measured

	 wait_for_power_on_no_beep:
	 Delay1mS(100);
	 A = Rcp_Timeout_Cnt;				// Load RC pulse timeout counter value
	 jnz	wait_for_power_on_ppm_not_missing	// If it is not zero - proceed

	 jnb	F.RCP_PPM, wait_for_power_on_ppm_not_missing	// If flag is not set (PWM) - branch

	 jmp	init_no_signal					// If ppm and pulses missing - go back to detect input signal

	 wait_for_power_on_ppm_not_missing:
	 Temp1 = RCP_STOP;
	 jb	F.RCP_PPM, ($+5)	// If flag is set (PPM) - branch

	 Temp1 = (RCP_STOP+5) 	// Higher than stop (for pwm)

	 C=0;
	 A = New_Rcp;			// Load new RC pulse value
	 A-=Temp1;		 		// Higher than stop (plus some hysteresis)?
	 jc	wait_for_power_on_loop	// No - start over

	 #if (MODE >= 1)	// Tail or multi
	 Temp1 = #P.Direction	// Check if bidirectional operation
	 A = @Temp1;
	 C=0;
	 A-=3;
	 jz 	wait_for_power_on_check_timeout	// Do not wait if bidirectional operation
	 #endif

	 lwait100ms			// Wait to see if start pulse was only a glitch

	 wait_for_power_on_check_timeout:
	 A = Rcp_Timeout_Cnt;		// Load RC pulse timeout counter value
	 jnz	($+4)				// If it is not zero - proceed

	 ajmp	init_no_signal			// If it is zero (pulses missing) - go back to detect input signal
	 */
}

//___________________________________________________________________________
//
// Start entry point
//___________________________________________________________________________


void init_start(void) {

	//zz EA = 0;
	switch_power_off();
	Requested_Pwm = Governor_Req_Pwm = Current_Pwm = Current_Pwm_Limited = 0;
	// enable interrupts? //zz EA = 1;

	Gov_Target = Gov_Integral = Gov_Integral_X = 0;

	Gov_Active = false;
	// clear flags here
	Demag_Detected_Metric = false;

	initialize_all_timings();

	// Motor start beginning

	//zzzCurrent_Average_Temp = GetAverageTemperature();

	check_temp_voltage_and_limit_power();

	// Set up start operating conditions
	Temp1 = P.Pwm_Freq;
	A = Temp1;
	Temp7 = A; // Store setting in Temp7
	P.Pwm_Freq = 2; // Set nondamped low frequency pwm mode
	decode_parameters(); // (Decode_parameters uses Temp1 and Temp8)
	Temp1 = P.Pwm_Freq;
	A = Temp7;
	Temp1 = Temp7; // Restore settings

	// Set max allowed power
	//zz EA = 0; // Disable interrupts to avoid that Requested_Pwm is overwritten
	Pwm_Limit = 0xff; // Set pwm limit to max
	//zz set_startup_pwm();
	Pwm_Limit = Requested_Pwm;
	Pwm_Limit_Spoolup = Requested_Pwm;
	Pwm_Limit_Low_Rpm = Requested_Pwm;

	//zz //zzEA = 1
	Requested_Pwm = 1; // Set low pwm again after calling set_startup_pwm
	Current_Pwm = 1;
	Current_Pwm_Limited = 1;
	Spoolup_Limit_Cnt = Auto_Bailout_Armed;
	Spoolup_Limit_Skip = 1;

	// Begin startup sequence

	F.STARTUP_PHASE = F.MOTOR_SPINNING = true;
	Startup_Ok_Cnt = 0;
	comm5comm6();
	comm6comm1();
	initialize_all_timings();
	calc_next_comm_timing();
	calc_new_wait_times();
	runState = run1;

} // init_start

void DoHousekeeping(void) {
	enum {
		validate_setpoint_start,
		initialState,
		damped_transition,
		initial_run_phase_done,
		run6_check_speed,
		wait_for_power_on,
		jmp_wait_for_power_on,
		run_to_next_state_main,
		init_no_signal,
		direct_start_check_setpoint,
		direct_start_check_rcp,
		run6_check_rcp_stop_count,
		run6_check_setpoint_timeout,
		run6_check_rcp_timeout,
		normal_run_check_startup_rot,
		run_to_wait_for_power_on,
		normal_run_checks,
		run6_check_setpoint_stop_count,
		finished_startup
	};
	uint8 startState = initialState;

	do {
		switch (startState) {
		case direct_start_check_rcp:
			if (New_Rcp > RCP_STOP) // Check if pulse is below stop value
				startState = run_to_wait_for_power_on;
			else {
				runState = run1;
				startState == normal_run_checks;
			}
			break;
		case normal_run_checks:
			//  Check if it is initial run phase
			if (F.DIR_CHANGE_BRAKE || F.INITIAL_RUN_PHASE)
				startState = initial_run_phase_done; // If a direction change - branch
			else {
				// Decrement startup rotaton count
				if (Startup_Rot_Cnt > 0)
					startState = normal_run_check_startup_rot; // Branch if counter is not zero
				else {
					F.INITIAL_RUN_PHASE = false;// Clear initial run phase flag
#if (MODE == MULTI_MODE)
					Pwm_Limit = 0xFF;
#endif
					startState = damped_transition; // Do damped transition if counter is zero
				}
			}
			break;
		case normal_run_check_startup_rot:
			if (New_Rcp < Startup_Rot_Cnt)// Load new pulse value
				startState = run_to_wait_for_power_on;
			else {
				runState = run1;
				startState = finished_startup;
			}
			break;
		case initial_run_phase_done:
			if (Rcp_Stop_Cnt == 0) {
				Pwm_Limit_Spoolup = Pwm_Spoolup_Beg; // If yes - set initial max powers
				Spoolup_Limit_Cnt = Auto_Bailout_Armed; // And set spoolup parameters
				Spoolup_Limit_Skip = 1;
			}
			startState = run6_check_rcp_stop_count;
			break;
		case run6_check_rcp_stop_count:
			if (Rcp_Stop_Cnt > RCP_STOP_LIMIT)
				startState = run_to_wait_for_power_on;
			else
				startState = run6_check_rcp_timeout;
			break;
		case run6_check_rcp_timeout:
			if (!F.RCP_PPM)
				startState = run6_check_speed;
			else if (Rcp_Timeout_Cnt == 0)
				startState = run_to_wait_for_power_on;
			// else stay
			break;
		case run6_check_speed:

			// Is Comm_Period4x more than 32ms (~1220 eRPM)?
			if (F.DIR_CHANGE_BRAKE) // Is it a direction change?
				Temp1 = 0x6000; // Bidirectional minimum speed
			else
				Temp1 = 0xf000; //Default minimum speed

			if (Comm_Period4x > Temp1)
				startState = run_to_wait_for_power_on; // Yes - go back to motor start
			else {
				runState = run1;
				startState = finished_startup;
			}
			break;
		case run_to_wait_for_power_on:

			//zzzclr EA
			switch_power_off();
			/*
			 Temp1, P.Pwm_Freq
			 A, @Temp1
			 Temp7, A; Store setting in Temp7
			 @Temp1,2 //Set low pwm mode (in order to turn off damping)
			 decode_parameters//(Decode_parameters uses Temp1 and Temp8)
			 Temp1, P.Pwm_Freq
			 A, Temp7
			 @Temp1, A; Restore settings
			 */

			Requested_Pwm = Governor_Req_Pwm = Current_Pwm
					= Current_Pwm_Limited = Pwm_Motor_Idle = 0;
			F.MOTOR_SPINNING = false; //Clear motor spinning flag

			//zzEA=1
			Delay1uS(1000); //Wait for pwm to be stopped
			switch_power_off();

#if ( MODE == MODE_MAIN)
			if(!Flags2.RCP_PPM)
			startState = run_to_next_state_main;
			else
			if(Rcp_Timeout_Cnt==0)
			startState = init_no_signal;// If it is zero (pulses missing) - go back to detect input signal
			else
			startState = run_to_next_state_main; //If it is not zero - branch
			break;
			case run_to_next_state_main:
			if (Pgm_Main_Rearm_Start == 0)// Is re-armed start enabled?
			startState = jmp_wait_for_power_on; No - do like tail and start immediately
			else
			startState = validate_rcp_start;// Yes - go back to validate RC pulse
			case jmp_wait_for_power_on:
			startState = wait_for_power_on;// Go back to wait for power on
#else	//  Tail or multi
			if (!F.RCP_PPM)
				startState = jmp_wait_for_power_on;
			else if (Rcp_Timeout_Cnt == 0)
				startState = init_no_signal;
			else
				startState = jmp_wait_for_power_on;
		case jmp_wait_for_power_on:
			startState = wait_for_power_on; //Go back to wait for power on
			break;
#endif

			break;
		case finished_startup:
			break;
		}
	} while (startState != finished_startup);

} // DoHousekeeping


//___________________________________________________________________________
//
// Run entry point
//___________________________________________________________________________

int main(void) {

	//zz	damped_transition;
	// Transition from nondamped to damped if applicable
	switch_power_off(); // Switch off power while changing pwm mode
	decode_parameters();

	init_start();

	while (true) {

		evaluate_comparator_integrity();
		setup_comm_wait();

		switch (runState) {
		case run1:
			// Run 1 = B(p-on) + C(n-pwm) - comparator A evaluated
			// Out_cA changes from low to high
			wait_for_comp_out_high(); // Wait zero cross wait and wait for high
			calc_governor_target(); // Calculate governor target
			wait_for_comm(); // Wait from zero cross to commutation
			comm1comm2(); // Commutate
			runState++;
			break;
			// Run 2 = A(p-on) + C(n-pwm) - comparator B evaluated
			// Out_cB changes from high to low
		case run2:
			wait_for_comp_out_low();
			calc_governor_prop_error();
			set_pwm_limit_low_rpm();
			wait_for_comm();
			comm2comm3();
			runState++;
			break;
			// Run 3 = A(p-on) + B(n-pwm) - comparator C evaluated
			// Out_cC changes from low to high
		case run3:
			wait_for_comp_out_high();
			calc_governor_int_error();
			wait_for_comm();
			comm3comm4();
			runState++;
			break;
			// Run 4 = C(p-on) + B(n-pwm) - comparator A evaluated
			// Out_cA changes from high to low
		case run4:
			wait_for_comp_out_low();
			evaluate_comparator_integrity();
			setup_comm_wait();
			calc_governor_prop_correction();
			wait_for_comm();
			comm4comm5();
			runState++;
			break;
			// Run 5 = C(p-on) + A(n-pwm) - comparator B evaluated
			// Out_cB changes from low to high
		case run5:
			wait_for_comp_out_high();
			calc_governor_int_correction();
			wait_for_comm();
			comm5comm6();
			runState++;
			break;
			// Run 6 = B(p-on) + A(n-pwm) - comparator C evaluated
			// Out_cC changes from high to low
		case run6:
			wait_for_comp_out_low();
			start_adc_conversion();
			evaluate_comparator_integrity();
			setup_comm_wait();
			check_temp_voltage_and_limit_power();
			wait_for_comm();
			comm6comm1();
			runState = run1;
		} // switch

		calc_next_comm_timing();
		wait_advance_timing();
		calc_new_wait_times();
		wait_before_zc_scan();

		DoHousekeeping();

	} // main commutation loop

	return 0;
}
// main


