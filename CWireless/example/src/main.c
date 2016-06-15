#include "board.h"

#define TIME_INTERVAL   (1)
#define VIN_PIN 1
#define VOUT_PIN 0
#define CURRENT_PIN 2
#define LOAD_PIN 3
#define intFactor 1
#define MPPTFactor 50
#define VMAX 4800
#define FMAX 1800
#define FMIN 400
#define ncycles 1000
#define VLimit 3100
#define delayFactor 200

#define enableMPPT
//#define enableLoad
//#define enableFreq

static volatile bool On;
static volatile bool over;
static ADC_CLOCK_SETUP_T ADCSetup;
static volatile bool enableOut;
static volatile bool enablePrev;
static volatile int32_t vout;
static volatile uint16_t time;
static volatile uint16_t freq;
static volatile uint16_t freqOld;
static volatile bool controlFlag;
static volatile uint16_t times;
static int32_t Vmeasure;
static int32_t Imeasure;
static int32_t Vold;
static int32_t Iold;
static int16_t voutOld;
static uint16_t cycles;

static uint16_t readADC(uint8_t id)
{
	uint16_t dataADC;
	uint16_t first;
	Chip_ADC_EnableChannel(LPC_ADC, id, ENABLE);
    Chip_ADC_SetStartMode(LPC_ADC, ADC_START_NOW, ADC_TRIGGERMODE_RISING);

	/* Waiting for A/D conversion complete */
	while (Chip_ADC_ReadStatus(LPC_ADC, id, ADC_DR_DONE_STAT) != SET) {}
	/* Read ADC value */
	Chip_ADC_ReadValue(LPC_ADC, id, &first);

	Chip_ADC_SetStartMode(LPC_ADC, ADC_START_NOW, ADC_TRIGGERMODE_RISING);

		/* Waiting for A/D conversion complete */
	while (Chip_ADC_ReadStatus(LPC_ADC, id, ADC_DR_DONE_STAT) != SET) {}
		/* Read ADC value */
	Chip_ADC_ReadValue(LPC_ADC, id, &dataADC);

	int32_t diff = first - dataADC;
	if(diff < 0 ) diff = -diff;
	while(diff > 200) {
		first = dataADC;
		Chip_ADC_SetStartMode(LPC_ADC, ADC_START_NOW, ADC_TRIGGERMODE_RISING);

				/* Waiting for A/D conversion complete */
		while (Chip_ADC_ReadStatus(LPC_ADC, id, ADC_DR_DONE_STAT) != SET) {}
				/* Read ADC value */
		Chip_ADC_ReadValue(LPC_ADC, id, &dataADC);
		diff = first - dataADC;
		if(diff < 0 ) diff = -diff;
	}
	Chip_ADC_EnableChannel(LPC_ADC, id, DISABLE);
	return dataADC;
}

void DCACControl(void) {
	if(enableOut < enablePrev) {
		LPC_IOCON->PINSEL[3] = LPC_IOCON->PINSEL[3] && !(1 << 6);
		LPC_IOCON->PINSEL[3] = LPC_IOCON->PINSEL[3] && !(1 << 12);
	} else if(enableOut > enablePrev){
		LPC_IOCON->PINSEL[3] |= (1 << 6);
		LPC_IOCON->PINSEL[3] |= (1 << 12);
	}
}

void DCDCControl(void) {
	uint16_t voltage = readADC(VOUT_PIN);
	if(voltage > VLimit) {
		DEBUGOUT("WOOP\n");
		if(over) {
			vout = vout - 10;
			DEBUGOUT("Overvoltage %d\n",voltage);
		} else {
			over = true;
		}

	} else {
		over = false;
	}
	if(vout > 4800) vout = 4800; //Limit duty cycle
	if(vout < 0) vout = 0; //Minimal duty cycle

	Chip_PWM_SetMatch(LPC_PWM1, 1, vout);
	Chip_PWM_LatchEnable(LPC_PWM1, 1, PWM_OUT_ENABLED);
	Chip_PWM_SetMatch(LPC_PWM1, 2, vout);
	Chip_PWM_LatchEnable(LPC_PWM1, 2, PWM_OUT_ENABLED);
}


void DCACSetFreq(uint16_t freq) {
	if(freq < FMIN) {
		freq = FMIN;
	}
	if(freq > FMAX) {
		freq = FMAX;
	}
	LPC_MCPWM->LIM[0] = freq;
	LPC_MCPWM->MAT[0] = (freq>>1)-12;
}

void MPPT(int32_t Vmeas, int32_t Imeas) { //PUT MPPT here
	int32_t P = Vmeas*Imeas - Vold*Iold;
	DEBUGOUT("MPPT: %d %d old %d %d\n", Vmeas, Imeas, Vold, Iold);
	if(P >= 0) { //Power has increased;
		if(voutOld > vout) { //Decreased the voltage
			vout = vout - MPPTFactor;
		} else { //Increased the voltage
			vout = vout + MPPTFactor;
		}
	} else {	//Power decreased
		if(voutOld > vout) { //Decreased the voltage
			vout = vout + MPPTFactor;
		} else { //Increased the voltage
			vout = vout - MPPTFactor;
		}
	}
	if(Vmeas < 50) {
		vout = 0;
	}
	if(vout > VMAX) {
		vout = VMAX;
	}
	if(vout < 0) {
		vout = 0;
	}
	voutOld = vout;
	Vold = Vmeas;
	Iold = Imeas;
}

void MPPTFreq(int32_t Vmeas, int32_t Imeas) { //PUT MPPT here
	int32_t P = Vmeas*Imeas - Vold*Iold;
	if(P >= 0) { //Power has increased;
		if(freqOld > freq) { //Decreased the voltage
			freq = freq - 1;
		} else { //Increased the voltage
			freq = freq + 1;
		}
	} else {	//Power decreased
		if(freqOld > freq) { //Decreased the voltage
			freq = freq + 1;
		} else { //Increased the voltage
			freq = freq - 1;
		}
	}
	if(freq > FMAX) {
		freq = FMAX;
	}
	if(freq < FMIN) {
		freq = FMIN;
	}
	DCACSetFreq(freq);
	freqOld = freq;
	Vold = Vmeas;
	Iold = Imeas;
}


void RIT_IRQHandler(void) {
	/* Clearn interrupt */
	Chip_RIT_ClearInt(LPC_RITIMER);
	if(controlFlag)
		DEBUGOUT("Overloaded\n");
	controlFlag = true;
}

void setupClock(void) {
	/* Disconnect the Main PLL if it is connected already */
		if (Chip_Clock_IsMainPLLConnected()) {
			Chip_Clock_DisablePLL(SYSCTL_MAIN_PLL, SYSCTL_PLL_CONNECT);
		}

		/* Disable the PLL if it is enabled */
		if (Chip_Clock_IsMainPLLEnabled()) {
			Chip_Clock_DisablePLL(SYSCTL_MAIN_PLL, SYSCTL_PLL_ENABLE);
		}

		/* Enable the crystal */
		if (!Chip_Clock_IsCrystalEnabled())
			Chip_Clock_EnableCrystal();
		while(!Chip_Clock_IsCrystalEnabled()) {}

		/* Set PLL0 Source to Crystal Oscillator */
		Chip_Clock_SetCPUClockDiv(0);
		Chip_Clock_SetMainPLLSource(SYSCTL_PLLCLKSRC_MAINOSC);

		/* FCCO = ((19+1) * 2 * 12MHz) / (0+1) = 480MHz */
		Chip_Clock_SetupPLL(SYSCTL_MAIN_PLL, 19, 0);

		Chip_Clock_EnablePLL(SYSCTL_MAIN_PLL, SYSCTL_PLL_ENABLE);

		/* 480MHz / (3+1) = 120MHz */
		Chip_Clock_SetCPUClockDiv(3);
		while (!Chip_Clock_IsMainPLLLocked()) {} /* Wait for the PLL to Lock */

		Chip_Clock_EnablePLL(SYSCTL_MAIN_PLL, SYSCTL_PLL_CONNECT);

		Chip_SYSCTL_SetFLASHAccess(FLASHTIM_120MHZ_CPU);
}

/* Polling routine for ADC example */

/**
 * @brief	Main entry point
 * @return	Nothing
 */
int main(void) {
	SystemCoreClockUpdate();
	Board_Init();
	setupClock();
	SystemCoreClockUpdate();

	On = true;
	enableOut = false;
	controlFlag = false;

	Board_LED_Set(0, On);
	DEBUGOUT("Starting\n");
	/* Initialize RITimer */
	Chip_RIT_Init(LPC_RITIMER);

	LPC_IOCON->PINSEL[4] |= 0x00000555; //Change this after you know which pwm outputs are needed.
	LPC_IOCON->PINMODE[3] |= (3 << 6);
	LPC_IOCON->PINMODE[3] |= (3 << 12);

	LPC_IOCON->PINSEL[1] |= (1 << 14);
	LPC_IOCON->PINSEL[1] |= (1 << 16);
	LPC_IOCON->PINSEL[1] |= (1 << 18);
	LPC_IOCON->PINSEL[1] |= (1 << 20);
	LPC_IOCON->PINMODE[1] |= (2 << 14);
	LPC_IOCON->PINMODE[1] |= (2 << 16);
	LPC_IOCON->PINMODE[1] |= (2 << 18);
	LPC_IOCON->PINMODE[1] |= (2 << 20);

	LPC_SYSCTL->PCLKSEL[0] |= (1 << 12); //PCLK_PWM1 = CCLK
	LPC_IOCON->PINMODE[4] |= (3 << 26);
	LPC_SYSCTL->PCONP |= (1 << 17); //Enable clock
	LPC_SYSCTL->PCLKSEL[1] |= (1 << 30); //PCLKMPWM = CCLK
	LPC_SYSCTL->PCLKSEL[0] |= (1 << 24);

	Chip_PWM_Init(LPC_PWM1);
	LPC_PWM1->PR = 0;
	Chip_PWM_SetMatch(LPC_PWM1, 0, 6000);
	Chip_PWM_SetMatch(LPC_PWM1, 1, 0);
	Chip_PWM_SetMatch(LPC_PWM1, 2, 200);

	Chip_PWM_ResetOnMatchEnable(LPC_PWM1, 0);
	Chip_PWM_SetCountClockSrc(LPC_PWM1, PWM_CAPSRC_RISING_PCLK, 0);
	Chip_PWM_SetControlMode(LPC_PWM1, 0, PWM_SINGLE_EDGE_CONTROL_MODE, PWM_OUT_ENABLED);
	Chip_PWM_SetControlMode(LPC_PWM1, 1, PWM_SINGLE_EDGE_CONTROL_MODE, PWM_OUT_ENABLED);

	Chip_PWM_LatchEnable(LPC_PWM1, 0, PWM_OUT_ENABLED);
	Chip_PWM_LatchEnable(LPC_PWM1, 1, PWM_OUT_ENABLED);
	Chip_PWM_LatchEnable(LPC_PWM1, 2, PWM_OUT_ENABLED);

	Chip_PWM_Enable(LPC_PWM1);
	Chip_PWM_Reset(LPC_PWM1);

	Chip_GPIO_Init(LPC_GPIO);

	LPC_MCPWM->CON_SET |= (1 <<3);
	DCACSetFreq(600);
	LPC_MCPWM->DT = 12;
	LPC_MCPWM->INTEN_SET |= 1;
	LPC_MCPWM->INTF_SET |= 1;
	freq = 800;
	NVIC_EnableIRQ(RITIMER_IRQn);
	LPC_MCPWM->CON_SET |= 1;



	Chip_ADC_Init(LPC_ADC, &ADCSetup);
	Chip_ADC_SetBurstCmd(LPC_ADC, DISABLE);

	/* Configure RIT for a 1s interrupt tick rate */
	Chip_RIT_SetTimerInterval(LPC_RITIMER, TIME_INTERVAL);


	/* LED is toggled in interrupt handler */
	vout = 0;
	while (1) {
		if(controlFlag) {
			bool emergency = Chip_GPIO_GetPinState(LPC_GPIO,2,13);
			if(emergency) {
				enableOut = false;
				vout = 0;
			} else {
			#ifdef enableLoad
				enableOut = !Chip_GPIO_GetPinState(LPC_GPIO,0,28);
			#else
				enableOut = true;
			#endif
			}
			Board_LED_Set(0, enableOut);
			DCDCControl();
			DCACControl();
			Vmeasure += readADC(VIN_PIN);
			Imeasure += readADC(CURRENT_PIN);
			times++;
			if(times >= delayFactor) {
				DEBUGOUT("%d %d %d %d\n",readADC(VIN_PIN), readADC(VOUT_PIN), readADC(CURRENT_PIN), vout);
				times = 0;
				cycles++;
				if(cycles < ncycles) {
					#ifdef enableMPPT
						MPPT(Vmeasure/delayFactor, Imeasure/delayFactor);
					#endif
					Vmeasure = 0;
					Imeasure = 0;
				} else if(cycles < 2*ncycles) {
					#ifdef enableFreq
						MPPTFreq(Vmeasure/delayFactor, Imeasure/delayFactor);
					#endif
					Vmeasure = 0;
					Imeasure = 0;
				} else {
					cycles = 0;
				}
			}
			if(enablePrev != enableOut) {
				DEBUGOUT("TOGGLING %d\n",enableOut);
			}
			enablePrev = enableOut;
			controlFlag = false;
			if(emergency) return 0;
		}
	}
}

/**
 * @}
 */
