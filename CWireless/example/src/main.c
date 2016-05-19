#include "board.h"

#define TIME_INTERVAL   (1)
#define VIN_PIN 0
#define VOUT_PIN 1
#define CURRENT_PIN 2
#define LOAD_PIN 3
#define intFactor 1

#define enableMPPT
#define enableLoad


static volatile bool On;
static ADC_CLOCK_SETUP_T ADCSetup;
static volatile bool enableOut;
static volatile bool enablePrev;
static volatile uint16_t vout;
static volatile uint16_t time;
static int32_t dutyInt;
static volatile bool controlFlag;
static volatile uint16_t times;

static uint16_t readADC(uint8_t id)
{
	uint16_t dataADC;
	Chip_ADC_EnableChannel(LPC_ADC, id, ENABLE);
    Chip_ADC_SetStartMode(LPC_ADC, ADC_START_NOW, ADC_TRIGGERMODE_RISING);

	/* Waiting for A/D conversion complete */
	while (Chip_ADC_ReadStatus(LPC_ADC, id, ADC_DR_DONE_STAT) != SET) {}
	/* Read ADC value */
	Chip_ADC_ReadValue(LPC_ADC, id, &dataADC);
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
	if(enableOut < enablePrev) {
		//Set output low when the output should be disabled.
		Chip_PWM_SetMatch(LPC_PWM1, 1, 0);
		Chip_PWM_LatchEnable(LPC_PWM1, 1, PWM_OUT_ENABLED);
		return;
	} else if(enableOut > enablePrev){
		Chip_PWM_SetMatch(LPC_PWM1, 1, 6000);
		Chip_PWM_LatchEnable(LPC_PWM1, 1, PWM_OUT_ENABLED);
	}
	uint16_t vin = readADC(VIN_PIN);
	uint16_t currentOut = readADC(VOUT_PIN);

	dutyInt += (currentOut-vout)*intFactor; //Integration of the error
	if(dutyInt > 50) dutyInt = 50; //Limit integration
	if(dutyInt < -50) dutyInt = -50; //Limit integration
	int32_t D = (vout-vin)*6000/(vout) + dutyInt;
	if(D > 5500) D = 5500; //Limit duty cycle
	if(D < 0) D = 0; //Minimal duty cycle
	Chip_PWM_SetMatch(LPC_PWM1, 1, D);
	Chip_PWM_LatchEnable(LPC_PWM1, 1, PWM_OUT_ENABLED);
	//DEBUGOUT("%d %d %d\n", D, vin, currentOut);
}

void MPPT(void) { //PUT MPPT here

}

void DCACSetFreq(uint16_t freq) {
	if(freq < 600) {
		freq = 600;
	}
	if(freq > 1800) {
		freq = 1800;
	}
	LPC_MCPWM->LIM[0] = freq;
	LPC_MCPWM->MAT[0] = (freq>>1)-12;
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
	LPC_SYSCTL->PCLKSEL[0] |= (1 << 12); //PCLK_PWM1 = CCLK

	LPC_SYSCTL->PCONP |= (1 << 17); //Enable clock
	LPC_SYSCTL->PCLKSEL[1] |= (1 << 30); //PCLKMPWM = CCLK

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

	LPC_MCPWM->CON_SET |= (1 <<3);
	DCACSetFreq(1200);
	LPC_MCPWM->DT = 12;
	LPC_MCPWM->INTEN_SET |= 1;
	LPC_MCPWM->INTF_SET |= 1;

	NVIC_EnableIRQ(RITIMER_IRQn);
	LPC_MCPWM->CON_SET |= 1;



	Chip_ADC_Init(LPC_ADC, &ADCSetup);
	Chip_ADC_SetBurstCmd(LPC_ADC, DISABLE);

	/* Configure RIT for a 1s interrupt tick rate */
	Chip_RIT_SetTimerInterval(LPC_RITIMER, TIME_INTERVAL);


	/* LED is toggled in interrupt handler */
	vout = 2000;
	while (1) {
		if(controlFlag) {
			#ifndef enableLoad
				enableOut = (readADC(LOAD_PIN) > 2000);
			#else
				enableOut = true;
			#endif
			DCACControl();
			DCDCControl();
			times++;
			if(times > 99) {
				times = 0;
				#ifndef enableMPPT
					MPPT();
				#endif
			}
			enablePrev = enableOut;
			controlFlag = false;
		}
	}
}

/**
 * @}
 */
