#include "board.h"

#define TIME_INTERVAL   (500)
static volatile bool On;
static ADC_CLOCK_SETUP_T ADCSetup;

static uint16_t readADC(void)
{
	uint16_t dataADC;
    Chip_ADC_SetStartMode(LPC_ADC, ADC_START_NOW, ADC_TRIGGERMODE_RISING);

	/* Waiting for A/D conversion complete */
	while (Chip_ADC_ReadStatus(LPC_ADC, ADC_CH0, ADC_DR_DONE_STAT) != SET) {}
	/* Read ADC value */
	Chip_ADC_ReadValue(LPC_ADC, ADC_CH0, &dataADC);
	DEBUGOUT("test %d\n",dataADC);
	return dataADC;
}

void RIT_IRQHandler(void)
{
	/* Clearn interrupt */
	Chip_RIT_ClearInt(LPC_RITIMER);

	/* Toggle LED */
	On = (readADC() > 1000);
	Board_LED_Set(0, On);
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

		/* 432MHz / (3+1) = 120MHz */
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
int main(void)
{
	SystemCoreClockUpdate();
	Board_Init();
	setupClock();
	SystemCoreClockUpdate();
	On = true;
	Board_LED_Set(0, On);
	DEBUGOUT("test\n");
	/* Initialize RITimer */
	Chip_RIT_Init(LPC_RITIMER);

	LPC_IOCON->PINSEL[4] |= 0x00000555;
	LPC_IOCON->PINSEL[1] |= (1 << 14);
	LPC_SYSCTL->PCLKSEL[0] |= (1 << 12); //PCLK_PWM1 = CCLK
	//LPC_SYSCTL->PCLKSEL[0] |= (2 << 24); //PCLK_ADC = CCLK/2

	Chip_PWM_Init(LPC_PWM1);
	Chip_PWM_SetMatch(LPC_PWM1, 0, 100000);
	Chip_PWM_SetMatch(LPC_PWM1, 1, 50000);
	Chip_PWM_SetMatch(LPC_PWM1, 2, 25000);
	Chip_PWM_SetMatch(LPC_PWM1, 3, 10000);
	Chip_PWM_SetMatch(LPC_PWM1, 4, 5000);
	Chip_PWM_SetMatch(LPC_PWM1, 5, 2500);
	Chip_PWM_SetMatch(LPC_PWM1, 6, 1250);
	Chip_PWM_ResetOnMatchEnable(LPC_PWM1, 0);
	Chip_PWM_SetCountClockSrc(LPC_PWM1, PWM_CAPSRC_RISING_PCLK, 0);
	Chip_PWM_SetControlMode(LPC_PWM1, 0, PWM_SINGLE_EDGE_CONTROL_MODE, PWM_OUT_ENABLED);
	Chip_PWM_SetControlMode(LPC_PWM1, 1, PWM_SINGLE_EDGE_CONTROL_MODE, PWM_OUT_ENABLED);
	Chip_PWM_SetControlMode(LPC_PWM1, 2, PWM_SINGLE_EDGE_CONTROL_MODE, PWM_OUT_ENABLED);
	Chip_PWM_SetControlMode(LPC_PWM1, 3, PWM_SINGLE_EDGE_CONTROL_MODE, PWM_OUT_ENABLED);
	Chip_PWM_SetControlMode(LPC_PWM1, 4, PWM_SINGLE_EDGE_CONTROL_MODE, PWM_OUT_ENABLED);
	Chip_PWM_SetControlMode(LPC_PWM1, 5, PWM_SINGLE_EDGE_CONTROL_MODE, PWM_OUT_ENABLED);
	Chip_PWM_LatchEnable(LPC_PWM1, 0, PWM_OUT_ENABLED);
	Chip_PWM_LatchEnable(LPC_PWM1, 1, PWM_OUT_ENABLED);
	Chip_PWM_LatchEnable(LPC_PWM1, 2, PWM_OUT_ENABLED);
	Chip_PWM_LatchEnable(LPC_PWM1, 3, PWM_OUT_ENABLED);
	Chip_PWM_LatchEnable(LPC_PWM1, 4, PWM_OUT_ENABLED);
	Chip_PWM_LatchEnable(LPC_PWM1, 5, PWM_OUT_ENABLED);
	Chip_PWM_Reset(LPC_PWM1);
	Chip_PWM_Enable(LPC_PWM1);

	Chip_ADC_Init(LPC_ADC, &ADCSetup);
	Chip_ADC_EnableChannel(LPC_ADC, ADC_CH0, ENABLE);
	Chip_ADC_SetBurstCmd(LPC_ADC, DISABLE);

	/* Configure RIT for a 1s interrupt tick rate */
	Chip_RIT_SetTimerInterval(LPC_RITIMER, TIME_INTERVAL);

	NVIC_EnableIRQ(RITIMER_IRQn);




	/* LED is toggled in interrupt handler */
	while (1) {}
}

/**
 * @}
 */
