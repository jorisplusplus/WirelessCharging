#include "board.h"

/*****************************************************************************
 * Private types/enumerations/variables
 ****************************************************************************/

/*****************************************************************************
 * Public types/enumerations/variables
 ****************************************************************************/

/*****************************************************************************
 * Private functions
 ****************************************************************************/

#define TIME_INTERVAL   (100)
static volatile bool On;

/*****************************************************************************
 * Public functions
 ****************************************************************************/

/**
 * @brief	RIT interrupt handler
 * @return	Nothing
 */
void RIT_IRQHandler(void)
{
	/* Clearn interrupt */
	Chip_RIT_ClearInt(LPC_RITIMER);

	/* Toggle LED */
	On = (bool) !On;
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
	DEBUGINIT();
	/* Initialize RITimer */
	Chip_RIT_Init(LPC_RITIMER);

	LPC_IOCON->PINSEL[4] = 0x00000555;
	LPC_SYSCTL->PCLKSEL[0] |= (1 << 12); //PCLK_PWM1 = CCLK


	Chip_PWM_Init(LPC_PWM1);
	Chip_PWM_SetMatch(LPC_PWM1, 0, 6000);
	Chip_PWM_SetMatch(LPC_PWM1, 1, 500);
	Chip_PWM_SetMatch(LPC_PWM1, 2, 250);
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


	/* Configure RIT for a 1s interrupt tick rate */
	Chip_RIT_SetTimerInterval(LPC_RITIMER, TIME_INTERVAL);

	NVIC_EnableIRQ(RITIMER_IRQn);




	/* LED is toggled in interrupt handler */
	while (1) {
		int p = DEBUGIN();
			DEBUGOUT("%d\n",p);
	}
}

/**
 * @}
 */
