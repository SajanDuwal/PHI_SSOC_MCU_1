/*
 * adc.c
 *
 *  Created on: Dec 15, 2024
 *      Author: sajanduwal
 */


#include "main.h"
#include "adc.h"

/* Variables for ADC conversion data */
__IO uint16_t uhADCxConvertedData = VAR_CONVERTED_DATA_INIT_VALUE; /* ADC group regular conversion data */

/* Variables for ADC conversion data computation to physical values */
__IO int16_t hADCxConvertedData_Temperature_DegreeCelsius; /* Value of temperature calculated from ADC conversion data (unit: degree Celsius) */

void Activate_ADC(void) {
	__IO uint32_t wait_loop_index = 0U;
	__IO uint32_t backup_setting_adc_dma_transfer = 0U;
#if (USE_TIMEOUT == 1)
	  uint32_t Timeout = 0U; /* Variable used for timeout management */
	  #endif /* USE_TIMEOUT */

	/*## Operation on ADC hierarchical scope: ADC instance #####################*/

	/* Note: Hardware constraint (refer to description of the functions         */
	/*       below):                                                            */
	/*       On this STM32 series, setting of these features is conditioned to  */
	/*       ADC state:                                                         */
	/*       ADC must be disabled.                                              */
	/* Note: In this example, all these checks are not necessary but are        */
	/*       implemented anyway to show the best practice usages                */
	/*       corresponding to reference manual procedure.                       */
	/*       Software can be optimized by removing some of these checks, if     */
	/*       they are not relevant considering previous settings and actions    */
	/*       in user application.                                               */
	if (LL_ADC_IsEnabled(ADC) == 0) {
		/* Enable ADC internal voltage regulator */
		LL_ADC_EnableInternalRegulator(ADC);

		/* Delay for ADC internal voltage regulator stabilization.                */
		/* Compute number of CPU cycles to wait for, from delay in us.            */
		/* Note: Variable divided by 2 to compensate partially                    */
		/*       CPU processing cycles (depends on compilation optimization).     */
		/* Note: If system core clock frequency is below 200kHz, wait time        */
		/*       is only a few CPU processing cycles.                             */
		wait_loop_index = ((LL_ADC_DELAY_INTERNAL_REGUL_STAB_US
				* (SystemCoreClock / (100000 * 2))) / 10);
		while (wait_loop_index != 0) {
			wait_loop_index--;
		}

		/* Disable ADC DMA transfer request during calibration */
		/* Note: Specificity of this STM32 series: Calibration factor is          */
		/*       available in data register and also transferred by DMA.          */
		/*       To not insert ADC calibration factor among ADC conversion data   */
		/*       in DMA destination address, DMA transfer must be disabled during */
		/*       calibration.                                                     */
		backup_setting_adc_dma_transfer = LL_ADC_REG_GetDMATransfer(ADC);
		LL_ADC_REG_SetDMATransfer(ADC, LL_ADC_REG_DMA_TRANSFER_NONE);

		/* Run ADC self calibration */
		LL_ADC_StartCalibration(ADC);

		/* Poll for ADC effectively calibrated */
#if (USE_TIMEOUT == 1)
	    Timeout = ADC_CALIBRATION_TIMEOUT_MS;
	    #endif /* USE_TIMEOUT */

		while (LL_ADC_IsCalibrationOnGoing(ADC) != 0) {
#if (USE_TIMEOUT == 1)
	      /* Check Systick counter flag to decrement the time-out value */
	      if (LL_SYSTICK_IsActiveCounterFlag())
	      {
	        if(Timeout-- == 0)
	        {
	          /* Error: Time-out */
	          Error_Handler();
	        }
	      }
	    #endif /* USE_TIMEOUT */
		}

		/* Restore ADC DMA transfer request after calibration */
		LL_ADC_REG_SetDMATransfer(ADC, backup_setting_adc_dma_transfer);

		/* Delay between ADC end of calibration and ADC enable.                   */
		/* Note: Variable divided by 2 to compensate partially                    */
		/*       CPU processing cycles (depends on compilation optimization).     */
		wait_loop_index = (ADC_DELAY_CALIB_ENABLE_CPU_CYCLES >> 1);
		while (wait_loop_index != 0) {
			wait_loop_index--;
		}

		/* Enable ADC */
		LL_ADC_Enable(ADC);

		/* Poll for ADC ready to convert */
#if (USE_TIMEOUT == 1)
	    Timeout = ADC_ENABLE_TIMEOUT_MS;
	    #endif /* USE_TIMEOUT */

		while (LL_ADC_IsActiveFlag_ADRDY(ADC) == 0) {
#if (USE_TIMEOUT == 1)
	      /* Check Systick counter flag to decrement the time-out value */
	      if (LL_SYSTICK_IsActiveCounterFlag())
	      {
	        if(Timeout-- == 0)
	        {
	          /* Error: Time-out */
	          Error_Handler();
	        }
	      }
	    #endif /* USE_TIMEOUT */
		}

		/* Note: ADC flag ADRDY is not cleared here to be able to check ADC       */
		/*       status afterwards.                                               */
		/*       This flag should be cleared at ADC Deactivation, before a new    */
		/*       ADC activation, using function "LL_ADC_ClearFlag_ADRDY()".       */
	}

}

/**
 * @brief  Perform ADC group regular conversion start, poll for conversion
 *         completion.*/

void ConversionStartPoll_ADC_GrpRegular(void) {
#if (USE_TIMEOUT == 1)
	  uint32_t Timeout = 0U; /* Variable used for timeout management */
	  #endif /* USE_TIMEOUT */

	/* Start ADC group regular conversion */

	if ((LL_ADC_IsEnabled(ADC) == 1) && (LL_ADC_IsDisableOngoing(ADC) == 0)
			&& (LL_ADC_REG_IsConversionOngoing(ADC) == 0)) {
		LL_ADC_REG_StartConversion(ADC);
	} else {
		/* Error: ADC conversion start could not be performed */
		Error_Handler();
	}

#if (USE_TIMEOUT == 1)
	  Timeout = ADC_UNITARY_CONVERSION_TIMEOUT_MS;
	  #endif /* USE_TIMEOUT */

	while (LL_ADC_IsActiveFlag_EOC(ADC) == 0) {
#if (USE_TIMEOUT == 1)
	    /* Check Systick counter flag to decrement the time-out value */
	    if (LL_SYSTICK_IsActiveCounterFlag())
	    {
	      if(Timeout-- == 0)
	      {
	        /* Error: Time-out */
	        Error_Handler();
	      }
	    }
	  #endif /* USE_TIMEOUT */
	}

	/* Clear flag ADC group regular end of unitary conversion */
	LL_ADC_ClearFlag_EOC(ADC);
}

uint16_t ReadTemperature() {
	Activate_ADC();
	/* Init variable containing ADC conversion data */
	uhADCxConvertedData = VAR_CONVERTED_DATA_INIT_VALUE;

	/* Perform ADC group regular conversion start, poll for conversion        */
	/* completion.                                                            */
	ConversionStartPoll_ADC_GrpRegular();

	/* Retrieve ADC conversion data */
	/* (data scale corresponds to ADC resolution: 12 bits) */
	uhADCxConvertedData = LL_ADC_REG_ReadConversionData12(ADC);

	/* Turn LED on after ADC conversion completion */

	/* Computation of ADC conversions raw data to physical values             */
	/* using LL ADC driver helper macro.                                      */
	hADCxConvertedData_Temperature_DegreeCelsius = __LL_ADC_CALC_TEMPERATURE(
			VDDA_APPLI, uhADCxConvertedData, LL_ADC_RESOLUTION_12B);

	return hADCxConvertedData_Temperature_DegreeCelsius;
}
