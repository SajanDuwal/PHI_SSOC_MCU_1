/*
 * adc.h
 *
 *  Created on: Dec 15, 2024
 *      Author: sajanduwal
 */

#ifndef INC_ADC_H_
#define INC_ADC_H_

#define ADC_CALIBRATION_TIMEOUT_MS       (   1U)
#define ADC_ENABLE_TIMEOUT_MS            (   1U)
#define ADC_DISABLE_TIMEOUT_MS           (   1U)
#define ADC_STOP_CONVERSION_TIMEOUT_MS   (   1U)
#define ADC_CONVERSION_TIMEOUT_MS        (4000U)

/* Delay between ADC end of calibration and ADC enable.                     */
/* Delay estimation in CPU cycles: Case of ADC enable done                  */
/* immediately after ADC calibration, ADC clock setting slow                */
/* (LL_ADC_CLOCK_ASYNC_DIV32). Use a higher delay if ratio                  */
/* (CPU clock / ADC clock) is above 32.                                     */
#define ADC_DELAY_CALIB_ENABLE_CPU_CYCLES  (LL_ADC_DELAY_CALIB_ENABLE_ADC_CYCLES * 32)

/* Definitions of environment analog values */
/* Value of analog reference voltage (Vref+), connected to analog voltage   */
/* supply Vdda (unit: mV).                                                  */
#define VDDA_APPLI                       (3300UL)

/* Definitions of data related to this example */
/* ADC unitary conversion timeout */
/* Considering ADC settings, duration of 1 ADC conversion should always    */
/* be lower than 1ms.                                                      */
#define ADC_UNITARY_CONVERSION_TIMEOUT_MS (   1UL)

/* Init variable out of expected ADC conversion data range */
#define VAR_CONVERTED_DATA_INIT_VALUE    (__LL_ADC_DIGITAL_SCALE(LL_ADC_RESOLUTION_12B) + 1)

//################## INTERNAL TEMPERATURE ###############################
void Activate_ADC(void);
void ConversionStartPoll_ADC_GrpRegular(void);
uint16_t ReadTemperature(void);

#endif /* INC_ADC_H_ */
