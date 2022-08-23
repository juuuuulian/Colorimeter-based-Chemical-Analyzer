/*
 * adc_v0.h
 *
 *  Created on: Apr 14, 2022
 *      Author: julian
 */

#ifndef ADC_V0_H_
#define ADC_V0_H_

#define ADC_CTL_DITHER          0x00000040

void initAdc0Ss3();
void setAdc0Ss3Log2AverageCount(uint8_t);
void setAdc0Ss3Mux(uint8_t);
int16_t readAdc0Ss3();


#endif /* ADC_V0_H_ */
