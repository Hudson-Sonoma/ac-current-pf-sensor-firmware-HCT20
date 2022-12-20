/* 
  Copyright (c) 2022 Hudson Sonoma LLC
  tim@hudson-sonoma.com
  AC Current sense for the Microchip ATtiny 2-series microcontroller 
  
  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program.  If not, see <https://www.gnu.org/licenses/>.
*/
#ifndef HARDWARE_H
#define HARDWARE_H

#include <Arduino.h>
#include <stdint.h>
#include <stdbool.h>
#include <sampling.hpp>

#ifdef MILLIS_USE_TIMERA0
  #error "This sketch takes over TCA0 - please use a different timer for millis"
#endif

void RTC_init(void);
void ADC_init();
void TCA0_init(unsigned long freqInHz);
void TCA0_setFrequency(unsigned long freqInHz);
void printResetReason();
uint16_t readSupplyVoltage();


void Hardware_Init()  {
    RTC_init();    // trigger RTC_PIT_vect once per second in SLEEP_MODE_STANDBY and SLEEP_MODE_PWR_SAVE
    ADC_init();  // configure to use 1.024V reference
    //TCA0_init(SAMPLE_FREQ);   // configure TCA0 to trigger 481*60 times per second in AWAKE AND SLEEP_MODE_STANDBY
}

void RTC_init(void)
{
  /* Initialize RTC: */
  while (RTC.STATUS > 0)
  {
    ;                                   /* Wait for all register to be synchronized */
  }
  RTC.CTRLA |= RTC_RUNSTDBY_bm;            /* Run RTC in RUNSTANDBY */
  RTC.CLKSEL = RTC_CLKSEL_INT32K_gc;    /* 32.768kHz Internal Ultra-Low-Power Oscillator (OSCULP32K) */

  RTC.PITINTCTRL = RTC_PI_bm;           /* PIT Interrupt: enabled */

  RTC.PITCTRLA = RTC_PERIOD_CYC32768_gc /*  RTC_PERIOD_CYC8192_gc=4Hz*/
  | RTC_PITEN_bm;                       /* Enable PIT counter: enabled */
 /* RTC_PERIOD_CYC32768_gc= 1Hz   RTC_PERIOD_CYC4096_gc=8HZ   RTC_PERIOD_CYC1024_gc=32HZ*/
}

void ADC_init() 
{
    analogReference(INTERNAL2V048); // centered around Vcc/2 stabilized by op amp
    analogClockSpeed(3000,1); //    6Mhz ADC
    //ADC0.CTRLA &= ~ADC_LOWLAT_bm; // turn off low latency mode  140->320 at 8Mhz.  244->420 at 4Mhz.
}

void TCA0_init(unsigned long freqInHz) 
{
  takeOverTCA0();
  TCA0_setFrequency(freqInHz);
  TCA0.SINGLE.CTRLA |= TCA_SINGLE_RUNSTDBY_bm; // enable TCA0 in SLEEP_MODE_STANDBY mode
  TCA0.SINGLE.CTRLB = (TCA_SINGLE_CMP0EN_bm | TCA_SINGLE_WGMODE_SINGLESLOPE_gc); // Single slope PWM mode OVF interrupt at BOTTOM, PWM on WO0
  TCA0.SINGLE.INTCTRL = TCA_SINGLE_OVF_bm; // enable overflow interrupt
  //TCA0.SINGLE.CMP0 = 0; // set compare value to 0
}

void TCA0_setFrequency(unsigned long freqInHz) 
{
  unsigned long tempperiod = (F_CPU / freqInHz);
  byte presc = 0;
  while (tempperiod > 65536 && presc < 7) {
    presc++;
    tempperiod      = tempperiod >> (presc > 4 ? 2 : 1);
  }
  uint16_t Period   = tempperiod;
  Serial.printf("TCA0_setFrequency: %lu Hz, prescaler %d, period %u\r\n", freqInHz, presc, Period);
  // TCA0_setFrequency: 4 Hz, prescaler 2, period 62500
  TCA0.SINGLE.CTRLA = (presc << 1) | TCA_SINGLE_ENABLE_bm;
  TCA0.SINGLE.PER   = Period;
}

void printResetReason() 
{
  uint8_t reset_flags = GPIOR0;
  if (reset_flags & RSTCTRL_UPDIRF_bm) {
    Serial.println("Reset by UPDI (code just upoloaded now)");
  }
  if (reset_flags & RSTCTRL_WDRF_bm) {
    Serial.println("reset by WDT timeout");
  }
  if (reset_flags & RSTCTRL_SWRF_bm) {
    Serial.println("reset at request of user code. OR CRASH!");
  }
  if (reset_flags & RSTCTRL_EXTRF_bm) {
    Serial.println("Reset because reset pin brought low");
  }
  if (reset_flags & RSTCTRL_BORF_bm) {
    Serial.println("Reset by voltage brownout");
  }
  if (reset_flags & RSTCTRL_PORF_bm) {
    Serial.println("Reset by power on");
  }
}

// attiny 2-series supply voltage measurement
// https://www.microchip.com/forums/m5200/m5200-attiny-2-series-supply-voltage-measurement.html
// https://github.com/SpenceKonde/megaTinyCore/tree/master/megaavr/libraries/megaTinyCore/examples/readTempVcc
uint16_t readSupplyVoltage() { // returns value in millivolts to avoid floating point
  #if MEGATINYCORE_SERIES!=2
  analogReference(VDD);
  VREF.CTRLA = VREF_ADC0REFSEL_1V5_gc;
  // there is a settling time between when reference is turned on, and when it becomes valid.
  // since the reference is normally turned on only when it is requested, this virtually guarantees
  // that the first reading will be garbage; subsequent readings taken immediately after will be fine.
  // VREF.CTRLB|=VREF_ADC0REFEN_bm;
  // delay(10);
  uint16_t reading = analogRead(ADC_INTREF);
  Serial.print(reading);
  Serial.println(" (discarded)");
  reading = analogRead(ADC_INTREF);
  Serial.println(reading);
  uint32_t intermediate = 1023UL * 1500;
  reading = intermediate / reading;
  return reading;
  #else
  analogReference(INTERNAL1V024);
  int32_t vddmeasure = analogReadEnh(ADC_VDDDIV10, 12); // Take it at 12 bits
  vddmeasure *= 10; // since we measured 1/10th VDD
  int16_t returnval = vddmeasure >> 2; // divide by 4 to get into millivolts.
  if (vddmeasure & 0x02) {
    // if last two digits were 0b11 or 0b10 we should round up
    returnval++;
  }
  return returnval;
  #endif
}

uint16_t sample_slow_chABC(uint16_t max_iter, ADC_Channel &chA, ADC_Channel &chB, ADC_Channel &chC) {

    uint16_t i = 0;
    for (i = 0; i < max_iter; i++) {
      chA.add_sample(analogReadDiff(chA._pinP, chA._pinN,12));
      chB.add_sample(analogReadDiff(chB._pinP, chB._pinN,12));
      chC.add_sample(analogReadDiff(chC._pinP, chC._pinN,12));

    }
    return i;

}




// attiny 2-series fast sample three channels 12-bit samples
// max_iter iterations
// returns number of iterations completed
uint16_t hardware_fast_sample_chABC(uint16_t max_iter, ADC_Channel &chA, ADC_Channel &chB, ADC_Channel &chC) {

    static_assert(MEGATINYCORE_SERIES==2, "hardware_fast_sample_chABC only works on attiny 2-series");

    uint32_t Araw =0;
    uint32_t Braw =0;
    uint32_t Craw =0;
    uint8_t pinA = digitalPinToAnalogInput(chA._pinP);
    uint8_t pinB = digitalPinToAnalogInput(chB._pinP);
    uint8_t pinC = digitalPinToAnalogInput(chC._pinP);
    uint8_t pinN = digitalPinToAnalogInput(chA._pinN);

    //ADC0.COMMAND = (0x1 << 4) | 0x2; // 12 bit, run on muxpos change
    ADC0.COMMAND = (0x1 << 7) | (0x1 << 4) | 0x2 ; // differential, 12 bit, run on muxpos change
    ADC0.MUXNEG = pinN;
    uint16_t i = 0;
    for (i = 0; i < max_iter; i++) {
        /* Start conversion */
        ADC0.MUXPOS = pinA;
        if (i>0) { chC.add_sample(Craw);}
        /* Wait for result ready */
        while (!(ADC0.INTFLAGS & ADC_RESRDY_bm));
        /* Combine two bytes */
        Araw = ADC0.RESULT;

        /* Start conversion */
        ADC0.MUXPOS = pinB;
        /* Do something usefule while ADC is going */ 
        chA.add_sample(Araw);
        /* Wait for result ready */
        while (!(ADC0.INTFLAGS & ADC_RESRDY_bm));
        /* Combine two bytes */
        Braw = ADC0.RESULT;

        /* Start conversion */
        ADC0.MUXPOS = pinC;
        /* Do something usefule while ADC is going */ 
        chB.add_sample(Braw);
        /* Wait for result ready */
        while (!(ADC0.INTFLAGS & ADC_RESRDY_bm));
        /* Combine two bytes */
        Craw = ADC0.RESULT;

    }
    chC.add_sample(Craw);
    ADC0.COMMAND = 0;
    return i;
}


#endif // HARDWARE_H