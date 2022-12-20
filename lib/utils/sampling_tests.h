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
#ifndef SAMPLING_TESTS_H
#define SAMPLING_TESTS_H

#include <Arduino.h>
#include <stdint.h>
#include <stdbool.h>

#include <unity.h>

#include <sampling.hpp>
#ifdef UNITY_CONFIG_HW_TEST
#include <hardware.hpp>
#endif

void test_simulated_square_wave_60() {
    ADC_Channel chA(0,1);
    uint16_t i;
    for (i = 0; i < SAMPLES_PER_50HZ_CYCLE; i++) {
        int32_t diff;
        diff = chA.squareWaveSource(i,SAMPLES_PER_60HZ_CYCLE,1000);
        chA.add_sample(diff);
    }
    uint16_t rms = chA.getRMS_RAW();
    uint16_t numSamples = chA.getNumCycleSamples();
    TEST_PRINTF("RMS: %d numSamples %d 60hz %d 50hz %d", rms, numSamples,chA.get_rms_assume_60hz(), chA.get_rms_assume_50hz());
    TEST_ASSERT_INT_WITHIN(0, 1000, rms);
    TEST_ASSERT_INT_WITHIN(0, SAMPLES_PER_60HZ_CYCLE, numSamples);

}

void test_simulated_square_wave_50() {
    ADC_Channel chA(0,1);
    uint16_t i;
    for ( i = 0; i < SAMPLES_PER_50HZ_CYCLE; i++) {
        int32_t diff;
        diff = chA.squareWaveSource(i,SAMPLES_PER_50HZ_CYCLE,1000);
        chA.add_sample(diff);
    }
    uint16_t rms = chA.getRMS_RAW();
    uint16_t numSamples = chA.getNumCycleSamples();
    TEST_PRINTF("RMS: %d numSamples %d 60hz %d 50hz %d", rms, numSamples,chA.get_rms_assume_60hz(), chA.get_rms_assume_50hz());
    TEST_ASSERT_INT_WITHIN(0, 1000, rms);
    TEST_ASSERT_INT_WITHIN(0, SAMPLES_PER_50HZ_CYCLE, numSamples);


}

void test_simulated_dirty_sine_wave_65() {
    ADC_Channel chA(0,1);
    const uint16_t PERIOD = SAMPLES_PER_60HZ_CYCLE*60/65;
    uint16_t i;
    for ( i = 0; i < SAMPLES_PER_50HZ_CYCLE; i++) {
        int32_t diff;
        diff = chA.sineWaveSource(i,PERIOD,1000);
        chA.add_sample(diff);
    }
    uint16_t rms = chA.getRMS_RAW();
    uint16_t numSamples = chA.getNumCycleSamples();
    TEST_PRINTF("RMS: %d numSamples %d 60hz %d 50hz %d", rms, numSamples,chA.get_rms_assume_60hz(), chA.get_rms_assume_50hz());
    TEST_ASSERT_INT_WITHIN(23, 707, rms);   // NOT ACCURATE AT 65 HZ - 3% error
    TEST_ASSERT_INT_WITHIN(0, SAMPLES_PER_60HZ_CYCLE, numSamples);

}

void test_simulated_dirty_sine_wave_60() {
    ADC_Channel chA(0,1);
    uint16_t i;
    for ( i = 0; i < SAMPLES_PER_50HZ_CYCLE; i++) {
        int32_t diff;
        diff = chA.sineWaveSource(i,SAMPLES_PER_60HZ_CYCLE,1000);
        chA.add_sample(diff);
    }
    uint16_t rms = chA.getRMS_RAW();
    uint16_t numSamples = chA.getNumCycleSamples();
    TEST_PRINTF("RMS: %d numSamples %d 60hz %d 50hz %d", rms, numSamples,chA.get_rms_assume_60hz(), chA.get_rms_assume_50hz());
    TEST_ASSERT_INT_WITHIN(0, 707, rms);
    TEST_ASSERT_INT_WITHIN(0, SAMPLES_PER_60HZ_CYCLE, numSamples);

}

void test_simulated_dirty_sine_wave_50() {
    ADC_Channel chA(0,1);
    uint16_t i;
    for ( i = 0; i < SAMPLES_PER_50HZ_CYCLE; i++) {
        int32_t diff;
        diff = chA.sineWaveSource(i,SAMPLES_PER_50HZ_CYCLE,1000);
        chA.add_sample(diff);
    }
    uint16_t rms = chA.getRMS_RAW();
    uint16_t numSamples = chA.getNumCycleSamples();
    TEST_PRINTF("RMS: %d numSamples %d 60hz %d 50hz %d", rms, numSamples,chA.get_rms_assume_60hz(), chA.get_rms_assume_50hz());
    TEST_ASSERT_INT_WITHIN(0, 707, rms);
    TEST_ASSERT_INT_WITHIN(0, SAMPLES_PER_50HZ_CYCLE, numSamples);

}

void test_simulated_dirty_sine_wave_45() {
    ADC_Channel chA(0,1);
    const uint16_t PERIOD = SAMPLES_PER_60HZ_CYCLE*60/45;
    uint16_t i;
    for ( i = 0; i < (SAMPLES_PER_50HZ_CYCLE); i++) {
        int32_t diff;
        diff = chA.sineWaveSource(i,PERIOD,1000);
        chA.add_sample(diff);
    }
    uint16_t rms = chA.getRMS_RAW();
    uint16_t numSamples = chA.getNumCycleSamples();
    TEST_PRINTF("RMS: %d numSamples %d 60hz %d 50hz %d", rms, numSamples,chA.get_rms_assume_60hz(), chA.get_rms_assume_50hz());
    TEST_ASSERT_INT_WITHIN(29, 707, rms);  // NOT ACCURATE AT 45 HZ - off by 4%
    TEST_ASSERT_INT_WITHIN(0, SAMPLES_PER_50HZ_CYCLE, numSamples);


}


void test_simulated_zero() {
    ADC_Channel chA(0,1);
    uint16_t i;
    for ( i = 0; i < SAMPLES_PER_50HZ_CYCLE; i++) {
        int32_t diff;
        diff = 0;
        chA.add_sample(diff);
        //TEST_PRINTF("%d,%d, %d", i, diff, chA.inCycle());
    }
    uint16_t rms = chA.getRMS_RAW();
    uint16_t numSamples = chA.getNumCycleSamples();
    TEST_PRINTF("RMS: %d numSamples %d 60hz %d 50hz %d", rms, numSamples,chA.get_rms_assume_60hz(), chA.get_rms_assume_50hz());
    TEST_ASSERT_INT_WITHIN(0, 0, rms);
    TEST_ASSERT_INT_WITHIN(0, SAMPLES_PER_60HZ_CYCLE, numSamples);  // returns shorter 60Hz cycle

}

void test_simulated_constant321() {
    ADC_Channel chA(0,1);
    uint16_t i;
    for ( i = 0; i < SAMPLES_PER_50HZ_CYCLE; i++) {
        int32_t diff;
        diff = 321;
        chA.add_sample(diff);
        //TEST_PRINTF("%d,%d, %d", i, diff, chA.inCycle());
    }
     uint16_t rms = chA.getRMS_RAW();
    uint16_t numSamples = chA.getNumCycleSamples();
    TEST_PRINTF("RMS: %d numSamples %d 60hz %d 50hz %d", rms, numSamples,chA.get_rms_assume_60hz(), chA.get_rms_assume_50hz());
    TEST_ASSERT_INT_WITHIN(0, 321, rms);
    TEST_ASSERT_INT_WITHIN(0, SAMPLES_PER_60HZ_CYCLE, numSamples); // returns shorter 60Hz cycle

}

void test_simulated_random_noise() {
    ADC_Channel chA(0,1);
    uint16_t i;
    for ( i = 0; i < SAMPLES_PER_50HZ_CYCLE; i++) {
        int32_t diff;
        diff = rand() % 4096 - 2048;
        chA.add_sample(diff);
        //TEST_PRINTF("%d,%d, %d", i, diff, chA.inCycle());
        // DO ALL TOTAL_SAMPLES, don't break out after 1.5 cycles.
    }
    uint16_t rms = chA.getRMS_RAW();
    uint16_t numSamples = chA.getNumCycleSamples();
    TEST_PRINTF("RMS: %d numSamples %d 60hz %d 50hz %d", rms, numSamples,chA.get_rms_assume_60hz(), chA.get_rms_assume_50hz());
    TEST_ASSERT_INT_WITHIN(SAMPLES_PER_50HZ_CYCLE-SAMPLES_PER_60HZ_CYCLE, SAMPLES_PER_60HZ_CYCLE, numSamples);  // returns shorter 60Hz cycle


}

void test_sampling_does_not_overflow() {
    uint16_t max_iter = MAX_SAMPLES; // fails at SAMPLES_PER_60HZ_CYCLE*3 overflow 
    ADC_Channel chA(0,1);
    uint16_t i = 0;
    for (i = 0; i < max_iter; i++) {
      chA.add_sample(2048);
    }
    TEST_ASSERT_TRUE(chA.is_overflow() == 0); // not detecting overflow now
}

void test_adc_range() {

    TEST_PRINTF("CALCS_FACTOR: %f, MAX_PEAK_CURRENT: %f, MAX_RMS_CURRENT: %f", CALCS_FACTOR,CALCS_PEAK_CURRENT,CALCS_MAX_RMS_CURRENT);
    TEST_ASSERT_TRUE(true);
}

#ifdef UNITY_CONFIG_HW_TEST
void test_hw_analogRead_loop_timing() {
    ADC_Channel chA(PIN_PA4,PIN_PA7);  // one cycle
    ADC_Channel chB(PIN_PA5,PIN_PA7);  // one cycle
    ADC_Channel chC(PIN_PA6,PIN_PA7);  // one cycle   

    uint32_t t1 = millis();
    uint32_t diff = 0;
    uint16_t i;
    for ( i = 0; i < 1000; i++) {
        diff = analogReadDiff(chA._pinP,chA._pinN,12); chA.add_sample(diff);
        diff = analogReadDiff(chB._pinP,chB._pinN,12); chB.add_sample(diff);
        diff = analogReadDiff(chC._pinP,chC._pinN,12); chC.add_sample(diff);
    }
    uint32_t t2 = millis();
    TEST_PRINTF("3 channels: %dus", t2 - t1);
    TEST_PRINTF("ADC0.CTRLA: %d", ADC0.CTRLA);  
    TEST_PRINTF("ADC0.PGACTRL: %d", ADC0.PGACTRL);
    TEST_ASSERT_INT_WITHIN(1, 74, t2-t1);  //TEST_ASSERT_INT_WITHIN(delta, expected, actual)
}

void test_hw_analogRead_loop_timing_fast() {

    ADC_init();
    ADC_Channel chA(PIN_PA4,PIN_PA7);  // one cycle
    ADC_Channel chB(PIN_PA5,PIN_PA7);  // one cycle
    ADC_Channel chC(PIN_PA6,PIN_PA7);  // one cycle   

    uint32_t t1 = millis();

    // 10,000 iterations, don't break the loop, channels A, B, C
    hardware_fast_sample_chABC(10000,chA,chB,chC);

    uint32_t t2 = millis();
    TEST_PRINTF("fast: %fus", (t2 - t1)/10.0);
    TEST_PRINTF("ADC0.CTRLA: %X", ADC0.CTRLA);  
    TEST_PRINTF("ADC0.CTRLB: %X", ADC0.CTRLB);  
    TEST_PRINTF("ADC0.CTRLC: %X", ADC0.CTRLC);  
    TEST_PRINTF("ADC0.CTRLD: %X", ADC0.CTRLD);  
    TEST_PRINTF("ADC0.CTRLE: %X", ADC0.CTRLE);  
    TEST_PRINTF("ADC0.CTRLF: %X", ADC0.CTRLF);  
    TEST_PRINTF("ADC0.COMMAND: %X", ADC0.COMMAND);
    TEST_PRINTF("ADC0.MUXPOS: %X", ADC0.MUXPOS);
    TEST_PRINTF("ADC0.MUXNEG: %X", ADC0.MUXNEG); 
    TEST_PRINTF("(0x1 << 7) | (0x1 << 4) | 0x2: %X", (0x1 << 7) | (0x1 << 4) | 0x2);

    TEST_PRINTF("max A raw: %d", chA._max);
    TEST_PRINTF("max B raw: %d", chB._max);
    TEST_PRINTF("max C raw: %d", chC._max);
    TEST_PRINTF("CHA RMS: %d", chA.getRMS_RAW());
    TEST_PRINTF("CHB RMS: %d", chB.getRMS_RAW());
    TEST_PRINTF("CHC RMS: %d", chC.getRMS_RAW());
    TEST_PRINTF("A Total Samples: %d", chA.getNumTotalSamples());
    TEST_ASSERT_TRUE(chA.getNumTotalSamples() == chB.getNumTotalSamples() && chB.getNumTotalSamples() == chC.getNumTotalSamples());
    float samples_per_60HZ_cycle = 10000000.0/((t2-t1)*60);
    TEST_PRINTF("samples_per_60HZ_cycle: %f", samples_per_60HZ_cycle);
    TEST_ASSERT_INT_WITHIN(3, 394, t2-t1);  //  34.6us +- 1 us   TEST_ASSERT_INT_WITHIN(delta, expected, actual)
    TEST_ASSERT_FLOAT_WITHIN(3,SAMPLES_PER_60HZ_CYCLE, samples_per_60HZ_cycle);  //  477 samples per 60Hz cycle
}




void test_analogRead_loop_timing_slow() {

    ADC_init();
    ADC_Channel chA(PIN_PA4,PIN_PA7);  // one cycle
    ADC_Channel chB(PIN_PA5,PIN_PA7);  // one cycle
    ADC_Channel chC(PIN_PA6,PIN_PA7);  // one cycle   

    uint32_t t1 = millis();

    // 10,000 iterations, don't break the loop, channels A, B, C
    sample_slow_chABC(10000,chA,chB,chC);

    uint32_t t2 = millis();
    TEST_PRINTF("slow: %fus", (t2 - t1)/10.0);
    TEST_PRINTF("max A raw: %d", chA._max);
    TEST_PRINTF("max B raw: %d", chB._max);
    TEST_PRINTF("max C raw: %d", chC._max);
    TEST_PRINTF("CHA RMS: %d", chA.getRMS_RAW());
    TEST_PRINTF("CHB RMS: %d", chB.getRMS_RAW());
    TEST_PRINTF("CHC RMS: %d", chC.getRMS_RAW());
    TEST_PRINTF("A Total Samples: %d", chA.getNumTotalSamples());
    TEST_ASSERT_TRUE(chA.getNumTotalSamples() == chB.getNumTotalSamples() && chB.getNumTotalSamples() == chC.getNumTotalSamples());
    float samples_per_60HZ_cycle = 10000000.0/((t2-t1)*60);
    TEST_PRINTF("samples_per_60HZ_cycle: %f", samples_per_60HZ_cycle);
    TEST_ASSERT_INT_WITHIN(3, 742, t2-t1);  // TEST_ASSERT_INT_WITHIN(delta, expected, actual)
    TEST_ASSERT_FLOAT_WITHIN(2,225, samples_per_60HZ_cycle);  // samples per 60Hz cycle
}
#endif




#endif // SAMPLING_TESTS_H