#include <Arduino.h>
#include <unity.h>

#include <checksum.hpp>
#include <sampling.hpp>
#include <sampling_tests.h>

#ifdef UNITY_CONFIG_HW_TEST
#include "unity_config_hw_test.hpp"
#include <hardware.hpp>

#include "wiring_private.h"
#include "pins_arduino.h"
#include "Arduino.h"
#endif

//#define UNITY_INCLUDE_PRINT_FORMATTED

void setUp(void)
{
//    ArduinoFakeReset();
}

void tearDown(void) {
    // clean stuff up here
}

void test_crc_calc_and_check() {
    uint16_t result = 0;
    for (uint16_t data = 0; data < 65535; data++) {    
        uint8_t crc, crc2;
        crc = sht20_crc8((uint8_t *) &data, 2);
        crc2 = sht20_crc8_2((uint8_t *) &data, 2);
        result += crc != crc2;
    }
    TEST_ASSERT_TRUE(result == 0);
}

void unsigned_math_test() {
    uint8_t a = 0;
    uint8_t b = 255;
    TEST_ASSERT_INT_WITHIN(0,1,(uint8_t)(a-b));
    a = 255;
    b = 254; 
    TEST_ASSERT_INT_WITHIN(0,1,(uint8_t)(a-b));

    int16_t diff = 2048;
    uint32_t sq = ((uint32_t)diff*(uint32_t)diff);
    TEST_ASSERT_INT_WITHIN(0,4194304,sq);

}

void setup() {
#ifdef UNITY_CONFIG_HW_TEST
    //Hardware_Init();
    delay(2000);
    ADC_init(); 
#endif
    UNITY_BEGIN();

    // checksum.hpp
    RUN_TEST(test_crc_calc_and_check);

    RUN_TEST(unsigned_math_test);

    // samnpling.hpp.
    RUN_TEST(test_simulated_square_wave_60);
    RUN_TEST(test_simulated_square_wave_50);
    RUN_TEST(test_simulated_dirty_sine_wave_65);
    RUN_TEST(test_simulated_dirty_sine_wave_60);
    RUN_TEST(test_simulated_dirty_sine_wave_50);
    RUN_TEST(test_simulated_dirty_sine_wave_45);
    RUN_TEST(test_simulated_zero);
    RUN_TEST(test_simulated_constant321);
    RUN_TEST(test_simulated_random_noise);
    RUN_TEST(test_sampling_does_not_overflow);
    RUN_TEST(test_adc_range);
#ifdef UNITY_CONFIG_HW_TEST
    RUN_TEST(test_hw_analogRead_loop_timing);
    RUN_TEST(test_hw_analogRead_loop_timing_fast);
    RUN_TEST(test_analogRead_loop_timing_slow);
#endif  
    UNITY_END();
}

void loop() {}

#ifndef ARDUINO
int main( int argc, char **argv) {
    setup();
}
#endif