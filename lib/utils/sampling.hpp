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
#ifndef SAMPLING_H
#define SAMPLING_H

#include <Arduino.h>
#include <stdint.h>
#include <stdbool.h>

#define UNITY_INCLUDE_PRINT_FORMATTED 1


#define SAMPLES_PER_60HZ_CYCLE 425 //352 // This value determined by running test/test_run_tests.cpp:303 and seeing how long the loop takes.
#define SAMPLES_PER_50HZ_CYCLE SAMPLES_PER_60HZ_CYCLE*60/50
#define SAMPLE_FREQ (SAMPLES_PER_60HZ_CYCLE*60) // Hz
#define MAX_SAMPLES (SAMPLES_PER_50HZ_CYCLE)  // One 50Hz cycle, that contains a 60Hz cycle that will be used if it is balanced.

const float CALCS_PRIMARY_MAX_ARMS = 50.0;
const float CALC_SECONDARY_MAX_ARMS = 0.050; // 50mA
const uint16_t CALCS_I_TURNS = (CALCS_PRIMARY_MAX_ARMS/CALC_SECONDARY_MAX_ARMS); // 100A is 50mA -> 2000 turns
const float CALCS_R_BURDEN = 20.0; // 
const float CALCS_V_MAX = 2.048;
const uint16_t CALCS_RAW_MAX = 2048; // (2^12/2)
const float CALCS_FACTOR = ((CALCS_V_MAX/CALCS_RAW_MAX)/CALCS_R_BURDEN)*CALCS_I_TURNS;
const float CALCS_PEAK_CURRENT = CALCS_RAW_MAX * CALCS_FACTOR;
const float CALCS_MAX_RMS_CURRENT = (CALCS_RAW_MAX * CALCS_FACTOR)*0.707;

class ADC_Channel {
  public:
    // Create channel ready to sample pinP - pinN
    ADC_Channel(uint8_t pinP, uint8_t pinN) {
      this->_pinP = pinP;
      this->_pinN = pinN;
      this->reset();
    }

    void reset() {
      this->_tickNum = 0;
      this->_lastSample = 0;
      this->_sum2_60 = 0;
      this->_sum2_e = 0;
      this->_n_60 = 0;
      this->_n_e = 0;
      this->_overflow = 0;
      this->_max = 0;
      this->_min = 0;
      this->_p = 0;
      this->_n = 0;
      this->_pe = 0;
      this->_ne = 0;
      this->_lastSample = 0;
      this->_crossguard = 3;
      this->_phaseTriggered=0;
      this->_phaseCounter = 0;
      this->_phaseCount=0;

    }

  // call in isr to mark beginning of cycle
  void phase_mark() {
    _phaseCounter = 0;
    _phaseTriggered++;
  }  

    void add_sample(int16_t diff) {
        //_max = (_max > diff) ? _max : diff;
        //_min = (_min < diff) ? _min : diff;
        _crossguard--;
        noInterrupts();
        _phaseCounter++; // set to zero in ISR
        interrupts();
        if(((diff ^ _lastSample) & _crossguard) >> 15) {  // If crossed unambiguously (one but not both samples negative and crossGuard negative)
          _crossguard = 10;  
          if (_phaseTriggered == 1) {  // on first call after phase mark.
            _phaseTriggered++;
            noInterrupts();
            _phaseCount = _phaseCounter;
            interrupts();
            PORTB.OUTSET = PIN3_bm; delayMicroseconds(5); PORTB.OUTCLR = PIN3_bm;
          }
        }
        uint32_t sq =  ((uint32_t)diff*(uint32_t)diff);
        if (_tickNum < SAMPLES_PER_60HZ_CYCLE) {
          _sum2_60 += sq;
          _n_60++;
          if (diff >= 0) { _p++; }
          if (diff < 0) { _n++; }
        } else {
          _sum2_e += sq;
          _n_e++;
          if (diff >= 0) { _pe++; }
          if (diff < 0) { _ne++; }
        }
        //if (_sum2_60 < sq) { _overflow=1;}
        //if (_sum2_e < sq) { _overflow=1;}
        _tickNum++;
        _lastSample = diff;
        return;

    }

    uint16_t get_phase_count() {
      return _phaseCount;
    }

  // void add_sample(int16_t diff) {
  //       _max = (_max > diff) ? _max : diff;
  //       _min = (_min < diff) ? _min : diff;
  //       uint32_t sq =  ((uint32_t)diff*(uint32_t)diff);
  //       if (_tickNum < SAMPLES_PER_60HZ_CYCLE) {
  //         _sum2_60 += sq;
  //         _n_60++;
  //         if (diff >= 0) { _p++; }
  //         if (diff < 0) { _n++; }
  //       } else {
  //         _sum2_e += sq;
  //         _n_e++;
  //         if (diff >= 0) { _pe++; }
  //         if (diff < 0) { _ne++; }
  //       }
  //       //if (_sum2_60 < sq) { _overflow=1;}
  //       //if (_sum2_e < sq) { _overflow=1;}
  //       _tickNum++;
  //       return;

  //   }

    // void setTotalCycles(uint16_t totalSamples) {
    //   this->_totalSamples = totalSamples;
    // }

    bool is_overflow() {
      return _overflow;
    }

    bool is_like_dc() {
      if ( (_p + _pe) == 0 || (_n + _ne) == 0) {
        return true;
      } else {
        return false;
      }
    }

    bool is_more_like_60hz() {
      return (abs(_p-_n) < abs(_p+_pe - _n-_ne));
    }

    bool is_more_like_50hz() {
      return !is_more_like_60hz();
    }

    // Returns the RMS raw values. -2048 to +2047 maps to -2.048V to +2.048V, centered around Vcc/2
    // If like dc (constant), or not like 50hz, we return the shorter 60hz sample
    uint16_t getRMS_RAW() {
        return this->is_more_like_60hz() ? this->get_rms_assume_60hz() : this->get_rms_assume_50hz();

    }

    uint16_t get_rms_assume_60hz() {
        uint16_t numSum = _n_60;
        uint32_t sum2   = _sum2_60;
        if(numSum == 0) {
          return 0; 
        }
        return sqrt(sum2/numSum );  // AC sampling
    }

    uint16_t get_rms_assume_50hz() {
        uint16_t numSum = (_n_60+_n_e);
        uint32_t sum2   = (_sum2_60+_sum2_e);
        if(numSum == 0) {
          return 0; 
        }
        return sqrt(sum2/numSum );  // AC sampling
    }

    uint16_t getNumTotalSamples() {
       //return _totalSamples;
       return _n_60+_n_e;
    }

    uint16_t getNumCycleSamples() {
       //return _totalSamples;
       return this->is_more_like_60hz() ? (_n_60) : (_n_60+_n_e);
    }

    float getRMSAmps() {
      return getRMS_RAW() * CALCS_FACTOR;
    }

    int16_t squareWaveSource(uint16_t sampleNum, uint16_t period, int16_t amplitude) {
       uint16_t idx = sampleNum % period;  
       if (idx < period/2) {
         return amplitude;
       } else {
         return -amplitude;
       } 
    }

    int16_t sineWaveSource(uint16_t sampleNum, uint16_t period, int16_t amplitude) {
        uint16_t idx = sampleNum % period;  
        return amplitude * sin(idx * 2 * PI / period) + amplitude/20.0 * sin((idx+10) * 2 * PI / (period/40.0));
     }


  //private:
    uint8_t _pinP;
    uint8_t _pinN;

    uint16_t _tickNum;
    uint32_t _sum2_60;
    uint32_t _sum2_e;
    uint16_t _n_60;
    uint16_t _n_e;
    uint8_t _overflow;

    int16_t _max;
    int16_t _min;
    int16_t _p;   // number positive samples in a 60Hz cycle
    int16_t _n;   // number negative samples in a 60Hz cycle
    int16_t _pe;  // number positive samples in the rest of the 50Hz cycle
    int16_t _ne;  // number negative samples in the rest of the 50Hz cycle
    int16_t _lastSample;
    int16_t _crossguard=3;
    volatile uint8_t _phaseTriggered=0;
    uint16_t _phaseCounter=0;
    volatile uint16_t _phaseCount=0;

};


#endif  // SAMPLING_H

