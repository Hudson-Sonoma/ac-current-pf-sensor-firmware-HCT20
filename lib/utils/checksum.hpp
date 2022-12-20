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
#ifndef CHECKSUM_H
#define CHECKSUM_H

#include <Arduino.h>
#include <stdint.h>
#include <stdbool.h>

#define CHECKCRC_SHIFTED_DIVISOR                       0x988000


//  AUTHOR: Rob Tillaart, Viktor Balint
//  LICENCE: MIT License
uint8_t sht20_crc8(const uint8_t *data, uint8_t len)
{
  // CRC-8 formula from page 14 of SHT spec pdf
  // Sensirion_Humidity_Sensors_SHT2x_CRC_Calculation.pdf
  const uint8_t POLY = 0x31;
  uint8_t crc = 0x00;

  for (uint8_t j = len; j; --j)
  {
    crc ^= *data++;

    for (uint8_t i = 8; i; --i)
    {
      crc = (crc & 0x80) ? (crc << 1) ^ POLY : (crc << 1);
    }
  }
  return crc;
}

const uint16_t POLYNOMIAL = 0x131; 
// crc of uint8_t array of length n, with polynomial 0x131 (x^8+x^5+x^4+1)
uint8_t sht20_crc8_2(uint8_t *data, uint8_t n)
{
    uint8_t crc = 0;
    for (uint8_t i = 0; i < n; ++i)
    {
        crc = crc ^ data[i];
        for (uint8_t j = 0; j < 8; ++j)
        {
            if ((crc & 0x80) != 0)
            {
                crc = (crc << 1) ^ POLYNOMIAL;
            }
            else
            {
                crc = (crc << 1);
            }
        }
    }
    return crc;
}

#endif // CHECKSUM_H