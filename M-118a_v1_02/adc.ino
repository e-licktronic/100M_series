// Copyright 2009 Emilie Gillet.
//
// Author: Emilie Gillet (emilie.o.gillet@gmail.com)
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.
//
// -----------------------------------------------------------------------------
//
// Interface to the onboard ADC converter, and analog multiplexer.

#include "adc.h"

namespace avrlib {

/* static */
uint8_t Adc::admux_value_ = ADC_DEFAULT << 6;

/* static */
uint8_t AdcInputScanner::current_pin_;

/* static */
uint8_t AdcInputScanner::num_inputs_;

/* static */
int16_t AdcInputScanner::state_[8];

}  // namespace avrlib
