// Copyright 2012 Emilie Gillet.
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

#ifndef GRIDS_HARDWARE_CONFIG_H_
#define GRIDS_HARDWARE_CONFIG_H_

#include "base.h"
#include "gpio.h"
#include "parallel_io.h"
//#include "serial.h"
#include "spi.h"

namespace grids {

  enum AdcChannel {
    ADC_CHANNEL_X_CV,
    ADC_CHANNEL_Y_CV,
    ADC_CHANNEL_RANDOMNESS_CV,
    ADC_CHANNEL_BD_DENSITY_CV,
    ADC_CHANNEL_SD_DENSITY_CV,
    ADC_CHANNEL_TEMPO,
    ADC_CHANNEL_HH_DENSITY_CV,
    ADC_CHANNEL_LAST
  };

  enum LedBits {
    LED_RED = 1,//RED LED
    LED_GREEN = 8,//GREEN LED
    LED_YELLOW = LED_RED | LED_GREEN
  };

  enum InputBits {
    INPUT_CLOCK = 2,
    INPUT_RESET = 4,
    INPUT_SW_RESET = 8
  };

  using avrlib::Gpio;
  using avrlib::ParallelPort;
  using avrlib::PortB;
  using avrlib::PortD;
  //using avrlib::Serial;
  //using avrlib::SerialPort0;
  using avrlib::SpiMaster;

  typedef ParallelPort<PortD, avrlib::PARALLEL_NIBBLE_HIGH> Leds;
  typedef ParallelPort<PortD, avrlib::PARALLEL_NIBBLE_LOW> Inputs;
  typedef ParallelPort<PortB, avrlib::PARALLEL_BYTE> Outputs;//Initialize Parallel Byte to get the complete byte Cf parallel_io.h
  //typedef SpiMaster<Gpio<PortB, 2>, avrlib::MSB_FIRST, 2> ShiftRegister;
  //typedef Serial<SerialPort0, 115200, avrlib::POLLED, avrlib::DISABLED> MidiInput;
}  // namespace grids

#endif  // GRIDS_HARDWARE_CONFIG_H_
