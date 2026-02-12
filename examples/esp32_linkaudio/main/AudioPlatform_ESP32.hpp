/* Copyright 2025, Ableton AG, Berlin. All rights reserved.
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 *  If you would like to incorporate Link into a proprietary software application,
 *  please contact <link-devs@ableton.com>.
 */

#pragma once

#include "AudioEngine.hpp"
#include <ableton/link/HostTimeFilter.hpp>
#include <driver/i2s_std.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "bsp/esp-box-3.h"

namespace ableton
{
namespace linkaudio
{

template <typename Link>
class AudioPlatform
{
public:
  AudioPlatform(Link& link);
  ~AudioPlatform();

  AudioEngine<Link> mEngine;

private:
  static void audioTask(void* userData);

  void initialize();
  void uninitialize();
  void start();
  void stop();

  link::HostTimeFilter<typename Link::Clock> mHostTimeFilter;
  double mSampleTime;
  i2s_chan_handle_t mTxHandle;
  i2s_chan_handle_t mRxHandle;
  esp_codec_dev_handle_t mSpkHandle;
  esp_codec_dev_handle_t mMicHandle;
  TaskHandle_t mTaskHandle;
  size_t mTestBeepFramesRemaining;
  bool mRunning;
};

} // namespace linkaudio
} // namespace ableton

#include "AudioPlatform_ESP32.ipp"
