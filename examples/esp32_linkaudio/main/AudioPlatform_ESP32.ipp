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

#include <esp_timer.h>
#include <esp_log.h>
#include <esp_err.h>
#include <vector>
#include <chrono>
#include <ableton/util/FloatIntConversion.hpp>

namespace ableton
{
namespace linkaudio
{

static const char* TAG = "AudioPlatform";

template <typename Link>
AudioPlatform<Link>::AudioPlatform(Link& link)
  : mEngine(link)
  , mSampleTime(0.)
  , mTxHandle(nullptr)
  , mRxHandle(nullptr)
  , mSpkHandle(nullptr)
  , mMicHandle(nullptr)
  , mTaskHandle(nullptr)
  , mTestBeepFramesRemaining(88200) // 2 second beep
  , mRunning(false)
{
  mEngine.setSampleRate(44100.);
  mEngine.setNumFrames(256); // Optimized for network buffers
  initialize();
  start();
}

template <typename Link>
AudioPlatform<Link>::~AudioPlatform()
{
  stop();
  uninitialize();
}

template <typename Link>
void AudioPlatform<Link>::audioTask(void* userData)
{
  AudioPlatform& platform = *static_cast<AudioPlatform*>(userData);
  AudioEngine<Link>& engine = platform.mEngine;
  const size_t numFrames = engine.mBuffers[0].size();
  
  std::vector<int16_t> i2s_tx_buffer(numFrames * 2);
  std::vector<int16_t> i2s_rx_buffer(numFrames * 2);
  std::vector<double> input_left(numFrames);
  std::vector<double> input_right(numFrames);

  ESP_LOGI(TAG, "Audio task started");

  while (platform.mRunning)
  {
    if (platform.mTestBeepFramesRemaining > 0)
    {
      for (size_t i = 0; i < numFrames; ++i)
      {
        double val = 0.4 * sin(2.0 * M_PI * 440.0 * platform.mSampleTime / engine.mSampleRate);
        i2s_tx_buffer[i * 2] = util::floatToInt16(val);
        i2s_tx_buffer[i * 2 + 1] = util::floatToInt16(val);
        platform.mSampleTime += 1.0;
      }
      platform.mTestBeepFramesRemaining = (platform.mTestBeepFramesRemaining > numFrames) ? 
                                          (platform.mTestBeepFramesRemaining - numFrames) : 0;
      
      esp_codec_dev_write(platform.mSpkHandle, i2s_tx_buffer.data(), i2s_tx_buffer.size() * sizeof(int16_t));
      continue;
    }

    // Read from microphone
    int res = esp_codec_dev_read(platform.mMicHandle, i2s_rx_buffer.data(), i2s_rx_buffer.size() * sizeof(int16_t));
    
    if (res == ESP_CODEC_DEV_OK) {
      for (size_t i = 0; i < numFrames; ++i)
      {
        input_left[i] = util::int16ToFloat<double>(i2s_rx_buffer[i * 2]);
        input_right[i] = util::int16ToFloat<double>(i2s_rx_buffer[i * 2 + 1]);
      }
    } else {
      std::fill(input_left.begin(), input_left.end(), 0.0);
      std::fill(input_right.begin(), input_right.end(), 0.0);
      vTaskDelay(1);
    }

    const auto hostTime = platform.mHostTimeFilter.sampleTimeToHostTime(platform.mSampleTime);
    platform.mSampleTime += static_cast<double>(numFrames);

    const auto bufferBeginAtOutput = hostTime + engine.mOutputLatency.load();

    // Process through engine
    engine.audioCallback(bufferBeginAtOutput, numFrames, input_left.data(), input_right.data());

    for (size_t i = 0; i < numFrames; ++i)
    {
      // Mix Metronome (Left) and Link Audio (Right) to BOTH channels for the speaker
      double mixed = engine.mBuffers[0][i] + engine.mBuffers[1][i];
      int16_t sample = util::floatToInt16(mixed);
      i2s_tx_buffer[i * 2] = sample;
      i2s_tx_buffer[i * 2 + 1] = sample;
    }

    esp_codec_dev_write(platform.mSpkHandle, i2s_tx_buffer.data(), i2s_tx_buffer.size() * sizeof(int16_t));
  }

  vTaskDelete(NULL);
}

template <typename Link>
void AudioPlatform<Link>::initialize()
{
  ESP_LOGI(TAG, "Initializing Audio BSP with 44.1kHz Philips Standard...");
  i2s_std_config_t i2s_config = {
    .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(44100),
    .slot_cfg = I2S_STD_PHILIP_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_STEREO),
    .gpio_cfg = {
      .mclk = BSP_I2S_MCLK,
      .bclk = BSP_I2S_SCLK,
      .ws = BSP_I2S_LCLK,
      .dout = BSP_I2S_DOUT,
      .din = BSP_I2S_DSIN,
      .invert_flags = {
        .mclk_inv = false,
        .bclk_inv = false,
        .ws_inv = false,
      },
    },
  };
  
  ESP_ERROR_CHECK(bsp_audio_init(&i2s_config));

  ESP_LOGI(TAG, "Initializing Codecs via esp_codec_dev...");
  mSpkHandle = bsp_audio_codec_speaker_init();
  mMicHandle = bsp_audio_codec_microphone_init();
  
  if (!mSpkHandle || !mMicHandle) {
    ESP_LOGE(TAG, "Failed to initialize codec handles!");
    return;
  }

  esp_codec_dev_sample_info_t fs = {
    .bits_per_sample = 16,
    .channel = 2,
    .channel_mask = 0,
    .sample_rate = 44100,
    .mclk_multiple = 256,
  };

  ESP_ERROR_CHECK(esp_codec_dev_open(mSpkHandle, &fs));
  ESP_ERROR_CHECK(esp_codec_dev_open(mMicHandle, &fs));

  esp_codec_dev_set_out_vol(mSpkHandle, 80);
  esp_codec_dev_set_in_gain(mMicHandle, 30.0);

  // Configure PA Control Pin (Speaker Enable)
  gpio_config_t io_conf = {
    .pin_bit_mask = (1ULL << BSP_POWER_AMP_IO),
    .mode = GPIO_MODE_OUTPUT,
    .pull_up_en = GPIO_PULLUP_DISABLE,
    .pull_down_en = GPIO_PULLDOWN_DISABLE,
    .intr_type = GPIO_INTR_DISABLE,
  };
  gpio_config(&io_conf);
  gpio_set_level(BSP_POWER_AMP_IO, 0);

  const auto latencyUs = static_cast<long long>(mEngine.mBuffers[0].size() * 1e6 / mEngine.mSampleRate);
  mEngine.mOutputLatency.store(std::chrono::microseconds(latencyUs));
}

template <typename Link>
void AudioPlatform<Link>::uninitialize()
{
  gpio_set_level(BSP_POWER_AMP_IO, 0);
  if (mSpkHandle) {
    esp_codec_dev_close(mSpkHandle);
    esp_codec_dev_delete(mSpkHandle);
  }
  if (mMicHandle) {
    esp_codec_dev_close(mMicHandle);
    esp_codec_dev_delete(mMicHandle);
  }
}

template <typename Link>
void AudioPlatform<Link>::start()
{
  mRunning = true;
  
  vTaskDelay(pdMS_TO_TICKS(100));
  gpio_set_level(BSP_POWER_AMP_IO, 1);
  ESP_LOGI(TAG, "Speaker PA Enabled");

  xTaskCreatePinnedToCore(audioTask, "audio_task", 16384, this, configMAX_PRIORITIES - 1, &mTaskHandle, 1);
}

template <typename Link>
void AudioPlatform<Link>::stop()
{
  mRunning = false;
  if (mTaskHandle)
  {
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

} // namespace linkaudio
} // namespace ableton
