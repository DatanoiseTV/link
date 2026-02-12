#include <ableton/LinkAudio.hpp>
#include "AudioPlatform.hpp"
#include <esp_event.h>
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <nvs_flash.h>
#include <protocol_examples_common.h>
#include <iostream>
#include <iomanip>
#include <esp_wifi.h>
#include "bsp/esp-box-3.h"

static const char* TAG = "Main";

extern "C" {
unsigned int if_nametoindex(const char* ifName) { return 0; }
char* if_indextoname(unsigned int ifIndex, char* ifName) { return nullptr; }

// Fix for missing libatomic in certain toolchains
bool __atomic_is_lock_free(unsigned int size, const volatile void *ptr) {
    return size <= 4;
}
}

struct AppState
{
  ableton::LinkAudio link;
  ableton::linkaudio::AudioPlatform<ableton::LinkAudio> audioPlatform;

  AppState()
    : link(120.0f, "ESP32-S3 Link Audio")
    , audioPlatform(link)
  {
    link.setChannelsChangedCallback([this]() { onChannelsChanged(); });
    link.enable(true);
    link.enableLinkAudio(true);
    link.enableStartStopSync(true);
  }

  void onChannelsChanged()
  {
    const auto channels = link.channels();
    auto& renderer = audioPlatform.mEngine.mLinkAudioRenderer;

    if (channels.empty()) {
      if (renderer.hasSource()) {
        printf("Link Audio: No channels available, removing source\n");
        renderer.removeSource();
      }
      return;
    }

    // Check if currently subscribed channel still exists
    bool currentExists = false;
    if (renderer.hasSource()) {
        for (const auto& ch : channels) {
            if (ch.id == renderer.sourceId()) {
                currentExists = true;
                break;
            }
        }
    }

    if (!currentExists) {
        if (renderer.hasSource()) renderer.removeSource();
        printf("Link Audio: Subscribing to '%s' from '%s'\n", 
               channels[0].name.c_str(), channels[0].peerName.c_str());
        renderer.createSource(channels[0].id);
    }
  }
};

void statusTask(void* userParam)
{
  auto state = static_cast<AppState*>(userParam);
  vTaskDelay(pdMS_TO_TICKS(2000));

  while (true)
  {
    if (!state) break;

    const auto sessionState = state->link.captureAppSessionState();
    const auto numPeers = state->link.numPeers();
    const auto buffered = state->audioPlatform.mEngine.mLinkAudioRenderer.buffered();
    
    printf("Peers: %u | Tempo: %.2f | Latency: %.3fs | Playing: %s\n", 
           (unsigned int)numPeers, 
           (double)sessionState.tempo(),
           (double)buffered,
           sessionState.isPlaying() ? "YES" : "NO");

    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}

extern "C" void app_main()
{
  esp_err_t ret = nvs_flash_init();
  if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    ESP_ERROR_CHECK(nvs_flash_erase());
    ret = nvs_flash_init();
  }
  ESP_ERROR_CHECK(ret);

  ESP_ERROR_CHECK(esp_netif_init());
  ESP_ERROR_CHECK(esp_event_loop_create_default());

  // Initialize BSP I2C for codecs
  ESP_ERROR_CHECK(bsp_i2c_init());

  ESP_ERROR_CHECK(example_connect());
  ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));

  ESP_LOGI(TAG, "Connected to network, launching AppState...");

  auto appState = std::make_unique<AppState>();
  ESP_LOGI(TAG, "AppState constructed successfully.");

  xTaskCreatePinnedToCore(statusTask, "status_task", 8192, appState.release(), 5, nullptr, 1);
}
