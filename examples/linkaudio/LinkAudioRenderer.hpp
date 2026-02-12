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

#include <ableton/LinkAudio.hpp>

#if defined(LINK_AUDIO)

#include <ableton/link_audio/Buffer.hpp>
#include <ableton/link_audio/Queue.hpp>
#include <ableton/util/FloatIntConversion.hpp>
#include <array>
#include <chrono>
#include <algorithm>

namespace ableton
{
namespace linkaudio
{

namespace
{

template <typename T>
T cubicInterpolate(const std::array<T, 4>& p, double t)
{
  double a = -0.5 * static_cast<double>(p[0]) + 1.5 * static_cast<double>(p[1])
             - 1.5 * static_cast<double>(p[2]) + 0.5 * static_cast<double>(p[3]);
  double b = static_cast<double>(p[0]) - 2.5 * static_cast<double>(p[1])
             + 2.0 * static_cast<double>(p[2]) - 0.5 * static_cast<double>(p[3]);
  double c = -0.5 * static_cast<double>(p[0]) + 0.5 * static_cast<double>(p[2]);
  double d = static_cast<double>(p[1]);
  return static_cast<T>(a * t * t * t + b * t * t + c * t + d);
}

template <typename T>
T linearInterpolateMap(T value, T inMin, T inMax, T outMin, T outMax)
{
  if (inMax == inMin) return outMin;
  return (value - inMin) * (outMax - outMin) / (inMax - inMin) + outMin;
}

} // namespace

template <typename Link>
class LinkAudioRenderer
{
  struct Buffer
  {
#ifdef ESP_PLATFORM
    std::array<float, 512> mSamples; 
#else
    std::array<double, 512> mSamples; // Should at least hold max network buffer size
#endif
    LinkAudioSource::BufferHandle::Info mInfo;
  };
  using Queue = link_audio::Queue<Buffer>;

public:
  LinkAudioRenderer(Link& link, double& sampleRate)
    : mLink(link)
    , mSink(mLink, "A Sink", 4096)
    , mSampleRate(sampleRate)
  {
#ifdef ESP_PLATFORM
    auto queue = Queue(256, {}); 
#else
    auto queue = Queue(2048, {});
#endif
    mpQueueWriter = std::make_shared<typename Queue::Writer>(std::move(queue.writer()));
    mpQueueReader = std::make_shared<typename Queue::Reader>(std::move(queue.reader()));
  }

  ~LinkAudioRenderer() { mpSource.reset(); }

  void send(double* pSamples,
            size_t numFrames,
            typename Link::SessionState sessionState,
            double sampleRate,
            const std::chrono::microseconds hostTime,
            double quantum)
  {
    // We can only send audio if the sink can provide a buffer to write to
    auto buffer = LinkAudioSink::BufferHandle(mSink);
    if (buffer)
    {
      // The sink expects 16 bit integers so the doubles have to be converted
      for (auto i = 0u; i < numFrames; ++i)
      {
        buffer.samples[i] = ableton::util::floatToInt16(pSamples[i]);
      }

      // The buffer is commited to Link Audio with the required timing information
      const auto beatsAtBufferBegin = sessionState.beatAtTime(hostTime, quantum);
      buffer.commit(sessionState,
                    beatsAtBufferBegin,
                    quantum,
                    static_cast<uint32_t>(numFrames),
                    1, // mono
                    static_cast<uint32_t>(sampleRate));
    }
  }

  void receive(double* pRightSamples,
               size_t numFrames,
               typename Link::SessionState sessionState,
               double sampleRate,
               const std::chrono::microseconds hostTime,
               double quantum)
  {
    // Clear output buffer first
    std::fill_n(pRightSamples, numFrames, 0.0);

    // Get all slots from the queue
    while (mpQueueReader->retainSlot())
    {
    }

    // Calculate the beat range we want to render to the output buffer
    constexpr auto kLatencyInBeats = 4;
    const auto targetBeatsAtBufferBegin =
      sessionState.beatAtTime(hostTime, quantum) - kLatencyInBeats;
    const auto targetBeatsAtBufferEnd =
      sessionState.beatAtTime(
        hostTime
          + std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::duration<double>((double(numFrames)) / sampleRate)),
        quantum)
      - kLatencyInBeats;

    // Drop slots that are too old
    while (mpQueueReader->numRetainedSlots() > 0)
    {
      const auto endBeats = (*mpQueueReader)[0]->mInfo.endBeats(sessionState, quantum);
      if (endBeats && *endBeats < targetBeatsAtBufferBegin)
      {
        mpQueueReader->releaseSlot();
        moLastFrameIdx = std::nullopt;
        moStartReadPos = std::nullopt;
      }
      else
      {
        break;
      }
    }

    // Return early if there is no audio buffer to read from
    if (mpQueueReader->numRetainedSlots() == 0)
    {
      moLastFrameIdx = std::nullopt;
      moStartReadPos = std::nullopt;
      mBuffered = 0;
      return;
    }

    // Return early if the next buffer is too new and we are not rendering
    const auto beginBeats = (*mpQueueReader)[0]->mInfo.beginBeats(sessionState, quantum);
    if (!moStartReadPos && beginBeats && *beginBeats > targetBeatsAtBufferBegin)
    {
      moLastFrameIdx = std::nullopt;
      moStartReadPos = std::nullopt;
      mBuffered = 0;
      return;
    }

    // Initialize start read position if not set
    if (!moStartReadPos)
    {
      const auto& info = (*mpQueueReader)[0]->mInfo;
      const auto startBufferBegin = info.beginBeats(sessionState, quantum);
      const auto startBufferEnd = info.endBeats(sessionState, quantum);

      if (!startBufferBegin || !startBufferEnd) {
          mpQueueReader->releaseSlot();
          return;
      }

      // Calculate initial position from target beat, ensuring it's not negative
      moStartReadPos = std::max(0.0, linearInterpolateMap(targetBeatsAtBufferBegin,
                                         *startBufferBegin,
                                         *startBufferEnd,
                                         0.0,
                                         double(info.numFrames)));
    }

    const auto startFramePos = *moStartReadPos;

    // Loop through queue to find end beat and collect total frames
    auto totalFrames = 0.0;
    auto foundEnd = false;

    for (auto i = 0u; i < mpQueueReader->numRetainedSlots(); ++i)
    {
      const auto& info = (*mpQueueReader)[i]->mInfo;
      const auto bBeats = info.beginBeats(sessionState, quantum);
      const auto eBeats = info.endBeats(sessionState, quantum);

      if (!bBeats || !eBeats) continue;

      if (targetBeatsAtBufferEnd >= *bBeats && targetBeatsAtBufferEnd < *eBeats)
      {
        const auto targetBeatsFrame = linearInterpolateMap(
          targetBeatsAtBufferEnd, *bBeats, *eBeats, 0.0, double(info.numFrames));

        totalFrames += targetBeatsFrame;
        foundEnd = true;
        break;
      }
      else
      {
        totalFrames += double(info.numFrames);
      }
    }

    // We don't have enough frames buffered
    if (!foundEnd)
    {
      mBuffered = 0;
      return;
    }

    // Take the frames we have already rendered into account
    totalFrames -= startFramePos;

    // Beat time jump or buffer issue, we can't continue rendering
    if (totalFrames <= 0.0 || totalFrames > 10000.0) // Safety cap
    {
      moLastFrameIdx = std::nullopt;
      moStartReadPos = std::nullopt;
      mBuffered = 0;
      return;
    }

    const auto frameIncrement = totalFrames / double(numFrames);
    auto readPos = startFramePos;

    auto getSample = [&](size_t idx) -> double
    {
      size_t bufferIdx = 0;
      size_t currentIdx = idx;

      while (bufferIdx < mpQueueReader->numRetainedSlots())
      {
        auto& currentBuffer = *((*mpQueueReader)[bufferIdx]);
        if (currentIdx < currentBuffer.mInfo.numFrames)
        {
          return static_cast<double>(currentBuffer.mSamples[currentIdx]);
        }
        currentIdx -= currentBuffer.mInfo.numFrames;
        ++bufferIdx;
      }

      return 0.0;
    };

    for (auto frame = 0u; frame < numFrames; ++frame)
    {
      const auto framePos = readPos + frame * frameIncrement;
      const auto frameIdx = static_cast<size_t>(std::max(0.0, std::floor(framePos)));
      const auto t = framePos - std::floor(framePos);

      // Update the interpolator cache
      // frameIdx is the index of the first sample needed for interpolation
      while (!moLastFrameIdx || (moLastFrameIdx && frameIdx > *moLastFrameIdx))
      {
        if (!moLastFrameIdx) {
            // Prime the cache
            mReceiverSampleCache[1] = getSample(frameIdx);
            mReceiverSampleCache[0] = (frameIdx > 0) ? getSample(frameIdx - 1) : 0.0;
            mReceiverSampleCache[2] = getSample(frameIdx + 1);
            mReceiverSampleCache[3] = getSample(frameIdx + 2);
            moLastFrameIdx = frameIdx;
        } else {
            mReceiverSampleCache[0] = mReceiverSampleCache[1];
            mReceiverSampleCache[1] = mReceiverSampleCache[2];
            mReceiverSampleCache[2] = mReceiverSampleCache[3];
            mReceiverSampleCache[3] = getSample(*moLastFrameIdx + 3);
            moLastFrameIdx = *moLastFrameIdx + 1;
        }
        
        // Safety break to prevent infinite loop/Watchdog if frameIdx is somehow huge
        if (frameIdx > *moLastFrameIdx + 1000) {
            moLastFrameIdx = frameIdx;
            break; 
        }
      }

      // Write the output frame
      pRightSamples[frame] = cubicInterpolate(mReceiverSampleCache, t);

      // Drop buffer if we've moved past it
      const auto& currentInfo = (*mpQueueReader)[0]->mInfo;
      if (frameIdx >= currentInfo.numFrames)
      {
        readPos -= double(currentInfo.numFrames);
        moLastFrameIdx = *moLastFrameIdx - currentInfo.numFrames;
        mpQueueReader->releaseSlot();
      }
    }

    // Update the read position for the next buffer
    *moStartReadPos = readPos + double(numFrames) * frameIncrement;

    mBuffered = float(totalFrames) / float(sampleRate);
  }

  void operator()(double* pLeftSamples,
                  double* pRightSamples,
                  size_t numFrames,
                  typename Link::SessionState sessionState,
                  double sampleRate,
                  const std::chrono::microseconds hostTime,
                  double quantum)
  {
    // Send the left channel to Link
    send(pLeftSamples, numFrames, sessionState, sampleRate, hostTime, quantum);

    // Write the received audio to the right channel
    receive(pRightSamples, numFrames, sessionState, sampleRate, hostTime, quantum);
  }


  bool hasSource() const { return mpSource != nullptr; }
  
  ChannelId sourceId() const { return mpSource ? mpSource->id() : ChannelId{}; }

  void removeSource()
  {
    if (mpSource)
    {
      mpSource.reset();

      while (mpQueueReader->retainSlot())
      {
      }

      while (mpQueueReader->numRetainedSlots() > 0)
      {
        mpQueueReader->releaseSlot();
      }

      moLastFrameIdx = std::nullopt;
      moStartReadPos = std::nullopt;
    }
  }

  float buffered() const { return mBuffered; }

  void createSource(const ChannelId& channelId)
  {
    mpSource = std::make_unique<LinkAudioSource>(
      mLink,
      channelId,
      [this](ableton::LinkAudioSource::BufferHandle bufferHandle)
      { onSourceBuffer(bufferHandle); });
  }

  void onSourceBuffer(const LinkAudioSource::BufferHandle bufferHandle)
  {
    if (bufferHandle.info.sampleRate == 0) return;

    if (mpQueueWriter->retainSlot())
    {
      auto& buffer = *((*mpQueueWriter)[0]);
      buffer.mInfo = bufferHandle.info;
      buffer.mInfo.numChannels = 1;

      const size_t numFrames = std::min(bufferHandle.info.numFrames, buffer.mSamples.size());
      buffer.mInfo.numFrames = numFrames;

      for (auto i = 0u; i < numFrames; ++i)
      {
#ifdef ESP_PLATFORM
        buffer.mSamples[i] = util::int16ToFloat<float>(
          bufferHandle.samples[i * bufferHandle.info.numChannels]);
#else
        buffer.mSamples[i] = util::int16ToFloat<double>(
          bufferHandle.samples[i * bufferHandle.info.numChannels]);
#endif
      }

      mpQueueWriter->releaseSlot();
    }
  }

  Link& mLink;
  LinkAudioSink mSink;
  std::unique_ptr<LinkAudioSource> mpSource;
  double& mSampleRate;

  std::optional<double> moStartReadPos;
  std::atomic<float> mBuffered = 0;

private:
  std::shared_ptr<typename Queue::Writer> mpQueueWriter;
  std::shared_ptr<typename Queue::Reader> mpQueueReader;

#ifdef ESP_PLATFORM
  std::array<float, 4> mReceiverSampleCache = {{0.0f, 0.0f, 0.0f, 0.0f}};
#else
  std::array<double, 4> mReceiverSampleCache = {{0.0, 0.0, 0.0, 0.0}};
#endif
  std::optional<size_t> moLastFrameIdx = std::nullopt;
};

} // namespace linkaudio
} // namespace ableton

#else

namespace ableton
{
namespace linkaudio
{

template <typename Link>
class LinkAudioRenderer
{
public:
  LinkAudioRenderer(Link&, double&) {}

  // In case we don't support LinkAudio we just copy the left output buffer to the right
  void operator()(double* pLeftSamples,
                  double* pRightSamples,
                  size_t numFrames,
                  typename Link::SessionState,
                  double,
                  const std::chrono::microseconds,
                  double)
  {
    std::copy_n(pLeftSamples, numFrames, pRightSamples);
  }
};

} // namespace linkaudio
} // namespace ableton

#endif