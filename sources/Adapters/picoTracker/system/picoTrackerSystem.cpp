/*
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * Copyright (c) 2024 xiphonics, inc.
 *
 * This file is part of the picoTracker firmware
 */

#include "picoTrackerSystem.h"
#include "Adapters/picoTracker/audio/picoTrackerAudio.h"
#include "Adapters/picoTracker/filesystem/picoTrackerFileSystem.h"
#include "Adapters/picoTracker/gui/GUIFactory.h"
#include "Adapters/picoTracker/midi/picoTrackerMidiService.h"
#include "Adapters/picoTracker/system/picoTrackerSamplePool.h"
#include "Adapters/picoTracker/timer/picoTrackerTimer.h"
#include "Application/Commands/NodeList.h"
#include "Application/Model/Config.h"
#include "Application/Player/SyncMaster.h"
#include "hardware/gpio.h"
#include "input.h"
#include "pico/rand.h"
#include <assert.h>
#include <fcntl.h>
#include <memory.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/time.h>
#include <time.h>
#include <unistd.h>

#include "Adapters/picoTracker/gui/InputTester.h"
#include "Adapters/picoTracker/platform/platform.h"
#include "critical_error_message.h"
#include "hardware/adc.h"
#include "pico/stdlib.h"

EventManager *picoTrackerSystem::eventManager_ = NULL;
bool picoTrackerSystem::invert_ = false;
unsigned int picoTrackerSystem::lastBeatCount_ = 0;

int picoTrackerSystem::MainLoop() {

  eventManager_->InstallMappings();
  return eventManager_->MainLoop();
};

enum SdCardStatus { SD_OK, SD_EXFAT, SD_MISSING };

static SdCardStatus checkSDCard(FileSystem *fs) {
  if (fs->isExFat()) {
    return SD_EXFAT;
  }
  if (!fs->chdir("/")) {
    return SD_MISSING;
  }
  return SD_OK;
}

static bool pollForValidSDCard() {
  drawInputTester();

  alignas(picoTrackerFileSystem) static char
      fsMemBuf[sizeof(picoTrackerFileSystem)];
  FileSystem::Install(new (fsMemBuf) picoTrackerFileSystem());

  auto fs = FileSystem::GetInstance();
  return checkSDCard(fs) == SD_OK;
}

void picoTrackerSystem::Boot(int argc, char **argv) {

  // Install System
  alignas(
      picoTrackerSystem) static char systemMemBuf[sizeof(picoTrackerSystem)];
  System::Install(new (systemMemBuf) picoTrackerSystem());

  // Install GUI Factory
  alignas(GUIFactory) static char guiMemBuf[sizeof(GUIFactory)];
  I_GUIWindowFactory::Install(new (guiMemBuf) GUIFactory());

  // Install Timers
  alignas(picoTrackerTimerService) static char
      timerMemBuf[sizeof(picoTrackerTimerService)];
  TimerService::GetInstance()->Install(new (timerMemBuf)
                                           picoTrackerTimerService());

  // Install FileSystem
  alignas(picoTrackerFileSystem) static char
      fsMemBuf[sizeof(picoTrackerFileSystem)];
  FileSystem::Install(new (fsMemBuf) picoTrackerFileSystem());

  // First check for SDCard
  auto fs = FileSystem::GetInstance();
  SdCardStatus sdStatus = checkSDCard(fs);
  if (sdStatus == SD_EXFAT) {
    Trace::Log("PICOTRACKERSYSTEM", "SDCARD exFAT not supported");
    critical_error_message("unsupported sdcard", 0x01, pollForValidSDCard);
  } else if (sdStatus == SD_MISSING || scanKeys()) {
    Trace::Log("PICOTRACKERSYSTEM", "SDCARD MISSING!!");
    critical_error_message("SDCARD MISSING", 0x01, pollForValidSDCard);
  }

  // Install MIDI
  // **NOTE**: MIDI install MUST happen before Audio install because it triggers
  // reading config file and config file needs to have MidiService already
  // installed in order to apply midi settings read from the config file
  alignas(picoTrackerMidiService) static char
      midiMemBuf[sizeof(picoTrackerMidiService)];
  MidiService::Install(new (midiMemBuf) picoTrackerMidiService());

  // Install Sound
  AudioSettings hint;
  hint.bufferSize_ = 1024;
  hint.preBufferCount_ = 8;
  alignas(picoTrackerAudio) static char audioMemBuf[sizeof(picoTrackerAudio)];
  Audio::Install(new (audioMemBuf) picoTrackerAudio(hint));

  // Install SamplePool
  alignas(picoTrackerSamplePool) static char
      samplePoolMemBuf[sizeof(picoTrackerSamplePool)];
  SamplePool::Install(new (samplePoolMemBuf) picoTrackerSamplePool());

  eventManager_ = I_GUIWindowFactory::GetInstance()->GetEventManager();
  eventManager_->Init();

#if PICO_RP2040 || PICO_RP2350
  // Order matters: enable the ADC block first, then configure the pin as a
  // high-Z analog input, then select the input channel.
  adc_init();

  adc_gpio_init(BATT_VOLTAGE_IN);

  // select analog MUX, GPIO 26=0, 27=1, 28=1, 29=3
  adc_select_input(3);

  Trace::Log("PICOTRACKERSYSTEM", "ADC INIT DONE");
#endif
};

void picoTrackerSystem::Shutdown() { delete Audio::GetInstance(); };

static int secbase;

unsigned long picoTrackerSystem::GetClock() {
  struct timeval tp;

  gettimeofday(&tp, NULL);
  if (!secbase) {
    secbase = tp.tv_sec;
    return long(tp.tv_usec / 1000.0);
  }
  return long((tp.tv_sec - secbase) * 1000 + tp.tv_usec / 1000.0);
}

void picoTrackerSystem::GetBatteryState(BatteryState &state) {
  // Battery ADC is read ~once/s, so discard stale first samples then average
  // the rest (idle ADC reads low; averaging also smooths supply ripple).
  constexpr int kAdcIgnoreSamples = 3;
  constexpr int kAdcAverageSamples = 4;

  adc_select_input(3); // ensure VSYS/BATT_VOLTAGE_DIVIDER is the selected channel (GPIO29)
  for (int i = 0; i < kAdcIgnoreSamples; ++i) {
    (void)adc_read();
  }
  uint32_t adc_sum = 0;
  for (int i = 0; i < kAdcAverageSamples; ++i) {
    adc_sum += adc_read();
  }
  uint32_t adc_reading = adc_sum / kAdcAverageSamples;

  // Reconstruct VSYS in millivolts.
  state.voltage_mv =
      (adc_reading * 3300u * BATT_VOLTAGE_DIVIDER + 2048u) / 4096u;

  // clamp the ends of the valid voltage range
  if (state.voltage_mv < 3325) {
    state.percentage = 0;
  } else if (state.voltage_mv > 3900) {
    state.percentage = 100;
  } else {
    // the function f(x) = 100 - (x - 3,900)^2 / 3,250 closely maps the original
    // measurements. It can be optimized for the rp2040 in integer math as
    //      100 - (100 * x - 390,000) ^ 2 / 33,000,000
    // with x / 33,000,000 being approximated by x >> 25 (2^25 = 33,554,432)
    // --> (100 - (100 * x - 390,000) ^ 2) >> 25
    uint32_t q = 100 * state.voltage_mv - 390000; // 100 * x - 390,000
    q *= q;                                       // q ^ 2
    q >>= 25;                                     // q / 33,000,000
    state.percentage = 100 - q;                   // 100 - q
  }

  state.charging = state.voltage_mv > 4000;
}

void picoTrackerSystem::SetDisplayBrightness(unsigned char value) {
  platform_brightness(value);
}

void picoTrackerSystem::Sleep(int millisec) {
  //	if (millisec>0)
  //		assert(0) ;
}

void picoTrackerSystem::PostQuitMessage() { eventManager_->PostQuitMessage(); }

unsigned int picoTrackerSystem::GetMemoryUsage() { return 0; }

void picoTrackerSystem::SystemPutChar(int c) { putchar(c); }

uint32_t picoTrackerSystem::GetRandomNumber() { return get_rand_32(); }

void picoTrackerSystem::SystemBootloader() { platform_bootloader(); }

void picoTrackerSystem::SystemReboot() { platform_reboot(); }

uint32_t picoTrackerSystem::Micros() { return micros(); }

uint32_t picoTrackerSystem::Millis() { return millis(); }
