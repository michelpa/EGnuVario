#ifndef VARIO_TONE_H
#define VARIO_TONE_H

#include "Arduino.h"
#include "toneAC.h"
// #include "VarioBeeper.h"
#include <DebugConfig.h>

#define BEEP_DEFAULT_VOLUME 5
#define PIN_AUDIO_AMP_ENA 19

class VarioTone
{
private:
    uint8_t _volume = BEEP_DEFAULT_VOLUME;
    bool _muted = false;
    bool _isAmpOn = false;

protected:
    const TickType_t delayT1 = pdMS_TO_TICKS(1); //(1) / portTICK_PERIOD_MS;
    TaskHandle_t _taskVarioBeeperHandle = NULL;

    void enableAmp();
    void disableAmp();

public:
    VarioTone();
    void init(double sinkingThreshold, double climbingThreshold, double nearClimbingSensitivity, uint8_t baseVolume = BEEP_DEFAULT_VOLUME);
    void generateTone(uint32_t freqHz, int ms);
    void generateTone(uint32_t freqHz, int ms, uint8_t volume);
    void setVolume(uint8_t newVolume = BEEP_DEFAULT_VOLUME);
    uint8_t getVolume();
    void mute();
    void unMute();
    bool isMute(void);
};

#endif //VARIO_TONE_H