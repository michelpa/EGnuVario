#ifndef VARIO_BEEPER_H
#define VARIO_BEEPER_H

#include "Arduino.h"
#include "toneAC.h"

#define BEEP_DEFAULT_VOLUME 5
#define PIN_AUDIO_AMP_ENA 19

#define BEEPER_SIZE 12

class VarioBeeper
{
private:
    const TickType_t delayT1 = (1) / portTICK_PERIOD_MS;

    bool _isAmpOn = false;
    bool _muted = false;

    float _msclimb[BEEPER_SIZE] = {-10.00, -3.00, -2.00, -1.00, -0.30, 0.10, 0.50, 1.00, 2.00, 3.00, 5.00, 10.00};
    uint16_t _hertz[BEEPER_SIZE] = {200, 293, 369, 440, 475, 493, 550, 595, 675, 745, 880, 1108};
    uint16_t _cycle[BEEPER_SIZE] = {200, 200, 200, 200, 600, 600, 550, 500, 400, 310, 250, 200};
    uint16_t _dutty[BEEPER_SIZE] = {100, 100, 100, 100, 100, 50, 50, 50, 50, 50, 50, 50};

    // silence sur zerotage
    // surcharge les autres configs
    bool _silentOnZerotage = false;

    // parametrage du zerotage
    // si activé, surcharge le fonctionnement normal
    bool _withZerotage = true;
    bool _isPreviousToneIsZerotage = false;
    float _zerotageLow = -0.7;
    float _zerotageHigh = 0.2;
    uint16_t _zerotageCycleLow = 1000;
    uint16_t _zerotageCycleHigh = 450;
    uint16_t _zerotageDutty = 5;

    //parametrage la zone sans bip
    bool _isNoBeepEnable = true;
    float _noBeepLow = -1.5;
    float _noBeepHigh = -0.7;

    // a implementer pour compatibilité
    bool _nearClimbingAlarm = false;
    bool _nearClimbingBeep = false;

    bool _isPlaying = false;
    bool _isSilencing = true;
    bool _playToneRunning = false;

    int32_t _remainingDuration = 0;
    int32_t _remainingSilence = 0;

    float_t _climbStep = 0.2;
    uint16_t _freq = 400;
    uint16_t _newFreq = 400;
    uint8_t _volume = BEEP_DEFAULT_VOLUME;
    uint32_t _startMillis;
    uint32_t _endMillis;
    uint32_t _delta;
    float_t _currentClimb;
    float_t _playingClimb = 0;

    TaskHandle_t _taskVarioSoundHandle = NULL;

    void enableAmp();
    void disableAmp();

    uint16_t getDutty(float_t climb);
    uint16_t getCycle(float_t climb);
    uint16_t getFrequency(float_t climb);
    uint16_t getFromArray(float_t climb, uint16_t *myArray);

    bool isZerotage(float_t climb);

    static void startTaskImpl(void *);

    void task();

    void playTone(float_t climb);
    void playToneSilence();

public:
    VarioBeeper();
    void startTask();
    void init(double sinkingThreshold, double climbingThreshold, double nearClimbingSensitivity, uint8_t baseVolume = BEEP_DEFAULT_VOLUME);
    void setVolume(uint8_t newVolume = BEEP_DEFAULT_VOLUME);
    uint8_t getVolume();
    void setVelocity(float_t climb);
    void generateTone(uint32_t freqHz, int ms);
    void generateTone(uint32_t freqHz, int ms, uint8_t volume);
    void mute();
    void unMute();
    bool isMute(void);
    void setGlidingAlarmState(bool state);
    void setGlidingBeepState(bool state);
};

extern VarioBeeper beeper;

#endif //VARIO_BEEPER_H