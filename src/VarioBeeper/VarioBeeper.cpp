
#include "VarioBeeper.h"

VarioBeeper::VarioBeeper()
{
    enableAmp();
    //start beep task
    // startTask();
}

void VarioBeeper::init(double sinkingThreshold, double climbingThreshold, double nearClimbingSensitivity, uint8_t baseVolume)
{

    _volume = baseVolume;

    // /* set threshold */
    // setThresholds(sinkingThreshold, climbingThreshold, nearClimbingSensitivity);

    // /* init private vars */
    // beepStartTime = 0;
    // beepState = 0;
    // beepType = BEEP_TYPE_SILENT;

    // //init le duty / cycle
    // CLIMBING_BEEP_HIGH_LENGTH = 500;
    // CLIMBING_BEEP_LOW_LENGTH = 500;
    // CLIMBING_BEEP_LENGTH = (CLIMBING_BEEP_HIGH_LENGTH + CLIMBING_BEEP_LOW_LENGTH);
}

void VarioBeeper::enableAmp()
{
    if (!_isAmpOn)
    {
        pinMode(PIN_AUDIO_AMP_ENA, OUTPUT);
        digitalWrite(PIN_AUDIO_AMP_ENA, HIGH);
        _isAmpOn = true;
    }
};

void VarioBeeper::disableAmp()
{
    if (_isAmpOn)
    {
        pinMode(PIN_AUDIO_AMP_ENA, OUTPUT);
        digitalWrite(PIN_AUDIO_AMP_ENA, LOW);
        _isAmpOn = false;
    }
}

void VarioBeeper::setVolume(uint8_t newVolume)
{
    _volume = newVolume;
}

uint8_t VarioBeeper::getVolume()
{
    return _volume;
}

void VarioBeeper::mute()
{
    noToneAC();
    _muted = true;
}

void VarioBeeper::unMute()
{
    _muted = false;
}

bool VarioBeeper::isMute(void)
{
    return _muted;
}

void VarioBeeper::generateTone(uint32_t fHz, int ms)
{
    enableAmp();
    vTaskDelay(delayT1 * 10);
    toneAC(fHz, _volume, ms);
    disableAmp();
}

void VarioBeeper::generateTone(uint32_t fHz, int ms, uint8_t volume)
{
    enableAmp();
    vTaskDelay(delayT1 * 10);
    toneAC(fHz, volume, ms);
    disableAmp();
}

void VarioBeeper::setGlidingAlarmState(bool state)
{
    _nearClimbingAlarm = state;
}

void VarioBeeper::setGlidingBeepState(bool state)
{
    _nearClimbingBeep = state;
}

void VarioBeeper::task()
{
    enableAmp();
    while (1)
    {
        // //play beep based on climb rate
        // if (abs(_previousClimb - _currentClimb) > 0.1)
        // {
        if (!_playToneRunning)
        {
            playTone(_currentClimb);
        }
        // }

        // // give time to other tasks
        vTaskDelay(delayT1 * 10);
    }
}

void VarioBeeper::startTaskImpl(void *parm)
{
    Serial.println("START Task VarioBeeper");

    // wrapper for task
    static_cast<VarioBeeper *>(parm)->task();
}

void VarioBeeper::startTask()
{
    // task creation
    xTaskCreate(this->startTaskImpl, "TaskVarioSound", 2048, this, 20, NULL);
}

uint16_t VarioBeeper::getDutty(float_t climb)
{
    if (isZerotage(climb))
    {
        return _zerotageDutty;
    }
    return getFromArray(climb, _dutty);
}

uint16_t VarioBeeper::getCycle(float_t climb)
{
    if (isZerotage(climb))
    {
        return _zerotageCycleLow - ((-_zerotageLow - climb) * (_zerotageCycleHigh - _zerotageCycleLow) / (_zerotageHigh - _zerotageLow));
    }
    return getFromArray(climb, _cycle);
}

uint16_t VarioBeeper::getFrequency(float_t climb)
{
    if (isZerotage(climb))
    {
        if (_silentOnZerotage)
        {
            return 0;
        }
    }
    return getFromArray(climb, _hertz);
}

bool VarioBeeper::isZerotage(float_t climb)
{

    return (_withZerotage && climb >= _zerotageLow && climb <= _zerotageHigh) ? true : false;
}

uint16_t VarioBeeper::getFromArray(float_t climb, uint16_t *myArray)
{
    uint16_t before = 0, after;
    float nbMs;
    int16_t nbByMs = 1;

    for (int i = 0; i < BEEPER_SIZE; i++)
    {
        if (climb < _msclimb[i])
        {
            before = myArray[i - 1];
            after = myArray[i];
            nbMs = _msclimb[i] - _msclimb[i - 1];
            nbByMs = (after - before) / nbMs;

            return before + (climb - _msclimb[i - 1]) * nbByMs;
        }
        else if (i == (BEEPER_SIZE - 1))
        {
            return before + (climb - _msclimb[i - 1]) * nbByMs;
        }
    }
}

void VarioBeeper::playTone(float_t climb)
{

    Serial.println("START playTone");

    Serial.print("_isPlaying:");
    Serial.println(_isPlaying);

    Serial.print("_remainingSilence:");
    Serial.println(_remainingSilence);

    Serial.print("_remainingDuration :");
    Serial.println(_remainingDuration);

    Serial.print("climb :");
    Serial.println(climb);
    _playToneRunning = true;
    // Serial.print("climb:");
    // Serial.println(climb);
    // Serial.print("isMute:");
    // Serial.println(isMute());
    // Serial.print("volume:");
    // Serial.println(_volume);

    uint8_t stepFrequency;
    float facteurLissage = 0.60;

    if (!isMute())
    {

        if (_isNoBeepEnable && climb > _noBeepLow && climb < _noBeepHigh)
        {
            if (!_isSilencing)
            {
                // pour supprimer le "tac"
                toneAC(30000, _volume);
                vTaskDelay(delayT1);

                //silencing
                noToneAC();
                // disableAmp();
                _isSilencing = true;

                // force stop running cycle if any
                _isPlaying = false;
                _remainingDuration = 0;
                _remainingSilence = 0;
            }
            _playToneRunning = false;
            return;
        }

        if (_isPlaying && _remainingSilence <= 0)
        {
            if (_remainingDuration > 0)
            {
                _newFreq = getFrequency(climb);
                if (_newFreq != 0)
                {
                    //lissage de la frequence
                    _newFreq = (_newFreq * facteurLissage) + (1.00 - facteurLissage) * _freq;

                    if (_newFreq != _freq)
                    {

                        _previousClimb = _currentClimb;
                        stepFrequency = max(round(abs(_freq - _newFreq) / 5), (double_t)1);
                        Serial.print("stepFrequency :");
                        Serial.println(stepFrequency);
                        // Serial.print("_newFreq:");
                        // Serial.println(_newFreq);
                        if (_freq < _newFreq)
                        {
                            for (uint16_t ch = _freq; ch <= _newFreq; ch += stepFrequency)
                            {
                                toneAC(ch, _volume);
                                vTaskDelay(delayT1);
                            }
                        }
                        else if (_freq > _newFreq)
                        {
                            for (uint16_t ch = _freq; ch >= _newFreq; ch -= stepFrequency)
                            {
                                toneAC(ch, _volume);
                                vTaskDelay(delayT1);
                            }
                        }
                    }
                    toneAC(_newFreq, _volume);
                }
                _freq = _newFreq;
                _endMillis = millis();
                _delta = (_endMillis - _startMillis);
                _remainingDuration -= _delta;
                _startMillis = millis();
            }
            else
            {
                _remainingSilence = getCycle(climb) * ((100 - getDutty(climb)) / 100.00);

                playToneSilence();

                _isPlaying = false;

                //silence start
                _startMillis = millis();
            }
        }
        else
        {
            if (_remainingSilence <= 0)
            {
                _startMillis = millis();
                _freq = getFrequency(climb);
                if (_freq == 0)
                {
                    // no beep
                    _isPlaying = false;
                    _remainingDuration = -1;
                    _remainingSilence = getCycle(climb) * ((100 - getDutty(climb)) / 100.00);

                    playToneSilence();
                }
                else
                {
                    _newFreq = _freq;
                    _remainingDuration = getCycle(climb) * (getDutty(climb) / 100.00);
                    _isSilencing = false;
                    _isPlaying = true;
                }
            }
            else
            {
                _endMillis = millis();
                _delta = (_endMillis - _startMillis);
                _remainingSilence -= _delta;
                _startMillis = millis();
                if (_remainingSilence < 80)
                {
                    enableAmp();
                }
            }
        }
    }
    else
    {
        noToneAC();
    }

    _playToneRunning = false;
}

void VarioBeeper::playToneSilence()
{
    if (_remainingSilence > 0)
    {
        if (!_isSilencing)
        {
            // pour supprimer le "tac"
            toneAC(30000, _volume);
            vTaskDelay(delayT1);
            noToneAC();
            disableAmp();
            _isSilencing = true;
        }
    }
}

void VarioBeeper::setVelocity(float_t climb)
{
    if (climb < -10)
    {
        climb = -10;
    }
    else if (climb > 10)
    {
        climb = 10;
    }

    _currentClimb = climb;
}
