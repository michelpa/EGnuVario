
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
    pinMode(PIN_AUDIO_AMP_ENA, OUTPUT);
    digitalWrite(PIN_AUDIO_AMP_ENA, HIGH);
};

void VarioBeeper::disableAmp()
{
    pinMode(PIN_AUDIO_AMP_ENA, OUTPUT);
    digitalWrite(PIN_AUDIO_AMP_ENA, LOW);
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
    toneAC(fHz, _volume, ms);
    disableAmp();
}

void VarioBeeper::generateTone(uint32_t fHz, int ms, uint8_t volume)
{
    enableAmp();
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
        const TickType_t delay = (10) / portTICK_PERIOD_MS;
        vTaskDelay(delay);
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
        return _zerotageCycle;
    }
    return getFromArray(climb, _cycle);
}

uint16_t VarioBeeper::getFrequency(float_t climb)
{
    // if (isZerotage(climb))
    // {
    //     return _zerotageFreq;
    // }
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

    const TickType_t delay = (1) / portTICK_PERIOD_MS;

    // Serial.println("START playTone");

    // Serial.print("_isPlaying:");
    // Serial.println(_isPlaying);

    // Serial.print("_remainingSilence:");
    // Serial.println(_remainingSilence);

    // Serial.print("_remainingDuration :");
    // Serial.println(_remainingDuration);

    // Serial.print("climb :");
    // Serial.println(climb);
    _playToneRunning = true;
    // Serial.print("climb:");
    // Serial.println(climb);
    // Serial.print("isMute:");
    // Serial.println(isMute());
    // Serial.print("volume:");
    // Serial.println(_volume);

    float facteurLissage = 0.60;
    if (!isMute())
    {
        uint8_t stepFrequency = 5;
        if (_isPlaying && _remainingSilence <= 0)
        {
            if (_remainingDuration > 0)
            {
                _newFreq = getFrequency(climb);
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
                            // toneAC(ch, _volume, 1, true);
                            toneAC(ch, _volume);
                            vTaskDelay(delay);
                            // delay(1);
                        }
                    }
                    else if (_freq > _newFreq)
                    {
                        for (uint16_t ch = _freq; ch >= _newFreq; ch -= stepFrequency)
                        {
                            // toneAC(ch, _volume, 1, true);
                            toneAC(ch, _volume);
                            vTaskDelay(delay);
                            // delay(1);
                        }
                    }
                }
                toneAC(_newFreq, _volume);
                _freq = _newFreq;
                _endMillis = millis();
                _delta = (_endMillis - _startMillis);
                _remainingDuration -= _delta;
                _startMillis = millis();
            }
            else
            {
                _remainingSilence = getCycle(climb) * ((100 - getDutty(climb)) / 100.00);
                if (_remainingSilence > 0)
                {
                    // pour supprimer le "tac"
                    toneAC(30000, _volume);
                    vTaskDelay(delay);
                    noToneAC();
                    //disableAmp();
                }

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
                _newFreq = _freq;
                _remainingDuration = getCycle(climb) * (getDutty(climb) / 100.00);

                _isPlaying = true;
                enableAmp();
            }
            else
            {
                _endMillis = millis();
                _delta = (_endMillis - _startMillis);
                _remainingSilence -= _delta;
                _startMillis = millis();
            }
        }
    }
    else
    {
        noToneAC();
    }

    _playToneRunning = false;
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
