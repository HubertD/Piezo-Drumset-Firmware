#pragma once

#include <stdint.h>
#include <array>
#include <numeric>

class DrumStateMachine
{
    public:
        enum class State
        {
            IDLE,
            TRIGGERED,
        };

    public:
        DrumStateMachine() 
        {
            _buffer.fill(0);
        }

        bool Update(unsigned time, unsigned adcValue)
        {
            _buffer[_bufferPos++] = adcValue;
            _bufferPos %= BUFFER_SIZE;

            switch (_state)
            {
                case State::IDLE:
                    return UpdateStateIdle(time, adcValue);

                case State::TRIGGERED:
                    return UpdateStateTriggered(time, adcValue);

                default:
                    return false;
            }
        }

        State GetState()
        {
            return _state;
        }

        unsigned GetVelocity()
        {
            return _velocity;
        }

    private:
        static constexpr unsigned BUFFER_SIZE { 32 };
        static constexpr unsigned THRESHOLD_TRIGGER { 4000 };
        static constexpr unsigned THRESHOLD_OFF { 50 };
        static constexpr unsigned TIME_REFRACT_MS { 25 };
        static constexpr unsigned VELOCITY_DIVIDER { 128 };
        static constexpr unsigned MAX_VELOCITY { 127 };
        
        State _state;
        std::array<uint16_t, BUFFER_SIZE> _buffer;
        unsigned _bufferPos { 0 };
        unsigned _timeout { 0 };
        unsigned _velocity { 0 };

        bool UpdateStateIdle(unsigned time, unsigned)
        {
            auto sum = static_cast<unsigned>(std::accumulate(_buffer.begin(), _buffer.end(), 0));
            if (sum > THRESHOLD_TRIGGER)
            {
                _state = State::TRIGGERED;
                _timeout = time + TIME_REFRACT_MS;
                _velocity = sum / VELOCITY_DIVIDER;
                if (_velocity > MAX_VELOCITY)
                {
                    _velocity = MAX_VELOCITY;
                }
                return true;
            }
            return false;
        }

        bool UpdateStateTriggered(unsigned time, unsigned adcValue)
        {
            if (adcValue > THRESHOLD_OFF)
            {
                _timeout = time + TIME_REFRACT_MS;
            }
            else if (_timeout <= time)
            {
                _state = State::IDLE;
                return true;
            }
            return false;
        }
        
};
